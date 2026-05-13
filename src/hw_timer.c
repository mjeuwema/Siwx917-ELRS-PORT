/**
 * @file hw_timer.c
 * @brief Hardware timer implementation for ELRS using SiWx917 Configurable
 * Timer (CT)
 *
 * This module provides precise timing for the ELRS protocol using the
 * Configurable Timer (CT) Counter 0. The CT runs from SOC_PLL (up to 180MHz),
 * providing crystal-accurate timing derived from the main XTAL oscillator.
 *
 * ELRS Timing Model:
 * - The packet interval is divided into two half-intervals (TICK and TOCK)
 * - TICK fires at the mid-point of the interval
 * - TOCK fires at the end of the interval (packet expected)
 * - Phase adjustments shift when the next TICK occurs
 * - Frequency adjustments modify the interval length for clock drift
 * compensation
 *
 * CT Timer Benefits:
 * - SOC_PLL clock (180MHz, crystal accuracy) - best resolution (~5.5ns)
 * - No GPIO pin dependencies - we only use the internal interrupt
 * - Buffer register for glitch-free interval updates
 * - Main M4 bus - no power domain crossing issues
 *
 * IMPORTANT: This implementation uses a custom CT clock init that:
 * - Uses SOC_PLL instead of INTF_PLL (which may not be ready at boot)
 * - Skips GPIO pin configuration (we don't need CT output pins)
 * - Properly waits for PLL lock before enabling CT clock
 *
 * Citation: SiWx917 RM Section 17 - Configurable Timer
 *
 * @copyright Copyright (c) 2024-2025
 */

#include "hw_timer.h"
#include "clock_update.h"
#include "cmsis_os2.h"
#include "rsi_ct.h" /* For RSI_CT_Config(), RSI_CT_Reset() */
#include "rsi_pll.h"
#include "rsi_rom_clks.h" /* For RSI_CLK_GetBaseClock(), RSI_CLK_SetCtClock() */
#include "sl_si91x_config_timer.h"
#include "system_si91x.h"
#include <stdio.h> /* For printf debug output */
#include <string.h>

/* ========================================================================== */
/*                              CONFIGURATION                                 */
/* ========================================================================== */

/**
 * CT Clock Configuration - DYNAMIC with explicit 16-bit Counter 0 mode
 *
 * We dynamically configure the CT clock divider at init time to achieve
 * a target frequency (2 MHz), regardless of the actual system clock speed.
 * This makes the code portable across different clock configurations.
 *
 * 16-BIT MODE:
 *   The unified SDK only supports 16-bit mode, but the hardware supports
 * The SDK path validates and writes Counter 0 as a 16-bit timer. ELRS RX
 * half-intervals fit in that range at the target 2 MHz timer base.
 *
 * Target: 2 MHz CT clock
 *   - 1 µs = 2 ticks (simple math!)
 *   - Resolution: 0.5 µs
 *   - 16-bit max: 32767 us half-interval at 2 MHz
 *
 * Formula: div_factor = system_clk / (2 * TARGET_TIMER_FREQ)
 * Example: 180 MHz / (2 * 2 MHz) = 45
 *          160 MHz / (2 * 2 MHz) = 40
 *
 * Note: rsi_ct.h's comment says cfg=0 is 32-bit, but its implementation clears
 * COUNTER32_BITMODE. That matches the SL_COUNTER_16BIT APIs used below.
 */
#define CT_TARGET_FREQ_HZ 2000000U /* 2 MHz target */
#define CT_TIMER_SOURCE CT_SOCPLLCLK
#define CT_MATCH_MAX 0xFFFFU
#define CT_CALIBRATION_MS 10U
#define CT_MIN_VALID_CALIBRATION_TICKS 10U
/* Upstream ESP32 RX timer uses 5 timer ticks/us for freq offset units. */
#define ELRS_FREQ_OFFSET_UNITS_PER_US 5
#define CT_TARGET_TICKS_PER_US 2U  /* At 2 MHz: 1 µs = 2 ticks */

/** Runtime-calculated ticks per µs (set during init) */
static uint32_t ct_freq_hz = CT_TARGET_FREQ_HZ;
static uint32_t ct_ticks_per_us = CT_TARGET_TICKS_PER_US;
static CT_CLK_SRC_SEL_T ct_runtime_source = CT_TIMER_SOURCE;
static uint32_t ct_runtime_div_factor = 1U;
static volatile bool ct_clock_enabled = false;

/** Minimum allowed half-interval to prevent timer underflow (100µs floor) */
#define MIN_HALF_INTERVAL_US 100U

/** Maximum half-interval that fits Counter 0's 16-bit match at target rate. */
#define MAX_HALF_INTERVAL_US 30000U

/** Default half-interval if not configured (2.5ms = 5ms full interval) */
#define DEFAULT_HALF_INTERVAL_US 2500U

/* ========================================================================== */
/*                              STATE STRUCTURE                               */
/* ========================================================================== */

/**
 * @brief Hardware timer state structure
 *
 * All timing values are in microseconds unless otherwise noted.
 *
 * Using Counter 0 in 16-bit CT mode. ELRS half intervals fit in one match.
 */
typedef struct {
  /* Timer configuration */
  uint32_t half_interval_us; /**< Base half-interval duration in µs */
  uint32_t interval_us;      /**< Full interval (2 * half_interval_us) */
  uint32_t match_value;      /**< CT Counter 0 match value (16-bit) */

  /* Monotonic timestamp tracking */
  volatile uint32_t total_half_ticks; /**< Total half-tick count since init */

  /* Phase/frequency adjustment - applied at next appropriate edge */
  volatile int32_t pending_phase_shift_us; /**< Pending phase adjustment */
  volatile int32_t freq_offset_units; /**< ELRS freq units, 5 units/us */
  volatile int32_t freq_offset_remainder_units; /**< Fractional carry */

  /* State tracking */
  volatile bool is_initialized; /**< Timer hardware initialized */
  volatile bool is_tock;        /**< true = TOCK (end), false = TICK (mid) */
  volatile bool is_running;     /**< Timer currently running */
  volatile bool is_paused;      /**< Timer paused (connection loss) */

  /* Callbacks */
  hw_timer_tick_callback_t tick_callback;
  hw_timer_tock_callback_t tock_callback;
} hw_timer_state_t;

/** Global timer state */
static hw_timer_state_t hw_timer = {0};

/** Interrupt flag for callback routing */
static volatile uint32_t ct_interrupt_flag = 0;

/* ========================================================================== */
/*                           FORWARD DECLARATIONS                             */
/* ========================================================================== */

static void hw_timer_ct_callback(void *callback_flag);
static uint32_t us_to_match_value(uint32_t us);
static uint32_t clamp_half_interval_us(int32_t interval_us);
static int32_t hw_timer_consume_freq_adjust_us(void);
static uint32_t hw_timer_get_ct_source_hz(CT_CLK_SRC_SEL_T source);
static const char *hw_timer_ct_source_name(CT_CLK_SRC_SEL_T source);
static uint32_t hw_timer_calibrate_ct_frequency(uint32_t register_ct_freq);
static uint32_t hw_timer_read_counter0(void);
static void hw_timer_write_match(uint32_t match_value, bool use_buffer);
static sl_status_t hw_timer_enable_ct_clock(void);
static sl_status_t hw_timer_apply_counter0_config(void);
static void hw_timer_reset_counter0(void);
static void hw_timer_force_stop_counter0(void);

/* ========================================================================== */
/*                           CRITICAL SECTIONS                                */
/* ========================================================================== */

/**
 * @brief Enter critical section (disable interrupts)
 * @return Previous interrupt state for restoration
 */
static inline uint32_t hw_timer_enter_critical(void) {
  uint32_t primask;
  __asm volatile("mrs %0, primask" : "=r"(primask));
  __asm volatile("cpsid i" ::: "memory");
  return primask;
}

/**
 * @brief Exit critical section (restore interrupt state)
 * @param primask Previous interrupt state from enter_critical
 */
static inline void hw_timer_exit_critical(uint32_t primask) {
  __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}

/* ========================================================================== */
/*                           HELPER FUNCTIONS                                 */
/* ========================================================================== */

/**
 * @brief Convert microseconds to CT match value
 * @param us Time in microseconds
 * @return Match value for CT Counter 0 (16-bit)
 *
 * Uses the runtime-calculated ct_ticks_per_us which is set during init
 * based on the actual system clock. Target is 2 MHz (2 ticks per µs).
 *
 * The configured CT frequency is used for conversion, then clamped to the
 * Counter 0 16-bit range. The FreeRTOS-based calibration pass is diagnostic
 * only because scheduler/tick latency is too coarse for ELRS RF slot timing.
 */
static uint32_t us_to_match_value(uint32_t us) {
  /*
   * CT clock is dynamically configured to 2 MHz at init time
   * 1 µs = 2 ticks (ct_ticks_per_us)
   *
   * The return value is clamped to Counter 0's 16-bit match range.
   */
  uint64_t ticks =
      (((uint64_t)us * (uint64_t)ct_freq_hz) + 999999ULL) / 1000000ULL;

  /* Minimum 1 tick */
  if (ticks < 1ULL) {
    ticks = 1ULL;
  }
  if (ticks > CT_MATCH_MAX) {
    ticks = CT_MATCH_MAX;
  }

  return (uint32_t)ticks;
}

static uint32_t clamp_half_interval_us(int32_t interval_us) {
  if (interval_us < (int32_t)MIN_HALF_INTERVAL_US) {
    return MIN_HALF_INTERVAL_US;
  }
  if (interval_us > (int32_t)MAX_HALF_INTERVAL_US) {
    return MAX_HALF_INTERVAL_US;
  }
  return (uint32_t)interval_us;
}

static int32_t hw_timer_consume_freq_adjust_us(void) {
  hw_timer.freq_offset_remainder_units += hw_timer.freq_offset_units;

  const int32_t adjust_us =
      hw_timer.freq_offset_remainder_units / ELRS_FREQ_OFFSET_UNITS_PER_US;
  hw_timer.freq_offset_remainder_units -=
      adjust_us * ELRS_FREQ_OFFSET_UNITS_PER_US;

  return adjust_us;
}

static uint32_t hw_timer_get_ct_source_hz(CT_CLK_SRC_SEL_T source) {
  switch (source) {
  case CT_ULPREFCLK:
    return system_clocks.m4ss_ref_clk != 0 ? system_clocks.m4ss_ref_clk
                                           : DEFAULT_40MHZ_CLOCK;
  case CT_INTFPLLCLK:
    return system_clocks.intf_pll_clock;
  case CT_SOCPLLCLK:
    return system_clocks.soc_pll_clock;
  case M4_SOCCLKFOROTHERCLKSCT:
    return system_clocks.soc_clock != 0 ? system_clocks.soc_clock
                                        : SystemCoreClock;
  default:
    return 0;
  }
}

static const char *hw_timer_ct_source_name(CT_CLK_SRC_SEL_T source) {
  switch (source) {
  case CT_ULPREFCLK:
    return "CT_ULPREFCLK";
  case CT_INTFPLLCLK:
    return "CT_INTFPLLCLK";
  case CT_SOCPLLCLK:
    return "CT_SOCPLLCLK";
  case M4_SOCCLKFOROTHERCLKSCT:
    return "M4_SOCCLKFOROTHERCLKSCT";
  default:
    return "UNKNOWN";
  }
}

static uint32_t hw_timer_read_counter0(void) {
  uint32_t count = 0;
  (void)sl_si91x_config_timer_get_count(SL_COUNTER_16BIT, SL_COUNTER_0,
                                        &count);
  return count & CT_MATCH_MAX;
}

static void hw_timer_write_match(uint32_t match_value, bool use_buffer) {
  if (match_value < 1U) {
    match_value = 1U;
  }
  if (match_value > CT_MATCH_MAX) {
    match_value = CT_MATCH_MAX;
  }

  if (use_buffer) {
    CT->CT_MATCH_BUF_REG_b.COUNTER_0_MATCH_BUF = (uint16_t)match_value;
  } else {
    CT->CT_MATCH_REG_b.COUNTER_0_MATCH = (uint16_t)match_value;
    CT->CT_MATCH_BUF_REG_b.COUNTER_0_MATCH_BUF = (uint16_t)match_value;
  }
}

static sl_status_t hw_timer_enable_ct_clock(void) {
  RSI_CLK_PeripheralClkEnable(M4CLK, CT_CLK, ENABLE_STATIC_CLK);
  const rsi_error_t clk_status = RSI_CLK_CtClkConfig(
      M4CLK, ct_runtime_source, ct_runtime_div_factor, ENABLE_STATIC_CLK);
  if (clk_status == RSI_OK) {
    ct_clock_enabled = true;
    return SL_STATUS_OK;
  }
  return SL_STATUS_FAIL;
}

static sl_status_t hw_timer_apply_counter0_config(void) {
  RSI_CT_Config(CT, 0); /* SDK implementation selects Counter 0 16-bit mode. */

  sl_config_timer_config_t ct_config = {
      .is_counter_mode_32bit_enabled = false,
      .is_counter0_soft_reset_enabled = false,
      .is_counter0_periodic_enabled = true,
      .is_counter0_trigger_enabled = false,
      .is_counter0_sync_trigger_enabled = false,
      .is_counter0_buffer_enabled = true,
      .is_counter1_soft_reset_enabled = false,
      .is_counter1_periodic_enabled = false,
      .is_counter1_trigger_enabled = false,
      .is_counter1_sync_trigger_enabled = false,
      .is_counter1_buffer_enabled = false,
      .counter0_direction = SL_COUNTER0_UP,
      .counter1_direction = SL_COUNTER1_UP,
  };

  sl_status_t status = sl_si91x_config_timer_set_configuration(&ct_config);
  if (status != SL_STATUS_OK) {
    return status;
  }

  status = sl_si91x_config_timer_set_match_count(SL_COUNTER_16BIT,
                                                 SL_COUNTER_0,
                                                 hw_timer.match_value);
  if (status != SL_STATUS_OK) {
    return status;
  }

  hw_timer_write_match(hw_timer.match_value, false);
  return SL_STATUS_OK;
}

static void hw_timer_reset_counter0(void) {
  RSI_CT_ClearControl(CT, COUNTER0_TRIG);
  (void)sl_si91x_config_timer_reset_counter(SL_COUNTER_0);
  NVIC_DisableIRQ(CT_IRQn);
  NVIC_ClearPendingIRQ(CT_IRQn);
}

static void hw_timer_force_stop_counter0(void) {
  hw_timer_reset_counter0();
  /*
   * Do not gate CT_CLK here. Some SDK helpers and diagnostics can touch CT
   * registers while ELRS is disconnected/rate-scanning, and those accesses may
   * hang when the peripheral clock is disabled. Treat "stopped" as a logical
   * state: IRQs are masked, pending IRQs are cleared, and the ISR ignores any
   * late callback until hw_timer_start() marks the timer running again.
   */
}

static uint32_t hw_timer_calibrate_ct_frequency(uint32_t register_ct_freq) {
  if (osKernelGetState() != osKernelRunning) {
    printf("hw_timer: calibration skipped (kernel not running)\n");
    return register_ct_freq;
  }

  hw_timer_reset_counter0();
  hw_timer_write_match(CT_MATCH_MAX, false);

  const uint32_t start_ms = osKernelGetTickCount();
  (void)sl_si91x_config_timer_start_on_software_trigger(SL_COUNTER_0);
  osDelay(CT_CALIBRATION_MS);
  const uint32_t elapsed_ms = osKernelGetTickCount() - start_ms;
  const uint32_t count = hw_timer_read_counter0();
  hw_timer_force_stop_counter0();

  if (elapsed_ms == 0U || count < CT_MIN_VALID_CALIBRATION_TICKS) {
    printf("hw_timer: calibration invalid count=%lu elapsed=%lu ms, using "
           "register rate\n",
           (unsigned long)count, (unsigned long)elapsed_ms);
    return register_ct_freq;
  }

  const uint32_t measured_hz =
      (uint32_t)(((uint64_t)count * 1000ULL) / (uint64_t)elapsed_ms);

  printf("hw_timer: calibrated CT count=%lu over %lu ms => %lu Hz\n",
         (unsigned long)count, (unsigned long)elapsed_ms,
         (unsigned long)measured_hz);

  if (register_ct_freq != 0U) {
    const uint32_t high =
        register_ct_freq > measured_hz ? register_ct_freq : measured_hz;
    const uint32_t low =
        register_ct_freq > measured_hz ? measured_hz : register_ct_freq;

    if (low != 0U && high > (low * 101U / 100U)) {
      printf("hw_timer: measured CT diagnostic=%lu Hz differs from register "
             "base=%lu Hz; using register base\n",
             (unsigned long)measured_hz, (unsigned long)register_ct_freq);
    } else {
      printf("hw_timer: using register CT base=%lu Hz (measured=%lu Hz)\n",
             (unsigned long)register_ct_freq, (unsigned long)measured_hz);
    }
    return register_ct_freq;
  }

  return measured_hz;
}

/**
 * @brief Update the CT match value for interval changes
 * @param interval_us New half-interval in microseconds
 *
 * Uses the buffer register for glitch-free updates when enabled.
 */
static void hw_timer_update_match(uint32_t interval_us) {
  /* Guard: Don't access CT hardware if not initialized */
  if (!hw_timer.is_initialized) {
    return;
  }

  /* Clamp to valid range */
  if (interval_us < MIN_HALF_INTERVAL_US) {
    interval_us = MIN_HALF_INTERVAL_US;
  }
  if (interval_us > MAX_HALF_INTERVAL_US) {
    interval_us = MAX_HALF_INTERVAL_US;
  }

  uint32_t new_match = us_to_match_value(interval_us);
  hw_timer.match_value = new_match;

  /* Buffering is enabled, so update Counter 0 through its match buffer. */
  hw_timer_write_match(new_match, true);
}

/* ========================================================================== */
/*                              ISR CALLBACK                                  */
/* ========================================================================== */

/**
 * @brief CT interrupt callback
 * @param callback_flag Pointer to interrupt flag (not used)
 *
 * Called on every half-interval expiration (counter hits peak/match).
 * Alternates between TICK and TOCK.
 *
 * MATCHES UPSTREAM ELRS ESP32_hwTimer.cpp callback() exactly:
 *   - FreqOffset applied to EVERY half-interval (both TICK and TOCK)
 *   - PhaseShift applied only on TICK->TOCK transition (before TOCK callback)
 *   - isTick toggled AFTER callbacks
 *
 * Citation: ExpressLRS ESP32_hwTimer.cpp lines 92-117
 */
static void hw_timer_ct_callback(void *callback_flag) {
  (void)callback_flag; /* Unused */

  if (!hw_timer.is_running || hw_timer.is_paused) {
    return;
  }

  /* Increment monotonic counter for timestamp tracking */
  hw_timer.total_half_ticks++;

  /*
   * Calculate next interval - FreqOffset applied to EVERY half-interval
   * Citation: ESP32_hwTimer.cpp line 100:
   *   uint32_t NextInterval = (HWtimerInterval >> 1) + FreqOffset;
   */
  int32_t next_interval =
      (int32_t)hw_timer.half_interval_us + hw_timer_consume_freq_adjust_us();

  if (hw_timer.is_tock) {
    /* ============================================================
     * TOCK (isTick == false in upstream) - Packet expected here
     * ============================================================
     * Citation: ESP32_hwTimer.cpp lines 106-111
     *   NextInterval += PhaseShift;
     *   timerAlarmWrite(timer, NextInterval, true);
     *   PhaseShift = 0;
     *   hwTimer::callbackTock();
     */

    /* Apply phase shift on TICK->TOCK (before TOCK fires) */
    next_interval += hw_timer.pending_phase_shift_us;
    hw_timer.pending_phase_shift_us = 0; /* Consume the adjustment */

    /* Update match value for next interval. */
    hw_timer_update_match(clamp_half_interval_us(next_interval));

    /* Invoke TOCK callback (packet timing) */
    if (hw_timer.tock_callback != NULL) {
      hw_timer.tock_callback();
    }

  } else {
    /* ============================================================
     * TICK (isTick == true in upstream) - Mid-interval
     * ============================================================
     * Citation: ESP32_hwTimer.cpp lines 103-104
     *   timerAlarmWrite(timer, NextInterval, true);
     *   hwTimer::callbackTick();
     */

    /* Update match value for next interval */
    hw_timer_update_match(clamp_half_interval_us(next_interval));

    /* Invoke TICK callback */
    if (hw_timer.tick_callback != NULL) {
      hw_timer.tick_callback();
    }
  }

  /* Toggle state AFTER callback - matches upstream line 113:
   * hwTimer::isTick = !hwTimer::isTick;
   */
  hw_timer.is_tock = !hw_timer.is_tock;
}

/* ========================================================================== */
/*                           PUBLIC API FUNCTIONS                             */
/* ========================================================================== */

/**
 * @brief Initialize the hardware timer
 * @param interval_us Full packet interval in microseconds
 * @return SL_STATUS_OK on success, error code otherwise
 *
 * Configures CT Counter 0 in periodic up-count mode with peak interrupt.
 * Clock source is 16MHz PLL for crystal-accurate timing.
 */
sl_status_t hw_timer_init(uint32_t interval_us) {
  sl_status_t status;

  printf(">>> hw_timer_init ENTRY (interval=%lu us) <<<\n", interval_us);

  /* Initialize state */
  memset(&hw_timer, 0, sizeof(hw_timer));

  /* Store interval configuration */
  hw_timer.interval_us = interval_us;
  hw_timer.half_interval_us = interval_us / 2;

  /* Validate interval */
  if (hw_timer.half_interval_us < MIN_HALF_INTERVAL_US) {
    hw_timer.half_interval_us = MIN_HALF_INTERVAL_US;
  }
  if (hw_timer.half_interval_us > MAX_HALF_INTERVAL_US) {
    hw_timer.half_interval_us = MAX_HALF_INTERVAL_US;
  }

  printf("hw_timer: half_interval=%lu us\n", hw_timer.half_interval_us);

  /* Initialize state variables */
  hw_timer.is_tock = true; /* First callback will be TOCK */
  hw_timer.is_running = false;
  hw_timer.is_paused = false;
  hw_timer.total_half_ticks = 0;
  hw_timer.pending_phase_shift_us = 0;
  hw_timer.freq_offset_units = 0;
  hw_timer.freq_offset_remainder_units = 0;

  /* ================================================================
   * DYNAMIC CLOCK CONFIGURATION
   * ================================================================
   * Configure CT clock to 2 MHz regardless of system clock speed.
   * This makes the timer code portable across different clock configurations.
   */

  /* Step 1: Select the same clock that will actually feed the CT peripheral.
   * SystemCoreClock is the M4 core clock, not necessarily the CT source. Using
   * it for CT_SOCPLLCLK made the ELRS cadence run slow when SOC PLL was lower
   * than the core clock.
   */
  uint32_t system_clk = SystemCoreClock;
  CT_CLK_SRC_SEL_T ct_source = CT_TIMER_SOURCE;
  uint32_t ct_source_clk = hw_timer_get_ct_source_hz(ct_source);

  if (ct_source_clk == 0) {
    ct_source = M4_SOCCLKFOROTHERCLKSCT;
    ct_source_clk = hw_timer_get_ct_source_hz(ct_source);
  }

  if (ct_source_clk == 0) {
    ct_source_clk = system_clk;
  }

  printf("hw_timer: SystemCoreClock=%lu Hz\n", system_clk);
  printf("hw_timer: clocks soc=%lu soc_pll=%lu intf_pll=%lu\n",
         system_clocks.soc_clock, system_clocks.soc_pll_clock,
         system_clocks.intf_pll_clock);
  printf("hw_timer: CT source=%s, source_clk=%lu Hz\n",
         hw_timer_ct_source_name(ct_source), ct_source_clk);

  /* Step 2: Calculate the divider to achieve 2 MHz
   * Formula: div_factor = system_clk / (2 * TARGET_FREQ)
   * The CT clock formula is: clk_out = clk_in / (2 * div_factor)
   */
  uint32_t div_factor = ct_source_clk / (2 * CT_TARGET_FREQ_HZ);

  /* Safety: div_factor must fit in 6 bits (max 63) */
  if (div_factor > 63) {
    div_factor = 63;
  }
  if (div_factor == 0) {
    div_factor = 1;
  }

  /* Step 3: Calculate actual CT frequency and ticks per µs */
  uint32_t actual_ct_freq = ct_source_clk / (2 * div_factor);
  ct_freq_hz = actual_ct_freq;
  ct_ticks_per_us = actual_ct_freq / 1000000;
  if (ct_ticks_per_us == 0) {
    ct_ticks_per_us = 1; /* Minimum 1 tick per µs */
  }

  printf("hw_timer: div=%lu, ct_freq=%lu Hz, ticks/us=%lu\n", div_factor,
         actual_ct_freq, ct_ticks_per_us);

  ct_runtime_source = ct_source;
  ct_runtime_div_factor = div_factor;

  /* Step 4: Configure CT clock using RSI API
   * Use CT_SOCPLLCLK as source and apply the calculated divider
   */
  printf("hw_timer: [1/7] RSI_CLK_PeripheralClkEnable...\n");
  RSI_CLK_PeripheralClkEnable(M4CLK, CT_CLK, ENABLE_STATIC_CLK);
  printf("hw_timer: [1/7] DONE\n");

  printf("hw_timer: [2/7] RSI_CLK_CtClkConfig...\n");
  rsi_error_t clk_status =
      RSI_CLK_CtClkConfig(M4CLK, ct_source, div_factor, ENABLE_STATIC_CLK);
  printf("hw_timer: [2/7] status=0x%04lX sel=%lu div=%lu\n",
         (unsigned long)clk_status,
         (unsigned long)M4CLK->CLK_CONFIG_REG5_b.CT_CLK_SEL,
         (unsigned long)M4CLK->CLK_CONFIG_REG5_b.CT_CLK_DIV_FAC);
  if (clk_status != RSI_OK) {
    printf("hw_timer: FAILED at CT clock config!\n");
    return SL_STATUS_FAIL;
  }
  ct_clock_enabled = true;

  uint32_t sdk_ct_freq = RSI_CLK_GetBaseClock(M4_CT);
  if (sdk_ct_freq != 0) {
    actual_ct_freq = sdk_ct_freq;
    ct_freq_hz = actual_ct_freq;
    ct_ticks_per_us = actual_ct_freq / 1000000;
    if (ct_ticks_per_us == 0) {
      ct_ticks_per_us = 1;
    }
  }
  printf("hw_timer: [2/7] actual CT base=%lu Hz, ticks/us=%lu\n",
         actual_ct_freq, ct_ticks_per_us);

  /* Step 5: Keep the hardware in the SDK's Counter 0 16-bit path. */
  printf("hw_timer: [3/7] RSI_CT_Config(CT, 0) for explicit 16-bit...\n");
  RSI_CT_Config(CT, 0); /* SDK implementation clears COUNTER32_BITMODE. */
  printf("hw_timer: [3/7] DONE\n");

  /* Configure CT for periodic up-count mode with buffer. */
  sl_config_timer_config_t ct_config = {
      .is_counter_mode_32bit_enabled = false,
      .is_counter0_soft_reset_enabled = false,
      .is_counter0_periodic_enabled = true, /* Periodic mode - auto reload */
      .is_counter0_trigger_enabled = false, /* We use software trigger */
      .is_counter0_sync_trigger_enabled = false,
      .is_counter0_buffer_enabled =
          true, /* Enable buffer for glitch-free updates */
      .is_counter1_soft_reset_enabled = false,
      .is_counter1_periodic_enabled = false,
      .is_counter1_trigger_enabled = false,
      .is_counter1_sync_trigger_enabled = false,
      .is_counter1_buffer_enabled = false,
      .counter0_direction = SL_COUNTER0_UP, /* Up counter */
      .counter1_direction = SL_COUNTER1_UP,
  };

  printf("hw_timer: [4/7] sl_si91x_config_timer_set_configuration...\n");
  status = sl_si91x_config_timer_set_configuration(&ct_config);
  printf("hw_timer: [4/7] status=0x%04lX\n", (unsigned long)status);
  if (status != SL_STATUS_OK) {
    printf("hw_timer: FAILED at set_configuration!\n");
    return status;
  }

  actual_ct_freq = hw_timer_calibrate_ct_frequency(actual_ct_freq);
  if (actual_ct_freq != 0U) {
    ct_freq_hz = actual_ct_freq;
    ct_ticks_per_us = (actual_ct_freq + 500000U) / 1000000U;
    if (ct_ticks_per_us == 0U) {
      ct_ticks_per_us = 1U;
    }
  }

  status = hw_timer_enable_ct_clock();
  if (status != SL_STATUS_OK) {
    printf("hw_timer: FAILED re-enabling CT clock after calibration!\n");
    return status;
  }

  status = sl_si91x_config_timer_set_configuration(&ct_config);
  if (status != SL_STATUS_OK) {
    printf("hw_timer: FAILED re-applying counter config after calibration!\n");
    return status;
  }

  hw_timer.match_value = us_to_match_value(hw_timer.half_interval_us);
  printf("hw_timer: final ct_freq=%lu Hz, ticks/us=%lu, match_value=%lu "
         "(16-bit max=%lu)\n",
         (unsigned long)ct_freq_hz, (unsigned long)ct_ticks_per_us,
         (unsigned long)hw_timer.match_value, (unsigned long)CT_MATCH_MAX);

  /* Set initial match value (counter 0) */
  printf("hw_timer: [5/7] sl_si91x_config_timer_set_match_count(16BIT, CNT0, "
         "%lu)...\n",
         hw_timer.match_value);
  status = sl_si91x_config_timer_set_match_count(SL_COUNTER_16BIT, SL_COUNTER_0,
                                                 hw_timer.match_value);
  printf("hw_timer: [5/7] status=0x%04lX\n", (unsigned long)status);
  if (status != SL_STATUS_OK) {
    printf("hw_timer: FAILED at set_match_count!\n");
    return status;
  }
  hw_timer_write_match(hw_timer.match_value, false);

  /* Configure interrupt flags - enable peak (match) interrupt for counter 0 */
  sl_config_timer_interrupt_flags_t int_flags = {
      .is_counter0_event_interrupt_enabled = false,
      .is_counter0_fifo_full_interrupt_enabled = false,
      .is_counter0_hit_zero_interrupt_enabled = false,
      .is_counter0_hit_peak_interrupt_enabled = true, /* Fire on match */
      .is_counter1_event_interrupt_enabled = false,
      .is_counter1_fifo_full_interrupt_enabled = false,
      .is_counter1_hit_zero_interrupt_enabled = false,
      .is_counter1_hit_peak_interrupt_enabled = false,
  };

  /* Clear any pending CT interrupt before registering callback */
  printf("hw_timer: [6/7] Clearing pending CT IRQ...\n");
  NVIC_ClearPendingIRQ(CT_IRQn);

  /* Set CT interrupt to priority 6 (below DMA at priority 5)
   *
   * CRITICAL: ELRS rate hopping performs SPI transitions from inside this ISR.
   * If CT_IRQn is at priority 5 (matching DMA), the DMA completion interrupt
   * cannot preempt this ISR, leading to a deadlock. Priority 6 ensures
   * the GSPI driver can always complete its transfer and clear the busy flag.
   */
  NVIC_SetPriority(CT_IRQn, 6);
  printf("hw_timer: [6/7] DONE (priority=6, preemptible by DMA)\n");

  /* Unregister any existing callback first (in case of re-init) */
  printf("hw_timer: [7/7] Unregistering existing callback (if any)...\n");
  sl_si91x_config_timer_unregister_callback(&int_flags);

  /* Register callback for CT interrupts */
  printf("hw_timer: [8/8] sl_si91x_config_timer_register_callback...\n");
  status = sl_si91x_config_timer_register_callback(
      hw_timer_ct_callback, (void *)&ct_interrupt_flag, &int_flags);
  printf("hw_timer: [8/8] status=0x%04lX\n", (unsigned long)status);

  if (status != SL_STATUS_OK) {
    printf("hw_timer: FAILED at register_callback!");
  } else {
    hw_timer.is_initialized = true;
    hw_timer_force_stop_counter0();
    printf("hw_timer: init COMPLETE OK (counter stopped)\n");
  }

  return status;
}

/**
 * @brief Pause the CT hardware timer ISR
 * Called by the SPI driver to prevent SPI reentrancy
 */
void hw_timer_pause_isr(void) {
  if (hw_timer.is_initialized) {
    NVIC_DisableIRQ(CT_IRQn);
  }
}

/**
 * @brief Resume the CT hardware timer ISR
 */
void hw_timer_resume_isr(void) {
  if (hw_timer.is_initialized && hw_timer.is_running && !hw_timer.is_paused) {
    NVIC_EnableIRQ(CT_IRQn);
  }
}

/**
 * @brief Start the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_start(void) {
  if (hw_timer.is_running) {
    return SL_STATUS_OK; /* Already running */
  }

  hw_timer_force_stop_counter0();

  sl_status_t status = hw_timer_enable_ct_clock();
  if (status != SL_STATUS_OK) {
    return status;
  }

  status = hw_timer_apply_counter0_config();
  if (status != SL_STATUS_OK) {
    hw_timer_force_stop_counter0();
    return status;
  }

  hw_timer_reset_counter0();
  hw_timer_write_match(hw_timer.match_value, false);

  hw_timer.is_running = true;
  hw_timer.is_paused = false;
  hw_timer.is_tock = true; /* First callback will be TOCK */

  NVIC_ClearPendingIRQ(CT_IRQn);
  NVIC_EnableIRQ(CT_IRQn);

  /* Start counter 0 via software trigger */
  status = sl_si91x_config_timer_start_on_software_trigger(SL_COUNTER_0);
  if (status != SL_STATUS_OK) {
    hw_timer.is_running = false;
    hw_timer.is_paused = false;
    hw_timer_force_stop_counter0();
  }
  return status;
}

/**
 * @brief Stop the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_stop(void) {
  hw_timer.is_running = false;
  hw_timer.is_paused = false;

  hw_timer_force_stop_counter0();
  return SL_STATUS_OK;
}

/**
 * @brief Deinitialize the hardware timer
 *
 * Stops the timer and unregisters the callback. Call this during shutdown.
 */
void hw_timer_deinit(void) {
  /* Stop timer if running */
  if (hw_timer.is_running) {
    hw_timer_stop();
    hw_timer.is_running = false;
  }

  /* Unregister callback */
  sl_config_timer_interrupt_flags_t int_flags = {
      .is_counter0_hit_peak_interrupt_enabled = true,
  };
  sl_si91x_config_timer_unregister_callback(&int_flags);

  /* Clear callbacks */
  hw_timer.tick_callback = NULL;
  hw_timer.tock_callback = NULL;

  /* Reset state */
  hw_timer.is_paused = false;
  hw_timer.total_half_ticks = 0;
}

/**
 * @brief Pause the timer (ELRS 4.0 connection loss handling)
 *
 * Pauses timing without losing state. Used when connection is lost
 * to prevent the timer from continuing to fire callbacks.
 */
void hw_timer_pause(void) {
  if (hw_timer.is_running && !hw_timer.is_paused) {
    sl_si91x_config_timer_select_action_event(HALT, SL_NO_EVENT, SL_NO_EVENT);
    hw_timer.is_paused = true;
  }
}

/**
 * @brief Resume the timer after pause
 *
 * Resumes timing after a pause. The timer will continue from
 * where it left off in terms of TICK/TOCK state.
 */
void hw_timer_resume(void) {
  if (hw_timer.is_running && hw_timer.is_paused) {
    /* Reset to known state - start with TOCK */
    hw_timer.is_tock = true;

    /* Resume from halt */
    sl_si91x_config_timer_resume_halt_event(SL_COUNTER_0);
    hw_timer.is_paused = false;
  }
}

void hw_timer_note_immediate_tock(void) {
  uint32_t primask = hw_timer_enter_critical();
  hw_timer.is_tock = false;
  hw_timer_exit_critical(primask);
}

/**
 * @brief Get current timestamp in microseconds
 * @return Monotonic timestamp in microseconds
 *
 * Calculation:
 *   timestamp = (total_half_ticks * half_interval_us) +
 * elapsed_in_current_period
 *
 * Where elapsed_in_current_period is derived from the counter value:
 *   For up-counter: elapsed = current_count / ticks_per_us
 */
uint32_t hw_timer_get_micros(void) {
  uint32_t count = 0;
  uint32_t half_ticks;
  uint32_t elapsed;

  /* Critical section to ensure atomic read */
  uint32_t primask = hw_timer_enter_critical();

  /* Reading CT registers while the peripheral clock is gated can hang. */
  if (ct_clock_enabled && hw_timer.is_running && !hw_timer.is_paused) {
    sl_si91x_config_timer_get_count(SL_COUNTER_16BIT, SL_COUNTER_0, &count);
  }

  /* Capture the counter atomically with the timer read */
  half_ticks = hw_timer.total_half_ticks;

  hw_timer_exit_critical(primask);

  /* Calculate elapsed time in current half-interval
   * CT counts UP, so count directly represents elapsed ticks
   *
   * We use the ratio of current count to match value, scaled by
   * half_interval_us: elapsed = (count * half_interval_us) / match_value
   *
   * This avoids needing to know the exact clock frequency.
   */
  if (hw_timer.match_value > 0) {
    elapsed = (count * hw_timer.half_interval_us) / hw_timer.match_value;
  } else {
    elapsed = 0;
  }

  /* Total time = completed half-ticks + elapsed in current */
  return (half_ticks * hw_timer.half_interval_us) + elapsed;
}

/**
 * @brief Apply a phase shift to the timer
 * @param shift_us Phase shift in microseconds (positive = delay, negative =
 * advance)
 *
 * The phase shift is applied on the next TOCK transition.
 * This effectively moves when the next TOCK (packet expected) occurs.
 */
void hw_timer_phase_shift(int32_t shift_us) {
  uint32_t primask = hw_timer_enter_critical();

  /* Clamp phase shift to prevent extreme values */
  int32_t max_shift =
      (int32_t)(MAX_HALF_INTERVAL_US - hw_timer.half_interval_us);
  int32_t min_shift =
      (int32_t)(MIN_HALF_INTERVAL_US - hw_timer.half_interval_us);

  if (shift_us > max_shift) {
    shift_us = max_shift;
  }
  if (shift_us < min_shift) {
    shift_us = min_shift;
  }

  hw_timer.pending_phase_shift_us = shift_us;

  hw_timer_exit_critical(primask);
}

/**
 * @brief Increment the frequency offset
 * @param delta Offset increment in upstream ELRS timer units
 *
 * The offset is applied to EVERY half-interval, matching upstream ELRS.
 * One unit is one ESP32 RX timer tick, or 1/5 us.
 */
void hw_timer_inc_freq_offset(int32_t delta) {
  uint32_t primask = hw_timer_enter_critical();

  hw_timer.freq_offset_units += delta;

  /* 500 upstream units is +/-100 us of average correction. */
  const int32_t max_freq_offset_units = 500;
  if (hw_timer.freq_offset_units > max_freq_offset_units) {
    hw_timer.freq_offset_units = max_freq_offset_units;
  }
  if (hw_timer.freq_offset_units < -max_freq_offset_units) {
    hw_timer.freq_offset_units = -max_freq_offset_units;
  }

  hw_timer_exit_critical(primask);
}

/**
 * @brief Reset the frequency offset to zero
 *
 * Called when resynchronizing or changing packet rates.
 */
void hw_timer_reset_freq_offset(void) {
  uint32_t primask = hw_timer_enter_critical();
  hw_timer.freq_offset_units = 0;
  hw_timer.freq_offset_remainder_units = 0;
  hw_timer_exit_critical(primask);
}

/**
 * @brief Set the packet interval
 * @param interval_us New full interval in microseconds
 *
 * Updates the timer interval for a new packet rate.
 * Takes effect on the next timer reconfiguration.
 *
 * The active ELRS RX intervals fit in Counter 0's 16-bit match range.
 */
void hw_timer_set_interval(uint32_t interval_us) {
  uint32_t primask = hw_timer_enter_critical();

  hw_timer.interval_us = interval_us;
  hw_timer.half_interval_us = interval_us / 2;

  /* Clamp to valid range */
  if (hw_timer.half_interval_us < MIN_HALF_INTERVAL_US) {
    hw_timer.half_interval_us = MIN_HALF_INTERVAL_US;
  }
  if (hw_timer.half_interval_us > MAX_HALF_INTERVAL_US) {
    hw_timer.half_interval_us = MAX_HALF_INTERVAL_US;
  }

  /* Calculate match value for Counter 0. */
  hw_timer.match_value = us_to_match_value(hw_timer.half_interval_us);

  hw_timer_exit_critical(primask);

  /* Apply new interval immediately if running */
  if (hw_timer.is_running && !hw_timer.is_paused) {
    hw_timer_update_match(hw_timer.half_interval_us);
  }
}

/**
 * @brief Register the TICK callback
 * @param callback Function to call on TICK (mid-interval)
 */
void hw_timer_set_tick_callback(hw_timer_tick_callback_t callback) {
  hw_timer.tick_callback = callback;
}

/**
 * @brief Register the TOCK callback
 * @param callback Function to call on TOCK (end of interval, packet expected)
 */
void hw_timer_set_tock_callback(hw_timer_tock_callback_t callback) {
  hw_timer.tock_callback = callback;
}

/**
 * @brief Check if timer is currently running
 * @return true if timer is running, false otherwise
 */
bool hw_timer_is_running(void) {
  return hw_timer.is_running && !hw_timer.is_paused;
}

/**
 * @brief Get the current frequency offset
 * @return Current frequency offset in microseconds
 */
int32_t hw_timer_get_freq_offset(void) { return hw_timer.freq_offset_units; }

/**
 * @brief Get the current timer interval
 * @return Current full interval in microseconds
 */
uint32_t hw_timer_get_interval(void) { return hw_timer.interval_us; }

uint32_t hw_timer_get_total_half_ticks(void) {
  return hw_timer.total_half_ticks;
}

uint32_t hw_timer_get_match_value(void) { return hw_timer.match_value; }

uint32_t hw_timer_get_ct_freq_hz(void) { return ct_freq_hz; }

uint32_t hw_timer_get_current_count(void) {
  if (!ct_clock_enabled || !hw_timer.is_running || hw_timer.is_paused) {
    return 0;
  }
  return hw_timer_read_counter0();
}
