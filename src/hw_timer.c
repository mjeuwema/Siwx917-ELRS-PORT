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
#include "rsi_ct.h" /* For RSI_CT_Config(), RSI_CT_Reset() */
#include "rsi_pll.h"
#include "rsi_rom_clks.h" /* For RSI_CLK_GetBaseClock(), RSI_CLK_SetCtClock() */
#include "sl_si91x_config_timer.h"
#include <stdio.h> /* For printf debug output */
#include <string.h>

/* ========================================================================== */
/*                              CONFIGURATION                                 */
/* ========================================================================== */

/**
 * CT Clock Configuration - DYNAMIC with 32-BIT MODE
 *
 * We dynamically configure the CT clock divider at init time to achieve
 * a target frequency (2 MHz), regardless of the actual system clock speed.
 * This makes the code portable across different clock configurations.
 *
 * 32-BIT MODE:
 *   The unified SDK only supports 16-bit mode, but the hardware supports
 * 32-bit. We use the low-level RSI_CT_Config(CT, 0) to enable 32-bit mode. This
 * is required for certain ELRS modes with longer intervals.
 *
 * Target: 2 MHz CT clock
 *   - 1 µs = 2 ticks (simple math!)
 *   - Resolution: 0.5 µs
 *   - 32-bit max: 2^32 / 2 MHz = 2147 seconds (way more than needed!)
 *
 * Formula: div_factor = system_clk / (2 * TARGET_TIMER_FREQ)
 * Example: 180 MHz / (2 * 2 MHz) = 45
 *          160 MHz / (2 * 2 MHz) = 40
 *
 * Citation: rsi_ct.h - RSI_CT_Config(pCT, cfg) where cfg=0 for 32-bit mode
 */
#define CT_TARGET_FREQ_HZ 2000000U /* 2 MHz target */
#define CT_TARGET_TICKS_PER_US 2U  /* At 2 MHz: 1 µs = 2 ticks */

/** Runtime-calculated ticks per µs (set during init) */
static uint32_t ct_ticks_per_us = CT_TARGET_TICKS_PER_US;

/** Minimum allowed half-interval to prevent timer underflow (100µs floor) */
#define MIN_HALF_INTERVAL_US 100U

/** Maximum allowed half-interval - 32-bit at 2MHz = 2147 seconds max! */
#define MAX_HALF_INTERVAL_US 1000000U /* 1 second max (practical limit) */

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
 * Using 32-bit CT mode: no cascading needed, direct µs to ticks conversion.
 */
typedef struct {
  /* Timer configuration */
  uint32_t half_interval_us; /**< Base half-interval duration in µs */
  uint32_t interval_us;      /**< Full interval (2 * half_interval_us) */
  uint32_t match_value;      /**< CT match value (32-bit) */

  /* Monotonic timestamp tracking */
  volatile uint32_t total_half_ticks; /**< Total half-tick count since init */

  /* Phase/frequency adjustment - applied at next appropriate edge */
  volatile int32_t pending_phase_shift_us; /**< Pending phase adjustment */
  volatile int32_t freq_offset_us; /**< Frequency offset per half-interval */

  /* State tracking */
  volatile bool is_initialized; /**< Timer hardware initialized */
  volatile bool is_tock;    /**< true = TOCK (end), false = TICK (mid) */
  volatile bool is_running; /**< Timer currently running */
  volatile bool is_paused;  /**< Timer paused (connection loss) */

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
 * @return Match value for CT (32-bit)
 *
 * Uses the runtime-calculated ct_ticks_per_us which is set during init
 * based on the actual system clock. Target is 2 MHz (2 ticks per µs).
 *
 * 32-bit mode: max value = 2^32-1 = 4,294,967,295 ticks
 * At 2 MHz: max interval = 2147 seconds (way more than needed!)
 */
static uint32_t us_to_match_value(uint32_t us) {
  /*
   * CT clock is dynamically configured to 2 MHz at init time
   * 1 µs = 2 ticks (ct_ticks_per_us)
   *
   * 32-bit mode - no clamping needed for practical ELRS intervals
   */
  uint32_t match_value = us * ct_ticks_per_us;

  /* Minimum 1 tick */
  if (match_value < 1) {
    match_value = 1;
  }

  return match_value;
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

  /* Update match value - CT API handles buffer if enabled */
  sl_si91x_config_timer_set_match_count(SL_COUNTER_16BIT, SL_COUNTER_0,
                                        new_match);
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

  /* Increment monotonic counter for timestamp tracking */
  hw_timer.total_half_ticks++;

  /*
   * Calculate next interval - FreqOffset applied to EVERY half-interval
   * Citation: ESP32_hwTimer.cpp line 100:
   *   uint32_t NextInterval = (HWtimerInterval >> 1) + FreqOffset;
   */
  int32_t next_interval =
      (int32_t)hw_timer.half_interval_us + hw_timer.freq_offset_us;

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

    /* Update match value for next interval (32-bit mode handles any value) */
    hw_timer_update_match((uint32_t)next_interval);

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
    hw_timer_update_match((uint32_t)next_interval);

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
  hw_timer.freq_offset_us = 0;

  /* ================================================================
   * DYNAMIC CLOCK CONFIGURATION
   * ================================================================
   * Configure CT clock to 2 MHz regardless of system clock speed.
   * This makes the timer code portable across different clock configurations.
   */

  /* Step 1: Get the current system clock speed from CMSIS variable */
  extern uint32_t SystemCoreClock;
  uint32_t system_clk = SystemCoreClock;
  printf("hw_timer: SystemCoreClock=%lu Hz\n", system_clk);

  /* Step 2: Calculate the divider to achieve 2 MHz
   * Formula: div_factor = system_clk / (2 * TARGET_FREQ)
   * The CT clock formula is: clk_out = clk_in / (2 * div_factor)
   */
  uint32_t div_factor = system_clk / (2 * CT_TARGET_FREQ_HZ);

  /* Safety: div_factor must fit in 6 bits (max 63) */
  if (div_factor > 63) {
    div_factor = 63;
  }
  if (div_factor == 0) {
    div_factor = 1;
  }

  /* Step 3: Calculate actual CT frequency and ticks per µs */
  uint32_t actual_ct_freq = system_clk / (2 * div_factor);
  ct_ticks_per_us = actual_ct_freq / 1000000;
  if (ct_ticks_per_us == 0) {
    ct_ticks_per_us = 1; /* Minimum 1 tick per µs */
  }

  printf("hw_timer: div=%lu, ct_freq=%lu Hz, ticks/us=%lu\n", div_factor,
         actual_ct_freq, ct_ticks_per_us);

  /* Step 4: Configure CT clock using RSI API
   * Use CT_SOCPLLCLK as source and apply the calculated divider
   */
  printf("hw_timer: [1/7] RSI_CLK_PeripheralClkEnable...\n");
  RSI_CLK_PeripheralClkEnable(M4CLK, CT_CLK, ENABLE_STATIC_CLK);
  printf("hw_timer: [1/7] DONE\n");

  printf("hw_timer: [2/7] RSI_CLK_CtClkConfig...\n");
  RSI_CLK_CtClkConfig(M4CLK, CT_SOCPLLCLK, div_factor, ENABLE_STATIC_CLK);
  printf("hw_timer: [2/7] DONE\n");

  /* Step 5: Enable 32-bit mode using low-level RSI API
   * The unified SDK only supports 16-bit, but hardware supports 32-bit.
   * RSI_CT_Config(CT, 0) enables 32-bit mode (cfg=0 for 32-bit, cfg=1 for
   * 16-bit) This combines both 16-bit counters into one 32-bit counter.
   */
  printf("hw_timer: [3/7] RSI_CT_Config(CT, 0) for 32-bit...\n");
  RSI_CT_Config(CT, 0); /* 0 = 32-bit mode */
  printf("hw_timer: [3/7] DONE\n");

  /* Calculate match value for the half-interval */
  hw_timer.match_value = us_to_match_value(hw_timer.half_interval_us);
  printf("hw_timer: match_value=%lu (16-bit max=65535)\n",
         hw_timer.match_value);

  /* Configure CT for periodic up-count mode with buffer
   * Note: We set is_counter_mode_32bit_enabled=true, but the SDK ignores it.
   * The actual 32-bit mode is enabled via RSI_CT_Config() above.
   */
  sl_config_timer_config_t ct_config = {
      .is_counter_mode_32bit_enabled =
          true, /* Request 32-bit mode (actual enable via RSI_CT_Config) */
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
  printf("hw_timer: [6/7] DONE\n");

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
    printf("hw_timer: init COMPLETE OK\n");
  }

  return status;
}

/**
 * @brief Start the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_start(void) {
  if (hw_timer.is_running) {
    return SL_STATUS_OK; /* Already running */
  }

  hw_timer.is_running = true;
  hw_timer.is_paused = false;
  hw_timer.is_tock = true; /* First callback will be TOCK */

  /* Reset counter to 0 before starting */
  sl_si91x_config_timer_reset_counter(SL_COUNTER_0);

  /* Start counter 0 via software trigger */
  return sl_si91x_config_timer_start_on_software_trigger(SL_COUNTER_0);
}

/**
 * @brief Stop the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_stop(void) {
  hw_timer.is_running = false;

  /* Stop by selecting no event (effectively halts counter) */
  return sl_si91x_config_timer_select_action_event(STOP, SL_NO_EVENT,
                                                   SL_NO_EVENT);
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

  /* Read current counter value */
  sl_si91x_config_timer_get_count(SL_COUNTER_16BIT, SL_COUNTER_0, &count);

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
 * @param delta Offset increment in microseconds
 *
 * The freq_offset is applied to EVERY half-interval, matching upstream ELRS.
 */
void hw_timer_inc_freq_offset(int32_t delta) {
  uint32_t primask = hw_timer_enter_critical();

  hw_timer.freq_offset_us += delta;

  /* Clamp frequency offset to reasonable bounds
   * Typical ELRS freq_offset is within ±100µs
   */
  const int32_t max_freq_offset = 500; /* ±500µs max */
  if (hw_timer.freq_offset_us > max_freq_offset) {
    hw_timer.freq_offset_us = max_freq_offset;
  }
  if (hw_timer.freq_offset_us < -max_freq_offset) {
    hw_timer.freq_offset_us = -max_freq_offset;
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
  hw_timer.freq_offset_us = 0;
  hw_timer_exit_critical(primask);
}

/**
 * @brief Set the packet interval
 * @param interval_us New full interval in microseconds
 *
 * Updates the timer interval for a new packet rate.
 * Takes effect on the next timer reconfiguration.
 *
 * With 32-bit mode at 16 MHz, any interval up to 268 seconds is supported
 * directly.
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

  /* Calculate match value (32-bit mode handles any value) */
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
int32_t hw_timer_get_freq_offset(void) { return hw_timer.freq_offset_us; }

/**
 * @brief Get the current timer interval
 * @return Current full interval in microseconds
 */
uint32_t hw_timer_get_interval(void) { return hw_timer.interval_us; }
