/**
 * @file hw_timer.c
 * @brief Hardware timer implementation for ELRS using SiWx917 ULP Timer
 *
 * This module provides precise timing for the ELRS protocol using ULP Timer 1.
 * The timer operates at 1µs resolution using the 32MHz reference clock.
 *
 * ELRS Timing Model:
 * - The packet interval is divided into two half-intervals (TICK and TOCK)
 * - TICK fires at the mid-point of the interval
 * - TOCK fires at the end of the interval (packet expected)
 * - Phase adjustments shift when the next TICK occurs
 * - Frequency adjustments modify the interval length for clock drift compensation
 *
 * Hardware Notes (SiWx917 RM Section 33):
 * - Timer settings are LOCKED while timer is running (Section 33.4.1)
 * - Must STOP timer before modifying MATCH register
 * - Timer counts DOWN from MATCH value to 0
 * - Reading count register returns time remaining before timeout
 *
 * @copyright Copyright (c) 2024-2026
 */

#include "hw_timer.h"
#include "sl_si91x_ulp_timer.h"
#include "rsi_rom_ulpss_clk.h"
#include <string.h>

/* ========================================================================== */
/*                              CONFIGURATION                                 */
/* ========================================================================== */

/** Timer instance used for ELRS timing (ULP Timer 1) */
#define ELRS_HW_TIMER_INSTANCE       ULP_TIMER_1

/** Minimum allowed half-interval to prevent timer underflow (100µs floor) */
#define MIN_HALF_INTERVAL_US         100U

/** Maximum allowed half-interval to prevent overflow (50ms ceiling) */
#define MAX_HALF_INTERVAL_US         50000U

/** Default half-interval if not configured (2.5ms = 5ms full interval) */
#define DEFAULT_HALF_INTERVAL_US     2500U

/* ========================================================================== */
/*                              STATE STRUCTURE                               */
/* ========================================================================== */

/**
 * @brief Hardware timer state structure
 *
 * All timing values are in microseconds unless otherwise noted.
 */
typedef struct {
    /* Timer configuration */
    uint32_t half_interval_us;      /**< Base half-interval duration */
    uint32_t interval_us;           /**< Full interval (2 * half_interval_us) */

    /* Monotonic timestamp tracking - FIX #1: Single counter approach */
    volatile uint32_t total_half_ticks;  /**< Total half-tick count since init */

    /* Phase/frequency adjustment - applied at next appropriate edge */
    volatile int32_t pending_phase_shift_us;   /**< Pending phase adjustment */
    volatile int32_t freq_offset_us;           /**< Frequency offset per FULL interval */

    /* State tracking */
    volatile bool is_tock;          /**< true = TOCK (end), false = TICK (mid) */
    volatile bool is_running;       /**< Timer currently running */
    volatile bool is_paused;        /**< Timer paused (connection loss) */

    /* Callbacks */
    hw_timer_tick_callback_t tick_callback;
    hw_timer_tock_callback_t tock_callback;
} hw_timer_state_t;

/** Global timer state */
static hw_timer_state_t hw_timer = {0};

/** ULP Timer configuration structure */
static ulp_timer_config_t ulp_timer_config = {
    .timer_num        = ELRS_HW_TIMER_INSTANCE,
    .timer_mode       = ULP_TIMER_MODE_PERIODIC,
    .timer_type       = ULP_TIMER_TYP_1US,
    .timer_match_value= DEFAULT_HALF_INTERVAL_US,
    .timer_direction  = DOWN_COUNTER
};

/* ========================================================================== */
/*                           FORWARD DECLARATIONS                             */
/* ========================================================================== */

static void hw_timer_ulp_callback(void);
static void hw_timer_reconfigure_interval(uint32_t new_interval_us);

/* ========================================================================== */
/*                           CRITICAL SECTIONS                                */
/* ========================================================================== */

/**
 * @brief Enter critical section (disable interrupts)
 * @return Previous interrupt state for restoration
 *
 * FIX #4: Atomicity protection for ISR-shared variables
 * Citation: ARM Cortex-M4 TRM - PRIMASK register disables all configurable interrupts
 */
static inline uint32_t hw_timer_enter_critical(void)
{
    uint32_t primask;
    __asm volatile ("mrs %0, primask" : "=r" (primask));
    __asm volatile ("cpsid i" ::: "memory");
    return primask;
}

/**
 * @brief Exit critical section (restore interrupt state)
 * @param primask Previous interrupt state from enter_critical
 */
static inline void hw_timer_exit_critical(uint32_t primask)
{
    __asm volatile ("msr primask, %0" :: "r" (primask) : "memory");
}

/* ========================================================================== */
/*                           TIMER RECONFIGURATION                            */
/* ========================================================================== */

/**
 * @brief Reconfigure timer interval while running
 * @param new_interval_us New interval value in microseconds
 *
 * FIX #5: Timer must be stopped before modifying MATCH register
 *
 * Citation: SiWx917 RM Section 33.4.1:
 * "Timer settings are locked while the timer is running. To reprogram
 * the timer, you must first set the MCUULP_TMRx_CNTRL.TMR_STOP bit."
 *
 * This function performs an atomic stop-reconfigure-start sequence.
 * The brief pause (<1µs) is acceptable for ELRS timing requirements.
 */
static void hw_timer_reconfigure_interval(uint32_t new_interval_us)
{
    /* Clamp to valid range - FIX #3: Both lower AND upper bounds */
    if (new_interval_us < MIN_HALF_INTERVAL_US) {
        new_interval_us = MIN_HALF_INTERVAL_US;
    }
    if (new_interval_us > MAX_HALF_INTERVAL_US) {
        new_interval_us = MAX_HALF_INTERVAL_US;
    }

    /*
     * Atomic stop-reconfigure-start sequence
     * Citation: SiWx917 RM Section 33.4.1 - must stop before write
     */
    sl_si91x_ulp_timer_stop(ELRS_HW_TIMER_INSTANCE);
    sl_si91x_ulp_timer_set_count(ELRS_HW_TIMER_INSTANCE, new_interval_us);
    sl_si91x_ulp_timer_start(ELRS_HW_TIMER_INSTANCE);
}

/* ========================================================================== */
/*                              ISR CALLBACK                                  */
/* ========================================================================== */

/**
 * @brief ULP Timer interrupt callback
 *
 * Called on every half-interval expiration. Alternates between TICK and TOCK.
 *
 * Timing adjustments:
 * - Phase shift: Applied on TOCK->TICK transition (shifts mid-point)
 * - Frequency offset: Applied on TICK->TOCK transition ONLY (FIX #2)
 *
 * FIX #1: Single counter (total_half_ticks) for monotonic timestamps
 * FIX #2: freq_offset applied once per full cycle (on TOCK only)
 * FIX #5: Uses stop-reconfigure-start for interval changes
 *
 * Citation: ELRS PFD expects freq_offset to adjust the FULL interval,
 * not each half-interval. Applying twice would double PFD sensitivity.
 */
static void hw_timer_ulp_callback(void)
{
    /* FIX #1: Increment single monotonic counter */
    hw_timer.total_half_ticks++;

    if (hw_timer.is_tock) {
        /* ============================================================
         * TOCK (End of interval) - Packet expected here
         * ============================================================
         * Next half-interval is TICK (mid-point)
         * Apply pending phase shift to adjust synchronization
         */
        hw_timer.is_tock = false;

        /* Calculate next interval with phase adjustment */
        int32_t next_interval = (int32_t)hw_timer.half_interval_us;

        /* Apply pending phase shift (consumed after use) */
        if (hw_timer.pending_phase_shift_us != 0) {
            next_interval += hw_timer.pending_phase_shift_us;
            hw_timer.pending_phase_shift_us = 0;  /* Consume the adjustment */
        }

        /* Reconfigure timer if interval changed */
        if (next_interval != (int32_t)hw_timer.half_interval_us) {
            hw_timer_reconfigure_interval((uint32_t)next_interval);
        }

        /* Invoke TOCK callback (packet timing) */
        if (hw_timer.tock_callback != NULL) {
            hw_timer.tock_callback();
        }

    } else {
        /* ============================================================
         * TICK (Mid-interval) - Halfway point
         * ============================================================
         * Next half-interval is TOCK (end of interval)
         * FIX #2: Apply frequency offset HERE ONLY (once per full cycle)
         */
        hw_timer.is_tock = true;

        /* Calculate next interval with frequency adjustment */
        int32_t next_interval = (int32_t)hw_timer.half_interval_us;

        /*
         * FIX #2: Apply freq_offset only on TICK->TOCK transition
         * This means freq_offset affects the FULL interval once,
         * matching official ELRS PFD expectations.
         *
         * Citation: ELRS FreqOffset represents adjustment for entire
         * packet interval. Applying to both halves would double sensitivity.
         */
        next_interval += hw_timer.freq_offset_us;

        /* Reconfigure timer if interval changed */
        if (next_interval != (int32_t)hw_timer.half_interval_us) {
            hw_timer_reconfigure_interval((uint32_t)next_interval);
        }

        /* Invoke TICK callback */
        if (hw_timer.tick_callback != NULL) {
            hw_timer.tick_callback();
        }
    }
}

/* ========================================================================== */
/*                           PUBLIC API FUNCTIONS                             */
/* ========================================================================== */

/**
 * @brief Initialize the hardware timer
 * @param interval_us Full packet interval in microseconds
 * @return SL_STATUS_OK on success, error code otherwise
 *
 * Configures ULP Timer 1 in periodic mode with 1µs resolution.
 *
 * Citation: SiWx917 RM Section 6.14.13.10 - ULP_TIMER_CLK_SEL
 * Value 4 selects RC_32MHZ_CLK as timer clock source.
 */
sl_status_t hw_timer_init(uint32_t interval_us)
{
    sl_status_t status;

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

    /* Initialize state variables */
    hw_timer.is_tock = true;            /* First callback will be TOCK */
    hw_timer.is_running = false;
    hw_timer.is_paused = false;
    hw_timer.total_half_ticks = 0;      /* FIX #1: Initialize monotonic counter */
    hw_timer.pending_phase_shift_us = 0;
    hw_timer.freq_offset_us = 0;

    /*
     * Configure ULP timer clock source
     * Citation: SiWx917 RM Section 6.14.13.10
     * ULPSS_ULP_TIMER_CLK_SEL = 4 -> RC_32MHZ_CLK
     */
    RSI_ULPSS_TimerClkConfig(ULPCLK,
                             ENABLE_STATIC_CLK,
                             0,      /* No clock division */
                             ULP_TIMER_REF_CLK,
                             0);     /* Skip wait states */

    /* Configure timer parameters */
    ulp_timer_config.timer_match_value = hw_timer.half_interval_us;

    status = sl_si91x_ulp_timer_set_configuration(&ulp_timer_config);
    if (status != SL_STATUS_OK) {
        return status;
    }

    /* Register interrupt callback */
    status = sl_si91x_ulp_timer_register_timeout_callback(
        ELRS_HW_TIMER_INSTANCE,
        hw_timer_ulp_callback);

    return status;
}

/**
 * @brief Start the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_start(void)
{
    if (hw_timer.is_running) {
        return SL_STATUS_OK;  /* Already running */
    }

    hw_timer.is_running = true;
    hw_timer.is_paused = false;
    hw_timer.is_tock = true;  /* First callback will be TOCK */

    return sl_si91x_ulp_timer_start(ELRS_HW_TIMER_INSTANCE);
}

/**
 * @brief Stop the hardware timer
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_stop(void)
{
    hw_timer.is_running = false;
    return sl_si91x_ulp_timer_stop(ELRS_HW_TIMER_INSTANCE);
}

/**
 * @brief Deinitialize the hardware timer
 *
 * Stops the timer and unregisters the callback. Call this during shutdown.
 */
void hw_timer_deinit(void)
{
    /* Stop timer if running */
    if (hw_timer.is_running) {
        sl_si91x_ulp_timer_stop(ELRS_HW_TIMER_INSTANCE);
        hw_timer.is_running = false;
    }

    /* Unregister callback */
    sl_si91x_ulp_timer_unregister_timeout_callback(ELRS_HW_TIMER_INSTANCE);

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
void hw_timer_pause(void)
{
    if (hw_timer.is_running && !hw_timer.is_paused) {
        sl_si91x_ulp_timer_stop(ELRS_HW_TIMER_INSTANCE);
        hw_timer.is_paused = true;
    }
}

/**
 * @brief Resume the timer after pause
 *
 * Resumes timing after a pause. The timer will continue from
 * where it left off in terms of TICK/TOCK state.
 */
void hw_timer_resume(void)
{
    if (hw_timer.is_running && hw_timer.is_paused) {
        /* Reset to known state - start with TOCK */
        hw_timer.is_tock = true;

        /* Reload the interval and restart */
        sl_si91x_ulp_timer_set_count(ELRS_HW_TIMER_INSTANCE,
                                      hw_timer.half_interval_us);
        sl_si91x_ulp_timer_start(ELRS_HW_TIMER_INSTANCE);
        hw_timer.is_paused = false;
    }
}

/**
 * @brief Get current timestamp in microseconds
 * @return Monotonic timestamp in microseconds
 *
 * FIX #1: Uses single total_half_ticks counter for monotonic timestamps.
 *
 * Calculation:
 *   timestamp = (total_half_ticks * half_interval_us) + elapsed_in_current_period
 *
 * Where elapsed_in_current_period is derived from the down-counter:
 *   elapsed = half_interval_us - current_count
 *
 * Citation: SiWx917 RM Section 33.6.1 - Reading MATCH register returns
 * time remaining before timeout (down-counter).
 */
uint32_t hw_timer_get_micros(void)
{
    uint32_t count = 0;
    uint32_t half_ticks;
    uint32_t elapsed;

    /*
     * FIX #4: Critical section to ensure atomic read of counter and timer
     * This prevents the ISR from incrementing total_half_ticks between
     * our read of the timer count and the counter value.
     */
    uint32_t primask = hw_timer_enter_critical();

    /* Read current timer count (time remaining) */
    sl_si91x_ulp_timer_get_count(ELRS_HW_TIMER_INSTANCE, &count);

    /* Capture the counter atomically with the timer read */
    half_ticks = hw_timer.total_half_ticks;

    hw_timer_exit_critical(primask);

    /*
     * Calculate elapsed time in current half-interval
     * Timer counts DOWN, so elapsed = interval - remaining
     *
     * Citation: SiWx917 RM Section 33.6.1 - count is time remaining
     */
    if (count <= hw_timer.half_interval_us) {
        elapsed = hw_timer.half_interval_us - count;
    } else {
        elapsed = 0;  /* Sanity check - shouldn't happen */
    }

    /* Total time = completed half-ticks + elapsed in current */
    return (half_ticks * hw_timer.half_interval_us) + elapsed;
}

/**
 * @brief Apply a phase shift to the timer
 * @param shift_us Phase shift in microseconds (positive = delay, negative = advance)
 *
 * The phase shift is applied on the next TOCK->TICK transition.
 * This effectively moves when the next TOCK (packet expected) occurs.
 *
 * FIX #3: Clamps the resulting interval to safe bounds.
 * FIX #4: Uses critical section for atomic write.
 */
void hw_timer_phase_shift(int32_t shift_us)
{
    uint32_t primask = hw_timer_enter_critical();

    /*
     * Clamp phase shift to prevent extreme values
     * FIX #3: Ensure base + shift stays within valid range
     */
    int32_t max_shift = (int32_t)(MAX_HALF_INTERVAL_US - hw_timer.half_interval_us);
    int32_t min_shift = (int32_t)(MIN_HALF_INTERVAL_US - hw_timer.half_interval_us);

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
 * FIX #2: The freq_offset is applied once per FULL interval (on TICK->TOCK).
 * This matches official ELRS PFD behavior where FreqOffset adjusts the
 * entire packet interval, not each half-interval.
 *
 * FIX #4: Uses critical section for atomic read-modify-write.
 */
void hw_timer_inc_freq_offset(int32_t delta)
{
    uint32_t primask = hw_timer_enter_critical();

    hw_timer.freq_offset_us += delta;

    /*
     * Clamp frequency offset to reasonable bounds
     * Typical ELRS freq_offset is within ±100µs
     */
    const int32_t max_freq_offset = 500;  /* ±500µs max */
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
void hw_timer_reset_freq_offset(void)
{
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
 */
void hw_timer_set_interval(uint32_t interval_us)
{
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

    hw_timer_exit_critical(primask);

    /* Apply new interval immediately if running */
    if (hw_timer.is_running && !hw_timer.is_paused) {
        hw_timer_reconfigure_interval(hw_timer.half_interval_us);
    }
}

/**
 * @brief Register the TICK callback
 * @param callback Function to call on TICK (mid-interval)
 */
void hw_timer_set_tick_callback(hw_timer_tick_callback_t callback)
{
    hw_timer.tick_callback = callback;
}

/**
 * @brief Register the TOCK callback
 * @param callback Function to call on TOCK (end of interval, packet expected)
 */
void hw_timer_set_tock_callback(hw_timer_tock_callback_t callback)
{
    hw_timer.tock_callback = callback;
}

/**
 * @brief Check if timer is currently running
 * @return true if timer is running, false otherwise
 */
bool hw_timer_is_running(void)
{
    return hw_timer.is_running && !hw_timer.is_paused;
}

/**
 * @brief Get the current frequency offset
 * @return Current frequency offset in microseconds
 */
int32_t hw_timer_get_freq_offset(void)
{
    return hw_timer.freq_offset_us;
}

/**
 * @brief Get the current timer interval
 * @return Current full interval in microseconds
 */
uint32_t hw_timer_get_interval(void)
{
    return hw_timer.interval_us;
}
