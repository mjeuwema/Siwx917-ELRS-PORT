/**
 * @file hw_timer.h
 * @brief Hardware Timer Abstraction for ELRS on SiWx917
 *
 * This module provides the hardware timer functionality required for ELRS
 * receiver operation. It implements the tick/tock callback pattern used by
 * ELRS 4.0 to maintain precise timing synchronization with the transmitter.
 *
 * Citation: ExpressLRS 4.0 src/lib/HWTIMER/hwTimer.h
 *   - hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock)
 *   - Timer fires twice per packet period (tick at mid-packet, tock when packet
 * expected)
 *
 * Citation: SiWx917 Family Reference Manual Section 33 - ULP Timers
 *   - Uses ULP Timer 1 in 1µs mode for precise timing
 *   - Timer0 reserved for SDK, Timer1 used for ELRS hwTimer
 *
 * IMPORTANT - Timer Hardware Constraints (RM Section 33.4.1):
 *   "Timer settings are LOCKED while the timer is running. To reprogram
 *   the timer, you must first set the MCUULP_TMRx_CNTRL.TMR_STOP bit."
 *   This implementation handles stop/reconfigure/start internally.
 *
 * The hwTimer module is CRITICAL for:
 *   1. Phase-locking to TX timing via PFD (Phase Frequency Detector)
 *   2. Predicting when the next packet arrives
 *   3. Synchronizing FHSS frequency hopping
 *   4. Calculating link quality from missed packets
 *
 * @copyright Copyright (c) 2024-2026
 */

#ifndef ELRS_HW_TIMER_H
#define ELRS_HW_TIMER_H

#include "sl_status.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/* Timer instance macro moved to hw_timer.c to avoid redefinition */

/**
 * Timer resolution bounds in microseconds
 * Citation: ExpressLRS common.h - intervalUs field in RF params
 * ELRS rates range from 2000µs (500Hz) to 40000µs (25Hz)
 */
#define HW_TIMER_MIN_INTERVAL_US 200    /* Minimum full interval (100µs half) */
#define HW_TIMER_MAX_INTERVAL_US 100000 /* Maximum interval (100ms) */

/*******************************************************************************
 * Callback Types
 ******************************************************************************/

/**
 * TICK callback function type - called at mid-interval
 * Citation: ExpressLRS hwTimer.h - HWtimerCallbackTick
 */
typedef void (*hw_timer_tick_callback_t)(void);

/**
 * TOCK callback function type - called when packet expected
 * Citation: ExpressLRS hwTimer.h - HWtimerCallbackTock
 */
typedef void (*hw_timer_tock_callback_t)(void);

/*******************************************************************************
 * Initialization & Control
 ******************************************************************************/

/**
 * @brief Initialize the hardware timer
 *
 * Sets up ULP Timer 1 for ELRS tick/tock operation. The timer will fire
 * twice per packet interval:
 *   - TOCK: At the expected packet arrival time
 *   - TICK: Mid-way between packets (for LQ calculation)
 *
 * Citation: SiWx917 RM Section 6.14.13.10 - ULP_TIMER_CLK_SEL
 *   Value 4 selects RC_32MHZ_CLK as timer clock source for 1µs resolution.
 *
 * @param interval_us Full packet interval in microseconds
 * @return SL_STATUS_OK on success, error code otherwise
 */
sl_status_t hw_timer_init(uint32_t interval_us);

/**
 * @brief Deinitialize the hardware timer
 *
 * Stops the timer and releases resources.
 */
void hw_timer_deinit(void);

/**
 * @brief Start the timer
 *
 * Begins timer operation. First callback will be TOCK.
 *
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_start(void);

/**
 * @brief Stop the timer
 *
 * Completely stops the timer and resets state.
 *
 * @return SL_STATUS_OK on success
 */
sl_status_t hw_timer_stop(void);

/**
 * @brief Pause the timer without resetting
 *
 * Citation: ExpressLRS hwTimer.cpp - pause()
 * Used during connection loss to save power while preserving state.
 */
void hw_timer_pause(void);

/**
 * @brief Resume a paused timer
 *
 * Citation: ExpressLRS hwTimer.cpp - resume()
 * Resumes from paused state, continuing TICK/TOCK sequence.
 */
void hw_timer_resume(void);

/**
 * @brief Pause the CT timer ISR to prevent SPI reentrancy
 *
 * This masks the CT IRQ so it becomes pending instead of firing,
 * allowing atomic operations in the main thread.
 */
void hw_timer_pause_isr(void);

/**
 * @brief Resume the CT timer ISR
 *
 * Unmasks the CT IRQ, allowing any pending timer interrupts to fire
 * immediately.
 */
void hw_timer_resume_isr(void);

/*******************************************************************************
 * Callback Registration
 ******************************************************************************/

/**
 * @brief Register the TICK callback
 *
 * Called at mid-interval, used for LQ calculation.
 *
 * @param callback Function to call on TICK
 */
void hw_timer_set_tick_callback(hw_timer_tick_callback_t callback);

/**
 * @brief Register the TOCK callback
 *
 * Called at end of interval when packet is expected.
 *
 * @param callback Function to call on TOCK
 */
void hw_timer_set_tock_callback(hw_timer_tock_callback_t callback);

/*******************************************************************************
 * Timer Adjustment (Phase Lock)
 ******************************************************************************/

/**
 * @brief Adjust timer phase for synchronization
 *
 * Called by PFD to adjust timer phase for phase-locking to TX.
 * Applied on the next TOCK->TICK transition.
 *
 * Positive values delay the next interval (slows down).
 * Negative values advance the next interval (speeds up).
 *
 * Citation: ExpressLRS hwTimer.cpp - phaseShift()
 *   "Shifts the timer phase by the given amount in microseconds"
 *
 * @param shift_us Phase adjustment in microseconds
 */
void hw_timer_phase_shift(int32_t shift_us);

/**
 * @brief Update the timer interval
 *
 * Called when air rate changes to update the packet interval.
 *
 * Citation: ExpressLRS hwTimer.cpp - updateInterval()
 *
 * @param interval_us New FULL packet interval in microseconds
 */
void hw_timer_set_interval(uint32_t interval_us);

/**
 * @brief Get the current timer interval
 *
 * @return Current full interval in microseconds
 */
uint32_t hw_timer_get_interval(void);

/**
 * @brief Increment frequency offset
 *
 * Called by PFD when timer is running slow relative to TX.
 * Adds specified delta to the frequency offset.
 *
 * NOTE: The freq_offset is applied once per FULL interval (on TICK->TOCK only)
 * to match official ELRS PFD behavior.
 *
 * Citation: ExpressLRS hwTimer.h line 68
 *   static ICACHE_RAM_ATTR void inline incFreqOffset() { FreqOffset++; }
 *
 * @param delta Offset increment in microseconds (typically 1 or -1)
 */
void hw_timer_inc_freq_offset(int32_t delta);

/**
 * @brief Increment frequency offset by 1µs (ELRS compatibility wrapper)
 *
 * Citation: ExpressLRS hwTimer.h - incFreqOffset()
 */
static inline void hw_timer_inc_freq_offset_1us(void) {
  hw_timer_inc_freq_offset(1);
}

/**
 * @brief Decrement frequency offset by 1µs (ELRS compatibility wrapper)
 *
 * Citation: ExpressLRS hwTimer.h - decFreqOffset()
 */
static inline void hw_timer_dec_freq_offset_1us(void) {
  hw_timer_inc_freq_offset(-1);
}

/**
 * @brief Reset frequency offset to zero
 *
 * Called when connection is lost to reset timer to initial phase.
 *
 * Citation: ExpressLRS hwTimer.cpp - resetFreqOffset()
 */
void hw_timer_reset_freq_offset(void);

/**
 * @brief Get current frequency offset
 *
 * Returns the accumulated frequency offset in microseconds.
 *
 * Citation: ExpressLRS hwTimer.h line 79
 *   static ICACHE_RAM_ATTR int32_t inline getFreqOffset() { return FreqOffset;
 * }
 *
 * @return Current frequency offset in microseconds
 */
int32_t hw_timer_get_freq_offset(void);

/*******************************************************************************
 * Status
 ******************************************************************************/

/**
 * @brief Check if timer is currently running
 *
 * @return true if timer is running (not stopped or paused)
 */
bool hw_timer_is_running(void);

/**
 * @brief Get microsecond timestamp
 *
 * Returns a monotonically increasing microsecond counter.
 * Uses the timer's internal counter for high precision.
 *
 * Implementation uses a single total_half_ticks counter plus
 * the current timer count for sub-interval precision.
 *
 * Citation: SiWx917 RM Section 33.6.1 - Reading count register
 * returns time remaining (down-counter).
 *
 * @return Current time in microseconds (monotonic)
 */
uint32_t hw_timer_get_micros(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_HW_TIMER_H */
