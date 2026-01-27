/**
 * @file pfd.h
 * @brief Phase Frequency Detector for ELRS Timing Synchronization
 * 
 * The PFD (Phase Frequency Detector) is responsible for maintaining
 * timing synchronization between the receiver and transmitter.
 * It measures the phase difference between expected and actual
 * packet arrival times and generates corrections for the hardware timer.
 * 
 * Citation: ExpressLRS 4.0 src/lib/FHSS/FHSS.cpp
 *   - PFD algorithm calculates phase offset from packet arrival
 *   - Uses exponential moving average for smooth corrections
 * 
 * Citation: ExpressLRS 4.0 src/lib/PFD/PFD.h
 *   - PFDClass manages phase/frequency tracking
 *   - calcResult() returns microsecond adjustment for hwTimer
 * 
 * How it works:
 *   1. When a packet arrives, record the offset from expected time
 *   2. Apply low-pass filtering to smooth out jitter
 *   3. Calculate a correction value for hwTimer::phaseShift()
 *   4. Gradually phase-lock the RX to the TX timing
 */

#ifndef ELRS_PFD_H
#define ELRS_PFD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/**
 * PFD Filter coefficients
 * Citation: ExpressLRS PFD.h
 *   - PFD_ALPHA controls how quickly to respond to phase errors
 *   - Higher values = faster response but more jitter
 *   - Lower values = slower response but smoother tracking
 */
#define PFD_ALPHA_DEFAULT           4       /* 2^4 = 16 samples EMA */
#define PFD_ALPHA_FAST              2       /* 2^2 = 4 samples for fast lock */

/**
 * Maximum phase correction per iteration
 * Citation: ExpressLRS PFD.cpp
 *   - Limits how much the timer can be adjusted in one step
 *   - Prevents oscillation and overcorrection
 */
#define PFD_MAX_CORRECTION_US       100     /* Maximum ±100µs per correction */

/**
 * Phase lock thresholds
 * Citation: ExpressLRS rx_main.cpp
 *   - When offset < threshold, consider locked
 */
#define PFD_LOCKED_THRESHOLD_US     10      /* Consider locked if <10µs offset */
#define PFD_UNLOCKED_THRESHOLD_US   50      /* Consider unlocked if >50µs offset */

/*******************************************************************************
 * PFD State
 ******************************************************************************/

/**
 * @brief PFD lock state
 */
typedef enum {
    PFD_UNLOCKED = 0,       /**< Not phase locked */
    PFD_LOCKING,            /**< In process of acquiring lock */
    PFD_LOCKED              /**< Phase locked to TX */
} pfd_lock_state_t;

/**
 * @brief PFD internal state structure
 */
typedef struct {
    int32_t     raw_offset;         /**< Raw phase offset from last packet (µs) */
    int32_t     filtered_offset;    /**< Filtered (averaged) offset (µs) */
    int32_t     freq_offset;        /**< Frequency drift correction (µs/packet) */
    uint8_t     alpha;              /**< Filter coefficient (2^alpha samples) */
    pfd_lock_state_t lock_state;    /**< Current lock state */
    uint32_t    locked_count;       /**< Consecutive locked samples */
    uint32_t    interval_us;        /**< Expected packet interval */
    int32_t     result;             /**< Correction value for hwTimer */
} pfd_state_t;

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize the PFD
 * 
 * Resets all state and prepares for phase tracking.
 * 
 * @param interval_us Expected packet interval in microseconds
 */
void pfd_init(uint32_t interval_us);

/**
 * @brief Reset PFD state
 * 
 * Called when connection is lost or rate changes.
 * Clears all accumulated offsets and resets to UNLOCKED state.
 */
void pfd_reset(void);

/*******************************************************************************
 * Phase Measurement
 ******************************************************************************/

/**
 * @brief Record a phase sample when packet arrives
 * 
 * Call this when a valid packet is received with the microsecond
 * offset from the expected arrival time.
 * 
 * Citation: ExpressLRS PFD.cpp - input()
 *   "Records the offset from expected arrival time"
 * 
 * @param offset_us Signed offset in microseconds
 *                  Positive = packet arrived late
 *                  Negative = packet arrived early
 */
void pfd_sample(int32_t offset_us);

/**
 * @brief Record external event (packet arrival) with slack compensation
 * 
 * This is the proper ELRS method for recording packet arrival timing.
 * It calculates the phase offset accounting for TOA and slack.
 * 
 * Citation: ExpressLRS PFD.cpp - extEvent()
 *   int32_t offset = expectedTock - now;
 *   input(offset);
 * 
 * @param packet_time_us  Microsecond timestamp of packet arrival + slack
 */
void pfd_ext_event(uint32_t packet_time_us);

/**
 * @brief Set the reference TOCK time for phase calculation
 * 
 * Called from hw_timer_tock() to record when TOCK fires.
 * This is the reference point for phase offset calculation.
 * 
 * @param tock_time_us  Microsecond timestamp of TOCK
 */
void pfd_set_tock_time(uint32_t tock_time_us);

/**
 * @brief Calculate the correction value
 * 
 * Returns the calculated phase/frequency correction to apply
 * to the hardware timer. Call this after pfd_sample().
 * 
 * Citation: ExpressLRS PFD.cpp - calcResult()
 *   "Returns the correction to apply to hwTimer"
 * 
 * @return Correction in microseconds for hw_timer_phase_shift()
 */
int32_t pfd_calc_result(void);

/*******************************************************************************
 * Interval Management
 ******************************************************************************/

/**
 * @brief Update the expected packet interval
 * 
 * Called when air rate changes.
 * 
 * @param interval_us New packet interval in microseconds
 */
void pfd_set_interval(uint32_t interval_us);

/**
 * @brief Get the current packet interval
 * 
 * @return Current interval in microseconds
 */
uint32_t pfd_get_interval(void);

/*******************************************************************************
 * Filter Control
 ******************************************************************************/

/**
 * @brief Set the filter alpha value
 * 
 * Controls how quickly the PFD responds to phase errors.
 * 
 * Citation: ExpressLRS PFD.h
 *   alpha = 2 means 2^2 = 4 sample moving average (fast)
 *   alpha = 4 means 2^4 = 16 sample moving average (smooth)
 * 
 * @param alpha Filter exponent (1-8)
 */
void pfd_set_alpha(uint8_t alpha);

/**
 * @brief Enable fast lock mode
 * 
 * Temporarily use faster alpha for quick acquisition.
 * Call pfd_normal_mode() to return to normal tracking.
 */
void pfd_fast_mode(void);

/**
 * @brief Return to normal tracking mode
 */
void pfd_normal_mode(void);

/*******************************************************************************
 * Status
 ******************************************************************************/

/**
 * @brief Get the current lock state
 * 
 * @return Current PFD lock state
 */
pfd_lock_state_t pfd_get_lock_state(void);

/**
 * @brief Check if PFD is locked
 * 
 * @return true if phase locked to TX
 */
bool pfd_is_locked(void);

/**
 * @brief Get the raw phase offset
 * 
 * @return Last measured offset in microseconds
 */
int32_t pfd_get_raw_offset(void);

/**
 * @brief Get the filtered phase offset
 * 
 * @return Filtered offset in microseconds
 */
int32_t pfd_get_filtered_offset(void);

/**
 * @brief Get the frequency offset
 * 
 * @return Frequency drift in microseconds per packet
 */
int32_t pfd_get_freq_offset(void);

/**
 * @brief Get pointer to PFD state (for debugging)
 * 
 * @return Pointer to internal state structure
 */
const pfd_state_t* pfd_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_PFD_H */
