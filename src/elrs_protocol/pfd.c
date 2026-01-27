/**
 * @file pfd.c
 * @brief Phase Frequency Detector Implementation
 * 
 * Implements phase tracking and correction calculation for ELRS
 * timing synchronization. Uses exponential moving average filtering
 * to smooth phase measurements and generate hwTimer corrections.
 * 
 * Citation: ExpressLRS 4.0 src/lib/PFD/PFD.cpp
 *   - Phase detection from packet arrival offset
 *   - EMA filtering for smooth corrections
 *   - Frequency offset tracking
 */

#include "pfd.h"
#include "rsi_debug.h"

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define PFD_DEBUG   0

#if PFD_DEBUG
    #define PFD_DBG(fmt, ...)    DEBUGOUT("[PFD] " fmt, ##__VA_ARGS__)
#else
    #define PFD_DBG(fmt, ...)    ((void)0)
#endif

/*******************************************************************************
 * Module State
 ******************************************************************************/

static pfd_state_t pfd = {0};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Clamp value to range
 */
static inline int32_t clamp(int32_t value, int32_t min_val, int32_t max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Exponential moving average update
 * 
 * Citation: ExpressLRS PFD.cpp / LPF.cpp
 *   result = ((result << alpha) - result + sample) >> alpha
 * 
 * Mathematical form:
 *   new = (old * (2^alpha - 1) + sample) / 2^alpha
 *       = old * (1 - 1/2^alpha) + sample * (1/2^alpha)
 * 
 * This is a standard EMA with alpha factor = 1/2^alpha
 * 
 * FIXED: Previous implementation was WRONG - used different formula:
 *   (current << alpha) + (sample - current) >> alpha  <-- WRONG
 * 
 * The correct ELRS formula:
 *   ((current << alpha) - current + sample) >> alpha
 */
static int32_t ema_update(int32_t current, int32_t sample, uint8_t alpha)
{
    /* ExpressLRS exact implementation */
    return ((current << alpha) - current + sample) >> alpha;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void pfd_init(uint32_t interval_us)
{
    PFD_DBG("Initializing PFD, interval=%lu µs\n", (unsigned long)interval_us);
    
    pfd.raw_offset = 0;
    pfd.filtered_offset = 0;
    pfd.freq_offset = 0;
    pfd.alpha = PFD_ALPHA_DEFAULT;
    pfd.lock_state = PFD_UNLOCKED;
    pfd.locked_count = 0;
    pfd.interval_us = interval_us;
    pfd.result = 0;
}

void pfd_reset(void)
{
    PFD_DBG("Resetting PFD\n");
    
    pfd.raw_offset = 0;
    pfd.filtered_offset = 0;
    pfd.freq_offset = 0;
    pfd.lock_state = PFD_UNLOCKED;
    pfd.locked_count = 0;
    pfd.result = 0;
}

/*******************************************************************************
 * Phase Measurement
 ******************************************************************************/

/* Last recorded TOCK time for offset calculation */
static uint32_t last_tock_time_us = 0;

void pfd_ext_event(uint32_t packet_time_us)
{
    /* Calculate phase offset from expected TOCK time
     * 
     * Citation: ExpressLRS PFD.cpp - extEvent()
     *   The offset is the difference between when we expected the packet
     *   (TOCK time) and when it actually arrived (packet_time_us which
     *   includes the processing slack).
     * 
     * Positive offset = packet arrived LATE (TOCK was in the past)
     * Negative offset = packet arrived EARLY (TOCK is in the future)
     */
    int32_t offset = (int32_t)(last_tock_time_us - packet_time_us);
    
    PFD_DBG("extEvent: packet=%lu, tock=%lu, offset=%ld\n",
            (unsigned long)packet_time_us, (unsigned long)last_tock_time_us, (long)offset);
    
    pfd_sample(offset);
}

/**
 * @brief Set the reference TOCK time for phase calculation
 * 
 * Called by hw_timer_tock to record the expected packet arrival time.
 */
void pfd_set_tock_time(uint32_t tock_time_us)
{
    last_tock_time_us = tock_time_us;
}

void pfd_sample(int32_t offset_us)
{
    int32_t abs_offset;
    
    /* Store raw offset */
    pfd.raw_offset = offset_us;
    
    PFD_DBG("Sample: offset=%ld µs\n", (long)offset_us);
    
    /* Apply exponential moving average filter
     * Citation: ExpressLRS PFD.cpp - input()
     */
    pfd.filtered_offset = ema_update(pfd.filtered_offset, offset_us, pfd.alpha);
    
    /* Update frequency offset estimate (how much timing drifts per packet)
     * This helps predict and correct for clock drift
     * 
     * Citation: ExpressLRS PFD.cpp
     *   freq_offset is filtered more aggressively (slower response)
     */
    int32_t new_freq_offset = offset_us - pfd.filtered_offset;
    pfd.freq_offset = ema_update(pfd.freq_offset, new_freq_offset, pfd.alpha + 2);
    
    /* Update lock state based on offset magnitude */
    abs_offset = (offset_us < 0) ? -offset_us : offset_us;
    
    switch (pfd.lock_state) {
        case PFD_UNLOCKED:
            if (abs_offset < PFD_UNLOCKED_THRESHOLD_US) {
                pfd.lock_state = PFD_LOCKING;
                pfd.locked_count = 1;
                PFD_DBG("State: UNLOCKED -> LOCKING\n");
            }
            break;
            
        case PFD_LOCKING:
            if (abs_offset < PFD_LOCKED_THRESHOLD_US) {
                pfd.locked_count++;
                if (pfd.locked_count > 10) {
                    pfd.lock_state = PFD_LOCKED;
                    PFD_DBG("State: LOCKING -> LOCKED\n");
                }
            } else if (abs_offset > PFD_UNLOCKED_THRESHOLD_US) {
                pfd.lock_state = PFD_UNLOCKED;
                pfd.locked_count = 0;
                PFD_DBG("State: LOCKING -> UNLOCKED\n");
            }
            break;
            
        case PFD_LOCKED:
            if (abs_offset > PFD_UNLOCKED_THRESHOLD_US) {
                pfd.lock_state = PFD_LOCKING;
                pfd.locked_count = 5;  /* Give some hysteresis */
                PFD_DBG("State: LOCKED -> LOCKING\n");
            }
            break;
    }
}

int32_t pfd_calc_result(void)
{
    int32_t correction;
    
    /* Calculate correction value
     * Citation: ExpressLRS PFD.cpp - calcResult()
     * 
     * The correction combines:
     *   1. Phase correction (move towards zero offset)
     *   2. Frequency correction (compensate for drift rate)
     */
    
    /* Phase correction - move towards zero offset */
    correction = -pfd.filtered_offset;
    
    /* Add frequency offset prediction */
    correction -= pfd.freq_offset;
    
    /* Clamp correction to prevent oscillation
     * Citation: ExpressLRS PFD.cpp
     *   Large corrections are limited to prevent overcorrection
     */
    correction = clamp(correction, -PFD_MAX_CORRECTION_US, PFD_MAX_CORRECTION_US);
    
    /* When locked, use smaller corrections for stability */
    if (pfd.lock_state == PFD_LOCKED) {
        correction = correction / 2;
    }
    
    pfd.result = correction;
    
    PFD_DBG("Result: correction=%ld µs (filtered=%ld, freq=%ld)\n",
            (long)correction, (long)pfd.filtered_offset, (long)pfd.freq_offset);
    
    return correction;
}

/*******************************************************************************
 * Interval Management
 ******************************************************************************/

void pfd_set_interval(uint32_t interval_us)
{
    PFD_DBG("Setting interval to %lu µs\n", (unsigned long)interval_us);
    pfd.interval_us = interval_us;
    
    /* Reset state on interval change */
    pfd_reset();
}

uint32_t pfd_get_interval(void)
{
    return pfd.interval_us;
}

/*******************************************************************************
 * Filter Control
 ******************************************************************************/

void pfd_set_alpha(uint8_t alpha)
{
    if (alpha >= 1 && alpha <= 8) {
        pfd.alpha = alpha;
        PFD_DBG("Alpha set to %d\n", alpha);
    }
}

void pfd_fast_mode(void)
{
    PFD_DBG("Entering fast mode\n");
    pfd.alpha = PFD_ALPHA_FAST;
}

void pfd_normal_mode(void)
{
    PFD_DBG("Entering normal mode\n");
    pfd.alpha = PFD_ALPHA_DEFAULT;
}

/*******************************************************************************
 * Status
 ******************************************************************************/

pfd_lock_state_t pfd_get_lock_state(void)
{
    return pfd.lock_state;
}

bool pfd_is_locked(void)
{
    return (pfd.lock_state == PFD_LOCKED);
}

int32_t pfd_get_raw_offset(void)
{
    return pfd.raw_offset;
}

int32_t pfd_get_filtered_offset(void)
{
    return pfd.filtered_offset;
}

int32_t pfd_get_freq_offset(void)
{
    return pfd.freq_offset;
}

const pfd_state_t* pfd_get_state(void)
{
    return &pfd;
}
