/**
 * @file lq_calc.c
 * @brief Link Quality Calculator Implementation
 * 
 * Implements packet tracking using a sliding window bit array.
 * Calculates LQ percentage for CRSF link statistics.
 * 
 * Citation: ExpressLRS 4.0 src/lib/LQ/LQ.cpp
 *   - Bit array tracks received packets efficiently
 *   - Window slides with each new sample
 *   - LQ = (count / window_size) * 100
 */

#include "lq_calc.h"
#include <string.h>

/*******************************************************************************
 * Module State
 ******************************************************************************/

static lq_calc_state_t lq_state = {0};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Set a bit in the bit array
 */
static inline void set_bit(uint8_t *array, uint8_t index, bool value)
{
    uint8_t byte_index = index / LQ_BITS_PER_BYTE;
    uint8_t bit_offset = index % LQ_BITS_PER_BYTE;
    
    if (value) {
        array[byte_index] |= (1U << bit_offset);
    } else {
        array[byte_index] &= ~(1U << bit_offset);
    }
}

/**
 * @brief Get a bit from the bit array
 */
static inline bool get_bit(const uint8_t *array, uint8_t index)
{
    uint8_t byte_index = index / LQ_BITS_PER_BYTE;
    uint8_t bit_offset = index % LQ_BITS_PER_BYTE;
    
    return (array[byte_index] & (1U << bit_offset)) != 0;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void lq_calc_init(void)
{
    memset(&lq_state, 0, sizeof(lq_state));
}

void lq_calc_reset(void)
{
    /* Clear the bit array but preserve total counts */
    uint32_t total_rx = lq_state.total_received;
    uint32_t total_exp = lq_state.total_expected;
    
    memset(lq_state.bit_array, 0, sizeof(lq_state.bit_array));
    lq_state.index = 0;
    lq_state.count = 0;
    lq_state.lq_value = 0;
    
    lq_state.total_received = total_rx;
    lq_state.total_expected = total_exp;
}

/*******************************************************************************
 * Packet Tracking
 ******************************************************************************/

void lq_calc_add(bool received)
{
    /* Get the old value at current index (being replaced) */
    bool old_value = get_bit(lq_state.bit_array, lq_state.index);
    
    /* Update count based on what we're replacing
     * Citation: ExpressLRS LQ.cpp - add()
     *   Subtract old value, add new value to count
     */
    if (old_value && !received) {
        /* Was 1, now 0 - decrement count */
        if (lq_state.count > 0) {
            lq_state.count--;
        }
    } else if (!old_value && received) {
        /* Was 0, now 1 - increment count */
        if (lq_state.count < LQ_CALC_WINDOW_SIZE) {
            lq_state.count++;
        }
    }
    
    /* Store new value */
    set_bit(lq_state.bit_array, lq_state.index, received);
    
    /* Advance index (circular buffer) */
    lq_state.index++;
    if (lq_state.index >= LQ_CALC_WINDOW_SIZE) {
        lq_state.index = 0;
    }
    
    /* Update LQ percentage
     * Citation: ExpressLRS LQ.cpp - getLQ()
     */
    lq_state.lq_value = (lq_state.count * 100) / LQ_CALC_WINDOW_SIZE;
    
    /* Update totals */
    if (received) {
        lq_state.total_received++;
    }
}

void lq_calc_packet_received(void)
{
    lq_calc_add(true);
}

void lq_calc_packet_missed(void)
{
    lq_calc_add(false);
}

void lq_calc_inc_expected(void)
{
    lq_state.total_expected++;
}

/*******************************************************************************
 * LQ Value
 ******************************************************************************/

uint8_t lq_calc_get_lq(void)
{
    return lq_state.lq_value;
}

uint8_t lq_calc_get_count(void)
{
    return lq_state.count;
}

uint8_t lq_calc_get_lq_raw(void)
{
    /* Citation: ExpressLRS LQCALC.h line 59
     * Returns raw packet count, not percentage
     */
    return lq_state.count;
}

bool lq_calc_current_is_set(void)
{
    /* Citation: ExpressLRS LQCALC.h line 96
     * Check if current slot already has a packet recorded
     */
    return get_bit(lq_state.bit_array, lq_state.index);
}

void lq_calc_add_current(void)
{
    /* Citation: ExpressLRS LQCALC.h lines 15-21 - add()
     * Set the bit for current period to true and update running LQ
     * Do nothing if already set (prevents double-counting)
     */
    if (lq_calc_current_is_set()) {
        return;
    }
    
    set_bit(lq_state.bit_array, lq_state.index, true);
    if (lq_state.count < LQ_CALC_WINDOW_SIZE) {
        lq_state.count++;
    }
    lq_state.total_received++;
    
    /* Update LQ percentage */
    lq_state.lq_value = (lq_state.count * 100) / LQ_CALC_WINDOW_SIZE;
}

void lq_calc_inc(void)
{
    /* Citation: ExpressLRS LQCALC.h lines 24-50 - inc()
     * Advance to next slot in the sliding window
     */
    
    /* Move to next slot */
    lq_state.index++;
    if (lq_state.index >= LQ_CALC_WINDOW_SIZE) {
        lq_state.index = 0;
    }
    
    /* If the slot we're moving into was set, we're about to lose that count */
    if (get_bit(lq_state.bit_array, lq_state.index)) {
        set_bit(lq_state.bit_array, lq_state.index, false);
        if (lq_state.count > 0) {
            lq_state.count--;
        }
    }
    
    /* Update LQ percentage */
    lq_state.lq_value = (lq_state.count * 100) / LQ_CALC_WINDOW_SIZE;
    
    lq_state.total_expected++;
}

bool lq_calc_lq_above(uint8_t threshold)
{
    return (lq_state.lq_value >= threshold);
}

/*******************************************************************************
 * Statistics
 ******************************************************************************/

uint32_t lq_calc_get_total_received(void)
{
    return lq_state.total_received;
}

uint32_t lq_calc_get_total_expected(void)
{
    return lq_state.total_expected;
}

uint8_t lq_calc_get_loss_rate(void)
{
    if (lq_state.total_expected == 0) {
        return 0;
    }
    
    uint32_t lost = lq_state.total_expected - lq_state.total_received;
    return (uint8_t)((lost * 100) / lq_state.total_expected);
}

uint8_t lq_calc_get_index(void)
{
    return lq_state.index;
}

const lq_calc_state_t* lq_calc_get_state(void)
{
    return &lq_state;
}
