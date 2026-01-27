/**
 * @file lq_calc.h
 * @brief Link Quality Calculator for ELRS
 * 
 * This module provides accurate link quality calculation based on
 * tracking received packets within a sliding window. It's used to
 * calculate the LQ percentage sent via CRSF telemetry.
 * 
 * Citation: ExpressLRS 4.0 src/lib/LQ/LQ.h
 *   - LQCalc class tracks packets in a sliding window
 *   - Uses bit array for efficient memory usage
 *   - LQ = (packets_received / window_size) * 100
 * 
 * Citation: CRSF Protocol Specification
 *   - LQ is 0-100% value in link statistics frame
 *   - Higher LQ = better link quality
 */

#ifndef ELRS_LQ_CALC_H
#define ELRS_LQ_CALC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/**
 * LQ calculation window size
 * Citation: ExpressLRS LQ.h - default window is 100 packets
 */
#define LQ_CALC_WINDOW_SIZE     100

/**
 * Number of bits per byte for the bit array
 */
#define LQ_BITS_PER_BYTE        8

/**
 * Size of the bit array in bytes
 */
#define LQ_ARRAY_SIZE           ((LQ_CALC_WINDOW_SIZE + LQ_BITS_PER_BYTE - 1) / LQ_BITS_PER_BYTE)

/*******************************************************************************
 * LQ Calculator State
 ******************************************************************************/

/**
 * @brief LQ calculator state structure
 */
typedef struct {
    uint8_t     bit_array[LQ_ARRAY_SIZE];   /**< Bit array tracking packet reception */
    uint8_t     index;                       /**< Current position in window */
    uint8_t     count;                       /**< Number of packets received in window */
    uint8_t     lq_value;                    /**< Calculated LQ (0-100) */
    uint32_t    total_received;              /**< Total packets received since init */
    uint32_t    total_expected;              /**< Total packets expected since init */
} lq_calc_state_t;

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize the LQ calculator
 * 
 * Resets all tracking state. Call at startup and when reconnecting.
 */
void lq_calc_init(void);

/**
 * @brief Reset the LQ calculator
 * 
 * Clears the sliding window but preserves total counts.
 * Called when connection is lost.
 */
void lq_calc_reset(void);

/*******************************************************************************
 * Packet Tracking
 ******************************************************************************/

/**
 * @brief Record a packet slot (received or missed)
 * 
 * Call this once per expected packet interval (from hwTimer tock).
 * 
 * Citation: ExpressLRS LQ.cpp - add()
 *   "Records whether a packet was received in this slot"
 * 
 * @param received true if packet was received, false if missed
 */
void lq_calc_add(bool received);

/**
 * @brief Record a received packet
 * 
 * Convenience function equivalent to lq_calc_add(true).
 */
void lq_calc_packet_received(void);

/**
 * @brief Record a missed packet
 * 
 * Convenience function equivalent to lq_calc_add(false).
 */
void lq_calc_packet_missed(void);

/**
 * @brief Increment the expected packet counter
 * 
 * Called each packet interval regardless of reception.
 * Used for overall statistics.
 */
void lq_calc_inc_expected(void);

/*******************************************************************************
 * LQ Value
 ******************************************************************************/

/**
 * @brief Get the current LQ value
 * 
 * Returns the link quality as a percentage (0-100).
 * 
 * Citation: ExpressLRS LQ.cpp - getLQ()
 *   Returns: (count * 100) / window_size
 * 
 * @return LQ percentage (0-100)
 */
uint8_t lq_calc_get_lq(void);

/**
 * @brief Get the raw count of received packets in window
 * 
 * Citation: ExpressLRS LQCALC.h - getLQRaw()
 *   Returns the raw count without percentage conversion.
 *   Used for minLqForChaos() check in GotConnection().
 * 
 * @return Number of packets received in the current window (0-100)
 */
uint8_t lq_calc_get_count(void);

/**
 * @brief Get raw LQ count (alias for get_count for ELRS compatibility)
 * 
 * Citation: ExpressLRS LQCALC.h line 59
 *   uint8_t getLQRaw() const { return LQ; }
 * 
 * @return Raw packet count in window (not percentage)
 */
uint8_t lq_calc_get_lq_raw(void);

/**
 * @brief Check if current slot has been marked as received
 * 
 * Citation: ExpressLRS LQCALC.h line 96
 *   bool ICACHE_RAM_ATTR currentIsSet() const {
 *       return LQArray[index] & LQmask;
 *   }
 * 
 * Used to prevent duplicate packet processing - if currentIsSet()
 * is already true when a packet arrives, it's a duplicate.
 * 
 * @return true if current period already has a packet recorded
 */
bool lq_calc_current_is_set(void);

/**
 * @brief Mark current slot as received without advancing
 * 
 * Citation: ExpressLRS LQCALC.h line 15-21 - add()
 *   Sets the bit for current period and updates running LQ.
 *   Does nothing if currentIsSet() is already true.
 * 
 * This is different from lq_calc_packet_received() which also
 * advances the window. Use this from ISR to mark packet arrival.
 */
void lq_calc_add_current(void);

/**
 * @brief Advance to next slot (called from TICK)
 * 
 * Citation: ExpressLRS LQCALC.h line 24-50 - inc()
 *   Advances the window position, wrapping at N.
 *   If the slot being advanced into was set, decrements LQ.
 * 
 * This handles the window sliding - called once per packet period.
 */
void lq_calc_inc(void);

/**
 * @brief Check if LQ is above threshold
 * 
 * @param threshold LQ threshold to check (0-100)
 * @return true if current LQ >= threshold
 */
bool lq_calc_lq_above(uint8_t threshold);

/*******************************************************************************
 * Statistics
 ******************************************************************************/

/**
 * @brief Get total packets received since init
 * 
 * @return Total received count
 */
uint32_t lq_calc_get_total_received(void);

/**
 * @brief Get total packets expected since init
 * 
 * @return Total expected count
 */
uint32_t lq_calc_get_total_expected(void);

/**
 * @brief Get overall packet loss rate
 * 
 * Calculated as: (1 - received/expected) * 100
 * 
 * @return Loss rate percentage (0-100)
 */
uint8_t lq_calc_get_loss_rate(void);

/**
 * @brief Get the current window index
 * 
 * For debugging purposes.
 * 
 * @return Current index in the sliding window
 */
uint8_t lq_calc_get_index(void);

/**
 * @brief Get pointer to state structure
 * 
 * For debugging purposes.
 * 
 * @return Pointer to internal state
 */
const lq_calc_state_t* lq_calc_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_LQ_CALC_H */
