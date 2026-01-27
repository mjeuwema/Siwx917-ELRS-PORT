/**
 * @file lpf.h
 * @brief Low Pass Filter for RSSI/SNR Smoothing
 * 
 * This module provides a simple exponential moving average (EMA)
 * low-pass filter for smoothing RSSI and SNR values. This reduces
 * noise and provides stable readings for link statistics.
 * 
 * Citation: ExpressLRS 4.0 src/lib/LPF/LPF.h
 *   - Uses exponential moving average filter
 *   - Configurable smoothing coefficient
 *   - Used for RSSI filtering in link stats
 * 
 * Filter formula:
 *   output = (1 - alpha) * output + alpha * input
 * 
 * Where alpha = 1 / 2^beta (beta is the smoothing factor)
 */

#ifndef ELRS_LPF_H
#define ELRS_LPF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/**
 * Default smoothing factors
 * Citation: ExpressLRS LPF.h
 *   Higher beta = more smoothing (slower response)
 *   Lower beta = less smoothing (faster response)
 */
#define LPF_BETA_RSSI       4   /* 2^4 = 16 sample EMA for RSSI */
#define LPF_BETA_SNR        3   /* 2^3 = 8 sample EMA for SNR */
#define LPF_BETA_FAST       2   /* 2^2 = 4 sample EMA for fast response */

/*******************************************************************************
 * LPF Instance Structure
 ******************************************************************************/

/**
 * @brief Low pass filter instance
 * 
 * Each filter instance maintains its own state and configuration.
 * Use separate instances for RSSI, SNR, etc.
 */
typedef struct {
    int32_t     value;          /**< Current filtered value (fixed point) */
    int32_t     raw_value;      /**< Last raw input value */
    uint8_t     beta;           /**< Smoothing factor (2^beta samples) */
    bool        initialized;    /**< True if filter has been seeded */
} lpf_t;

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize a low pass filter
 * 
 * Creates a new LPF instance with the specified smoothing factor.
 * 
 * @param lpf Pointer to LPF instance to initialize
 * @param beta Smoothing factor (1-8), higher = more smoothing
 */
void lpf_init(lpf_t *lpf, uint8_t beta);

/**
 * @brief Reset the filter state
 * 
 * Clears the filter and sets initialized to false.
 * Next sample will seed the filter.
 * 
 * @param lpf Pointer to LPF instance
 */
void lpf_reset(lpf_t *lpf);

/*******************************************************************************
 * Filter Operations
 ******************************************************************************/

/**
 * @brief Add a sample to the filter
 * 
 * Updates the filtered value with a new sample.
 * If this is the first sample, it seeds the filter.
 * 
 * Citation: ExpressLRS LPF.cpp - update()
 *   filtered = ((filtered << beta) - filtered + value) >> beta
 * 
 * @param lpf Pointer to LPF instance
 * @param value New sample value
 * @return Filtered value
 */
int32_t lpf_update(lpf_t *lpf, int32_t value);

/**
 * @brief Seed the filter with an initial value
 * 
 * Sets the filter output directly without smoothing.
 * Use this to initialize the filter to a known value.
 * 
 * @param lpf Pointer to LPF instance
 * @param value Initial value to seed
 */
void lpf_seed(lpf_t *lpf, int32_t value);

/*******************************************************************************
 * Getters
 ******************************************************************************/

/**
 * @brief Get the current filtered value
 * 
 * @param lpf Pointer to LPF instance
 * @return Current filtered value
 */
int32_t lpf_get_value(const lpf_t *lpf);

/**
 * @brief Get the last raw input value
 * 
 * @param lpf Pointer to LPF instance
 * @return Last raw sample value
 */
int32_t lpf_get_raw(const lpf_t *lpf);

/**
 * @brief Check if filter is initialized
 * 
 * @param lpf Pointer to LPF instance
 * @return true if filter has received at least one sample
 */
bool lpf_is_initialized(const lpf_t *lpf);

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Set the smoothing factor
 * 
 * @param lpf Pointer to LPF instance
 * @param beta New smoothing factor (1-8)
 */
void lpf_set_beta(lpf_t *lpf, uint8_t beta);

/**
 * @brief Get the smoothing factor
 * 
 * @param lpf Pointer to LPF instance
 * @return Current beta value
 */
uint8_t lpf_get_beta(const lpf_t *lpf);

/*******************************************************************************
 * Convenience Macros
 ******************************************************************************/

/**
 * @brief Create and initialize a static LPF instance
 * 
 * Usage: LPF_DEFINE(rssi_filter, LPF_BETA_RSSI);
 */
#define LPF_DEFINE(name, beta_val) \
    lpf_t name = { .value = 0, .raw_value = 0, .beta = (beta_val), .initialized = false }

#ifdef __cplusplus
}
#endif

#endif /* ELRS_LPF_H */
