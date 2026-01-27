/**
 * @file lpf.c
 * @brief Low Pass Filter Implementation
 * 
 * Simple exponential moving average filter for RSSI/SNR smoothing.
 * 
 * Citation: ExpressLRS 4.0 src/lib/LPF/LPF.cpp
 *   - EMA filter using bit-shift for efficiency
 *   - Seeding on first sample
 */

#include "lpf.h"
#include <stddef.h>  /* For NULL */

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void lpf_init(lpf_t *lpf, uint8_t beta)
{
    if (lpf == NULL) {
        return;
    }
    
    lpf->value = 0;
    lpf->raw_value = 0;
    lpf->beta = (beta >= 1 && beta <= 8) ? beta : LPF_BETA_RSSI;
    lpf->initialized = false;
}

void lpf_reset(lpf_t *lpf)
{
    if (lpf == NULL) {
        return;
    }
    
    lpf->value = 0;
    lpf->raw_value = 0;
    lpf->initialized = false;
}

/*******************************************************************************
 * Filter Operations
 ******************************************************************************/

int32_t lpf_update(lpf_t *lpf, int32_t value)
{
    if (lpf == NULL) {
        return 0;
    }
    
    /* Store raw value */
    lpf->raw_value = value;
    
    /* Seed filter on first sample */
    if (!lpf->initialized) {
        lpf->value = value;
        lpf->initialized = true;
        return value;
    }
    
    /* Apply exponential moving average
     * Citation: ExpressLRS LPF.cpp - update()
     *   result = ((result << beta) - result + sample) >> beta
     * 
     * This is mathematically equivalent to:
     *   result = (result * (2^beta - 1) + sample) / 2^beta
     * 
     * FIXED: Previous implementation used a different formula:
     *   scaled = (current << beta) + (sample - current)  <-- WRONG
     * 
     * ELRS formula:
     *   scaled = (current << beta) - current + sample
     *          = current * (2^beta - 1) + sample
     * 
     * The difference is subtle but affects PFD lock timing:
     *   - ELRS: new = old * (2^beta-1)/(2^beta) + sample/(2^beta)
     *   - Wrong: new = old + (sample-old)/(2^beta)  <-- Different coefficient!
     */
    int32_t result = ((lpf->value << lpf->beta) - lpf->value + value) >> lpf->beta;
    lpf->value = result;
    
    return lpf->value;
}

void lpf_seed(lpf_t *lpf, int32_t value)
{
    if (lpf == NULL) {
        return;
    }
    
    lpf->value = value;
    lpf->raw_value = value;
    lpf->initialized = true;
}

/*******************************************************************************
 * Getters
 ******************************************************************************/

int32_t lpf_get_value(const lpf_t *lpf)
{
    return (lpf != NULL) ? lpf->value : 0;
}

int32_t lpf_get_raw(const lpf_t *lpf)
{
    return (lpf != NULL) ? lpf->raw_value : 0;
}

bool lpf_is_initialized(const lpf_t *lpf)
{
    return (lpf != NULL) ? lpf->initialized : false;
}

/*******************************************************************************
 * Configuration
 ******************************************************************************/

void lpf_set_beta(lpf_t *lpf, uint8_t beta)
{
    if (lpf != NULL && beta >= 1 && beta <= 8) {
        lpf->beta = beta;
    }
}

uint8_t lpf_get_beta(const lpf_t *lpf)
{
    return (lpf != NULL) ? lpf->beta : 0;
}
