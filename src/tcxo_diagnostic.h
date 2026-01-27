/**
 * @file tcxo_diagnostic.h
 * @brief Deep TCXO Diagnostic for Core1121-HF Module
 */

#ifndef TCXO_DIAGNOSTIC_H
#define TCXO_DIAGNOSTIC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Deep TCXO diagnostic - tests multiple configurations
 * 
 * This comprehensive test tries:
 * 1. Different TCXO voltage trim values (1.8V, 3.0V, 3.3V)
 * 2. Extended startup delays (up to 500ms)
 * 3. Multiple clear/retry cycles
 * 
 * Run this if the HF_XOSC_START_ERR persists after normal initialization.
 * It will determine if the issue is software-fixable or hardware-related.
 */
void tcxo_deep_diagnostic(void);

/**
 * @brief Quick TCXO test with current configuration
 * 
 * Tests the TCXO with current settings and reports status.
 * Run this first before the deep diagnostic.
 */
void tcxo_quick_test(void);

#ifdef __cplusplus
}
#endif

#endif /* TCXO_DIAGNOSTIC_H */
