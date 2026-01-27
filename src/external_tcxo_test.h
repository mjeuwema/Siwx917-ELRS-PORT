/**
 * @file external_tcxo_test.h
 * @brief External TCXO Test Header for Core1121-HF Module
 */

#ifndef EXTERNAL_TCXO_TEST_H
#define EXTERNAL_TCXO_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Test 1: Skip SetTcxoMode entirely
 * For externally-powered TCXO, just try SetStandby(XOSC) directly
 */
void test1_skip_tcxo_mode(void);

/**
 * @brief Test 2: SetTcxoMode with timeout=0 (external TCXO mode)
 */
void test2_tcxo_timeout_zero(void);

/**
 * @brief Test 3: Calibration in STANDBY_RC mode
 * Verify chip functionality without XOSC
 */
void test3_calibrate_in_rc_mode(void);

/**
 * @brief Test 4: Crystal vs TCXO detection with extended delays
 */
void test4_crystal_mode_check(void);

/**
 * @brief Run all external TCXO tests
 */
void external_tcxo_test_all(void);

#ifdef __cplusplus
}
#endif

#endif /* EXTERNAL_TCXO_TEST_H */
