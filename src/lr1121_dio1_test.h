/**
 * @file lr1121_dio1_test.h
 * @brief Test suite header for LR1121 DIO1 interrupt functionality
 */

#ifndef LR1121_DIO1_TEST_H
#define LR1121_DIO1_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run all DIO1 interrupt tests
 * 
 * Runs a comprehensive test suite including:
 * - Pin state reading
 * - Callback registration
 * - TX_DONE interrupt
 * - RX_DONE interrupt
 * - Enable/disable functionality
 */
void lr1121_dio1_run_all_tests(void);

/**
 * @brief Quick verification test for DIO1
 * 
 * Simple test that initializes DIO1 and reads its state 10 times.
 * Useful for quick hardware verification.
 */
void lr1121_dio1_quick_test(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_DIO1_TEST_H */
