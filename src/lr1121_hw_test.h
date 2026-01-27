/**
 * @file lr1121_hw_test.h
 * @brief LR1121 hardware diagnostic test interface
 */

#ifndef LR1121_HW_TEST_H
#define LR1121_HW_TEST_H

/**
 * @brief Run comprehensive LR1121 hardware diagnostic tests
 * 
 * This function runs a series of tests to diagnose LR1121 communication issues:
 * - GPIO pin state verification
 * - Raw SPI transfer testing
 * - Reset sequence validation
 * - GetStatus/GetVersion command tests
 * - Wakeup sequence verification
 * 
 * Call this function BEFORE ELRS initialization to isolate hardware issues.
 * 
 * Test results are printed to console with detailed diagnostics.
 */
void lr1121_run_hardware_test(void);

#endif /* LR1121_HW_TEST_H */
