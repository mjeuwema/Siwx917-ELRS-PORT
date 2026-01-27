/**
 * @file busy_monitor_test.h
 * @brief LR1121 BUSY Pin Monitor Test Header
 *
 * This test monitors the LR1121 BUSY pin (GPIO_29) to determine
 * if the LR1121 chip boots and shows any activity.
 *
 * Citation: LR1121 Datasheet Section 4.2.1 "Reset Timing"
 * - BUSY is HIGH for approximately 230ms after reset
 * - BUSY goes LOW when chip is ready for commands
 */

#ifndef BUSY_MONITOR_TEST_H
#define BUSY_MONITOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the complete BUSY pin monitor test suite
 *
 * This test performs:
 * 1. Quick BUSY pin read test (no reset)
 * 2. Hardware reset and monitor BUSY response
 * 3. Extended monitoring for 5 seconds
 *
 * Expected Results:
 * - If LR1121 is working: BUSY goes HIGH for ~230ms after reset, then LOW
 * - If LR1121 is not working: BUSY stays permanently LOW or HIGH
 */
void busy_monitor_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* BUSY_MONITOR_TEST_H */
