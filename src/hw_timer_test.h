/**
 * @file hw_timer_test.h
 * @brief Hardware Timer Test for ELRS on SiWx917
 *
 * Test suite to validate the CT-based hardware timer is working correctly
 * for ELRS protocol timing requirements.
 */

#ifndef HW_TIMER_TEST_H
#define HW_TIMER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run all hardware timer tests
 *
 * Executes the complete test suite:
 *   1. Basic initialization
 *   2. Timing accuracy
 *   3. Interval change
 *   4. Phase shift
 *   5. Frequency offset
 *   6. Start/stop stress test
 *
 * @return 0 if all tests pass, number of failures otherwise
 */
int hw_timer_test_run_all(void);

/**
 * @brief Run timing-only quick test
 *
 * Quick test that runs the timer for 1 second and verifies
 * the callback rate matches the configured interval.
 * Useful for a quick sanity check.
 *
 * @return 0 on success, negative on failure
 */
int hw_timer_test_timing_only(void);

#ifdef __cplusplus
}
#endif

#endif /* HW_TIMER_TEST_H */
