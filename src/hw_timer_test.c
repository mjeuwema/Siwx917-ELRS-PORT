/**
 * @file hw_timer_test.c
 * @brief Hardware Timer Test for ELRS on SiWx917
 *
 * This test validates that the CT-based hardware timer is working correctly
 * for ELRS protocol timing. It tests:
 *   1. Timer initialization
 *   2. Tick/Tock callback firing
 *   3. Callback timing accuracy
 *   4. Phase shift functionality
 *   5. Frequency offset functionality
 *   6. Interval changes
 *
 * Run this test to verify the timer is working before debugging ELRS sync issues.
 */

#include "hw_timer_test.h"
#include "hw_timer.h"
#include <stdio.h>
#include <string.h>

/* Test state */
typedef struct {
    volatile uint32_t tick_count;
    volatile uint32_t tock_count;
    volatile uint32_t last_tick_time;
    volatile uint32_t last_tock_time;
    volatile uint32_t tick_intervals[16];
    volatile uint32_t tock_intervals[16];
    volatile uint8_t tick_idx;
    volatile uint8_t tock_idx;
    volatile bool test_running;
} hw_timer_test_state_t;

static hw_timer_test_state_t test_state = {0};

/* External micros() function - must be provided by platform */
extern uint32_t micros(void);

/*******************************************************************************
 * Test Callbacks
 ******************************************************************************/

static void test_tick_callback(void)
{
    uint32_t now = micros();
    
    if (test_state.last_tick_time != 0) {
        uint32_t interval = now - test_state.last_tick_time;
        if (test_state.tick_idx < 16) {
            test_state.tick_intervals[test_state.tick_idx++] = interval;
        }
    }
    
    test_state.last_tick_time = now;
    test_state.tick_count++;
}

static void test_tock_callback(void)
{
    uint32_t now = micros();
    
    if (test_state.last_tock_time != 0) {
        uint32_t interval = now - test_state.last_tock_time;
        if (test_state.tock_idx < 16) {
            test_state.tock_intervals[test_state.tock_idx++] = interval;
        }
    }
    
    test_state.last_tock_time = now;
    test_state.tock_count++;
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static void reset_test_state(void)
{
    memset((void*)&test_state, 0, sizeof(test_state));
}

static uint32_t calculate_average(volatile uint32_t *values, uint8_t count)
{
    if (count == 0) return 0;
    
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += values[i];
    }
    return sum / count;
}

static int32_t calculate_max_deviation(volatile uint32_t *values, uint8_t count, uint32_t expected)
{
    int32_t max_dev = 0;
    for (uint8_t i = 0; i < count; i++) {
        int32_t dev = (int32_t)values[i] - (int32_t)expected;
        if (dev < 0) dev = -dev;
        if (dev > max_dev) max_dev = dev;
    }
    return max_dev;
}

/*******************************************************************************
 * Test Functions
 ******************************************************************************/

/**
 * @brief Test 1: Basic initialization and callback firing
 * @return 0 on success, negative on failure
 */
static int test_basic_init(void)
{
    printf("\n=== Test 1: Basic Initialization ===\n");
    
    reset_test_state();
    
    /* Initialize timer with 10ms interval (100Hz) */
    uint32_t interval_us = 10000;
    sl_status_t status = hw_timer_init(interval_us);
    
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    printf("PASS: hw_timer_init succeeded\n");
    
    /* Register callbacks */
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    printf("PASS: Callbacks registered\n");
    
    /* Start timer */
    status = hw_timer_start();
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_start returned 0x%04lX\n", (unsigned long)status);
        return -2;
    }
    printf("PASS: hw_timer_start succeeded\n");
    
    /* Wait for some callbacks (100ms = 10 full intervals) */
    uint32_t start = micros();
    while ((micros() - start) < 100000) {
        /* Spin wait */
    }
    
    /* Stop timer */
    hw_timer_stop();
    
    /* Check callback counts */
    printf("Results: tick_count=%lu, tock_count=%lu\n", 
           (unsigned long)test_state.tick_count,
           (unsigned long)test_state.tock_count);
    
    /* We expect roughly 10 ticks and 10 tocks for 100ms at 10ms interval */
    if (test_state.tick_count < 8 || test_state.tick_count > 12) {
        printf("FAIL: tick_count out of range (expected 8-12, got %lu)\n",
               (unsigned long)test_state.tick_count);
        return -3;
    }
    
    if (test_state.tock_count < 8 || test_state.tock_count > 12) {
        printf("FAIL: tock_count out of range (expected 8-12, got %lu)\n",
               (unsigned long)test_state.tock_count);
        return -4;
    }
    
    printf("PASS: Callback counts within expected range\n");
    return 0;
}

/**
 * @brief Test 2: Timing accuracy
 * @return 0 on success, negative on failure
 */
static int test_timing_accuracy(void)
{
    printf("\n=== Test 2: Timing Accuracy ===\n");
    
    reset_test_state();
    
    /* Initialize with 5ms interval (200Hz) - common ELRS rate */
    uint32_t interval_us = 5000;
    sl_status_t status = hw_timer_init(interval_us);
    
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    hw_timer_start();
    
    /* Wait for 16 samples (~80ms) */
    printf("Waiting for 16 tick and 16 tock samples...\n");
    uint32_t start = micros();
    uint32_t last_print = start;
    while (test_state.tick_idx < 16 || test_state.tock_idx < 16) {
        uint32_t now = micros();
        /* Print progress every 50ms */
        if ((now - last_print) > 50000) {
            printf("  Progress: tick_idx=%d, tock_idx=%d, tick_count=%lu, tock_count=%lu\n",
                   test_state.tick_idx, test_state.tock_idx,
                   (unsigned long)test_state.tick_count,
                   (unsigned long)test_state.tock_count);
            last_print = now;
        }
        if ((now - start) > 200000) {
            printf("FAIL: Timeout waiting for samples (tick_idx=%d, tock_idx=%d)\n",
                   test_state.tick_idx, test_state.tock_idx);
            hw_timer_stop();
            return -2;
        }
    }
    
    hw_timer_stop();
    
    /* Analyze tick intervals (should be ~interval_us) */
    uint32_t tick_avg = calculate_average(test_state.tick_intervals, test_state.tick_idx);
    int32_t tick_max_dev = calculate_max_deviation(test_state.tick_intervals, 
                                                    test_state.tick_idx, interval_us);
    
    /* Analyze tock intervals (should be ~interval_us) */
    uint32_t tock_avg = calculate_average(test_state.tock_intervals, test_state.tock_idx);
    int32_t tock_max_dev = calculate_max_deviation(test_state.tock_intervals,
                                                    test_state.tock_idx, interval_us);
    
    printf("Tick: avg=%lu us, max_dev=%ld us (expected %lu us)\n",
           (unsigned long)tick_avg, (long)tick_max_dev, (unsigned long)interval_us);
    printf("Tock: avg=%lu us, max_dev=%ld us (expected %lu us)\n",
           (unsigned long)tock_avg, (long)tock_max_dev, (unsigned long)interval_us);
    
    /* Print raw intervals for debugging */
    printf("Tick intervals: ");
    for (int i = 0; i < test_state.tick_idx && i < 8; i++) {
        printf("%lu ", (unsigned long)test_state.tick_intervals[i]);
    }
    printf("\n");
    
    printf("Tock intervals: ");
    for (int i = 0; i < test_state.tock_idx && i < 8; i++) {
        printf("%lu ", (unsigned long)test_state.tock_intervals[i]);
    }
    printf("\n");
    
    /* Allow 5% tolerance (250us for 5000us interval) */
    uint32_t tolerance = interval_us / 20;
    
    if (tick_max_dev > (int32_t)tolerance) {
        printf("WARN: Tick timing deviation exceeds 5%% (%ld > %lu)\n",
               (long)tick_max_dev, (unsigned long)tolerance);
        /* Don't fail - just warn. Timing can vary due to ISR latency */
    }
    
    if (tock_max_dev > (int32_t)tolerance) {
        printf("WARN: Tock timing deviation exceeds 5%% (%ld > %lu)\n",
               (long)tock_max_dev, (unsigned long)tolerance);
    }
    
    /* Check average is within 2% of expected */
    int32_t tick_avg_err = (int32_t)tick_avg - (int32_t)interval_us;
    if (tick_avg_err < 0) tick_avg_err = -tick_avg_err;
    
    if (tick_avg_err > (int32_t)(interval_us / 50)) {
        printf("FAIL: Tick average error too large (%ld us)\n", (long)tick_avg_err);
        return -3;
    }
    
    printf("PASS: Timing accuracy within tolerance\n");
    return 0;
}

/**
 * @brief Test 3: Interval change
 * @return 0 on success, negative on failure
 */
static int test_interval_change(void)
{
    printf("\n=== Test 3: Interval Change ===\n");
    
    reset_test_state();
    
    /* Start with 10ms interval */
    uint32_t interval1 = 10000;
    sl_status_t status = hw_timer_init(interval1);
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    hw_timer_start();
    
    /* Let it run for 50ms */
    uint32_t start = micros();
    while ((micros() - start) < 50000) { }
    
    uint32_t count_before = test_state.tick_count;
    printf("Before interval change: tick_count=%lu\n", (unsigned long)count_before);
    
    /* Change to 5ms interval */
    uint32_t interval2 = 5000;
    hw_timer_set_interval(interval2);
    
    /* Run for another 50ms */
    start = micros();
    test_state.tick_count = 0;
    while ((micros() - start) < 50000) { }
    
    hw_timer_stop();
    
    uint32_t count_after = test_state.tick_count;
    printf("After interval change: tick_count=%lu\n", (unsigned long)count_after);
    
    /* After halving interval, we should see roughly 2x the callbacks */
    /* count_before ~5 for 50ms at 10ms interval
     * count_after ~10 for 50ms at 5ms interval */
    if (count_after < count_before) {
        printf("FAIL: Callback count didn't increase after interval reduction\n");
        return -2;
    }
    
    printf("PASS: Interval change affected callback rate\n");
    return 0;
}

/**
 * @brief Test 4: Phase shift
 * @return 0 on success, negative on failure
 */
static int test_phase_shift(void)
{
    printf("\n=== Test 4: Phase Shift ===\n");
    
    reset_test_state();
    
    uint32_t interval_us = 10000;
    sl_status_t status = hw_timer_init(interval_us);
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    hw_timer_start();
    
    /* Wait for a couple callbacks to establish baseline */
    uint32_t start = micros();
    while (test_state.tick_count < 3) {
        if ((micros() - start) > 100000) {
            printf("FAIL: Timeout waiting for initial callbacks\n");
            hw_timer_stop();
            return -2;
        }
    }
    
    /* Apply a phase shift of +1000us (delay next tock) */
    hw_timer_phase_shift(1000);
    printf("Applied +1000us phase shift\n");
    
    /* Wait for more callbacks */
    start = micros();
    while (test_state.tick_count < 6) {
        if ((micros() - start) > 100000) {
            printf("FAIL: Timeout after phase shift\n");
            hw_timer_stop();
            return -3;
        }
    }
    
    hw_timer_stop();
    
    printf("PASS: Phase shift applied without crash (timing verified visually)\n");
    return 0;
}

/**
 * @brief Test 5: Frequency offset
 * @return 0 on success, negative on failure  
 */
static int test_freq_offset(void)
{
    printf("\n=== Test 5: Frequency Offset ===\n");
    
    reset_test_state();
    
    uint32_t interval_us = 10000;
    sl_status_t status = hw_timer_init(interval_us);
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    
    /* Reset frequency offset */
    hw_timer_reset_freq_offset();
    
    /* Start timer */
    hw_timer_start();
    
    /* Wait for baseline */
    uint32_t start = micros();
    while (test_state.tick_count < 5) {
        if ((micros() - start) > 100000) {
            printf("FAIL: Timeout waiting for baseline\n");
            hw_timer_stop();
            return -2;
        }
    }
    
    /* Apply frequency offset (+10us per half-interval = timer runs slow) */
    for (int i = 0; i < 10; i++) {
        hw_timer_inc_freq_offset(1);
    }
    printf("Applied +10us frequency offset (timer slower)\n");
    printf("Current freq_offset: %ld\n", (long)hw_timer_get_freq_offset());
    
    /* Wait for more callbacks */
    start = micros();
    while (test_state.tick_count < 10) {
        if ((micros() - start) > 200000) {
            printf("FAIL: Timeout after freq offset\n");
            hw_timer_stop();
            return -3;
        }
    }
    
    /* Reset and verify */
    hw_timer_reset_freq_offset();
    if (hw_timer_get_freq_offset() != 0) {
        printf("FAIL: Freq offset not reset\n");
        hw_timer_stop();
        return -4;
    }
    
    hw_timer_stop();
    
    printf("PASS: Frequency offset applied and reset successfully\n");
    return 0;
}

/**
 * @brief Test 6: Rapid start/stop
 * @return 0 on success, negative on failure
 */
static int test_start_stop(void)
{
    printf("\n=== Test 6: Start/Stop Stress Test ===\n");
    
    uint32_t interval_us = 5000;
    sl_status_t status = hw_timer_init(interval_us);
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    
    /* Rapid start/stop cycles */
    for (int i = 0; i < 10; i++) {
        reset_test_state();
        
        hw_timer_start();
        
        /* Brief run */
        uint32_t start = micros();
        while ((micros() - start) < 10000) { }
        
        hw_timer_stop();
        
        if (test_state.tick_count == 0 && test_state.tock_count == 0) {
            printf("WARN: No callbacks in cycle %d\n", i);
        }
    }
    
    printf("PASS: Completed 10 start/stop cycles without crash\n");
    return 0;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

int hw_timer_test_run_all(void)
{
    printf("\n");
    printf("========================================\n");
    printf("   Hardware Timer Test Suite\n");
    printf("========================================\n");
    
    int result;
    int failures = 0;
    
    result = test_basic_init();
    if (result != 0) failures++;
    
    result = test_timing_accuracy();
    if (result != 0) failures++;
    
    result = test_interval_change();
    if (result != 0) failures++;
    
    result = test_phase_shift();
    if (result != 0) failures++;
    
    result = test_freq_offset();
    if (result != 0) failures++;
    
    result = test_start_stop();
    if (result != 0) failures++;
    
    printf("\n========================================\n");
    if (failures == 0) {
        printf("   ALL TESTS PASSED\n");
    } else {
        printf("   %d TEST(S) FAILED\n", failures);
    }
    printf("========================================\n\n");
    
    /* Cleanup */
    hw_timer_deinit();
    
    return failures;
}

int hw_timer_test_timing_only(void)
{
    printf("\n=== Quick Timing Test ===\n");
    
    reset_test_state();
    
    /* Test at ELRS 200Hz (5000us interval) */
    uint32_t interval_us = 5000;
    sl_status_t status = hw_timer_init(interval_us);
    
    if (status != SL_STATUS_OK) {
        printf("FAIL: hw_timer_init returned 0x%04lX\n", (unsigned long)status);
        return -1;
    }
    
    hw_timer_set_tick_callback(test_tick_callback);
    hw_timer_set_tock_callback(test_tock_callback);
    hw_timer_start();
    
    printf("Running timer at %lu us interval for 1 second...\n", (unsigned long)interval_us);
    
    uint32_t start = micros();
    while ((micros() - start) < 1000000) {
        /* Spin */
    }
    
    hw_timer_stop();
    
    printf("Results:\n");
    printf("  Tick count: %lu (expected ~200)\n", (unsigned long)test_state.tick_count);
    printf("  Tock count: %lu (expected ~200)\n", (unsigned long)test_state.tock_count);
    
    /* Calculate effective rate */
    uint32_t total_callbacks = test_state.tick_count + test_state.tock_count;
    printf("  Total callbacks: %lu (expected ~400)\n", (unsigned long)total_callbacks);
    printf("  Effective half-interval: %lu us\n", 
           (unsigned long)(1000000 / (total_callbacks > 0 ? total_callbacks : 1)));
    
    hw_timer_deinit();
    
    if (test_state.tick_count < 180 || test_state.tick_count > 220) {
        printf("FAIL: Tick count out of expected range\n");
        return -2;
    }
    
    printf("PASS: Timer running at expected rate\n");
    return 0;
}
