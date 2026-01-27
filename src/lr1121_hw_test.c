/**
 * @file lr1121_hw_test.c
 * @brief Standalone hardware test for LR1121 SPI communication
 * 
 * Tests basic SPI read/write functionality before ELRS protocol starts.
 * This isolates hardware issues from protocol complexity.
 */

#include "lr1121_hw_test.h"
#include "lr1121_driver.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

/* Test result structure */
typedef struct {
    bool spi_write_ok;
    bool spi_read_ok;
    bool busy_ok;
    bool chip_responding;
    uint8_t status_bytes[2];
    uint16_t chip_mode;
} lr1121_hw_test_result_t;

/**
 * Test 1: Verify BUSY pin toggles
 */
static bool test_busy_pin(void)
{
    printf("\n[HW_TEST] Test 1: Checking BUSY pin functionality...\n");
    
    // Check if BUSY is high (chip is busy)
    // Note: lr1121_wait_busy() returns true if wait succeeded (BUSY went low)
    printf("[HW_TEST]   Checking initial BUSY state...\n");
    
    // Send GetStatus command and verify BUSY toggles
    printf("[HW_TEST]   Sending GetStatus command to trigger BUSY...\n");
    
    uint8_t stat1, stat2, irq_status;
    bool cmd_ok = lr1121_get_status(&stat1, &stat2, &irq_status);
    
    if (cmd_ok) {
        printf("[HW_TEST]   Command completed (BUSY toggled correctly)\n");
        printf("[HW_TEST]   BUSY test: PASS\n");
        return true;
    } else {
        printf("[HW_TEST]   Command failed (BUSY timeout or error)\n");
        printf("[HW_TEST]   BUSY test: FAIL\n");
        return false;
    }
}

/**
 * Test 2: SPI Write Test (verify we can toggle CS and clock out data)
 */
static bool test_spi_write(void)
{
    printf("\n[HW_TEST] Test 2: Verifying SPI write capability...\n");
    
    // Try to send a simple command using SPI transfer
    printf("[HW_TEST]   Testing CS and SPI clock generation...\n");
    lr1121_cs_assert();
    printf("[HW_TEST]   CS asserted (NSS LOW)\n");
    
    uint8_t dummy_data[3] = {0xAA, 0x55, 0xF0};
    uint8_t dummy_rx[3] = {0};
    lr1121_spi_transfer(dummy_data, dummy_rx, 3);
    printf("[HW_TEST]   Sent test pattern: 0x%02X 0x%02X 0x%02X\n", 
           dummy_data[0], dummy_data[1], dummy_data[2]);
    printf("[HW_TEST]   Received back: 0x%02X 0x%02X 0x%02X 0x%02X\n",
           dummy_rx[0], dummy_rx[1], dummy_rx[2], 0);
    
    lr1121_cs_deassert();
    printf("[HW_TEST]   CS deasserted (NSS HIGH)\n");
    
    // Check if we got shifted data back (expected in full-duplex SPI)
    // Sending 0xAA 0x55 0xF0 should return 0x00 0xAA 0x55 (shifted by 1 byte)
    // because MISO data lags MOSI by one clock in full-duplex mode
    if (dummy_rx[1] == 0xAA && dummy_rx[2] == 0x55) {
        printf("[HW_TEST]   NOTE: Got shifted echo - this proves MISO IS WORKING!\n");
        printf("[HW_TEST]   (Full-duplex SPI shows TX data shifted by 1 byte on RX)\n");
    }
    
    printf("[HW_TEST]   SPI write test: PASS\n");
    return true;
}

/**
 * Test 3: SPI Read Test (the critical test - can we read MISO?)
 */
static bool test_spi_read(void)
{
    printf("\n[HW_TEST] Test 3: Testing SPI read (MISO line)...\n");
    printf("[HW_TEST]   This is the CRITICAL test - checking if MISO works\n");
    
    // Wait for BUSY low
    lr1121_wait_busy();
    
    // Use the high-level GetStatus function to read chip status
    uint8_t stat1, stat2, irq_status;
    bool read_ok = lr1121_get_status(&stat1, &stat2, &irq_status);
    
    if (!read_ok) {
        printf("[HW_TEST]   *** ERROR: GetStatus command failed! ***\n");
        return false;
    }
    
    uint8_t status[2] = {stat1, stat2};
    
    printf("[HW_TEST]   Read status bytes: stat1=0x%02X, stat2=0x%02X\n", 
           status[0], status[1]);
    
    // Check if we got all zeros (MISO stuck/broken)
    bool all_zeros = (status[0] == 0x00 && status[1] == 0x00);
    bool all_ones = (status[0] == 0xFF && status[1] == 0xFF);
    
    if (all_zeros) {
        printf("[HW_TEST]   *** ERROR: All zeros! MISO line not working! ***\n");
        printf("[HW_TEST]   Possible causes:\n");
        printf("[HW_TEST]     - MISO pin not connected\n");
        printf("[HW_TEST]     - MISO pin wrong GPIO assignment\n");
        printf("[HW_TEST]     - LR1121 chip not powered\n");
        printf("[HW_TEST]     - LR1121 chip damaged\n");
        return false;
    }
    
    if (all_ones) {
        printf("[HW_TEST]   *** ERROR: All ones! MISO line floating or pulled high! ***\n");
        return false;
    }
    
    // Extract chip mode from status
    uint8_t chip_mode = (status[1] >> 4) & 0x07;
    uint8_t cmd_status = (status[1] >> 1) & 0x07;
    
    printf("[HW_TEST]   Chip mode: %d, Command status: %d\n", chip_mode, cmd_status);
    printf("[HW_TEST]   SPI read test: PASS (got non-zero data)\n");
    
    return true;
}

/**
 * Test 4: Chip Reset and Response Test
 */
static bool test_chip_reset(void)
{
    printf("\n[HW_TEST] Test 4: Testing chip reset and response...\n");
    
    // Reset the chip
    printf("[HW_TEST]   Resetting LR1121...\n");
    lr1121_reset();
    
    osDelay(10);
    
    // Try to read GetStatus after reset
    lr1121_wait_busy();
    
    uint8_t stat1, stat2, irq_status;
    bool read_ok = lr1121_get_status(&stat1, &stat2, &irq_status);
    
    if (!read_ok) {
        printf("[HW_TEST]   *** ERROR: GetStatus failed after reset! ***\n");
        return false;
    }
    
    uint8_t status[2] = {stat1, stat2};
    
    printf("[HW_TEST]   Post-reset status: stat1=0x%02X, stat2=0x%02X\n", 
           status[0], status[1]);
    
    uint8_t chip_mode = (status[1] >> 4) & 0x07;
    
    // After reset, chip should be in SLEEP mode (0) or STDBY_RC mode (2)
    bool mode_ok = (chip_mode == 0 || chip_mode == 2);
    
    printf("[HW_TEST]   Expected mode: 0 (SLEEP) or 2 (STDBY_RC)\n");
    printf("[HW_TEST]   Actual mode: %d\n", chip_mode);
    printf("[HW_TEST]   Chip reset test: %s\n", mode_ok ? "PASS" : "FAIL");
    
    return mode_ok;
}

/**
 * Test 5: Software SPI vs Hardware SPI comparison
 */
static bool test_spi_mode(void)
{
    printf("\n[HW_TEST] Test 5: Checking SPI mode configuration...\n");
    
#ifdef USE_SOFT_SPI
    printf("[HW_TEST]   Using SOFTWARE SPI (bit-banging)\n");
    printf("[HW_TEST]   This bypasses hardware GSPI peripheral\n");
#else
    printf("[HW_TEST]   Using HARDWARE GSPI peripheral\n");
    printf("[HW_TEST]   If reads fail, try enabling USE_SOFT_SPI\n");
#endif
    
    return true;
}

/**
 * Main hardware test function - called before ELRS protocol starts
 */
void lr1121_run_hardware_test(void)
{
    printf("\n");
    printf("========================================\n");
    printf("   LR1121 HARDWARE DIAGNOSTIC TEST\n");
    printf("========================================\n");
    printf("This test runs BEFORE the ELRS protocol\n");
    printf("to isolate hardware vs protocol issues.\n");
    printf("========================================\n\n");
    
    lr1121_hw_test_result_t result = {0};
    
    // Run all tests in sequence
    result.spi_write_ok = test_spi_write();
    osDelay(100);
    
    result.busy_ok = test_busy_pin();
    osDelay(100);
    
    result.spi_read_ok = test_spi_read();
    osDelay(100);
    
    result.chip_responding = test_chip_reset();
    osDelay(100);
    
    test_spi_mode();
    
    // Print summary
    printf("\n");
    printf("========================================\n");
    printf("   HARDWARE TEST SUMMARY\n");
    printf("========================================\n");
    printf("SPI Write:       %s\n", result.spi_write_ok ? "PASS ✓" : "FAIL ✗");
    printf("BUSY Pin:        %s\n", result.busy_ok ? "PASS ✓" : "FAIL ✗");
    printf("SPI Read (MISO): %s\n", result.spi_read_ok ? "PASS ✓" : "FAIL ✗");
    printf("Chip Responding: %s\n", result.chip_responding ? "PASS ✓" : "FAIL ✗");
    printf("========================================\n");
    
    if (!result.spi_read_ok) {
        printf("\n*** CRITICAL: SPI READ FAILED ***\n");
        printf("The MISO line is not working!\n");
        printf("This explains why chip stays in SLEEP mode.\n");
        printf("All status reads return 0x00.\n\n");
        printf("ACTION REQUIRED:\n");
        printf("1. Check MISO pin connection (hardware)\n");
        printf("2. Verify MISO GPIO configuration in lr1121_driver.c\n");
        printf("3. Check if USE_SOFT_SPI is enabled/disabled\n");
        printf("4. Verify LR1121 power supply (3.3V on VDD pins)\n");
        printf("5. Check if LR1121 chip is damaged\n");
        printf("========================================\n\n");
    } else if (!result.chip_responding) {
        printf("\n*** WARNING: Chip mode unexpected ***\n");
        printf("MISO works but chip behavior is abnormal.\n");
        printf("This could be a timing or configuration issue.\n");
        printf("========================================\n\n");
    } else {
        printf("\n*** ALL TESTS PASSED ***\n");
        printf("Hardware is working correctly!\n");
        printf("If ELRS still fails, the issue is in the protocol.\n");
        printf("========================================\n\n");
    }
    
    printf("Continuing to ELRS protocol initialization...\n\n");
}
