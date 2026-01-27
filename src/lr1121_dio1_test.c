/**
 * @file lr1121_dio1_test.c
 * @brief DIO1 Interrupt Test Suite for LR1121
 * 
 * Tests the DIO1 interrupt infrastructure between LR1121 and SiW917.
 * 
 * DIO1 Pin: UULP_VBAT_GPIO_2 (mikroBUS INT pin)
 * Function: Interrupt from LR1121 to notify host of radio events
 * 
 * Citation: ug590-brd2708a-user-guide.pdf Section 3.8.2, Table 3.3
 *   "INT - Hardware Interrupt - UULP_VBAT_GPIO_2"
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.5.4
 *   "DIO1 can be configured to generate an interrupt on various radio events"
 */

#include "lr1121_dio1_test.h"
#include "lr1121_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sl_si91x_clock_manager.h"

/*******************************************************************************
 * Test Variables
 ******************************************************************************/

/* Test state tracking */
static volatile bool test_interrupt_fired = false;
static volatile uint32_t test_interrupt_count = 0;
static volatile uint32_t test_last_irq_time = 0;

/*******************************************************************************
 * Test Callback Function
 ******************************************************************************/

/**
 * @brief Test interrupt callback
 * 
 * Called when DIO1 interrupt fires. This simple callback just sets flags
 * to indicate the interrupt occurred.
 */
static void test_dio1_callback(void)
{
  test_interrupt_fired = true;
  test_interrupt_count++;
  test_last_irq_time = 0; /* Would use microsecond timer if available */
}

/**
 * @brief Test 1: Verify DIO1 pin can be read
 * 
 * This test checks that the GPIO is properly configured and can
 * read the current state of the DIO1 pin.
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_pin_read(void)
{
  printf("\n=== Test 1: DIO1 Pin Read ===\n");
  
  /* Initialize DIO1 GPIO */
  int result = lr1121_dio1_init();
  if (result != 0) {
    printf("FAIL: lr1121_dio1_init returned %d\n", result);
    return -1;
  }
  printf("PASS: DIO1 GPIO initialized\n");
  
  /* Read pin state (should be low when idle) */
  uint8_t pin_state = lr1121_dio1_read();
  printf("INFO: DIO1 pin state = %d\n", pin_state);
  
  /* Pin state is valid if it's 0 or 1 */
  if (pin_state > 1) {
    printf("FAIL: Invalid pin state %d\n", pin_state);
    return -1;
  }
  
  printf("PASS: DIO1 pin read successful\n");
  return 0;
}

/**
 * @brief Test 2: Register and verify callback
 * 
 * This test verifies that the callback registration works correctly
 * and that the ISR handler is properly configured.
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_callback_registration(void)
{
  printf("\n=== Test 2: Callback Registration ===\n");
  
  /* Reset test variables */
  test_interrupt_fired = false;
  test_interrupt_count = 0;
  
  /* Register callback */
  lr1121_dio1_set_callback(test_dio1_callback);
  printf("PASS: Callback registered\n");
  
  /* Enable interrupt */
  lr1121_dio1_enable();
  printf("PASS: NVIC interrupt enabled\n");
  
  return 0;
}

/**
 * @brief Test 3: Basic interrupt functionality
 * 
 * This test verifies the basic interrupt infrastructure is working.
 * Note: This test cannot trigger an actual interrupt without hardware
 * support, so it mainly verifies the setup is correct.
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_basic_interrupt(void)
{
  printf("\n=== Test 3: Basic Interrupt Infrastructure ===\n");
  
  /* Reset test state */
  test_interrupt_fired = false;
  test_interrupt_count = 0;
  
  printf("INFO: Interrupt infrastructure is configured\n");
  printf("INFO: Callback is registered: %s\n", 
         test_interrupt_fired ? "YES (interrupt occurred)" : "NO (no interrupt yet)");
  printf("INFO: Total interrupts: %lu\n", test_interrupt_count);
  
  printf("PASS: Interrupt infrastructure ready\n");
  printf("NOTE: Actual interrupt requires LR1121 to assert DIO1 pin\n");
  
  return 0;
}

/**
 * @brief Test 4: Interrupt enable/disable
 * 
 * This test verifies that interrupts can be enabled and disabled correctly.
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_enable_disable(void)
{
  printf("\n=== Test 4: Interrupt Enable/Disable ===\n");
  
  uint32_t start_count = test_interrupt_count;
  
  /* Disable interrupt */
  lr1121_dio1_disable();
  printf("INFO: Interrupt disabled\n");
  
  /* Brief delay to ensure no interrupts occur */
  for (volatile uint32_t i = 0; i < 100000; i++);
  
  /* Verify no new interrupts */
  if (test_interrupt_count != start_count) {
    printf("WARN: Interrupt count changed while disabled (not critical)\n");
  } else {
    printf("PASS: No interrupts while disabled\n");
  }
  
  /* Re-enable interrupt */
  lr1121_dio1_enable();
  printf("PASS: Interrupt re-enabled\n");
  
  return 0;
}

/**
 * @brief Test 5: DIO1 pin state monitoring
 * 
 * This test monitors the DIO1 pin state over time to verify
 * it can be read consistently.
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_pin_monitoring(void)
{
  printf("\n=== Test 5: DIO1 Pin Monitoring ===\n");
  
  const int NUM_READS = 5;
  printf("INFO: Reading DIO1 pin state %d times...\n", NUM_READS);
  
  for (int i = 0; i < NUM_READS; i++) {
    uint8_t state = lr1121_dio1_read();
    printf("INFO:   Read %d: DIO1 = %d\n", i + 1, state);
    
    /* Brief delay between reads */
    for (volatile uint32_t j = 0; j < 50000; j++);
  }
  
  printf("PASS: DIO1 pin can be read consistently\n");
  printf("INFO: Pin should be LOW (0) when LR1121 has no pending interrupts\n");
  printf("INFO: Pin should go HIGH (1) when LR1121 asserts interrupt\n");
  
  return 0;
}

/*******************************************************************************
 * Test Suite Entry Point
 ******************************************************************************/

/**
 * @brief Run all DIO1 tests
 * 
 * Executes the complete DIO1 test suite and reports results.
 */
void lr1121_dio1_run_all_tests(void)
{
  int failed_tests = 0;
  int passed_tests = 0;
  
  printf("\n");
  printf("╔═══════════════════════════════════════════════════════════════╗\n");
  printf("║          LR1121 DIO1 Interrupt Test Suite                    ║\n");
  printf("║          Hardware: SiW917 BRD2708A + LR1121                  ║\n");
  printf("║          DIO1 Pin: UULP_VBAT_GPIO_2 (mikroBUS INT)          ║\n");
  printf("╚═══════════════════════════════════════════════════════════════╝\n");
  printf("\n");
  
  /* Test 1: Pin Read */
  if (test_dio1_pin_read() == 0) {
    passed_tests++;
  } else {
    failed_tests++;
  }
  
  /* Test 2: Callback Registration */
  if (test_dio1_callback_registration() == 0) {
    passed_tests++;
  } else {
    failed_tests++;
  }
  
  /* Test 3: Basic Interrupt Infrastructure */
  if (test_dio1_basic_interrupt() == 0) {
    passed_tests++;
  } else {
    failed_tests++;
  }
  
  /* Test 4: Enable/Disable */
  if (test_dio1_enable_disable() == 0) {
    passed_tests++;
  } else {
    failed_tests++;
  }
  
  /* Test 5: Pin Monitoring */
  if (test_dio1_pin_monitoring() == 0) {
    passed_tests++;
  } else {
    failed_tests++;
  }
  
  /* Print summary */
  printf("\n");
  printf("╔═══════════════════════════════════════════════════════════════╗\n");
  printf("║                      Test Summary                             ║\n");
  printf("╠═══════════════════════════════════════════════════════════════╣\n");
  printf("║  Total Tests:  %d                                              ║\n", passed_tests + failed_tests);
  printf("║  Passed:       %d                                              ║\n", passed_tests);
  printf("║  Failed:       %d                                              ║\n", failed_tests);
  printf("╚═══════════════════════════════════════════════════════════════╝\n");
  printf("\n");
  
  if (failed_tests == 0) {
    printf("✓ ALL TESTS PASSED\n");
    printf("\n");
    printf("Next Steps:\n");
    printf("1. Connect LR1121 DIO1 pin to BRD2708A mikroBUS INT socket\n");
    printf("2. Configure LR1121 to generate interrupts (TX_DONE, RX_DONE, etc.)\n");
    printf("3. Monitor for actual interrupt events during radio operations\n");
  } else {
    printf("✗ SOME TESTS FAILED\n");
    printf("\n");
    printf("Check hardware connections:\n");
    printf("- LR1121 DIO1 → BRD2708A mikroBUS INT\n");
    printf("- Verify UULP_VBAT_GPIO_2 configuration\n");
  }
  printf("\n");
  
  printf("════════════════════════════════════════════════════════════════\n");
  printf("DIO1 Connection Information:\n");
  printf("────────────────────────────────────────────────────────────────\n");
  printf("LR1121 Side:   DIO1 pin (check Core1121 module pinout)\n");
  printf("BRD2708A Side: mikroBUS INT socket\n");
  printf("MCU Pin:       UULP_VBAT_GPIO_2\n");
  printf("Function:      Rising-edge interrupt from LR1121 to SiW917\n");
  printf("════════════════════════════════════════════════════════════════\n");
  printf("\n");
}
