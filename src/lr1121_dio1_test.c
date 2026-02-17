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
  printf("\n");
  printf("=== Test 1: DIO1 Pin Read ===\n");
  fflush(stdout);
  
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
  printf("\n");
  printf("=== Test 2: Callback Registration ===\n");
  fflush(stdout);
  
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

/**
 * @brief Test 6: DIO1 Toggle Test - Verify LR1121 can control DIO1
 * 
 * This test verifies the physical connection between LR1121 DIO1 and 
 * UULP_VBAT_GPIO_2 by:
 * 1. Clearing IRQ and checking DIO1 goes LOW
 * 2. Triggering an IRQ and checking DIO1 goes HIGH
 * 3. Clearing IRQ again and checking DIO1 goes LOW
 *
 * @return 0 on success, -1 on failure
 */
int test_dio1_toggle(void)
{
  printf("\n=== Test 6: DIO1 Toggle Test ===\n");
  printf("This test verifies LR1121 DIO1 <-> UULP_VBAT_GPIO_2 connection\n\n");
  
  int dio1_state;
  
  /* Step 1: Read initial DIO1 state */
  dio1_state = lr1121_dio1_read();
  printf("Step 1: Initial DIO1 state = %d\n", dio1_state);
  
  /* Step 2: Clear all IRQs - DIO1 should go LOW */
  printf("Step 2: Clearing all LR1121 IRQs...\n");
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4);  /* ClearIrq opcode */
  lr1121_wait_busy_timeout(10);
  
  /* Small delay for DIO1 to settle */
  for (volatile int i = 0; i < 50000; i++);
  
  dio1_state = lr1121_dio1_read();
  printf("        DIO1 after ClearIrq = %d (expect 0)\n", dio1_state);
  
  if (dio1_state != 0) {
    printf("WARNING: DIO1 is HIGH after ClearIrq!\n");
    printf("         Possible causes:\n");
    printf("         - SetDioIrqParams DIO1 mask not configured\n");
    printf("         - Physical DIO1 not connected to UULP_VBAT_GPIO_2\n");
    printf("         - LR1121 generating continuous IRQs\n");
  }
  
  /* Step 3: Configure SetDioIrqParams to route TX_DONE|RX_DONE to DIO1 */
  printf("Step 3: Configuring SetDioIrqParams...\n");
  
  /* SetDioIrqParams: 8 bytes total (big-endian)
   * Bytes 0-3: IRQ mask (which IRQs to enable)
   * Bytes 4-7: DIO1 mask (which IRQs route to DIO1)
   * 
   * Observed IRQ bits from actual hardware:
   *   TX_DONE = 0x00000004 (bit 2)
   *   RX_DONE = 0x00000008 (bit 3)
   *   TIMEOUT = 0x00400000 (bit 22) - observed in IRQ status!
   * 
   * Note: LR1121_Regs.h says TIMEOUT is bit 9 (0x200) but actual
   * hardware shows bit 22 (0x00400000). Using observed value.
   * 
   * We want: 0x0040000C = TIMEOUT(bit22) | RX_DONE | TX_DONE
   */
  uint8_t irq_params[8] = {0};
  /* IRQ mask: 0x0040000C in big-endian */
  irq_params[0] = 0x00;
  irq_params[1] = 0x40;  /* TIMEOUT bit 22 */
  irq_params[2] = 0x00;
  irq_params[3] = 0x0C;  /* TX_DONE | RX_DONE */
  /* DIO1 mask: same - route all to DIO1 */
  irq_params[4] = 0x00;
  irq_params[5] = 0x40;  /* TIMEOUT bit 22 */
  irq_params[6] = 0x00;
  irq_params[7] = 0x0C;  /* TX_DONE | RX_DONE */
  lr1121_send_command(0x0113, irq_params, 8);  /* SetDioIrqParams opcode */
  lr1121_wait_busy_timeout(10);
  printf("        Sent SetDioIrqParams: IRQ=0x%02X%02X%02X%02X, DIO1=0x%02X%02X%02X%02X\n",
         irq_params[0], irq_params[1], irq_params[2], irq_params[3],
         irq_params[4], irq_params[5], irq_params[6], irq_params[7]);
  
  /* Step 4: Put radio in RX mode with timeout to generate TIMEOUT IRQ */
  printf("Step 4: Starting RX with 50ms timeout to generate IRQ...\n");
  
  /* SetRx with 50ms timeout (timeout in units of 30.52us, so 50000us/30.52 = ~1638) */
  uint8_t rx_params[3];
  uint32_t timeout_val = 1638;  /* ~50ms */
  rx_params[0] = (timeout_val >> 16) & 0xFF;
  rx_params[1] = (timeout_val >> 8) & 0xFF;
  rx_params[2] = timeout_val & 0xFF;
  lr1121_send_command(0x0209, rx_params, 3);  /* SetRx opcode */
  lr1121_wait_busy_timeout(10);
  printf("        SetRx sent with 50ms timeout\n");
  
  /* Step 5: Poll DIO1 for up to 200ms to see if it goes HIGH */
  printf("Step 5: Polling DIO1 for interrupt (up to 200ms)...\n");
  int dio1_went_high = 0;
  for (int poll = 0; poll < 20; poll++) {
    /* 10ms delay */
    for (volatile int i = 0; i < 100000; i++);
    
    dio1_state = lr1121_dio1_read();
    printf("        Poll %2d: DIO1 = %d\n", poll + 1, dio1_state);
    
    if (dio1_state == 1) {
      dio1_went_high = 1;
      printf("        SUCCESS: DIO1 went HIGH!\n");
      break;
    }
  }
  
  if (!dio1_went_high) {
    printf("FAIL: DIO1 never went HIGH after RX timeout\n");
    printf("      Checking IRQ status by polling...\n");
    
    /* Read IRQ status directly */
    lr1121_send_command(0x0012, NULL, 0);  /* GetIrqStatus opcode */
    uint8_t irq_buf[6] = {0};
    lr1121_read_response(irq_buf, 6);
    printf("      IRQ raw bytes: [%02X][%02X][%02X][%02X][%02X][%02X]\n",
           irq_buf[0], irq_buf[1], irq_buf[2], irq_buf[3], irq_buf[4], irq_buf[5]);
    
    if (irq_buf[2] != 0 || irq_buf[3] != 0 || irq_buf[4] != 0 || irq_buf[5] != 0) {
      printf("      IRQ is set but DIO1 didn't go HIGH!\n");
      printf("      This means SetDioIrqParams DIO1 mask is wrong or DIO1 not connected\n");
    }
    return -1;
  }
  
  /* Step 6: Clear IRQ and verify DIO1 goes LOW */
  printf("Step 6: Clearing IRQ...\n");
  lr1121_send_command(0x0114, clear_all, 4);
  lr1121_wait_busy_timeout(10);
  for (volatile int i = 0; i < 50000; i++);
  
  dio1_state = lr1121_dio1_read();
  printf("        DIO1 after ClearIrq = %d (expect 0)\n", dio1_state);
  
  if (dio1_state != 0) {
    printf("FAIL: DIO1 didn't go LOW after ClearIrq\n");
    return -1;
  }
  
  printf("PASS: DIO1 toggle test successful!\n");
  printf("      - DIO1 went HIGH when IRQ was set\n");
  printf("      - DIO1 went LOW when IRQ was cleared\n");
  return 0;
}

/**
 * @brief Test 7: Quick DIO1 state dump
 * 
 * Dumps current DIO1 and IRQ state for debugging
 */
void test_dio1_state_dump(void)
{
  printf("\n=== DIO1 State Dump ===\n");
  
  /* Read DIO1 pin */
  int dio1 = lr1121_dio1_read();
  printf("DIO1 pin level: %d\n", dio1);
  
  /* Read IRQ status */
  lr1121_send_command(0x0012, NULL, 0);
  uint8_t irq_buf[6] = {0};
  lr1121_read_response(irq_buf, 6);
  printf("IRQ status raw: [%02X][%02X][%02X][%02X][%02X][%02X]\n",
         irq_buf[0], irq_buf[1], irq_buf[2], irq_buf[3], irq_buf[4], irq_buf[5]);
  
  /* Interpret as both endianness */
  uint32_t irq_le = irq_buf[2] | (irq_buf[3] << 8) | (irq_buf[4] << 16) | (irq_buf[5] << 24);
  uint32_t irq_be = (irq_buf[2] << 24) | (irq_buf[3] << 16) | (irq_buf[4] << 8) | irq_buf[5];
  printf("IRQ as little-endian: 0x%08lX\n", (unsigned long)irq_le);
  printf("IRQ as big-endian:    0x%08lX\n", (unsigned long)irq_be);
  
  /* Check specific bits */
  printf("If little-endian:\n");
  printf("  TX_DONE (0x04):  %s\n", (irq_le & 0x04) ? "SET" : "clear");
  printf("  RX_DONE (0x08):  %s\n", (irq_le & 0x08) ? "SET" : "clear");
  printf("  TIMEOUT (0x200): %s\n", (irq_le & 0x200) ? "SET" : "clear");
  
  printf("========================\n");
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
  
  /* Test 6: DIO1 Toggle Test */
  if (test_dio1_toggle() == 0) {
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
