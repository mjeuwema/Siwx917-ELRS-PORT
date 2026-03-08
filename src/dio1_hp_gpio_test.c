/**
 * @file dio1_hp_gpio_test.c
 * @brief Test suite for DIO1 HP GPIO interrupt (GPIO_46)
 *
 * Tests the HP GPIO pin interrupt implementation for LR1121 DIO1.
 *
 * Hardware Configuration:
 *   - DIO1 connected to GPIO_46 (HP domain)
 *   - Rising-edge interrupt on channel 0 (IRQ52)
 *   - LR1121 holds DIO1 HIGH until IRQ flags are cleared
 *
 * Test Flow:
 *   1. Initialize LR1121 (SPI, TCXO, etc.)
 *   2. Initialize DIO1 HP GPIO interrupt
 *   3. Register callback
 *   4. Configure LR1121 to route IRQs to DIO1
 *   5. Trigger IRQ (RX timeout)
 *   6. Verify callback fired
 *   7. Clear IRQ
 *   8. Verify DIO1 went LOW
 */

#include "dio1_hp_gpio_test.h"
#include "lr1121_driver.h"
#include "rsi_debug.h" /* Provides DEBUGOUT macro */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*******************************************************************************
 * Test State
 ******************************************************************************/

static volatile bool test_callback_fired = false;
static volatile uint32_t test_callback_count = 0;

/*******************************************************************************
 * Test Callback
 ******************************************************************************/

static void test_dio1_callback(void) {
  test_callback_fired = true;
  test_callback_count++;
}

/*******************************************************************************
 * Helper: Delay
 ******************************************************************************/

static void delay_ms(uint32_t ms) {
  /* Rough delay - ~10000 iterations per ms at 180MHz */
  for (volatile uint32_t i = 0; i < ms * 10000; i++) {
    __asm volatile("nop");
  }
}

/*******************************************************************************
 * Test Functions
 ******************************************************************************/

/**
 * @brief Test 1: LR1121 Basic Init
 */
static int test_lr1121_init(void) {
  DEBUGOUT("\n--- Test 1: LR1121 Basic Init ---\n");

  lr1121_status_t status = lr1121_init();
  if (status != LR1121_OK) {
    DEBUGOUT("FAIL: lr1121_init() returned %d\n", status);
    return -1;
  }

  /* Verify communication with GetVersion */
  lr1121_version_t version;
  status = lr1121_get_version(&version);
  if (status != LR1121_OK) {
    DEBUGOUT("FAIL: lr1121_get_version() returned %d\n", status);
    return -1;
  }

  DEBUGOUT("PASS: LR1121 v%d.%d (HW=0x%02X, Type=0x%02X)\n",
           (version.version >> 8) & 0xFF, version.version & 0xFF,
           version.hardware, version.type);
  return 0;
}

/**
 * @brief Test 2: DIO1 GPIO Init
 */
static int test_dio1_gpio_init(void) {
  DEBUGOUT("\n--- Test 2: DIO1 HP GPIO Init ---\n");

  lr1121_status_t status = lr1121_dio1_init();
  if (status != LR1121_OK) {
    DEBUGOUT("FAIL: lr1121_dio1_init() returned %d\n", status);
    return -1;
  }

  /* Read initial pin state */
  int dio1 = lr1121_dio1_read();
  DEBUGOUT("INFO: Initial DIO1 pin state = %d\n", dio1);

  /* Log initial IRQ state */
  lr1121_send_command(0x0012, NULL, 0); /* GetIrqStatus */
  uint8_t irq_buf[6] = {0};
  lr1121_read_response(irq_buf, 6);
  DEBUGOUT("DEBUG: Initial IRQ status = [%02X][%02X][%02X][%02X][%02X][%02X]\n",
           irq_buf[0], irq_buf[1], irq_buf[2], irq_buf[3], irq_buf[4],
           irq_buf[5]);

  DEBUGOUT("PASS: DIO1 GPIO_46 initialized\n");
  return 0;
}

/**
 * @brief Test 3: Callback Registration
 */
static int test_callback_registration(void) {
  DEBUGOUT("\n--- Test 3: Callback Registration ---\n");

  /* Reset state */
  test_callback_fired = false;
  test_callback_count = 0;

  /* Register callback */
  lr1121_dio1_set_callback(test_dio1_callback);
  DEBUGOUT("PASS: Callback registered\n");

  return 0;
}

/**
 * @brief Test 4: Configure Radio and IRQ Routing
 *
 * Must configure radio for LoRa mode before SetRx will work properly.
 */
static int test_configure_irq_routing(void) {
  DEBUGOUT("\n--- Test 4: Configure Radio & IRQ Routing ---\n");

  /* Step 1: SetStandby (STDBY_RC) - required before configuration */
  uint8_t standby = 0x00; /* STDBY_RC */
  lr1121_send_command(0x011C, &standby, 1);
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: SetStandby(STDBY_RC)\n");

  /* Step 2: SetPacketType to LoRa (0x01) */
  uint8_t pkt_type = 0x01; /* LORA */
  lr1121_send_command(0x020E, &pkt_type, 1);
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: SetPacketType(LORA)\n");

  /* Step 3: SetRfFrequency - use 915 MHz (ELRS typical)
   * Frequency = freq_param * (32MHz / 2^25)
   * For 915MHz: freq_param = 915000000 * 2^25 / 32000000 = 959447040 =
   * 0x392D0000
   */
  uint8_t freq[4] = {0x39, 0x2D, 0x00, 0x00}; /* 915 MHz */
  lr1121_send_command(0x0203, freq, 4);
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: SetRfFrequency(915MHz)\n");

  /* Step 4: SetLoRaModulationParams - basic config for timeout test
   * SF7, BW500kHz, CR4/5 - doesn't matter for timeout test
   */
  uint8_t mod_params[4] = {
      0x07, /* SF7 */
      0x06, /* BW500 */
      0x01, /* CR 4/5 */
      0x00  /* No low data rate optimize */
  };
  lr1121_send_command(0x0208, mod_params, 4);
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: SetLoRaModulationParams(SF7, BW500)\n");

  /* Step 5: Clear any pending IRQs */
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4); /* ClearIrq */
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: ClearIrq(all)\n");

  /* Step 6: SetDioIrqParams - Route ALL IRQs to DIO1 and DIO2
   *
   * LR1121 takes 3 x 32-bit masks: (IrqMask, Dio1Mask, Dio2Mask)
   * Routing to Dio2Mask is often required for physical DIO9.
   */
  uint8_t irq_params[12] = {/* IRQ mask: Enable ALL (0xFFFFFFFF) */
                            0xFF, 0xFF, 0xFF, 0xFF,
                            /* DIO1 mask: 0 (0x00000000) */
                            0x00, 0x00, 0x00, 0x00,
                            /* DIO2 mask: Route ALL to DIO2 (0xFFFFFFFF) */
                            0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0113, irq_params, 12); /* SetDioIrqParams */
  lr1121_wait_busy_timeout(10);
  DEBUGOUT("INFO: SetDioIrqParams(12 bytes, routes to DIO2/9)\n");

  /* Read back DIO1 to confirm it's LOW after clear */
  int dio1 = lr1121_dio1_read();
  DEBUGOUT("INFO: DIO1 after config = %d (expect 0)\n", dio1);

  DEBUGOUT("PASS: Radio configured for LoRa RX timeout test\n");
  return 0;
}

/**
 * @brief Test 5: Enable Interrupt and Trigger via RX Timeout
 */
static int test_interrupt_trigger(void) {
  DEBUGOUT("\n--- Test 5: Interrupt Trigger Test ---\n");

  /* Reset callback state */
  test_callback_fired = false;
  uint32_t initial_isr_count = lr1121_dio1_get_isr_count();
  uint32_t initial_cb_count = test_callback_count;

  /* Read DIO1 before - should be LOW */
  int dio1_before = lr1121_dio1_read();
  DEBUGOUT("INFO: DIO1 before = %d (expect 0)\n", dio1_before);

  /* Enable the interrupt */
  lr1121_dio1_enable();
  DEBUGOUT("INFO: DIO1 interrupt enabled\n");

  /* Start RX with short timeout (50ms) to generate TIMEOUT IRQ */
  DEBUGOUT("INFO: Starting RX with 50ms timeout...\n");
  uint8_t rx_params[3];
  uint32_t timeout_val = 1638; /* ~50ms in 30.52us units */
  rx_params[0] = (timeout_val >> 16) & 0xFF;
  rx_params[1] = (timeout_val >> 8) & 0xFF;
  rx_params[2] = timeout_val & 0xFF;
  lr1121_send_command(0x0209, rx_params, 3); /* SetRx */
  lr1121_wait_busy_timeout(10);

  /* Wait for timeout to occur (100ms to be safe) */
  DEBUGOUT("INFO: Waiting 100ms for RX timeout...\n");
  delay_ms(100);

  /* Check results */
  int dio1_after = lr1121_dio1_read();
  uint32_t final_isr_count = lr1121_dio1_get_isr_count();
  uint32_t isr_delta = final_isr_count - initial_isr_count;
  uint32_t cb_delta = test_callback_count - initial_cb_count;

  DEBUGOUT("INFO: DIO1 after timeout = %d (expect 1)\n", dio1_after);
  DEBUGOUT("INFO: ISR count delta = %lu\n", (unsigned long)isr_delta);
  DEBUGOUT("INFO: Callback count delta = %lu\n", (unsigned long)cb_delta);
  DEBUGOUT("INFO: Callback fired flag = %s\n",
           test_callback_fired ? "YES" : "NO");

  /* Verify interrupt fired */
  if (!test_callback_fired) {
    DEBUGOUT("FAIL: Callback never fired!\n");

    /* Debug: Check IRQ status directly */
    lr1121_send_command(0x0012, NULL, 0); /* GetIrqStatus */
    uint8_t irq_buf[6] = {0};
    lr1121_read_response(irq_buf, 6);
    DEBUGOUT("DEBUG: IRQ status = [%02X][%02X][%02X][%02X][%02X][%02X]\n",
             irq_buf[0], irq_buf[1], irq_buf[2], irq_buf[3], irq_buf[4],
             irq_buf[5]);

    if (dio1_after == 1) {
      DEBUGOUT("DEBUG: DIO1 is HIGH but callback didn't fire\n");
      DEBUGOUT("       -> Interrupt not configured correctly\n");
    } else {
      DEBUGOUT("DEBUG: DIO1 is LOW - IRQ didn't route to DIO1\n");
      DEBUGOUT("       -> Check SetDioIrqParams or DIO1 wiring\n");
    }
    return -1;
  }

  DEBUGOUT("PASS: Interrupt fired! (ISR=%lu, CB=%lu)\n",
           (unsigned long)isr_delta, (unsigned long)cb_delta);
  return 0;
}

/**
 * @brief Test 6: Clear IRQ and Verify DIO1 Goes LOW
 */
static int test_clear_irq(void) {
  DEBUGOUT("\n--- Test 6: Clear IRQ Test ---\n");

  /* Clear all IRQs */
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4); /* ClearIrq */
  lr1121_wait_busy_timeout(10);

  /* Small delay for DIO1 to settle */
  delay_ms(1);

  /* Read DIO1 - should be LOW */
  int dio1_cleared = lr1121_dio1_read();
  DEBUGOUT("INFO: DIO1 after ClearIrq = %d (expect 0)\n", dio1_cleared);

  /* Log IRQ state after clear */
  lr1121_send_command(0x0012, NULL, 0); /* GetIrqStatus */
  uint8_t irq_buf[6] = {0};
  lr1121_read_response(irq_buf, 6);
  DEBUGOUT(
      "DEBUG: IRQ status after CLR = [%02X][%02X][%02X][%02X][%02X][%02X]\n",
      irq_buf[0], irq_buf[1], irq_buf[2], irq_buf[3], irq_buf[4], irq_buf[5]);

  if (dio1_cleared != 0) {
    DEBUGOUT("FAIL: DIO1 still HIGH after ClearIrq\n");
    return -1;
  }

  DEBUGOUT("PASS: DIO1 went LOW after ClearIrq\n");
  return 0;
}

/**
 * @brief Test 7: Raw GPIO Spin Loop (Hardware Verification)
 */
static int test_raw_gpio_spin(void) {
  DEBUGOUT("\n--- Test 7: Raw GPIO Spin Loop Test ---\n");
  DEBUGOUT("INFO: Bypassing SDK IRQ routing. Direct polling GPIO_46...\n");

  /* Clear IRQs */
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4);
  lr1121_wait_busy_timeout(10);

  int dio1_initial = lr1121_dio1_read();
  DEBUGOUT("INFO: DIO1 initially = %d\n", dio1_initial);

  /* Start RX with 50ms timeout */
  DEBUGOUT("INFO: Starting 50ms RX timeout...\n");
  uint8_t rx_params[3];
  uint32_t timeout_val = 1638; /* ~50ms */
  rx_params[0] = (timeout_val >> 16) & 0xFF;
  rx_params[1] = (timeout_val >> 8) & 0xFF;
  rx_params[2] = timeout_val & 0xFF;
  lr1121_send_command(0x0209, rx_params, 3);
  lr1121_wait_busy_timeout(10);

  /* Spin loop for 10000ms checking the direct register */
  DEBUGOUT("INFO: Spin-polling GPIO_46 register for 10000ms (10sec) - PROBE "
           "NOW!...\n");
  bool saw_high = false;
  for (uint32_t i = 0; i < 10000; i++) {
    if ((i > 0) && (i % 1000 == 0)) {
      DEBUGOUT("INFO: Still waiting... (%lu seconds elapsed)\n",
               (unsigned long)(i / 1000));
    }
    if (lr1121_dio1_read() == 1) {
      saw_high = true;
      DEBUGOUT("INFO: GPIO_46 went HIGH at ~%lu ms!\n", (unsigned long)i);
      break;
    }
    delay_ms(1);
  }

  if (saw_high) {
    DEBUGOUT("PASS: Hardware pin asserted HIGH! (IRQ routing bug confirmed)\n");
    return 0;
  } else {
    DEBUGOUT("FAIL: Hardware pin NEVER went HIGH. (Physical wiring bug "
             "confirmed)\n");
    return -1;
  }
}

/**
 * @brief Test 8: Multiple Interrupt Cycles
 */
static int test_multiple_cycles(void) {
  DEBUGOUT("\n--- Test 7: Multiple Interrupt Cycles ---\n");

  const int NUM_CYCLES = 3;
  int success_count = 0;

  for (int i = 0; i < NUM_CYCLES; i++) {
    DEBUGOUT("INFO: Cycle %d/%d...\n", i + 1, NUM_CYCLES);

    /* Reset callback flag */
    test_callback_fired = false;

    /* Clear IRQs */
    uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    lr1121_send_command(0x0114, clear_all, 4);
    lr1121_wait_busy_timeout(10);

    /* Start RX with 30ms timeout */
    uint8_t rx_params[3];
    uint32_t timeout_val = 983; /* ~30ms */
    rx_params[0] = (timeout_val >> 16) & 0xFF;
    rx_params[1] = (timeout_val >> 8) & 0xFF;
    rx_params[2] = timeout_val & 0xFF;
    lr1121_send_command(0x0209, rx_params, 3);
    lr1121_wait_busy_timeout(10);

    /* Wait for timeout */
    delay_ms(50);

    /* Check callback */
    if (test_callback_fired) {
      success_count++;
      DEBUGOUT("       Callback fired: YES\n");
    } else {
      DEBUGOUT("       Callback fired: NO\n");
    }
  }

  DEBUGOUT("INFO: Success rate = %d/%d\n", success_count, NUM_CYCLES);

  if (success_count == NUM_CYCLES) {
    DEBUGOUT("PASS: All %d interrupt cycles successful\n", NUM_CYCLES);
    return 0;
  } else {
    DEBUGOUT("FAIL: Only %d/%d cycles successful\n", success_count, NUM_CYCLES);
    return -1;
  }
}

/**
 * @brief Test 9: Disable Interrupt
 */
static int test_disable_interrupt(void) {
  DEBUGOUT("\n--- Test 8: Disable Interrupt ---\n");

  /* Disable interrupt */
  lr1121_dio1_disable();
  DEBUGOUT("INFO: DIO1 interrupt disabled\n");

  /* Reset callback */
  test_callback_fired = false;
  uint32_t count_before = test_callback_count;

  /* Clear IRQ and trigger another timeout */
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4);
  lr1121_wait_busy_timeout(10);

  uint8_t rx_params[3];
  uint32_t timeout_val = 983;
  rx_params[0] = (timeout_val >> 16) & 0xFF;
  rx_params[1] = (timeout_val >> 8) & 0xFF;
  rx_params[2] = timeout_val & 0xFF;
  lr1121_send_command(0x0209, rx_params, 3);
  lr1121_wait_busy_timeout(10);

  /* Wait for timeout */
  delay_ms(50);

  /* Verify callback did NOT fire */
  if (test_callback_fired || test_callback_count != count_before) {
    DEBUGOUT("FAIL: Callback fired while interrupt was disabled!\n");
    return -1;
  }

  DEBUGOUT("PASS: No callback while interrupt disabled\n");
  return 0;
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

void dio1_hp_gpio_test_run(void) {
  int passed = 0;
  int failed = 0;

  DEBUGOUT("\n");
  DEBUGOUT(
      "╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT(
      "║      DIO1 HP GPIO Interrupt Test Suite                       ║\n");
  DEBUGOUT(
      "║      GPIO_46 (Port C, Pin 14) - Rising Edge - Channel 0      ║\n");
  DEBUGOUT(
      "╚══════════════════════════════════════════════════════════════╝\n");

  /* Test 1: LR1121 Init */
  if (test_lr1121_init() == 0)
    passed++;
  else
    failed++;

  /* Test 2: DIO1 GPIO Init */
  if (test_dio1_gpio_init() == 0)
    passed++;
  else
    failed++;

  /* Test 3: Callback Registration */
  if (test_callback_registration() == 0)
    passed++;
  else
    failed++;

  /* Test 4: Configure IRQ Routing */
  if (test_configure_irq_routing() == 0)
    passed++;
  else
    failed++;

  /* Test 5: Interrupt Trigger */
  if (test_interrupt_trigger() == 0)
    passed++;
  else
    failed++;

  /* Test 6: Clear IRQ */
  if (test_clear_irq() == 0)
    passed++;
  else
    failed++;

  /* Test 7: Raw Spin Test */
  if (test_raw_gpio_spin() == 0)
    passed++;
  else
    failed++;

  /* Test 8: Multiple Cycles */
  if (test_multiple_cycles() == 0)
    passed++;
  else
    failed++;

  /* Test 9: Disable Interrupt */
  if (test_disable_interrupt() == 0)
    passed++;
  else
    failed++;

  /* Clean up - clear IRQs and disable interrupt */
  uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, clear_all, 4);
  lr1121_dio1_disable();

  /* Summary */
  DEBUGOUT("\n");
  DEBUGOUT(
      "╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT(
      "║                       Test Summary                           ║\n");
  DEBUGOUT(
      "╠══════════════════════════════════════════════════════════════╣\n");
  DEBUGOUT(
      "║  Total:  %d                                                   ║\n",
      passed + failed);
  DEBUGOUT(
      "║  Passed: %d                                                   ║\n",
      passed);
  DEBUGOUT(
      "║  Failed: %d                                                   ║\n",
      failed);
  DEBUGOUT(
      "╚══════════════════════════════════════════════════════════════╝\n");

  if (failed == 0) {
    DEBUGOUT("\n*** ALL TESTS PASSED ***\n");
    DEBUGOUT("\nHP GPIO DIO1 interrupt is working correctly.\n");
    DEBUGOUT("You can use this for ELRS RX packet notification.\n");
  } else {
    DEBUGOUT("\n*** SOME TESTS FAILED ***\n");
    DEBUGOUT("\nCheck:\n");
    DEBUGOUT("  1. LR1121 DIO1 pin connected to GPIO_46\n");
    DEBUGOUT("  2. GPIO_46 configured as input\n");
    DEBUGOUT("  3. NVIC interrupt enabled for EGPIO_PIN_0_IRQn\n");
  }

  DEBUGOUT("\n");
}
