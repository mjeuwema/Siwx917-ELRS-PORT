/**
 * @file lr1121_dio1_interrupt_test.h
 * @brief Comprehensive DIO1 Interrupt Test Suite Header
 * 
 * Tests the end-to-end interrupt-driven operation:
 *   LR1121 DIO1 → UULP_VBAT_GPIO_2 → NVIC → ISR → osThreadFlagsSet → Task
 * 
 * Test Summary:
 *   Test 1: DIO1 IRQ Configuration - configures LR1121 IRQs routed to DIO1
 *   Test 2: RX Timeout Interrupt - triggers DIO1 via RX timeout
 *   Test 3: osThreadFlagsWait() - verifies RTOS task signaling
 *   Test 4: Multiple Interrupts - stress tests consecutive interrupts
 *   Test 5: Pin State Correlation - verifies DIO1 HIGH/LOW matches IRQ status
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 9.2 "Interrupt Management"
 * Citation: siw917x-family-rm.pdf Section 8.3.6 "UULP GPIO interrupt"
 * Citation: CMSIS-RTOS2 osThreadFlagsSet/Wait API
 */

#ifndef LR1121_DIO1_INTERRUPT_TEST_H
#define LR1121_DIO1_INTERRUPT_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Individual Test Functions
 ******************************************************************************/

/**
 * @brief Test 1: Configure LR1121 with IRQs routed to DIO1
 * 
 * Verifies:
 *   - HAL initialization includes DIO1 GPIO setup
 *   - SetDioIrqParams routes RX_DONE and TIMEOUT to DIO1
 *   - IRQ status can be read and cleared
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_irq_configuration(void);

/**
 * @brief Test 2: Enter RX mode and verify DIO1 fires on TIMEOUT
 * 
 * Verifies:
 *   - Radio enters RX mode successfully
 *   - DIO1 interrupt fires when RX times out
 *   - ISR callback is invoked
 *   - IRQ status shows TIMEOUT bit set
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_rx_timeout_interrupt(void);

/**
 * @brief Test 3: Verify osThreadFlagsWait() is woken by DIO1 ISR
 * 
 * Verifies:
 *   - Task can block on osThreadFlagsWait()
 *   - ISR successfully calls osThreadFlagsSet()
 *   - Task wakes immediately when DIO1 fires
 *   - This is the core mechanism for interrupt-driven ELRS RX
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_thread_flag_signaling(void);

/**
 * @brief Test 4: Verify multiple consecutive interrupts work correctly
 * 
 * Verifies:
 *   - Interrupt system handles repeated IRQs
 *   - No missed interrupts
 *   - IRQ count increments correctly
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_multiple_interrupts(void);

/**
 * @brief Test 5: Verify DIO1 pin state correlates with IRQ status
 * 
 * Verifies:
 *   - DIO1 LOW when no IRQ pending
 *   - DIO1 HIGH when IRQ pending
 *   - DIO1 returns LOW after ClearIrq
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_pin_state_correlation(void);

/*******************************************************************************
 * Test Suite Entry Points
 ******************************************************************************/

/**
 * @brief Run all DIO1 interrupt tests
 * 
 * Runs the complete test suite:
 *   1. IRQ Configuration
 *   2. RX Timeout Interrupt
 *   3. Thread Flag Signaling
 *   4. Multiple Interrupts
 *   5. Pin State Correlation
 * 
 * Prints detailed results for each test and overall summary.
 */
void lr1121_dio1_interrupt_run_all_tests(void);

/**
 * @brief Quick verification test
 * 
 * Runs only Test 1 (IRQ Configuration) and Test 2 (RX Timeout Interrupt)
 * for fast verification that the interrupt system is working.
 * 
 * Use this for quick sanity checks during development.
 */
void lr1121_dio1_quick_test(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_DIO1_INTERRUPT_TEST_H */
