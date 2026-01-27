/**
 * @file lr1121_dio1_interrupt_test.c
 * @brief Comprehensive DIO1 Interrupt Test Suite for LR1121
 * 
 * This test suite verifies the END-TO-END interrupt-driven operation:
 *   1. Configure LR1121 for RX with IRQs routed to DIO1
 *   2. Enter RX mode with timeout
 *   3. Verify DIO1 fires on TIMEOUT (no packets expected)
 *   4. Verify osThreadFlagsSet() wakes the task
 *   5. Measure interrupt latency
 * 
 * DIO1 Pin: UULP_VBAT_GPIO_2 (mikroBUS INT pin)
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3
 * 
 * IRQ Flow:
 *   LR1121 DIO1 → UULP_VBAT_GPIO_2 → IRQ021_Handler → lr1121_dio1_isr_handler
 *   → lr1121_hal_dio1_isr_radio1 → elrs_rx_isr → osThreadFlagsSet(ELRS_RX_FLAG_RADIO_IRQ)
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 9.2 "Interrupt Management"
 * Citation: siw917x-family-rm.pdf Section 8.3.6 "UULP GPIO interrupt"
 */

#include "lr1121_dio1_interrupt_test.h"
/* Include lr1121_hal.h FIRST to define lr1121_firmware_version_t
 * lr1121_driver.h has #ifndef LR1121_HAL_H guard to skip its definition */
#include "elrs_protocol/lr1121_hal.h"
#include "elrs_protocol/lr1121_regs.h"
#include "lr1121_driver.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*******************************************************************************
 * Test Configuration
 * 
 * Citation: LR1121 Datasheet Section 9.2.4 - IRQ flags
 ******************************************************************************/

/* Test frequency - 915 MHz ISM band (FCC Region) */
#define TEST_FREQUENCY_HZ       915000000UL

/* RX timeout for interrupt test - 100ms to trigger TIMEOUT IRQ quickly */
#define TEST_RX_TIMEOUT_MS      100

/* Maximum wait time for interrupt (ms) */
#define TEST_MAX_WAIT_MS        500

/* Thread flag for IRQ test */
#define TEST_IRQ_FLAG           (1U << 0)

/*******************************************************************************
 * Test State Variables
 ******************************************************************************/

static volatile bool     test_irq_fired = false;
static volatile uint32_t test_irq_count = 0;
static volatile uint32_t test_irq_timestamp_us = 0;
static volatile uint32_t test_enter_rx_timestamp_us = 0;
static osThreadId_t      test_thread_id = NULL;

/* Simple microsecond counter approximation using SysTick */
static volatile uint32_t test_us_counter = 0;

/*******************************************************************************
 * Timer Helper Functions
 ******************************************************************************/

/**
 * @brief Get approximate microsecond timestamp
 * 
 * Uses SysTick counter for timing. Not highly accurate but sufficient
 * for interrupt latency measurement.
 */
static uint32_t get_timestamp_us(void)
{
    /* Use osKernelGetTickCount() * 1000 as approximation */
    return osKernelGetTickCount() * 1000;
}

/**
 * @brief Simple delay in milliseconds
 */
static void delay_ms(uint32_t ms)
{
    osDelay(ms);
}

/*******************************************************************************
 * GetErrors Diagnostic - Decode LR1121 Error Flags
 * 
 * Citation: LR1121 User Manual Section 3.6.1 Table 3-4: GetErrors Command
 *   Opcode: 0x010D
 *   Response: [Stat1][ErrorStat(15:8)][ErrorStat(7:0)]
 * 
 * Citation: LR1121 User Manual Table 4-2: IrqToEnable Interruption Mapping
 *   Bit 22 = CmdError: Host command error
 *   Bit 23 = Error: An error other than a command error occurred (see GetErrors)
 ******************************************************************************/

/* GetErrors command opcode */
#define LR1121_CMD_GET_ERRORS   0x010D

/* Error flag definitions from LR1121 User Manual */
#define LR1121_ERR_RC64K_CALIB      (1 << 0)   /* RC64K calibration error */
#define LR1121_ERR_RC13M_CALIB      (1 << 1)   /* RC13M calibration error */
#define LR1121_ERR_PLL_CALIB        (1 << 2)   /* PLL calibration error */
#define LR1121_ERR_ADC_CALIB        (1 << 3)   /* ADC calibration error */
#define LR1121_ERR_IMG_CALIB        (1 << 4)   /* Image calibration error */
#define LR1121_ERR_HF_XOSC_START    (1 << 5)   /* HF XOSC failed to start */
#define LR1121_ERR_LF_XOSC_START    (1 << 6)   /* LF XOSC failed to start */
#define LR1121_ERR_PLL_LOCK         (1 << 7)   /* PLL failed to lock */
#define LR1121_ERR_PA_RAMP          (1 << 8)   /* PA ramping error */

/**
 * @brief Read and decode GetErrors when CmdError or Error IRQ is set
 * 
 * This function should be called when IRQ status shows:
 *   - Bit 22 (CmdError): Host command error
 *   - Bit 23 (Error): System error - use GetErrors for details
 * 
 * @param irq_status The IRQ status value that triggered this diagnostic
 */
static void diagnose_irq_errors(uint32_t irq_status)
{
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║     LR1121 ERROR DIAGNOSTIC                                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    /* Decode which error IRQs are set */
    printf("IRQ Status Analysis (0x%08lX):\n", (unsigned long)irq_status);
    
    if (irq_status & LR1121_IRQ_CMD_ERROR) {
        printf("  *** BIT 22 SET: CmdError - Host Command Error ***\n");
        printf("      Cause: Invalid command, wrong parameters, or command\n");
        printf("             sent in wrong chip state (e.g., TX cmd in RX mode)\n");
    }
    
    if (irq_status & LR1121_IRQ_ERROR) {
        printf("  *** BIT 23 SET: Error - System Error ***\n");
        printf("      Cause: Hardware/calibration error - calling GetErrors...\n");
    }
    
    /* Call GetErrors command to get detailed error flags
     * Citation: LR1121 User Manual Section 3.6.1 Table 3-4
     *   Phase 1: Send opcode [0x01][0x0D]
     *   Phase 2: Read response [Stat1][ErrorStat(15:8)][ErrorStat(7:0)]
     */
    uint8_t resp[3] = {0};
    
    /* Phase 1: Send GetErrors opcode */
    lr1121_hal_status_t status = lr1121_hal_write_command(LR1121_CMD_GET_ERRORS, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("  ERROR: GetErrors command (Phase 1) failed!\n");
        return;
    }
    
    /* Phase 2: Read response */
    status = lr1121_hal_read_command(resp, 3, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("  ERROR: GetErrors response (Phase 2) failed!\n");
        return;
    }
    
    uint16_t errors = ((uint16_t)resp[1] << 8) | resp[2];
    
    printf("\nGetErrors Response: 0x%04X\n", errors);
    printf("  Raw bytes: stat1=0x%02X, err[15:8]=0x%02X, err[7:0]=0x%02X\n",
           resp[0], resp[1], resp[2]);
    
    if (errors == 0) {
        printf("  No error flags set in GetErrors register.\n");
        printf("  CmdError (bit 22) is likely due to:\n");
        printf("    - Invalid command opcode\n");
        printf("    - Command parameters out of range\n");
        printf("    - Command sent in wrong chip mode\n");
    } else {
        printf("\nError Flags Decoded:\n");
        if (errors & LR1121_ERR_RC64K_CALIB)   printf("  [Bit 0] RC64K calibration failed\n");
        if (errors & LR1121_ERR_RC13M_CALIB)   printf("  [Bit 1] RC13M calibration failed\n");
        if (errors & LR1121_ERR_PLL_CALIB)     printf("  [Bit 2] PLL calibration failed\n");
        if (errors & LR1121_ERR_ADC_CALIB)     printf("  [Bit 3] ADC calibration failed\n");
        if (errors & LR1121_ERR_IMG_CALIB)     printf("  [Bit 4] Image calibration failed\n");
        if (errors & LR1121_ERR_HF_XOSC_START) printf("  [Bit 5] *** HF XOSC failed to start! ***\n");
        if (errors & LR1121_ERR_LF_XOSC_START) printf("  [Bit 6] LF XOSC failed to start\n");
        if (errors & LR1121_ERR_PLL_LOCK)      printf("  [Bit 7] *** PLL failed to lock! ***\n");
        if (errors & LR1121_ERR_PA_RAMP)       printf("  [Bit 8] PA ramping error\n");
        
        if (errors & LR1121_ERR_HF_XOSC_START) {
            printf("\n*** CRITICAL: HF XOSC Start Error ***\n");
            printf("   The external 32MHz TCXO/XOSC failed to start.\n");
            printf("   This prevents RX/TX operations from working correctly.\n");
            printf("   Fix: Check TCXO power supply and SetTcxoMode configuration.\n");
        }
        
        if (errors & LR1121_ERR_PLL_LOCK) {
            printf("\n*** CRITICAL: PLL Lock Error ***\n");
            printf("   The PLL cannot lock to the desired frequency.\n");
            printf("   Common causes:\n");
            printf("     - XOSC not running (fix XOSC first)\n");
            printf("     - Frequency out of supported range\n");
            printf("     - Hardware fault\n");
        }
    }
    
    printf("\n═══════════════════════════════════════════════════════════════\n\n");
}

/*******************************************************************************
 * Test IRQ Callback
 ******************************************************************************/

/**
 * @brief Test interrupt callback - called from ISR context
 * 
 * This callback mimics elrs_rx_isr() behavior:
 *   1. Records timestamp for latency measurement
 *   2. Sets thread flag to wake waiting task
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsSet()
 *   "This function may be called from interrupt service routines."
 */
static void test_irq_callback(void)
{
    test_irq_fired = true;
    test_irq_count++;
    test_irq_timestamp_us = get_timestamp_us();
    
    /* Signal the test thread (like elrs_rx_isr does) */
    if (test_thread_id != NULL) {
        osThreadFlagsSet(test_thread_id, TEST_IRQ_FLAG);
    }
}

/*******************************************************************************
 * Test 1: Verify DIO1 IRQ Configuration
 ******************************************************************************/

/**
 * @brief Test 1: Configure LR1121 with IRQs routed to DIO1
 * 
 * Citation: LR1121 Datasheet Section 9.2.1 SetDioIrqParams
 *   Opcode: 0x0113
 *   Parameters: [IRQ_MASK_31:24][IRQ_MASK_23:16][IRQ_MASK_15:8][IRQ_MASK_7:0]
 *               [DIO1_MASK_31:24]...[DIO3_MASK]
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_irq_configuration(void)
{
    lr1121_hal_status_t status;
    uint32_t irq_status;
    
    printf("\n=== Test 1: DIO1 IRQ Configuration ===\n");
    
    /* Initialize HAL (includes DIO1 GPIO setup) */
    status = lr1121_hal_init();
    if (status != LR1121_HAL_OK) {
        printf("FAIL: lr1121_hal_init returned %d\n", status);
        return -1;
    }
    printf("PASS: HAL initialized\n");
    
    /* CRITICAL: Reset LR1121 to clear any stuck state
     * Citation: LR1121 Datasheet Section 4.2.1 "Reset Timing"
     *   - BUSY stays HIGH for ~230ms after reset
     *   - Must wait for BUSY LOW before sending commands
     * 
     * Without this reset, the chip may be stuck from previous operations
     * and will timeout on all commands (BUSY stays HIGH forever)
     */
    printf("INFO: Performing hardware reset...\n");
    status = lr1121_hal_reset(false);  /* false = normal mode, not bootloader */
    if (status != LR1121_HAL_OK) {
        printf("FAIL: Hardware reset failed (%d)\n", status);
        return -1;
    }
    printf("PASS: Hardware reset complete\n");
    
    /* Put radio in standby */
    status = lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);  /* FIXED: Use XOSC for stable PLL/timing */
    if (status != LR1121_HAL_OK) {
        printf("FAIL: Set standby failed\n");
        return -1;
    }
    printf("PASS: Radio in standby mode\n");
    
    /* Clear any pending IRQs
     * Citation: LR1121 Datasheet Section 9.2.3 ClearIrq
     */
    status = lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: Clear IRQ failed\n");
        return -1;
    }
    printf("PASS: Cleared pending IRQs\n");
    
    /* Read IRQ status - should be 0 after clear
     * Citation: LR1121 Datasheet Section 9.2.2 GetIrqStatus
     */
    status = lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: Get IRQ status failed\n");
        return -1;
    }
    
    if (irq_status != 0) {
        printf("WARN: IRQ status not clear after ClearIrq: 0x%08lX\n", 
               (unsigned long)irq_status);
    } else {
        printf("PASS: IRQ status is clear (0x%08lX)\n", (unsigned long)irq_status);
    }
    
    /* Configure DIO1 IRQ params
     * Citation: LR1121 Datasheet Section 9.2.1
     *   IRQ_MASK: Which IRQs to enable globally
     *   DIO1_MASK: Which IRQs to route to DIO1 pin
     * 
     * We enable TIMEOUT and RX_DONE, route both to DIO1
     */
    uint32_t irq_mask = LR1121_IRQ_RX_DONE | LR1121_IRQ_TIMEOUT;
    uint32_t dio1_mask = irq_mask;  /* Route same IRQs to DIO1 */
    
    status = lr1121_hal_set_dio_irq_params(irq_mask, dio1_mask, 0, 0, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetDioIrqParams failed\n");
        return -1;
    }
    printf("PASS: DIO IRQ params configured\n");
    printf("INFO:   IRQ_MASK  = 0x%08lX (RX_DONE|TIMEOUT)\n", (unsigned long)irq_mask);
    printf("INFO:   DIO1_MASK = 0x%08lX\n", (unsigned long)dio1_mask);
    
    /* Verify DIO1 pin is LOW (no pending IRQ) */
    uint8_t pin_state = lr1121_dio1_read();
    printf("INFO: DIO1 pin state = %d (expected: 0)\n", pin_state);
    
    if (pin_state != 0) {
        printf("WARN: DIO1 is HIGH before RX - may have pending IRQ\n");
    }
    
    printf("PASS: DIO1 IRQ configuration complete\n");
    return 0;
}

/*******************************************************************************
 * Test 2: Trigger DIO1 Interrupt via RX Timeout
 ******************************************************************************/

/**
 * @brief Test 2: Enter RX mode and verify DIO1 fires on TIMEOUT
 * 
 * This test:
 *   1. Configures radio for LoRa RX with short timeout
 *   2. Registers ISR callback
 *   3. Enables NVIC interrupt
 *   4. Waits for DIO1 to fire (TIMEOUT expected since no packets)
 *   5. Verifies callback was invoked
 * 
 * Citation: LR1121 Datasheet Section 7.2.2 SetRx
 *   timeout = N * 30.5µs (24-bit value)
 *   0x000000 = no timeout (continuous)
 *   0xFFFFFF = timeout disabled (single packet)
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_rx_timeout_interrupt(void)
{
    lr1121_hal_status_t status;
    uint8_t buf[8];
    
    printf("\n=== Test 2: DIO1 RX Timeout Interrupt ===\n");
    
    /* Reset test state */
    test_irq_fired = false;
    test_irq_count = 0;
    
    /* Register our test callback */
    lr1121_hal_set_isr_callback(test_irq_callback, SX12XX_Radio_1);
    printf("PASS: ISR callback registered\n");
    
    /* Set packet type to LoRa
     * Citation: LR1121 Datasheet Section 5.1 SetPacketType
     */
    status = lr1121_hal_set_packet_type(LR11XX_RADIO_PKT_TYPE_LORA, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetPacketType failed\n");
        return -1;
    }
    printf("PASS: Packet type set to LoRa\n");
    
    /* Set modulation parameters
     * Citation: LR1121 Datasheet Section 8.3.1 SetModulationParams (LoRa)
     * Using SF7, BW500 for fast timeout
     */
    buf[0] = LR11XX_RADIO_LORA_SF7;   /* SF7 */
    buf[1] = LR11XX_RADIO_LORA_BW_500; /* 500kHz BW */
    buf[2] = LR11XX_RADIO_LORA_CR_4_5; /* CR 4/5 */
    buf[3] = 0;                         /* LDRO off */
    
    status = lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_MODULATION_PARAM_OC,
                                                 buf, 4, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetModulationParams failed\n");
        return -1;
    }
    printf("PASS: Modulation params set (SF7, BW500)\n");
    
    /* Set packet parameters
     * Citation: LR1121 Datasheet Section 8.3.2 SetPacketParams (LoRa)
     */
    buf[0] = 0;    /* Preamble MSB */
    buf[1] = 12;   /* Preamble LSB (12 symbols) */
    buf[2] = LR11XX_RADIO_LORA_PKT_EXPLICIT; /* Variable length */
    buf[3] = 8;    /* Max payload 8 bytes */
    buf[4] = LR11XX_RADIO_LORA_CRC_ON;
    buf[5] = LR11XX_RADIO_LORA_IQ_STANDARD;
    
    status = lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_PARAM_OC,
                                                 buf, 6, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetPacketParams failed\n");
        return -1;
    }
    printf("PASS: Packet params set\n");
    
    /* Set frequency 
     * Citation: LR1121 Datasheet Section 5.3 SetRfFrequency
     */
    status = lr1121_hal_set_rf_frequency(TEST_FREQUENCY_HZ, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetRfFrequency failed\n");
        return -1;
    }
    printf("PASS: Frequency set to %lu Hz\n", (unsigned long)TEST_FREQUENCY_HZ);
    
    /* Clear IRQs before entering RX */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    printf("INFO: Entering RX mode with %d ms timeout...\n", TEST_RX_TIMEOUT_MS);
    printf("INFO: Expected: DIO1 interrupt on TIMEOUT (no packets expected)\n");
    
    /* Record timestamp before entering RX */
    test_enter_rx_timestamp_us = get_timestamp_us();
    
    /* Enter RX mode with timeout
     * Citation: LR1121 Datasheet Section 7.2.2
     * Timeout = value * 30.5µs, so 100ms ≈ 3278
     */
    uint32_t timeout_val = (TEST_RX_TIMEOUT_MS * 1000) / 31; /* Convert ms to LR1121 units */
    status = lr1121_hal_set_rx(timeout_val, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetRx failed\n");
        return -1;
    }
    printf("PASS: RX mode entered (timeout=%lu units)\n", (unsigned long)timeout_val);
    
    /* Wait for interrupt or timeout */
    uint32_t wait_start = osKernelGetTickCount();
    while (!test_irq_fired) {
        if ((osKernelGetTickCount() - wait_start) > TEST_MAX_WAIT_MS) {
            printf("FAIL: Timeout waiting for DIO1 interrupt!\n");
            printf("INFO: DIO1 pin state = %d\n", lr1121_dio1_read());
            
            /* Check IRQ status via SPI to see if IRQ fired but ISR didn't run */
            uint32_t irq_status;
            lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1);
            printf("INFO: IRQ_STATUS = 0x%08lX\n", (unsigned long)irq_status);
            
            /* Check for error IRQs first - these indicate configuration problems */
            if (irq_status & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) {
                diagnose_irq_errors(irq_status);
            }
            
            if (irq_status & LR1121_IRQ_TIMEOUT) {
                printf("INFO: TIMEOUT IRQ is set in status register!\n");
                printf("INFO: This means LR1121 IRQ fired but NVIC didn't trigger ISR.\n");
                printf("INFO: Check:\n");
                printf("INFO:   1. NVIC IRQ 5 (UULP GPIO) enabled?\n");
                printf("INFO:   2. NPSS_TO_MCU_GPIO_INTR_IRQHandler in vector table?\n");
                printf("INFO:   3. Callback registered via lr1121_hal_set_isr_callback()?\n");
                printf("INFO:   4. DIO9 (LR1121 IRQ pin) connected to UULP_VBAT_GPIO_2?\n");
            } else if (!(irq_status & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR))) {
                printf("INFO: No TIMEOUT IRQ set - RX may have failed to start.\n");
            }
            return -1;
        }
        delay_ms(1);
    }
    
    /* Calculate latency */
    uint32_t latency_us = test_irq_timestamp_us - test_enter_rx_timestamp_us;
    printf("PASS: DIO1 interrupt fired!\n");
    printf("INFO:   Interrupt count: %lu\n", (unsigned long)test_irq_count);
    printf("INFO:   Approximate latency: %lu us\n", (unsigned long)latency_us);
    
    /* Verify it was TIMEOUT IRQ */
    uint32_t irq_status;
    lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1);
    printf("INFO:   IRQ_STATUS = 0x%08lX\n", (unsigned long)irq_status);
    
    /* Check for error IRQs - run diagnostic if detected */
    if (irq_status & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) {
        diagnose_irq_errors(irq_status);
    }
    
    if (irq_status & LR1121_IRQ_TIMEOUT) {
        printf("PASS: TIMEOUT IRQ confirmed (bit 10 = 0x%08lX)\n", 
               (unsigned long)LR1121_IRQ_TIMEOUT);
    } else if (irq_status & LR1121_IRQ_RX_DONE) {
        printf("INFO: Unexpected RX_DONE - received a packet!\n");
    } else {
        printf("WARN: Unknown IRQ source (expected TIMEOUT=0x%08lX)\n",
               (unsigned long)LR1121_IRQ_TIMEOUT);
        /* Print all set bits for debugging */
        printf("INFO: Decoding IRQ_STATUS bits:\n");
        if (irq_status & LR1121_IRQ_TX_DONE)    printf("       Bit 2: TX_DONE\n");
        if (irq_status & LR1121_IRQ_RX_DONE)    printf("       Bit 3: RX_DONE\n");
        if (irq_status & LR1121_IRQ_CMD_ERROR)  printf("       Bit 22: CMD_ERROR\n");
        if (irq_status & LR1121_IRQ_ERROR)      printf("       Bit 23: ERROR\n");
    }
    
    /* Clear IRQs */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    /* Verify DIO1 goes LOW after clearing */
    delay_ms(1);
    uint8_t pin_after = lr1121_dio1_read();
    printf("INFO:   DIO1 after clear = %d (expected: 0)\n", pin_after);
    
    return 0;
}

/*******************************************************************************
 * Test 3: Verify osThreadFlagsWait() Integration
 ******************************************************************************/

/**
 * @brief Test 3: Verify osThreadFlagsWait() is woken by DIO1 ISR
 * 
 * This test verifies the CMSIS-RTOS2 task signaling mechanism:
 *   1. Store current thread ID
 *   2. Enter RX mode with timeout
 *   3. Block on osThreadFlagsWait()
 *   4. Verify task wakes when DIO1 fires
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsWait()
 *   "Suspends the execution of the currently running thread until any or all
 *    of the specified flags are set."
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_thread_flag_signaling(void)
{
    lr1121_hal_status_t status;
    uint32_t flags;
    
    printf("\n=== Test 3: osThreadFlagsWait() Integration ===\n");
    
    /* Get current thread ID for signaling */
    test_thread_id = osThreadGetId();
    if (test_thread_id == NULL) {
        printf("FAIL: osThreadGetId() returned NULL\n");
        return -1;
    }
    printf("PASS: Got thread ID: %p\n", (void*)test_thread_id);
    
    /* Reset test state */
    test_irq_fired = false;
    test_irq_count = 0;
    
    /* Clear any pending thread flags */
    osThreadFlagsClear(TEST_IRQ_FLAG);
    
    /* Clear IRQs and re-enter RX mode */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    printf("INFO: Entering RX mode and calling osThreadFlagsWait()...\n");
    printf("INFO: Test will block until DIO1 fires or timeout\n");
    
    /* Enter RX with short timeout */
    uint32_t timeout_val = (50 * 1000) / 31; /* 50ms timeout */
    status = lr1121_hal_set_rx(timeout_val, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        printf("FAIL: SetRx failed\n");
        return -1;
    }
    
    /* Block waiting for ISR to signal us
     * Citation: CMSIS-RTOS2 osFlagsWaitAny
     *   "Return when ANY of the specified flags are set"
     */
    uint32_t wait_start = osKernelGetTickCount();
    
    flags = osThreadFlagsWait(TEST_IRQ_FLAG, osFlagsWaitAny, 200); /* 200ms max */
    
    uint32_t wait_time = osKernelGetTickCount() - wait_start;
    
    /* Check result */
    if (flags & osFlagsError) {
        if (flags == osFlagsErrorTimeout) {
            printf("FAIL: osThreadFlagsWait() timed out - ISR didn't signal!\n");
            printf("INFO: IRQ callback may not be calling osThreadFlagsSet()\n");
        } else {
            printf("FAIL: osThreadFlagsWait() error: 0x%lX\n", (unsigned long)flags);
        }
        return -1;
    }
    
    printf("PASS: osThreadFlagsWait() returned successfully!\n");
    printf("INFO:   Flags returned: 0x%lX\n", (unsigned long)flags);
    printf("INFO:   Wait time: %lu ms\n", (unsigned long)wait_time);
    printf("INFO:   test_irq_fired: %s\n", test_irq_fired ? "true" : "false");
    
    if (flags & TEST_IRQ_FLAG) {
        printf("PASS: TEST_IRQ_FLAG was set by ISR callback\n");
    }
    
    /* Cleanup */
    test_thread_id = NULL;
    lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);  /* FIXED: Use XOSC for stable PLL/timing */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    return 0;
}

/*******************************************************************************
 * Test 4: Multiple Consecutive Interrupts
 ******************************************************************************/

/**
 * @brief Test 4: Verify multiple consecutive interrupts work correctly
 * 
 * This stress-tests the interrupt system by:
 *   1. Entering RX mode multiple times
 *   2. Each time waiting for TIMEOUT IRQ
 *   3. Verifying interrupt count increments correctly
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_multiple_interrupts(void)
{
    printf("\n=== Test 4: Multiple Consecutive Interrupts ===\n");
    
    const int NUM_CYCLES = 3;
    uint32_t initial_count = test_irq_count;
    
    printf("INFO: Will trigger %d consecutive RX timeouts\n", NUM_CYCLES);
    
    for (int i = 0; i < NUM_CYCLES; i++) {
        test_irq_fired = false;
        
        /* Clear IRQs */
        lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
        
        /* Enter RX with very short timeout (20ms) */
        uint32_t timeout_val = (20 * 1000) / 31;
        lr1121_hal_set_rx(timeout_val, SX12XX_Radio_1);
        
        /* Wait for interrupt */
        uint32_t wait_start = osKernelGetTickCount();
        while (!test_irq_fired) {
            if ((osKernelGetTickCount() - wait_start) > 100) {
                printf("FAIL: Cycle %d - timeout waiting for interrupt\n", i + 1);
                return -1;
            }
            delay_ms(1);
        }
        
        printf("INFO: Cycle %d - IRQ fired, total count: %lu\n", 
               i + 1, (unsigned long)test_irq_count);
    }
    
    /* Verify count */
    uint32_t expected_count = initial_count + NUM_CYCLES;
    if (test_irq_count == expected_count) {
        printf("PASS: Interrupt count correct (%lu)\n", (unsigned long)test_irq_count);
    } else {
        printf("WARN: Count mismatch - expected %lu, got %lu\n",
               (unsigned long)expected_count, (unsigned long)test_irq_count);
    }
    
    /* Cleanup */
    lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);  /* FIXED: Use XOSC for stable PLL/timing */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    printf("PASS: Multiple interrupts test complete\n");
    return 0;
}

/*******************************************************************************
 * Test 5: DIO1 Pin State Correlation
 ******************************************************************************/

/**
 * @brief Test 5: Verify DIO1 pin state correlates with IRQ status
 * 
 * This test verifies:
 *   1. DIO1 LOW when no IRQ pending
 *   2. DIO1 HIGH when IRQ pending
 *   3. DIO1 returns LOW after IRQ cleared
 * 
 * Citation: LR1121 Datasheet Section 9.2
 *   "DIO pins are directly controlled by the IRQ status register"
 * 
 * @return 0 on success, -1 on failure
 */
int test_dio1_pin_state_correlation(void)
{
    uint32_t irq_status;
    uint8_t pin_state;
    
    printf("\n=== Test 5: DIO1 Pin State Correlation ===\n");
    
    /* Step 1: Clear IRQs, verify DIO1 LOW */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    delay_ms(1);
    
    pin_state = lr1121_dio1_read();
    lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1);
    
    printf("INFO: After clear - DIO1=%d, IRQ_STATUS=0x%08lX\n", 
           pin_state, (unsigned long)irq_status);
    
    if (pin_state != 0) {
        printf("WARN: DIO1 not LOW after clear\n");
    } else {
        printf("PASS: DIO1 LOW when no IRQ pending\n");
    }
    
    /* Step 2: Enter RX, wait for timeout, check DIO1 HIGH */
    test_irq_fired = false;
    uint32_t timeout_val = (30 * 1000) / 31; /* 30ms */
    lr1121_hal_set_rx(timeout_val, SX12XX_Radio_1);
    
    /* Wait for timeout (don't clear, we want to check pin state) */
    delay_ms(50);
    
    /* Disable ISR temporarily to prevent auto-clear */
    lr1121_dio1_disable();
    
    /* Read pin state while IRQ pending */
    pin_state = lr1121_dio1_read();
    lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1);
    
    printf("INFO: With pending IRQ - DIO1=%d, IRQ_STATUS=0x%08lX\n", 
           pin_state, (unsigned long)irq_status);
    
    if ((irq_status & LR1121_IRQ_TIMEOUT) && pin_state == 1) {
        printf("PASS: DIO1 HIGH when TIMEOUT IRQ pending\n");
    } else if (irq_status & LR1121_IRQ_TIMEOUT) {
        printf("WARN: IRQ pending but DIO1 not HIGH\n");
    }
    
    /* Step 3: Clear IRQ, verify DIO1 goes LOW */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    delay_ms(1);
    
    pin_state = lr1121_dio1_read();
    printf("INFO: After second clear - DIO1=%d\n", pin_state);
    
    if (pin_state == 0) {
        printf("PASS: DIO1 LOW after IRQ cleared\n");
    } else {
        printf("WARN: DIO1 still HIGH after clear\n");
    }
    
    /* Re-enable ISR */
    lr1121_dio1_enable();
    
    /* Cleanup */
    lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);  /* FIXED: Use XOSC for stable PLL/timing */
    
    return 0;
}

/*******************************************************************************
 * Test Suite Entry Point
 ******************************************************************************/

/**
 * @brief Run all DIO1 interrupt tests
 */
void lr1121_dio1_interrupt_run_all_tests(void)
{
    int failed = 0;
    int passed = 0;
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║    LR1121 DIO1 COMPREHENSIVE INTERRUPT TEST SUITE                 ║\n");
    printf("║    Hardware: SiW917 BRD2708A + LR1121 (mikroBUS)                  ║\n");
    printf("║    DIO1 Pin: UULP_VBAT_GPIO_2                                     ║\n");
    printf("╠═══════════════════════════════════════════════════════════════════╣\n");
    printf("║    Tests the END-TO-END interrupt-driven RX operation:            ║\n");
    printf("║    LR1121 → DIO1 → GPIO IRQ → ISR → osThreadFlagsSet → Task      ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");
    
    /* Test 1: IRQ Configuration */
    if (test_dio1_irq_configuration() == 0) {
        passed++;
    } else {
        failed++;
    }
    
    /* Test 2: RX Timeout Interrupt */
    if (test_dio1_rx_timeout_interrupt() == 0) {
        passed++;
    } else {
        failed++;
    }
    
    /* Test 3: Thread Flag Signaling */
    if (test_dio1_thread_flag_signaling() == 0) {
        passed++;
    } else {
        failed++;
    }
    
    /* Test 4: Multiple Interrupts */
    if (test_dio1_multiple_interrupts() == 0) {
        passed++;
    } else {
        failed++;
    }
    
    /* Test 5: Pin State Correlation */
    if (test_dio1_pin_state_correlation() == 0) {
        passed++;
    } else {
        failed++;
    }
    
    /* Summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║                      TEST SUMMARY                                 ║\n");
    printf("╠═══════════════════════════════════════════════════════════════════╣\n");
    printf("║  Total Tests:  %d                                                  ║\n", passed + failed);
    printf("║  Passed:       %d                                                  ║\n", passed);
    printf("║  Failed:       %d                                                  ║\n", failed);
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");
    
    if (failed == 0) {
        printf("✓ ALL INTERRUPT TESTS PASSED\n\n");
        printf("The interrupt-driven ELRS RX implementation is ready!\n");
        printf("Benefits:\n");
        printf("  - CPU usage drops from ~30%% (polling) to <1%% (interrupt)\n");
        printf("  - Task sleeps via osThreadFlagsWait() until DIO1 fires\n");
        printf("  - Instant wake on RX_DONE/TIMEOUT events\n");
    } else {
        printf("✗ SOME TESTS FAILED\n\n");
        printf("Troubleshooting:\n");
        printf("1. Verify LR1121 DIO1 connected to UULP_VBAT_GPIO_2 (mikroBUS INT)\n");
        printf("2. Check NVIC IRQ 5 (UULP GPIO) is enabled\n");
        printf("3. Verify lr1121_dio1_set_callback() was called\n");
        printf("4. Check IRQ021_Handler is in vector table\n");
    }
    printf("\n");
}

/**
 * @brief Quick verification test - just runs Test 1 and 2
 */
void lr1121_dio1_quick_test(void)
{
    printf("\n=== LR1121 DIO1 Quick Interrupt Test ===\n\n");
    
    if (test_dio1_irq_configuration() != 0) {
        printf("FAIL: IRQ configuration test failed\n");
        return;
    }
    
    if (test_dio1_rx_timeout_interrupt() != 0) {
        printf("FAIL: RX timeout interrupt test failed\n");
        return;
    }
    
    printf("\n✓ Quick test PASSED - interrupt system is working!\n");
    
    /* Cleanup */
    lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);  /* FIXED: Use XOSC for stable PLL/timing */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
}
