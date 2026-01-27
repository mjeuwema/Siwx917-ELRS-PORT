/**
 * @file lr1121_hal.c
 * @brief LR1121 HAL Implementation for SiW917
 * 
 * This file implements the HAL bridge between ELRS protocol code and
 * the native SiW917 LR1121 driver (lr1121_driver.c).
 * 
 * Implementation Notes:
 * - Uses existing lr1121_driver.c for low-level SPI operations
 * - Single radio implementation (no Gemini dual-radio support yet)
 * - DIO1 interrupt support via EGPIO
 * 
 * Citation: ExpressLRS 4.0 LR1121_hal.cpp
 * Citation: siw917x-family-rm.pdf Section 11 "GPIO"
 */

#include "lr1121_hal.h"
#include "../lr1121_driver.h"
#include "rsi_debug.h"
#include "cmsis_os2.h"

#include <string.h>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * BUSY timeout in milliseconds
 * 
 * Citation: LR1121 Datasheet Section 3.1 "Command Interface Timing"
 *   - Most commands complete in < 1ms
 *   - SetPacketType can take up to 10ms during mode transitions
 *   - Calibration commands can take 100ms+
 *   - Firmware updates/image calibration can take significantly longer
 * 
 * Citation: User documentation on LR1121 timeouts:
 *   "Standard Value: 1000ms (1 second)"
 *   "While ELRS targets low latency, the hardware 'wait for busy' timeout
 *    is typically kept long (100ms to 1000ms) in the low-level driver to
 *    account for one-time operations like firmware updates or image calibration."
 *   "During normal packet cycling (TX/RX), BUSY typically stays high for
 *    only 10µs to 100µs."
 * 
 * FIX: INCREASED from 50ms to 1000ms (1 second) per standard practice.
 * This prevents BUSY timeout errors during rate cycling (SetPacketType 0x020E)
 * and allows for calibration/initialization commands that take longer.
 */
#define HAL_BUSY_TIMEOUT_MS     1000U

/**
 * Debug output enable
 */
#define HAL_DEBUG_ENABLE        1

#if HAL_DEBUG_ENABLE
    #define HAL_DBG(fmt, ...)   DEBUGOUT("[HAL] " fmt, ##__VA_ARGS__)
#else
    #define HAL_DBG(fmt, ...)   ((void)0)
#endif

/*******************************************************************************
 * Static Variables
 ******************************************************************************/

static bool hal_initialized = false;

/* ISR callbacks for DIO1 interrupts */
static lr1121_isr_callback_t isr_callback_1 = NULL;
static lr1121_isr_callback_t isr_callback_2 = NULL;

/*******************************************************************************
 * HAL Implementation
 ******************************************************************************/

lr1121_hal_status_t lr1121_hal_init(void)
{
    lr1121_status_t status;
    
    if (hal_initialized) {
        HAL_DBG("Already initialized\n");
        return LR1121_HAL_OK;
    }
    
    HAL_DBG("Initializing HAL...\n");
    
    /* Initialize the low-level driver */
    status = lr1121_init();
    if (status != LR1121_OK) {
        HAL_DBG("Driver init failed: %d\n", status);
        return LR1121_HAL_ERROR_SPI;
    }
    
    /* Configure DIO1 interrupt pin for hardware interrupt-driven packet handling
     * 
     * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
     *   DIO1 → INT pin → UULP_VBAT_GPIO_2
     * 
     * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 5.2 "DIO Pins"
     *   DIO1 goes HIGH when configured IRQs fire (RX_DONE, TX_DONE, TIMEOUT)
     * 
     * The interrupt is enabled later when elrs_rx_init() calls
     * lr1121_hal_set_isr_callback() to register the ELRS ISR handler.
     */
    lr1121_status_t dio1_status = lr1121_dio1_init();
    if (dio1_status != LR1121_OK) {
        HAL_DBG("WARNING: DIO1 interrupt init failed: %d (falling back to polling)\n", dio1_status);
        /* Non-fatal - we can still poll IRQ status */
    } else {
        HAL_DBG("DIO1 interrupt GPIO initialized\n");
    }
    
    hal_initialized = true;
    HAL_DBG("HAL initialized\n");
    
    return LR1121_HAL_OK;
}

void lr1121_hal_end(void)
{
    if (!hal_initialized) {
        return;
    }
    
    HAL_DBG("Deinitializing HAL...\n");
    
    /* Clear callbacks */
    isr_callback_1 = NULL;
    isr_callback_2 = NULL;
    
    /* Deinitialize low-level driver */
    lr1121_deinit();
    
    hal_initialized = false;
    HAL_DBG("HAL deinitialized\n");
}

lr1121_hal_status_t lr1121_hal_reset(bool bootloader)
{
    lr1121_status_t status;
    
    HAL_DBG("Reset (bootloader=%d)\n", bootloader);
    
    if (bootloader) {
        /* To enter bootloader mode:
         * 1. Hold BUSY LOW while resetting
         * 2. Wait for bootloader to initialize
         * 
         * Citation: LR1121 User Manual - Bootloader Mode Entry
         * 
         * NOTE: This requires BUSY pin to be configurable as output,
         * which we don't currently support. Log warning and proceed
         * with normal reset.
         */
        HAL_DBG("WARNING: Bootloader mode not fully supported\n");
    }
    
    /* Perform hardware reset */
    status = lr1121_reset();
    if (status != LR1121_OK) {
        HAL_DBG("Reset failed: %d\n", status);
        return LR1121_HAL_ERROR_BUSY_TIMEOUT;
    }
    
    /* Wait for chip to be ready */
    if (!lr1121_hal_wait_on_busy(SX12XX_Radio_All)) {
        HAL_DBG("Post-reset BUSY timeout\n");
        return LR1121_HAL_ERROR_BUSY_TIMEOUT;
    }
    
    return LR1121_HAL_OK;
}

lr1121_hal_status_t lr1121_hal_wakeup(void)
{
    HAL_DBG("========== WAKEUP START ==========\n");
    HAL_DBG("Wakeup: Toggling NSS to wake chip from SLEEP\n");
    
    /* Toggle NSS: LOW for 1ms (generates falling edge), then HIGH
     * Citation: Waveshare uses 1ms, datasheet doesn't specify minimum */
    HAL_DBG("Wakeup: Asserting CS (NSS LOW)...\n");
    lr1121_cs_assert();    /* NSS LOW - falling edge wakes chip */
    osDelay(1);            /* 1ms hold time */
    HAL_DBG("Wakeup: Deasserting CS (NSS HIGH)...\n");
    lr1121_cs_deassert();  /* NSS HIGH - return to idle state */
    
    /* Wait for chip to complete wakeup sequence
     * Citation: LR1121 datasheet - wakeup time is typically a few ms */
    HAL_DBG("Wakeup: Waiting 5ms for chip to wake...\n");
    osDelay(5);  /* 5ms for wakeup + TCXO stabilization margin */
    
    /* Optionally wait for BUSY to go LOW (chip ready in STDBY_RC)
     * Note: If chip was already awake, this will return immediately */
    HAL_DBG("Wakeup: Checking BUSY...\n");
    if (!lr1121_hal_wait_on_busy(SX12XX_Radio_All)) {
        HAL_DBG("Wakeup: *** BUSY TIMEOUT *** (chip may not have woken up!)\n");
        /* Don't return error - chip might have already been awake */
    } else {
        HAL_DBG("Wakeup: BUSY cleared ✓\n");
    }
    
    HAL_DBG("Wakeup: Complete - chip should be in STDBY_RC\n");
    HAL_DBG("========== WAKEUP END ==========\n");
    return LR1121_HAL_OK;
}

lr1121_hal_status_t lr1121_hal_write_command(uint16_t opcode, 
                                              SX12XX_Radio_Number_t radioNumber)
{
    (void)radioNumber;  /* Single radio - ignore selector */
    
    if (!hal_initialized) {
        return LR1121_HAL_ERROR_NOT_INITIALIZED;
    }
    
    /* Wait for BUSY */
    if (!lr1121_hal_wait_on_busy(radioNumber)) {
        HAL_DBG("WriteCmd: BUSY timeout (opcode=0x%04X)\n", opcode);
        return LR1121_HAL_ERROR_BUSY_TIMEOUT;
    }
    
    /* Send command using low-level driver */
    if (!lr1121_send_command(opcode, NULL, 0)) {
        HAL_DBG("WriteCmd: SPI error (opcode=0x%04X)\n", opcode);
        return LR1121_HAL_ERROR_SPI;
    }
    
    return LR1121_HAL_OK;
}

lr1121_hal_status_t lr1121_hal_write_command_with_data(uint16_t opcode, 
                                                        const uint8_t *buffer, 
                                                        uint8_t size, 
                                                        SX12XX_Radio_Number_t radioNumber)
{
    (void)radioNumber;  /* Single radio - ignore selector */
    
    if (!hal_initialized) {
        return LR1121_HAL_ERROR_NOT_INITIALIZED;
    }
    
    if (buffer == NULL && size > 0) {
        return LR1121_HAL_ERROR_INVALID_PARAM;
    }
    
    /* Wait for BUSY */
    if (!lr1121_hal_wait_on_busy(radioNumber)) {
        HAL_DBG("WriteCmdData: BUSY timeout (opcode=0x%04X)\n", opcode);
        return LR1121_HAL_ERROR_BUSY_TIMEOUT;
    }
    
    /* Send command with parameters using low-level driver */
    if (!lr1121_send_command(opcode, buffer, size)) {
        HAL_DBG("WriteCmdData: SPI error (opcode=0x%04X)\n", opcode);
        return LR1121_HAL_ERROR_SPI;
    }
    
    return LR1121_HAL_OK;
}

lr1121_hal_status_t lr1121_hal_read_command(uint8_t *buffer, 
                                             uint8_t size, 
                                             SX12XX_Radio_Number_t radioNumber)
{
    (void)radioNumber;  /* Single radio - ignore selector */
    
    if (!hal_initialized) {
        return LR1121_HAL_ERROR_NOT_INITIALIZED;
    }
    
    if (buffer == NULL || size == 0) {
        return LR1121_HAL_ERROR_INVALID_PARAM;
    }
    
    /* Wait for BUSY */
    if (!lr1121_hal_wait_on_busy(radioNumber)) {
        HAL_DBG("ReadCmd: BUSY timeout\n");
        return LR1121_HAL_ERROR_BUSY_TIMEOUT;
    }
    
    /* Read response using low-level driver */
    if (!lr1121_read_response(buffer, size)) {
        HAL_DBG("ReadCmd: SPI error\n");
        return LR1121_HAL_ERROR_SPI;
    }
    
    return LR1121_HAL_OK;
}

bool lr1121_hal_wait_on_busy(SX12XX_Radio_Number_t radioNumber)
{
    (void)radioNumber;  /* Single radio - ignore selector */
    
    /* Use the driver's wait function with configurable timeout
     * Timeout is in milliseconds - increased to 50ms for mode transitions */
    return lr1121_wait_busy_timeout(HAL_BUSY_TIMEOUT_MS);
}

void lr1121_hal_set_isr_callback(lr1121_isr_callback_t callback, 
                                  SX12XX_Radio_Number_t radioNumber)
{
    if (radioNumber & SX12XX_Radio_1) {
        isr_callback_1 = callback;
        HAL_DBG("ISR callback 1 %s\n", callback ? "set" : "cleared");
        
        /* Enable/disable DIO1 GPIO interrupt based on callback presence
         * 
         * Citation: lr1121_driver.h - DIO1 interrupt API
         *   lr1121_dio1_set_callback() - Register the callback in driver
         *   lr1121_dio1_enable() - Enable NVIC interrupt
         *   lr1121_dio1_disable() - Disable NVIC interrupt
         * 
         * The callback chain is:
         *   NVIC → NPSS_TO_MCU_GPIO_INTR_IRQHandler()
         *        → lr1121_dio1_isr_handler()
         *        → lr1121_hal_dio1_isr_radio1()
         *        → isr_callback_1() [the ELRS handler: elrs_rx_isr()]
         */
        if (callback != NULL) {
            /* Register our HAL ISR as the DIO1 callback */
            lr1121_dio1_set_callback(lr1121_hal_dio1_isr_radio1);
            /* Enable DIO1 interrupt in NVIC */
            lr1121_dio1_enable();
            HAL_DBG("DIO1 interrupt ENABLED for Radio 1\n");
        } else {
            /* Disable DIO1 interrupt */
            lr1121_dio1_disable();
            lr1121_dio1_set_callback(NULL);
            HAL_DBG("DIO1 interrupt DISABLED for Radio 1\n");
        }
    }
    
    if (radioNumber & SX12XX_Radio_2) {
        isr_callback_2 = callback;
        HAL_DBG("ISR callback 2 %s\n", callback ? "set" : "cleared");
        /* Note: Radio 2 DIO1 not implemented - would need second GPIO pin */
    }
}

lr1121_hal_status_t lr1121_hal_get_version(lr1121_firmware_version_t *version,
                                            SX12XX_Radio_Number_t radioNumber)
{
    lr1121_version_t drv_version;
    lr1121_status_t status;
    
    (void)radioNumber;  /* Single radio */
    
    if (version == NULL) {
        return LR1121_HAL_ERROR_INVALID_PARAM;
    }
    
    /* Use existing driver function */
    status = lr1121_get_version(&drv_version);
    if (status != LR1121_OK) {
        HAL_DBG("GetVersion failed: %d\n", status);
        return LR1121_HAL_ERROR_SPI;
    }
    
    /* Copy to ELRS-compatible structure */
    version->hardware = drv_version.hardware;
    version->type = drv_version.type;
    version->version = drv_version.version;
    
    HAL_DBG("Version: HW=%02X Type=%02X FW=%04X\n", 
            version->hardware, version->type, version->version);
    
    return LR1121_HAL_OK;
}

/*******************************************************************************
 * DIO1 Interrupt Handler (to be connected to GPIO interrupt)
 * 
 * TODO: Connect this to actual GPIO interrupt when interrupt support is added
 ******************************************************************************/

/**
 * @brief DIO1 interrupt handler for Radio 1
 * 
 * Call this from the GPIO ISR when DIO1 rising edge is detected.
 */
void lr1121_hal_dio1_isr_radio1(void)
{
    if (isr_callback_1 != NULL) {
        isr_callback_1();
    }
}

/**
 * @brief DIO1 interrupt handler for Radio 2
 */
void lr1121_hal_dio1_isr_radio2(void)
{
    if (isr_callback_2 != NULL) {
        isr_callback_2();
    }
}

/*******************************************************************************
 * Extended HAL Functions for Radio Configuration
 * 
 * These functions provide higher-level access to commonly used commands.
 ******************************************************************************/

/**
 * @brief Set LR1121 to standby mode
 * 
 * @param mode LR1121_MODE_STDBY_RC or LR1121_MODE_STDBY_XOSC
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_standby(lr11xx_RadioOperatingModes_t mode,
                                            SX12XX_Radio_Number_t radioNumber)
{
    uint8_t standby_cfg = (mode == LR1121_MODE_STDBY_XOSC) ? 0x01 : 0x00;
    
    return lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC, 
                                               &standby_cfg, 1, radioNumber);
}

/**
 * @brief Set RF frequency
 * 
 * @param freq_hz Frequency in Hz
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_rf_frequency(uint32_t freq_hz,
                                                 SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4];
    
    /* LR1121 expects frequency in Hz as 32-bit big-endian */
    buf[0] = (freq_hz >> 24) & 0xFF;
    buf[1] = (freq_hz >> 16) & 0xFF;
    buf[2] = (freq_hz >> 8) & 0xFF;
    buf[3] = freq_hz & 0xFF;
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RF_FREQUENCY_OC,
                                               buf, 4, radioNumber);
}

/**
 * @brief Set packet type (LoRa or FSK)
 * 
 * @param pkt_type LR11XX_RADIO_PKT_TYPE_LORA or LR11XX_RADIO_PKT_TYPE_GFSK
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_packet_type(lr11xx_radio_pkt_type_t pkt_type,
                                                SX12XX_Radio_Number_t radioNumber)
{
    uint8_t type = (uint8_t)pkt_type;
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_TYPE_OC,
                                               &type, 1, radioNumber);
}

/**
 * @brief Write data to TX buffer
 * 
 * @param offset Offset in buffer (usually 0)
 * @param data Data to write
 * @param len Length of data
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_write_buffer(uint8_t offset,
                                             const uint8_t *data,
                                             uint8_t len,
                                             SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[256];  /* Max buffer size */
    
    if (len > 254) {
        return LR1121_HAL_ERROR_INVALID_PARAM;
    }
    
    buf[0] = offset;
    memcpy(&buf[1], data, len);
    
    return lr1121_hal_write_command_with_data(LR11XX_REGMEM_WRITE_BUFFER8_OC,
                                               buf, len + 1, radioNumber);
}

/**
 * @brief Read data from RX buffer
 * 
 * Citation: LR1121 Datasheet Section 14.5 "ReadBuffer8" (opcode 0x010A)
 *   The ReadBuffer8 command response format is:
 *   [Stat1][Stat2][DATA[0]][DATA[1]]...[DATA[len-1]]
 * 
 *   Where:
 *     Stat1 = Status byte 1 (command status/mode)
 *     Stat2 = Status byte 2 (additional status)
 *     DATA  = The actual buffer data starting at the requested offset
 * 
 * BUG FIX: Previous code read (len + 1) bytes and skipped only 1 byte.
 *          This caused the Stat2 byte to be included as DATA[0], shifting
 *          all data by 1 byte and causing CRC mismatches.
 * 
 * FIX: Read (len + 2) bytes and skip the first 2 status bytes.
 * 
 * @param offset Offset in buffer
 * @param data Buffer to store data
 * @param len Length to read
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_read_buffer(uint8_t offset,
                                            uint8_t *data,
                                            uint8_t len,
                                            SX12XX_Radio_Number_t radioNumber)
{
    uint8_t cmd_buf[2];
    lr1121_hal_status_t status;
    
    /* Send read buffer command with offset and length */
    cmd_buf[0] = offset;
    cmd_buf[1] = len;
    
    status = lr1121_hal_write_command_with_data(LR11XX_REGMEM_READ_BUFFER8_OC,
                                                 cmd_buf, 2, radioNumber);
    if (status != LR1121_HAL_OK) {
        return status;
    }
    
    /* Read response: 2 status bytes + actual data
     * 
     * Citation: LR1121 Datasheet - All read commands return 2 status bytes
     *           before the actual data payload.
     * 
     * Response format: [Stat1][Stat2][DATA[0]]...[DATA[len-1]]
     */
    uint8_t rx_buf[258];  /* Max: 2 status + 256 data bytes */
    status = lr1121_hal_read_command(rx_buf, len + 2, radioNumber);
    if (status != LR1121_HAL_OK) {
        return status;
    }
    
    /* Skip BOTH status bytes (Stat1, Stat2), copy only actual data
     * 
     * BUG FIX: Was skipping only 1 byte, causing off-by-one data shift
     */
    memcpy(data, &rx_buf[2], len);
    
    return LR1121_HAL_OK;
}

/**
 * @brief Clear IRQ status
 * 
 * @param irq_mask IRQ bits to clear
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_clear_irq(uint32_t irq_mask,
                                          SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4];
    
    buf[0] = (irq_mask >> 24) & 0xFF;
    buf[1] = (irq_mask >> 16) & 0xFF;
    buf[2] = (irq_mask >> 8) & 0xFF;
    buf[3] = irq_mask & 0xFF;
    
    return lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CLEAR_IRQ_OC,
                                               buf, 4, radioNumber);
}

/**
 * @brief Get IRQ status
 * 
 * Citation: LR1121 User Manual Rev 1.2 Section 4.1, Table 4-3 (SetDioIrqParams response format)
 *   The response format for commands returning IRQ status is:
 *   [Stat1][Stat2][IrqStatus(31:24)][IrqStatus(23:16)][IrqStatus(15:8)][IrqStatus(7:0)]
 *   Total: 6 bytes (2 status bytes + 4 IRQ bytes)
 * 
 * Note: This is different from GetStatus (0x0100) which returns chip mode/state.
 *       GetIrqStatus (0x0012) returns the actual IRQ flags.
 * 
 * BUG FIX: Was reading only 5 bytes and parsing buf[1] as IrqStatus[31:24].
 *          But buf[1] is Stat2, causing incorrect IRQ values (e.g., 0x1300C000).
 *          Now correctly reads 6 bytes and parses buf[2-5] as IrqStatus.
 * 
 * @param irq_status Pointer to store IRQ status
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_get_irq_status(uint32_t *irq_status,
                                               SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[6];  /* 2 status bytes (Stat1, Stat2) + 4 IRQ bytes */
    lr1121_hal_status_t status;
    
    /* Send GetIrqStatus command (opcode 0x0012)
     * Citation: Semtech lr11xx_system.c lr11xx_system_get_irq_status()
     */
    status = lr1121_hal_write_command(LR11XX_SYSTEM_GET_IRQ_STATUS_OC, radioNumber);
    if (status != LR1121_HAL_OK) {
        return status;
    }
    
    status = lr1121_hal_read_command(buf, 6, radioNumber);
    if (status != LR1121_HAL_OK) {
        return status;
    }
    
    /* IRQ status is in bytes 2-5 (big-endian), after Stat1 and Stat2
     * Citation: LR1121 User Manual Rev 1.2 Section 4.1, Table 4-3:
     *   buf[0] = Stat1
     *   buf[1] = Stat2  
     *   buf[2] = IrqStatus[31:24]
     *   buf[3] = IrqStatus[23:16]
     *   buf[4] = IrqStatus[15:8]
     *   buf[5] = IrqStatus[7:0]
     */
    *irq_status = ((uint32_t)buf[2] << 24) | 
                  ((uint32_t)buf[3] << 16) | 
                  ((uint32_t)buf[4] << 8) | 
                  (uint32_t)buf[5];
    
    return LR1121_HAL_OK;
}

/**
 * @brief Start RX mode - ELRS 4.0 COMPATIBLE
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 479-484
 *   case LR1121_MODE_RX_CONT:
 *       buf[0] = 0xFF;
 *       buf[1] = 0xFF;
 *       buf[2] = 0xFF;  // 0xFFFFFF = continuous RX (no timeout)
 *       hal.WriteCommand(LR11XX_RADIO_SET_RX_OC, buf, 3, radioNumber);
 * 
 * Citation: LR1121 Datasheet Section 7.2.2 "SetRx"
 *   - Timeout is 24-bit value in steps of 15.625µs (1/64000 seconds)
 *   - 0x000000 = no timeout (single RX, returns to standby after first packet)
 *   - 0xFFFFFF = continuous RX (stays in RX mode indefinitely)
 * 
 * @param timeout_ms Timeout parameter:
 *                   - 0 or 0xFFFFFF: continuous RX mode (ELRS default)
 *                   - Other values: timeout in milliseconds
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_rx(uint32_t timeout_ms,
                                       SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[3];
    uint32_t timeout_steps;
    
    /* ELRS uses 0 or 0xFFFFFF for continuous RX
     * Citation: ELRS RXnb() always uses SetMode(LR1121_MODE_RX_CONT) = 0xFFFFFF */
    if (timeout_ms == 0 || timeout_ms == 0xFFFFFF) {
        timeout_steps = 0xFFFFFF;  /* Continuous RX */
    } else {
        /* Convert milliseconds to 15.625µs steps (multiply by 64) */
        timeout_steps = timeout_ms * 64;
        if (timeout_steps > 0xFFFFFF) {
            timeout_steps = 0xFFFFFE;  /* Max timeout before continuous */
        }
    }
    
    buf[0] = (timeout_steps >> 16) & 0xFF;
    buf[1] = (timeout_steps >> 8) & 0xFF;
    buf[2] = timeout_steps & 0xFF;
    
    HAL_DBG("SetRx: timeout=0x%06lX (%s)\n", 
            (unsigned long)timeout_steps, 
            timeout_steps == 0xFFFFFF ? "continuous" : "timed");
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_OC,
                                               buf, 3, radioNumber);
}

/**
 * @brief Start TX mode
 * 
 * @param timeout_ms Timeout in ms
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_tx(uint32_t timeout_ms,
                                       SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[3];
    
    /* Convert to 15.625µs steps */
    uint32_t timeout_steps = (timeout_ms == 0) ? 0 : (timeout_ms * 64);
    
    buf[0] = (timeout_steps >> 16) & 0xFF;
    buf[1] = (timeout_steps >> 8) & 0xFF;
    buf[2] = timeout_steps & 0xFF;
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_TX_OC,
                                               buf, 3, radioNumber);
}

/**
 * @brief Enter continuous RX mode - EXACT ELRS 4.0 EQUIVALENT
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 708-711
 *   void ICACHE_RAM_ATTR LR1121Driver::RXnb()
 *   {
 *       SetMode(LR1121_MODE_RX_CONT, SX12XX_Radio_All);
 *   }
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 479-484 (SetMode for RX_CONT)
 *   case LR1121_MODE_RX_CONT:
 *       buf[0] = 0xFF;
 *       buf[1] = 0xFF;
 *       buf[2] = 0xFF;
 *       hal.WriteCommand(LR11XX_RADIO_SET_RX_OC, buf, 3, radioNumber);
 * 
 * This function is the EXACT equivalent of ELRS RXnb() - just one SetRx command
 * with 0xFFFFFF timeout for continuous reception.
 * 
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_rxnb(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[3] = {0xFF, 0xFF, 0xFF};  /* Continuous RX */
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_OC,
                                               buf, 3, radioNumber);
}

/**
 * @brief Configure DIO and IRQ parameters - ELRS 4.0 COMPATIBLE VERSION
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 559-564
 *   void LR1121Driver::SetDioIrqParams()
 *   {
 *       uint8_t buf[8] = {0};
 *       buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
 *       hal.WriteCommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, buf, sizeof(buf), SX12XX_Radio_All);
 *   }
 * 
 * Citation: LR1121 Datasheet Section 4.1.1 "SetDioIrqParams"
 *   - 8 bytes total (NOT 16!)
 *   - Bytes 0-3: IrqMask (little-endian, LSB in byte 3)
 *   - Bytes 4-7: Dio1Mask (little-endian, LSB in byte 7)
 * 
 * NOTE: This function ignores dio2_mask and dio3_mask for ELRS compatibility.
 *       ELRS only uses DIO1 for interrupt handling.
 */
lr1121_hal_status_t lr1121_hal_set_dio_irq_params(uint32_t irq_mask,
                                                   uint32_t dio1_mask,
                                                   uint32_t dio2_mask,
                                                   uint32_t dio3_mask,
                                                   SX12XX_Radio_Number_t radioNumber)
{
    (void)dio2_mask;  /* Unused - ELRS only uses DIO1 */
    (void)dio3_mask;  /* Unused - ELRS only uses DIO1 */
    
    /* ELRS format: 8 bytes, little-endian masks */
    uint8_t params[8] = {0};
    
    /* IRQ enable mask (little-endian: LSB in byte 3)
     * For ELRS: only TX_DONE (0x04) and RX_DONE (0x08) = 0x0C in byte 3 */
    params[3] = irq_mask & 0xFF;
    
    /* DIO1 mask (little-endian: LSB in byte 7)
     * Mirror the IRQ mask to route selected IRQs to DIO1 */
    params[7] = dio1_mask & 0xFF;
    
    HAL_DBG("SetDioIrqParams (ELRS): irq=0x%02X, dio1=0x%02X\n", 
            params[3], params[7]);
    
    return lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC,
                                               params, 8, radioNumber);
}

/**
 * @brief Configure DIO and IRQ for ELRS - Simplified ELRS 4.0 version
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 559-564
 *   ELRS only enables TX_DONE and RX_DONE, routed to DIO1.
 *   This is the EXACT function ELRS uses.
 * 
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_dio_irq_params_elrs(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[8] = {0};
    
    /* Only enable TX_DONE (0x04) and RX_DONE (0x08) = 0x0C */
    buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
    /* Mirror to DIO1 */
    buf[7] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
    
    HAL_DBG("SetDioIrqParams (ELRS): TX_DONE|RX_DONE → DIO1\n");
    
    return lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC,
                                               buf, 8, radioNumber);
}

/**
 * @brief Get IRQ status and clear in one transaction - ELRS 4.0 optimization
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 567-578
 *   uint32_t ICACHE_RAM_ATTR LR1121Driver::GetIrqStatus(SX12XX_Radio_Number_t radioNumber)
 *   {
 *       uint8_t status[6] = {0};
 *       status[0] = LR11XX_SYSTEM_CLEAR_IRQ_OC >> 8;
 *       status[1] = LR11XX_SYSTEM_CLEAR_IRQ_OC & 0xFF;
 *       status[2] = 0xFF;
 *       status[3] = 0xFF;
 *       status[4] = 0xFF;
 *       status[5] = 0xFF;
 *       hal.ReadCommand(status, sizeof(status), radioNumber);
 *       return status[2] << 24 | status[3] << 16 | status[4] << 8 | status[5];
 *   }
 * 
 * This is a clever optimization: ClearIrq returns the IRQ status before clearing,
 * so we can read and clear in a single SPI transaction.
 * 
 * @param irq_status Pointer to store IRQ status
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_get_irq_and_clear(uint32_t *irq_status,
                                                  SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status[6] = {0};
    lr1121_hal_status_t hal_status;
    
    /* Encode ClearIrq command into the buffer with mask = 0xFFFFFFFF (clear all) */
    status[0] = (LR11XX_SYSTEM_CLEAR_IRQ_OC >> 8) & 0xFF;   /* 0x01 */
    status[1] = LR11XX_SYSTEM_CLEAR_IRQ_OC & 0xFF;          /* 0x14 */
    status[2] = 0xFF;  /* Clear mask byte 0 (MSB) */
    status[3] = 0xFF;  /* Clear mask byte 1 */
    status[4] = 0xFF;  /* Clear mask byte 2 */
    status[5] = 0xFF;  /* Clear mask byte 3 (LSB) */
    
    /* Send command and read response - the response contains the IRQ status
     * that was active BEFORE clearing */
    hal_status = lr1121_hal_read_command(status, sizeof(status), radioNumber);
    if (hal_status != LR1121_HAL_OK) {
        return hal_status;
    }
    
    /* Parse IRQ status from response (big-endian in bytes 2-5) */
    *irq_status = ((uint32_t)status[2] << 24) | 
                  ((uint32_t)status[3] << 16) | 
                  ((uint32_t)status[4] << 8) | 
                  (uint32_t)status[5];
    
    return LR1121_HAL_OK;
}

/**
 * @brief SF6 register fix for SX127x compatibility
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 311-338
 *   void LR1121Driver::CorrectRegisterForSF6(uint8_t sf, SX12XX_Radio_Number_t radioNumber)
 *   {
 *       // 8.3.1 SetModulationParams
 *       // - SF6 can be made compatible with the SX127x family in implicit mode via a register setting.
 *       // - Set bit 18 of register at address 0xf20414 to 1
 *       // - Set bit 23 of register at address 0xf20414 to 0. This information is from Semtech in an email.
 *       
 *       if ((lr11xx_radio_lora_sf_t)sf == LR11XX_RADIO_LORA_SF6) {
 *           uint8_t wrbuf[12];
 *           // Address 0x00f20414
 *           wrbuf[0] = 0x00; wrbuf[1] = 0xf2; wrbuf[2] = 0x04; wrbuf[3] = 0x14;
 *           // Mask 0x00840000 (bit 18 and bit 23)
 *           wrbuf[4] = 0x00; wrbuf[5] = 0b10000100; wrbuf[6] = 0x00; wrbuf[7] = 0x00;
 *           // Data 0x00040000 (bit 18=1, bit 23=0)
 *           wrbuf[8] = 0x00; wrbuf[9] = 0b00000100; wrbuf[10] = 0x00; wrbuf[11] = 0x00;
 *           hal.WriteCommand(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, sizeof(wrbuf), radioNumber);
 *       }
 *   }
 * 
 * This function should be called AFTER SetModulationParams when using SF6 on SubGHz.
 * 
 * @param sf Spreading factor (only applies fix if sf == 6)
 * @param freq_hz Current frequency in Hz (only applies fix if < 1GHz)
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_correct_register_for_sf6(uint8_t sf,
                                                         uint32_t freq_hz,
                                                         SX12XX_Radio_Number_t radioNumber)
{
    /* Only apply fix for SF6 on SubGHz frequencies */
    if (sf != 6 || freq_hz >= 1000000000UL) {
        return LR1121_HAL_OK;  /* No fix needed */
    }
    
    HAL_DBG("Applying SF6 SX127x compatibility fix for %lu Hz\n", (unsigned long)freq_hz);
    
    /* WriteRegMemMask32 command parameters:
     * - Address: 0x00f20414 (4 bytes)
     * - Mask: 0x00840000 (bits 18 and 23) (4 bytes)
     * - Data: 0x00040000 (bit 18=1, bit 23=0) (4 bytes)
     */
    uint8_t wrbuf[12];
    
    /* Address 0x00f20414 (big-endian) */
    wrbuf[0] = 0x00;
    wrbuf[1] = 0xf2;
    wrbuf[2] = 0x04;
    wrbuf[3] = 0x14;
    
    /* Mask 0x00840000 - bits 18 and 23 (big-endian)
     * Bit 23 = 0x00800000, Bit 18 = 0x00040000
     * Combined = 0x00840000 */
    wrbuf[4] = 0x00;
    wrbuf[5] = 0x84;  /* 0b10000100 - bits 23 and 18 in this byte */
    wrbuf[6] = 0x00;
    wrbuf[7] = 0x00;
    
    /* Data 0x00040000 - bit 18=1, bit 23=0 (big-endian) */
    wrbuf[8] = 0x00;
    wrbuf[9] = 0x04;  /* 0b00000100 - only bit 18 set */
    wrbuf[10] = 0x00;
    wrbuf[11] = 0x00;
    
    return lr1121_hal_write_command_with_data(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC,
                                               wrbuf, sizeof(wrbuf), radioNumber);
}

/**
 * @brief Set LoRa sync word
 * 
 * Citation: LR1121 Datasheet Section 7.3.3 "SetLoRaSyncWord"
 * Opcode: 0x022B
 * 
 * ELRS uses private network sync word 0x1424
 */
lr1121_hal_status_t lr1121_hal_set_lora_sync_word(uint16_t sync_word,
                                                   SX12XX_Radio_Number_t radioNumber)
{
    uint8_t params[2];
    
    params[0] = (sync_word >> 8) & 0xFF;
    params[1] = sync_word & 0xFF;
    
    HAL_DBG("SetLoRaSyncWord: 0x%04X\n", sync_word);
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_LORA_SYNC_WORD_OC,
                                               params, 2, radioNumber);
}

/**
 * @brief Enable/disable RX boosted mode
 * 
 * Citation: LR1121 Datasheet Section 7.2.5 "SetRxBoosted"
 * Opcode: 0x0227
 * 
 * Enables higher gain in RX for better sensitivity
 */
lr1121_hal_status_t lr1121_hal_set_rx_boosted(bool enable,
                                               SX12XX_Radio_Number_t radioNumber)
{
    uint8_t param = enable ? 0x01 : 0x00;
    
    HAL_DBG("SetRxBoosted: %s\n", enable ? "enabled" : "disabled");
    
    return lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_BOOSTED_OC,
                                               &param, 1, radioNumber);
}
