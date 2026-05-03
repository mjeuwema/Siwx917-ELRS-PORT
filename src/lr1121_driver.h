/**
 * @file lr1121_driver.h
 * @brief Minimal LR1121 driver for SIW917 - GetVersion test
 *
 * This is a minimal standalone implementation to test SPI communication
 * with the LR1121 radio transceiver via the SIW917 GSPI peripheral.
 *
 * Documentation Citations:
 * - 61252685.LR1121_V2_1_data_sheet.pdf Section 3 "SPI Interface"
 * - 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1 "Reset Timing"
 * - siw917x-family-rm.pdf Section 20 "Generic SPI Primary (GSPI)"
 * - ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Pinout"
 *
 * Hardware Connections (BRD2708A mikroBUS socket):
 * - GPIO_25: SCK  (SPI Clock)
 * - GPIO_26: MISO (SPI Data from LR1121)
 * - GPIO_27: MOSI (SPI Data to LR1121)
 * - GPIO_28: NSS  (Chip Select, Active Low)
 * - GPIO_29: BUSY (Busy Signal, Active High when busy)
 * - GPIO_30: RST  (Hardware Reset, Active Low)
 */

#ifndef LR1121_DRIVER_H
#define LR1121_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * LR1121 Command Opcodes
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 11.1 "Command List"
 * Commands are 16-bit opcodes sent MSB first.
 ******************************************************************************/

/**
 * @brief System GetVersion command opcode
 *
 * Citation: LR1121 Datasheet Section 11.1
 * Response: [Status][HW Version][FW Type][FW Version MSB][FW Version LSB]
 */
#define LR1121_CMD_GET_VERSION 0x0101

/**
 * @brief System SetStandby command opcode
 * Parameter: 1 byte (0x00 = STDBY_RC, 0x01 = STDBY_XOSC)
 */
#define LR1121_CMD_SET_STANDBY 0x011C

/**
 * @brief System ClearErrors command opcode
 */
#define LR1121_CMD_CLEAR_ERRORS 0x010E

/**
 * @brief System GetStatus command opcode
 */
#define LR1121_CMD_GET_STATUS 0x0100

/**
 * @brief System SetTcxoMode command opcode
 *
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 * Parameters: [Voltage Trim (1 byte)][Delay (3 bytes, in 30.52µs steps)]
 *
 * Voltage Trim Values (for Core1121-HF TCXO module):
 *   0x00 = 1.6V
 *   0x01 = 1.7V
 *   0x02 = 1.8V
 *   0x03 = 2.2V
 *   0x04 = 2.4V
 *   0x05 = 2.7V
 *   0x06 = 3.0V
 *   0x07 = 3.3V
 *
 * Required for TCXO-based modules to start the oscillator and lock PLL.
 */
#define LR1121_CMD_SET_TCXO_MODE 0x0117 /* FIX: Was 0x0097 (wrong!) */

/*******************************************************************************
 * LR1121 Firmware Type Constants (Use Case)
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 11.1.1 (GetVersion)
 * The "Type" field in GetVersion response corresponds to the "Use Case":
 * - 0x01: LR1110
 * - 0x02: LR1120
 * - 0x03: LR1121
 * - 0xDF: Bootloader
 ******************************************************************************/

#define LR1121_FIRMWARE_TYPE_NORMAL 0x03     /* LR1121 Use Case */
#define LR1121_FIRMWARE_TYPE_BOOTLOADER 0xDF /* Bootloader mode */

/*******************************************************************************
 * Pin Definitions for BRD2708A mikroBUS Socket
 *
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
 * Citation: siw917x-family-rm.pdf Section 11 "GPIO"
 ******************************************************************************/

#define LR1121_PIN_SCK 25  /* SPI Clock */
#define LR1121_PIN_MISO 26 /* SPI Master In Slave Out */
#define LR1121_PIN_MOSI 27 /* SPI Master Out Slave In */
#define LR1121_PIN_NSS 28  /* SPI Chip Select (Active Low) */
#define LR1121_PIN_BUSY 29 /* Busy signal (Active High) */
#define LR1121_PIN_RST 30  /* Reset signal (Active Low) */

/*******************************************************************************
 * Timing Constants
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1 "Reset Timing"
 ******************************************************************************/

/**
 * @brief BUSY pin timeout in microseconds
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1
 * Some commands have long execution times (up to 230ms after reset).
 * Using 500ms timeout for safety during initialization.
 */
#define LR1121_BUSY_TIMEOUT_US 500000U

/**
 * @brief Reset recovery delay in milliseconds
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1
 * "BUSY is HIGH for approximately 230ms after reset"
 */
#define LR1121_RESET_RECOVERY_MS 300U

/**
 * @brief Minimum reset pulse width in milliseconds
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1
 * "Minimum reset pulse width is 100µs"
 *
 * FIX: Increased from 1ms to 100ms to ensure the LR1121 fully resets
 * from any hung state. Semtech recommends longer reset for stuck chips.
 */
#define LR1121_RESET_PULSE_MS 100U

/**
 * @brief TCXO Configuration Mode Selection
 *
 * Citation: LR1121 User Manual Section 6.3.2 "SetTcxoMode"
 * Citation: ExpressLRS GitHub Discussion #3045 - TCXO power sources
 *
 * IMPORTANT FIX: Even for externally-powered TCXO modules, SetTcxoMode
 * may still be needed to:
 * 1. Configure the TCXO detection circuitry timing
 * 2. Set the timeout for XOSC stabilization
 * 3. Enable the XOSC input path in the LR1121
 *
 * If SetStandby(XOSC) causes the chip to fall back to SLEEP mode (0),
 * this indicates the XOSC/TCXO failed to start. SetTcxoMode must be
 * called even for externally-powered modules in this case.
 *
 * Set this to 1 to SKIP SetTcxoMode (only if STANDBY_XOSC works!)
 * Set this to 0 to ENABLE SetTcxoMode (recommended - try this first!)
 */
#define LR1121_TCXO_EXTERNAL_POWER                                             \
  0 /* FIX: Enable SetTcxoMode to configure XOSC path */

/**
 * @brief TCXO voltage trim value (only used if LR1121_TCXO_EXTERNAL_POWER == 0)
 *
 * Citation: LR1121 User Manual Section 6.3.2 "SetTcxoMode"
 *
 * Voltage Trim Values:
 *   0x00 = 1.6V
 *   0x01 = 1.7V
 *   0x02 = 1.8V  <-- Most modules with internal VTCXO use this
 *   0x03 = 2.2V
 *   0x04 = 2.4V
 *   0x05 = 2.7V
 *   0x06 = 3.0V
 *   0x07 = 3.3V
 *
 * NOTE: Only relevant for modules where TCXO is powered from LR1121's
 * internal VTCXO regulator. For Core1121-HF, this value is IGNORED
 * because LR1121_TCXO_EXTERNAL_POWER is set to 1.
 */
/**
 * FIX: Core1121-HF uses externally-powered 3.3V TCXO
 * Even though TCXO is externally powered, SetTcxoMode must be called
 * with the correct voltage trim to configure the XOSC input path.
 * Use 0x07 (3.3V) to match the external TCXO voltage.
 */
#define LR1121_TCXO_VOLTAGE_TRIM                                               \
  0x07 /* 3.3V - matches Core1121-HF external TCXO */

/**
 * @brief TCXO startup delay in 30.52µs steps
 *
 * Citation: LR1121 Datasheet Section 11.2.5
 *   5ms startup  = 5000µs / 30.52µs  ≈ 164 steps  (0x0000A4)
 *   10ms startup = 10000µs / 30.52µs ≈ 328 steps  (0x000148)
 *   50ms startup = 50000µs / 30.52µs ≈ 1638 steps (0x000666)
 *
 * Using 50ms for robust TCXO startup on cold boot.
 */
#define LR1121_TCXO_DELAY_MS50 0x000666
#define LR1121_TCXO_DELAY_MS10 0x000148
#define LR1121_TCXO_DELAY LR1121_TCXO_DELAY_MS50 /* Use 50ms for reliability   \
                                                  */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief LR1121 firmware version information
 *
 * Returned by lr1121_get_version() function.
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 11.1.1
 */
typedef struct {
  uint8_t hardware; /**< Hardware version */
  uint8_t type;     /**< Firmware type: 0xF3=normal, 0xDF=bootloader */
  uint16_t version; /**< Firmware version (e.g., 0x0104 = v1.4) */
} lr1121_version_t;

/**
 * @brief LR1121 initialization result codes
 */
typedef enum {
  LR1121_OK = 0,             /**< Success */
  LR1121_ERROR_SPI_INIT,     /**< SPI initialization failed */
  LR1121_ERROR_BUSY_TIMEOUT, /**< BUSY pin timeout */
  LR1121_ERROR_INVALID_TYPE, /**< Invalid firmware type in response */
  LR1121_ERROR_GPIO_INIT,    /**< GPIO initialization failed */
} lr1121_status_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize LR1121 driver (GPIO, SPI, reset sequence)
 *
 * Initialization Sequence:
 * 1. Configure GPIO pins (BUSY, RST as GPIO, NSS handled by GSPI)
 * 2. Initialize GSPI peripheral via SDK
 * 3. Perform hardware reset sequence
 * 4. Wait for BUSY to go LOW
 *
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_init(void);

/**
 * @brief Deinitialize LR1121 driver
 */
void lr1121_deinit(void);

/**
 * @brief Hardware reset the LR1121
 *
 * Reset Sequence:
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1
 * 1. Drive RST LOW for >= 100µs
 * 2. Drive RST HIGH
 * 3. Wait 300ms for BUSY to go LOW
 *
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_reset(void);

/**
 * @brief Get LR1121 firmware version
 *
 * SPI Protocol:
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 3.1
 *
 * Phase 1 (Command):
 * 1. Wait for BUSY LOW
 * 2. Assert NSS (CS LOW)
 * 3. Send opcode [0x01][0x01]
 * 4. Deassert NSS (CS HIGH)
 *
 * Phase 2 (Response):
 * 5. Wait for BUSY LOW (command processing complete)
 * 6. Assert NSS (CS LOW)
 * 7. Clock out 5 NOP bytes, receive response:
 *    [Status][HW][Type][VersionMSB][VersionLSB]
 * 8. Deassert NSS (CS HIGH)
 *
 * @param[out] version Pointer to version structure to fill
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_get_version(lr1121_version_t *version);

/**
 * @brief Wait for BUSY pin to go LOW
 *
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 3.1
 * "BUSY is HIGH while the LR1121 is processing a command"
 *
 * @return true if BUSY went LOW within timeout, false on timeout
 */
bool lr1121_wait_busy(void);

/**
 * @brief Test LR1121 communication by reading and printing version
 *
 * Convenience function that initializes, reads version, and prints results.
 * Useful for validating hardware connection and SPI communication.
 */
void lr1121_test_communication(void);

/**
 * @brief Send SetTcxoMode command to initialize TCXO oscillator
 *
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *
 * Required for TCXO-based modules (like Core1121-HF) to start the
 * oscillator and achieve PLL lock. BUSY will be LOW when PLL is locked.
 *
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_set_tcxo_mode(void);

/**
 * @brief De-initialize and re-initialize the GSPI peripheral
 *
 * This function performs a complete GSPI reset sequence to clear
 * any hung state from prior failed operations.
 *
 * Citation: siw917x-family-rm.pdf Section 20 "Generic SPI Primary (GSPI)"
 */
void lr1121_reinit_spi(void);

/**
 * @brief Run hardware verification test after reset
 *
 * This function verifies GPIO connectivity and LR1121 response AFTER
 * the LR1121 has been reset. Tests include:
 * - BUSY pin state verification (should be LOW after reset)
 * - MISO pin response with CS asserted
 * - Quick SPI command/response test
 *
 * Note: Must be called AFTER lr1121_reset() for meaningful results.
 *
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
 */
void lr1121_hw_verification_test(void);

/**
 * @brief GPIO Toggle Test for Multimeter Verification
 *
 * This function slowly toggles each SPI output pin (SCK, MOSI, CS, RST)
 * so you can verify with a multimeter that the SiWx917 GPIOs are
 * actually outputting voltage.
 *
 * The test runs in a loop, toggling each pin for about 2 seconds each.
 * Use a multimeter to measure:
 * - GPIO_25 (SCK)  - Should toggle 0V <-> 3.3V
 * - GPIO_27 (MOSI) - Should toggle 0V <-> 3.3V
 * - GPIO_28 (CS)   - Should toggle 0V <-> 3.3V
 * - GPIO_30 (RST)  - Should toggle 0V <-> 3.3V
 *
 * Input pins (MISO, BUSY) are NOT toggled - they should read the
 * state of what's connected.
 *
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
 *
 * @param cycles Number of toggle cycles per pin (each cycle ~500ms)
 *               Use 0 for infinite loop (reset MCU to exit)
 */
void lr1121_gpio_toggle_test(uint32_t cycles);

/*******************************************************************************
 * Low-Level Functions (used by standalone test suite)
 ******************************************************************************/

/**
 * @brief Wait for BUSY pin to go LOW with timeout
 *
 * @param timeout_ms Timeout in milliseconds
 * @return true if BUSY went LOW, false on timeout
 */
bool lr1121_wait_busy_timeout(uint32_t timeout_ms);

/**
 * @brief Send a command to the LR1121 (Phase 1 of SPI protocol)
 *
 * Citation: LR1121 User Manual - SPI Communication
 * Sends opcode + parameters with CS control.
 *
 * @param opcode 16-bit command opcode
 * @param params Parameter bytes (can be NULL)
 * @param param_len Number of parameter bytes
 * @return true on success
 */
bool lr1121_send_command(uint16_t opcode, const uint8_t *params,
                         uint16_t param_len);

/**
 * @brief Read response from LR1121 (Phase 2 of SPI protocol)
 *
 * Citation: LR1121 User Manual - SPI Communication
 * Clocks out NOP bytes to receive response data.
 *
 * @param response Buffer to store response
 * @param response_len Number of bytes to read
 * @return true on success
 */
bool lr1121_read_response(uint8_t *response, uint16_t response_len);

/**
 * @brief Read a response using bit-banged GPIO instead of the GSPI SDK.
 *
 * This is a narrow recovery path for LR1121 response phases where the SDK GSPI
 * transfer can block after an RX_DONE event. The command phase should already
 * have completed and BUSY should already be LOW.
 *
 * @param response Buffer to store response
 * @param response_len Number of bytes to read
 * @return true on success
 */
bool lr1121_read_response_soft(uint8_t *response, uint16_t response_len);

/**
 * @brief Execute the ELRS LR1121 custom GetPacket opcode (0x0700).
 *
 * This opcode is provided by the ELRS LR1121 transceiver firmware (F3xx) and
 * returns packet metadata plus payload in one response. The helper keeps the
 * command framing centralized so the C++ ELRS driver can fall back cleanly if
 * the SiW917 GSPI path cannot complete the transaction.
 *
 * @param response Buffer to store metadata + packet bytes
 * @param response_len Number of bytes to read
 * @param use_soft_response Use GPIO bit-banged clocking for the response phase;
 *                          otherwise use the polled/bare-metal GSPI path
 * @return true on success
 */
bool lr1121_elrs_get_packet(uint8_t *response, uint16_t response_len,
                            bool use_soft_response);

/**
 * @brief Execute the ELRS LR1121 custom SetRfFrequency_SetRx opcode (0x0701).
 *
 * The ELRS LR1121 transceiver firmware provides this helper so FHSS hopping can
 * retune and re-enter continuous RX in one radio command. On SiW917, using the
 * same soft GPIO SPI path as custom GetPacket avoids GSPI SDK transaction
 * timing problems in the tight hop window.
 *
 * @param freq_hz Target RF frequency in Hz
 * @param use_soft_command Use GPIO bit-banged clocking for the command phase;
 *                         otherwise use the normal GSPI command path
 * @return true on success
 */
bool lr1121_elrs_set_freq_set_rx(uint32_t freq_hz, bool use_soft_command);

/**
 * @brief Get LR1121 status bytes
 *
 * Citation: LR1121 User Manual Section 2.1 (Status Byte)
 *
 * stat1 format:
 *   Bits [4:1] = cmd_status:
 *     0 = FAIL (command not executed)
 *     1 = PERR (parameter error)
 *     2 = SPI_ERR (SPI communication error)
 *     3 = OK (command executed successfully)
 *     4 = DATA_AVAIL (data available for read)
 *
 * stat2 format:
 *   Bits [6:4] = chip_mode:
 *     0 = SLEEP
 *     1 = STANDBY_RC
 *     2 = STANDBY_XOSC
 *     3 = FS (Frequency Synthesis)
 *     4 = RX
 *     5 = TX
 *
 * @param stat1 Status byte 1 (command status) - use (stat1 >> 1) & 0x07
 * @param stat2 Status byte 2 (chip mode) - use (stat2 >> 4) & 0x07
 * @param irq_status IRQ status bits
 * @return true on success
 */
bool lr1121_get_status(uint8_t *stat1, uint8_t *stat2, uint8_t *irq_status);

/**
 * @brief LR1121 Chip Mode Constants
 * Citation: LR1121 User Manual Section 2.1
 */
#define LR1121_CHIP_MODE_SLEEP 0
#define LR1121_CHIP_MODE_STDBY_RC 1
#define LR1121_CHIP_MODE_STDBY_XOSC 2
#define LR1121_CHIP_MODE_FS 3
#define LR1121_CHIP_MODE_RX 4
#define LR1121_CHIP_MODE_TX 5

/**
 * @brief LR1121 Command Status Constants
 * Citation: LR1121 User Manual Section 2.1
 */
#define LR1121_CMD_STATUS_FAIL 0
#define LR1121_CMD_STATUS_PERR 1
#define LR1121_CMD_STATUS_SPI_ERR 2
#define LR1121_CMD_STATUS_OK 3
#define LR1121_CMD_STATUS_DATA_AVAIL 4

/**
 * @brief Decode command status from stat1 byte
 * @param stat1 Raw status byte 1 from GetStatus
 * @return Command status (0-4)
 */
#define LR1121_GET_CMD_STATUS(stat1) (((stat1) >> 1) & 0x07)

/**
 * @brief Decode chip mode from stat2 byte
 * @param stat2 Raw status byte 2 from GetStatus
 * @return Chip mode (0-5)
 */
#define LR1121_GET_CHIP_MODE(stat2) (((stat2) >> 4) & 0x07)

/*******************************************************************************
 * Raw SPI Functions for Single-Phase Commands
 *
 * These functions are exposed for commands like GetTemperature and
 * GetRandomNumber that require single-phase SPI transactions (response returned
 * during command).
 *
 * Citation: LR1121 User Manual - Some commands return data during the command
 * transaction itself, not in a separate Phase 2 NOP read.
 ******************************************************************************/

/**
 * @brief Assert CS (drive LOW) for SPI transaction
 */
void lr1121_cs_assert(void);

/**
 * @brief Deassert CS (drive HIGH) after SPI transaction
 */
void lr1121_cs_deassert(void);

/**
 * @brief Raw SPI transfer (full-duplex)
 *
 * Performs a full-duplex SPI transfer without CS control.
 * Caller must handle CS assertion/deassertion.
 *
 * @param tx_data Data to transmit (can be NULL for receive-only)
 * @param rx_data Buffer for received data (can be NULL for transmit-only)
 * @param length Number of bytes to transfer
 * @return true on success, false on error
 */
bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data,
                         uint16_t length);

/**
 * @brief Raw full-duplex SPI transfer using the polled/bare-metal GSPI path.
 *
 * Caller must handle CS assertion/deassertion. This is intended for hot-path
 * ELRS LR1121 transactions where the Silicon Labs interrupt-driven GSPI driver
 * can contend with DIO/timer timing.
 */
bool lr1121_spi_transfer_polled(const uint8_t *tx_data, uint8_t *rx_data,
                                uint16_t length);

/*******************************************************************************
 * Firmware Version Structure (for OTA updates)
 *
 * Note: If lr1121_hal.h is included first, it defines this type.
 * This guards against duplicate definition.
 ******************************************************************************/
#ifndef LR1121_HAL_H /* Only define if lr1121_hal.h not included */
typedef struct {
  uint8_t hardware; /* Hardware version */
  uint8_t type;     /* Firmware type (0x03=LR1121, 0xDF=Bootloader) */
  uint16_t version; /* Firmware version (e.g., 0x0104 = v1.4) */
} lr1121_firmware_version_t;
#endif

/*******************************************************************************
 * Bootloader Command Opcodes for Firmware Updates
 *
 * Note: Primary definitions are in elrs_protocol/lr1121_regs.h
 * These are provided here as fallbacks if lr1121_regs.h isn't included.
 ******************************************************************************/
/* Use raw values - don't define macros that conflict with lr1121_regs.h enum */
#define LR1121_OPCODE_GET_VERSION 0x0101    /* Normal mode GetVersion */
#define LR1121_OPCODE_BL_GET_VERSION 0x0101 /* Bootloader GetVersion (same opcode as app mode) */
#define LR1121_OPCODE_BL_ERASE_FLASH 0x8000 /* Erase flash */
#define LR1121_OPCODE_BL_WRITE_FLASH 0x8003 /* Write encrypted flash */
#define LR1121_OPCODE_BL_REBOOT 0x8005      /* Reboot to app */

/*******************************************************************************
 * Firmware Update Functions
 *
 * Citation: ExpressLRS LR1121.cpp firmware update implementation
 ******************************************************************************/

/**
 * @brief Get firmware version using specified command opcode
 *
 * @param version Output structure for version info
 * @param opcode Command opcode (LR11XX_SYSTEM_GET_VERSION_OC or
 * LR11XX_BL_GET_VERSION_OC)
 * @return true on success, false on failure
 */
bool lr1121_get_firmware_version(lr1121_firmware_version_t *version,
                                 uint16_t opcode);

/**
 * @brief Begin firmware update (enters bootloader, erases flash)
 *
 * @param expected_size Expected total firmware size in bytes
 * @return 0 on success, negative error code on failure
 */
int lr1121_begin_update(uint32_t expected_size);

/**
 * @brief Write firmware data bytes to LR1121 flash
 *
 * @param data Pointer to data buffer
 * @param size Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
int lr1121_write_update_bytes(const uint8_t *data, uint32_t size);

/**
 * @brief End firmware update (reboot to new firmware)
 *
 * @return 0 on success, negative error code on failure
 */
int lr1121_end_update(void);

/*******************************************************************************
 * DIO1 Interrupt Functions
 *
 * Citation: LR1121 Datasheet - DIO1 is used for IRQ signaling
 * Connected to SiW917 UULP_VBAT_GPIO_2 on BRD2708A
 ******************************************************************************/

/* DIO1 callback function type */
typedef void (*lr1121_dio1_callback_t)(void);

/**
 * @brief Initialize DIO1 GPIO for interrupt input
 * @return LR1121_OK on success
 */
lr1121_status_t lr1121_dio1_init(void);

/**
 * @brief Read DIO1 pin state
 * @return 1 if HIGH, 0 if LOW
 */
int lr1121_dio1_read(void);

/**
 * @brief Set DIO1 interrupt callback
 * @param callback Function to call on DIO1 rising edge
 */
void lr1121_dio1_set_callback(lr1121_dio1_callback_t callback);

/**
 * @brief Enable DIO1 NVIC interrupt
 */
void lr1121_dio1_enable(void);

/**
 * @brief Disable DIO1 NVIC interrupt
 */
void lr1121_dio1_disable(void);

/**
 * @brief Pause DIO1 NVIC interrupt (for SPI re-entrancy)
 */
void lr1121_dio1_pause_isr(void);

/**
 * @brief Resume DIO1 NVIC interrupt (for SPI re-entrancy)
 */
void lr1121_dio1_resume_isr(void);

/**
 * @brief Get DIO1 ISR count for debugging
 * @return Number of times the DIO1 ISR callback was entered
 */
uint32_t lr1121_dio1_get_isr_count(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_DRIVER_H */
