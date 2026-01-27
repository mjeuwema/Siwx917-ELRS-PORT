/**
 * @file lr1121_hal.h
 * @brief LR1121 Hardware Abstraction Layer for SiW917
 * 
 * This HAL bridges ELRS protocol code to the native SiW917 LR1121 driver.
 * It provides the same interface as ELRS's LR1121Hal class but in C.
 * 
 * The HAL handles:
 *   - SPI communication via lr1121_driver.c
 *   - BUSY pin polling
 *   - Hardware reset
 *   - DIO1 interrupt handling
 * 
 * Citation: ExpressLRS 4.0 LR1121_hal.h
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 3 "SPI Interface"
 */

#ifndef LR1121_HAL_H
#define LR1121_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "lr1121_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Radio Number Selection
 * 
 * ELRS supports dual-radio configurations (Gemini mode).
 * For SiW917 single-radio implementation, we only use Radio_1.
 * 
 * Citation: ExpressLRS SX12xxDriverCommon.h
 ******************************************************************************/

typedef enum {
    SX12XX_Radio_NONE = 0,
    SX12XX_Radio_1    = (1 << 0),   /* Primary radio */
    SX12XX_Radio_2    = (1 << 1),   /* Secondary radio (Gemini) */
    SX12XX_Radio_All  = (SX12XX_Radio_1 | SX12XX_Radio_2)
} SX12XX_Radio_Number_t;

/*******************************************************************************
 * Firmware Version Structure
 * 
 * Citation: ExpressLRS LR1121.h
 ******************************************************************************/

typedef struct {
    uint8_t  hardware;      /* Hardware version */
    uint8_t  type;          /* Firmware type (0x03=LR1121, 0xDF=Bootloader) */
    uint16_t version;       /* Firmware version (e.g., 0x0104 = v1.4) */
} __attribute__((packed)) lr1121_firmware_version_t;

/*******************************************************************************
 * HAL Status Codes
 ******************************************************************************/

typedef enum {
    LR1121_HAL_OK = 0,
    LR1121_HAL_ERROR_SPI,
    LR1121_HAL_ERROR_BUSY_TIMEOUT,
    LR1121_HAL_ERROR_INVALID_PARAM,
    LR1121_HAL_ERROR_NOT_INITIALIZED,
} lr1121_hal_status_t;

/*******************************************************************************
 * ISR Callback Type
 ******************************************************************************/

typedef void (*lr1121_isr_callback_t)(void);

/*******************************************************************************
 * HAL Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize the LR1121 HAL
 * 
 * Sets up GPIO pins, SPI interface, and interrupts.
 * Must be called before any other HAL functions.
 * 
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_init(void);

/**
 * @brief Deinitialize the LR1121 HAL
 * 
 * Releases GPIO pins, SPI, and interrupts.
 */
void lr1121_hal_end(void);

/**
 * @brief Perform hardware reset of the LR1121
 * 
 * Reset sequence:
 *   1. Drive RST LOW for >= 100µs
 *   2. Drive RST HIGH
 *   3. Wait ~300ms for BUSY to go LOW
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 4.2.1
 * 
 * @param bootloader If true, hold BUSY LOW during reset to enter bootloader
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_reset(bool bootloader);

/**
 * @brief Wake LR1121 from SLEEP mode
 * 
 * Wake sequence:
 *   1. Toggle NSS LOW for ~10ms
 *   2. Drive NSS HIGH
 *   3. Wait for BUSY to go LOW
 * 
 * After wakeup, chip enters STDBY_RC mode.
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 7.2.2
 *   "A falling edge on NSS wakes the chip from sleep mode"
 * Citation: Waveshare lr11xx_hal.c lr11xx_hal_wakeup() function
 * 
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_wakeup(void);

/**
 * @brief Write command to LR1121 (Phase 1 of SPI protocol)
 * 
 * Protocol:
 *   1. Wait for BUSY LOW
 *   2. Assert NSS
 *   3. Send 16-bit opcode MSB first
 *   4. Send parameter bytes
 *   5. Deassert NSS
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 3.1
 * 
 * @param opcode 16-bit command opcode
 * @param radioNumber Which radio (Radio_1, Radio_2, or Radio_All)
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_write_command(uint16_t opcode, 
                                              SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Write command with data to LR1121
 * 
 * @param opcode 16-bit command opcode
 * @param buffer Parameter data buffer
 * @param size Number of parameter bytes
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_write_command_with_data(uint16_t opcode, 
                                                        const uint8_t *buffer, 
                                                        uint8_t size, 
                                                        SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Read response from LR1121 (Phase 2 of SPI protocol)
 * 
 * Protocol:
 *   1. Wait for BUSY LOW
 *   2. Assert NSS
 *   3. Send NOP bytes, receive response
 *   4. Deassert NSS
 * 
 * Note: First byte received is status byte, actual data starts at byte 1.
 * 
 * @param buffer Buffer to store response
 * @param size Number of bytes to read
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_read_command(uint8_t *buffer, 
                                             uint8_t size, 
                                             SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Wait for BUSY pin to go LOW
 * 
 * The BUSY pin indicates when the LR1121 is processing a command.
 * Must wait for BUSY LOW before starting a new SPI transaction.
 * 
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf Section 3.1
 * 
 * @param radioNumber Which radio
 * @return true if BUSY went LOW within timeout, false on timeout
 */
bool lr1121_hal_wait_on_busy(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Register DIO1 interrupt callback
 * 
 * DIO1 is used for TX/RX done interrupts.
 * 
 * @param callback Function to call on DIO1 rising edge
 * @param radioNumber Which radio (Radio_1 or Radio_2)
 */
void lr1121_hal_set_isr_callback(lr1121_isr_callback_t callback, 
                                  SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Get firmware version from LR1121
 * 
 * @param version Pointer to store version info
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_get_version(lr1121_firmware_version_t *version,
                                            SX12XX_Radio_Number_t radioNumber);

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
                                            SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Set RF frequency
 * 
 * @param freq_hz Frequency in Hz
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_rf_frequency(uint32_t freq_hz,
                                                 SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Set packet type (LoRa or FSK)
 * 
 * @param pkt_type LR11XX_RADIO_PKT_TYPE_LORA or LR11XX_RADIO_PKT_TYPE_GFSK
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_packet_type(lr11xx_radio_pkt_type_t pkt_type,
                                                SX12XX_Radio_Number_t radioNumber);

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
                                             SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Read data from RX buffer
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
                                            SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Clear IRQ status
 * 
 * @param irq_mask IRQ bits to clear
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_clear_irq(uint32_t irq_mask,
                                          SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Get IRQ status
 * 
 * @param irq_status Pointer to store IRQ status
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_get_irq_status(uint32_t *irq_status,
                                               SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Start RX mode
 * 
 * @param timeout_ms Timeout in ms (0 = continuous, 0xFFFFFF = single)
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_rx(uint32_t timeout_ms,
                                       SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Enter continuous RX mode - EXACT ELRS 4.0 EQUIVALENT
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 708-711
 *   This is the EXACT equivalent of ELRS RXnb() function.
 * 
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_rxnb(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Start TX mode
 * 
 * @param timeout_ms Timeout in ms
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_tx(uint32_t timeout_ms,
                                       SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Configure DIO and IRQ parameters
 * 
 * Sets which IRQs are enabled and routes them to DIO pins.
 * 
 * Citation: LR1121 Datasheet Section 9.2.1 "SetDioIrqParams"
 * 
 * @param irq_mask IRQ enable mask
 * @param dio1_mask IRQs to route to DIO1
 * @param dio2_mask IRQs to route to DIO2
 * @param dio3_mask IRQs to route to DIO3
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_dio_irq_params(uint32_t irq_mask,
                                                   uint32_t dio1_mask,
                                                   uint32_t dio2_mask,
                                                   uint32_t dio3_mask,
                                                   SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Set LoRa sync word
 * 
 * Citation: LR1121 Datasheet Section 7.3.3 "SetLoRaSyncWord"
 * 
 * @param sync_word 16-bit sync word (ELRS uses 0x1424)
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_lora_sync_word(uint16_t sync_word,
                                                   SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Enable/disable RX boosted mode for better sensitivity
 * 
 * Citation: LR1121 Datasheet Section 7.2.5 "SetRxBoosted"
 * 
 * @param enable true to enable boosted mode
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_rx_boosted(bool enable,
                                               SX12XX_Radio_Number_t radioNumber);

/*******************************************************************************
 * ELRS 4.0 Optimized Functions
 * 
 * These functions match ELRS 4.0 LR1121.cpp exactly for maximum compatibility.
 ******************************************************************************/

/**
 * @brief Configure DIO and IRQ for ELRS - Simplified ELRS 4.0 version
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 559-564
 *   ELRS only enables TX_DONE and RX_DONE, routed to DIO1.
 * 
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_set_dio_irq_params_elrs(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Get IRQ status and clear in one transaction - ELRS 4.0 optimization
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 567-578
 *   This is a clever optimization: ClearIrq returns the IRQ status before clearing,
 *   so we can read and clear in a single SPI transaction.
 * 
 * @param irq_status Pointer to store IRQ status
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_get_irq_and_clear(uint32_t *irq_status,
                                                  SX12XX_Radio_Number_t radioNumber);

/**
 * @brief SF6 register fix for SX127x compatibility
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp line 311-338
 *   SF6 can be made compatible with SX127x family in implicit mode via register setting.
 *   Should be called AFTER SetModulationParams when using SF6 on SubGHz.
 * 
 * @param sf Spreading factor (only applies fix if sf == 6)
 * @param freq_hz Current frequency in Hz (only applies fix if < 1GHz)
 * @param radioNumber Which radio
 * @return LR1121_HAL_OK on success
 */
lr1121_hal_status_t lr1121_hal_correct_register_for_sf6(uint8_t sf,
                                                         uint32_t freq_hz,
                                                         SX12XX_Radio_Number_t radioNumber);

/*******************************************************************************
 * DIO1 ISR Handlers (to be called from GPIO interrupt)
 ******************************************************************************/

/**
 * @brief DIO1 interrupt handler for Radio 1
 * Call this from the GPIO ISR when DIO1 rising edge is detected.
 */
void lr1121_hal_dio1_isr_radio1(void);

/**
 * @brief DIO1 interrupt handler for Radio 2
 */
void lr1121_hal_dio1_isr_radio2(void);

/*******************************************************************************
 * Convenience Macros
 ******************************************************************************/

/**
 * @brief Undefined pin constant (for optional pins)
 */
#define UNDEF_PIN   0xFF

/**
 * @brief Check if radio number includes Radio_1
 */
#define HAL_RADIO_IS_1(rn)   (((rn) & SX12XX_Radio_1) != 0)

/**
 * @brief Check if radio number includes Radio_2
 */
#define HAL_RADIO_IS_2(rn)   (((rn) & SX12XX_Radio_2) != 0)

#ifdef __cplusplus
}
#endif

#endif /* LR1121_HAL_H */
