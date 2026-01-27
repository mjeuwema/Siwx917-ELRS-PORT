/**
 * @file LR1121Driver.h
 * @brief ELRS 4.0 Compatible LR1121 Radio Driver (C Implementation)
 * 
 * This is a direct C port of ExpressLRS 4.0 src/lib/LR1121Driver/LR1121.h
 * 
 * The driver provides:
 *   - LoRa and FSK modulation support
 *   - Dual radio (Gemini) support
 *   - Non-blocking TX/RX with interrupt callbacks
 *   - Power management with pending updates
 *   - SF6 compatibility fix for SX127x
 * 
 * Citation: ExpressLRS 4.0 LR1121.h
 * https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/LR1121Driver/LR1121.h
 */

#ifndef LR1121_DRIVER_ELRS_H
#define LR1121_DRIVER_ELRS_H

#include <stdint.h>
#include <stdbool.h>
#include "lr1121_regs.h"
#include "lr1121_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants from ELRS 4.0
 ******************************************************************************/

#define RADIO_SNR_SCALE             4

/* Firmware type for LR1121 application firmware */
#define LR1121_FIRMWARE_TYPE        0xF3

/* Expected firmware version (ELRS custom firmware) */
#define LR11XX_FIRMWARE_VERSION     0x0104

/* Power pending none marker - must not be valid power value */
#define PWRPENDING_NONE             0x7F

/* Undefined pin constant */
#ifndef UNDEF_PIN
#define UNDEF_PIN                   0xFF
#endif

/* RX buffer size */
#define LR1121_RX_BUFFER_SIZE       32

/*******************************************************************************
 * Firmware Version Structure (matches ELRS)
 ******************************************************************************/

typedef struct {
    uint8_t  hardware;      /* Hardware version */
    uint8_t  type;          /* Firmware type (0xF3=LR1121, 0xDF=Bootloader) */
    uint16_t version;       /* Firmware version (e.g., 0x0104 = v1.4) */
} __attribute__((packed)) lr1121_fw_version_t;

/*******************************************************************************
 * RX Result enum (matches ELRS SX12xxDriverCommon)
 ******************************************************************************/

typedef enum {
    SX12XX_RX_OK = 0,
    SX12XX_RX_CRC_FAIL,
    SX12XX_RX_TIMEOUT
} SX12XX_RX_Status_t;

/*******************************************************************************
 * Callback Types
 ******************************************************************************/

/**
 * TX done callback - called when transmission completes
 */
typedef void (*lr1121_tx_done_callback_t)(void);

/**
 * RX done callback - called when packet received
 * @param status RX result (OK, CRC_FAIL, TIMEOUT)
 * @return true if packet was valid and processed
 */
typedef bool (*lr1121_rx_done_callback_t)(SX12XX_RX_Status_t status);

/*******************************************************************************
 * Driver State Structure
 ******************************************************************************/

typedef struct {
    /* Initialization state */
    bool initialized;
    
    /* Current frequency */
    uint32_t currFreq;
    
    /* Payload configuration */
    uint8_t PayloadLength;
    bool IQinverted;
    
    /* Modulation mode */
    bool useFSK;
    bool modeSupportsFei;
    
    /* Power management */
    uint8_t pwrCurrentLF;       /* Current SubGHz power */
    uint8_t pwrPendingLF;       /* Pending SubGHz power */
    uint8_t pwrCurrentHF;       /* Current 2.4GHz power */
    uint8_t pwrPendingHF;       /* Pending 2.4GHz power */
    bool pwrForceUpdate;
    
    /* Band tracking per radio */
    bool radio1isSubGHz;
    bool radio2isSubGHz;
    
    /* Fallback mode after TX/RX */
    lr11xx_RadioOperatingModes_t fallBackMode;
    
    /* Radio state for ISR handling */
    SX12XX_Radio_Number_t processingPacketRadio;
    SX12XX_Radio_Number_t transmittingRadio;
    SX12XX_Radio_Number_t strongestReceivingRadio;
    
    /* Second radio packet data (Gemini mode) */
    bool hasSecondRadioGotData;
    
    /* RSSI/SNR from last packet */
    int8_t LastPacketRSSI;
    int8_t LastPacketRSSI2;
    int8_t LastPacketSNRRaw;
    
    /* RX buffers */
    uint8_t rx_buf[LR1121_RX_BUFFER_SIZE];
    uint8_t rx2_buf[LR1121_RX_BUFFER_SIZE];
    uint8_t RXdataBuffer[LR1121_RX_BUFFER_SIZE];
    uint8_t RXdataBufferSecond[LR1121_RX_BUFFER_SIZE];
    
    /* Callbacks */
    lr1121_tx_done_callback_t TXdoneCallback;
    lr1121_rx_done_callback_t RXdoneCallback;
    
    /* SNR fuzzy threshold */
    int8_t FuzzySNRThreshold;
    
} lr1121_driver_state_t;

/*******************************************************************************
 * Global Driver Instance
 ******************************************************************************/

extern lr1121_driver_state_t lr1121_driver;

/*******************************************************************************
 * Driver API Functions (ELRS 4.0 Compatible)
 ******************************************************************************/

/**
 * @brief Initialize the LR1121 driver
 * 
 * Citation: ELRS LR1121Driver::Begin()
 * 
 * @param minimumFrequency Minimum frequency for image calibration
 * @param maximumFrequency Maximum frequency for image calibration
 * @return true on success
 */
bool LR1121Driver_Begin(uint32_t minimumFrequency, uint32_t maximumFrequency);

/**
 * @brief Shutdown the LR1121 driver
 * 
 * Citation: ELRS LR1121Driver::End()
 */
void LR1121Driver_End(void);

/**
 * @brief Configure radio parameters
 * 
 * Citation: ELRS LR1121Driver::Config()
 * 
 * @param bw Bandwidth
 * @param sf Spreading factor
 * @param cr Coding rate
 * @param freq Frequency in Hz
 * @param PreambleLength Preamble length
 * @param InvertIQ IQ inversion
 * @param PayloadLength Payload length
 * @param setFSKModulation true for FSK, false for LoRa
 * @param fskSyncWord1 FSK sync word byte 1
 * @param fskSyncWord2 FSK sync word byte 2
 * @param radioNumber Which radio(s) to configure
 */
void LR1121Driver_Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                         uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
                         bool setFSKModulation, uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                         SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Set RF frequency
 * 
 * Citation: ELRS LR1121Driver::SetFrequencyReg()
 * 
 * @param freq Frequency in Hz
 * @param radioNumber Which radio(s)
 * @param doRx If true, also start RX
 * @param rxTime RX timeout (if doRx)
 */
void LR1121Driver_SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber,
                                   bool doRx, uint32_t rxTime);

/**
 * @brief Set output power (schedules update for next TX)
 * 
 * Citation: ELRS LR1121Driver::SetOutputPower()
 * 
 * @param power Power in dBm
 * @param isSubGHz true for SubGHz band, false for 2.4GHz
 */
void LR1121Driver_SetOutputPower(int8_t power, bool isSubGHz);

/**
 * @brief Start CW (continuous wave) test mode
 * 
 * Citation: ELRS LR1121Driver::startCWTest()
 * 
 * @param freq Frequency in Hz
 * @param radioNumber Which radio
 */
void LR1121Driver_StartCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Non-blocking transmit
 * 
 * Citation: ELRS LR1121Driver::TXnb()
 * 
 * @param data Data to transmit
 * @param sendGeminiBuffer true to send different data on radio 2
 * @param dataGemini Data for radio 2 (if sendGeminiBuffer)
 * @param radioNumber Which radio(s) to transmit on
 */
void LR1121Driver_TXnb(uint8_t *data, bool sendGeminiBuffer, uint8_t *dataGemini,
                        SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Non-blocking receive (continuous RX mode)
 * 
 * Citation: ELRS LR1121Driver::RXnb()
 */
void LR1121Driver_RXnb(void);

/**
 * @brief Get IRQ status
 * 
 * Citation: ELRS LR1121Driver::GetIrqStatus()
 * Note: This also clears the IRQ in one transaction (ELRS optimization)
 * 
 * @param radioNumber Which radio
 * @return IRQ status flags
 */
uint32_t LR1121Driver_GetIrqStatus(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Clear IRQ status
 * 
 * Citation: ELRS LR1121Driver::ClearIrqStatus()
 * 
 * @param radioNumber Which radio(s)
 */
void LR1121Driver_ClearIrqStatus(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Start instantaneous RSSI measurement
 * 
 * Citation: ELRS LR1121Driver::StartRssiInst()
 * 
 * @param radioNumber Which radio
 */
void LR1121Driver_StartRssiInst(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Get instantaneous RSSI result
 * 
 * Citation: ELRS LR1121Driver::GetRssiInst()
 * 
 * @param radioNumber Which radio
 * @return RSSI in dBm
 */
int8_t LR1121Driver_GetRssiInst(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Process packet statistics after RX
 * 
 * Citation: ELRS LR1121Driver::GetLastPacketStats()
 */
void LR1121Driver_GetLastPacketStats(void);

/**
 * @brief Check if second radio received packet (Gemini)
 * 
 * Citation: ELRS LR1121Driver::CheckForSecondPacket()
 */
void LR1121Driver_CheckForSecondPacket(void);

/**
 * @brief Get firmware version
 * 
 * Citation: ELRS LR1121Driver::GetFirmwareVersion()
 * 
 * @param radioNumber Which radio
 * @param command Version command (default: GET_VERSION)
 * @return Firmware version structure
 */
lr1121_fw_version_t LR1121Driver_GetFirmwareVersion(SX12XX_Radio_Number_t radioNumber,
                                                      uint16_t command);

/**
 * @brief Check if frequency error is available
 * 
 * Citation: ELRS LR1121Driver::FrequencyErrorAvailable()
 * 
 * @return false (not supported on LR1121)
 */
bool LR1121Driver_FrequencyErrorAvailable(void);

/**
 * @brief Get frequency error status
 * 
 * Citation: ELRS LR1121Driver::GetFrequencyErrorbool()
 * 
 * @param radioNumber Which radio
 * @return false (not supported)
 */
bool LR1121Driver_GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber);

/**
 * @brief Set TX idle mode (FS mode for faster TX turnaround)
 * 
 * Citation: ELRS LR1121Driver::SetTxIdleMode()
 */
void LR1121Driver_SetTxIdleMode(void);

/*******************************************************************************
 * Callback Registration
 ******************************************************************************/

/**
 * @brief Set TX done callback
 * 
 * @param callback Function to call when TX completes
 */
void LR1121Driver_SetTXDoneCallback(lr1121_tx_done_callback_t callback);

/**
 * @brief Set RX done callback
 * 
 * @param callback Function to call when packet received
 */
void LR1121Driver_SetRXDoneCallback(lr1121_rx_done_callback_t callback);

/**
 * @brief Remove all callbacks
 * 
 * Citation: ELRS SX12xxDriverCommon::RemoveCallbacks()
 */
void LR1121Driver_RemoveCallbacks(void);

/*******************************************************************************
 * Internal Functions (exposed for ISR handlers)
 ******************************************************************************/

/**
 * @brief ISR callback for Radio 1
 * Called from HAL when DIO1 interrupt fires
 */
void LR1121Driver_IsrCallback_1(void);

/**
 * @brief ISR callback for Radio 2
 * Called from HAL when DIO1_2 interrupt fires
 */
void LR1121Driver_IsrCallback_2(void);

/*******************************************************************************
 * Hardware Configuration (platform-specific)
 * 
 * These should be defined in your platform's targets.h or similar
 ******************************************************************************/

#ifndef GPIO_PIN_NSS_2
#define GPIO_PIN_NSS_2              UNDEF_PIN
#endif

#ifndef GPIO_PIN_BUSY_2
#define GPIO_PIN_BUSY_2             UNDEF_PIN
#endif

#ifndef GPIO_PIN_DIO1_2
#define GPIO_PIN_DIO1_2             UNDEF_PIN
#endif

#ifndef GPIO_PIN_RST_2
#define GPIO_PIN_RST_2              UNDEF_PIN
#endif

/* RF Switch Control - 8 bytes for DIO configuration
 * Citation: ELRS targets.h LR1121_RFSW_CTRL array
 */
#ifndef LR1121_RFSW_CTRL_COUNT
#define LR1121_RFSW_CTRL_COUNT      0
#endif

/* Hardware DCDC regulator option */
#ifndef OPT_USE_HARDWARE_DCDC
#define OPT_USE_HARDWARE_DCDC       0
#endif

/* Use SX1276 RFO HF PA (low power PA) */
#ifndef OPT_USE_SX1276_RFO_HF
#define OPT_USE_SX1276_RFO_HF       0
#endif

#ifdef __cplusplus
}
#endif

#endif /* LR1121_DRIVER_ELRS_H */
