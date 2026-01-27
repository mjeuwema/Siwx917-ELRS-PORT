/**
 * @file LR1121Driver.c
 * @brief ELRS 4.0 Compatible LR1121 Radio Driver Implementation (C)
 * 
 * This is a direct C port of ExpressLRS 4.0 src/lib/LR1121Driver/LR1121.cpp
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp
 * https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/LR1121Driver/LR1121.cpp
 */

#include "LR1121Driver.h"
#include "lr1121_hal.h"
#include "lr1121_regs.h"
#include "rsi_debug.h"

#include <string.h>

/*******************************************************************************
 * Debug Logging
 ******************************************************************************/

#define DRIVER_DEBUG_ENABLE     1

#if DRIVER_DEBUG_ENABLE
    #define DBGLN(fmt, ...)     DEBUGOUT("[LR1121] " fmt "\n", ##__VA_ARGS__)
#else
    #define DBGLN(fmt, ...)     ((void)0)
#endif

/*******************************************************************************
 * Global Driver Instance
 ******************************************************************************/

lr1121_driver_state_t lr1121_driver = {
    .initialized = false,
    .currFreq = 0,
    .PayloadLength = 8,
    .IQinverted = false,
    .useFSK = false,
    .modeSupportsFei = false,
    .pwrCurrentLF = 0,
    .pwrPendingLF = PWRPENDING_NONE,
    .pwrCurrentHF = 0,
    .pwrPendingHF = PWRPENDING_NONE,
    .pwrForceUpdate = false,
    .radio1isSubGHz = true,
    .radio2isSubGHz = true,
    .fallBackMode = LR1121_MODE_FS,
    .processingPacketRadio = SX12XX_Radio_1,
    .transmittingRadio = SX12XX_Radio_1,
    .strongestReceivingRadio = SX12XX_Radio_1,
    .hasSecondRadioGotData = false,
    .LastPacketRSSI = 0,
    .LastPacketRSSI2 = 0,
    .LastPacketSNRRaw = 0,
    .TXdoneCallback = NULL,
    .RXdoneCallback = NULL,
    .FuzzySNRThreshold = 0
};

/*******************************************************************************
 * Forward Declarations - Internal Functions
 ******************************************************************************/

static void SetMode(lr11xx_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber);
static void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr, SX12XX_Radio_Number_t radioNumber);
static void SetPacketParamsLoRa(uint8_t PreambleLength, lr11xx_RadioLoRaPacketLengthsModes_t HeaderType,
                                 uint8_t PayloadLength, uint8_t InvertIQ, SX12XX_Radio_Number_t radioNumber);
static void ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF, uint32_t Fdev, SX12XX_Radio_Number_t radioNumber);
static void SetPacketParamsFSK(uint8_t PreambleLength, uint8_t PayloadLength, SX12XX_Radio_Number_t radioNumber);
static void SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2, SX12XX_Radio_Number_t radioNumber);
static void SetDioIrqParams(void);
static void SetDioAsRfSwitch(void);
static void CorrectRegisterForSF6(uint8_t sf, SX12XX_Radio_Number_t radioNumber);
static void CommitOutputPower(void);
static void WriteOutputPower(uint8_t power, bool isSubGHz, SX12XX_Radio_Number_t radioNumber);
static void SetPaConfig(bool isSubGHz, SX12XX_Radio_Number_t radioNumber);
static void DecodeRssiSnr(SX12XX_Radio_Number_t radioNumber, const uint8_t *buf);
static bool RXnbISR(SX12XX_Radio_Number_t radioNumber);
static void TXnbISR(void);
static void IsrCallback(SX12XX_Radio_Number_t radioNumber);
static int8_t fuzzy_snr(int8_t snr1, int8_t snr2, int8_t threshold);

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static inline int8_t constrain_int8(int8_t val, int8_t min, int8_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief Fuzzy SNR calculation for diversity
 * 
 * Citation: ELRS common.h fuzzy_snr()
 */
static int8_t fuzzy_snr(int8_t snr1, int8_t snr2, int8_t threshold)
{
    (void)threshold;
    /* Simple: return better SNR */
    return (snr1 > snr2) ? snr1 : snr2;
}

/*******************************************************************************
 * Initialization / Shutdown
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp Begin() - lines 108-158
 */
bool LR1121Driver_Begin(uint32_t minimumFrequency, uint32_t maximumFrequency)
{
    lr1121_hal_status_t hal_status;
    
    DBGLN("Begin(%lu, %lu)", (unsigned long)minimumFrequency, (unsigned long)maximumFrequency);
    
    /* Initialize HAL */
    hal_status = lr1121_hal_init();
    if (hal_status != LR1121_HAL_OK) {
        DBGLN("HAL init failed: %d", hal_status);
        return false;
    }
    
    /* Reset the radio */
    hal_status = lr1121_hal_reset(false);
    if (hal_status != LR1121_HAL_OK) {
        DBGLN("Reset failed: %d", hal_status);
        return false;
    }
    
    /* Validate LR1121 version */
    lr1121_fw_version_t version = LR1121Driver_GetFirmwareVersion(SX12XX_Radio_1, LR11XX_SYSTEM_GET_VERSION_OC);
    DBGLN("LR1121 #1: HW=%02X Type=%02X FW=%04X", version.hardware, version.type, version.version);
    
    /* Register ISR callbacks with HAL */
    lr1121_hal_set_isr_callback(LR1121Driver_IsrCallback_1, SX12XX_Radio_1);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
        lr1121_hal_set_isr_callback(LR1121Driver_IsrCallback_2, SX12XX_Radio_2);
    }
    
    /* Clear errors */
    lr1121_hal_write_command(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_All);
    
    /* Set fallback mode to FS (frequency synthesis)
     * Citation: ELRS LR1121.cpp line 127-135
     * Do not enable for dual radio TX - causes timing issues */
    uint8_t FBbuf[1] = {LR11XX_RADIO_FALLBACK_FS};
    lr1121_driver.fallBackMode = LR1121_MODE_FS;
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC, FBbuf, 1, SX12XX_Radio_All);
    
    /* Enable RX boosted mode */
    uint8_t abuf[1] = {1};
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_BOOSTED_OC, abuf, 1, SX12XX_Radio_All);
    
    /* Configure RF switch DIOs */
    SetDioAsRfSwitch();
    
    /* Configure DIO IRQ parameters */
    SetDioIrqParams();
    
    /* Enable DC-DC if configured */
    if (OPT_USE_HARDWARE_DCDC) {
        uint8_t RegMode[1] = {1};
        lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_REGMODE_OC, RegMode, 1, SX12XX_Radio_All);
    }
    
    /* Calibrate image for frequency range */
    uint8_t CalImagebuf[2];
    CalImagebuf[0] = ((minimumFrequency / 1000000) - 1) / 4;
    CalImagebuf[1] = 1 + ((maximumFrequency / 1000000) + 1) / 4;
    lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CALIBRATE_IMAGE_OC, CalImagebuf, 2, SX12XX_Radio_All);
    
    lr1121_driver.initialized = true;
    DBGLN("Begin complete");
    
    return true;
}

/**
 * Citation: ELRS LR1121.cpp End() - lines 64-69
 */
void LR1121Driver_End(void)
{
    DBGLN("End");
    SetMode(LR1121_MODE_SLEEP, SX12XX_Radio_All);
    lr1121_hal_end();
    LR1121Driver_RemoveCallbacks();
    lr1121_driver.initialized = false;
}

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp Config() - lines 168-239
 */
void LR1121Driver_Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                         uint8_t PreambleLength, bool InvertIQ, uint8_t _PayloadLength,
                         bool setFSKModulation, uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                         SX12XX_Radio_Number_t radioNumber)
{
    lr1121_driver.PayloadLength = _PayloadLength;
    
    bool isSubGHz = freq < 1000000000UL;
    
    if (radioNumber & SX12XX_Radio_1) {
        lr1121_driver.radio1isSubGHz = isSubGHz;
    }
    if (radioNumber & SX12XX_Radio_2) {
        lr1121_driver.radio2isSubGHz = isSubGHz;
    }
    
    lr1121_driver.IQinverted = InvertIQ;
    lr11xx_radio_lora_iq_t inverted = InvertIQ ? LR11XX_RADIO_LORA_IQ_INVERTED : LR11XX_RADIO_LORA_IQ_STANDARD;
    
    /* IQ is always STANDARD for 900MHz band */
    if (isSubGHz) {
        inverted = LR11XX_RADIO_LORA_IQ_STANDARD;
    }
    
    SetMode(LR1121_MODE_STDBY_RC, radioNumber);
    
    lr1121_driver.useFSK = setFSKModulation;
    
    /* Set packet type */
    uint8_t pktType = setFSKModulation ? LR11XX_RADIO_PKT_TYPE_GFSK : LR11XX_RADIO_PKT_TYPE_LORA;
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_TYPE_OC, &pktType, 1, radioNumber);
    
    if (setFSKModulation) {
        DBGLN("Config FSK");
        uint32_t bitrate = (uint32_t)bw * 10000;
        uint8_t bwf = sf;
        uint32_t fdev = (uint32_t)cr * 1000;
        ConfigModParamsFSK(bitrate, bwf, fdev, radioNumber);
        SetPacketParamsFSK(PreambleLength, lr1121_driver.PayloadLength, radioNumber);
        SetFSKSyncWord(fskSyncWord1, fskSyncWord2, radioNumber);
    } else {
        DBGLN("Config LoRa");
        ConfigModParamsLoRa(bw, sf, cr, radioNumber);
        
        lr11xx_RadioLoRaPacketLengthsModes_t packetLengthType = LR1121_LORA_PACKET_FIXED_LENGTH;
        SetPacketParamsLoRa(PreambleLength, packetLengthType, lr1121_driver.PayloadLength, inverted, radioNumber);
    }
    
    LR1121Driver_SetFrequencyReg(freq, radioNumber, false, 0);
    
    LR1121Driver_ClearIrqStatus(radioNumber);
    
    SetPaConfig(isSubGHz, radioNumber);
    lr1121_driver.pwrForceUpdate = true;
    CommitOutputPower();
}

/**
 * Citation: ELRS LR1121.cpp ConfigModParamsLoRa() - lines 500-518
 */
static void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4];
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = 0x00;  /* LowDataRateOptimize off */
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_MODULATION_PARAM_OC, buf, 4, radioNumber);
    
    if ((radioNumber & SX12XX_Radio_1) && lr1121_driver.radio1isSubGHz) {
        CorrectRegisterForSF6(sf, SX12XX_Radio_1);
    }
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
        if ((radioNumber & SX12XX_Radio_2) && lr1121_driver.radio2isSubGHz) {
            CorrectRegisterForSF6(sf, SX12XX_Radio_2);
        }
    }
}

/**
 * Citation: ELRS LR1121.cpp SetPacketParamsLoRa() - lines 520-532
 */
static void SetPacketParamsLoRa(uint8_t PreambleLength, lr11xx_RadioLoRaPacketLengthsModes_t HeaderType,
                                 uint8_t PayloadLength, uint8_t InvertIQ, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[6];
    buf[0] = 0;                 /* MSB PreambleLength */
    buf[1] = PreambleLength;    /* LSB PreambleLength */
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = LR11XX_RADIO_LORA_CRC_OFF;
    buf[5] = InvertIQ;
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_PARAM_OC, buf, 6, radioNumber);
}

/**
 * Citation: ELRS LR1121.cpp ConfigModParamsFSK() - lines 241-256
 */
static void ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF, uint32_t Fdev, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[10];
    buf[0] = (Bitrate >> 24) & 0xFF;
    buf[1] = (Bitrate >> 16) & 0xFF;
    buf[2] = (Bitrate >> 8) & 0xFF;
    buf[3] = Bitrate & 0xFF;
    buf[4] = LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF;
    buf[5] = BWF;
    buf[6] = (Fdev >> 24) & 0xFF;
    buf[7] = (Fdev >> 16) & 0xFF;
    buf[8] = (Fdev >> 8) & 0xFF;
    buf[9] = Fdev & 0xFF;
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_MODULATION_PARAM_OC, buf, 10, radioNumber);
}

/**
 * Citation: ELRS LR1121.cpp SetPacketParamsFSK() - lines 258-272
 */
static void SetPacketParamsFSK(uint8_t PreambleLength, uint8_t PayloadLength, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[9];
    buf[0] = 0;                                             /* MSB PreambleLength */
    buf[1] = PreambleLength;                                /* LSB PreambleLength */
    buf[2] = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
    buf[3] = 16;                                            /* SyncWordLen in bits */
    buf[4] = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
    buf[5] = LR11XX_RADIO_GFSK_PKT_FIX_LEN;
    buf[6] = PayloadLength;
    buf[7] = LR11XX_RADIO_GFSK_CRC_OFF;
    buf[8] = LR11XX_RADIO_GFSK_DC_FREE_WHITENING;
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_PARAM_OC, buf, 9, radioNumber);
}

/**
 * Citation: ELRS LR1121.cpp SetFSKSyncWord() - lines 274-280
 */
static void SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t synbuf[8] = {fskSyncWord1, fskSyncWord2, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC, synbuf, 8, radioNumber);
}

/**
 * Citation: ELRS LR1121.cpp SetDioAsRfSwitch() - lines 282-309
 */
static void SetDioAsRfSwitch(void)
{
    uint8_t switchbuf[8];
    
#if LR1121_RFSW_CTRL_COUNT == 8
    extern const uint8_t LR1121_RFSW_CTRL[8];
    switchbuf[0] = LR1121_RFSW_CTRL[0];  /* RfswEnable */
    switchbuf[1] = LR1121_RFSW_CTRL[1];  /* RfSwStbyCfg */
    switchbuf[2] = LR1121_RFSW_CTRL[2];  /* RfSwRxCfg */
    switchbuf[3] = LR1121_RFSW_CTRL[3];  /* RfSwTxCfg */
    switchbuf[4] = LR1121_RFSW_CTRL[4];  /* RfSwTxHPCfg */
    switchbuf[5] = LR1121_RFSW_CTRL[5];  /* RfSwTxHfCfg */
    switchbuf[6] = LR1121_RFSW_CTRL[6];  /* Unused */
    switchbuf[7] = LR1121_RFSW_CTRL[7];  /* RfSwWifiCfg */
#else
    /* WAVESHARE Core1121-HF configuration - CORRECTED for PE4259
     * 
     * PE4259 RF switch wiring on Core1121-HF:
     *   DIO5 (bit0) → 100Ω → VDD (switch power enable)
     *   DIO6 (bit1) → 100Ω → CTRL (path selector)
     *   RF1 ← RFO_HP_LF / RFO_LP_LF (LR1121 TX output)
     *   RF2 ← RFI_P_LF / RFI_N_LF (LR1121 RX input)
     *
     * PE4259 truth table:
     *   CTRL=LOW  → RFC to RF1 → TX path
     *   CTRL=HIGH → RFC to RF2 → RX path
     *
     * DIO bit mapping: bit0=DIO5, bit1=DIO6
     * DIO5 must be HIGH to power the switch in all modes!
     */
    switchbuf[0] = 0b00000011;  /* RfswEnable: DIO5+DIO6 */
    switchbuf[1] = 0b00000001;  /* RfSwStbyCfg: DIO5=1 (power ON), DIO6=0 */
    switchbuf[2] = 0b00000011;  /* RfSwRxCfg: DIO5=1 (power), DIO6=1 (CTRL HIGH) → RF2 → RX */
    switchbuf[3] = 0b00000001;  /* RfSwTxCfg: DIO5=1 (power), DIO6=0 (CTRL LOW) → RF1 → TX */
    switchbuf[4] = 0b00000001;  /* RfSwTxHPCfg: DIO5=1 (power), DIO6=0 (CTRL LOW) → RF1 → TX */
    switchbuf[5] = 0b00000001;  /* RfSwTxHfCfg: DIO5=1 (power), DIO6=0 */
    switchbuf[6] = 0;           /* Unused */
    switchbuf[7] = 0b00000001;  /* RfSwWifiCfg: DIO5=1 (power), DIO6=0 */
#endif
    
    lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC, switchbuf, 8, SX12XX_Radio_All);
}

/**
 * Citation: ELRS LR1121.cpp CorrectRegisterForSF6() - lines 311-339
 */
static void CorrectRegisterForSF6(uint8_t sf, SX12XX_Radio_Number_t radioNumber)
{
    if ((lr11xx_radio_lora_sf_t)sf == LR11XX_RADIO_LORA_SF6) {
        uint8_t wrbuf[12];
        /* Address 0x00f20414 */
        wrbuf[0] = 0x00;
        wrbuf[1] = 0xf2;
        wrbuf[2] = 0x04;
        wrbuf[3] = 0x14;
        /* Mask: bit18=1, bit23=0 */
        wrbuf[4] = 0x00;
        wrbuf[5] = 0b10000100;
        wrbuf[6] = 0x00;
        wrbuf[7] = 0x00;
        /* Data: bit18=1, bit23=0 */
        wrbuf[8] = 0x00;
        wrbuf[9] = 0b00000100;
        wrbuf[10] = 0x00;
        wrbuf[11] = 0x00;
        lr1121_hal_write_command_with_data(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, 12, radioNumber);
    }
}

/**
 * Citation: ELRS LR1121.cpp SetDioIrqParams() - lines 559-564
 */
static void SetDioIrqParams(void)
{
    uint8_t buf[8] = {0};
    buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
    lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, buf, 8, SX12XX_Radio_All);
}

/*******************************************************************************
 * Mode Control
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp SetMode() - lines 451-498
 */
static void SetMode(lr11xx_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[5] = {0};
    
    switch (OPmode) {
    case LR1121_MODE_SLEEP:
        lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_SLEEP_OC, buf, 5, radioNumber);
        break;
        
    case LR1121_MODE_STDBY_RC:
        buf[0] = 0x00;
        lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC, buf, 1, radioNumber);
        break;
        
    case LR1121_MODE_STDBY_XOSC:
        buf[0] = 0x01;
        lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC, buf, 1, radioNumber);
        break;
        
    case LR1121_MODE_FS:
        lr1121_hal_write_command(LR11XX_SYSTEM_SET_FS_OC, radioNumber);
        break;
        
    case LR1121_MODE_RX_CONT:
        buf[0] = 0xFF;
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_OC, buf, 3, radioNumber);
        break;
        
    case LR1121_MODE_TX:
        lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_TX_OC, buf, 3, radioNumber);
        break;
        
    case LR1121_MODE_CAD:
        /* Not implemented */
        break;
        
    default:
        break;
    }
}

void LR1121Driver_SetTxIdleMode(void)
{
    SetMode(LR1121_MODE_FS, SX12XX_Radio_All);
}

/*******************************************************************************
 * Frequency Control
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp SetFrequencyReg() - lines 534-557
 */
void LR1121Driver_SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber,
                                   bool doRx, uint32_t rxTime)
{
    (void)rxTime;
    
    uint8_t buf[7] = {
        (uint8_t)(freq >> 24),
        (uint8_t)(freq >> 16),
        (uint8_t)(freq >> 8),
        (uint8_t)(freq),
        0xFF,
        0xFF,
        0xFF,
    };
    
    if (doRx) {
        /* SetRfFrequency_SetRX combined command */
        lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_FREQ_SET_RX, buf, 7, radioNumber);
    } else {
        /* SetRfFrequency only */
        lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RF_FREQUENCY_OC, buf, 4, radioNumber);
    }
    
    lr1121_driver.currFreq = freq;
}

/*******************************************************************************
 * Power Control
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp SetOutputPower() - lines 344-373
 */
void LR1121Driver_SetOutputPower(int8_t power, bool isSubGHz)
{
    uint8_t pwrNew;
    
    if (isSubGHz) {
        if (OPT_USE_SX1276_RFO_HF) {
            pwrNew = constrain_int8(power, LR1121_POWER_MIN_LP_PA, LR1121_POWER_MAX_LP_PA);
        } else {
            pwrNew = constrain_int8(power, LR1121_POWER_MIN_HP_PA, LR1121_POWER_MAX_HP_PA);
        }
        
        if ((lr1121_driver.pwrPendingLF == PWRPENDING_NONE && lr1121_driver.pwrCurrentLF != pwrNew) ||
            lr1121_driver.pwrPendingLF != pwrNew) {
            lr1121_driver.pwrPendingLF = pwrNew;
        }
    } else {
        pwrNew = constrain_int8(power, LR1121_POWER_MIN_HF_PA, LR1121_POWER_MAX_HF_PA);
        
        if ((lr1121_driver.pwrPendingHF == PWRPENDING_NONE && lr1121_driver.pwrCurrentHF != pwrNew) ||
            lr1121_driver.pwrPendingHF != pwrNew) {
            lr1121_driver.pwrPendingHF = pwrNew;
        }
    }
}

/**
 * Citation: ELRS LR1121.cpp CommitOutputPower() - lines 375-400
 */
static void CommitOutputPower(void)
{
    if (lr1121_driver.pwrPendingLF != PWRPENDING_NONE) {
        lr1121_driver.pwrCurrentLF = lr1121_driver.pwrPendingLF;
        lr1121_driver.pwrPendingLF = PWRPENDING_NONE;
        lr1121_driver.pwrForceUpdate = true;
    }
    
    if (lr1121_driver.pwrPendingHF != PWRPENDING_NONE) {
        lr1121_driver.pwrCurrentHF = lr1121_driver.pwrPendingHF;
        lr1121_driver.pwrPendingHF = PWRPENDING_NONE;
        lr1121_driver.pwrForceUpdate = true;
    }
    
    if (lr1121_driver.pwrForceUpdate) {
        WriteOutputPower(lr1121_driver.radio1isSubGHz ? lr1121_driver.pwrCurrentLF : lr1121_driver.pwrCurrentHF,
                         lr1121_driver.radio1isSubGHz, SX12XX_Radio_1);
        if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
            WriteOutputPower(lr1121_driver.radio2isSubGHz ? lr1121_driver.pwrCurrentLF : lr1121_driver.pwrCurrentHF,
                             lr1121_driver.radio2isSubGHz, SX12XX_Radio_2);
        }
        lr1121_driver.pwrForceUpdate = false;
    }
}

/**
 * Citation: ELRS LR1121.cpp WriteOutputPower() - lines 402-408
 */
static void WriteOutputPower(uint8_t power, bool isSubGHz, SX12XX_Radio_Number_t radioNumber)
{
    (void)isSubGHz;
    uint8_t Txbuf[2] = {power, LR11XX_RADIO_RAMP_48_US};
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_TX_PARAMS_OC, Txbuf, 2, radioNumber);
}

/**
 * Citation: ELRS LR1121.cpp SetPaConfig() - lines 410-449
 */
static void SetPaConfig(bool isSubGHz, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t Pabuf[4] = {0};
    
    if (isSubGHz) {
        if (OPT_USE_SX1276_RFO_HF) {
            /* 900M low power RF Amp */
            Pabuf[0] = LR11XX_RADIO_PA_SEL_LP;
            Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
            Pabuf[2] = 0x07;  /* PaDutyCycle */
        } else {
            /* 900M high power RF Amp */
            Pabuf[0] = LR11XX_RADIO_PA_SEL_HP;
            Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
            Pabuf[2] = 0x04;  /* PaDutyCycle */
            Pabuf[3] = 0x07;  /* PaHPSel - for +22dBm */
        }
    } else {
        /* 2.4G RF Amp */
        Pabuf[0] = LR11XX_RADIO_PA_SEL_HF;
        Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        Pabuf[2] = 0x00;  /* PaDutyCycle */
        Pabuf[3] = 0x00;  /* PaHPSel */
    }
    
    lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PA_CFG_OC, Pabuf, 4, radioNumber);
}

/*******************************************************************************
 * TX / RX Operations
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp TXnb() - lines 601-662
 */
void LR1121Driver_TXnb(uint8_t *data, bool sendGeminiBuffer, uint8_t *dataGemini,
                        SX12XX_Radio_Number_t radioNumber)
{
    lr1121_driver.transmittingRadio = radioNumber;
    
    if (radioNumber == SX12XX_Radio_NONE) {
        SetMode(lr1121_driver.fallBackMode, SX12XX_Radio_All);
        return;
    }
    
    /* For normal diversity mode, put unused radio in FS mode */
    if (GPIO_PIN_NSS_2 != UNDEF_PIN && radioNumber != SX12XX_Radio_All) {
        if (radioNumber == SX12XX_Radio_1) {
            SetMode(lr1121_driver.fallBackMode, SX12XX_Radio_2);
        } else {
            SetMode(lr1121_driver.fallBackMode, SX12XX_Radio_1);
        }
    }
    
    uint8_t outBuffer[32] = {0};
    uint8_t length = lr1121_driver.PayloadLength + 3;  /* 3 extra bytes for timeout */
    
    /* Copy data (no FEC encoding in this implementation) */
    memcpy(outBuffer, data, lr1121_driver.PayloadLength);
    
    if (sendGeminiBuffer) {
        lr1121_hal_write_command_with_data(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length, SX12XX_Radio_1);
        memcpy(outBuffer, dataGemini, lr1121_driver.PayloadLength);
        lr1121_hal_write_command_with_data(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length, SX12XX_Radio_2);
    } else {
        lr1121_hal_write_command_with_data(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length, radioNumber);
    }
}

/**
 * Citation: ELRS LR1121.cpp RXnb() - lines 708-711
 */
void LR1121Driver_RXnb(void)
{
    SetMode(LR1121_MODE_RX_CONT, SX12XX_Radio_All);
}

/**
 * Citation: ELRS LR1121.cpp startCWTest() - lines 161-166
 */
void LR1121Driver_StartCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber)
{
    LR1121Driver_Config(LR11XX_RADIO_LORA_BW_62, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_8,
                        freq, 12, false, 8, false, 0, 0, radioNumber);
    lr1121_hal_write_command(LR11XX_RADIO_SET_TX_CW_OC, radioNumber);
}

/*******************************************************************************
 * IRQ Handling
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp GetIrqStatus() - lines 567-579
 * This also clears IRQ in one transaction
 */
uint32_t LR1121Driver_GetIrqStatus(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status[6] = {0};
    status[0] = (LR11XX_SYSTEM_CLEAR_IRQ_OC >> 8) & 0xFF;
    status[1] = LR11XX_SYSTEM_CLEAR_IRQ_OC & 0xFF;
    status[2] = 0xFF;
    status[3] = 0xFF;
    status[4] = 0xFF;
    status[5] = 0xFF;
    lr1121_hal_read_command(status, 6, radioNumber);
    return ((uint32_t)status[2] << 24) | ((uint32_t)status[3] << 16) | 
           ((uint32_t)status[4] << 8) | (uint32_t)status[5];
}

/**
 * Citation: ELRS LR1121.cpp ClearIrqStatus() - lines 581-589
 */
void LR1121Driver_ClearIrqStatus(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CLEAR_IRQ_OC, buf, 4, radioNumber);
}

/*******************************************************************************
 * ISR Callbacks
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp TXnbISR() - lines 591-599
 */
static void TXnbISR(void)
{
    CommitOutputPower();
    if (lr1121_driver.TXdoneCallback) {
        lr1121_driver.TXdoneCallback();
    }
}

/**
 * Citation: ELRS LR1121.cpp RXnbISR() - lines 692-706
 */
static bool RXnbISR(SX12XX_Radio_Number_t radioNumber)
{
    /* Get packet using optimized command */
    lr1121_hal_write_command(LR11XX_RADIO_GET_PACKET, radioNumber);
    lr1121_hal_read_command(lr1121_driver.rx_buf, lr1121_driver.PayloadLength + 6, radioNumber);
    
    /* Copy payload (skip 6 header bytes) */
    memcpy(lr1121_driver.RXdataBuffer, lr1121_driver.rx_buf + 6, lr1121_driver.PayloadLength);
    
    if (lr1121_driver.RXdoneCallback) {
        if (!lr1121_driver.RXdoneCallback(SX12XX_RX_OK)) {
            return false;
        }
    }
    return true;
}

/**
 * Citation: ELRS LR1121.cpp IsrCallback() - lines 792-810
 */
static void IsrCallback(SX12XX_Radio_Number_t radioNumber)
{
    lr1121_driver.processingPacketRadio = radioNumber;
    SX12XX_Radio_Number_t otherRadioNumber = (radioNumber == SX12XX_Radio_1) ? SX12XX_Radio_2 : SX12XX_Radio_1;
    
    uint32_t irqStatus = LR1121Driver_GetIrqStatus(radioNumber);
    
    if (irqStatus & LR1121_IRQ_TX_DONE) {
        TXnbISR();
        if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
            LR1121Driver_ClearIrqStatus(otherRadioNumber);
        }
    } else if (irqStatus & LR1121_IRQ_RX_DONE) {
        RXnbISR(radioNumber);
    }
}

void LR1121Driver_IsrCallback_1(void)
{
    IsrCallback(SX12XX_Radio_1);
}

void LR1121Driver_IsrCallback_2(void)
{
    IsrCallback(SX12XX_Radio_2);
}

/*******************************************************************************
 * RSSI / SNR Functions
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp DecodeRssiSnr() - lines 664-690
 */
static void DecodeRssiSnr(SX12XX_Radio_Number_t radioNumber, const uint8_t *buf)
{
    /* RSSI = -RssiPkt/2 (dBm) */
    int8_t rssi = -(int8_t)(buf[lr1121_driver.useFSK ? 3 : 5] / 2);
    
    if (radioNumber == SX12XX_Radio_1) {
        lr1121_driver.LastPacketRSSI = rssi;
    } else {
        lr1121_driver.LastPacketRSSI2 = rssi;
    }
    
    lr1121_driver.LastPacketSNRRaw = lr1121_driver.useFSK ? 0 : (int8_t)buf[4];
}

void LR1121Driver_StartRssiInst(SX12XX_Radio_Number_t radioNumber)
{
    lr1121_hal_write_command(LR11XX_RADIO_GET_RSSI_INST_OC, radioNumber);
}

int8_t LR1121Driver_GetRssiInst(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status[2] = {0};
    lr1121_hal_read_command(status, 2, radioNumber);
    return -(int8_t)(status[1] / 2);
}

/**
 * Citation: ELRS LR1121.cpp GetLastPacketStats() - lines 750-780
 */
void LR1121Driver_GetLastPacketStats(void)
{
    SX12XX_Radio_Number_t radioNumber = (lr1121_driver.processingPacketRadio == SX12XX_Radio_1) ? 
                                         SX12XX_Radio_2 : SX12XX_Radio_1;
    
    lr1121_driver.strongestReceivingRadio = lr1121_driver.processingPacketRadio;
    DecodeRssiSnr(lr1121_driver.processingPacketRadio, lr1121_driver.rx_buf);
    
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
        if (lr1121_driver.hasSecondRadioGotData) {
            int8_t firstSNR = lr1121_driver.LastPacketSNRRaw;
            DecodeRssiSnr(radioNumber, lr1121_driver.rx2_buf);
            lr1121_driver.LastPacketSNRRaw = fuzzy_snr(lr1121_driver.LastPacketSNRRaw, firstSNR, 
                                                        lr1121_driver.FuzzySNRThreshold);
            lr1121_driver.strongestReceivingRadio = (lr1121_driver.LastPacketRSSI > lr1121_driver.LastPacketRSSI2) ? 
                                                     SX12XX_Radio_1 : SX12XX_Radio_2;
        }
    }
}

/**
 * Citation: ELRS LR1121.cpp CheckForSecondPacket() - lines 731-748
 */
void LR1121Driver_CheckForSecondPacket(void)
{
    lr1121_driver.hasSecondRadioGotData = false;
    
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
        uint8_t processingRadioIdx = (lr1121_driver.processingPacketRadio == SX12XX_Radio_1) ? 0 : 1;
        uint8_t secondRadioIdx = !processingRadioIdx;
        SX12XX_Radio_Number_t radio[2] = {SX12XX_Radio_1, SX12XX_Radio_2};
        
        uint32_t secondIrqStatus = LR1121Driver_GetIrqStatus(radio[secondRadioIdx]);
        if (secondIrqStatus & LR1121_IRQ_RX_DONE) {
            lr1121_hal_write_command(LR11XX_RADIO_GET_PACKET, radio[secondRadioIdx]);
            lr1121_hal_read_command(lr1121_driver.rx2_buf, lr1121_driver.PayloadLength + 6, radio[secondRadioIdx]);
            memcpy(lr1121_driver.RXdataBufferSecond, lr1121_driver.rx2_buf + 6, lr1121_driver.PayloadLength);
            lr1121_driver.hasSecondRadioGotData = true;
        }
    }
}

/*******************************************************************************
 * Version / Status Functions
 ******************************************************************************/

/**
 * Citation: ELRS LR1121.cpp GetFirmwareVersion() - lines 825-837
 */
lr1121_fw_version_t LR1121Driver_GetFirmwareVersion(SX12XX_Radio_Number_t radioNumber, uint16_t command)
{
    uint8_t buffer[5] = {0};
    lr1121_fw_version_t version = {0};
    
    lr1121_hal_write_command(command, radioNumber);
    lr1121_hal_read_command(buffer, 5, radioNumber);
    lr1121_hal_wait_on_busy(radioNumber);
    
    version.hardware = buffer[1];
    version.type = buffer[2];
    version.version = ((uint16_t)buffer[3] << 8) | buffer[4];
    
    return version;
}

bool LR1121Driver_FrequencyErrorAvailable(void)
{
    return false;  /* Not supported on LR1121 */
}

bool LR1121Driver_GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber)
{
    (void)radioNumber;
    return false;  /* Not supported on LR1121 */
}

/*******************************************************************************
 * Callback Management
 ******************************************************************************/

void LR1121Driver_SetTXDoneCallback(lr1121_tx_done_callback_t callback)
{
    lr1121_driver.TXdoneCallback = callback;
}

void LR1121Driver_SetRXDoneCallback(lr1121_rx_done_callback_t callback)
{
    lr1121_driver.RXdoneCallback = callback;
}

void LR1121Driver_RemoveCallbacks(void)
{
    lr1121_driver.TXdoneCallback = NULL;
    lr1121_driver.RXdoneCallback = NULL;
    lr1121_hal_set_isr_callback(NULL, SX12XX_Radio_All);
}
