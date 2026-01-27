/**
 * @file lr1121_regs.h
 * @brief LR1121 Register Definitions and Constants for ELRS Protocol
 * 
 * Ported from ELRS 4.0 src/lib/LR1121Driver/LR1121_Regs.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * These definitions are required for ELRS protocol compatibility.
 * 
 * Citation: LR11xx User Manual - Semtech
 * Citation: ExpressLRS 4.0 LR1121_Regs.h
 */

#ifndef LR1121_REGS_H
#define LR1121_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Crystal Frequency
 ******************************************************************************/

#define LR1121_XTAL_FREQ        32000000

/*******************************************************************************
 * Power Limits
 * 
 * Citation: LR1121 Datasheet Section 6.1.1
 ******************************************************************************/

#define LR1121_POWER_MIN_LP_PA  (-17)   /* Low Power PA */
#define LR1121_POWER_MAX_LP_PA  (14)
#define LR1121_POWER_MIN_HP_PA  (-9)    /* High Power PA */
#define LR1121_POWER_MAX_HP_PA  (22)
#define LR1121_POWER_MIN_HF_PA  (-18)   /* High Frequency PA (2.4 GHz) */
#define LR1121_POWER_MAX_HF_PA  (13)

/*******************************************************************************
 * IRQ Flags
 * 
 * Citation: LR1121 User Manual Rev 1.2, Section 4.1, Table 4-2 "IrqToEnable Interruption Mapping"
 * 
 * IMPORTANT: These bit positions differ from SX126x/LR1110!
 * The LR1121 uses DIO9 (not DIO1) for interrupt output.
 * 
 * Bit | Interrupt          | Description
 * ----|--------------------|-------------------------------------------------
 *  0  | RFU                | Reserved
 *  1  | RFU                | Reserved
 *  2  | TxDone             | Packet transmission completed
 *  3  | RxDone             | Packet received
 *  4  | PreambleDetected   | Preamble detected
 *  5  | SyncWordValid      | Valid sync word / LoRa header detected
 *  6  | HeaderErr          | LoRa header CRC error
 *  7  | Err                | Packet received with CRC error
 *  8  | CadDone            | LoRa Channel activity detection finished
 *  9  | CadDetected        | LoRa Channel activity detected
 * 10  | Timeout            | RX or TX timeout  <-- CRITICAL FIX: Was bit 11!
 * 11  | LrFhssHop          | LR-FHSS intra-packet hopping
 * 21  | LBD                | Low Battery Detection
 * 22  | CmdError           | Host command error
 * 23  | Error              | An error other than a command error occurred
 * 24  | FskLenError        | Packet received with length error
 * 25  | FskAddrError       | Packet received with address error
 ******************************************************************************/

#define LR1121_IRQ_TX_DONE          (1UL << 2)   /* 0x00000004 - Bit 2 */
#define LR1121_IRQ_RX_DONE          (1UL << 3)   /* 0x00000008 - Bit 3 */
#define LR1121_IRQ_PREAMBLE_DETECT  (1UL << 4)   /* 0x00000010 - Bit 4 */
#define LR1121_IRQ_SYNC_WORD        (1UL << 5)   /* 0x00000020 - Bit 5 (SyncWordValid/HeaderValid) */
#define LR1121_IRQ_HEADER_ERR       (1UL << 6)   /* 0x00000040 - Bit 6 (LoRa header CRC error) */
#define LR1121_IRQ_CRC_ERR          (1UL << 7)   /* 0x00000080 - Bit 7 (Packet CRC error) */
#define LR1121_IRQ_CAD_DONE         (1UL << 8)   /* 0x00000100 - Bit 8 */
#define LR1121_IRQ_CAD_DETECT       (1UL << 9)   /* 0x00000200 - Bit 9 */
#define LR1121_IRQ_TIMEOUT          (1UL << 10)  /* 0x00000400 - Bit 10 - FIXED: Was 0x800! */
#define LR1121_IRQ_LR_FHSS_HOP      (1UL << 11)  /* 0x00000800 - Bit 11 */
#define LR1121_IRQ_LBD              (1UL << 21)  /* 0x00200000 - Bit 21 (Low Battery) */
#define LR1121_IRQ_CMD_ERROR        (1UL << 22)  /* 0x00400000 - Bit 22 */
#define LR1121_IRQ_ERROR            (1UL << 23)  /* 0x00800000 - Bit 23 */
#define LR1121_IRQ_FSK_LEN_ERROR    (1UL << 24)  /* 0x01000000 - Bit 24 */
#define LR1121_IRQ_FSK_ADDR_ERROR   (1UL << 25)  /* 0x02000000 - Bit 25 */
#define LR1121_IRQ_RADIO_NONE       0

/* Legacy alias for backward compatibility (was incorrectly named) */
#define LR1121_IRQ_HEADER_VALID     LR1121_IRQ_SYNC_WORD  /* Bit 5 - same as SyncWordValid */

/*******************************************************************************
 * Operating Modes
 * 
 * Citation: LR1121 Datasheet Section 3.1
 ******************************************************************************/

typedef enum {
    LR1121_MODE_SLEEP = 0x00,       /* Sleep mode */
    LR1121_MODE_STDBY_RC,           /* Standby with RC oscillator */
    LR1121_MODE_STDBY_XOSC,         /* Standby with XOSC oscillator */
    LR1121_MODE_FS,                 /* Frequency synthesis mode */
    LR1121_MODE_RX_CONT,            /* Continuous receive mode */
    LR1121_MODE_TX,                 /* Transmit mode */
    LR1121_MODE_CAD                 /* Channel activity detection */
} lr11xx_RadioOperatingModes_t;

/*******************************************************************************
 * Radio Commands - System
 * 
 * Citation: LR1121 Datasheet Section 11.1
 ******************************************************************************/

typedef enum {
    /* LR1121 System Command Opcodes
     * Citation: Semtech lr11xx_system.c lines 93-122
     * Verified against Waveshare Core1121_XF_Demo reference driver
     */
    LR11XX_SYSTEM_GET_STATUS_OC              = 0x0100,
    LR11XX_SYSTEM_GET_VERSION_OC             = 0x0101,
    LR11XX_SYSTEM_GET_IRQ_STATUS_OC          = 0x0012,  /* GetIrqStatus - returns 32-bit IRQ flags */
    LR11XX_SYSTEM_GET_ERRORS_OC              = 0x010D,
    LR11XX_SYSTEM_CLEAR_ERRORS_OC            = 0x010E,
    LR11XX_SYSTEM_CALIBRATE_OC               = 0x010F,  /* Calibrate - NOT 0x0100! */
    LR11XX_SYSTEM_SET_REGMODE_OC             = 0x0110,
    LR11XX_SYSTEM_CALIBRATE_IMAGE_OC         = 0x0111,
    LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC    = 0x0112,
    LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC        = 0x0113,
    LR11XX_SYSTEM_CLEAR_IRQ_OC               = 0x0114,
    LR11XX_SYSTEM_CFG_LFCLK_OC               = 0x0116,
    LR11XX_SYSTEM_SET_TCXO_MODE_OC           = 0x0117,  /* SetTcxoMode - NOT 0x0102! */
    LR11XX_SYSTEM_REBOOT_OC                  = 0x0118,
    LR11XX_SYSTEM_GET_VBAT_OC                = 0x0119,
    LR11XX_SYSTEM_GET_TEMP_OC                = 0x011A,
    LR11XX_SYSTEM_SET_SLEEP_OC               = 0x011B,
    LR11XX_SYSTEM_SET_STANDBY_OC             = 0x011C,  /* SetStandby - NOT 0x0105! */
    LR11XX_SYSTEM_SET_FS_OC                  = 0x011D,
    LR11XX_SYSTEM_GET_RANDOM_OC              = 0x0120,
    LR11XX_SYSTEM_ERASE_INFOPAGE_OC          = 0x0121,
    LR11XX_SYSTEM_WRITE_INFOPAGE_OC          = 0x0122,
    LR11XX_SYSTEM_READ_INFOPAGE_OC           = 0x0123,
    LR11XX_SYSTEM_READ_UID_OC                = 0x0125,
    LR11XX_SYSTEM_READ_JOIN_EUI_OC           = 0x0126,
    LR11XX_SYSTEM_READ_PIN_OC                = 0x0127,
    LR11XX_SYSTEM_ENABLE_SPI_CRC_OC          = 0x0128,
    LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_OC = 0x012A,
} lr11xx_system_commands_e;

/*******************************************************************************
 * Radio Commands - Radio
 * 
 * Citation: LR1121 Datasheet Section 11.2
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_RESET_STATS_OC               = 0x0200,
    LR11XX_RADIO_GET_STATS_OC                 = 0x0201,
    LR11XX_RADIO_GET_PKT_TYPE_OC              = 0x0202,
    LR11XX_RADIO_GET_RXBUFFER_STATUS_OC       = 0x0203,
    LR11XX_RADIO_GET_PKT_STATUS_OC            = 0x0204,
    LR11XX_RADIO_GET_RSSI_INST_OC             = 0x0205,
    LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC        = 0x0206,
    LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC   = 0x0208,
    LR11XX_RADIO_SET_RX_OC                    = 0x0209,
    LR11XX_RADIO_SET_TX_OC                    = 0x020A,
    LR11XX_RADIO_SET_RF_FREQUENCY_OC          = 0x020B,
    LR11XX_RADIO_AUTOTXRX_OC                  = 0x020C,
    LR11XX_RADIO_SET_CAD_PARAMS_OC            = 0x020D,
    LR11XX_RADIO_SET_PKT_TYPE_OC              = 0x020E,
    LR11XX_RADIO_SET_MODULATION_PARAM_OC      = 0x020F,
    LR11XX_RADIO_SET_PKT_PARAM_OC             = 0x0210,
    LR11XX_RADIO_SET_TX_PARAMS_OC             = 0x0211,
    LR11XX_RADIO_SET_PKT_ADRS_OC              = 0x0212,
    LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC   = 0x0213,
    LR11XX_RADIO_SET_RX_DUTY_CYCLE_OC         = 0x0214,
    LR11XX_RADIO_SET_PA_CFG_OC                = 0x0215,
    LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC  = 0x0217,
    LR11XX_RADIO_SET_CAD_OC                   = 0x0218,
    LR11XX_RADIO_SET_TX_CW_OC                 = 0x0219,
    LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_OC  = 0x021A,
    LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_OC     = 0x021B,
    LR11XX_RADIO_SET_GFSK_CRC_PARAMS_OC       = 0x0224,
    LR11XX_RADIO_SET_GFSK_WHITENING_PARAMS_OC = 0x0225,
    LR11XX_RADIO_SET_RX_BOOSTED_OC            = 0x0227,
    LR11XX_RADIO_SET_RSSI_CALIBRATION_OC      = 0x0229,
    LR11XX_RADIO_SET_LORA_SYNC_WORD_OC        = 0x022B,
    LR11XX_RADIO_SET_LR_FHSS_SYNC_WORD_OC     = 0x022D,
    LR11XX_RADIO_CFG_BLE_BEACON_OC            = 0x022E,
    LR11XX_RADIO_GET_LORA_RX_INFO_OC          = 0x0230,
    LR11XX_RADIO_BLE_BEACON_SEND_OC           = 0x0231,
    /* Experimental firmware commands */
    LR11XX_RADIO_GET_PACKET                   = 0x0700,
    LR11XX_RADIO_SET_FREQ_SET_RX              = 0x0701,
    LR11XX_RADIO_SET_RX_GET_PACKET            = 0x0702,
    LR11XX_RADIO_SET_FREQ_SET_RX_GET_PACKET   = 0x0703,
    LR11XX_RADIO_WRITE_BUFFER8_SET_TX         = 0x0704,
    LR11XX_RADIO_WRITE_BUFFER8_SET_FREQ_SET_TX = 0x0705,
} lr11xx_radio_commands_e;

/*******************************************************************************
 * Register Memory Commands
 * 
 * Citation: LR1121 Datasheet Section 11.3
 ******************************************************************************/

typedef enum {
    LR11XX_REGMEM_WRITE_REGMEM32_OC      = 0x0105,
    LR11XX_REGMEM_READ_REGMEM32_OC       = 0x0106,
    LR11XX_REGMEM_WRITE_MEM8_OC          = 0x0107,
    LR11XX_REGMEM_READ_MEM8_OC           = 0x0108,
    LR11XX_REGMEM_WRITE_BUFFER8_OC       = 0x0109,
    LR11XX_REGMEM_READ_BUFFER8_OC        = 0x010A,
    LR11XX_REGMEM_CLEAR_RXBUFFER_OC      = 0x010B,
    LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC = 0x010C,
} lr11xx_regmem_commands_e;

/*******************************************************************************
 * Bootloader Commands
 * 
 * Citation: LR1121 Datasheet Section 11.4
 ******************************************************************************/

typedef enum {
    LR11XX_BL_GET_STATUS_OC            = 0x0100,
    LR11XX_BL_GET_VERSION_OC           = 0x0101,
    LR11XX_BL_ERASE_FLASH_OC           = 0x8000,
    LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
    LR11XX_BL_REBOOT_OC                = 0x8005,
    LR11XX_BL_GET_PIN_OC               = 0x800B,
    LR11XX_BL_READ_CHIP_EUI_OC         = 0x800C,
    LR11XX_BL_READ_JOIN_EUI_OC         = 0x800D,
} lr11xx_bootloader_commands_e;

/*******************************************************************************
 * PA Selection
 * 
 * Citation: LR1121 Datasheet Section 6.1
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_PA_SEL_LP = 0x00,  /* Low-power Power Amplifier */
    LR11XX_RADIO_PA_SEL_HP = 0x01,  /* High-power Power Amplifier */
    LR11XX_RADIO_PA_SEL_HF = 0x02,  /* High-frequency Power Amplifier (2.4 GHz) */
} lr11xx_radio_pa_selection_t;

/*******************************************************************************
 * Fallback Modes
 * 
 * Citation: LR1121 Datasheet Section 5.4
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_FALLBACK_STDBY_RC   = 0x01,  /* Standby RC (Default) */
    LR11XX_RADIO_FALLBACK_STDBY_XOSC = 0x02,  /* Standby XOSC */
    LR11XX_RADIO_FALLBACK_FS         = 0x03   /* FS */
} lr11xx_radio_fallback_modes_t;

/*******************************************************************************
 * Ramp Time
 * 
 * Citation: LR1121 Datasheet Section 6.1.2
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_RAMP_16_US  = 0x00,
    LR11XX_RADIO_RAMP_32_US  = 0x01,
    LR11XX_RADIO_RAMP_48_US  = 0x02,  /* Default */
    LR11XX_RADIO_RAMP_64_US  = 0x03,
    LR11XX_RADIO_RAMP_80_US  = 0x04,
    LR11XX_RADIO_RAMP_96_US  = 0x05,
    LR11XX_RADIO_RAMP_112_US = 0x06,
    LR11XX_RADIO_RAMP_128_US = 0x07,
    LR11XX_RADIO_RAMP_144_US = 0x08,
    LR11XX_RADIO_RAMP_160_US = 0x09,
    LR11XX_RADIO_RAMP_176_US = 0x0A,
    LR11XX_RADIO_RAMP_192_US = 0x0B,
    LR11XX_RADIO_RAMP_208_US = 0x0C,
    LR11XX_RADIO_RAMP_240_US = 0x0D,
    LR11XX_RADIO_RAMP_272_US = 0x0E,
    LR11XX_RADIO_RAMP_304_US = 0x0F,
} lr11xx_radio_ramp_time_t;

/*******************************************************************************
 * LoRa Network Type
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_NETWORK_PRIVATE = 0x00,
    LR11XX_RADIO_LORA_NETWORK_PUBLIC  = 0x01,
} lr11xx_radio_lora_network_type_t;

/*******************************************************************************
 * LoRa Spreading Factor
 * 
 * Citation: LR1121 Datasheet Section 7.2.1
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_SF5  = 0x05,
    LR11XX_RADIO_LORA_SF6  = 0x06,
    LR11XX_RADIO_LORA_SF7  = 0x07,
    LR11XX_RADIO_LORA_SF8  = 0x08,
    LR11XX_RADIO_LORA_SF9  = 0x09,
    LR11XX_RADIO_LORA_SF10 = 0x0A,
    LR11XX_RADIO_LORA_SF11 = 0x0B,
    LR11XX_RADIO_LORA_SF12 = 0x0C,
} lr11xx_radio_lora_sf_t;

/*******************************************************************************
 * LoRa Bandwidth
 * 
 * Citation: LR1121 Datasheet Section 7.2.1
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_BW_10  = 0x08,  /* 10.42 kHz */
    LR11XX_RADIO_LORA_BW_15  = 0x01,  /* 15.63 kHz */
    LR11XX_RADIO_LORA_BW_20  = 0x09,  /* 20.83 kHz */
    LR11XX_RADIO_LORA_BW_31  = 0x02,  /* 31.25 kHz */
    LR11XX_RADIO_LORA_BW_41  = 0x0A,  /* 41.67 kHz */
    LR11XX_RADIO_LORA_BW_62  = 0x03,  /* 62.50 kHz */
    LR11XX_RADIO_LORA_BW_125 = 0x04,  /* 125.00 kHz */
    LR11XX_RADIO_LORA_BW_250 = 0x05,  /* 250.00 kHz */
    LR11XX_RADIO_LORA_BW_500 = 0x06,  /* 500.00 kHz */
    LR11XX_RADIO_LORA_BW_200 = 0x0D,  /* 203.00 kHz - 2.4 GHz only */
    LR11XX_RADIO_LORA_BW_400 = 0x0E,  /* 406.00 kHz - 2.4 GHz only */
    LR11XX_RADIO_LORA_BW_800 = 0x0F,  /* 812.00 kHz - 2.4 GHz only */
} lr11xx_radio_lora_bw_t;

/*******************************************************************************
 * LoRa Coding Rate
 * 
 * Citation: LR1121 Datasheet Section 7.2.1
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_NO_CR     = 0x00,
    LR11XX_RADIO_LORA_CR_4_5    = 0x01,  /* 4/5 Short Interleaver */
    LR11XX_RADIO_LORA_CR_4_6    = 0x02,  /* 4/6 Short Interleaver */
    LR11XX_RADIO_LORA_CR_4_7    = 0x03,  /* 4/7 Short Interleaver */
    LR11XX_RADIO_LORA_CR_4_8    = 0x04,  /* 4/8 Short Interleaver */
    LR11XX_RADIO_LORA_CR_LI_4_5 = 0x05,  /* 4/5 Long Interleaver */
    LR11XX_RADIO_LORA_CR_LI_4_6 = 0x06,  /* 4/6 Long Interleaver */
    LR11XX_RADIO_LORA_CR_LI_4_8 = 0x07,  /* 4/8 Long Interleaver */
} lr11xx_radio_lora_cr_t;

/*******************************************************************************
 * Intermediary Mode
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_MODE_SLEEP        = 0x00,
    LR11XX_RADIO_MODE_STANDBY_RC   = 0x01,
    LR11XX_RADIO_MODE_STANDBY_XOSC = 0x02,
    LR11XX_RADIO_MODE_FS           = 0x03
} lr11xx_radio_intermediary_mode_t;

/*******************************************************************************
 * LoRa CRC
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_CRC_OFF = 0x00,
    LR11XX_RADIO_LORA_CRC_ON  = 0x01,
} lr11xx_radio_lora_crc_t;

/*******************************************************************************
 * LoRa Packet Length Modes
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_PKT_EXPLICIT = 0x00,  /* Variable length, header included */
    LR11XX_RADIO_LORA_PKT_IMPLICIT = 0x01,  /* Fixed length, no header */
    LR1121_LORA_PACKET_VARIABLE_LENGTH = 0x00,
    LR1121_LORA_PACKET_FIXED_LENGTH    = 0x01,
    LR1121_LORA_PACKET_EXPLICIT = 0x00,
    LR1121_LORA_PACKET_IMPLICIT = 0x01,
} lr11xx_RadioLoRaPacketLengthsModes_t;

/*******************************************************************************
 * LoRa IQ
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_LORA_IQ_STANDARD = 0x00,
    LR11XX_RADIO_LORA_IQ_INVERTED = 0x01,
} lr11xx_radio_lora_iq_t;

/*******************************************************************************
 * Packet Type
 * 
 * Citation: LR1121 Datasheet Section 5.1
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_PKT_NONE         = 0x00,
    LR11XX_RADIO_PKT_TYPE_GFSK    = 0x01,
    LR11XX_RADIO_PKT_TYPE_LORA    = 0x02,
    LR11XX_RADIO_PKT_TYPE_BPSK    = 0x03,
    LR11XX_RADIO_PKT_TYPE_LR_FHSS = 0x04,
    LR11XX_RADIO_PKT_TYPE_RANGING = 0x05,
} lr11xx_radio_pkt_type_t;

/*******************************************************************************
 * PA Regulator Supply
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_PA_REG_SUPPLY_VREG = 0x00,
    LR11XX_RADIO_PA_REG_SUPPLY_VBAT = 0x01
} lr11xx_radio_pa_reg_supply_t;

/*******************************************************************************
 * RX Duty Cycle Mode
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX  = 0x00,
    LR11XX_RADIO_RX_DUTY_CYCLE_MODE_CAD = 0x01,
} lr11xx_radio_rx_duty_cycle_mode_t;

/*******************************************************************************
 * GFSK CRC Type
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_CRC_OFF         = 0x01,
    LR11XX_RADIO_GFSK_CRC_1_BYTE      = 0x00,
    LR11XX_RADIO_GFSK_CRC_2_BYTES     = 0x02,
    LR11XX_RADIO_GFSK_CRC_1_BYTE_INV  = 0x04,
    LR11XX_RADIO_GFSK_CRC_2_BYTES_INV = 0x06,
} lr11xx_radio_gfsk_crc_type_t;

/*******************************************************************************
 * GFSK DC Free
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_DC_FREE_OFF                   = 0x00,
    LR11XX_RADIO_GFSK_DC_FREE_WHITENING             = 0x01,
    LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP = 0x03,
} lr11xx_radio_gfsk_dc_free_t;

/*******************************************************************************
 * GFSK Packet Length Modes
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_PKT_FIX_LEN           = 0x00,
    LR11XX_RADIO_GFSK_PKT_VAR_LEN           = 0x01,
    LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP = 0x02,
} lr11xx_radio_gfsk_pkt_len_modes_t;

/*******************************************************************************
 * GFSK Address Filtering
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE      = 0x00,
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS = 0x01,
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} lr11xx_radio_gfsk_address_filtering_t;

/*******************************************************************************
 * GFSK Preamble Detector
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07
} lr11xx_radio_gfsk_preamble_detector_t;

/*******************************************************************************
 * GFSK Bandwidth
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_BW_4800   = 0x1F,
    LR11XX_RADIO_GFSK_BW_5800   = 0x17,
    LR11XX_RADIO_GFSK_BW_7300   = 0x0F,
    LR11XX_RADIO_GFSK_BW_9700   = 0x1E,
    LR11XX_RADIO_GFSK_BW_11700  = 0x16,
    LR11XX_RADIO_GFSK_BW_14600  = 0x0E,
    LR11XX_RADIO_GFSK_BW_19500  = 0x1D,
    LR11XX_RADIO_GFSK_BW_23400  = 0x15,
    LR11XX_RADIO_GFSK_BW_29300  = 0x0D,
    LR11XX_RADIO_GFSK_BW_39000  = 0x1C,
    LR11XX_RADIO_GFSK_BW_46900  = 0x14,
    LR11XX_RADIO_GFSK_BW_58600  = 0x0C,
    LR11XX_RADIO_GFSK_BW_78200  = 0x1B,
    LR11XX_RADIO_GFSK_BW_93800  = 0x13,
    LR11XX_RADIO_GFSK_BW_117300 = 0x0B,
    LR11XX_RADIO_GFSK_BW_156200 = 0x1A,
    LR11XX_RADIO_GFSK_BW_187200 = 0x12,
    LR11XX_RADIO_GFSK_BW_234300 = 0x0A,
    LR11XX_RADIO_GFSK_BW_312000 = 0x19,
    LR11XX_RADIO_GFSK_BW_373600 = 0x11,
    LR11XX_RADIO_GFSK_BW_467000 = 0x09
} lr11xx_radio_gfsk_bw_t;

/*******************************************************************************
 * GFSK Pulse Shape
 ******************************************************************************/

typedef enum {
    LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF   = 0x00,
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03 = 0x08,
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05 = 0x09,
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07 = 0x0A,
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1  = 0x0B
} lr11xx_radio_gfsk_pulse_shape_t;

/*******************************************************************************
 * SNR Scale Factor
 ******************************************************************************/

#define RADIO_SNR_SCALE 4

#ifdef __cplusplus
}
#endif

#endif /* LR1121_REGS_H */
