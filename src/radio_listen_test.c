/**
 * @file radio_listen_test.c
 * @brief Radio Listen Test Implementation - Initialize and Listen for Packets
 * 
 * This test initializes the LR1121 radio for ELRS-compatible LoRa reception
 * and continuously listens for incoming packets, displaying packet data
 * and signal quality metrics.
 * 
 * Implementation Notes:
 *   - Uses low-level lr1121_driver.c for SPI communication
 *   - Polls IRQ status (DIO1 interrupt not connected in test setup)
 *   - Prints received packets in hex format with RSSI/SNR
 * 
 * Citations:
 *   - 61252685.LR1121_V2_1_data_sheet.pdf Section 3 "SPI Interface"
 *   - 61252685.LR1121_V2_1_data_sheet.pdf Section 7 "LoRa Modulation"
 *   - 61252685.LR1121_V2_1_data_sheet.pdf Section 9 "IRQ System"
 *   - ExpressLRS 4.0 src/lib/LR1121Driver/LR1121.cpp
 */

#include "radio_listen_test.h"
#include "lr1121_driver.h"
#include "rsi_debug.h"

#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

/*******************************************************************************
 * LR1121 Command Opcodes
 * 
 * Citation: LR1121 Datasheet Section 11 "Command Reference"
 ******************************************************************************/

/* System Commands */
#define LR1121_CMD_GET_STATUS           0x0100
#define LR1121_CMD_GET_VERSION          0x0101
#define LR1121_CMD_GET_ERRORS           0x010D
#define LR1121_CMD_CLEAR_ERRORS         0x010E
/* CRITICAL FIX: Calibrate opcode is 0x010F per Semtech lr11xx_system.c line 99
 * Citation: LR11XX_SYSTEM_CALIBRATE_OC = 0x010F (NOT 0x0100 which is GetStatus!) */
#define LR1121_CMD_CALIBRATE            0x010F
#define LR1121_CMD_SET_REGMODE          0x0110
#define LR1121_CMD_CALIBRATE_IMAGE      0x0111
#define LR1121_CMD_SET_DIO_AS_RF_SWITCH 0x0112
#define LR1121_CMD_SET_DIO_IRQ_PARAMS   0x0113
#define LR1121_CMD_CLEAR_IRQ            0x0114
/* Citation: Semtech lr11xx_system.c line 111 - LR11XX_SYSTEM_SET_STANDBY_OC = 0x011C */
#define LR1121_CMD_SET_STANDBY          0x011C  /* Verified against Waveshare Core1121_XF_Demo */
#define LR1121_CMD_SET_FS               0x011D

/* Radio Commands */
#define LR1121_CMD_GET_RX_BUFFER_STATUS 0x0203
#define LR1121_CMD_GET_PACKET_STATUS    0x0204
#define LR1121_CMD_GET_RSSI_INST        0x0205
#define LR1121_CMD_SET_RX               0x0209
#define LR1121_CMD_SET_TX               0x020A
#define LR1121_CMD_SET_RF_FREQUENCY     0x020B
#define LR1121_CMD_SET_PACKET_TYPE      0x020E
#define LR1121_CMD_SET_MODULATION_PARAMS 0x020F
#define LR1121_CMD_SET_PACKET_PARAMS    0x0210
#define LR1121_CMD_SET_TX_PARAMS        0x0211
#define LR1121_CMD_SET_PA_CONFIG        0x0215
#define LR1121_CMD_SET_RX_BOOSTED       0x0227
#define LR1121_CMD_SET_LORA_SYNC_WORD   0x022B

/* Buffer Commands */
#define LR1121_CMD_WRITE_BUFFER8        0x0109
#define LR1121_CMD_READ_BUFFER8         0x010A
#define LR1121_CMD_CLEAR_RX_BUFFER      0x010B

/*******************************************************************************
 * LR1121 IRQ Flags
 * 
 * Citation: LR1121 Datasheet Section 9.2.4
 ******************************************************************************/

#define LR1121_IRQ_TX_DONE              0x00000004
#define LR1121_IRQ_RX_DONE              0x00000008
#define LR1121_IRQ_PREAMBLE_DETECTED    0x00000010
/* IRQ Flags - Citation: LR1121 User Manual Rev 1.2, Table 4-2 "IrqToEnable Interruption Mapping"
 * CRITICAL FIX: TIMEOUT is bit 10 (0x400), NOT bit 11 (0x800)!
 */
#define LR1121_IRQ_SYNC_WORD_VALID      0x00000020  /* Bit 5 */
#define LR1121_IRQ_HEADER_ERROR         0x00000040  /* Bit 6 - LoRa header CRC error */
#define LR1121_IRQ_CRC_ERROR            0x00000080  /* Bit 7 - Packet CRC error */
#define LR1121_IRQ_CAD_DONE             0x00000100  /* Bit 8 */
#define LR1121_IRQ_CAD_DETECTED         0x00000200  /* Bit 9 */
#define LR1121_IRQ_TIMEOUT              0x00000400  /* Bit 10 - FIXED! Was 0x800 */
#define LR1121_IRQ_LR_FHSS_HOP          0x00000800  /* Bit 11 */
#define LR1121_IRQ_ALL                  0x0FFF

/* Legacy alias - HEADER_VALID was incorrectly named, it's actually SYNC_WORD_VALID */
#define LR1121_IRQ_HEADER_VALID         LR1121_IRQ_SYNC_WORD_VALID

/*******************************************************************************
 * LR1121 Packet Types
 ******************************************************************************/

#define LR1121_PKT_TYPE_GFSK            0x01
#define LR1121_PKT_TYPE_LORA            0x02

/*******************************************************************************
 * Standby Modes
 ******************************************************************************/

#define LR1121_STANDBY_RC               0x00
#define LR1121_STANDBY_XOSC             0x01

/*******************************************************************************
 * Static Variables
 ******************************************************************************/

static radio_test_stats_t stats;
static uint32_t current_freq_hz = RADIO_TEST_FREQ_HZ;
static bool radio_initialized = false;

/*******************************************************************************
 * Debug Output Control
 ******************************************************************************/

#define RADIO_TEST_DEBUG    1

#if RADIO_TEST_DEBUG
    #define RADIO_DBG(fmt, ...)     DEBUGOUT("[RADIO_RX] " fmt, ##__VA_ARGS__)
#else
    #define RADIO_DBG(fmt, ...)     ((void)0)
#endif

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Simple delay in milliseconds
 */
static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 10000; j++) { }
    }
}

/**
 * @brief Execute LR1121 command with optional parameters and response
 * 
 * Citation: LR1121 Datasheet Section 3.1 "SPI Protocol"
 * 
 * @param opcode 16-bit command opcode
 * @param params Parameter bytes (NULL if none)
 * @param param_len Number of parameter bytes
 * @param response Response buffer (NULL if not needed)
 * @param response_len Expected response length
 * @return true on success
 */
static bool lr1121_command(uint16_t opcode, const uint8_t *params, 
                           uint16_t param_len, uint8_t *response, 
                           uint16_t response_len)
{
    /* Wait for chip ready */
    if (!lr1121_wait_busy_timeout(100)) {
        RADIO_DBG("BUSY timeout before cmd 0x%04X\n", opcode);
        return false;
    }
    
    /* Send command */
    if (!lr1121_send_command(opcode, params, param_len)) {
        RADIO_DBG("Failed to send cmd 0x%04X\n", opcode);
        return false;
    }
    
    /* Wait for processing */
    if (!lr1121_wait_busy_timeout(100)) {
        RADIO_DBG("BUSY timeout after cmd 0x%04X\n", opcode);
        return false;
    }
    
    /* Read response if needed */
    if (response != NULL && response_len > 0) {
        if (!lr1121_read_response(response, response_len)) {
            RADIO_DBG("Failed to read response for cmd 0x%04X\n", opcode);
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Set standby mode
 * 
 * Citation: LR1121 Datasheet Section 3.3.3 "SetStandby"
 * 
 * @param mode 0x00 = STDBY_RC, 0x01 = STDBY_XOSC
 */
static bool set_standby(uint8_t mode)
{
    return lr1121_command(LR1121_CMD_SET_STANDBY, &mode, 1, NULL, 0);
}

/**
 * @brief Set packet type
 * 
 * Citation: LR1121 Datasheet Section 7.1.1 "SetPacketType"
 * 
 * @param pkt_type 0x01 = GFSK, 0x02 = LoRa
 */
static bool set_packet_type(uint8_t pkt_type)
{
    return lr1121_command(LR1121_CMD_SET_PACKET_TYPE, &pkt_type, 1, NULL, 0);
}

/**
 * @brief Set RF frequency
 * 
 * Citation: LR1121 Datasheet Section 7.2.1 "SetRfFrequency"
 * 
 * @param freq_hz Frequency in Hz
 */
static bool set_rf_frequency(uint32_t freq_hz)
{
    uint8_t params[4];
    
    /* Frequency is sent as 32-bit big-endian value in Hz */
    params[0] = (freq_hz >> 24) & 0xFF;
    params[1] = (freq_hz >> 16) & 0xFF;
    params[2] = (freq_hz >> 8) & 0xFF;
    params[3] = freq_hz & 0xFF;
    
    return lr1121_command(LR1121_CMD_SET_RF_FREQUENCY, params, 4, NULL, 0);
}

/**
 * @brief Set LoRa modulation parameters
 * 
 * Citation: LR1121 Datasheet Section 7.3.1 "SetModulationParams"
 * 
 * @param sf Spreading factor (0x05-0x0C = SF5-SF12)
 * @param bw Bandwidth (0x04=125kHz, 0x05=250kHz, 0x06=500kHz)
 * @param cr Coding rate (0x01=4/5, 0x02=4/6, 0x03=4/7, 0x04=4/8)
 * @param ldro Low data rate optimize (0x00=off, 0x01=on)
 */
static bool set_lora_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t params[4] = {sf, bw, cr, ldro};
    return lr1121_command(LR1121_CMD_SET_MODULATION_PARAMS, params, 4, NULL, 0);
}

/**
 * @brief Set LoRa packet parameters
 * 
 * Citation: LR1121 Datasheet Section 7.3.2 "SetPacketParams"
 * 
 * @param preamble_len Preamble length in symbols
 * @param header_type 0x00=Variable (explicit), 0x01=Fixed (implicit)
 * @param payload_len Payload length in bytes
 * @param crc_type 0x00=OFF, 0x01=ON
 * @param invert_iq 0x00=Standard, 0x01=Inverted
 */
static bool set_lora_packet_params(uint16_t preamble_len, uint8_t header_type,
                                   uint8_t payload_len, uint8_t crc_type, 
                                   uint8_t invert_iq)
{
    uint8_t params[6];
    
    params[0] = (preamble_len >> 8) & 0xFF;  /* Preamble MSB */
    params[1] = preamble_len & 0xFF;          /* Preamble LSB */
    params[2] = header_type;
    params[3] = payload_len;
    params[4] = crc_type;
    params[5] = invert_iq;
    
    return lr1121_command(LR1121_CMD_SET_PACKET_PARAMS, params, 6, NULL, 0);
}

/**
 * @brief Configure DIO and IRQ parameters
 * 
 * Citation: LR1121 Datasheet Section 9.2.1 "SetDioIrqParams"
 * 
 * @param irq_mask IRQ enable mask
 * @param dio1_mask IRQs to route to DIO1
 * @param dio2_mask IRQs to route to DIO2
 * @param dio3_mask IRQs to route to DIO3
 */
static bool set_dio_irq_params(uint32_t irq_mask, uint32_t dio1_mask,
                               uint32_t dio2_mask, uint32_t dio3_mask)
{
    uint8_t params[16];
    
    /* IRQ enable mask */
    params[0] = (irq_mask >> 24) & 0xFF;
    params[1] = (irq_mask >> 16) & 0xFF;
    params[2] = (irq_mask >> 8) & 0xFF;
    params[3] = irq_mask & 0xFF;
    
    /* DIO1 mask */
    params[4] = (dio1_mask >> 24) & 0xFF;
    params[5] = (dio1_mask >> 16) & 0xFF;
    params[6] = (dio1_mask >> 8) & 0xFF;
    params[7] = dio1_mask & 0xFF;
    
    /* DIO2 mask */
    params[8] = (dio2_mask >> 24) & 0xFF;
    params[9] = (dio2_mask >> 16) & 0xFF;
    params[10] = (dio2_mask >> 8) & 0xFF;
    params[11] = dio2_mask & 0xFF;
    
    /* DIO3 mask */
    params[12] = (dio3_mask >> 24) & 0xFF;
    params[13] = (dio3_mask >> 16) & 0xFF;
    params[14] = (dio3_mask >> 8) & 0xFF;
    params[15] = dio3_mask & 0xFF;
    
    return lr1121_command(LR1121_CMD_SET_DIO_IRQ_PARAMS, params, 16, NULL, 0);
}

/**
 * @brief Clear IRQ flags
 * 
 * Citation: LR1121 Datasheet Section 9.2.3 "ClearIrq"
 */
static bool clear_irq(uint32_t irq_mask)
{
    uint8_t params[4];
    
    params[0] = (irq_mask >> 24) & 0xFF;
    params[1] = (irq_mask >> 16) & 0xFF;
    params[2] = (irq_mask >> 8) & 0xFF;
    params[3] = irq_mask & 0xFF;
    
    return lr1121_command(LR1121_CMD_CLEAR_IRQ, params, 4, NULL, 0);
}

/**
 * @brief Get IRQ status
 * 
 * Citation: LR1121 User Manual Rev 1.2 Section 4.1, Table 4-3
 *   Response format: [Stat1][Stat2][IrqStatus(31:24)][IrqStatus(23:16)][IrqStatus(15:8)][IrqStatus(7:0)]
 *   Total: 6 bytes (2 status bytes + 4 IRQ bytes)
 * 
 * BUG FIX: Was reading only 5 bytes and parsing response[1] as IrqStatus[31:24].
 *          But response[1] is Stat2, causing incorrect IRQ values (e.g., 0x1300C000).
 *          Now correctly reads 6 bytes and parses response[2-5] as IrqStatus.
 * 
 * @param[out] irq_status Pointer to store IRQ flags
 */
static bool get_irq_status(uint32_t *irq_status)
{
    uint8_t response[6];  /* 2 status bytes (Stat1, Stat2) + 4 IRQ bytes */
    
    if (!lr1121_command(LR1121_CMD_GET_STATUS, NULL, 0, response, 6)) {
        return false;
    }
    
    /* IRQ status is in bytes 2-5 (big-endian), after Stat1 and Stat2
     * Citation: LR1121 User Manual Rev 1.2 Section 4.1, Table 4-3:
     *   response[0] = Stat1
     *   response[1] = Stat2  
     *   response[2] = IrqStatus[31:24]
     *   response[3] = IrqStatus[23:16]
     *   response[4] = IrqStatus[15:8]
     *   response[5] = IrqStatus[7:0]
     */
    *irq_status = ((uint32_t)response[2] << 24) |
                  ((uint32_t)response[3] << 16) |
                  ((uint32_t)response[4] << 8) |
                  (uint32_t)response[5];
    
    return true;
}

/**
 * @brief Enter RX mode
 * 
 * Citation: LR1121 Datasheet Section 7.2.2 "SetRx"
 * 
 * @param timeout_ms Timeout in ms (0 = continuous = 0xFFFFFF)
 */
static bool set_rx(uint32_t timeout_ms)
{
    uint8_t params[3];
    uint32_t timeout_val;
    
    /* Timeout in 15.625 µs steps
     * Citation: LR1121 Datasheet Section 7.2.2
     * timeout_ms * 1000 / 15.625 = timeout_ms * 64
     * 
     * 0x000000 = No timeout (single RX)
     * 0xFFFFFF = Continuous RX
     */
    if (timeout_ms == 0) {
        timeout_val = 0xFFFFFF;  /* Continuous RX */
    } else {
        timeout_val = timeout_ms * 64;
        if (timeout_val > 0xFFFFFF) {
            timeout_val = 0xFFFFFF;
        }
    }
    
    params[0] = (timeout_val >> 16) & 0xFF;
    params[1] = (timeout_val >> 8) & 0xFF;
    params[2] = timeout_val & 0xFF;
    
    return lr1121_command(LR1121_CMD_SET_RX, params, 3, NULL, 0);
}

/**
 * @brief Get RX buffer status
 * 
 * Citation: LR1121 Datasheet Section 7.4.1 "GetRxBufferStatus"
 * 
 * @param[out] payload_len Received payload length
 * @param[out] start_offset Buffer start offset
 */
static bool get_rx_buffer_status(uint8_t *payload_len, uint8_t *start_offset)
{
    uint8_t response[3];  /* 1 status + 1 length + 1 offset */
    
    if (!lr1121_command(LR1121_CMD_GET_RX_BUFFER_STATUS, NULL, 0, response, 3)) {
        return false;
    }
    
    *payload_len = response[1];
    *start_offset = response[2];
    
    return true;
}

/**
 * @brief Get packet status (RSSI, SNR)
 * 
 * Citation: LR1121 Datasheet Section 7.4.2 "GetPacketStatus"
 * 
 * @param[out] rssi_pkt RSSI of last packet (dBm, -rssi/2)
 * @param[out] snr_pkt SNR of last packet (dB, snr/4)
 */
static bool get_packet_status(int16_t *rssi_pkt, int8_t *snr_pkt)
{
    uint8_t response[4];  /* 1 status + 1 rssi + 1 snr + 1 signal_rssi */
    
    if (!lr1121_command(LR1121_CMD_GET_PACKET_STATUS, NULL, 0, response, 4)) {
        return false;
    }
    
    /* RSSI is -response[1]/2 in dBm
     * SNR is response[2]/4 in dB (signed)
     * Citation: LR1121 Datasheet Section 7.4.2
     */
    *rssi_pkt = -((int16_t)response[1] / 2);
    *snr_pkt = (int8_t)response[2] / 4;
    
    return true;
}

/**
 * @brief Read data from RX buffer
 * 
 * Citation: LR1121 Datasheet Section 3.7.4 "ReadBuffer"
 * 
 * @param offset Start offset in buffer
 * @param buffer Destination buffer
 * @param len Number of bytes to read
 */
static bool read_buffer(uint8_t offset, uint8_t *buffer, uint8_t len)
{
    uint8_t cmd_params[2];
    uint8_t response[256];
    
    if (len > 254) {
        return false;
    }
    
    cmd_params[0] = offset;
    cmd_params[1] = len;
    
    if (!lr1121_command(LR1121_CMD_READ_BUFFER8, cmd_params, 2, response, len + 1)) {
        return false;
    }
    
    /* Skip status byte, copy data */
    memcpy(buffer, &response[1], len);
    
    return true;
}

/**
 * @brief Enable RX boosted mode for better sensitivity
 * 
 * Citation: LR1121 Datasheet Section 7.2.5 "SetRxBoosted"
 * 
 * @param boosted true to enable boosted mode
 */
static bool set_rx_boosted(bool boosted)
{
    uint8_t param = boosted ? 0x01 : 0x00;
    return lr1121_command(LR1121_CMD_SET_RX_BOOSTED, &param, 1, NULL, 0);
}

/**
 * @brief Set LoRa sync word
 * 
 * Citation: LR1121 Datasheet Section 7.3.3 "SetLoRaSyncWord"
 * Citation: ExpressLRS - Uses private network sync word 0x1424
 * 
 * @param sync_word 16-bit sync word
 */
static bool set_lora_sync_word(uint16_t sync_word)
{
    uint8_t params[2];
    
    params[0] = (sync_word >> 8) & 0xFF;
    params[1] = sync_word & 0xFF;
    
    return lr1121_command(LR1121_CMD_SET_LORA_SYNC_WORD, params, 2, NULL, 0);
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

radio_test_status_t radio_listen_init(void)
{
    lr1121_status_t drv_status;
    lr1121_version_t version;
    
    RADIO_DBG("\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("  Radio Listen Test - Initialization\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("\n");
    
    /* Reset statistics */
    memset(&stats, 0, sizeof(stats));
    stats.best_rssi = -150;   /* Start with very low value */
    stats.worst_rssi = 0;     /* Start with high value */
    
    /* Initialize LR1121 driver (GPIO, SPI, reset) */
    RADIO_DBG("Step 1: Initialize LR1121 driver...\n");
    drv_status = lr1121_init();
    if (drv_status != LR1121_OK) {
        RADIO_DBG("ERROR: LR1121 driver init failed: %d\n", drv_status);
        return RADIO_TEST_ERROR_INIT;
    }
    RADIO_DBG("  LR1121 driver initialized\n");
    
    /* Get and display firmware version */
    RADIO_DBG("Step 2: Read firmware version...\n");
    drv_status = lr1121_get_version(&version);
    if (drv_status == LR1121_OK) {
        RADIO_DBG("  Hardware:  0x%02X\n", version.hardware);
        RADIO_DBG("  Type:      0x%02X (%s)\n", version.type,
                  version.type == 0x03 ? "LR1121" : 
                  version.type == 0xDF ? "Bootloader" : "Unknown");
        RADIO_DBG("  Firmware:  v%d.%d (0x%04X)\n", 
                  (version.version >> 8), (version.version & 0xFF),
                  version.version);
    } else {
        RADIO_DBG("  WARNING: Could not read version (status=%d)\n", drv_status);
    }
    
    /* Set to standby RC mode for configuration */
    RADIO_DBG("Step 3: Set standby RC mode...\n");
    if (!set_standby(LR1121_STANDBY_RC)) {
        RADIO_DBG("ERROR: SetStandby(RC) failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  Standby RC mode active\n");
    
    /* Set packet type to LoRa */
    RADIO_DBG("Step 4: Set packet type to LoRa...\n");
    if (!set_packet_type(LR1121_PKT_TYPE_LORA)) {
        RADIO_DBG("ERROR: SetPacketType failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  Packet type: LoRa\n");
    
    /* Set LoRa modulation parameters */
    RADIO_DBG("Step 5: Configure LoRa modulation...\n");
    RADIO_DBG("  SF=%d, BW=0x%02X, CR=0x%02X, LDRO=%d\n",
              RADIO_TEST_SF, RADIO_TEST_BW, RADIO_TEST_CR, RADIO_TEST_LDRO);
    if (!set_lora_modulation_params(RADIO_TEST_SF, RADIO_TEST_BW, 
                                     RADIO_TEST_CR, RADIO_TEST_LDRO)) {
        RADIO_DBG("ERROR: SetModulationParams failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  Modulation configured\n");
    
    /* Set LoRa packet parameters */
    RADIO_DBG("Step 6: Configure packet parameters...\n");
    RADIO_DBG("  Preamble=%d, Header=%s, PayloadLen=%d, CRC=%s, IQ=%s\n",
              RADIO_TEST_PREAMBLE_LEN,
              RADIO_TEST_HEADER_TYPE ? "Implicit" : "Explicit",
              RADIO_TEST_PAYLOAD_LEN,
              RADIO_TEST_CRC_TYPE ? "ON" : "OFF",
              RADIO_TEST_INVERT_IQ ? "Inverted" : "Standard");
    if (!set_lora_packet_params(RADIO_TEST_PREAMBLE_LEN, RADIO_TEST_HEADER_TYPE,
                                 RADIO_TEST_PAYLOAD_LEN, RADIO_TEST_CRC_TYPE,
                                 RADIO_TEST_INVERT_IQ)) {
        RADIO_DBG("ERROR: SetPacketParams failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  Packet parameters configured\n");
    
    /* Set RF frequency */
    RADIO_DBG("Step 7: Set RF frequency...\n");
    RADIO_DBG("  Frequency: %" PRIu32 " Hz (%.3f MHz)\n", 
              current_freq_hz, (float)current_freq_hz / 1000000.0f);
    if (!set_rf_frequency(current_freq_hz)) {
        RADIO_DBG("ERROR: SetRfFrequency failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  Frequency set\n");
    
    /* Set LoRa sync word (ELRS private network) */
    RADIO_DBG("Step 8: Set LoRa sync word...\n");
    if (!set_lora_sync_word(0x1424)) {  /* ELRS sync word */
        RADIO_DBG("WARNING: SetLoRaSyncWord failed (non-critical)\n");
    } else {
        RADIO_DBG("  Sync word: 0x1424 (ELRS private)\n");
    }
    
    /* Enable RX boosted mode for better sensitivity */
    RADIO_DBG("Step 9: Enable RX boosted mode...\n");
    if (!set_rx_boosted(true)) {
        RADIO_DBG("WARNING: SetRxBoosted failed (non-critical)\n");
    } else {
        RADIO_DBG("  RX boosted mode enabled\n");
    }
    
    /* Configure IRQ parameters */
    RADIO_DBG("Step 10: Configure IRQ...\n");
    uint32_t irq_mask = LR1121_IRQ_RX_DONE | LR1121_IRQ_TIMEOUT | 
                        LR1121_IRQ_CRC_ERROR | LR1121_IRQ_HEADER_ERROR;
    if (!set_dio_irq_params(irq_mask, irq_mask, 0, 0)) {
        RADIO_DBG("ERROR: SetDioIrqParams failed\n");
        return RADIO_TEST_ERROR_CONFIG;
    }
    RADIO_DBG("  IRQ configured: RX_DONE, TIMEOUT, CRC_ERROR, HEADER_ERROR\n");
    
    /* Clear any pending IRQs */
    clear_irq(LR1121_IRQ_ALL);
    
    radio_initialized = true;
    
    RADIO_DBG("\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("  Radio Initialization Complete!\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("\n");
    
    return RADIO_TEST_OK;
}

void radio_listen_run(void)
{
    uint32_t irq_status;
    uint8_t payload_len, start_offset;
    uint8_t packet_buffer[64];
    int16_t rssi;
    int8_t snr;
    uint32_t loop_count = 0;
    uint32_t last_status_print = 0;
    
    if (!radio_initialized) {
        RADIO_DBG("ERROR: Radio not initialized! Call radio_listen_init() first.\n");
        return;
    }
    
    RADIO_DBG("\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("  Entering Continuous RX Mode\n");
    RADIO_DBG("========================================\n");
    RADIO_DBG("\n");
    RADIO_DBG("Frequency: %" PRIu32 " Hz (%.3f MHz)\n", 
              current_freq_hz, (float)current_freq_hz / 1000000.0f);
    RADIO_DBG("Listening for packets... (reset MCU to stop)\n");
    RADIO_DBG("\n");
    
    /* Enter continuous RX mode */
    if (!set_rx(RADIO_TEST_RX_TIMEOUT_MS)) {
        RADIO_DBG("ERROR: Failed to enter RX mode\n");
        return;
    }
    
    RADIO_DBG("RX mode active. Waiting for packets...\n\n");
    
    /* Main RX loop */
    while (1) {
        loop_count++;
        
        /* Poll IRQ status */
        if (!get_irq_status(&irq_status)) {
            RADIO_DBG("WARNING: Failed to get IRQ status\n");
            delay_ms(100);
            continue;
        }
        
        /* Check for RX_DONE */
        if (irq_status & LR1121_IRQ_RX_DONE) {
            /* Get buffer status */
            if (get_rx_buffer_status(&payload_len, &start_offset)) {
                if (payload_len > 0 && payload_len <= sizeof(packet_buffer)) {
                    /* Read packet data */
                    if (read_buffer(start_offset, packet_buffer, payload_len)) {
                        /* Get packet status (RSSI/SNR) */
                        if (get_packet_status(&rssi, &snr)) {
                            stats.last_rssi = rssi;
                            stats.last_snr = snr;
                            
                            /* Update best/worst RSSI */
                            if (rssi > stats.best_rssi) stats.best_rssi = rssi;
                            if (rssi < stats.worst_rssi) stats.worst_rssi = rssi;
                        }
                        
                        stats.packets_received++;
                        
                        /* Print packet info */
                        RADIO_DBG("=== PACKET #%" PRIu32 " ===\n", stats.packets_received);
                        RADIO_DBG("  RSSI: %d dBm, SNR: %d dB\n", rssi, snr);
                        RADIO_DBG("  Length: %d bytes\n", payload_len);
                        RADIO_DBG("  Data: ");
                        for (int i = 0; i < payload_len && i < 16; i++) {
                            DEBUGOUT("%02X ", packet_buffer[i]);
                        }
                        if (payload_len > 16) {
                            DEBUGOUT("...");
                        }
                        DEBUGOUT("\n\n");
                    }
                }
            }
            
            /* Clear RX_DONE IRQ and re-enter RX mode */
            clear_irq(LR1121_IRQ_RX_DONE);
            set_rx(RADIO_TEST_RX_TIMEOUT_MS);
        }
        
        /* Check for CRC error */
        if (irq_status & LR1121_IRQ_CRC_ERROR) {
            stats.crc_errors++;
            RADIO_DBG("CRC Error (total: %" PRIu32 ")\n", stats.crc_errors);
            
            clear_irq(LR1121_IRQ_CRC_ERROR);
            set_rx(RADIO_TEST_RX_TIMEOUT_MS);
        }
        
        /* Check for header error */
        if (irq_status & LR1121_IRQ_HEADER_ERROR) {
            RADIO_DBG("Header Error\n");
            
            clear_irq(LR1121_IRQ_HEADER_ERROR);
            set_rx(RADIO_TEST_RX_TIMEOUT_MS);
        }
        
        /* Check for timeout */
        if (irq_status & LR1121_IRQ_TIMEOUT) {
            stats.timeout_count++;
            
            clear_irq(LR1121_IRQ_TIMEOUT);
            set_rx(RADIO_TEST_RX_TIMEOUT_MS);
        }
        
        /* Print periodic status (every ~10 seconds) */
        if (loop_count - last_status_print >= 10000) {
            last_status_print = loop_count;
            RADIO_DBG("--- Status: RX=%" PRIu32 ", CRC_ERR=%" PRIu32 
                      ", Best RSSI=%d dBm ---\n",
                      stats.packets_received, stats.crc_errors, stats.best_rssi);
        }
        
        /* Small delay to avoid hammering the SPI bus */
        delay_ms(1);
    }
}

uint8_t radio_listen_check_packet(uint8_t *buffer, uint8_t max_len, 
                                   int16_t *rssi, int8_t *snr)
{
    uint32_t irq_status;
    uint8_t payload_len, start_offset;
    
    if (!radio_initialized || buffer == NULL) {
        return 0;
    }
    
    /* Check IRQ status */
    if (!get_irq_status(&irq_status)) {
        return 0;
    }
    
    /* Not RX_DONE? */
    if (!(irq_status & LR1121_IRQ_RX_DONE)) {
        return 0;
    }
    
    /* Get buffer status */
    if (!get_rx_buffer_status(&payload_len, &start_offset)) {
        clear_irq(LR1121_IRQ_RX_DONE);
        return 0;
    }
    
    /* Validate length */
    if (payload_len == 0 || payload_len > max_len) {
        clear_irq(LR1121_IRQ_RX_DONE);
        return 0;
    }
    
    /* Read packet */
    if (!read_buffer(start_offset, buffer, payload_len)) {
        clear_irq(LR1121_IRQ_RX_DONE);
        return 0;
    }
    
    /* Get RSSI/SNR */
    if (rssi != NULL && snr != NULL) {
        get_packet_status(rssi, snr);
        stats.last_rssi = *rssi;
        stats.last_snr = *snr;
    }
    
    stats.packets_received++;
    
    /* Clear IRQ */
    clear_irq(LR1121_IRQ_RX_DONE);
    
    return payload_len;
}

const radio_test_stats_t* radio_listen_get_stats(void)
{
    return &stats;
}

void radio_listen_reset_stats(void)
{
    memset(&stats, 0, sizeof(stats));
    stats.best_rssi = -150;
    stats.worst_rssi = 0;
}

radio_test_status_t radio_listen_set_frequency(uint32_t freq_hz)
{
    if (!radio_initialized) {
        current_freq_hz = freq_hz;
        return RADIO_TEST_OK;
    }
    
    /* Need to go to standby to change frequency */
    if (!set_standby(LR1121_STANDBY_RC)) {
        return RADIO_TEST_ERROR_CONFIG;
    }
    
    if (!set_rf_frequency(freq_hz)) {
        return RADIO_TEST_ERROR_CONFIG;
    }
    
    current_freq_hz = freq_hz;
    
    /* Re-enter RX mode */
    set_rx(RADIO_TEST_RX_TIMEOUT_MS);
    
    return RADIO_TEST_OK;
}

void radio_listen_test_run(void)
{
    radio_test_status_t status;
    
    RADIO_DBG("\n");
    RADIO_DBG("********************************************************\n");
    RADIO_DBG("*                                                      *\n");
    RADIO_DBG("*    ELRS Radio Listen Test                            *\n");
    RADIO_DBG("*    Initialize LR1121 and Listen for Packets          *\n");
    RADIO_DBG("*                                                      *\n");
    RADIO_DBG("********************************************************\n");
    RADIO_DBG("\n");
    
    /* Initialize radio */
    status = radio_listen_init();
    if (status != RADIO_TEST_OK) {
        RADIO_DBG("\n");
        RADIO_DBG("*** INITIALIZATION FAILED (status=%d) ***\n", status);
        RADIO_DBG("Check hardware connections and try again.\n");
        RADIO_DBG("\n");
        return;
    }
    
    /* Enter continuous RX mode (does not return) */
    radio_listen_run();
}
