/**
 * @file radio_listen_test.h
 * @brief Radio Listen Test - Initialize LR1121 and Listen for Packets
 * 
 * This test module initializes the LR1121 radio for ELRS-compatible
 * LoRa reception and continuously listens for incoming packets.
 * 
 * Test Purpose:
 *   - Verify radio initialization sequence
 *   - Configure LoRa modulation parameters
 *   - Enter continuous RX mode
 *   - Display received packets and signal quality (RSSI/SNR)
 * 
 * Hardware Requirements:
 *   - SiWG917Y (BRD2708A) + LR1121 on mikroBUS socket
 *   - Optionally: ELRS TX module for packet generation
 * 
 * Citations:
 *   - 61252685.LR1121_V2_1_data_sheet.pdf Section 7 (LoRa Modulation)
 *   - 61252685.LR1121_V2_1_data_sheet.pdf Section 9 (IRQ System)
 *   - ExpressLRS 4.0 common.cpp (rate tables)
 */

#ifndef RADIO_LISTEN_TEST_H
#define RADIO_LISTEN_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Test Configuration - Adjust these for your environment
 ******************************************************************************/

/**
 * @brief Default test frequency (Hz)
 * 
 * Citation: FCC Part 15.247 - 902-928 MHz ISM band (US)
 * Citation: ETSI EN 300 220 - 868-870 MHz (EU)
 * 
 * Common ELRS frequencies:
 *   915 MHz (US)
 *   868 MHz (EU)
 *   433 MHz (LRS)
 */
#define RADIO_TEST_FREQ_HZ          915000000UL

/**
 * @brief LoRa Modulation Parameters
 * 
 * Citation: ExpressLRS common.cpp RATE_LORA_900_100HZ
 *   SF7, BW500, CR4/7 → 100 Hz packet rate
 * 
 * Citation: LR1121 Datasheet Section 7.2.1
 *   SF: 0x05-0x0C (SF5-SF12)
 *   BW: 0x06 = 500 kHz, 0x04 = 125 kHz
 *   CR: 0x01 = 4/5, 0x03 = 4/7, 0x04 = 4/8
 */
#define RADIO_TEST_SF               0x07    /* SF7 (LR11XX_RADIO_LORA_SF7) */
#define RADIO_TEST_BW               0x06    /* 500 kHz (LR11XX_RADIO_LORA_BW_500) */
#define RADIO_TEST_CR               0x03    /* CR 4/7 (LR11XX_RADIO_LORA_CR_4_7) */
#define RADIO_TEST_LDRO             0x00    /* Low Data Rate Optimize OFF */

/**
 * @brief LoRa Packet Parameters
 * 
 * Citation: ExpressLRS OTA.h - Standard 4-byte packet
 * Citation: LR1121 Datasheet Section 7.3.2
 */
#define RADIO_TEST_PREAMBLE_LEN     8       /* Preamble symbols */
#define RADIO_TEST_HEADER_TYPE      0x01    /* Implicit header (fixed length) */
#define RADIO_TEST_PAYLOAD_LEN      8       /* ELRS uses 4-8 byte packets */
#define RADIO_TEST_CRC_TYPE         0x00    /* CRC OFF (handled by ELRS OTA layer) */
#define RADIO_TEST_INVERT_IQ        0x00    /* Standard IQ (0x00) for 900 MHz */

/**
 * @brief RX timeout in milliseconds
 * 
 * 0 = Continuous RX (0xFFFFFF in LR1121 format)
 * >0 = Single RX with timeout
 */
#define RADIO_TEST_RX_TIMEOUT_MS    0       /* Continuous RX */

/*******************************************************************************
 * Result Codes
 ******************************************************************************/

typedef enum {
    RADIO_TEST_OK = 0,
    RADIO_TEST_ERROR_INIT,
    RADIO_TEST_ERROR_RESET,
    RADIO_TEST_ERROR_CONFIG,
    RADIO_TEST_ERROR_TIMEOUT,
} radio_test_status_t;

/*******************************************************************************
 * Packet Statistics
 ******************************************************************************/

typedef struct {
    uint32_t packets_received;      /* Total packets received */
    uint32_t crc_errors;            /* Packets with CRC errors */
    uint32_t timeout_count;         /* RX timeout events */
    int16_t  last_rssi;             /* Last packet RSSI (dBm) */
    int8_t   last_snr;              /* Last packet SNR (dB) */
    int16_t  best_rssi;             /* Best (highest) RSSI seen */
    int16_t  worst_rssi;            /* Worst (lowest) RSSI seen */
} radio_test_stats_t;

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize the radio for RX mode
 * 
 * Initialization sequence:
 *   1. Initialize LR1121 driver (GPIO, SPI, reset)
 *   2. Configure TCXO (SetTcxoMode for external TCXO)
 *   3. Set standby mode (XOSC)
 *   4. Configure LoRa packet type
 *   5. Set modulation parameters (SF, BW, CR)
 *   6. Set packet parameters (preamble, header, payload)
 *   7. Set RF frequency
 *   8. Configure IRQ masks
 * 
 * Citation: LR1121 Datasheet Section 3.3 "Typical Application"
 * 
 * @return RADIO_TEST_OK on success, error code otherwise
 */
radio_test_status_t radio_listen_init(void);

/**
 * @brief Enter continuous RX mode and listen for packets
 * 
 * This function enters RX mode and polls for incoming packets.
 * It runs in an infinite loop, printing packet info when received.
 * 
 * Citation: LR1121 Datasheet Section 7.2.2 "SetRx"
 * 
 * Note: This function does not return normally. Reset MCU to exit.
 */
void radio_listen_run(void);

/**
 * @brief Single-shot packet check (non-blocking)
 * 
 * Checks if a packet has been received since last check.
 * Useful for integration into existing event loops.
 * 
 * @param[out] buffer Buffer to store received packet (must be at least payload_len bytes)
 * @param[in]  max_len Maximum bytes to read
 * @param[out] rssi Received signal strength (dBm)
 * @param[out] snr Signal-to-noise ratio (dB)
 * @return Number of bytes received, 0 if no packet
 */
uint8_t radio_listen_check_packet(uint8_t *buffer, uint8_t max_len, int16_t *rssi, int8_t *snr);

/**
 * @brief Get current packet statistics
 * 
 * @return Pointer to statistics structure
 */
const radio_test_stats_t* radio_listen_get_stats(void);

/**
 * @brief Reset packet statistics
 */
void radio_listen_reset_stats(void);

/**
 * @brief Set test frequency
 * 
 * @param freq_hz Frequency in Hz
 * @return RADIO_TEST_OK on success
 */
radio_test_status_t radio_listen_set_frequency(uint32_t freq_hz);

/**
 * @brief Complete test entry point (init + listen loop)
 * 
 * Convenience function that calls radio_listen_init() followed by
 * radio_listen_run(). Does not return.
 */
void radio_listen_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_LISTEN_TEST_H */
