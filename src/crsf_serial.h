/**
 * @file crsf_serial.h
 * @brief CRSF Serial Output to Flight Controller for SiW917
 *
 * Outputs CRSF RC channel data and link statistics to a flight controller
 * via USART. Compatible with Betaflight, INAV, Ardupilot, etc.
 *
 * Pin Configuration:
 *   Uses the generated `Driver_USART0` routing from `config/RTE_Device_917.h`.
 *   Connect USART0 TX to the flight controller CRSF RX input.
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#ifndef CRSF_SERIAL_H
#define CRSF_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define CRSF_SERIAL_BAUDRATE_DEFAULT   420000  /* ELRS standard */

/* Use existing CRSF definitions from crsf_protocol.h if available */
#ifndef CRSF_SYNC_BYTE
#define CRSF_SYNC_BYTE          0xC8
#endif
#ifndef CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#define CRSF_FRAMETYPE_RC       0x16    /* RC channels packed */
#else
#define CRSF_FRAMETYPE_RC       CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#endif
#ifndef CRSF_FRAMETYPE_LINK_STATISTICS
#define CRSF_FRAMETYPE_LINK     0x14    /* Link statistics */
#else
#define CRSF_FRAMETYPE_LINK     CRSF_FRAMETYPE_LINK_STATISTICS
#endif
#ifndef CRSF_CRC_POLY
#define CRSF_CRC_POLY           0xD5
#endif

#define CRSF_SERIAL_NUM_CHANNELS       16
#define CRSF_SERIAL_CHANNEL_MIN        172     /* 988us */
#define CRSF_SERIAL_CHANNEL_MID        992     /* 1500us */
#define CRSF_SERIAL_CHANNEL_MAX        1811    /* 2012us */
#define CRSF_SERIAL_MAX_FRAME_SIZE     26

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Link statistics for CRSF output
 */
typedef struct {
    uint8_t uplink_rssi_1;      /* RSSI antenna 1 (dBm * -1) */
    uint8_t uplink_rssi_2;      /* RSSI antenna 2 (dBm * -1) */
    uint8_t uplink_lq;          /* Link quality 0-100% */
    int8_t  uplink_snr;         /* SNR in dB */
    uint8_t active_antenna;     /* 0 = ant1, 1 = ant2 */
    uint8_t rf_mode;            /* RF mode index */
    uint8_t uplink_tx_power;    /* TX power index */
    uint8_t downlink_rssi;      /* Downlink RSSI (dBm * -1) */
    uint8_t downlink_lq;        /* Downlink LQ 0-100% */
    int8_t  downlink_snr;       /* Downlink SNR in dB */
} crsf_link_stats_t;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize CRSF serial output
 *
 * Configures USART0 for CRSF output at specified baud rate.
 *
 * @param baud_rate Baud rate (0 = default 420000)
 * @return 0 on success, negative on error
 */
int crsf_serial_init(uint32_t baud_rate);

/**
 * @brief Deinitialize CRSF serial output
 */
void crsf_serial_deinit(void);

/**
 * @brief Check if CRSF serial is initialized
 * @return true if ready
 */
bool crsf_serial_is_ready(void);

/**
 * @brief Send RC channels to flight controller
 *
 * Packs and transmits 16 channels in CRSF format.
 *
 * @param channels Array of 16 channel values (CRSF format: 172-1811)
 * @return 0 on success, negative on error
 */
int crsf_serial_send_channels(const uint32_t *channels);

/**
 * @brief Send link statistics to flight controller
 *
 * @param stats Link statistics structure
 * @return 0 on success, negative on error
 */
int crsf_serial_send_link_stats(const crsf_link_stats_t *stats);

/**
 * @brief Send a prebuilt CRSF frame to the flight controller
 *
 * @param frame Complete CRSF frame including sync byte, length, type and CRC
 * @param frame_len Total frame length in bytes
 * @return 0 on success, negative on error
 */
int crsf_serial_send_frame(const uint8_t *frame, uint32_t frame_len);

/**
 * @brief Get number of frames sent
 * @return Total frames transmitted
 */
uint32_t crsf_serial_get_tx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_SERIAL_H */
