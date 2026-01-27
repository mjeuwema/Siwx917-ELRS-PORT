/**
 * @file crsf.h
 * @brief CRSF Serial Output Interface for SiWx917
 * 
 * Provides functions to build and transmit CRSF frames to flight controllers.
 * Used by ELRS RX to output decoded channel data.
 * 
 * Citation: ELRS 4.0 src/lib/CrsfProtocol/crsf_protocol.h
 * Citation: TBS CRSF Protocol Specification
 * 
 * Usage:
 *   1. Call crsf_init() with desired baud rate (420800 for ELRS)
 *   2. After receiving ELRS packets, call crsf_send_rc_channels()
 *   3. Optionally send link statistics with crsf_send_link_stats()
 */

#ifndef ELRS_CRSF_H
#define ELRS_CRSF_H

#include "elrs_platform.h"
#include "crsf_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/**
 * CRSF standard baud rates
 * Citation: TBS CRSF spec / ELRS implementation
 */
#define CRSF_BAUDRATE           420800      /* ELRS standard (inverted from 400k) */
#define CRSF_BAUDRATE_TBS       400000      /* Original TBS Crossfire */

/**
 * Frame transmission intervals
 * Citation: CRSF spec - RC channels sent at link rate, stats less frequently
 */
#define CRSF_RC_INTERVAL_MS     4           /* ~250Hz max for RC channels */
#define CRSF_STATS_INTERVAL_MS  100         /* 10Hz for link stats */

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize CRSF UART output
 * 
 * Configures USART0 for CRSF serial output at specified baud rate.
 * 
 * @param baud_rate Desired baud rate (use CRSF_BAUDRATE for ELRS, or 0 for default)
 * @return true on success, false on failure
 * 
 * Citation: SiWx917 Family RM Section 30 - USART Configuration
 */
bool crsf_init(uint32_t baud_rate);

/**
 * @brief Deinitialize CRSF output
 * 
 * Stops UART and releases resources.
 */
void crsf_deinit(void);

/*******************************************************************************
 * Frame Building
 ******************************************************************************/

/**
 * @brief Build RC channels packed frame
 * 
 * Packs 16 channels (11-bit each) into CRSF frame format.
 * 
 * @param frame Output buffer (must be at least 32 bytes)
 * @param channels Array of 16 channel values (CRSF format: 172-1811, center 992)
 * @return Frame length (26 bytes) or 0 on error
 * 
 * Citation: CRSF spec - FRAMETYPE_RC_CHANNELS_PACKED (0x16)
 * Citation: ELRS crsf_protocol.h lines 181-199 (channel packing)
 */
uint8_t crsf_build_rc_channels_frame(uint8_t *frame, const uint16_t *channels);

/**
 * @brief Build link statistics frame
 * 
 * @param frame Output buffer (must be at least 16 bytes)
 * @param stats Link statistics data
 * @return Frame length (14 bytes) or 0 on error
 * 
 * Citation: CRSF spec - FRAMETYPE_LINK_STATISTICS (0x14)
 */
uint8_t crsf_build_link_stats_frame(uint8_t *frame, const crsf_link_stats_t *stats);

/*******************************************************************************
 * Frame Transmission
 ******************************************************************************/

/**
 * @brief Send raw CRSF frame
 * 
 * Transmits a pre-built CRSF frame over UART.
 * 
 * @param frame Frame data to send
 * @param len Frame length
 * @return true on success, false on error
 */
bool crsf_send_frame(const uint8_t *frame, uint8_t len);

/**
 * @brief Send RC channels to flight controller
 * 
 * Builds and sends RC channels packed frame.
 * This is the main function called after decoding ELRS packets.
 * 
 * @param channels Array of 16 channel values in CRSF format
 * @return true on success, false on error
 * 
 * Example:
 *   const elrs_channel_data_t *ch = elrs_rx_get_channels();
 *   crsf_send_rc_channels(ch->ch);
 */
bool crsf_send_rc_channels(const uint16_t *channels);

/**
 * @brief Send link statistics to flight controller
 * 
 * Builds and sends link statistics frame.
 * Call this periodically (e.g., every 100ms) to update FC telemetry.
 * 
 * @param stats Link statistics data
 * @return true on success, false on error
 */
bool crsf_send_link_stats(const crsf_link_stats_t *stats);

/*******************************************************************************
 * Status
 ******************************************************************************/

/**
 * @brief Check if CRSF output is initialized
 * @return true if initialized and ready
 */
bool crsf_is_initialized(void);

/**
 * @brief Get number of frames sent
 * @return Total frames transmitted since init
 */
uint32_t crsf_get_frames_sent(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_CRSF_H */
