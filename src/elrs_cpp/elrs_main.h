/**
 * @file elrs_main.h
 * @brief Main ELRS RX application interface for SiW917
 *
 * C-compatible interface for the ELRS receiver functionality.
 * Call from your main application loop.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Connection state enumeration
 */
typedef enum {
    ELRS_CONNECTED = 0,
    ELRS_TENTATIVE,
    ELRS_DISCONNECTED,
    ELRS_BINDING,
    ELRS_RADIO_FAILED
} elrs_connection_state_t;

/**
 * @brief Channel data callback function type
 * @param channels Array of 16 channel values (CRSF format: 172-1811, center 992)
 * @param num_channels Number of valid channels
 */
typedef void (*elrs_channel_callback_t)(const uint32_t *channels, uint8_t num_channels);

/**
 * @brief Link statistics structure
 */
typedef struct {
    int8_t rssi_1;        // RSSI antenna 1 (negative dBm)
    int8_t rssi_2;        // RSSI antenna 2 (negative dBm)
    uint8_t lq;           // Link quality 0-100%
    int8_t snr;           // Signal to noise ratio
    uint8_t rf_mode;      // Current RF mode index
    uint8_t active_ant;   // Active antenna (0 or 1)
} elrs_link_stats_t;

/**
 * @brief Initialize the ELRS receiver
 *
 * Sets up the radio, timer, and all ELRS subsystems.
 * Must be called once at startup after peripherals are initialized.
 *
 * @return true if initialization successful
 */
bool elrs_init(void);

/**
 * @brief Process ELRS tasks (call from main loop)
 *
 * This should be called frequently from the main application loop.
 * Handles radio events, connection management, telemetry, etc.
 */
void elrs_loop(void);

/**
 * @brief Get current connection state
 * @return Current connection state
 */
elrs_connection_state_t elrs_get_connection_state(void);

/**
 * @brief Check if receiver is connected
 * @return true if connected or tentative
 */
bool elrs_is_connected(void);

/**
 * @brief Get current channel data
 * @param channels Output array for 16 channels (must be at least 16 elements)
 * @return Number of valid channels
 */
uint8_t elrs_get_channels(uint32_t *channels);

/**
 * @brief Get link statistics
 * @param stats Output structure for link stats
 */
void elrs_get_link_stats(elrs_link_stats_t *stats);

/**
 * @brief Set channel data callback
 *
 * Called whenever new channel data is received.
 *
 * @param callback Function to call with new channel data
 */
void elrs_set_channel_callback(elrs_channel_callback_t callback);

/**
 * @brief Enter binding mode
 */
void elrs_enter_binding_mode(void);

/**
 * @brief Exit binding mode
 */
void elrs_exit_binding_mode(void);

/**
 * @brief Enter WiFi configuration mode
 * 
 * Starts WiFi AP and HTTP server for configuration and OTA updates.
 * This function blocks until device is rebooted from web UI.
 * 
 * Access points:
 *   SSID: ELRS_TEST_AP
 *   Password: elrs1234
 *   URL: http://192.168.10.10/
 */
void elrs_enter_wifi_mode(void);

/**
 * @brief Check if in WiFi mode
 * @return true if WiFi configuration mode is active
 */
bool elrs_is_wifi_mode(void);

/**
 * @brief Set new binding UID and save to persistent storage
 * 
 * Updates the runtime UID, saves to NVM3, and reinitializes FHSS.
 * Call this after receiving a new UID from binding or WiFi config.
 *
 * @param new_uid 6-byte UID array
 * @return true if successfully saved, false on error
 */
bool elrs_set_uid(const uint8_t new_uid[6]);

/**
 * @brief Get uplink LQ (Link Quality percentage)
 * @return LQ value 0-100
 */
uint8_t elrs_get_lq(void);

/**
 * @brief Get current RF rate/mode name
 * @return Static string describing current mode
 */
const char* elrs_get_rate_name(void);

#ifdef __cplusplus
}
#endif
