/**
 * @file elrs_main.h
 * @brief ELRS 4.0 Main Flow Controller
 * 
 * This module provides the complete ELRS 4.0 receiver flow, coordinating:
 *   - Hardware timer (tick/tock callbacks)
 *   - Phase-frequency detection (PFD)
 *   - Link quality calculation
 *   - Radio IRQ handling
 *   - Connection state management
 *   - CRSF output to flight controller
 * 
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 *   - setup() initializes all subsystems
 *   - loop() handles main processing
 *   - HWtimerCallbackTick/Tock for timing
 *   - ProcessRFPacket for packet handling
 * 
 * This is the main entry point for ELRS functionality.
 * Call elrs_main_setup() once at startup, then elrs_main_loop()
 * continuously from your main task.
 */

#ifndef ELRS_MAIN_H
#define ELRS_MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "elrs_rx.h"
#include "crsf_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * ELRS Receiver Mode
 ******************************************************************************/

/**
 * @brief Operating mode of the receiver
 */
typedef enum {
    ELRS_MODE_RX = 0,       /**< Normal RX operation */
    ELRS_MODE_BINDING,      /**< In binding mode */
    ELRS_MODE_WIFI,         /**< WiFi OTA update mode */
    ELRS_MODE_IDLE          /**< Idle/standby mode */
} elrs_mode_t;

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief ELRS main flow configuration structure
 * 
 * Note: This is distinct from elrs_config_t in elrs_config.h which handles
 * NVM3 persistent storage. This struct is for runtime initialization.
 */
typedef struct {
    /* Binding information */
    uint8_t     uid[6];             /**< Unique ID from binding phrase */
    uint8_t     model_id;           /**< Model match ID (0 = disabled) */
    
    /* RF settings */
    fhss_domain_e   domain;         /**< Regulatory domain */
    elrs_rate_index_t initial_rate; /**< Initial air rate */
    int8_t      tx_power_dbm;       /**< TX power for telemetry (dBm) */
    
    /* Feature enables */
    bool        telemetry_enabled;  /**< Enable telemetry transmission */
    bool        model_match_enabled; /**< Enable model match */
    bool        wifi_on_no_conn;    /**< Enter WiFi mode on no connection */
    uint32_t    wifi_timeout_ms;    /**< WiFi timeout (0 = disabled) */
    
    /* Rate switching behavior 
     * Citation: ExpressLRS options.h - lock_on_first_connection
     *   When true, locks the RF rate on first successful connection
     *   preventing rate cycling until power cycle.
     */
    bool        lock_on_first_connection; /**< Lock rate on first connection */
    
    /* CRSF output */
    uint32_t    crsf_baud_rate;     /**< CRSF serial baud rate */
} elrs_main_config_t;

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

/**
 * @brief Callback for connection state changes
 */
typedef void (*elrs_connect_callback_t)(elrs_connection_state_t state);

/**
 * @brief Callback for channel data updates
 */
typedef void (*elrs_channels_callback_t)(const elrs_channel_data_t *channels);

/**
 * @brief Callback for WiFi mode entry
 * 
 * This callback is triggered when ELRS decides to enter WiFi mode
 * (either auto-timeout or manual request). The callback should start
 * the WiFi HTTP server.
 */
typedef void (*elrs_wifi_mode_callback_t)(void);

/*******************************************************************************
 * Initialization
 ******************************************************************************/

/**
 * @brief Initialize the ELRS receiver
 * 
 * This is the main setup function that initializes all ELRS subsystems:
 *   - Hardware timer
 *   - PFD
 *   - Link quality calculator
 *   - Radio (LR1121)
 *   - FHSS
 *   - CRSF output
 * 
 * Citation: ExpressLRS rx_main.cpp - setup()
 * 
 * @param config Configuration parameters
 * @return true on success, false on failure
 */
bool elrs_main_setup(const elrs_main_config_t *config);

/**
 * @brief Get default configuration
 * 
 * Fills the config structure with sensible defaults.
 * 
 * @param config Pointer to config structure to fill
 */
void elrs_main_get_default_config(elrs_main_config_t *config);

/**
 * @brief Shutdown ELRS receiver
 * 
 * Stops all ELRS processing and releases resources.
 */
void elrs_main_shutdown(void);

/*******************************************************************************
 * Main Loop
 ******************************************************************************/

/**
 * @brief Main ELRS processing loop
 * 
 * Call this continuously from your main task. This function:
 *   1. Processes pending radio IRQs
 *   2. Handles connection state updates
 *   3. Sends CRSF output to flight controller
 *   4. Processes telemetry (if enabled)
 * 
 * Citation: ExpressLRS rx_main.cpp - loop()
 * 
 * The actual timing-critical work happens in the hwTimer callbacks
 * (Tick/Tock) which are called from interrupt context.
 * 
 * @return true if a packet was processed this iteration
 */
bool elrs_main_loop(void);

/*******************************************************************************
 * Mode Control
 ******************************************************************************/

/**
 * @brief Get current operating mode
 * 
 * @return Current ELRS mode
 */
elrs_mode_t elrs_main_get_mode(void);

/**
 * @brief Enter WiFi OTA update mode
 * 
 * Stops radio operation and enables WiFi for OTA updates.
 * 
 * @return true if mode entered successfully
 */
bool elrs_main_enter_wifi_mode(void);

/**
 * @brief Exit WiFi mode and resume normal operation
 * 
 * @return true if resumed successfully
 */
bool elrs_main_exit_wifi_mode(void);

/**
 * @brief Enter binding mode
 * 
 * Starts listening for binding packets.
 * 
 * @return true if binding mode entered
 */
bool elrs_main_enter_binding_mode(void);

/**
 * @brief Exit binding mode
 */
void elrs_main_exit_binding_mode(void);

/*******************************************************************************
 * Status & Statistics
 ******************************************************************************/

/**
 * @brief Check if connected to transmitter
 * 
 * @return true if connected
 */
bool elrs_main_is_connected(void);

/**
 * @brief Get connection state
 * 
 * @return Current connection state
 */
elrs_connection_state_t elrs_main_get_connection_state(void);

/**
 * @brief Get current link quality
 * 
 * @return LQ percentage (0-100)
 */
uint8_t elrs_main_get_lq(void);

/**
 * @brief Get current RSSI
 * 
 * @return RSSI in dBm (filtered)
 */
int8_t elrs_main_get_rssi(void);

/**
 * @brief Get current SNR
 * 
 * @return SNR in dB (filtered)
 */
int8_t elrs_main_get_snr(void);

/**
 * @brief Get channel data
 * 
 * @return Pointer to current channel data
 */
const elrs_channel_data_t* elrs_main_get_channels(void);

/**
 * @brief Get link statistics for CRSF
 * 
 * @param stats Pointer to stats structure to fill
 */
void elrs_main_get_link_stats(crsf_link_stats_t *stats);

/*******************************************************************************
 * Rate Control
 ******************************************************************************/

/**
 * @brief Get current air rate
 * 
 * @return Current rate index
 */
elrs_rate_index_t elrs_main_get_rate(void);

/**
 * @brief Force rate change (for testing)
 * 
 * @param rate New rate index
 * @return true if rate changed
 */
bool elrs_main_set_rate(elrs_rate_index_t rate);

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

/**
 * @brief Set connection state change callback
 * 
 * @param callback Function to call when connection state changes
 */
void elrs_main_set_connect_callback(elrs_connect_callback_t callback);

/**
 * @brief Set channel data update callback
 * 
 * @param callback Function to call when channels are updated
 */
void elrs_main_set_channels_callback(elrs_channels_callback_t callback);

/**
 * @brief Set WiFi mode entry callback
 * 
 * The callback is called when ELRS enters WiFi mode (auto or manual).
 * Use this to start the WiFi HTTP server from the main task context.
 * 
 * @param callback Function to call when WiFi mode is entered
 */
void elrs_main_set_wifi_callback(elrs_wifi_mode_callback_t callback);

/*******************************************************************************
 * hwTimer Callbacks (called from ISR context)
 ******************************************************************************/

/**
 * @brief hwTimer TICK callback
 * 
 * Called at mid-packet interval. Handles:
 *   - LQ calculation
 *   - Telemetry transmission
 * 
 * Citation: ExpressLRS rx_main.cpp - HWtimerCallbackTick()
 * 
 * WARNING: Called from ISR context. Keep fast!
 */
void elrs_main_hw_timer_tick(void);

/**
 * @brief hwTimer TOCK callback
 * 
 * Called when packet is expected. Handles:
 *   - Starting RX
 *   - PFD timing
 *   - Timeout detection
 * 
 * Citation: ExpressLRS rx_main.cpp - HWtimerCallbackTock()
 * 
 * WARNING: Called from ISR context. Keep fast!
 */
void elrs_main_hw_timer_tock(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_MAIN_H */
