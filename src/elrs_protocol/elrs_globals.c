/**
 * @file elrs_globals.c
 * @brief Global variable definitions for ELRS protocol
 * 
 * This file contains the actual definitions of global variables
 * declared extern in the header files.
 */

#include "elrs_platform.h"
#include "fhss.h"
#include "ota.h"

/*******************************************************************************
 * UID (Unique Identifier)
 * 
 * The 6-byte UID derived from the binding phrase.
 * Default is all zeros (unbound). Set via elrs_config or binding process.
 ******************************************************************************/
uint8_t UID[UID_LEN] = {0, 0, 0, 0, 0, 0};

/*******************************************************************************
 * Firmware Options
 * 
 * Runtime configuration options. Initialized with defaults.
 ******************************************************************************/
elrs_options_t firmwareOptions = {
    .domain = DOMAIN_FCC915,    /* Default to FCC 915 MHz */
    .hasUID = 0,
    .uid = {0, 0, 0, 0, 0, 0},
    .uart_baud = 420000,        /* Default CRSF baud rate */
    .lock_on_first_connection = false,
    .is_airport = false,
};

/*******************************************************************************
 * Link Statistics
 * 
 * Updated by RX and transmitted to TX via telemetry.
 ******************************************************************************/
elrs_link_stats_t linkStats = {
    .uplink_RSSI_1 = 0,
    .uplink_RSSI_2 = 0,
    .uplink_Link_quality = 0,
    .uplink_SNR = 0,
    .active_antenna = 0,
    .rf_Mode = 0,
    .uplink_TX_Power = 0,
    .downlink_RSSI_1 = 0,
    .downlink_Link_quality = 0,
    .downlink_SNR = 0,
};
