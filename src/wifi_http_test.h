/***************************************************************************/ /**
 * @file wifi_http_test.h
 * @brief WiFi AP + HTTP Server Test for ELRS OTA preparation
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * WiFi Access Point + HTTP Server test module.
 * This tests the WiFi and HTTP server functionality needed for ELRS OTA updates.
 *
 * Test Procedure:
 * 1. Start WiFi in Access Point mode (creates "ELRS_TEST_AP" network)
 * 2. Start HTTP server on port 80
 * 3. Serve test page at http://192.168.10.10/
 * 4. Log client connections and HTTP requests
 *
 * Citation: Silicon Labs WiseConnect SDK 3.5.2
 * - wifi_access_point_soc example for AP mode
 * - wifi_http_server_soc example for HTTP server
 *
 ******************************************************************************/

#ifndef WIFI_HTTP_TEST_H
#define WIFI_HTTP_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/* WiFi AP Settings */
#define WIFI_TEST_AP_SSID        "ELRS_TEST_AP"
#define WIFI_TEST_AP_PASSWORD    "elrs1234"       /* Minimum 8 characters for WPA2 */
#define WIFI_TEST_AP_CHANNEL     6                /* 2.4GHz channel */

/* IP Configuration (Static for AP mode) */
#define WIFI_TEST_IP_ADDRESS     0x0A0AA8C0       /* 192.168.10.10 */
#define WIFI_TEST_SUBNET_MASK    0x00FFFFFF       /* 255.255.255.0 */
#define WIFI_TEST_GATEWAY        0x0A0AA8C0       /* 192.168.10.10 (self) */

/* HTTP Server Settings */
#define WIFI_TEST_HTTP_PORT      80

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize and start WiFi AP + HTTP server test
 *
 * This function:
 * 1. Initializes WiFi in AP mode with SSID "ELRS_TEST_AP"
 * 2. Configures static IP 192.168.10.10
 * 3. Starts HTTP server on port 80
 * 4. Registers request handlers for test endpoints
 *
 * The function blocks in a FreeRTOS task and handles requests.
 */
void wifi_http_test_run(void);

/**
 * @brief Check if WiFi AP is running
 * @return true if AP is active, false otherwise
 */
bool wifi_http_test_is_ap_running(void);

/**
 * @brief Check if HTTP server is running
 * @return true if server is active, false otherwise
 */
bool wifi_http_test_is_server_running(void);

/**
 * @brief Get the number of connected clients
 * @return Number of WiFi clients currently connected to AP
 */
uint32_t wifi_http_test_get_client_count(void);

/**
 * @brief Get the number of HTTP requests served
 * @return Total count of HTTP requests handled
 */
uint32_t wifi_http_test_get_request_count(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_HTTP_TEST_H */
