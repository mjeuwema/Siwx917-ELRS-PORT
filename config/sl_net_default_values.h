/***************************************************************************/ /**
 * @file  sl_net_default_values.h
 * @brief Network default configuration for WiFi AP + HTTP test
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * Configuration for ELRS WiFi OTA testing:
 * - AP Mode with static IP
 * - SSID: ELRS_TEST_AP
 * - IP: 192.168.10.10
 *
 ******************************************************************************/

#pragma once

#include "sl_net_wifi_types.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/*******************************************************************************
 * IP Type Configuration
 ******************************************************************************/
#ifdef SLI_SI91X_ENABLE_IPV6
#define REQUIRED_IP_TYPE SL_IPV6
#else
#define REQUIRED_IP_TYPE SL_IPV4
#endif

/*******************************************************************************
 * WiFi Client Configuration (not used in AP test, but required by SDK)
 ******************************************************************************/
#ifndef DEFAULT_WIFI_CLIENT_PROFILE_SSID
#define DEFAULT_WIFI_CLIENT_PROFILE_SSID "YOUR_AP_SSID"
#endif

#ifndef DEFAULT_WIFI_CLIENT_CREDENTIAL
#define DEFAULT_WIFI_CLIENT_CREDENTIAL "YOUR_AP_PASSPHRASE"
#endif

#ifndef DEFAULT_WIFI_CLIENT_SECURITY_TYPE
#define DEFAULT_WIFI_CLIENT_SECURITY_TYPE SL_WIFI_WPA2
#endif

#ifndef DEFAULT_WIFI_CLIENT_ENCRYPTION_TYPE
#define DEFAULT_WIFI_CLIENT_ENCRYPTION_TYPE SL_WIFI_DEFAULT_ENCRYPTION
#endif

/*******************************************************************************
 * WiFi Access Point Configuration
 ******************************************************************************/
#ifndef DEFAULT_WIFI_AP_PROFILE_SSID
#define DEFAULT_WIFI_AP_PROFILE_SSID "ELRS_TEST_AP"
#endif

#ifndef DEFAULT_WIFI_AP_CREDENTIAL
#define DEFAULT_WIFI_AP_CREDENTIAL "elrs1234"
#endif

/*******************************************************************************
 * IP Address Configuration
 *
 * Note: IP addresses are in little-endian format
 * 0x0A0AA8C0 = 192.168.10.10 (C0=192, A8=168, 0A=10, 0A=10)
 ******************************************************************************/

/* IP address: 192.168.10.10 */
#ifndef DEFAULT_WIFI_MODULE_IP_ADDRESS
#define DEFAULT_WIFI_MODULE_IP_ADDRESS 0x0A0AA8C0
#endif

/* Subnet mask: 255.255.255.0 */
#ifndef DEFAULT_WIFI_SN_MASK_ADDRESS
#define DEFAULT_WIFI_SN_MASK_ADDRESS 0x00FFFFFF
#endif

/* Gateway: 192.168.10.10 (self in AP mode) */
#ifndef DEFAULT_WIFI_GATEWAY_ADDRESS
#define DEFAULT_WIFI_GATEWAY_ADDRESS 0x0A0AA8C0
#endif

/*******************************************************************************
 * Default WiFi Client Profile (required by SDK even if not used)
 ******************************************************************************/
#define DEFAULT_WIFI_CLIENT_PROFILE \
  (sl_net_wifi_client_profile_t)    \
  {                                 \
    .config = { \
        .ssid.value = DEFAULT_WIFI_CLIENT_PROFILE_SSID, \
        .ssid.length = sizeof(DEFAULT_WIFI_CLIENT_PROFILE_SSID)-1, \
        .channel.channel = SL_WIFI_AUTO_CHANNEL, \
        .channel.band = SL_WIFI_AUTO_BAND, \
        .channel.bandwidth = SL_WIFI_AUTO_BANDWIDTH, \
        .channel_bitmap.channel_bitmap_2_4 = SL_WIFI_DEFAULT_CHANNEL_BITMAP, \
        .bssid = {{0}}, \
        .bss_type = SL_WIFI_BSS_TYPE_INFRASTRUCTURE, \
        .security = DEFAULT_WIFI_CLIENT_SECURITY_TYPE, \
        .encryption = DEFAULT_WIFI_CLIENT_ENCRYPTION_TYPE, \
        .client_options = 0, \
        .credential_id = SL_NET_DEFAULT_WIFI_CLIENT_CREDENTIAL_ID, \
    }, \
    .ip = { \
        .mode = SL_IP_MANAGEMENT_DHCP, \
        .type = REQUIRED_IP_TYPE, \
        .host_name = NULL, \
        .ip = {{{0}}}, \
    }                  \
  }

/*******************************************************************************
 * Default WiFi Access Point Profile
 ******************************************************************************/
#define DEFAULT_WIFI_ACCESS_POINT_PROFILE \
  (sl_net_wifi_ap_profile_t)              \
  {                                       \
    .config = { \
        .ssid.value = DEFAULT_WIFI_AP_PROFILE_SSID, \
        .ssid.length = sizeof(DEFAULT_WIFI_AP_PROFILE_SSID)-1, \
        .channel.channel = SL_WIFI_AUTO_CHANNEL, \
        .channel.band = SL_WIFI_AUTO_BAND, \
        .channel.bandwidth = SL_WIFI_AUTO_BANDWIDTH, \
        .security = SL_WIFI_WPA2, \
        .encryption = SL_WIFI_CCMP_ENCRYPTION, \
        .rate_protocol = SL_WIFI_RATE_PROTOCOL_AUTO, \
        .options = 0, \
        .credential_id = SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID, \
        .keepalive_type = SL_SI91X_AP_NULL_BASED_KEEP_ALIVE, \
        .beacon_interval = 100, \
        .client_idle_timeout = 0xFF, \
        .dtim_beacon_count = 3, \
        .maximum_clients = 4, \
        .beacon_stop = 0, \
        .tdi_flags = SL_WIFI_TDI_NONE, \
        .is_11n_enabled = 1, \
    }, \
    .ip = { \
      .mode      = SL_IP_MANAGEMENT_STATIC_IP, \
      .type      = SL_IPV4, \
      .host_name = NULL, \
      .ip        = { \
         .v4.ip_address.value = DEFAULT_WIFI_MODULE_IP_ADDRESS, \
         .v4.gateway.value    = DEFAULT_WIFI_GATEWAY_ADDRESS, \
         .v4.netmask.value    = DEFAULT_WIFI_SN_MASK_ADDRESS \
      }, \
    }                        \
  }

/*******************************************************************************
 * Default Credentials
 ******************************************************************************/
static sl_net_wifi_psk_credential_entry_t default_wifi_client_credential = {
  .type        = SL_NET_WIFI_PSK,
  .data_length = sizeof(DEFAULT_WIFI_CLIENT_CREDENTIAL) - 1,
  .data        = DEFAULT_WIFI_CLIENT_CREDENTIAL
};

static sl_net_wifi_psk_credential_entry_t default_wifi_ap_credential = {
  .type        = SL_NET_WIFI_PSK,
  .data_length = sizeof(DEFAULT_WIFI_AP_CREDENTIAL) - 1,
  .data        = DEFAULT_WIFI_AP_CREDENTIAL
};

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
