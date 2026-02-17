/***************************************************************************/ /**
 * @file wifi_http_test.c
 * @brief WiFi AP + HTTP Server with ELRS Web UI
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Implementation of WiFi Access Point + HTTP Server serving the exact
 * ExpressLRS web interface for OTA firmware updates.
 *
 * Citation: Silicon Labs WiseConnect SDK 3.5.2 Examples
 * - wifi_access_point_soc: AP mode initialization sequence
 * - wifi_http_server_soc: HTTP server request handling
 *
 * Citation: ExpressLRS src/lib/WIFI/devWIFI.cpp
 * - WebUpdateSendContent(): Serves gzip-compressed assets
 * - API endpoints: /config, /networks, /update, /reboot, etc.
 *
 ******************************************************************************/

#include "wifi_http_test.h"
#include "rsi_debug.h"

/* WiseConnect SDK Includes */
#include "sl_net.h"
#include "sl_wifi.h"
#include "sl_net_wifi_types.h"
#include "sl_http_server.h"
#include "sl_utility.h"
#include "sl_wifi_callback_framework.h"

/* CMSIS-RTOS2 for FreeRTOS */
#include "cmsis_os2.h"

/* CMSIS Core for NVIC_SystemReset() 
 * Citation: siw917x-family-rm.pdf Section 7.3.5 "Reset request from host or processor"
 * - "Cortex M4 can request for reset on SYSRESETREQ"
 */
#include "core_cm4.h"

/* Watchdog Timer for Full Device Reset
 * 
 * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
 * - "When the timeout occurs, it generates an interrupt for the processor.
 *    If the processor does not service the interrupt, a watchdog reset is 
 *    generated and the ENTIRE DEVICE is reset."
 * 
 * Citation: siw917x-family-rm.pdf Section 7.3.5 - Reset request from host or processor
 * - "Cortex M4 can request for reset on SYSRESETREQ... the digital blocks are reset."
 *   (NOTE: This is M4 ONLY, NOT the NWP!)
 * 
 * CRITICAL FINDING: sl_si91x_soc_nvic_reset() claims to reset both M4 and NWP, 
 * but the implementation ONLY triggers SYSRESETREQ which resets M4 only.
 * After OTA firmware update, the NWP is in undefined state and WiFi hangs.
 * 
 * SOLUTION: Use Watchdog Timer reset to reset the ENTIRE device (M4 + NWP).
 * Configure WDT with minimal timeout (0 = 2^0 = 1 clock cycle at 32kHz = 31.25us)
 * and don't service the interrupt - this triggers full device reset.
 */
#include "rsi_wwdt.h"
#include "rsi_power_save.h"

/* ELRS Web Content - Pre-built gzip-compressed assets */
#include "elrs_web/WebContent.h"

/* ELRS Configuration Storage (NVM3-backed persistent settings) */
#include "elrs_config.h"

/* LR1121 Driver for firmware OTA updates */
#include "lr1121_driver.h"

/* SiWx917 Firmware Update API
 * Citation: AN1431 - SiWx917 SoC Firmware Update Application Note
 * - sl_si91x_fwup_start(): Send RPS header (64 bytes)
 * - sl_si91x_fwup_load(): Send firmware content in chunks
 * - sl_si91x_fwup_abort(): Abort firmware update before reset
 */
#include "firmware_upgradation.h"

/*******************************************************************************
 * SDK Internal State Variables (External Declarations)
 * 
 * CRITICAL FIX: After WDT reset, the SDK's internal state variables persist
 * in RAM because WDT reset doesn't clear RAM. These variables tell the SDK
 * that the device is "already initialized" which causes it to skip the
 * proper NWP handshake sequence, leading to hangs at sl_net_init().
 * 
 * Citation: sl_si91x_driver.c lines 178-179:
 *   bool device_initialized = false;
 *   bool interface_is_up[SL_WIFI_MAX_INTERFACE_INDEX] = { false, ... };
 * 
 * By declaring these as extern and clearing them before sl_net_init(),
 * we force the SDK to perform a fresh initialization.
 ******************************************************************************/
extern bool device_initialized;                    /* SDK's device init flag */
extern bool interface_is_up[];                     /* Per-interface status array */
#define SL_WIFI_MAX_INTERFACE_INDEX 5              /* From sl_wifi_types.h */

#include <string.h>
#include <stdio.h>
#include <ctype.h>

/*******************************************************************************
 * Macros
 ******************************************************************************/

/* Response buffer size for JSON responses */
#define RESPONSE_BUFFER_SIZE     4096

/* Maximum HTTP headers */
#define MAX_HTTP_HEADERS         8

/*******************************************************************************
 * Base64 Decoding Implementation
 * 
 * Purpose: Workaround for binary data corruption during HTTP multipart upload.
 * 
 * ROOT CAUSE: The HTTP/SPI stack corrupts binary data containing 0x0A (LF):
 *   - Byte preceding 0x0A gets its high bit set (e.g., 0x4A -> 0xAA)  
 *   - The 0x0A byte itself is STRIPPED entirely
 *   This appears to be text-mode line-ending processing somewhere in the stack.
 * 
 * SOLUTION: Client encodes firmware as base64 (which uses only safe ASCII chars),
 *           server decodes back to binary. No 0x0A bytes in transit = no corruption.
 * 
 * Citation: RFC 4648 - The Base16, Base32, and Base64 Data Encodings
 ******************************************************************************/

/* Base64 decoding lookup table (-1 = invalid, -2 = padding '=') */
static const int8_t base64_decode_table[256] = {
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 0x00-0x0F */
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 0x10-0x1F */
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,  /* 0x20-0x2F: +, / */
  52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-2,-1,-1,  /* 0x30-0x3F: 0-9, = */
  -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,  /* 0x40-0x4F: A-O */
  15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,  /* 0x50-0x5F: P-Z */
  -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,  /* 0x60-0x6F: a-o */
  41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,  /* 0x70-0x7F: p-z */
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  /* 0x80-0xFF: invalid */
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
};

/**
 * @brief Decode base64 data in-place
 * 
 * @param data   Input base64 data, output binary data (decoded in-place)
 * @param len    Length of input base64 data  
 * @return       Length of decoded binary data, or -1 on error
 * 
 * Citation: RFC 4648 Section 4 - Base 64 Encoding
 * Every 4 base64 characters decode to 3 bytes of binary data.
 * Padding with '=' indicates fewer output bytes for the final group.
 */
static int32_t base64_decode_inplace(uint8_t *data, uint32_t len)
{
  uint32_t i = 0;      /* Input index */
  uint32_t j = 0;      /* Output index */
  uint32_t pad = 0;    /* Padding count */
  
  /* Skip any whitespace/newlines that might be in the base64 data */
  while (len > 0) {
    /* Process 4 characters at a time */
    uint8_t quad[4];
    uint32_t quad_idx = 0;
    
    /* Collect 4 valid base64 characters */
    while (quad_idx < 4 && i < len) {
      uint8_t c = data[i++];
      
      /* Skip whitespace (CR, LF, space, tab) */
      if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
        continue;
      }
      
      int8_t val = base64_decode_table[c];
      if (val == -1) {
        /* Invalid character */
        DEBUGOUT("[Base64] Invalid char 0x%02X at position %lu\n", c, (unsigned long)(i-1));
        return -1;
      }
      if (val == -2) {
        /* Padding '=' */
        pad++;
        quad[quad_idx++] = 0;
      } else {
        quad[quad_idx++] = (uint8_t)val;
      }
    }
    
    /* If we didn't get 4 characters, we're done or error */
    if (quad_idx < 4) {
      if (quad_idx == 0) {
        /* Clean end */
        break;
      }
      /* Incomplete quad is an error */
      DEBUGOUT("[Base64] Incomplete quad at end, got %lu chars\n", (unsigned long)quad_idx);
      return -1;
    }
    
    /* Decode the 4 base64 chars to 3 bytes */
    /* Citation: RFC 4648 - bit arrangement:
     *   char0[5:0] char1[5:4] -> byte0[7:0]
     *   char1[3:0] char2[5:2] -> byte1[7:0]  
     *   char2[1:0] char3[5:0] -> byte2[7:0]
     */
    data[j++] = (quad[0] << 2) | (quad[1] >> 4);
    if (pad < 2) {
      data[j++] = (quad[1] << 4) | (quad[2] >> 2);
    }
    if (pad < 1) {
      data[j++] = (quad[2] << 6) | quad[3];
    }
    
    /* If we hit padding, we're done */
    if (pad > 0) {
      break;
    }
  }
  
  return (int32_t)j;
}

/*******************************************************************************
 * Full Device Reset via Watchdog Timer
 * 
 * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
 * "When the timeout occurs, it generates an interrupt for the processor.
 *  If the processor does not service the interrupt, a watchdog reset is 
 *  generated and the ENTIRE DEVICE is reset."
 * 
 * Citation: siw917x-family-rm.pdf Section 39 - WDT (Watchdog Timer)
 * "The WDT (Watchdog Timer) generates an early warning interrupt and, 
 *  if not serviced, a system reset."
 * 
 * CRITICAL: sl_si91x_soc_nvic_reset() ONLY resets M4 via SYSRESETREQ!
 * After OTA firmware update, NWP is in undefined state and WiFi hangs.
 * Watchdog reset is the ONLY way to reset BOTH M4 AND NWP.
 ******************************************************************************/

/**
 * @brief Perform full device reset using Watchdog Timer
 * 
 * This function triggers a full SoC reset that resets BOTH the M4 processor
 * AND the NWP (Network Wireless Processor). This is required after OTA
 * firmware updates because:
 * 
 * 1. SYSRESETREQ (used by sl_si91x_soc_nvic_reset()) only resets M4
 * 2. After firmware update, NWP may be in undefined state  
 * 3. On reboot, WiFi init hangs waiting for TA_is_active flag
 * 4. Watchdog reset resets the ENTIRE device to clean state
 * 
 * Citation: siw917x-family-rm.pdf Section 39.3.2 - Programming Sequence
 * 1. Power up the watchdog domain
 * 2. Enable the watchdog timer using WWD_TIMER_EN (0xAA)
 * 3. Configure system reset timer (minimal value for fast reset)
 * 4. Configure interrupt timer (must be less than reset timer)
 * 5. Start the timer using WWD_TIMER_RSTART
 * 6. Do NOT service the interrupt -> triggers full device reset
 * 
 * @note This function never returns - device will reset
 */
static void trigger_full_device_reset(void)
{
  DEBUGOUT("[System] Triggering FULL device reset via Watchdog Timer\n");
  DEBUGOUT("[System] Citation: siw917x-family-rm.pdf Section 7.3.4\n");
  DEBUGOUT("[System] 'If the processor does not service the interrupt,\n");
  DEBUGOUT("[System]  a watchdog reset is generated and the ENTIRE DEVICE is reset.'\n");
  
  /* Disable interrupts to prevent any handler from servicing the WDT */
  __disable_irq();
  
  /* Power up the watchdog domain if needed
   * Citation: siw917x-family-rm.pdf Section 9 - Power Architecture
   * MCU_AON_NPSS_PWRCTRL_SET_REG controls power to NPSS peripherals
   */
  RSI_PS_NpssPeriPowerUp(SLPSS_PWRGATE_ULP_MCUWDT);
  
  /* Small delay to ensure power domain is stable */
  for (volatile uint32_t i = 0; i < 10000; i++) {
    /* Wait */
  }
  
  /* CRITICAL: Configure WDT to trigger full Power-On Reset (POR) instead of non-POR reset
   * Citation: siw917x-family-rm.pdf Section 9.14.7 - MCUAON_WDT_CHIP_RST Register
   * Base address: 0x2404_8000, Offset: 0x01C
   * MCU_WDT_BASED_CHIP_RESET bit [0]:
   *   - When set to '1' (default): non-POR reset triggered by WDT (only resets M4)
   *   - When cleared to '0': Power-On Reset (POR) triggered by WDT (resets ENTIRE device including NWP)
   * 
   * This is the KEY to resetting BOTH the M4 and NWP processors!
   */
  #define MCUAON_WDT_CHIP_RST_REG  (*(volatile uint32_t *)(0x24048000 + 0x01C))
  MCUAON_WDT_CHIP_RST_REG = 0x0;  /* Clear bit 0 to enable POR mode */
  DEBUGOUT("[System] WDT chip reset configured for POR mode (both M4 and NWP will reset)\n");
  
  /* Initialize the WDT
   * Citation: siw917x-family-rm.pdf Section 39.5.3
   * Write 0xAA to WWD_TIMER_EN to enable WDT
   */
  RSI_WWDT_Init(MCU_WDT);
  
  /* Configure timers for immediate reset
   * Citation: siw917x-family-rm.pdf Section 39.5.1, 39.5.2
   * - System reset interval: 2^WWD_SYSTEM_RESET_TIMER clock cycles
   * - Interrupt interval: 2^WWD_INTERRUPT_TIMER clock cycles  
   * - Using 32kHz clock: value 0 = 2^0 = 1 cycle = 31.25us
   * - Using value 1 for interrupt (62.5us) and 2 for reset (125us)
   */
  RSI_WWDT_ConfigIntrTimer(MCU_WDT, 1);      /* ~62.5us at 32kHz */
  RSI_WWDT_ConfigSysRstTimer(MCU_WDT, 3);    /* ~250us at 32kHz */
  
  /* Mask the WDT interrupt so it won't be serviced
   * Citation: siw917x-family-rm.pdf Section 39 - if interrupt not serviced, reset occurs
   */
  RSI_WWDT_IntrMask();
  
  /* Start the watchdog timer
   * Citation: siw917x-family-rm.pdf Section 39.5.3
   * Write 1 to WWD_TIMER_RSTART to start timer
   */
  RSI_WWDT_Start(MCU_WDT);
  
  DEBUGOUT("[System] Watchdog started - full POR device reset in ~250us...\n");
  
  /* Wait for watchdog to trigger full device reset */
  while (1) {
    /* Infinite loop - WDT will reset the entire SoC */
    __WFI();  /* Wait for interrupt (reset) */
  }
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Case-insensitive string comparison (portable alternative to strcasecmp)
 * 
 * @param s1 First string
 * @param s2 Second string
 * @return 0 if equal (case-insensitive), non-zero otherwise
 */
static int str_icmp(const char *s1, const char *s2)
{
  while (*s1 && *s2) {
    int c1 = tolower((unsigned char)*s1);
    int c2 = tolower((unsigned char)*s2);
    if (c1 != c2) {
      return c1 - c2;
    }
    s1++;
    s2++;
  }
  return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

/*******************************************************************************
 * CORS Headers for Browser Compatibility
 * 
 * Citation: MDN Web Docs - Cross-Origin Resource Sharing (CORS)
 * Browsers block cross-origin requests unless server sends these headers.
 * Since we're serving from http://192.168.10.10, any JavaScript fetch() 
 * needs CORS headers to allow the request.
 ******************************************************************************/
#define CORS_HEADER_ALLOW_ORIGIN   "Access-Control-Allow-Origin"
#define CORS_HEADER_ALLOW_METHODS  "Access-Control-Allow-Methods"
#define CORS_HEADER_ALLOW_HEADERS  "Access-Control-Allow-Headers"
#define CORS_VALUE_ALLOW_ORIGIN    "*"
#define CORS_VALUE_ALLOW_METHODS   "GET, POST, OPTIONS"
#define CORS_VALUE_ALLOW_HEADERS   "Content-Type, Accept"

/* Firmware version */
#define ELRS_VERSION             "4.0.0-SiWx917"
#define ELRS_TARGET              "SIWG917Y_LR1121"

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/* State tracking */
static volatile bool ap_running = false;
static volatile bool server_running = false;
static volatile uint32_t client_count = 0;
static volatile uint32_t request_count = 0;
static volatile bool config_save_pending = false;  /* Deferred NVM3 save flag */

/* HTTP Server handle */
static sl_http_server_t server_handle = { 0 };

/* Response buffer */
static char response_buffer[RESPONSE_BUFFER_SIZE];

/* Config initialization flag */
static bool config_initialized = false;

/*******************************************************************************
 * WiFi AP Profile Configuration
 ******************************************************************************/

/* AP Credential */
static sl_net_wifi_psk_credential_entry_t wifi_ap_credential = {
  .type        = SL_NET_WIFI_PSK,
  .data_length = sizeof(WIFI_TEST_AP_PASSWORD) - 1,
  .data        = WIFI_TEST_AP_PASSWORD
};

/* AP Profile Configuration */
static sl_net_wifi_ap_profile_t wifi_ap_profile = {
  .config = {
    .ssid.value   = WIFI_TEST_AP_SSID,
    .ssid.length  = sizeof(WIFI_TEST_AP_SSID) - 1,
    .channel      = {
      .channel    = WIFI_TEST_AP_CHANNEL,
      .band       = SL_WIFI_BAND_2_4GHZ,
      .bandwidth  = SL_WIFI_BANDWIDTH_20MHz,
    },
    .security            = SL_WIFI_WPA2,
    .encryption          = SL_WIFI_CCMP_ENCRYPTION,
    .rate_protocol       = SL_WIFI_RATE_PROTOCOL_AUTO,
    .options             = 0,
    .credential_id       = SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID,
    .keepalive_type      = SL_SI91X_AP_NULL_BASED_KEEP_ALIVE,
    .beacon_interval     = 100,
    .client_idle_timeout = 0xFF,
    .dtim_beacon_count   = 3,
    .maximum_clients     = 4,
    .beacon_stop         = 0,
    .tdi_flags           = SL_WIFI_TDI_NONE,
    .is_11n_enabled      = 1,
  },
  .ip = {
    .mode      = SL_IP_MANAGEMENT_STATIC_IP,
    .type      = SL_IPV4,
    .host_name = NULL,
    .ip        = {
      .v4.ip_address.value = WIFI_TEST_IP_ADDRESS,
      .v4.gateway.value    = WIFI_TEST_GATEWAY,
      .v4.netmask.value    = WIFI_TEST_SUBNET_MASK,
    },
  },
};

/*******************************************************************************
 * Request Type Strings (for logging)
 ******************************************************************************/
static const char *request_type_str[] = {
  [SL_HTTP_REQUEST_GET]    = "GET",
  [SL_HTTP_REQUEST_POST]   = "POST",
  [SL_HTTP_REQUEST_PUT]    = "PUT",
  [SL_HTTP_REQUEST_DELETE] = "DELETE",
  [SL_HTTP_REQUEST_HEAD]   = "HEAD",
};

/*******************************************************************************
 * CORS Helper - Adds CORS headers to response
 * 
 * Citation: MDN Web Docs - CORS
 * Required for browser JavaScript to make fetch() requests to our API endpoints.
 ******************************************************************************/
static sl_http_header_t cors_headers[] = {
  { .key = CORS_HEADER_ALLOW_ORIGIN,  .value = CORS_VALUE_ALLOW_ORIGIN },
  { .key = CORS_HEADER_ALLOW_METHODS, .value = CORS_VALUE_ALLOW_METHODS },
  { .key = CORS_HEADER_ALLOW_HEADERS, .value = CORS_VALUE_ALLOW_HEADERS },
};
#define CORS_HEADER_COUNT 3

/**
 * @brief Handle OPTIONS preflight requests (CORS)
 * 
 * Citation: MDN Web Docs - Preflight request
 * Browsers send OPTIONS request before POST to check if CORS is allowed.
 */
static sl_status_t handle_cors_preflight(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  
  DEBUGOUT("[HTTP] OPTIONS %s (CORS preflight)\n", req->uri.path);
  
  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = cors_headers;
  response.header_count         = CORS_HEADER_COUNT;
  response.data                 = (uint8_t *)"";
  response.current_data_length  = 0;
  response.expected_data_length = 0;
  
  return sl_http_server_send_response(handle, &response);
}

/*******************************************************************************
 * ELRS Web Asset Handler
 *
 * Citation: ExpressLRS devWIFI.cpp - WebUpdateSendContent()
 * Serves pre-compressed gzip assets with Content-Encoding: gzip header
 ******************************************************************************/

/**
 * @brief Serve ELRS web assets (gzip-compressed)
 * 
 * Matches the exact ELRS behavior from devWIFI.cpp:
 * - Looks up path in WEB_ASSETS array
 * - Sends gzip-compressed content with appropriate headers
 * 
 * @param handle HTTP server handle
 * @param path URL path to serve
 * @return sl_status_t SL_STATUS_OK if asset found and sent
 */
static sl_status_t serve_web_asset(sl_http_server_t *handle, 
                                   const char *path)
{
  sl_http_server_response_t response = { 0 };
  
  /* Headers for gzip-compressed content */
  sl_http_header_t headers[2] = {
    { .key = "Content-Encoding", .value = "gzip" },
    { .key = "Cache-Control",    .value = "no-cache" }
  };

  /* Search for asset in ELRS web content */
  for (size_t i = 0; i < WEB_ASSETS_COUNT; i++) {
    if (strcmp(path, WEB_ASSETS[i].path) == 0) {
      DEBUGOUT("[HTTP] Serving asset: %s (%u bytes gzip)\n", 
               path, (unsigned int)WEB_ASSETS[i].size);

      /* Determine content type based on asset's content_type field
       * Citation: sl_http_server_types.h - SL_HTTP_CONTENT_TYPE_TEXT_JAVASCRIPT
       * Browsers require correct Content-Type to execute JavaScript (MIME sniffing protection)
       */
      if (strstr(WEB_ASSETS[i].content_type, "javascript")) {
        response.content_type = SL_HTTP_CONTENT_TYPE_TEXT_JAVASCRIPT;
      } else if (strstr(WEB_ASSETS[i].content_type, "css")) {
        response.content_type = SL_HTTP_CONTENT_TYPE_TEXT_CSS;
      } else if (strstr(WEB_ASSETS[i].content_type, "html")) {
        response.content_type = SL_HTTP_CONTENT_TYPE_TEXT_HTML;
      } else {
        response.content_type = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
      }

      response.response_code        = SL_HTTP_RESPONSE_OK;
      response.headers              = headers;
      response.header_count         = 2;
      response.data                 = (uint8_t *)WEB_ASSETS[i].data;
      response.current_data_length  = WEB_ASSETS[i].size;
      response.expected_data_length = WEB_ASSETS[i].size;

      return sl_http_server_send_response(handle, &response);
    }
  }

  return SL_STATUS_NOT_FOUND;
}

/*******************************************************************************
 * HTTP Request Handlers
 ******************************************************************************/

/**
 * @brief Handler for root path "/" - serve index.html directly
 * 
 * Citation: ExpressLRS devWIFI.cpp - WebUpdateHandleRoot()
 */
static sl_status_t handle_root(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  request_count++;
  DEBUGOUT("[HTTP] %s / -> Serving /index.html\n", request_type_str[req->type]);
  
  /* Serve index.html directly instead of redirect */
  return serve_web_asset(handle, "/index.html");
}

/**
 * @brief Handler for /index.html - serve ELRS main page
 */
static sl_status_t handle_index(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  request_count++;
  DEBUGOUT("[HTTP] %s /index.html\n", request_type_str[req->type]);
  return serve_web_asset(handle, "/index.html");
}

/**
 * @brief Handler for GET /config endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - GetConfiguration()
 * Returns JSON configuration from NVM3-backed persistent storage
 */
static sl_status_t handle_config_get(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  /* Include CORS headers for browser compatibility */
  sl_http_header_t headers[5] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = "Cache-Control",             .value = "no-cache" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = CORS_HEADER_ALLOW_HEADERS,   .value = CORS_VALUE_ALLOW_HEADERS }
  };
  int len;

  (void)req;
  request_count++;
  DEBUGOUT("[HTTP] GET /config\n");

  /* Generate JSON from stored configuration
   * Citation: elrs_config.c - elrs_config_to_json()
   */
  len = elrs_config_to_json(response_buffer, RESPONSE_BUFFER_SIZE);
  if (len < 0) {
    DEBUGOUT("[HTTP] ERROR: Failed to generate config JSON\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE, 
                   "{\"error\":\"Failed to read configuration\"}");
  }

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 5;  /* Include CORS headers */
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for POST /config endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - UpdateConfiguration()
 * Parses JSON body, updates config, and saves to NVM3
 */
static sl_status_t handle_config_post(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  /* Include CORS headers for browser compatibility */
  sl_http_header_t headers[5] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = "Cache-Control",             .value = "no-cache" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = CORS_HEADER_ALLOW_HEADERS,   .value = CORS_VALUE_ALLOW_HEADERS }
  };
  static char body_buffer[1024];
  int result;

  request_count++;
  DEBUGOUT("[HTTP] POST /config - Updating configuration\n");

  /* Read request body */
  sl_http_recv_req_data_t recv_data = {
    .request       = req,
    .buffer        = (uint8_t *)body_buffer,
    .buffer_length = sizeof(body_buffer) - 1
  };

  sl_status_t status = sl_http_server_read_request_data(handle, &recv_data);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[HTTP] ERROR: Failed to read request body: 0x%lX\n", status);
    int len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Failed to read request body\"}");
    response.response_code        = SL_HTTP_RESPONSE_BAD_REQUEST;
    response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
    response.headers              = headers;
    response.header_count         = 5;  /* Include CORS headers */
    response.data                 = (uint8_t *)response_buffer;
    response.current_data_length  = len;
    response.expected_data_length = len;
    return sl_http_server_send_response(handle, &response);
  }

  /* Null-terminate the body */
  body_buffer[recv_data.received_data_length] = '\0';
  DEBUGOUT("[HTTP] Received config JSON (%u bytes): %s\n", 
           (unsigned int)recv_data.received_data_length, body_buffer);

  /* Parse JSON and update configuration
   * Citation: elrs_config.c - elrs_config_from_json()
   */
  result = elrs_config_from_json(body_buffer, recv_data.received_data_length);
  if (result != 0) {
    DEBUGOUT("[HTTP] ERROR: Failed to parse config JSON: %d\n", result);
    int len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Invalid JSON format\"}");
    response.response_code        = SL_HTTP_RESPONSE_BAD_REQUEST;
    response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
    response.headers              = headers;
    response.header_count         = 5;  /* Include CORS headers */
    response.data                 = (uint8_t *)response_buffer;
    response.current_data_length  = len;
    response.expected_data_length = len;
    return sl_http_server_send_response(handle, &response);
  }

  /* Mark config for deferred save
   * 
   * IMPORTANT: NVM3 writes can hang when called from HTTP callback context
   * while WiFi is active (NWP/M4 flash contention). Instead of saving here,
   * we set a flag that the main loop checks and performs the save outside
   * the HTTP callback context.
   * 
   * Citation: Silicon Labs Common Flash mode - NWP and M4 share flash access
   */
  DEBUGOUT("[HTTP] Config updated - marking for deferred save\n");
  extern void wifi_http_request_config_save(void);
  wifi_http_request_config_save();

  /* Success response - config will be saved by main loop */
  int len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                     "{\"status\":\"ok\",\"msg\":\"Configuration updated (saving...)\"}");

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 5;  /* Include CORS headers */
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  DEBUGOUT("[HTTP] Sending response, save will happen in main loop\n");

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for /config endpoint - dispatches to GET, POST, or OPTIONS
 * 
 * Citation: ExpressLRS devWIFI.cpp - GetConfiguration() / UpdateConfiguration()
 * Citation: MDN Web Docs - CORS preflight for POST requests with JSON body
 */
static sl_status_t handle_config(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  DEBUGOUT("[HTTP] /config handler called - method type: %d\n", req->type);
  
  if (req->type == SL_HTTP_REQUEST_POST) {
    return handle_config_post(handle, req);
  } else if (req->type == SL_HTTP_REQUEST_GET) {
    return handle_config_get(handle, req);
  } else {
    /* Handle OPTIONS preflight and other methods with CORS response */
    DEBUGOUT("[HTTP] /config - CORS preflight or unknown method\n");
    return handle_cors_preflight(handle, req);
  }
}

/**
 * @brief Handler for /networks endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - WebUpdateSendNetworks()
 * Returns empty array since we're in AP mode
 */
static sl_status_t handle_networks(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static const char networks_json[] = "[]";

  request_count++;
  DEBUGOUT("[HTTP] %s /networks\n", request_type_str[req->type]);

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = &header;
  response.header_count         = 1;
  response.data                 = (uint8_t *)networks_json;
  response.current_data_length  = sizeof(networks_json) - 1;
  response.expected_data_length = sizeof(networks_json) - 1;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for /reboot endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - HandleReboot()
 */
static sl_status_t handle_reboot(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static const char reboot_msg[] = "{\"status\":\"ok\",\"msg\":\"Rebooting...\"}";

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("**********************************************************\n");
  DEBUGOUT("**** /reboot ENDPOINT CALLED! Method: %s ****\n", request_type_str[req->type]);
  DEBUGOUT("**********************************************************\n");

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = &header;
  response.header_count         = 1;
  response.data                 = (uint8_t *)reboot_msg;
  response.current_data_length  = sizeof(reboot_msg) - 1;
  response.expected_data_length = sizeof(reboot_msg) - 1;

  sl_status_t status = sl_http_server_send_response(handle, &response);
  
  /* 
   * Schedule full device reset after response sent
   * 
   * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
   * "If the processor does not service the interrupt, a watchdog reset is
   *  generated and the ENTIRE DEVICE is reset."
   * 
   * CRITICAL: sl_si91x_soc_nvic_reset() only resets M4 via SYSRESETREQ!
   * After firmware update, NWP is in undefined state - must use WDT reset.
   */
  DEBUGOUT("[System] Rebooting...\n");
  
  /* Small delay to ensure HTTP response is sent before reset */
  osDelay(500);
  
  /* CRITICAL: Gracefully shut down WiFi stack BEFORE reset */
  DEBUGOUT("[System] Shutting down WiFi AP interface cleanly...\n");
  sl_status_t deinit_status = sl_net_deinit(SL_NET_WIFI_AP_INTERFACE);
  if (deinit_status == SL_STATUS_OK) {
    DEBUGOUT("[System] WiFi AP interface shut down successfully.\n");
  } else {
    DEBUGOUT("[System] WARNING: WiFi deinit returned 0x%lx\n", deinit_status);
  }
  
  /* Wait for NWP to complete shutdown */
  osDelay(500);
  
  /* Trigger full device reset via Watchdog Timer */
  trigger_full_device_reset();
  
  /* Should never reach here */
  return status;
}

/*******************************************************************************
 * SiWx917 MCU Firmware Update (RPS Format)
 * 
 * Citation: AN1431 - SiWx917 SoC Firmware Update Application Note
 * Citation: siw917x-family-rm.pdf Section 48.8.1 - RPS Header Format
 * 
 * The RPS (Remote Programming Service) format is a proprietary binary format
 * used by the SiWx917 bootloader. It consists of:
 *   - 64-byte RPS header (magic word, image size, CRC, version, etc.)
 *   - Boot descriptors
 *   - Application binary image
 *   - Optional digital signature (trailer)
 ******************************************************************************/

/* RPS Header Size - fixed 64 bytes */
#define RPS_HEADER_SIZE         64

/* RPS Magic Word for valid firmware image 
 * Citation: AN1431 Table 2.1 - "Magic Word: 0x900D900D"
 */
#define RPS_MAGIC_WORD          0x900D900D

/* RPS Control Flag bits
 * Citation: AN1431 Table 2.1 - Control flag bitmap
 */
#define RPS_CTRL_MCU_IMAGE      (1 << 0)  /* 1=MCU image, 0=NWP image */
#define RPS_CTRL_ENCRYPTED      (1 << 1)  /* 1=Encrypted, 0=Not encrypted */
#define RPS_CTRL_MIC_CHECK      (1 << 2)  /* 1=MIC based, 0=CRC based */
#define RPS_CTRL_SIGNED         (1 << 3)  /* 1=Digitally signed */

/**
 * @brief RPS Header structure
 * 
 * Citation: AN1431 Table 2.1 - RPS Header Fields
 * Total size: 64 bytes
 */
typedef struct __attribute__((packed)) {
  uint16_t control_flag;      /* Offset 0x00: Image info bitmap */
  uint16_t sha_type;          /* Offset 0x02: 1=SHA256, 2=SHA384, 3=SHA512 */
  uint32_t magic_word;        /* Offset 0x04: Must be 0x900D900D */
  uint32_t image_size;        /* Offset 0x08: Size of the image */
  uint32_t fw_version;        /* Offset 0x0C: Firmware version */
  uint32_t flash_location;    /* Offset 0x10: Flash address */
  uint32_t crc;               /* Offset 0x14: CRC of the image */
  uint8_t  mic[16];           /* Offset 0x18: Message Integrity Code */
  uint32_t reserved1;         /* Offset 0x28: Reserved */
  uint32_t ext_fw_version;    /* Offset 0x2C: Extended FW version info */
  uint8_t  reserved2[16];     /* Offset 0x30: Reserved (or combined image info) */
} rps_header_t;

/* Static assertion to verify header size */
_Static_assert(sizeof(rps_header_t) == RPS_HEADER_SIZE, 
               "RPS header must be exactly 64 bytes");

/* Firmware update state machine */
typedef enum {
  FW_UPDATE_IDLE = 0,
  FW_UPDATE_HEADER_RECEIVED,
  FW_UPDATE_IN_PROGRESS,
  FW_UPDATE_COMPLETE,
  FW_UPDATE_ERROR
} fw_update_state_t;

/* Firmware update context */
static struct {
  fw_update_state_t state;
  rps_header_t      header;
  uint32_t          bytes_received;
  uint32_t          expected_size;
  bool              header_stripped;
} fw_update_ctx = { .state = FW_UPDATE_IDLE };

/**
 * @brief Validate RPS header
 * 
 * @param header Pointer to RPS header
 * @return 0 on success, negative error code on failure
 */
static int validate_rps_header(const rps_header_t *header)
{
  /* Check magic word
   * Citation: AN1431 - "Magic Word: 0x900D900D"
   */
  if (header->magic_word != RPS_MAGIC_WORD) {
    DEBUGOUT("[FW] ERROR: Invalid magic word 0x%08lX (expected 0x%08lX)\n",
             (unsigned long)header->magic_word, (unsigned long)RPS_MAGIC_WORD);
    return -1;
  }
  
  /* Validate image size - must be reasonable */
  if (header->image_size == 0 || header->image_size > (8 * 1024 * 1024)) {
    DEBUGOUT("[FW] ERROR: Invalid image size %lu bytes\n", 
             (unsigned long)header->image_size);
    return -2;
  }
  
  DEBUGOUT("[FW] RPS Header Validation PASSED:\n");
  DEBUGOUT("[FW]   Control Flag: 0x%04X (%s, %s, %s, %s)\n",
           header->control_flag,
           (header->control_flag & RPS_CTRL_MCU_IMAGE) ? "MCU" : "NWP",
           (header->control_flag & RPS_CTRL_ENCRYPTED) ? "Encrypted" : "Plain",
           (header->control_flag & RPS_CTRL_MIC_CHECK) ? "MIC" : "CRC",
           (header->control_flag & RPS_CTRL_SIGNED) ? "Signed" : "Unsigned");
  DEBUGOUT("[FW]   Image Size: %lu bytes\n", (unsigned long)header->image_size);
  DEBUGOUT("[FW]   FW Version: 0x%08lX\n", (unsigned long)header->fw_version);
  DEBUGOUT("[FW]   Flash Loc:  0x%08lX\n", (unsigned long)header->flash_location);
  DEBUGOUT("[FW]   CRC:        0x%08lX\n", (unsigned long)header->crc);
  
  return 0;
}

/**
 * @brief Handler for POST /update endpoint (MCU firmware upload)
 * 
 * Citation: AN1431 Section 4.1.2 - "Firmware Update via M4 as Host"
 * 
 * This endpoint handles firmware uploads for the SiWx917 MCU using the
 * "M4 as Host" update mechanism. The firmware must be in RPS format.
 * 
 * Expected headers:
 *   X-FileSize: <total file size in bytes>
 *   Content-Type: multipart/form-data
 * 
 * Update flow:
 *   1. Parse multipart header to find firmware binary
 *   2. Extract and validate 64-byte RPS header
 *   3. Call sl_si91x_fwup_start() with RPS header
 *   4. Stream remaining data via sl_si91x_fwup_load() in chunks
 *   5. On completion, device will reset and boot new firmware
 */
static sl_status_t handle_update(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[4] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = "Connection",                .value = "close" }
  };
  static uint8_t data_buffer[1024];  /* Receive buffer for firmware chunks */
  uint32_t expected_size = 0;
  sl_status_t status;
  int result;
  int len;

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("************************************************************\n");
  DEBUGOUT("**** POST /update - SiWx917 MCU Firmware Update ****\n");
  DEBUGOUT("************************************************************\n");

  /* Handle OPTIONS preflight for CORS */
  if (req->type != SL_HTTP_REQUEST_POST) {
    return handle_cors_preflight(handle, req);
  }

  /* Get X-FileSize header 
   * Citation: ELRS WebUI sends X-FileSize for total firmware size
   */
  sl_http_header_t request_headers[MAX_HTTP_HEADERS];
  sl_status_t hdr_status = sl_http_server_get_request_headers(handle, req, 
                                                               request_headers, 
                                                               MAX_HTTP_HEADERS);
  if (hdr_status != SL_STATUS_OK) {
    DEBUGOUT("[FW] Failed to get request headers: 0x%lx\n", (unsigned long)hdr_status);
  } else {
    for (uint16_t i = 0; i < req->request_header_count && i < MAX_HTTP_HEADERS; i++) {
      if (request_headers[i].key != NULL && request_headers[i].value != NULL) {
        DEBUGOUT("[FW] Header: %s = %s\n", 
                 request_headers[i].key,
                 request_headers[i].value);
        
        if (str_icmp(request_headers[i].key, "X-FileSize") == 0) {
          expected_size = (uint32_t)atoi(request_headers[i].value);
        }
      }
    }
  }

  if (expected_size == 0) {
    DEBUGOUT("[FW] ERROR: X-FileSize header missing or zero\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"X-FileSize header required\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  /* Validate minimum size for RPS header */
  if (expected_size < RPS_HEADER_SIZE + 256) {
    DEBUGOUT("[FW] ERROR: File too small for valid firmware (%lu bytes)\n",
             (unsigned long)expected_size);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"File too small for valid firmware\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  DEBUGOUT("[FW] MCU firmware upload starting, expected size: %lu bytes\n",
           (unsigned long)expected_size);

  /* Initialize update context */
  memset(&fw_update_ctx, 0, sizeof(fw_update_ctx));
  fw_update_ctx.expected_size = expected_size;
  fw_update_ctx.state = FW_UPDATE_IDLE;

  /* Read and process firmware data in chunks */
  uint32_t total_received = 0;
  uint32_t chunk_number = 0;
  bool rps_header_sent = false;
  uint8_t header_buffer[RPS_HEADER_SIZE];
  uint32_t header_bytes_collected = 0;
  
  while (total_received < expected_size) {
    sl_http_recv_req_data_t recv_data = {
      .request       = req,
      .buffer        = data_buffer,
      .buffer_length = sizeof(data_buffer)
    };

    status = sl_http_server_read_request_data(handle, &recv_data);
    if (status != SL_STATUS_OK) {
      if (recv_data.received_data_length == 0) {
        break;
      }
    }

    if (recv_data.received_data_length > 0) {
      chunk_number++;
      uint8_t *write_ptr = data_buffer;
      uint32_t bytes_available = recv_data.received_data_length;
      
      DEBUGOUT("[FW] Chunk #%lu: %lu bytes\n", 
               (unsigned long)chunk_number, (unsigned long)bytes_available);

      /* Strip multipart header from the FIRST chunk */
      if (!fw_update_ctx.header_stripped) {
        /* Look for "\r\n\r\n" which marks end of multipart headers */
        for (uint32_t i = 0; i + 3 < bytes_available; i++) {
          if (data_buffer[i] == '\r' && data_buffer[i+1] == '\n' &&
              data_buffer[i+2] == '\r' && data_buffer[i+3] == '\n') {
            uint32_t header_size = i + 4;
            write_ptr = data_buffer + header_size;
            bytes_available -= header_size;
            fw_update_ctx.header_stripped = true;
            DEBUGOUT("[FW] Stripped multipart header (%lu bytes)\n",
                     (unsigned long)header_size);
            break;
          }
        }
        
        if (!fw_update_ctx.header_stripped) {
          DEBUGOUT("[FW] WARNING: Multipart header not found, assuming raw binary\n");
          fw_update_ctx.header_stripped = true;
        }
      }
      
      /* Truncate at expected_size to avoid multipart boundary data */
      uint32_t bytes_to_process = bytes_available;
      if (total_received + bytes_to_process > expected_size) {
        bytes_to_process = expected_size - total_received;
      }

      /* Collect RPS header (first 64 bytes) before sending to bootloader */
      if (!rps_header_sent) {
        uint32_t header_bytes_needed = RPS_HEADER_SIZE - header_bytes_collected;
        uint32_t bytes_for_header = (bytes_to_process < header_bytes_needed) 
                                     ? bytes_to_process : header_bytes_needed;
        
        memcpy(header_buffer + header_bytes_collected, write_ptr, bytes_for_header);
        header_bytes_collected += bytes_for_header;
        write_ptr += bytes_for_header;
        bytes_to_process -= bytes_for_header;
        
        if (header_bytes_collected == RPS_HEADER_SIZE) {
          /* We have complete RPS header - validate and send to bootloader */
          memcpy(&fw_update_ctx.header, header_buffer, RPS_HEADER_SIZE);
          
          /* Debug: Show first 16 bytes of received header */
          DEBUGOUT("[FW] RPS Header first 16 bytes: ");
          for (int i = 0; i < 16; i++) {
            DEBUGOUT("%02X ", header_buffer[i]);
          }
          DEBUGOUT("\n");
          
          result = validate_rps_header(&fw_update_ctx.header);
          if (result != 0) {
            DEBUGOUT("[FW] ERROR: RPS header validation failed: %d\n", result);
            DEBUGOUT("[FW] Expected magic at offset 4: 0x900D900D\n");
            DEBUGOUT("[FW] Received magic: 0x%02X%02X%02X%02X\n",
                     header_buffer[7], header_buffer[6], header_buffer[5], header_buffer[4]);
            len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                           "{\"status\":\"error\",\"msg\":\"Invalid firmware format. Expected .rps file with 0x900D900D magic at offset 4. Got: 0x%02X%02X%02X%02X\"}",
                           header_buffer[7], header_buffer[6], header_buffer[5], header_buffer[4]);
            response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
            goto send_response;
          }
          
          /* Send RPS header to bootloader
           * Citation: firmware_upgradation.h - sl_si91x_fwup_start()
           * "Send the RPS header content of the firmware file"
           */
          DEBUGOUT("[FW] Sending RPS header to bootloader...\n");
          status = sl_si91x_fwup_start(header_buffer);
          if (status != SL_STATUS_OK) {
            DEBUGOUT("[FW] ERROR: sl_si91x_fwup_start() failed: 0x%lX\n", 
                     (unsigned long)status);
            len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                           "{\"status\":\"error\",\"msg\":\"Failed to start firmware update (0x%lX)\"}",
                           (unsigned long)status);
            response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
            goto send_response;
          }
          
          rps_header_sent = true;
          fw_update_ctx.state = FW_UPDATE_HEADER_RECEIVED;
          DEBUGOUT("[FW] RPS header accepted by bootloader\n");
        }
      }
      
      /* Send remaining firmware content to bootloader */
      if (rps_header_sent && bytes_to_process > 0) {
        /* Citation: firmware_upgradation.h - sl_si91x_fwup_load()
         * "Send the firmware file content"
         * Note: Maximum chunk size is uint16_t (65535 bytes)
         * 
         * Citation: sl_additional_status.h
         * - SL_STATUS_SI91X_FW_UPDATE_DONE (0x1DD03): Firmware update successful
         * - SL_STATUS_SI91X_FW_UPDATE_FAILED (0x1DD04): Firmware update failed
         * The final fwup_load() returns 0x1DD03 when update completes successfully!
         */
        status = sl_si91x_fwup_load(write_ptr, (uint16_t)bytes_to_process);
        if (status == SL_STATUS_SI91X_FW_UPDATE_DONE) {
          /* SUCCESS! Firmware update completed */
          DEBUGOUT("[FW] sl_si91x_fwup_load() returned FW_UPDATE_DONE (0x1DD03) - SUCCESS!\n");
          fw_update_ctx.bytes_received += bytes_to_process;
          fw_update_ctx.state = FW_UPDATE_COMPLETE;
          /* Break out of receive loop - update is complete */
          break;
        } else if (status != SL_STATUS_OK) {
          DEBUGOUT("[FW] ERROR: sl_si91x_fwup_load() failed: 0x%lX\n", 
                   (unsigned long)status);
          
          /* Abort the update
           * Citation: firmware_upgradation.h - sl_si91x_fwup_abort()
           */
          sl_si91x_fwup_abort();
          
          len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                         "{\"status\":\"error\",\"msg\":\"Firmware write failed (0x%lX)\"}",
                         (unsigned long)status);
          response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
          goto send_response;
        }
        
        fw_update_ctx.bytes_received += bytes_to_process;
      }
      
      total_received += bytes_available;
      
      /* Progress logging every 10KB */
      if (total_received % 10240 < bytes_available) {
        DEBUGOUT("[FW] Progress: %lu / %lu bytes (%lu%%)\n",
                 (unsigned long)total_received,
                 (unsigned long)expected_size,
                 (unsigned long)(total_received * 100 / expected_size));
      }
    }
  }

  DEBUGOUT("[FW] Upload complete, total received: %lu bytes\n",
           (unsigned long)total_received);
  DEBUGOUT("[FW] Firmware bytes sent to bootloader: %lu bytes\n",
           (unsigned long)fw_update_ctx.bytes_received);

  /* Validate we received enough data */
  if (!rps_header_sent || fw_update_ctx.bytes_received < fw_update_ctx.header.image_size - RPS_HEADER_SIZE - 100) {
    DEBUGOUT("[FW] ERROR: Incomplete firmware upload\n");
    sl_si91x_fwup_abort();
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Incomplete firmware upload\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  /* Success! Firmware has been written to backup location.
   * Citation: AN1431 Section 3 - "After reboot, the current firmware is 
   * replaced by the new firmware in flash memory"
   */
  fw_update_ctx.state = FW_UPDATE_COMPLETE;
  
  DEBUGOUT("[FW] SiWx917 firmware update SUCCESS!\n");
  DEBUGOUT("[FW] Firmware written to backup location.\n");
  DEBUGOUT("[FW] Device will boot with new firmware after reset.\n");
  
  len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                 "{\"status\":\"ok\",\"msg\":\"Update complete! Rebooting in 2 seconds...\"}");
  response.response_code = SL_HTTP_RESPONSE_OK;

send_response:
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  {
    sl_status_t send_status = sl_http_server_send_response(handle, &response);
    
    /* Auto-reboot ONLY on successful firmware update (not on errors)
     * Citation: AN1431 Section 3 - Device must reboot to apply new firmware
     * Citation: siw917x-family-rm.pdf Section 7.3.5 - NVIC_SystemReset() triggers
     *           SYSRESETREQ in AIRCR register for Cortex-M4 reset
     */
    if (fw_update_ctx.state == FW_UPDATE_COMPLETE) {
      DEBUGOUT("[FW] Firmware update complete - preparing for reboot...\n");
      
      /* Wait for HTTP response to be fully sent to client */
      osDelay(2000);  /* 2 second delay to ensure HTTP response is received by client */
      
      /* CRITICAL: Gracefully shut down WiFi stack BEFORE reset
       * Citation: Silicon Labs WiseConnect SDK - sl_net_deinit() properly
       * shuts down the network interface and allows NWP to complete all
       * pending operations before reset.
       * 
       * Without this, the NWP is in an active state during reset and
       * may not reinitialize properly after POR.
       */
      DEBUGOUT("[FW] Shutting down WiFi AP interface cleanly...\n");
      sl_status_t deinit_status = sl_net_deinit(SL_NET_WIFI_AP_INTERFACE);
      if (deinit_status == SL_STATUS_OK) {
        DEBUGOUT("[FW] WiFi AP interface shut down successfully.\n");
      } else {
        DEBUGOUT("[FW] WARNING: WiFi deinit returned 0x%lx (continuing anyway)\n", deinit_status);
      }
      
      /* Additional delay for NWP to fully complete shutdown sequence */
      DEBUGOUT("[FW] Waiting for NWP to become idle...\n");
      osDelay(1000);
      
      /* CRITICAL: Use Watchdog reset for FULL device reset (M4 + NWP)
       * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
       * "If the processor does not service the interrupt, a watchdog reset is
       *  generated and the ENTIRE DEVICE is reset."
       * 
       * sl_si91x_soc_nvic_reset() only triggers SYSRESETREQ which resets M4 only.
       * After firmware update, NWP is in undefined state -> WiFi hangs on boot.
       */
      DEBUGOUT("[FW] Triggering full device reset via Watchdog Timer!\n");
      trigger_full_device_reset();
      /* Should never reach here */
    }
    
    return send_status;
  }
}

/**
 * @brief Handler for /forceupdate endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - WebUploadForceUpdateHandler()
 * This endpoint is used by the ELRS web UI "Save & Reboot" button.
 * It triggers a reboot after configuration changes.
 */
static sl_status_t handle_forceupdate(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static const char forceupdate_msg[] = "{\"status\":\"ok\",\"msg\":\"Rebooting to apply changes...\"}";

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("**********************************************************\n");
  DEBUGOUT("**** /forceupdate ENDPOINT CALLED! Method: %s ****\n", request_type_str[req->type]);
  DEBUGOUT("**********************************************************\n");

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = &header;
  response.header_count         = 1;
  response.data                 = (uint8_t *)forceupdate_msg;
  response.current_data_length  = sizeof(forceupdate_msg) - 1;
  response.expected_data_length = sizeof(forceupdate_msg) - 1;

  sl_status_t status = sl_http_server_send_response(handle, &response);
  
  /* 
   * Schedule full device reset after response sent
   * 
   * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
   * "If the processor does not service the interrupt, a watchdog reset is
   *  generated and the ENTIRE DEVICE is reset."
   */
  DEBUGOUT("[System] Rebooting via /forceupdate...\n");
  
  /* Small delay to ensure HTTP response is sent before reset */
  osDelay(500);
  
  /* CRITICAL: Gracefully shut down WiFi stack BEFORE reset */
  DEBUGOUT("[System] Shutting down WiFi AP interface cleanly...\n");
  sl_status_t deinit_status = sl_net_deinit(SL_NET_WIFI_AP_INTERFACE);
  if (deinit_status == SL_STATUS_OK) {
    DEBUGOUT("[System] WiFi AP interface shut down successfully.\n");
  } else {
    DEBUGOUT("[System] WARNING: WiFi deinit returned 0x%lx\n", deinit_status);
  }
  
  /* Wait for NWP to complete shutdown */
  osDelay(500);
  
  /* Trigger full device reset via Watchdog Timer */
  trigger_full_device_reset();
  
  /* Should never reach here */
  return status;
}

/**
 * @brief Handler for /options.json endpoint
 */
static sl_status_t handle_options(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static const char options_json[] = "{}";

  request_count++;
  DEBUGOUT("[HTTP] %s /options.json\n", request_type_str[req->type]);

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = &header;
  response.header_count         = 1;
  response.data                 = (uint8_t *)options_json;
  response.current_data_length  = sizeof(options_json) - 1;
  response.expected_data_length = sizeof(options_json) - 1;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for /hardware.json endpoint
 */
static sl_status_t handle_hardware(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static const char hardware_json[] = "{}";

  request_count++;
  DEBUGOUT("[HTTP] %s /hardware.json\n", request_type_str[req->type]);

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = &header;
  response.header_count         = 1;
  response.data                 = (uint8_t *)hardware_json;
  response.current_data_length  = sizeof(hardware_json) - 1;
  response.expected_data_length = sizeof(hardware_json) - 1;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for /sethome endpoint
 * 
 * Citation: ExpressLRS devWIFI.cpp - WebUpdateSetHome()
 * This endpoint sets the Home Network WiFi credentials.
 * 
 * The ELRS web UI sends a POST to this endpoint with form data:
 * network=<ssid>&password=<password>
 */
static sl_status_t handle_sethome(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t header = { .key = "Content-Type", .value = "application/json" };
  static char body_buffer[512];
  
  request_count++;
  DEBUGOUT("[HTTP] %s /sethome\n", request_type_str[req->type]);

  if (req->type == SL_HTTP_REQUEST_POST) {
    /* Read request body */
    sl_http_recv_req_data_t recv_data = {
      .request       = req,
      .buffer        = (uint8_t *)body_buffer,
      .buffer_length = sizeof(body_buffer) - 1
    };

    sl_status_t status = sl_http_server_read_request_data(handle, &recv_data);
    if (status == SL_STATUS_OK && recv_data.received_data_length > 0) {
      body_buffer[recv_data.received_data_length] = '\0';
      DEBUGOUT("[HTTP] SetHome body: %s\n", body_buffer);
      
      /* Parse form data: network=<ssid>&password=<password> */
      char* network_start = strstr(body_buffer, "network=");
      char* password_start = strstr(body_buffer, "password=");
      
      if (network_start != NULL) {
        network_start += 8;
        char ssid[33] = {0};
        char password[65] = {0};
        
        /* Extract SSID (up to & or end) */
        char* ssid_end = strchr(network_start, '&');
        if (ssid_end) {
          size_t len = ssid_end - network_start;
          if (len > 32) len = 32;
          strncpy(ssid, network_start, len);
        }
        
        /* Extract password */
        if (password_start != NULL) {
          password_start += 9;
          char* pwd_end = strchr(password_start, '&');
          size_t len = pwd_end ? (size_t)(pwd_end - password_start) : strlen(password_start);
          if (len > 64) len = 64;
          strncpy(password, password_start, len);
        }
        
        DEBUGOUT("[HTTP] SetHome SSID='%s', Password='%s'\n", ssid, password);
        
        /* Save WiFi credentials */
        elrs_config_set_wifi(ssid, password, 6);  /* Channel 6 default */
        elrs_config_save();
        
        int len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE, 
                          "{\"status\":\"ok\",\"msg\":\"Home Network Set\"}");
        response.response_code = SL_HTTP_RESPONSE_OK;
        response.data = (uint8_t *)response_buffer;
        response.current_data_length = len;
        response.expected_data_length = len;
      }
    }
  }
  
  if (response.data == NULL) {
    static const char ok_msg[] = "{\"status\":\"ok\"}";
    response.response_code = SL_HTTP_RESPONSE_OK;
    response.data = (uint8_t *)ok_msg;
    response.current_data_length = sizeof(ok_msg) - 1;
    response.expected_data_length = sizeof(ok_msg) - 1;
  }
  
  response.content_type = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers = &header;
  response.header_count = 1;
  
  return sl_http_server_send_response(handle, &response);
}

/*******************************************************************************
 * LR1121 Firmware OTA Handlers
 * 
 * Citation: ExpressLRS src/lib/WIFI/lr1121.cpp
 * These endpoints match the ELRS LR1121 firmware update protocol:
 *   GET  /lr1121.json - Returns LR1121 firmware version info
 *   POST /lr1121      - Firmware upload (X-FileSize header, multipart body)
 ******************************************************************************/

/**
 * @brief Handler for GET /lr1121.json endpoint
 * 
 * Citation: ExpressLRS lr1121.cpp - GetLR1121Status()
 * Returns JSON with LR1121 firmware version information:
 * {
 *   "manual": false,
 *   "radio1": { "hardware": 34, "type": 3, "firmware": 258 }
 * }
 */
static sl_status_t handle_lr1121_status(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  (void)req; /* Unused - GET request needs no body/headers */
  
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[4] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = "Cache-Control",             .value = "no-cache" }
  };
  lr1121_firmware_version_t version;
  int len;

  request_count++;
  DEBUGOUT("[HTTP] GET /lr1121.json - LR1121 firmware status\n");

  /* Initialize LR1121 if needed and reset to get clean state */
  lr1121_status_t init_status = lr1121_init();
  if (init_status != LR1121_OK) {
    DEBUGOUT("[HTTP] LR1121 init failed: %d\n", init_status);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"error\":\"LR1121 initialization failed\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Reset LR1121 to ensure clean state */
  lr1121_status_t reset_status = lr1121_reset();
  if (reset_status != LR1121_OK) {
    DEBUGOUT("[HTTP] LR1121 reset failed: %d\n", reset_status);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"error\":\"LR1121 reset failed\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Get firmware version using system GetVersion command (0x0101) */
  if (!lr1121_get_firmware_version(&version, 0x0101)) {
    DEBUGOUT("[HTTP] Failed to read LR1121 version\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"error\":\"Failed to read LR1121 firmware version\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Build JSON response matching ELRS format 
   * Citation: ExpressLRS lr1121.cpp - ReadStatusForRadio()
   */
  len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                 "{"
                 "\"manual\":false,"
                 "\"radio1\":{"
                   "\"hardware\":%u,"
                   "\"type\":%u,"
                   "\"firmware\":%u"
                 "}"
                 "}",
                 (unsigned)version.hardware,
                 (unsigned)version.type,
                 (unsigned)version.version);

  DEBUGOUT("[HTTP] LR1121 status: HW=0x%02X Type=0x%02X FW=0x%04X\n",
           version.hardware, version.type, version.version);
  response.response_code = SL_HTTP_RESPONSE_OK;

send_response:
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for POST /lr1121 endpoint (firmware upload)
 * 
 * Citation: ExpressLRS lr1121.cpp - WebUploadLR1121DataHandler/ResponseHandler
 * 
 * Expected headers:
 *   X-FileSize: <total file size in bytes>
 *   X-Radio: 1 (or 2 for dual radio)
 * 
 * The firmware data is sent as multipart/form-data or raw binary.
 * We accumulate data and flash it to the LR1121.
 */
static sl_status_t handle_lr1121_upload(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[4] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = "Connection",                .value = "close" }
  };
  static uint8_t data_buffer[1024];  /* Receive buffer for firmware chunks */
  uint32_t expected_size = 0;
  int result;
  int len;

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("************************************************************\n");
  DEBUGOUT("**** POST /lr1121 - LR1121 Firmware Upload ****\n");
  DEBUGOUT("************************************************************\n");

  /* Handle OPTIONS preflight for CORS */
  if (req->type != SL_HTTP_REQUEST_POST) {
    return handle_cors_preflight(handle, req);
  }

  /* Get X-FileSize header 
   * Citation: ExpressLRS lr1121.cpp - WebUploadLR1121DataHandler
   * "const uint32_t expectedFilesize = request->header("X-FileSize").toInt();"
   * 
   * Citation: Silicon Labs sl_http_server.h - sl_http_server_get_request_headers()
   * "This function extracts all headers from a given HTTP request"
   */
  sl_http_header_t request_headers[MAX_HTTP_HEADERS];
  sl_status_t hdr_status = sl_http_server_get_request_headers(handle, req, 
                                                               request_headers, 
                                                               MAX_HTTP_HEADERS);
  if (hdr_status != SL_STATUS_OK) {
    DEBUGOUT("[HTTP] Failed to get request headers: 0x%lx\n", (unsigned long)hdr_status);
  } else {
    for (uint16_t i = 0; i < req->request_header_count && i < MAX_HTTP_HEADERS; i++) {
      if (request_headers[i].key != NULL && request_headers[i].value != NULL) {
        DEBUGOUT("[HTTP] Header: %s = %s\n", 
                 request_headers[i].key,
                 request_headers[i].value);
        
        if (str_icmp(request_headers[i].key, "X-FileSize") == 0) {
          expected_size = (uint32_t)atoi(request_headers[i].value);
        }
      }
    }
  }

  if (expected_size == 0) {
    DEBUGOUT("[HTTP] ERROR: X-FileSize header missing or zero\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"X-FileSize header required\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  DEBUGOUT("[HTTP] LR1121 firmware upload starting, expected size: %lu bytes\n",
           (unsigned long)expected_size);

  /* Initialize LR1121 and begin update */
  lr1121_status_t init_status = lr1121_init();
  if (init_status != LR1121_OK) {
    DEBUGOUT("[HTTP] ERROR: LR1121 init failed: %d\n", init_status);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"LR1121 initialization failed\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Begin firmware update (enters bootloader, erases flash) */
  result = lr1121_begin_update(expected_size);
  if (result != 0) {
    DEBUGOUT("[HTTP] ERROR: Failed to begin LR1121 update: %d\n", result);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Failed to enter bootloader mode\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Read and write firmware data in chunks
   * 
   * IMPORTANT: Browser sends multipart/form-data which includes headers BEFORE
   * the actual binary data. The format is:
   *   ------WebKitFormBoundaryXXXXX\r\n
   *   Content-Disposition: form-data; name="file"; filename="lr1121_fw.bin"\r\n
   *   Content-Type: application/octet-stream\r\n
   *   \r\n
   *   <ACTUAL BINARY DATA>
   *   ------WebKitFormBoundaryXXXXX--\r\n
   * 
   * We need to strip this header from the first chunk by finding "\r\n\r\n"
   * which marks the end of headers and start of binary data.
   */
  uint32_t total_received = 0;
  bool header_stripped = false;  /* Track if we've stripped the multipart header */
  uint32_t chunk_number = 0;     /* DIAG: Track chunk number for debugging */
  
  while (total_received < expected_size) {
    sl_http_recv_req_data_t recv_data = {
      .request       = req,
      .buffer        = data_buffer,
      .buffer_length = sizeof(data_buffer)
    };

    sl_status_t status = sl_http_server_read_request_data(handle, &recv_data);
    if (status != SL_STATUS_OK) {
      /* If no more data, we may have received everything or connection closed */
      if (recv_data.received_data_length == 0) {
        break;
      }
    }

    if (recv_data.received_data_length > 0) {
      chunk_number++;
      uint8_t *write_ptr = data_buffer;
      uint32_t bytes_available = recv_data.received_data_length;
      
      /*************************************************************************
       * DIAGNOSTIC: Hex dump of RAW HTTP data BEFORE any processing
       * This shows exactly what the HTTP layer delivered to us.
       *************************************************************************/
      DEBUGOUT("\n=== HTTP CHUNK #%lu RAW DATA (before processing) ===\n", 
               (unsigned long)chunk_number);
      DEBUGOUT("[HTTP DIAG] Chunk size: %lu bytes\n", (unsigned long)bytes_available);
      DEBUGOUT("[HTTP DIAG] Total so far: %lu / %lu bytes\n",
               (unsigned long)total_received, (unsigned long)expected_size);
      
      /* Print first 64 bytes of raw data */
      DEBUGOUT("[HTTP DIAG] RAW bytes[0..63]: ");
      for (uint32_t i = 0; i < 64 && i < bytes_available; i++) {
        DEBUGOUT("%02X ", data_buffer[i]);
        if ((i + 1) % 16 == 0) DEBUGOUT("\n                              ");
      }
      DEBUGOUT("\n");
      
      /* Strip multipart header from the FIRST chunk only */
      if (!header_stripped) {
        /* Look for "\r\n\r\n" which marks end of multipart headers */
        for (uint32_t i = 0; i + 3 < bytes_available; i++) {
          if (data_buffer[i] == '\r' && data_buffer[i+1] == '\n' &&
              data_buffer[i+2] == '\r' && data_buffer[i+3] == '\n') {
            /* Found header end - skip past it */
            uint32_t header_size = i + 4;
            write_ptr = data_buffer + header_size;
            bytes_available -= header_size;
            header_stripped = true;
            
            DEBUGOUT("[HTTP DIAG] Found multipart header end at offset %lu\n",
                     (unsigned long)i);
            DEBUGOUT("[HTTP DIAG] Header size (stripped): %lu bytes\n",
                     (unsigned long)header_size);
            DEBUGOUT("[HTTP DIAG] Remaining firmware data: %lu bytes\n",
                     (unsigned long)bytes_available);
            
            /*****************************************************************
             * DIAGNOSTIC: First 32 bytes of ACTUAL firmware AFTER stripping
             * Compare this with expected firmware signature to detect corruption
             * 
             * Expected LR1121 v1.4 firmware signature (big-endian):
             *   Offset 0: Start of encrypted firmware block
             *   Check your source .bin file with: hexdump -C lr1121_firmware_v1.4.bin | head
             *****************************************************************/
            DEBUGOUT("[HTTP DIAG] First 32 bytes AFTER stripping (ACTUAL FIRMWARE):\n");
            DEBUGOUT("[HTTP DIAG] FW[0..15]:  ");
            for (int j = 0; j < 16 && j < (int)bytes_available; j++) {
              DEBUGOUT("%02X ", write_ptr[j]);
            }
            DEBUGOUT("\n");
            DEBUGOUT("[HTTP DIAG] FW[16..31]: ");
            for (int j = 16; j < 32 && j < (int)bytes_available; j++) {
              DEBUGOUT("%02X ", write_ptr[j]);
            }
            DEBUGOUT("\n");
            
            break;
          }
        }
        
        if (!header_stripped) {
          /* Header spans multiple chunks - this shouldn't happen with 1KB buffer
           * but handle it gracefully by skipping this chunk */
          DEBUGOUT("[HTTP DIAG] WARNING: Multipart header not found in first %lu bytes!\n",
                   (unsigned long)bytes_available);
          DEBUGOUT("[HTTP DIAG] First 128 bytes as ASCII: ");
          for (uint32_t i = 0; i < 128 && i < bytes_available; i++) {
            char c = (char)data_buffer[i];
            DEBUGOUT("%c", (c >= 32 && c < 127) ? c : '.');
          }
          DEBUGOUT("\n");
          continue;
        }
      } else {
        /*************************************************************************
         * DIAGNOSTIC: For subsequent chunks (after first), show first 16 bytes
         * This helps verify data continuity across chunks
         *************************************************************************/
        DEBUGOUT("[HTTP DIAG] Chunk #%lu firmware bytes[0..15]: ",
                 (unsigned long)chunk_number);
        for (int j = 0; j < 16 && j < (int)bytes_available; j++) {
          DEBUGOUT("%02X ", write_ptr[j]);
        }
        DEBUGOUT("\n");
      }
      
      /* Calculate how many bytes to actually write - truncate at expected_size
       * to avoid writing multipart form boundary data appended by HTTP layer */
      uint32_t bytes_to_write = bytes_available;
      if (total_received + bytes_to_write > expected_size) {
        bytes_to_write = expected_size - total_received;
        DEBUGOUT("[HTTP DIAG] Truncating final chunk from %lu to %lu bytes (multipart boundary)\n",
                 (unsigned long)bytes_available,
                 (unsigned long)bytes_to_write);
        
        /* Show what we're truncating - might be boundary marker */
        DEBUGOUT("[HTTP DIAG] Truncated bytes (boundary?): ");
        for (uint32_t i = bytes_to_write; i < bytes_available && i < bytes_to_write + 64; i++) {
          char c = (char)write_ptr[i];
          DEBUGOUT("%c", (c >= 32 && c < 127) ? c : '.');
        }
        DEBUGOUT("\n");
      }
      
      if (bytes_to_write > 0) {
        DEBUGOUT("[HTTP DIAG] Writing %lu bytes to LR1121 driver buffer...\n",
                 (unsigned long)bytes_to_write);
        
        result = lr1121_write_update_bytes(write_ptr, bytes_to_write);
        if (result != 0) {
          DEBUGOUT("[HTTP] ERROR: write failed! %d\n", result);
          len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                         "{\"status\":\"error\",\"msg\":\"Flash write failed\"}");
          response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
          goto send_response;
        }
      }
      total_received += bytes_to_write;
      
      /* Progress logging every 10KB */
      if (total_received % 10240 < bytes_to_write) {
        DEBUGOUT("[HTTP] Progress: %lu / %lu bytes (%lu%%)\n",
                 (unsigned long)total_received,
                 (unsigned long)expected_size,
                 (unsigned long)(total_received * 100 / expected_size));
      }
      
      DEBUGOUT("=== END HTTP CHUNK #%lu ===\n\n", (unsigned long)chunk_number);
    }
  }

  DEBUGOUT("[HTTP] Upload complete, total received: %lu bytes\n",
           (unsigned long)total_received);

  /* Complete the update (flush buffer, reboot to app) */
  result = lr1121_end_update();
  
  if (result == 0) {
    DEBUGOUT("[HTTP] LR1121 firmware update SUCCESS!\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"ok\",\"msg\":\"Update complete. Refresh page to see new version.\"}");
    response.response_code = SL_HTTP_RESPONSE_OK;
  } else if (result == -1) {
    DEBUGOUT("[HTTP] ERROR: Not enough data uploaded\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Not enough data uploaded!\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
  } else {
    DEBUGOUT("[HTTP] ERROR: Update failed, code %d\n", result);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Update failed, refresh and try again.\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
  }

send_response:
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  return sl_http_server_send_response(handle, &response);
}

/*******************************************************************************
 * Simple Binary-Safe Firmware Upload Page (HTML)
 * 
 * This page provides a workaround for HTTP binary data corruption by:
 * 1. Reading the firmware file as ArrayBuffer in JavaScript
 * 2. Encoding to base64 (safe ASCII characters only)
 * 3. Sending via POST to /lr1121b64 endpoint
 * 4. Server decodes base64 back to binary
 * 
 * No 0x0A (LF) bytes in transit = no corruption!
 ******************************************************************************/
static const char upload_html_page[] = 
"<!DOCTYPE html>\n"
"<html><head>\n"
"<title>LR1121 Firmware Upload</title>\n"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<style>\n"
"body{font-family:Arial,sans-serif;max-width:600px;margin:20px auto;padding:10px;background:#1a1a2e;color:#eee}\n"
"h1{color:#4cc9f0;text-align:center}\n"
".box{background:#16213e;padding:20px;border-radius:8px;margin:20px 0}\n"
"input[type=file]{margin:10px 0;padding:10px;width:100%;box-sizing:border-box}\n"
"button{background:#4cc9f0;color:#000;border:none;padding:15px 30px;font-size:16px;cursor:pointer;border-radius:5px;width:100%}\n"
"button:disabled{background:#555;cursor:not-allowed}\n"
"#status{margin-top:15px;padding:10px;border-radius:5px;text-align:center}\n"
".progress{height:20px;background:#333;border-radius:10px;overflow:hidden;margin:10px 0}\n"
".progress-bar{height:100%;background:#4cc9f0;width:0%;transition:width 0.3s}\n"
".info{font-size:12px;color:#888;margin-top:10px}\n"
".success{background:#198754;color:#fff}\n"
".error{background:#dc3545;color:#fff}\n"
".warning{background:#ffc107;color:#000}\n"
"</style></head><body>\n"
"<h1>LR1121 Firmware Upload</h1>\n"
"<div class=\"box\">\n"
"<p>Select LR1121 firmware file (.bin):</p>\n"
"<input type=\"file\" id=\"file\" accept=\".bin\">\n"
"<div class=\"progress\"><div class=\"progress-bar\" id=\"progress\"></div></div>\n"
"<button id=\"upload\" disabled>Upload Firmware</button>\n"
"<div id=\"status\"></div>\n"
"<div class=\"info\">Uses base64 encoding to prevent binary corruption during transfer.</div>\n"
"</div>\n"
"<script>\n"
"const fileInput=document.getElementById('file');\n"
"const uploadBtn=document.getElementById('upload');\n"
"const statusDiv=document.getElementById('status');\n"
"const progressBar=document.getElementById('progress');\n"
"let fileData=null,fileName='';\n"
"\n"
"fileInput.onchange=function(){\n"
"  const f=this.files[0];\n"
"  if(!f){uploadBtn.disabled=true;return;}\n"
"  fileName=f.name;\n"
"  statusDiv.textContent='Reading file...';\n"
"  statusDiv.className='';\n"
"  const reader=new FileReader();\n"
"  reader.onload=function(e){\n"
"    fileData=e.target.result;\n"
"    statusDiv.textContent='Ready: '+fileName+' ('+fileData.byteLength+' bytes)';\n"
"    uploadBtn.disabled=false;\n"
"  };\n"
"  reader.onerror=function(){statusDiv.textContent='Error reading file';statusDiv.className='error';};\n"
"  reader.readAsArrayBuffer(f);\n"
"};\n"
"\n"
"uploadBtn.onclick=async function(){\n"
"  if(!fileData)return;\n"
"  uploadBtn.disabled=true;\n"
"  statusDiv.textContent='Encoding to base64...';\n"
"  statusDiv.className='warning';\n"
"  progressBar.style.width='10%';\n"
"  \n"
"  await new Promise(r=>setTimeout(r,50));\n"
"  \n"
"  const bytes=new Uint8Array(fileData);\n"
"  let b64='';\n"
"  const chunk=32768;\n"
"  for(let i=0;i<bytes.length;i+=chunk){\n"
"    b64+=btoa(String.fromCharCode.apply(null,bytes.slice(i,i+chunk)));\n"
"    progressBar.style.width=(10+40*i/bytes.length)+'%';\n"
"    await new Promise(r=>setTimeout(r,0));\n"
"  }\n"
"  \n"
"  statusDiv.textContent='Uploading ('+b64.length+' base64 chars)...';\n"
"  progressBar.style.width='50%';\n"
"  \n"
"  try{\n"
"    const resp=await fetch('/lr1121b64',{\n"
"      method:'POST',\n"
"      headers:{'Content-Type':'text/plain','X-FileSize':''+fileData.byteLength},\n"
"      body:b64\n"
"    });\n"
"    progressBar.style.width='90%';\n"
"    const result=await resp.json();\n"
"    progressBar.style.width='100%';\n"
"    if(result.status==='ok'){\n"
"      statusDiv.textContent='SUCCESS: '+result.msg;\n"
"      statusDiv.className='success';\n"
"    }else{\n"
"      statusDiv.textContent='ERROR: '+result.msg;\n"
"      statusDiv.className='error';\n"
"    }\n"
"  }catch(e){\n"
"    statusDiv.textContent='Network error: '+e.message;\n"
"    statusDiv.className='error';\n"
"  }\n"
"  uploadBtn.disabled=false;\n"
"};\n"
"</script></body></html>";

/**
 * @brief Handler for /upload.html - Simple firmware upload page
 * 
 * This serves a standalone HTML page that bypasses the ELRS web UI
 * and uses base64 encoding to safely transfer binary firmware data.
 */
static sl_status_t handle_upload_page(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[2] = {
    { .key = "Content-Type",  .value = "text/html; charset=utf-8" },
    { .key = "Cache-Control", .value = "no-cache" }
  };

  (void)req;
  request_count++;
  DEBUGOUT("[HTTP] GET /upload.html - Binary-safe upload page\n");

  response.response_code        = SL_HTTP_RESPONSE_OK;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_HTML;
  response.headers              = headers;
  response.header_count         = 2;
  response.data                 = (uint8_t *)upload_html_page;
  response.current_data_length  = sizeof(upload_html_page) - 1;
  response.expected_data_length = sizeof(upload_html_page) - 1;

  return sl_http_server_send_response(handle, &response);
}

/**
 * @brief Handler for POST /lr1121b64 endpoint (base64-encoded firmware upload)
 * 
 * This endpoint accepts base64-encoded firmware data and decodes it before
 * writing to the LR1121. This bypasses the binary data corruption issue
 * where 0x0A (LF) bytes are stripped or corrupted during HTTP transfer.
 * 
 * Expected headers:
 *   X-FileSize: <expected decoded file size in bytes>
 *   Content-Type: text/plain (base64 data)
 * 
 * Body: Base64-encoded firmware binary
 */
static sl_status_t handle_lr1121_upload_base64(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[4] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = "Connection",                .value = "close" }
  };
  /* Base64 buffer - needs to be larger than binary because base64 expands by 4/3 
   * 1536 bytes base64 -> 1152 bytes binary */
  static uint8_t data_buffer[1536];
  uint32_t expected_size = 0;
  int result;
  int len;

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("************************************************************\n");
  DEBUGOUT("**** POST /lr1121b64 - Base64 Firmware Upload ****\n");
  DEBUGOUT("************************************************************\n");

  /* Handle OPTIONS preflight for CORS */
  if (req->type != SL_HTTP_REQUEST_POST) {
    return handle_cors_preflight(handle, req);
  }

  /* Get X-FileSize header (expected DECODED size) */
  sl_http_header_t request_headers[MAX_HTTP_HEADERS];
  sl_status_t hdr_status = sl_http_server_get_request_headers(handle, req, 
                                                               request_headers, 
                                                               MAX_HTTP_HEADERS);
  if (hdr_status != SL_STATUS_OK) {
    DEBUGOUT("[HTTP] Failed to get request headers: 0x%lx\n", (unsigned long)hdr_status);
  } else {
    for (uint16_t i = 0; i < req->request_header_count && i < MAX_HTTP_HEADERS; i++) {
      if (request_headers[i].key != NULL && request_headers[i].value != NULL) {
        DEBUGOUT("[HTTP] Header: %s = %s\n", 
                 request_headers[i].key,
                 request_headers[i].value);
        
        if (str_icmp(request_headers[i].key, "X-FileSize") == 0) {
          expected_size = (uint32_t)atoi(request_headers[i].value);
        }
      }
    }
  }

  if (expected_size == 0) {
    DEBUGOUT("[HTTP] ERROR: X-FileSize header missing or zero\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"X-FileSize header required (decoded size)\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  DEBUGOUT("[HTTP] Base64 firmware upload starting, expected decoded size: %lu bytes\n",
           (unsigned long)expected_size);

  /* Initialize LR1121 and begin update */
  lr1121_status_t init_status = lr1121_init();
  if (init_status != LR1121_OK) {
    DEBUGOUT("[HTTP] ERROR: LR1121 init failed: %d\n", init_status);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"LR1121 initialization failed\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Begin firmware update (enters bootloader, erases flash) */
  result = lr1121_begin_update(expected_size);
  if (result != 0) {
    DEBUGOUT("[HTTP] ERROR: Failed to begin LR1121 update: %d\n", result);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Failed to enter bootloader mode\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
    goto send_response;
  }

  /* Read and decode base64 data in chunks */
  uint32_t total_decoded = 0;
  uint32_t chunk_number = 0;
  uint8_t leftover_buf[4];  /* Buffer for base64 characters that span chunks */
  uint32_t leftover_len = 0;
  
  while (total_decoded < expected_size) {
    sl_http_recv_req_data_t recv_data = {
      .request       = req,
      .buffer        = data_buffer + leftover_len,
      .buffer_length = sizeof(data_buffer) - leftover_len
    };

    sl_status_t status = sl_http_server_read_request_data(handle, &recv_data);
    if (status != SL_STATUS_OK) {
      if (recv_data.received_data_length == 0) {
        break;
      }
    }

    if (recv_data.received_data_length > 0) {
      chunk_number++;
      uint32_t chunk_len = recv_data.received_data_length + leftover_len;
      
      /* Prepend any leftover bytes from previous chunk */
      if (leftover_len > 0) {
        memmove(data_buffer + leftover_len, data_buffer, recv_data.received_data_length);
        memcpy(data_buffer, leftover_buf, leftover_len);
        leftover_len = 0;
      }
      
      DEBUGOUT("[B64] Chunk #%lu: %lu base64 bytes\n", 
               (unsigned long)chunk_number, (unsigned long)chunk_len);

      /* Base64 must be decoded in multiples of 4 characters
       * Save any remainder for the next chunk */
      uint32_t decode_len = (chunk_len / 4) * 4;
      leftover_len = chunk_len - decode_len;
      
      if (leftover_len > 0) {
        memcpy(leftover_buf, data_buffer + decode_len, leftover_len);
      }
      
      if (decode_len == 0) {
        /* Not enough data yet, continue reading */
        continue;
      }

      /* Decode base64 in-place */
      int32_t decoded_bytes = base64_decode_inplace(data_buffer, decode_len);
      if (decoded_bytes < 0) {
        DEBUGOUT("[HTTP] ERROR: Base64 decode failed!\n");
        len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Base64 decode error\"}");
        response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
        goto send_response;
      }
      
      DEBUGOUT("[B64] Decoded %ld binary bytes\n", (long)decoded_bytes);
      
      /* Diagnostic: Show first 16 bytes of decoded firmware */
      if (chunk_number == 1) {
        DEBUGOUT("[B64] First 16 decoded bytes: ");
        for (int i = 0; i < 16 && i < decoded_bytes; i++) {
          DEBUGOUT("%02X ", data_buffer[i]);
        }
        DEBUGOUT("\n");
      }
      
      /* Write decoded data to LR1121 */
      result = lr1121_write_update_bytes(data_buffer, decoded_bytes);
      if (result != 0) {
        DEBUGOUT("[HTTP] ERROR: write failed! %d\n", result);
        len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Flash write failed\"}");
        response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
        goto send_response;
      }
      
      total_decoded += decoded_bytes;
      
      /* Progress logging every 10KB */
      if (total_decoded % 10240 < (uint32_t)decoded_bytes) {
        DEBUGOUT("[B64] Progress: %lu / %lu bytes (%lu%%)\n",
                 (unsigned long)total_decoded,
                 (unsigned long)expected_size,
                 (unsigned long)(total_decoded * 100 / expected_size));
      }
    }
  }

  /* Handle any remaining base64 characters (should have padding) */
  if (leftover_len > 0) {
    DEBUGOUT("[B64] Processing %lu leftover base64 chars\n", (unsigned long)leftover_len);
    /* This shouldn't normally happen if the base64 was properly padded */
    int32_t decoded_bytes = base64_decode_inplace(leftover_buf, leftover_len);
    if (decoded_bytes > 0) {
      result = lr1121_write_update_bytes(leftover_buf, decoded_bytes);
      if (result != 0) {
        DEBUGOUT("[HTTP] ERROR: final write failed!\n");
        len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Final flash write failed\"}");
        response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
        goto send_response;
      }
      total_decoded += decoded_bytes;
    }
  }

  DEBUGOUT("[HTTP] Base64 upload complete, total decoded: %lu bytes\n",
           (unsigned long)total_decoded);

  /* Complete the update (flush buffer, reboot to app) */
  result = lr1121_end_update();
  
  if (result == 0) {
    DEBUGOUT("[HTTP] LR1121 firmware update SUCCESS!\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"ok\",\"msg\":\"Update complete! Refresh page to verify.\"}");
    response.response_code = SL_HTTP_RESPONSE_OK;
  } else if (result == -1) {
    DEBUGOUT("[HTTP] ERROR: Not enough data uploaded\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Not enough data uploaded!\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
  } else {
    DEBUGOUT("[HTTP] ERROR: Update failed, code %d\n", result);
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Update failed, refresh and try again.\"}");
    response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
  }

send_response:
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  return sl_http_server_send_response(handle, &response);
}


/**
 * @brief Handler for POST /updateb64 endpoint (base64-encoded MCU firmware upload)
 * 
 * Citation: AN1431 - SiWx917 SoC Firmware Update Application Note
 * 
 * This endpoint accepts base64-encoded firmware data and decodes it before
 * passing to the SiWx917 firmware update APIs. This bypasses the binary data
 * corruption issue where 0x0A (LF) bytes are stripped during HTTP transfer.
 * 
 * Expected headers:
 *   X-FileSize: <expected decoded file size in bytes>
 *   Content-Type: text/plain (base64 data)
 * 
 * Body: Base64-encoded firmware binary in RPS format
 */
static sl_status_t handle_update_base64(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  sl_http_header_t headers[4] = {
    { .key = "Content-Type",              .value = "application/json" },
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = "Connection",                .value = "close" }
  };
  /* Base64 buffer - 1536 bytes base64 -> 1152 bytes binary */
  static uint8_t data_buffer[1536];
  uint32_t expected_size = 0;
  sl_status_t status;
  int len;

  request_count++;
  DEBUGOUT("\n");
  DEBUGOUT("************************************************************\n");
  DEBUGOUT("**** POST /updateb64 - Base64 MCU Firmware Upload ****\n");
  DEBUGOUT("************************************************************\n");

  /* Handle OPTIONS preflight for CORS */
  if (req->type != SL_HTTP_REQUEST_POST) {
    return handle_cors_preflight(handle, req);
  }

  /* Get X-FileSize header (expected DECODED size) */
  sl_http_header_t request_headers[MAX_HTTP_HEADERS];
  sl_status_t hdr_status = sl_http_server_get_request_headers(handle, req, 
                                                               request_headers, 
                                                               MAX_HTTP_HEADERS);
  if (hdr_status == SL_STATUS_OK) {
    for (uint16_t i = 0; i < req->request_header_count && i < MAX_HTTP_HEADERS; i++) {
      if (request_headers[i].key != NULL && request_headers[i].value != NULL) {
        DEBUGOUT("[FW] Header: %s = %s\n", 
                 request_headers[i].key,
                 request_headers[i].value);
        
        if (str_icmp(request_headers[i].key, "X-FileSize") == 0) {
          expected_size = (uint32_t)atoi(request_headers[i].value);
        }
      }
    }
  }

  if (expected_size == 0) {
    DEBUGOUT("[FW] ERROR: X-FileSize header missing or zero\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"X-FileSize header required (decoded size)\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  if (expected_size < RPS_HEADER_SIZE + 256) {
    DEBUGOUT("[FW] ERROR: File too small for valid firmware\n");
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"File too small for valid firmware\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  DEBUGOUT("[FW] Base64 MCU firmware upload, expected decoded size: %lu bytes\n",
           (unsigned long)expected_size);

  /* Initialize update context */
  memset(&fw_update_ctx, 0, sizeof(fw_update_ctx));
  fw_update_ctx.expected_size = expected_size;
  fw_update_ctx.header_stripped = true;  /* No multipart header in base64 mode */

  /* Read, decode, and process base64 data in chunks */
  uint32_t total_decoded = 0;
  uint32_t chunk_number = 0;
  uint8_t leftover_buf[4];
  uint32_t leftover_len = 0;
  bool rps_header_sent = false;
  uint8_t header_buffer[RPS_HEADER_SIZE];
  uint32_t header_bytes_collected = 0;
  
  while (total_decoded < expected_size) {
    sl_http_recv_req_data_t recv_data = {
      .request       = req,
      .buffer        = data_buffer + leftover_len,
      .buffer_length = sizeof(data_buffer) - leftover_len
    };

    status = sl_http_server_read_request_data(handle, &recv_data);
    if (status != SL_STATUS_OK && recv_data.received_data_length == 0) {
      break;
    }

    if (recv_data.received_data_length > 0) {
      chunk_number++;
      uint32_t chunk_len = recv_data.received_data_length + leftover_len;
      
      /* Prepend any leftover bytes from previous chunk */
      if (leftover_len > 0) {
        memmove(data_buffer + leftover_len, data_buffer, recv_data.received_data_length);
        memcpy(data_buffer, leftover_buf, leftover_len);
        leftover_len = 0;
      }

      /* Base64 must be decoded in multiples of 4 characters */
      uint32_t decode_len = (chunk_len / 4) * 4;
      leftover_len = chunk_len - decode_len;
      
      if (leftover_len > 0) {
        memcpy(leftover_buf, data_buffer + decode_len, leftover_len);
      }
      
      if (decode_len == 0) continue;

      /* Decode base64 in-place */
      int32_t decoded_bytes = base64_decode_inplace(data_buffer, decode_len);
      if (decoded_bytes < 0) {
        DEBUGOUT("[FW] ERROR: Base64 decode failed!\n");
        len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                       "{\"status\":\"error\",\"msg\":\"Base64 decode error\"}");
        response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
        goto send_response;
      }

      uint8_t *write_ptr = data_buffer;
      uint32_t bytes_to_process = (uint32_t)decoded_bytes;

      /* Collect RPS header (first 64 bytes) */
      if (!rps_header_sent) {
        uint32_t header_bytes_needed = RPS_HEADER_SIZE - header_bytes_collected;
        uint32_t bytes_for_header = (bytes_to_process < header_bytes_needed) 
                                     ? bytes_to_process : header_bytes_needed;
        
        memcpy(header_buffer + header_bytes_collected, write_ptr, bytes_for_header);
        header_bytes_collected += bytes_for_header;
        write_ptr += bytes_for_header;
        bytes_to_process -= bytes_for_header;
        
        if (header_bytes_collected == RPS_HEADER_SIZE) {
          memcpy(&fw_update_ctx.header, header_buffer, RPS_HEADER_SIZE);
          
          int result = validate_rps_header(&fw_update_ctx.header);
          if (result != 0) {
            len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                           "{\"status\":\"error\",\"msg\":\"Invalid firmware format (bad RPS header)\"}");
            response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
            goto send_response;
          }
          
          DEBUGOUT("[FW] Sending RPS header to bootloader...\n");
          status = sl_si91x_fwup_start(header_buffer);
          if (status != SL_STATUS_OK) {
            DEBUGOUT("[FW] ERROR: sl_si91x_fwup_start() failed: 0x%lX\n", 
                     (unsigned long)status);
            len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                           "{\"status\":\"error\",\"msg\":\"Failed to start firmware update\"}");
            response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
            goto send_response;
          }
          
          rps_header_sent = true;
          DEBUGOUT("[FW] RPS header accepted by bootloader\n");
        }
      }
      
      /* Send remaining firmware content to bootloader 
       * Citation: sl_additional_status.h
       * - SL_STATUS_SI91X_FW_UPDATE_DONE (0x1DD03): Firmware update successful
       * - SL_STATUS_SI91X_FW_UPDATE_FAILED (0x1DD04): Firmware update failed
       */
      if (rps_header_sent && bytes_to_process > 0) {
        status = sl_si91x_fwup_load(write_ptr, (uint16_t)bytes_to_process);
        if (status == SL_STATUS_SI91X_FW_UPDATE_DONE) {
          /* SUCCESS! Firmware update completed */
          DEBUGOUT("[FW] sl_si91x_fwup_load() returned FW_UPDATE_DONE (0x1DD03) - SUCCESS!\n");
          fw_update_ctx.bytes_received += bytes_to_process;
          fw_update_ctx.state = FW_UPDATE_COMPLETE;
          /* Break out of receive loop - update is complete */
          break;
        } else if (status != SL_STATUS_OK) {
          DEBUGOUT("[FW] ERROR: sl_si91x_fwup_load() failed: 0x%lX\n", 
                   (unsigned long)status);
          sl_si91x_fwup_abort();
          len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                         "{\"status\":\"error\",\"msg\":\"Firmware write failed\"}");
          response.response_code = SL_HTTP_RESPONSE_INTERNAL_SERVER_ERROR;
          goto send_response;
        }
        fw_update_ctx.bytes_received += bytes_to_process;
      }
      
      total_decoded += decoded_bytes;
      
      /* Progress logging every 10KB */
      if (total_decoded % 10240 < (uint32_t)decoded_bytes) {
        DEBUGOUT("[FW] Progress: %lu / %lu bytes (%lu%%)\n",
                 (unsigned long)total_decoded,
                 (unsigned long)expected_size,
                 (unsigned long)(total_decoded * 100 / expected_size));
      }
    }
  }

  DEBUGOUT("[FW] Base64 upload complete, total decoded: %lu bytes\n",
           (unsigned long)total_decoded);

  /* Validate completion */
  if (!rps_header_sent || fw_update_ctx.bytes_received < fw_update_ctx.header.image_size - RPS_HEADER_SIZE - 100) {
    sl_si91x_fwup_abort();
    len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                   "{\"status\":\"error\",\"msg\":\"Incomplete firmware upload\"}");
    response.response_code = SL_HTTP_RESPONSE_BAD_REQUEST;
    goto send_response;
  }

  fw_update_ctx.state = FW_UPDATE_COMPLETE;
  DEBUGOUT("[FW] SiWx917 firmware update SUCCESS!\n");
  
  len = snprintf(response_buffer, RESPONSE_BUFFER_SIZE,
                 "{\"status\":\"ok\",\"msg\":\"Update complete! Rebooting in 2 seconds...\"}");
  response.response_code = SL_HTTP_RESPONSE_OK;

send_response:
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;
  response.data                 = (uint8_t *)response_buffer;
  response.current_data_length  = len;
  response.expected_data_length = len;

  {
    sl_status_t send_status = sl_http_server_send_response(handle, &response);
    
    /* Auto-reboot ONLY on successful firmware update (not on errors)
     * Citation: AN1431 Section 3 - Device must reboot to apply new firmware
     * Citation: siw917x-family-rm.pdf Section 7.3.4 - Watchdog Reset
     * "a watchdog reset is generated and the ENTIRE DEVICE is reset"
     * 
     * CRITICAL FIX: After firmware update, we must allow the NWP to fully
     * finalize writing to flash before triggering reset. The NWP bootloader
     * may still be flushing buffers even after FW_UPDATE_DONE is returned.
     * 
     * Additionally, we need to gracefully shut down the WiFi stack BEFORE
     * reset to ensure NWP is in a clean state for the next boot.
     */
    if (fw_update_ctx.state == FW_UPDATE_COMPLETE) {
      DEBUGOUT("[FW] Firmware update complete!\n");
      DEBUGOUT("[FW] Waiting 3 seconds for NWP to finalize flash write...\n");
      osDelay(3000);
      
      /* Gracefully shut down WiFi to put NWP in clean state
       * Citation: Silicon Labs WiseConnect SDK - sl_net_deinit()
       * This ensures NWP completes all pending operations before reset.
       */
      DEBUGOUT("[FW] Deinitializing WiFi stack for clean shutdown...\n");
      sl_status_t deinit_status = sl_net_deinit(SL_NET_WIFI_AP_INTERFACE);
      if (deinit_status != SL_STATUS_OK) {
        DEBUGOUT("[FW] WARNING: sl_net_deinit() returned 0x%lX (continuing anyway)\n", 
                 (unsigned long)deinit_status);
      } else {
        DEBUGOUT("[FW] WiFi stack deinitialized successfully.\n");
      }
      
      /* Additional delay after deinit to ensure NWP is fully idle */
      DEBUGOUT("[FW] Waiting 1 second for NWP to become idle...\n");
      osDelay(1000);
      
      DEBUGOUT("[FW] Triggering full device reset via Watchdog Timer!\n");
      trigger_full_device_reset();
    }
    
    return send_status;
  }
}

/**
 * @brief Default handler - try to serve as web asset, else 404
 * 
 * NOTE: This handler catches ALL requests not matched by registered handlers.
 * We log every request here to help debug which endpoint the Web UI calls.
 */
static sl_status_t handle_not_found(sl_http_server_t *handle, sl_http_server_request_t *req)
{
  sl_http_server_response_t response = { 0 };
  static const char not_found_msg[] = "404 Not Found";
  /* Include CORS headers in all responses including 404s */
  sl_http_header_t headers[4] = {
    { .key = CORS_HEADER_ALLOW_ORIGIN,    .value = CORS_VALUE_ALLOW_ORIGIN },
    { .key = CORS_HEADER_ALLOW_METHODS,   .value = CORS_VALUE_ALLOW_METHODS },
    { .key = CORS_HEADER_ALLOW_HEADERS,   .value = CORS_VALUE_ALLOW_HEADERS },
    { .key = "Cache-Control",             .value = "no-cache" }
  };

  request_count++;
  
  /* Log EVERY unhandled request - this helps debug missing endpoints */
  DEBUGOUT("[HTTP] UNHANDLED: %s %s\n", 
           (req->type < 5) ? request_type_str[req->type] : "UNKNOWN", 
           req->uri.path);
  
  /* Try to serve as a web asset first (for asset paths) */
  sl_status_t status = serve_web_asset(handle, req->uri.path);
  if (status == SL_STATUS_OK) {
    DEBUGOUT("[HTTP] -> HANDLED as web asset\n");
    return status;
  }

  DEBUGOUT("[HTTP] -> 404 Not Found\n");

  response.response_code        = SL_HTTP_RESPONSE_NOT_FOUND;
  response.content_type         = SL_HTTP_CONTENT_TYPE_TEXT_PLAIN;
  response.headers              = headers;
  response.header_count         = 4;  /* Include CORS headers */
  response.data                 = (uint8_t *)not_found_msg;
  response.current_data_length  = sizeof(not_found_msg) - 1;
  response.expected_data_length = sizeof(not_found_msg) - 1;

  return sl_http_server_send_response(handle, &response);
}

/*******************************************************************************
 * HTTP Handler Registration Table
 * 
 * Citation: ExpressLRS devWIFI.cpp handler registrations
 ******************************************************************************/
static sl_http_server_handler_t http_handlers[] = {
  /* Main page */
  { .uri = "/",              .handler = handle_root },
  { .uri = "/index.html",    .handler = handle_index },
  
  /* ELRS API endpoints */
  { .uri = "/config",        .handler = handle_config },
  { .uri = "/networks",      .handler = handle_networks },
  { .uri = "/reboot",        .handler = handle_reboot },
  { .uri = "/forceupdate",   .handler = handle_forceupdate },
  { .uri = "/update",        .handler = handle_update },
  { .uri = "/options.json",  .handler = handle_options },
  { .uri = "/hardware.json", .handler = handle_hardware },
  { .uri = "/sethome",       .handler = handle_sethome },
  
  /* LR1121 Firmware OTA endpoints 
   * Citation: ExpressLRS lr1121.cpp - addLR1121Handlers()
   */
  { .uri = "/lr1121.json",   .handler = handle_lr1121_status },
  { .uri = "/lr1121",        .handler = handle_lr1121_upload },
  
  /* Binary-safe upload endpoints (workaround for HTTP data corruption)
   * These use base64 encoding to avoid 0x0A (LF) corruption issue.
   */
  { .uri = "/upload.html",    .handler = handle_upload_page },      /* LR1121 firmware (.bin) */

  { .uri = "/lr1121b64",      .handler = handle_lr1121_upload_base64 },
  { .uri = "/updateb64",      .handler = handle_update_base64 },
};

#define HTTP_HANDLER_COUNT (sizeof(http_handlers) / sizeof(http_handlers[0]))

/*******************************************************************************
 * WiFi Event Callbacks
 ******************************************************************************/

static sl_status_t ap_client_connected(sl_wifi_event_t event, void *data,
                                       uint32_t data_length, void *arg)
{
  (void)event;
  (void)data_length;
  (void)arg;

  client_count++;
  DEBUGOUT("[WiFi] Client connected! (Total: %lu) MAC: ", client_count);
  print_mac_address((sl_mac_address_t *)data);
  DEBUGOUT("\n");

  return SL_STATUS_OK;
}

static sl_status_t ap_client_disconnected(sl_wifi_event_t event, void *data,
                                          uint32_t data_length, void *arg)
{
  (void)event;
  (void)data_length;
  (void)arg;

  if (client_count > 0) {
    client_count--;
  }
  DEBUGOUT("[WiFi] Client disconnected. (Remaining: %lu) MAC: ", client_count);
  print_mac_address((sl_mac_address_t *)data);
  DEBUGOUT("\n");

  return SL_STATUS_OK;
}

/*******************************************************************************
 * Boot Diagnostics - Reset Cause and NWP State Detection
 * 
 * Citation: siw917x-family-rm.pdf Section 9.18.11 - MCU_FSM_WAKEUP_STATUS_REG
 * Base: 0x24048100, Offset: 0x038 = 0x24048138
 * 
 * Citation: siw917x-family-rm.pdf Section 24.5.51 - MCR_MCU_P2P_COMM_STATUS_REG
 * Base: 0x46008000, Offset: 0x174 = 0x46008174
 * Shows NWP_ACTIVE_STATUS (bit 3) to determine if NWP is awake
 ******************************************************************************/

/* Sleep FSM Register Base */
#define SLEEP_FSM_BASE                  0x24048100UL

/* MCU_FSM_WAKEUP_STATUS_REG - Offset 0x038 from Sleep FSM Base
 * Citation: siw917x-family-rm.pdf Section 9.18.11
 */
#define MCU_FSM_WAKEUP_STATUS_REG       (*(volatile uint32_t *)(SLEEP_FSM_BASE + 0x038))

/* Wakeup status bit definitions
 * Citation: siw917x-family-rm.pdf Section 9.18.11 WAKEUP_STATUS field
 */
#define WAKEUP_BIT_HOST_RESET           (1U << 10)  /* Host reset request */
#define WAKEUP_BIT_NWP_WDT_WINDOW       (1U << 9)   /* NWP watchdog window reset */
#define WAKEUP_BIT_NWP_WDT              (1U << 8)   /* NWP watchdog reset */
#define WAKEUP_BIT_MCU_WDT_WINDOW       (1U << 6)   /* MCU watchdog window reset */
#define WAKEUP_BIT_MCU_WDT              (1U << 5)   /* MCU watchdog reset */
#define WAKEUP_BIT_MCU_PROC_WAKE        (1U << 4)   /* MCU processor wake status */
#define WAKEUP_BIT_DEBUG_POWERUP        (1U << 3)   /* Core debug power-up request */
#define WAKEUP_BIT_HOST_WAKEUP          (1U << 2)   /* Host based wake-up */
#define WAKEUP_BIT_TIMEOUT              (1U << 1)   /* Timeout wake-up */
#define WAKEUP_BIT_WAKEUP_IND           (1U << 0)   /* Wake-up indication */
#define WAKEUP_BIT_FIRST_POR            (1U << 16)  /* MCU_FIRST_POWERUP_POR */
#define WAKEUP_BIT_FIRST_RESET          (1U << 17)  /* MCU_FIRST_POWERUP_RESET_N */

/* MCU Configuration Register Base */
#define MCU_CONFIG_BASE                 0x46008000UL

/* MCR_MCU_P2P_COMM_STATUS_REG - Offset 0x174
 * Citation: siw917x-family-rm.pdf Section 24.5.51
 */
#define MCU_P2P_COMM_STATUS_REG         (*(volatile uint32_t *)(MCU_CONFIG_BASE + 0x174))

/* P2P Communication Status bits */
#define P2P_NWP_ACTIVE_STATUS           (1U << 3)   /* NWP is active */
#define P2P_NWP_WAKEUP_MCU              (1U << 2)   /* NWP wakes up MCU */
#define P2P_MCU_ACTIVE_STATUS           (1U << 1)   /* MCU is active */
#define P2P_MCU_WAKEUP_NWP              (1U << 0)   /* MCU wakes up NWP */

/* HOST_INTF_REG_OUT - Used for NWP boot status detection
 * Citation: siw917x-family-rm.pdf Section 15.5.14 (SPI interface registers)
 * Address: 0x4105003C
 * Returns 0xABxx when NWP is ready (0xAB = WIFI_REGISTER_VALID)
 */
#define HOST_INTF_REG_OUT               (*(volatile uint32_t *)0x4105003CUL)
#define WIFI_REGISTER_VALID             0xAB

/**
 * @brief Print boot diagnostics - reset cause and NWP state
 * 
 * This function reads hardware status registers to determine:
 * 1. Why the device reset (POR, watchdog, host reset, etc.)
 * 2. Current state of the NWP (Network Wireless Processor)
 * 
 * CRITICAL for debugging WiFi hang after OTA update.
 * 
 * @return The MCU_FSM_WAKEUP_STATUS_REG value (used for WDT workaround)
 */
static uint32_t print_boot_diagnostics(void)
{
  uint32_t wakeup_status = MCU_FSM_WAKEUP_STATUS_REG;
  uint32_t p2p_status = MCU_P2P_COMM_STATUS_REG;
  
  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║               BOOT DIAGNOSTICS                               ║\n");
  DEBUGOUT("╠══════════════════════════════════════════════════════════════╣\n");
  DEBUGOUT("║ Citation: siw917x-family-rm.pdf Section 9.18.11 & 24.5.51    ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");
  
  /* Raw register values */
  DEBUGOUT("[Diag] MCU_FSM_WAKEUP_STATUS_REG = 0x%08lX\n", (unsigned long)wakeup_status);
  DEBUGOUT("[Diag] MCU_P2P_COMM_STATUS_REG   = 0x%08lX\n", (unsigned long)p2p_status);
  DEBUGOUT("\n");
  
  /* Decode reset cause */
  DEBUGOUT("[Diag] === RESET CAUSE ANALYSIS ===\n");
  
  if (wakeup_status & WAKEUP_BIT_FIRST_POR) {
    DEBUGOUT("[Diag] ★ MCU_FIRST_POWERUP_POR: System came out of Power-On Reset\n");
  }
  if (wakeup_status & WAKEUP_BIT_FIRST_RESET) {
    DEBUGOUT("[Diag] ★ MCU_FIRST_POWERUP_RESET_N: System came out of Reset\n");
  }
  if (wakeup_status & WAKEUP_BIT_MCU_WDT) {
    DEBUGOUT("[Diag] ★ MCU_WDT_RESET: MCU Watchdog triggered reset (bit 5)\n");
    DEBUGOUT("[Diag]   -> This is the OTA reboot path via trigger_full_device_reset()\n");
  }
  if (wakeup_status & WAKEUP_BIT_MCU_WDT_WINDOW) {
    DEBUGOUT("[Diag] ★ MCU_WDT_WINDOW_RESET: MCU Watchdog window reset (bit 6)\n");
  }
  if (wakeup_status & WAKEUP_BIT_NWP_WDT) {
    DEBUGOUT("[Diag] ★ NWP_WDT_RESET: NWP Watchdog triggered reset (bit 8)\n");
  }
  if (wakeup_status & WAKEUP_BIT_NWP_WDT_WINDOW) {
    DEBUGOUT("[Diag] ★ NWP_WDT_WINDOW_RESET: NWP Watchdog window reset (bit 9)\n");
  }
  if (wakeup_status & WAKEUP_BIT_HOST_RESET) {
    DEBUGOUT("[Diag] ★ HOST_RESET: Host reset request (bit 10)\n");
  }
  if (wakeup_status & WAKEUP_BIT_HOST_WAKEUP) {
    DEBUGOUT("[Diag]   HOST_WAKEUP: Host based wake-up (bit 2)\n");
  }
  if (wakeup_status & WAKEUP_BIT_TIMEOUT) {
    DEBUGOUT("[Diag]   TIMEOUT_WAKEUP: Timeout wake-up (bit 1)\n");
  }
  if (wakeup_status & WAKEUP_BIT_DEBUG_POWERUP) {
    DEBUGOUT("[Diag]   DEBUG_POWERUP: Core debug power-up request (bit 3)\n");
  }
  
  /* Check if no reset cause bits are set (cold boot) */
  if ((wakeup_status & 0x7FF) == 0 && !(wakeup_status & (WAKEUP_BIT_FIRST_POR | WAKEUP_BIT_FIRST_RESET))) {
    DEBUGOUT("[Diag]   No reset cause bits set - possible cold boot or cleared status\n");
  }
  
  DEBUGOUT("\n");
  
  /* Decode NWP state */
  DEBUGOUT("[Diag] === NWP (Network Wireless Processor) STATE ===\n");
  
  if (p2p_status & P2P_NWP_ACTIVE_STATUS) {
    DEBUGOUT("[Diag] ✓ NWP_ACTIVE_STATUS = 1: NWP is ACTIVE\n");
  } else {
    DEBUGOUT("[Diag] ✗ NWP_ACTIVE_STATUS = 0: NWP is SLEEPING/NOT READY\n");
    DEBUGOUT("[Diag]   -> WiFi init may hang waiting for NWP!\n");
  }
  
  if (p2p_status & P2P_NWP_WAKEUP_MCU) {
    DEBUGOUT("[Diag]   NWP_WAKEUP_MCU = 1: NWP wants to wake up MCU\n");
  }
  
  if (p2p_status & P2P_MCU_ACTIVE_STATUS) {
    DEBUGOUT("[Diag]   MCU_ACTIVE_STATUS = 1: MCU marked as active\n");
  } else {
    DEBUGOUT("[Diag]   MCU_ACTIVE_STATUS = 0: MCU not marked as active\n");
  }
  
  if (p2p_status & P2P_MCU_WAKEUP_NWP) {
    DEBUGOUT("[Diag]   MCU_WAKEUP_NWP = 1: MCU is waking up NWP\n");
  }
  
  DEBUGOUT("\n");
  
  /* Summary and recommendations */
  DEBUGOUT("[Diag] === BOOT PATH SUMMARY ===\n");
  
  if (wakeup_status & WAKEUP_BIT_MCU_WDT) {
    /* This is the OTA reboot path */
    DEBUGOUT("[Diag] Boot reason: MCU Watchdog Reset (OTA reboot path)\n");
    
    if (wakeup_status & WAKEUP_BIT_FIRST_POR) {
      DEBUGOUT("[Diag] ✓ POR bit is set - full device reset occurred (M4 + NWP)\n");
    } else {
      DEBUGOUT("[Diag] ✗ POR bit NOT set - possible partial reset issue!\n");
      DEBUGOUT("[Diag]   -> NWP may not have reset properly\n");
    }
    
    if (!(p2p_status & P2P_NWP_ACTIVE_STATUS)) {
      DEBUGOUT("[Diag] ⚠ WARNING: NWP not active after WDT reset!\n");
      DEBUGOUT("[Diag]   -> sl_net_init() will likely hang waiting for NWP boot\n");
      DEBUGOUT("[Diag]   -> Adding extra delay to allow NWP boot...\n");
      
      /* Add extra delay to allow NWP to boot */
      DEBUGOUT("[Diag]   Waiting 500ms for NWP to initialize...\n");
      osDelay(500);
      
      /* Re-read status */
      p2p_status = MCU_P2P_COMM_STATUS_REG;
      DEBUGOUT("[Diag]   After delay: P2P_STATUS = 0x%08lX\n", (unsigned long)p2p_status);
      if (p2p_status & P2P_NWP_ACTIVE_STATUS) {
        DEBUGOUT("[Diag]   ✓ NWP is now active!\n");
      } else {
        DEBUGOUT("[Diag]   ✗ NWP still not active - WiFi init will likely fail\n");
      }
    }
  } else if (wakeup_status & WAKEUP_BIT_FIRST_POR) {
    DEBUGOUT("[Diag] Boot reason: Power-On Reset (cold boot)\n");
  } else if (wakeup_status & WAKEUP_BIT_FIRST_RESET) {
    DEBUGOUT("[Diag] Boot reason: System Reset (debugger or external reset)\n");
  } else {
    DEBUGOUT("[Diag] Boot reason: Unknown (no status bits set)\n");
  }
  
  DEBUGOUT("\n");
  DEBUGOUT("[Diag] Proceeding with WiFi initialization...\n");
  DEBUGOUT("════════════════════════════════════════════════════════════════\n");
  DEBUGOUT("\n");
  
  return wakeup_status;
}

/*******************************************************************************
 * NWP Reset Workaround for WDT Reset Recovery - ENHANCED VERSION
 * 
 * PROBLEM: After WDT-triggered reset, the NWP (Network Wireless Processor)
 * reports itself as "active" (P2P_STATUS_REG bit 3 = 1), but it's not actually
 * ready to respond to commands. This causes sl_net_init() to hang forever in
 * rsi_waitfor_boardready() waiting for HOST_INTF_REG_OUT to return 0xABxx.
 * 
 * ROOT CAUSE:
 * ===========
 * On the SiWG917, a physical reset button press (Hardware Reset) fully cycles 
 * the power domains and resets the registers of both the M4 (Application) and 
 * NWP (Wireless) processors simultaneously.
 * 
 * However, when triggering a "Full Device Reset" via the Watchdog Timer:
 *   - The NWP often retains certain state information or doesn't clear its 
 *     "active" flag fast enough for the M4.
 *   - The M4 boots up, sees the NWP is "already active," and skips the 
 *     necessary loading sequence, leading to the hang at sl_net_init().
 * 
 * SOLUTION: Force NWP De-assertion during Boot
 * ============================================
 * We need to harden the NWP-Fix logic. Instead of just clearing the bit and 
 * waiting, we explicitly force the NWP into a clean state during the early 
 * boot phase BEFORE the Wi-Fi stack initializes.
 * 
 * Citation: siw917x-family-rm.pdf Section 24.5.51 - MCR_MCU_P2P_COMM_STATUS_REG
 *   Bit 3 (NWP_ACTIVE_STATUS): "Status bit to indicate the NWP active state"
 *   Bit 0 (MCU_WAKEUP_NWP): "This bit is used to wakeup the NWP from sleep"
 * 
 * Citation: siw917x-family-rm.pdf Section 9.10.13 - M4SS_TASS_CTRL_SET_REG
 *   Bit 2 (M4SS_CTRL_TASS_AON_PWR_DMN_RST_BYPASS): NWP Reset Pin Bypass
 *   Bit 1 (M4SS_CTRL_TASS_AON_DISABLE_ISOLATION_BYPASS): NWP Isolation Bypass
 *   Bit 0 (M4SS_CTRL_TASS_AON_PWRGATE_EN): NWP Power On control
 * 
 * Citation: siw917x-family-rm.pdf Section 9.10.14 - M4SS_TASS_CTRL_CLEAR_REG
 *   Same bits but for CLEARING (disabling NWP power/isolation)
 ******************************************************************************/

/* M4SS Power Control registers for NWP domain control
 * Citation: siw917x-family-rm.pdf Section 9.10.13/9.10.14
 * Base: 0x24048000 (M4SS Power Control)
 */
#define M4SS_PWRCTRL_BASE               0x24048000UL
#define M4SS_TASS_CTRL_SET_REG          (*(volatile uint32_t *)(M4SS_PWRCTRL_BASE + 0x034))
#define M4SS_TASS_CTRL_CLEAR_REG        (*(volatile uint32_t *)(M4SS_PWRCTRL_BASE + 0x038))

/* M4SS_TASS_CTRL bit definitions */
#define TASS_AON_PWR_DMN_RST_BYPASS     (1U << 2)  /* NWP Reset Pin Bypass control */
#define TASS_AON_DISABLE_ISO_BYPASS     (1U << 1)  /* NWP Isolation Bypass control */
#define TASS_AON_PWRGATE_EN             (1U << 0)  /* NWP Power Gate enable */

/* M4SS_P2P_INTR registers for sleep/wakeup indication
 * Citation: siw917x-family-rm.pdf Section 24 - MCU Configuration
 * NOTE: M4SS_P2P_INTR_SET_REG and M4SS_P2P_INTR_CLR_REG are already defined 
 *       in the SDK (system_si91x.h), so we use those directly.
 */
#define M4_WAKEUP_TA_BIT                (1U << 0)  /* Sleep/wakeup indication bit */

/**
 * @brief Force NWP (Network Wireless Processor) into clean state after WDT reset
 * 
 * ENHANCED FIX: Force-cycle the NWP power/reset domain
 * ====================================================
 * This function implements a more aggressive NWP reset sequence that mimics
 * what a hardware reset does. Instead of just waiting for NWP to become ready,
 * we:
 *   1. Force clear TA_is_active to prevent SDK from entering the hanging loop
 *   2. Pulse the M4_wakeup_TA bit to ensure NWP state machine moves
 *   3. Wait for NWP to fully initialize and respond
 * 
 * @param wakeup_status The MCU_FSM_WAKEUP_STATUS_REG value from boot diagnostics
 * @return true if workaround was applied, false otherwise
 */
static bool force_nwp_reinit_after_wdt(uint32_t wakeup_status)
{
  uint32_t p2p_status;
  uint32_t timeout_count;
  const uint32_t MAX_WAIT_MS = 3000;  /* Max 3 seconds for NWP boot */
  const uint32_t POLL_INTERVAL_MS = 50;
  
  /* Only apply workaround after WDT reset */
  if (!(wakeup_status & WAKEUP_BIT_MCU_WDT)) {
    DEBUGOUT("[NWP-Fix] Not a WDT reset, skipping workaround\n");
    return false;
  }
  
  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║      NWP FULL HARDWARE RESET WORKAROUND (WDT Reset)          ║\n");
  DEBUGOUT("║  Citation: siw917x-family-rm.pdf Section 9.10.13/14          ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] WDT reset detected - applying FULL HARDWARE RESET workaround\n");
  
  /* Read current state for diagnostics */
  p2p_status = MCU_P2P_COMM_STATUS_REG;
  DEBUGOUT("[NWP-Fix] === BEFORE FIX ===\n");
  DEBUGOUT("[NWP-Fix] P2P_STATUS = 0x%08lX\n", (unsigned long)p2p_status);
  DEBUGOUT("[NWP-Fix]   TA_is_active (bit 3) = %lu\n", (unsigned long)((p2p_status >> 3) & 1));
  DEBUGOUT("[NWP-Fix]   NWP_wakeup_MCU (bit 2) = %lu\n", (unsigned long)((p2p_status >> 2) & 1));
  DEBUGOUT("[NWP-Fix]   MCU_active (bit 1) = %lu\n", (unsigned long)((p2p_status >> 1) & 1));
  DEBUGOUT("[NWP-Fix]   M4_wakeup_NWP (bit 0) = %lu\n", (unsigned long)((p2p_status >> 0) & 1));
  DEBUGOUT("[NWP-Fix] HOST_INTF_REG_OUT = 0x%08lX\n", (unsigned long)HOST_INTF_REG_OUT);
  DEBUGOUT("[NWP-Fix] M4SS_TASS_CTRL_SET_REG = 0x%08lX\n", (unsigned long)M4SS_TASS_CTRL_SET_REG);
  
  /*
   * CRITICAL INSIGHT: The NWP bootloader responds with 0xAB (HOST_INTF ready),
   * BUT the WiFi stack inside NWP is in a corrupted state after WDT reset.
   * 
   * The SDK checks TA_is_active=1 and calls rsi_waitfor_boardready() which
   * expects the NWP to respond to commands - but it doesn't, causing hang.
   * 
   * SOLUTION: Force a COMPLETE NWP hardware reset using ALL THREE control bits:
   *   - Bit 0: Power gate (turn off NWP power)
   *   - Bit 1: Isolation bypass (isolate NWP domain)
   *   - Bit 2: Reset bypass (assert NWP reset) <- KEY!
   * 
   * This mimics what the physical RESET button does.
   */
  
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] ★ APPLYING FULL NWP HARDWARE RESET SEQUENCE ★\n");
  DEBUGOUT("[NWP-Fix] This sequence mimics the physical RESET button behavior.\n");
  DEBUGOUT("\n");
  
  /* =========================================================================
   * PHASE 1: ASSERT RESET AND POWER-OFF NWP
   * Citation: siw917x-family-rm.pdf Section 9.10.14 - M4SS_TASS_CTRL_CLEAR_REG
   *   "Writing 1 to bit 0 allows M4SS to turn off NWP always-on domain"
   *   "Writing 1 to bit 2 allows M4SS to turn on NWP always-on reset pin in bypass"
   * ========================================================================= */
  DEBUGOUT("[NWP-Fix] Phase 1: Asserting NWP reset and disabling power...\n");
  
  /* 1a. Clear MCU_wakeup_NWP AND MCU_ACTIVE_STATUS to stop any ongoing handshake
   * Citation: siw917x-family-rm.pdf Section 24.5.51 - MCR_MCU_P2P_COMM_STATUS_REG
   *   Bit 1 (MCU_ACTIVE_STATUS): "This bit is used to indicate to the NWP that MCU is active"
   *   Bit 0 (MCU_WAKEUP_NWP): "This bit is used to wakeup the NWP from sleep"
   * 
   * Clearing both bits tells NWP: "MCU is going quiet, stop expecting communication"
   */
  DEBUGOUT("[NWP-Fix]   1a: Clearing M4_wakeup_NWP and MCU_ACTIVE_STATUS...\n");
  MCU_P2P_COMM_STATUS_REG &= ~(P2P_MCU_WAKEUP_NWP | P2P_MCU_ACTIVE_STATUS);
  osDelay(5);
  
  /* 1b. Assert NWP reset via reset bypass (CLEAR register clears the reset bypass,
   *     which causes reset to be applied - the naming is counterintuitive) */
  DEBUGOUT("[NWP-Fix]   1b: Asserting NWP reset (RST_BYPASS)...\n");
  M4SS_TASS_CTRL_CLEAR_REG = TASS_AON_PWR_DMN_RST_BYPASS;
  osDelay(5);
  
  /* 1c. Enable isolation to fully isolate NWP domain */
  DEBUGOUT("[NWP-Fix]   1c: Enabling NWP isolation (ISO_BYPASS)...\n");
  M4SS_TASS_CTRL_SET_REG = TASS_AON_DISABLE_ISO_BYPASS;
  osDelay(5);
  
  /* 1d. Disable NWP power gate */
  DEBUGOUT("[NWP-Fix]   1d: Disabling NWP power gate...\n");
  M4SS_TASS_CTRL_CLEAR_REG = TASS_AON_PWRGATE_EN;
  
  /* Wait for power to fully discharge */
  DEBUGOUT("[NWP-Fix]   Waiting 100ms for NWP power discharge...\n");
  osDelay(100);
  
  /* Verify NWP is fully off */
  p2p_status = MCU_P2P_COMM_STATUS_REG;
  DEBUGOUT("[NWP-Fix]   After power-off: P2P_STATUS = 0x%08lX\n", (unsigned long)p2p_status);
  DEBUGOUT("[NWP-Fix]   TA_is_active = %lu (should be 0)\n", (unsigned long)((p2p_status >> 3) & 1));
  
  /* =========================================================================
   * PHASE 2: RE-ENABLE NWP IN PROPER SEQUENCE
   * Citation: siw917x-family-rm.pdf Section 9.10.13 - M4SS_TASS_CTRL_SET_REG
   *   "Writing 1 to bit 0 allows M4SS to turn on NWP always-on domain"
   * ========================================================================= */
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] Phase 2: Re-enabling NWP in proper boot sequence...\n");
  
  /* 2a. Re-enable NWP power gate */
  DEBUGOUT("[NWP-Fix]   2a: Enabling NWP power gate...\n");
  M4SS_TASS_CTRL_SET_REG = TASS_AON_PWRGATE_EN;
  osDelay(20);
  
  /* 2b. Disable isolation */
  DEBUGOUT("[NWP-Fix]   2b: Disabling NWP isolation...\n");
  M4SS_TASS_CTRL_CLEAR_REG = TASS_AON_DISABLE_ISO_BYPASS;
  osDelay(5);
  
  /* 2c. De-assert reset (enable reset bypass to release reset) */
  DEBUGOUT("[NWP-Fix]   2c: De-asserting NWP reset...\n");
  M4SS_TASS_CTRL_SET_REG = TASS_AON_PWR_DMN_RST_BYPASS;
  
  /* Wait for NWP to start booting */
  DEBUGOUT("[NWP-Fix]   Waiting 200ms for NWP boot sequence...\n");
  osDelay(200);
  
  /* =========================================================================
   * PHASE 3: INITIATE NWP WAKEUP HANDSHAKE
   * Citation: siw917x-family-rm.pdf Section 24.5.51 - MCR_MCU_P2P_COMM_STATUS_REG
   *   "Bit 0 (MCU_WAKEUP_NWP): This bit is used to wakeup the NWP from sleep."
   * ========================================================================= */
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] Phase 3: Initiating NWP wakeup handshake...\n");
  
  /* Pulse the wakeup indication */
  DEBUGOUT("[NWP-Fix]   3a: Pulsing M4SS_P2P_INTR wakeup signal...\n");
  M4SS_P2P_INTR_CLR_REG = M4_WAKEUP_TA_BIT;
  osDelay(5);
  M4SS_P2P_INTR_SET_REG = M4_WAKEUP_TA_BIT;
  osDelay(10);
  
  /* Set M4_wakeup_NWP AND MCU_ACTIVE_STATUS to complete handshake
   * Citation: siw917x-family-rm.pdf Section 24.5.51 - MCR_MCU_P2P_COMM_STATUS_REG
   *   Bit 1 (MCU_ACTIVE_STATUS): "This bit is used to indicate to the NWP that MCU is active"
   *   Bit 0 (MCU_WAKEUP_NWP): "This bit is used to wakeup the NWP from sleep"
   * 
   * Setting BOTH bits tells NWP: "MCU is active and ready for communication"
   */
  DEBUGOUT("[NWP-Fix]   3b: Setting M4_wakeup_NWP and MCU_ACTIVE_STATUS...\n");
  MCU_P2P_COMM_STATUS_REG |= (P2P_MCU_WAKEUP_NWP | P2P_MCU_ACTIVE_STATUS);
  
  /* =========================================================================
   * PHASE 4: WAIT FOR NWP TO FULLY INITIALIZE
   * The NWP is ready when HOST_INTF_REG_OUT bits [15:8] = 0xAB
   * ========================================================================= */
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] Phase 4: Waiting for NWP to fully initialize (max %lu ms)...\n",
           (unsigned long)MAX_WAIT_MS);
  
  timeout_count = 0;
  while (timeout_count < MAX_WAIT_MS) {
    osDelay(POLL_INTERVAL_MS);
    timeout_count += POLL_INTERVAL_MS;
    
    uint32_t host_intf_out = HOST_INTF_REG_OUT;
    p2p_status = MCU_P2P_COMM_STATUS_REG;
    
    /* NWP is ready when bits [15:8] = 0xAB (WIFI_REGISTER_VALID) */
    if (((host_intf_out >> 8) & 0xFF) == WIFI_REGISTER_VALID) {
      DEBUGOUT("[NWP-Fix] ✓ NWP bootloader ready after full reset! (%lu ms)\n", (unsigned long)timeout_count);
      DEBUGOUT("[NWP-Fix]   HOST_INTF = 0x%08lX (0xAB signature present)\n", (unsigned long)host_intf_out);
      DEBUGOUT("[NWP-Fix]   P2P_STATUS = 0x%08lX\n", (unsigned long)p2p_status);
      DEBUGOUT("[NWP-Fix]   TA_is_active = %lu\n", (unsigned long)((p2p_status >> 3) & 1));
      
      /*
       * =====================================================================
       * CRITICAL FIX PHASE 4a: Wait for WiFi firmware to fully initialize
       * =====================================================================
       * 
       * The 0xAB signature indicates the NWP BOOTLOADER is ready, but the
       * WiFi FIRMWARE inside NWP needs additional time to initialize after
       * a power-cycle reset.
       * 
       * Citation: siw917x-family-rm.pdf Section 7 "Resets":
       *   "On any reset, execution enters the Security Bootloader and then
       *    hands off to the Application Bootloader"
       * 
       * The bootloader reports ready quickly, but then loads and starts the
       * WiFi firmware. If we call sl_net_init() too soon, the SDK sends
       * commands to a WiFi stack that isn't ready yet, causing hangs.
       * 
       * SOLUTION: Wait additional time AND verify P2P_STATUS stabilizes
       * =====================================================================
       */
      DEBUGOUT("\n");
      DEBUGOUT("[NWP-Fix] Phase 4a: Waiting for NWP WiFi firmware to fully start...\n");
      DEBUGOUT("[NWP-Fix]   (0xAB = bootloader ready, need to wait for WiFi stack)\n");
      
      /* Wait for WiFi firmware to initialize after bootloader handshake
       * Empirically, 500ms is typically sufficient, but we monitor P2P_STATUS
       * to ensure the NWP state machine has stabilized.
       */
      const uint32_t WIFI_FIRMWARE_INIT_DELAY_MS = 500;
      const uint32_t WIFI_FIRMWARE_POLL_MS = 50;
      uint32_t wifi_init_elapsed = 0;
      uint32_t stable_count = 0;
      uint32_t last_p2p_status = p2p_status;
      
      while (wifi_init_elapsed < WIFI_FIRMWARE_INIT_DELAY_MS) {
        osDelay(WIFI_FIRMWARE_POLL_MS);
        wifi_init_elapsed += WIFI_FIRMWARE_POLL_MS;
        
        uint32_t current_p2p = MCU_P2P_COMM_STATUS_REG;
        uint32_t current_host_intf = HOST_INTF_REG_OUT;
        
        /* Check if P2P_STATUS has stabilized (same value for 3 consecutive reads) */
        if (current_p2p == last_p2p_status) {
          stable_count++;
        } else {
          stable_count = 0;
          last_p2p_status = current_p2p;
        }
        
        /* Print progress */
        DEBUGOUT("[NWP-Fix]   ...%lu ms: P2P=0x%08lX, HOST_INTF=0x%08lX, stable=%lu\n",
                 (unsigned long)wifi_init_elapsed,
                 (unsigned long)current_p2p,
                 (unsigned long)current_host_intf,
                 (unsigned long)stable_count);
        
        /* If stable for 3+ polls (150ms) and still has 0xAB, WiFi firmware is ready */
        if (stable_count >= 3 && ((current_host_intf >> 8) & 0xFF) == WIFI_REGISTER_VALID) {
          DEBUGOUT("[NWP-Fix] ✓ P2P_STATUS stable - WiFi firmware is ready!\n");
          break;
        }
      }
      
      DEBUGOUT("[NWP-Fix] Phase 4a complete: WiFi init wait = %lu ms\n", 
               (unsigned long)wifi_init_elapsed);
      
      /*
       * =====================================================================
       * CRITICAL FIX PHASE 4b: Clear SDK's internal software state
       * =====================================================================
       * 
       * Even though the NWP hardware reset succeeded (HOST_INTF = 0xAB), the
       * SDK's internal state variables persist in RAM across WDT reset.
       * 
       * If device_initialized = true (from before WDT reset), the SDK thinks:
       *   "Device is already initialized, skip bootloader handshake"
       * 
       * This causes sl_net_init() to call rsi_waitfor_boardready() with
       * incorrect assumptions about NWP state, leading to infinite hang.
       * 
       * Citation: sl_si91x_driver.c line 601:
       *   device_initialized = true; // Set after successful init
       * 
       * Citation: sl_si91x_driver.c line 724 (in sl_si91x_driver_deinit):
       *   device_initialized = false; // Cleared on deinit
       * 
       * By forcing device_initialized = false, we tell the SDK:
       *   "This is a fresh boot - do FULL initialization!"
       * =====================================================================
       */
      DEBUGOUT("\n");
      DEBUGOUT("[NWP-Fix] === CLEARING SDK INTERNAL STATE ===\n");
      DEBUGOUT("[NWP-Fix] device_initialized BEFORE: %s\n", device_initialized ? "true" : "false");
      
      /* Force clear SDK state to ensure fresh initialization */
      device_initialized = false;
      
      /* Also clear all interface status flags */
      for (int i = 0; i < SL_WIFI_MAX_INTERFACE_INDEX; i++) {
        interface_is_up[i] = false;
      }
      
      DEBUGOUT("[NWP-Fix] device_initialized AFTER: %s (forced to false)\n", 
               device_initialized ? "true" : "false");
      DEBUGOUT("[NWP-Fix] interface_is_up[]: all cleared to false\n");
      
      DEBUGOUT("\n");
      DEBUGOUT("[NWP-Fix] === NWP RESET SUCCESSFUL ===\n");
      DEBUGOUT("[NWP-Fix] NWP hardware reset complete, SDK state cleared.\n");
      DEBUGOUT("[NWP-Fix] sl_net_init() should now perform FULL initialization.\n");
      DEBUGOUT("\n");
      return true;
    }
    
    /* Print progress every 500ms */
    if ((timeout_count % 500) == 0) {
      DEBUGOUT("[NWP-Fix]   ...%lu ms: P2P=0x%08lX, HOST_INTF=0x%08lX, TA_active=%lu\n",
               (unsigned long)timeout_count, 
               (unsigned long)p2p_status,
               (unsigned long)host_intf_out,
               (unsigned long)((p2p_status >> 3) & 1));
    }
  }
  
  /* =========================================================================
   * TIMEOUT - NWP FAILED TO RESPOND
   * ========================================================================= */
  DEBUGOUT("\n");
  DEBUGOUT("[NWP-Fix] ✗ NWP FAILED TO RESPOND AFTER FULL RESET!\n");
  DEBUGOUT("[NWP-Fix]   Final P2P_STATUS = 0x%08lX\n", (unsigned long)MCU_P2P_COMM_STATUS_REG);
  DEBUGOUT("[NWP-Fix]   Final HOST_INTF = 0x%08lX\n", (unsigned long)HOST_INTF_REG_OUT);
  DEBUGOUT("[NWP-Fix]\n");
  DEBUGOUT("[NWP-Fix]   Possible causes:\n");
  DEBUGOUT("[NWP-Fix]     1. NWP firmware (ROM/Flash) is corrupted\n");
  DEBUGOUT("[NWP-Fix]     2. Power supply issue to NWP domain\n");
  DEBUGOUT("[NWP-Fix]     3. M4SS_TASS_CTRL not working as expected\n");
  DEBUGOUT("[NWP-Fix]     4. Need physical RESET button press to recover\n");
  
  /*
   * Even though NWP hardware reset failed, still clear SDK state.
   * This gives sl_net_init() the best chance of recovering.
   */
  DEBUGOUT("[NWP-Fix]\n");
  DEBUGOUT("[NWP-Fix] Clearing SDK state anyway (device_initialized = %s -> false)...\n",
           device_initialized ? "true" : "false");
  device_initialized = false;
  for (int i = 0; i < SL_WIFI_MAX_INTERFACE_INDEX; i++) {
    interface_is_up[i] = false;
  }
  
  DEBUGOUT("[NWP-Fix]\n");
  DEBUGOUT("[NWP-Fix]   Attempting sl_net_init() anyway (may hang)...\n");
  DEBUGOUT("[NWP-Fix]   TIP: Press physical RESET button if it hangs!\n");
  DEBUGOUT("\n");
  
  return false;
}

/*******************************************************************************
 * Main Test Function
 ******************************************************************************/

void wifi_http_test_run(void)
{
  sl_status_t status;
  sl_http_server_config_t server_config = { 0 };
  uint32_t wakeup_status;

  /* ===== BOOT DIAGNOSTICS ===== */
  wakeup_status = print_boot_diagnostics();

  DEBUGOUT("\n");
  DEBUGOUT("============================================================\n");
  DEBUGOUT("     ExpressLRS WiFi OTA Server - SiWx917 + LR1121\n");
  DEBUGOUT("============================================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Configuration:\n");
  DEBUGOUT("  SSID:     %s\n", WIFI_TEST_AP_SSID);
  DEBUGOUT("  Password: %s\n", WIFI_TEST_AP_PASSWORD);
  DEBUGOUT("  IP:       192.168.10.10\n");
  DEBUGOUT("  Port:     %d\n", WIFI_TEST_HTTP_PORT);
  DEBUGOUT("  Version:  %s\n", ELRS_VERSION);
  DEBUGOUT("  Target:   %s\n", ELRS_TARGET);
  DEBUGOUT("\n");

  /* ===== NWP WORKAROUND FOR WDT RESET ===== 
   * CRITICAL: Apply workaround BEFORE sl_net_init() to prevent hang!
   * See force_nwp_reinit_after_wdt() documentation for details.
   */
  force_nwp_reinit_after_wdt(wakeup_status);

  /* Step 1: Initialize WiFi AP Interface
   * IMPORTANT: Must be done FIRST in Common Flash mode!
   * Citation: SiWx917 Reference Manual - Common Flash requires NWP to be
   * initialized before any flash access (including NVM3) can occur.
   * 
   * NOTE: If called from elrs_main_task(), NWP may already be initialized
   * for NVM3 access. Check device_initialized flag to skip redundant init.
   * Citation: sl_si91x_driver.c - device_initialized tracks NWP init state
   */
  DEBUGOUT("[1/5] Initializing WiFi AP interface...\n");
  
  if (device_initialized) {
    /* NWP already initialized (e.g., by elrs_main_task for NVM3 access)
     * Skip sl_net_init() to avoid re-initialization issues.
     * 
     * Citation: ELRS 4.0 flow - WiFi is deferred until needed
     *   elrs_main_task() calls sl_net_init() early for NVM3, then
     *   wifi_http_test_run() is called later when WiFi mode is needed.
     *   At that point, NWP is already active, we just need to bring
     *   up the AP interface with sl_net_up().
     */
    DEBUGOUT("[WiFi] NWP already initialized (device_initialized=true)\n");
    DEBUGOUT("[WiFi] Skipping sl_net_init() - will proceed to sl_net_up()\n");
  } else {
    /* Fresh boot - need full NWP initialization */
    
    /* Detailed pre-init diagnostics to help debug hangs */
    DEBUGOUT("[Diag] === PRE-INIT REGISTER STATE ===\n");
    DEBUGOUT("[Diag] P2P_COMM_STATUS = 0x%08lX\n", (unsigned long)MCU_P2P_COMM_STATUS_REG);
    DEBUGOUT("[Diag]   TA_is_active (bit 3) = %lu\n", (unsigned long)((MCU_P2P_COMM_STATUS_REG >> 3) & 1));
    DEBUGOUT("[Diag]   MCU_active (bit 1) = %lu\n", (unsigned long)((MCU_P2P_COMM_STATUS_REG >> 1) & 1));
    DEBUGOUT("[Diag]   M4_wakeup_NWP (bit 0) = %lu\n", (unsigned long)((MCU_P2P_COMM_STATUS_REG >> 0) & 1));
    DEBUGOUT("[Diag] HOST_INTF_REG_OUT = 0x%08lX\n", (unsigned long)HOST_INTF_REG_OUT);
    DEBUGOUT("[Diag]   Ready signature = 0x%02lX (expect 0xAB)\n", (unsigned long)((HOST_INTF_REG_OUT >> 8) & 0xFF));
    DEBUGOUT("[Diag] M4SS_TASS_CTRL = 0x%08lX\n", (unsigned long)M4SS_TASS_CTRL_SET_REG);
    DEBUGOUT("[Diag] ===================================\n");
    DEBUGOUT("[Diag] About to call sl_net_init(SL_NET_WIFI_AP_INTERFACE)...\n");
    DEBUGOUT("[Diag] If output stops here, sl_net_init() is hanging in rsi_waitfor_boardready()!\n");
    DEBUGOUT("[Diag] Look for TA_is_active=1 above - that's the root cause!\n");

    status = sl_net_init(SL_NET_WIFI_AP_INTERFACE, NULL, NULL, NULL);
    DEBUGOUT("[Diag] sl_net_init() returned! status = 0x%lX\n", (unsigned long)status);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("ERROR: sl_net_init failed: 0x%lX\n", status);
      /* Decode common error codes */
      if (status == 0x10003) {
        DEBUGOUT("[Diag] Error 0x10003 = SL_STATUS_TIMEOUT - NWP boot timeout\n");
      } else if (status == 0x10002) {
        DEBUGOUT("[Diag] Error 0x10002 = SL_STATUS_FAIL - General failure\n");
      }
      return;
    }
    DEBUGOUT("      WiFi AP interface initialized.\n");
    DEBUGOUT("[Diag] P2P_STATUS after init: 0x%08lX\n", (unsigned long)MCU_P2P_COMM_STATUS_REG);

    /* Step 1.5: Initialize ELRS Configuration Storage (NVM3)
     * Citation: elrs_config.h - Loads saved config or initializes defaults
     * NOTE: In Common Flash mode, NVM3 requires NWP to be initialized first
     * for flash access coordination. This is why we init AFTER sl_net_init().
     * 
     * Only needed if we did sl_net_init() here (fresh boot path).
     * If called from elrs_main_task(), NVM3 was already initialized there.
     */
    DEBUGOUT("[1.5/5] Initializing ELRS configuration storage...\n");
    if (elrs_config_init() != 0) {
      DEBUGOUT("WARNING: ELRS config init failed, using defaults\n");
    } else {
      config_initialized = true;
      DEBUGOUT("      ELRS configuration loaded from NVM3.\n");
    }
  }

  /* Step 2: Set Credentials */
  DEBUGOUT("[2/5] Setting WiFi credentials...\n");

  status = sl_net_set_credential(SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID,
                                 SL_NET_WIFI_PSK,
                                 wifi_ap_credential.data,
                                 wifi_ap_credential.data_length);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("ERROR: sl_net_set_credential failed: 0x%lX\n", status);
    return;
  }
  DEBUGOUT("      Credentials set.\n");

  /* Step 3: Register WiFi Event Callbacks */
  DEBUGOUT("[3/5] Registering WiFi callbacks...\n");

  sl_wifi_set_callback(SL_WIFI_CLIENT_CONNECTED_EVENTS, ap_client_connected, NULL);
  sl_wifi_set_callback(SL_WIFI_CLIENT_DISCONNECTED_EVENTS, ap_client_disconnected, NULL);
  DEBUGOUT("      Callbacks registered.\n");

  /* Step 4: Bring Up WiFi AP */
  DEBUGOUT("[4/5] Starting WiFi Access Point...\n");

  status = sl_net_set_profile(SL_NET_WIFI_AP_INTERFACE,
                              SL_NET_DEFAULT_WIFI_AP_PROFILE_ID,
                              &wifi_ap_profile);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("ERROR: sl_net_set_profile failed: 0x%lX\n", status);
    return;
  }

  status = sl_net_up(SL_NET_WIFI_AP_INTERFACE, SL_NET_DEFAULT_WIFI_AP_PROFILE_ID);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("ERROR: sl_net_up failed: 0x%lX\n", status);
    return;
  }
  ap_running = true;
  DEBUGOUT("      WiFi AP started! SSID: %s\n", WIFI_TEST_AP_SSID);

  /* Step 5: Initialize and Start HTTP Server */
  DEBUGOUT("[5/5] Starting HTTP server with ELRS Web UI...\n");

  server_config.port             = WIFI_TEST_HTTP_PORT;
  server_config.default_handler  = handle_not_found;
  server_config.handlers_list    = http_handlers;
  server_config.handlers_count   = HTTP_HANDLER_COUNT;
  server_config.client_idle_time = 30;

  status = sl_http_server_init(&server_handle, &server_config);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("ERROR: sl_http_server_init failed: 0x%lX\n", status);
    return;
  }

  status = sl_http_server_start(&server_handle);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("ERROR: sl_http_server_start failed: 0x%lX\n", status);
    sl_http_server_deinit(&server_handle);
    return;
  }
  server_running = true;

  DEBUGOUT("\n");
  DEBUGOUT("============================================================\n");
  DEBUGOUT("  ExpressLRS Web UI Server RUNNING!\n");
  DEBUGOUT("============================================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Connect to WiFi: %s\n", WIFI_TEST_AP_SSID);
  DEBUGOUT("Password: %s\n", WIFI_TEST_AP_PASSWORD);
  DEBUGOUT("\n");
  DEBUGOUT("Open in browser:\n");
  DEBUGOUT("  ELRS Config:    http://192.168.10.10/\n");
  DEBUGOUT("  LR1121 Upload:  http://192.168.10.10/upload.html  <-- Binary-safe!\n");
  DEBUGOUT("\n");
  DEBUGOUT("ELRS Web Assets: %u files loaded\n", (unsigned int)WEB_ASSETS_COUNT);
  DEBUGOUT("\n");
  DEBUGOUT("Waiting for connections...\n");
  DEBUGOUT("============================================================\n");

  /* Main Loop */
  while (server_running) {
    osDelay(100);  /* Check more frequently for pending saves */

    /* Check for deferred config save
     * NVM3 writes hang when called from HTTP callback context.
     * Doing it here in the main loop avoids the NWP/M4 flash contention.
     */
    if (config_save_pending) {
      config_save_pending = false;
      DEBUGOUT("[WiFi] Performing deferred config save...\n");
      int result = elrs_config_save();
      if (result == 0) {
        DEBUGOUT("[WiFi] Config saved to NVM3 successfully!\n");
      } else {
        DEBUGOUT("[WiFi] ERROR: Config save failed: %d\n", result);
      }
    }

    static uint32_t tick = 0;
    if (++tick % 300 == 0) {  /* Every 30 seconds (300 * 100ms) */
      DEBUGOUT("[Status] Clients: %lu, Requests: %lu, Uptime: %lu ms\n",
               client_count, request_count, osKernelGetTickCount());
    }
  }

  /* Cleanup */
  DEBUGOUT("Stopping HTTP server...\n");
  sl_http_server_stop(&server_handle);
  sl_http_server_deinit(&server_handle);

  DEBUGOUT("Stopping WiFi AP...\n");
  sl_net_down(SL_NET_WIFI_AP_INTERFACE);
  ap_running = false;

  DEBUGOUT("WiFi HTTP test complete.\n");
}

/*******************************************************************************
 * Deferred Config Save
 * 
 * Called from HTTP POST handler to request a config save.
 * The actual save happens in the main loop to avoid NWP/M4 flash contention.
 ******************************************************************************/

void wifi_http_request_config_save(void)
{
  DEBUGOUT("[WiFi] Config save requested (will save in main loop)\n");
  config_save_pending = true;
}

/*******************************************************************************
 * Status Query Functions
 ******************************************************************************/

bool wifi_http_test_is_ap_running(void)
{
  return ap_running;
}

bool wifi_http_test_is_server_running(void)
{
  return server_running;
}

uint32_t wifi_http_test_get_client_count(void)
{
  return client_count;
}

uint32_t wifi_http_test_get_request_count(void)
{
  return request_count;
}
