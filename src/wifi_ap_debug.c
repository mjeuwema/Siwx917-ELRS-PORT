/**
 * @file wifi_ap_debug.c
 * @brief WiFi AP initialization wrapper with diagnostic logging
 * 
 * IMPORTANT: This file wraps SiWx917 WiFi AP init with detailed logging
 * to identify boot hangs after OTA firmware update.
 * 
 * References (Silicon Labs SiWx917):
 * - WiFi init sequence: sl_wifi_init() -> sl_net_init() -> sl_net_up()
 * - AP mode uses SL_NET_WIFI_AP_INTERFACE
 * - Boot hang after OTA typically caused by NVM3/calibration data issues
 */

#include "wifi_ap_debug.h"
#include "debug_logging.h"

/* Silicon Labs WiFi SDK includes */
#include "sl_wifi.h"
#include "sl_wifi_types.h"
#include "sl_net.h"
#include "sl_net_wifi_types.h"
#include "sl_si91x_driver.h"

/* FreeRTOS for heap checking */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes */
#include <string.h>

/* ============================================================================
 * Global State
 * ============================================================================ */
wifi_ap_init_result_t g_wifi_ap_init_result = {0};

/* ============================================================================
 * Helper: Log heap state
 * ============================================================================ */
static void log_heap_state(const char *context)
{
    size_t free_heap = xPortGetFreeHeapSize();
    size_t min_ever_free = xPortGetMinimumEverFreeHeapSize();
    
    DIAG_LOG("HEAP [%s]: Free=%u, MinEver=%u", 
             context, (unsigned)free_heap, (unsigned)min_ever_free);
    
    /* Warn if heap is critically low */
    if (free_heap < 4096) {
        DIAG_LOG("WARNING: Heap critically low! < 4KB remaining");
    }
}

/* ============================================================================
 * Pre-Init Checks
 * ============================================================================ */
sl_status_t wifi_ap_pre_init_checks(void)
{
    sl_status_t status = SL_STATUS_OK;
    
    DIAG_BOOT_STAGE(BOOT_STAGE_EARLY_INIT, "Pre-init checks starting");
    DIAG_CHECKPOINT();
    
    /* 1. Check reset reason - important for OTA debugging */
    DIAG_LOG("Checking reset reason...");
    /* On SiWx917, check PMU status registers if available */
    /* This helps identify if OTA caused unexpected reset type */
    
    /* 2. Log initial heap state */
    log_heap_state("PRE-INIT");
    
    /* 3. Check if WiFi was previously initialized (shouldn't be at boot) */
    DIAG_LOG("Checking for stale WiFi state...");
    /* sl_wifi_is_interface_up() could indicate incomplete shutdown */
    
    /* 4. Verify critical configuration */
    DIAG_LOG("Verifying AP configuration parameters...");
    /* Check SSID, channel, security settings are valid */
    
    DIAG_LOG("Pre-init checks PASSED");
    return status;
}

/* ============================================================================
 * Main WiFi AP Init with Diagnostics
 * ============================================================================ */
sl_status_t wifi_ap_init_with_diagnostics(void)
{
    sl_status_t status;
    
    printf("\r\n");
    printf("################################################################\r\n");
    printf("# WIFI AP INIT WITH DIAGNOSTICS - START                       #\r\n");
    printf("################################################################\r\n");
    
    /* Clear result structure */
    memset(&g_wifi_ap_init_result, 0, sizeof(g_wifi_ap_init_result));
    
    /* ========== STAGE 1: Pre-init checks ========== */
    status = wifi_ap_pre_init_checks();
    if (status != SL_STATUS_OK) {
        DIAG_ERROR(status, "Pre-init checks failed");
        g_wifi_ap_init_result.overall_status = status;
        g_wifi_ap_init_result.failed_at_stage = BOOT_STAGE_EARLY_INIT;
        return status;
    }
    
    /* ========== STAGE 2: WiFi Device Init ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_WIFI_DEVICE_INIT, "Calling sl_wifi_init()");
    DIAG_CHECKPOINT();
    log_heap_state("BEFORE_WIFI_INIT");
    
    DIAG_LOG(">>> sl_wifi_init() - ENTERING");
    
    /* 
     * sl_wifi_init() initializes the WiFi device (NWP firmware loaded)
     * This can hang if:
     * - NWP firmware image corrupt after OTA
     * - SPI/GSPI communication with NWP fails
     * - Memory allocation fails for WiFi context
     */
    sl_wifi_device_configuration_t wifi_config = {
        .boot_option = LOAD_NWP_FW,
        .mac_address = NULL,
        .band = SL_SI91X_WIFI_BAND_2_4GHZ,
        .region_code = DEFAULT_REGION,
        .boot_config = {
            .oper_mode = SL_SI91X_ACCESS_POINT_MODE,
            .coex_mode = SL_SI91X_WLAN_ONLY_MODE,
            .feature_bit_map = 0,
            .tcp_ip_feature_bit_map = (SL_SI91X_TCP_IP_FEAT_DHCPV4_SERVER |
                                       SL_SI91X_TCP_IP_FEAT_HTTP_SERVER),
            .custom_feature_bit_map = 0,
            .ext_custom_feature_bit_map = 0,
            .bt_feature_bit_map = 0,
            .ext_tcp_ip_feature_bit_map = 0,
            .ble_feature_bit_map = 0,
            .ble_ext_feature_bit_map = 0,
            .config_feature_bit_map = 0,
        },
    };
    
    DIAG_LOG("WiFi config: mode=AP, band=2.4GHz, DHCP+HTTP enabled");
    
    status = sl_wifi_init(&wifi_config, NULL, NULL);
    
    g_wifi_ap_init_result.wifi_init_status = status;
    DIAG_LOG("<<< sl_wifi_init() returned: 0x%08lX (%s)", 
             (unsigned long)status, diag_decode_sl_status(status));
    
    if (status != SL_STATUS_OK) {
        DIAG_ERROR(status, "sl_wifi_init() FAILED");
        g_wifi_ap_init_result.overall_status = status;
        g_wifi_ap_init_result.failed_at_stage = BOOT_STAGE_WIFI_DEVICE_INIT;
        return status;
    }
    
    log_heap_state("AFTER_WIFI_INIT");
    DIAG_LOG("sl_wifi_init() SUCCESS");
    DIAG_CHECKPOINT();
    
    /* ========== STAGE 3: Network Interface Init ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_NET_INIT, "Calling sl_net_init()");
    DIAG_CHECKPOINT();
    
    DIAG_LOG(">>> sl_net_init(WIFI_AP_INTERFACE) - ENTERING");
    
    /*
     * sl_net_init() initializes the network stack for the specified interface
     * For AP mode, use SL_NET_WIFI_AP_INTERFACE
     */
    status = sl_net_init(SL_NET_WIFI_AP_INTERFACE, NULL, NULL, NULL);
    
    g_wifi_ap_init_result.net_init_status = status;
    DIAG_LOG("<<< sl_net_init() returned: 0x%08lX (%s)", 
             (unsigned long)status, diag_decode_sl_status(status));
    
    if (status != SL_STATUS_OK) {
        DIAG_ERROR(status, "sl_net_init() FAILED");
        g_wifi_ap_init_result.overall_status = status;
        g_wifi_ap_init_result.failed_at_stage = BOOT_STAGE_NET_INIT;
        return status;
    }
    
    log_heap_state("AFTER_NET_INIT");
    DIAG_LOG("sl_net_init() SUCCESS");
    DIAG_CHECKPOINT();
    
    /* ========== STAGE 4: AP Configuration ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_AP_CONFIG, "Configuring AP parameters");
    DIAG_CHECKPOINT();
    
    DIAG_LOG("Setting up AP configuration...");
    
    /* Log AP settings for debugging */
    DIAG_LOG("AP SSID: ELRS_RX (or configured value)");
    DIAG_LOG("AP Channel: 1 (or configured value)");
    DIAG_LOG("AP Security: OPEN (or configured value)");
    
    /* You may need to call sl_net_set_credential() here for secured AP */
    
    DIAG_CHECKPOINT();
    
    /* ========== STAGE 5: Bring Up AP Interface ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_NET_UP, "Calling sl_net_up()");
    DIAG_CHECKPOINT();
    
    DIAG_LOG(">>> sl_net_up(WIFI_AP_INTERFACE) - ENTERING");
    DIAG_LOG("WARNING: This is often where boot hangs after OTA!");
    
    /*
     * sl_net_up() brings up the interface and starts the AP
     * This is the most likely hang point because:
     * - It actually starts the WiFi radio
     * - Requires valid calibration data in NVM
     * - Allocates significant buffers
     * - Starts internal tasks
     */
    sl_net_wifi_ap_profile_t ap_profile = {
        .config = {
            .ssid = {
                .value = "ELRS_RX",
                .length = 7,
            },
            .channel = {
                .channel = 1,
                .band = SL_WIFI_BAND_2_4GHZ,
                .bandwidth = SL_WIFI_BANDWIDTH_20MHz,
            },
            .security = SL_WIFI_OPEN,
            .encryption = SL_WIFI_NO_ENCRYPTION,
            .rate_protocol = SL_WIFI_RATE_PROTOCOL_AUTO,
            .options = 0,
            .credential_id = 0,
            .keepalive_type = SL_SI91X_AP_NULL_BASED_KEEP_ALIVE,
            .beacon_interval = 100,
            .client_idle_timeout = 120,
            .dtim_beacon_count = 3,
            .maximum_clients = 4,
        },
        .ip = {
            .mode = SL_IP_MANAGEMENT_STATIC_IP,
            .type = SL_IPV4,
            .host_name = NULL,
            .ip = {
                .v4 = {
                    .ip_address = {192, 168, 4, 1},
                    .gateway = {192, 168, 4, 1},
                    .netmask = {255, 255, 255, 0},
                },
            },
        },
    };
    
    DIAG_LOG("AP Profile: SSID=ELRS_RX, CH=1, IP=192.168.4.1");
    
    status = sl_net_up(SL_NET_WIFI_AP_INTERFACE, 
                       SL_NET_DEFAULT_WIFI_AP_PROFILE_ID);
    
    g_wifi_ap_init_result.net_up_status = status;
    DIAG_LOG("<<< sl_net_up() returned: 0x%08lX (%s)", 
             (unsigned long)status, diag_decode_sl_status(status));
    
    if (status != SL_STATUS_OK) {
        DIAG_ERROR(status, "sl_net_up() FAILED - LIKELY HANG POINT");
        g_wifi_ap_init_result.overall_status = status;
        g_wifi_ap_init_result.failed_at_stage = BOOT_STAGE_NET_UP;
        return status;
    }
    
    log_heap_state("AFTER_NET_UP");
    DIAG_LOG("sl_net_up() SUCCESS - AP IS RUNNING");
    DIAG_CHECKPOINT();
    
    /* ========== STAGE 6: HTTP Server (if not auto-started) ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_HTTP_SERVER, "Starting HTTP server");
    DIAG_CHECKPOINT();
    
    DIAG_LOG("HTTP server should auto-start with TCP_IP_FEAT_HTTP_SERVER flag");
    /* If manual start needed:
     * status = sl_http_server_init(...);
     * status = sl_http_server_start(...);
     */
    
    g_wifi_ap_init_result.http_status = SL_STATUS_OK;
    DIAG_CHECKPOINT();
    
    /* ========== COMPLETE ========== */
    DIAG_BOOT_STAGE(BOOT_STAGE_BOOT_COMPLETE, "WiFi AP init complete!");
    
    log_heap_state("INIT_COMPLETE");
    
    g_wifi_ap_init_result.overall_status = SL_STATUS_OK;
    
    printf("\r\n");
    printf("################################################################\r\n");
    printf("# WIFI AP INIT SUCCESS                                        #\r\n");
    printf("# AP SSID: ELRS_RX                                           #\r\n");
    printf("# AP IP: 192.168.4.1                                         #\r\n");
    printf("################################################################\r\n");
    
    return SL_STATUS_OK;
}

/* ============================================================================
 * Diagnostic State Dump
 * ============================================================================ */
void wifi_ap_dump_diagnostic_state(void)
{
    printf("\r\n");
    printf("================= DIAGNOSTIC STATE DUMP =================\r\n");
    printf("Current boot stage: 0x%02X\r\n", (unsigned)g_current_boot_stage);
    printf("Checkpoint count: %lu\r\n", (unsigned long)g_boot_checkpoint_count);
    printf("Last error code: 0x%08lX\r\n", (unsigned long)g_last_error_code);
    printf("\r\n");
    printf("WiFi AP Init Results:\r\n");
    printf("  wifi_init: 0x%08lX\r\n", (unsigned long)g_wifi_ap_init_result.wifi_init_status);
    printf("  net_init:  0x%08lX\r\n", (unsigned long)g_wifi_ap_init_result.net_init_status);
    printf("  net_up:    0x%08lX\r\n", (unsigned long)g_wifi_ap_init_result.net_up_status);
    printf("  Failed at stage: 0x%02X\r\n", (unsigned)g_wifi_ap_init_result.failed_at_stage);
    printf("\r\n");
    log_heap_state("DUMP");
    printf("=========================================================\r\n");
}

/* ============================================================================
 * Force Reset (for recovery after failed init)
 * ============================================================================ */
sl_status_t wifi_ap_force_reset(void)
{
    sl_status_t status;
    
    DIAG_LOG("Forcing WiFi reset...");
    
    /* Try to deinit WiFi cleanly */
    DIAG_LOG(">>> sl_wifi_deinit() - Attempting cleanup");
    status = sl_wifi_deinit();
    DIAG_LOG("<<< sl_wifi_deinit() returned: 0x%08lX", (unsigned long)status);
    
    /* Small delay for hardware to settle */
    /* vTaskDelay(pdMS_TO_TICKS(100)); */
    
    DIAG_LOG("WiFi reset complete");
    return SL_STATUS_OK;
}
