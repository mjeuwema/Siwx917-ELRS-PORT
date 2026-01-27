/**
 * @file wifi_ap_debug.h
 * @brief WiFi AP initialization with comprehensive diagnostic logging
 * 
 * This module wraps the Silicon Labs WiFi AP initialization sequence with
 * detailed logging at every step to identify boot hangs after OTA update.
 * 
 * CRITICAL: The hang after OTA likely occurs because:
 * 1. NVM3 flash state corruption
 * 2. WiFi calibration data missing/corrupt after flash erase
 * 3. Network interface not properly reset before re-init
 * 4. Task/memory state not properly cleaned up
 * 5. Interrupt state issues after OTA reboot
 * 
 * This wrapper helps identify the exact failure point.
 */

#ifndef WIFI_AP_DEBUG_H
#define WIFI_AP_DEBUG_H

#include "sl_status.h"
#include "debug_logging.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * WiFi AP Init Result Structure - Captures state at failure
 * ============================================================================ */
typedef struct {
    sl_status_t overall_status;
    sl_status_t wifi_init_status;
    sl_status_t net_init_status;
    sl_status_t net_up_status;
    sl_status_t ap_start_status;
    sl_status_t dhcp_status;
    sl_status_t http_status;
    boot_stage_t failed_at_stage;
    uint32_t    failed_at_checkpoint;
} wifi_ap_init_result_t;

/* Global result structure - accessible via debugger */
extern wifi_ap_init_result_t g_wifi_ap_init_result;

/* ============================================================================
 * Main Entry Point - Use this instead of direct WiFi init calls
 * ============================================================================ */

/**
 * @brief Initialize WiFi AP with comprehensive diagnostic logging
 * 
 * This function wraps the entire WiFi AP initialization sequence with
 * logging at every checkpoint. Call this from app_init() or equivalent.
 * 
 * Sequence:
 * 1. Pre-init checks (NVM3 state, memory, etc.)
 * 2. sl_wifi_init() - WiFi device initialization
 * 3. sl_net_init() - Network stack init for AP interface
 * 4. sl_net_up() - Bring up AP interface
 * 5. DHCP server start (if applicable)
 * 6. HTTP server start
 * 
 * @return SL_STATUS_OK on success, error code on failure
 */
sl_status_t wifi_ap_init_with_diagnostics(void);

/**
 * @brief Check WiFi module state before initialization
 * 
 * Performs pre-flight checks:
 * - NVM3 integrity
 * - Available heap memory
 * - Previous init state cleanup
 * 
 * @return SL_STATUS_OK if ready to init, error otherwise
 */
sl_status_t wifi_ap_pre_init_checks(void);

/**
 * @brief Dump diagnostic state (call if boot hangs or from watchdog)
 * 
 * Outputs current boot stage, last error, memory state.
 */
void wifi_ap_dump_diagnostic_state(void);

/**
 * @brief Reset WiFi module state for clean reinit
 * 
 * Call this before wifi_ap_init_with_diagnostics() if previous
 * init attempt failed or after OTA update.
 * 
 * @return SL_STATUS_OK on success
 */
sl_status_t wifi_ap_force_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_AP_DEBUG_H */
