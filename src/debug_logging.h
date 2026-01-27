/**
 * @file debug_logging.h
 * @brief Diagnostic logging for ELRS WiFi AP boot debugging
 * 
 * This module provides timestamped diagnostic logging to identify
 * where the boot sequence hangs after OTA firmware update.
 * 
 * Usage: Include this header and use DIAG_LOG() macro at key checkpoints.
 * Output goes to UART/printf - ensure USART is initialized early.
 */

#ifndef DEBUG_LOGGING_H
#define DEBUG_LOGGING_H

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration - Enable/Disable diagnostic logging
 * ============================================================================ */
#define DIAG_LOGGING_ENABLED    1   /* Set to 0 to disable all diagnostic logging */
#define DIAG_LOG_TIMESTAMPS     1   /* Include tick count in logs */
#define DIAG_LOG_FUNCTION_NAME  1   /* Include function name in logs */

/* ============================================================================
 * Boot Stage Identifiers - Used to track progress through initialization
 * ============================================================================ */
typedef enum {
    BOOT_STAGE_RESET           = 0x00,  /* Fresh reset */
    BOOT_STAGE_EARLY_INIT      = 0x01,  /* Before any SDK init */
    BOOT_STAGE_CLOCK_INIT      = 0x02,  /* Clock configuration */
    BOOT_STAGE_NVIC_INIT       = 0x03,  /* Interrupt controller setup */
    BOOT_STAGE_GPIO_INIT       = 0x04,  /* GPIO initialization */
    BOOT_STAGE_USART_INIT      = 0x05,  /* USART/debug console init */
    BOOT_STAGE_NVM3_INIT       = 0x06,  /* NVM3 flash storage init */
    BOOT_STAGE_WIFI_DEVICE_INIT = 0x10, /* sl_wifi_init() */
    BOOT_STAGE_NET_INIT        = 0x11,  /* sl_net_init() */
    BOOT_STAGE_NET_UP          = 0x12,  /* sl_net_up() - WiFi interface up */
    BOOT_STAGE_AP_CONFIG       = 0x13,  /* AP configuration setup */
    BOOT_STAGE_AP_START        = 0x14,  /* sl_wifi_start_ap() */
    BOOT_STAGE_DHCP_SERVER     = 0x15,  /* DHCP server start */
    BOOT_STAGE_HTTP_SERVER     = 0x16,  /* HTTP server start */
    BOOT_STAGE_RTOS_START      = 0x20,  /* FreeRTOS scheduler start */
    BOOT_STAGE_TASK_CREATE     = 0x21,  /* Task creation */
    BOOT_STAGE_GSPI_INIT       = 0x30,  /* GSPI for LR1121 */
    BOOT_STAGE_LR1121_INIT     = 0x31,  /* LR1121 radio init */
    BOOT_STAGE_ELRS_INIT       = 0x32,  /* ELRS protocol init */
    BOOT_STAGE_BOOT_COMPLETE   = 0xFF,  /* Boot sequence complete */
} boot_stage_t;

/* ============================================================================
 * Global boot stage tracker - can be read via debugger if hang occurs
 * ============================================================================ */
extern volatile boot_stage_t g_current_boot_stage;
extern volatile uint32_t g_last_error_code;
extern volatile uint32_t g_boot_checkpoint_count;

/* ============================================================================
 * Diagnostic Logging Macros
 * ============================================================================ */

#if DIAG_LOGGING_ENABLED

/* Get tick count for timestamp - assumes FreeRTOS or HAL tick available */
uint32_t diag_get_tick_count(void);

/* Core logging function */
void diag_log_impl(const char *file, int line, const char *func, 
                   const char *fmt, ...);

/* Set and log boot stage transition */
void diag_set_boot_stage(boot_stage_t stage, const char *description);

/* Log an error with code */
void diag_log_error(uint32_t error_code, const char *context);

#if DIAG_LOG_FUNCTION_NAME
  #define DIAG_LOG(fmt, ...) \
      diag_log_impl(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#else
  #define DIAG_LOG(fmt, ...) \
      diag_log_impl(__FILE__, __LINE__, "", fmt, ##__VA_ARGS__)
#endif

/* Boot stage transition macro */
#define DIAG_BOOT_STAGE(stage, desc) diag_set_boot_stage(stage, desc)

/* Error logging macro */
#define DIAG_ERROR(code, ctx) diag_log_error(code, ctx)

/* Status check macro - logs if status != expected */
#define DIAG_CHECK_STATUS(status, expected, context) do { \
    if ((status) != (expected)) { \
        DIAG_LOG("ERROR: %s returned 0x%lX (expected 0x%lX)", \
                 context, (uint32_t)(status), (uint32_t)(expected)); \
        diag_log_error((uint32_t)(status), context); \
    } else { \
        DIAG_LOG("OK: %s returned 0x%lX", context, (uint32_t)(status)); \
    } \
} while(0)

/* Silicon Labs specific status check */
#define DIAG_CHECK_SL_STATUS(status, context) \
    DIAG_CHECK_STATUS(status, SL_STATUS_OK, context)

#else /* DIAG_LOGGING_ENABLED == 0 */

#define DIAG_LOG(fmt, ...)           ((void)0)
#define DIAG_BOOT_STAGE(stage, desc) ((void)0)
#define DIAG_ERROR(code, ctx)        ((void)0)
#define DIAG_CHECK_STATUS(s, e, c)   ((void)0)
#define DIAG_CHECK_SL_STATUS(s, c)   ((void)0)

#endif /* DIAG_LOGGING_ENABLED */

/* ============================================================================
 * Checkpoint markers - increment counter for watchdog/debugger visibility
 * ============================================================================ */
#define DIAG_CHECKPOINT() do { \
    g_boot_checkpoint_count++; \
    DIAG_LOG("CHECKPOINT #%lu", (unsigned long)g_boot_checkpoint_count); \
} while(0)

/* ============================================================================
 * Memory-mapped debug register for SWD/debugger visibility
 * Write boot stage to a known RAM location that debugger can read
 * ============================================================================ */
#define DIAG_DEBUG_RAM_ADDR  0x20000000  /* Adjust for SiWx917 RAM base */
#define DIAG_WRITE_DEBUG_REG(val) \
    (*(volatile uint32_t *)(DIAG_DEBUG_RAM_ADDR) = (uint32_t)(val))

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOGGING_H */
