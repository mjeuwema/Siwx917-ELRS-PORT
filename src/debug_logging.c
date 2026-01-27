/**
 * @file debug_logging.c
 * @brief Implementation of diagnostic logging for ELRS WiFi AP boot debugging
 * 
 * Provides timestamped logging output via UART to identify boot hangs.
 */

#include "debug_logging.h"
#include <stdarg.h>
#include <string.h>

#if DIAG_LOGGING_ENABLED

/* ============================================================================
 * Global State Variables - Visible to debugger
 * ============================================================================ */
volatile boot_stage_t g_current_boot_stage = BOOT_STAGE_RESET;
volatile uint32_t g_last_error_code = 0;
volatile uint32_t g_boot_checkpoint_count = 0;

/* Boot stage names for human-readable output */
static const char* boot_stage_names[] = {
    [BOOT_STAGE_RESET]           = "RESET",
    [BOOT_STAGE_EARLY_INIT]      = "EARLY_INIT",
    [BOOT_STAGE_CLOCK_INIT]      = "CLOCK_INIT",
    [BOOT_STAGE_NVIC_INIT]       = "NVIC_INIT",
    [BOOT_STAGE_GPIO_INIT]       = "GPIO_INIT",
    [BOOT_STAGE_USART_INIT]      = "USART_INIT",
    [BOOT_STAGE_NVM3_INIT]       = "NVM3_INIT",
    [BOOT_STAGE_WIFI_DEVICE_INIT] = "WIFI_DEVICE_INIT",
    [BOOT_STAGE_NET_INIT]        = "NET_INIT",
    [BOOT_STAGE_NET_UP]          = "NET_UP",
    [BOOT_STAGE_AP_CONFIG]       = "AP_CONFIG",
    [BOOT_STAGE_AP_START]        = "AP_START",
    [BOOT_STAGE_DHCP_SERVER]     = "DHCP_SERVER",
    [BOOT_STAGE_HTTP_SERVER]     = "HTTP_SERVER",
    [BOOT_STAGE_RTOS_START]      = "RTOS_START",
    [BOOT_STAGE_TASK_CREATE]     = "TASK_CREATE",
    [BOOT_STAGE_GSPI_INIT]       = "GSPI_INIT",
    [BOOT_STAGE_LR1121_INIT]     = "LR1121_INIT",
    [BOOT_STAGE_ELRS_INIT]       = "ELRS_INIT",
    [BOOT_STAGE_BOOT_COMPLETE]   = "BOOT_COMPLETE",
};

/* ============================================================================
 * Weak tick function - override with actual FreeRTOS/HAL tick
 * ============================================================================ */
__attribute__((weak))
uint32_t diag_get_tick_count(void)
{
    /* Try to use FreeRTOS tick if available */
#if defined(configUSE_TICK_HOOK) || defined(FREERTOS_H)
    extern volatile uint32_t xTaskGetTickCount(void);
    return xTaskGetTickCount();
#else
    /* Fallback: simple incrementing counter (not accurate timing) */
    static volatile uint32_t s_tick_counter = 0;
    return s_tick_counter++;
#endif
}

/* ============================================================================
 * Core Logging Implementation
 * ============================================================================ */
void diag_log_impl(const char *file, int line, const char *func, 
                   const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int offset = 0;
    
    /* Extract just filename from path */
    const char *filename = file;
    const char *p = file;
    while (*p) {
        if (*p == '/' || *p == '\\') {
            filename = p + 1;
        }
        p++;
    }
    
#if DIAG_LOG_TIMESTAMPS
    uint32_t tick = diag_get_tick_count();
    offset = snprintf(buffer, sizeof(buffer), "[%08lu] ", (unsigned long)tick);
#endif

    /* Add boot stage marker */
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                       "[S:%02X] ", (unsigned int)g_current_boot_stage);

#if DIAG_LOG_FUNCTION_NAME
    if (func && func[0]) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                           "%s:%d %s(): ", filename, line, func);
    } else {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                           "%s:%d: ", filename, line);
    }
#else
    offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                       "%s:%d: ", filename, line);
#endif

    /* Format user message */
    va_start(args, fmt);
    vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
    va_end(args);
    
    /* Output - use printf which should be redirected to UART */
    printf("%s\r\n", buffer);
    
    /* Also write to debug RAM location for debugger visibility */
    DIAG_WRITE_DEBUG_REG(g_current_boot_stage | (g_boot_checkpoint_count << 8));
}

/* ============================================================================
 * Boot Stage Transition
 * ============================================================================ */
void diag_set_boot_stage(boot_stage_t stage, const char *description)
{
    boot_stage_t prev_stage = g_current_boot_stage;
    g_current_boot_stage = stage;
    g_boot_checkpoint_count++;
    
    /* Write to debug RAM for debugger */
    DIAG_WRITE_DEBUG_REG(stage);
    
    /* Get stage name safely */
    const char *stage_name = "UNKNOWN";
    if (stage <= BOOT_STAGE_ELRS_INIT || stage == BOOT_STAGE_BOOT_COMPLETE) {
        if (stage < sizeof(boot_stage_names)/sizeof(boot_stage_names[0]) &&
            boot_stage_names[stage] != NULL) {
            stage_name = boot_stage_names[stage];
        }
    }
    
    printf("\r\n");
    printf("========================================\r\n");
    printf("BOOT STAGE: 0x%02X -> 0x%02X [%s]\r\n", 
           (unsigned int)prev_stage, (unsigned int)stage, stage_name);
    if (description && description[0]) {
        printf("DESC: %s\r\n", description);
    }
    printf("========================================\r\n");
}

/* ============================================================================
 * Error Logging
 * ============================================================================ */
void diag_log_error(uint32_t error_code, const char *context)
{
    g_last_error_code = error_code;
    
    printf("\r\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
    printf("ERROR at boot stage 0x%02X\r\n", (unsigned int)g_current_boot_stage);
    printf("Error code: 0x%08lX\r\n", (unsigned long)error_code);
    printf("Context: %s\r\n", context ? context : "unknown");
    printf("Checkpoint: %lu\r\n", (unsigned long)g_boot_checkpoint_count);
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
    
    /* Write error info to debug RAM */
    DIAG_WRITE_DEBUG_REG(0xFF000000 | (g_current_boot_stage << 16) | (error_code & 0xFFFF));
}

#endif /* DIAG_LOGGING_ENABLED */

/* ============================================================================
 * Silicon Labs Status Code Decoder (helpful for debugging)
 * ============================================================================ */
const char* diag_decode_sl_status(uint32_t status)
{
    switch (status) {
        case 0x0000: return "SL_STATUS_OK";
        case 0x0001: return "SL_STATUS_FAIL";
        case 0x0002: return "SL_STATUS_INVALID_STATE";
        case 0x0003: return "SL_STATUS_NOT_READY";
        case 0x0004: return "SL_STATUS_BUSY";
        case 0x0005: return "SL_STATUS_IN_PROGRESS";
        case 0x0006: return "SL_STATUS_ABORT";
        case 0x0007: return "SL_STATUS_TIMEOUT";
        case 0x0010: return "SL_STATUS_INVALID_PARAMETER";
        case 0x0011: return "SL_STATUS_NULL_POINTER";
        case 0x0012: return "SL_STATUS_INVALID_CONFIGURATION";
        case 0x0013: return "SL_STATUS_INVALID_MODE";
        case 0x0014: return "SL_STATUS_INVALID_HANDLE";
        case 0x0020: return "SL_STATUS_NO_MORE_RESOURCE";
        case 0x0021: return "SL_STATUS_ALLOCATION_FAILED";
        case 0x0022: return "SL_STATUS_EMPTY";
        case 0x0023: return "SL_STATUS_FULL";
        case 0x0024: return "SL_STATUS_WOULD_BLOCK";
        case 0x0025: return "SL_STATUS_OWNERSHIP";
        case 0x0030: return "SL_STATUS_NOT_AVAILABLE";
        case 0x0031: return "SL_STATUS_NOT_SUPPORTED";
        case 0x0032: return "SL_STATUS_INITIALIZATION";
        case 0x0033: return "SL_STATUS_NOT_INITIALIZED";
        case 0x0034: return "SL_STATUS_ALREADY_INITIALIZED";
        case 0x0035: return "SL_STATUS_DELETED";
        case 0x0040: return "SL_STATUS_NET_MQTT_NOT_CONNECTED";
        /* WiFi specific codes (0x0B00 range) */
        case 0x0B01: return "SL_STATUS_WIFI_INVALID_KEY";
        case 0x0B02: return "SL_STATUS_WIFI_INVALID_CREDENTIAL";
        case 0x0B03: return "SL_STATUS_WIFI_NO_AP_FOUND";
        case 0x0B21: return "SL_STATUS_WIFI_NOT_CONNECTED";
        case 0x0B22: return "SL_STATUS_WIFI_ALREADY_CONNECTED";
        /* Add more as needed */
        default: return "UNKNOWN_STATUS";
    }
}
