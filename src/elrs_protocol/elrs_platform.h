/**
 * @file elrs_platform.h
 * @brief Platform compatibility layer for ELRS protocol on SiW917
 * 
 * This header provides platform-specific definitions to allow ELRS protocol
 * code (originally written for ESP32/Arduino) to compile on SiW917.
 * 
 * ELRS 4.0 Protocol Port - SiW917 Target
 */

#ifndef ELRS_PLATFORM_H
#define ELRS_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*******************************************************************************
 * ESP32/Arduino Macro Replacements
 ******************************************************************************/

/**
 * ICACHE_RAM_ATTR - ESP8266/ESP32 attribute to place function in IRAM
 * On SiW917, we don't need this - functions run from flash or can be
 * placed in RAM via linker script if needed for timing-critical code.
 */
#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/**
 * PACKED - Ensure struct packing with no padding
 * GCC attribute for both platforms
 */
#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

/**
 * WORD_ALIGNED_ATTR - 4-byte alignment for DMA buffers
 */
#ifndef WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif

/**
 * WORD_PADDED - Pad size to 4-byte boundary
 */
#ifndef WORD_PADDED
#define WORD_PADDED(size) (((size)+3) & ~3)
#endif

/*******************************************************************************
 * Platform Detection
 ******************************************************************************/

/* Define that we're targeting SiW917 */
#define TARGET_SIW917       1
#define PLATFORM_SIW917     1

/* This is a receiver target */
#define TARGET_RX           1

/* Radio type - LR1121 Sub-GHz */
#define RADIO_LR1121        1

/*******************************************************************************
 * Regulatory Domain Configuration
 * 
 * Define one of these based on your region:
 *   Regulatory_Domain_FCC_915  - USA 915 MHz
 *   Regulatory_Domain_EU_868   - Europe 868 MHz
 *   Regulatory_Domain_AU_915   - Australia 915 MHz
 *   Regulatory_Domain_IN_866   - India 866 MHz
 *   Regulatory_Domain_AU_433   - Australia 433 MHz
 *   Regulatory_Domain_EU_433   - Europe 433 MHz
 *   Regulatory_Domain_US_433   - USA 433 MHz
 ******************************************************************************/
#ifndef Regulatory_Domain_FCC_915
#define Regulatory_Domain_FCC_915   1
#endif

/*******************************************************************************
 * OTA Version - Must match TX for compatibility
 * 
 * From ELRS targets.h - Used to XOR with OtaCrcInitializer and macSeed
 * to reduce compatibility with previous versions.
 ******************************************************************************/
#define OTA_VERSION_ID      4

/*******************************************************************************
 * LR1121 Radio Constants
 * 
 * Citation: LR1121 datasheet section 13.1.1
 * For LR1121, frequencies are stored directly in Hz (not register values)
 ******************************************************************************/
#define FREQ_STEP           1       /* LR1121 uses Hz directly */

/* 
 * FreqCorrectionMax for LR1121 - maximum frequency correction in Hz
 * Citation: ELRS FHSS.h line 9 - TODO needs verification for LR1121
 */
#define FreqCorrectionMax   ((int32_t)(100000/FREQ_STEP))
#define FreqCorrectionMin   (-FreqCorrectionMax)

/*******************************************************************************
 * Utility Functions / Macros
 ******************************************************************************/

/**
 * constrain - Limit value to range [low, high]
 */
#ifndef constrain
#define constrain(val, low, high) ((val) < (low) ? (low) : ((val) > (high) ? (high) : (val)))
#endif

/**
 * bit - Return value with bit n set
 */
#ifndef bit
#define bit(n) (1UL << (n))
#endif

/**
 * min/max macros
 */
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

/*******************************************************************************
 * Debug/Logging Macros
 * 
 * These replace ELRS logging.h macros. Implement as needed for your
 * debug output (UART, RTT, etc.)
 ******************************************************************************/

/* Uncomment to enable debug output */
// #define ELRS_DEBUG_ENABLE

#ifdef ELRS_DEBUG_ENABLE
    #include <stdio.h>
    #define DBGLN(fmt, ...)     printf(fmt "\n", ##__VA_ARGS__)
    #define DBG(fmt, ...)       printf(fmt, ##__VA_ARGS__)
    #define DBGCR               printf("\n")
#else
    #define DBGLN(fmt, ...)     ((void)0)
    #define DBG(fmt, ...)       ((void)0)
    #define DBGCR               ((void)0)
#endif

/*******************************************************************************
 * UID (Unique Identifier) - Binding Phrase
 * 
 * The 6-byte UID is derived from the binding phrase and must match TX.
 * This is declared extern and defined in elrs_config.c
 ******************************************************************************/
#define UID_LEN             6
extern uint8_t UID[UID_LEN];

/*******************************************************************************
 * Firmware Options Structure (Simplified for RX)
 * 
 * This is a simplified version of ELRS options.h for receiver-only use
 ******************************************************************************/
typedef struct {
    uint8_t     domain;         /* Regulatory domain index */
    uint8_t     hasUID;         /* 1 if UID is set from binding phrase */
    uint8_t     uid[6];         /* UID derived from binding phrase */
    uint32_t    uart_baud;      /* CRSF UART baud rate */
    bool        lock_on_first_connection;
    bool        is_airport;     /* Airport mode enabled */
} elrs_options_t;

extern elrs_options_t firmwareOptions;

/*******************************************************************************
 * Link Statistics Structure
 * 
 * Global link statistics updated by RX and reported to TX via telemetry
 ******************************************************************************/
typedef struct {
    uint8_t     uplink_RSSI_1;
    uint8_t     uplink_RSSI_2;
    uint8_t     uplink_Link_quality;
    int8_t      uplink_SNR;
    uint8_t     active_antenna;
    uint8_t     rf_Mode;
    uint8_t     uplink_TX_Power;
    uint8_t     downlink_RSSI_1;
    uint8_t     downlink_Link_quality;
    int8_t      downlink_SNR;
} elrs_link_stats_t;

extern elrs_link_stats_t linkStats;

#ifdef __cplusplus
}
#endif

#endif /* ELRS_PLATFORM_H */
