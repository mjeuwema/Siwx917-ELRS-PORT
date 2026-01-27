/**
 * @file telemetry_protocol.h
 * @brief ELRS Telemetry Protocol Constants
 * 
 * Ported from ELRS 4.0 src/lib/TelemetryProtocol/telemetry_protocol.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * Defines telemetry data chunk sizes for OTA packets.
 */

#ifndef ELRS_TELEMETRY_PROTOCOL_H
#define ELRS_TELEMETRY_PROTOCOL_H

#include "elrs_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Telemetry Data Size Constants
 * 
 * Citation: ELRS telemetry_protocol.h lines 3-15
 * 
 * These define how much telemetry data fits in each OTA packet type.
 ******************************************************************************/

/** 
 * 4-byte (standard) OTA packet downlink telemetry
 * Shift bits for package index
 */
#define ELRS4_DATA_DL_SHIFT             1
#define ELRS4_DATA_DL_BYTES_PER_CALL    5
#define ELRS4_DATA_DL_MAX_PACKAGES      (255 >> ELRS4_DATA_DL_SHIFT)

/**
 * 8-byte (fullres) OTA packet downlink telemetry
 */
#define ELRS8_DATA_DL_BYTES_PER_CALL    10
#define ELRS8_DATA_DL_SHIFT             3
#define ELRS8_DATA_DL_MAX_PACKAGES      (255 >> ELRS8_DATA_DL_SHIFT)

/**
 * Uplink data bytes per call
 */
#define ELRS4_DATA_UL_BYTES_PER_CALL    5
#define ELRS8_DATA_UL_BYTES_PER_CALL    10

/**
 * Uplink buffer size
 */
#define ELRS_DATA_UL_BUFFER             65
#define ELRS_MSP_MAX_PACKAGES           ((ELRS_DATA_UL_BUFFER / ELRS4_DATA_UL_BYTES_PER_CALL) + 1)

/**
 * Airport mode buffer length
 */
#define AP_MAX_BUF_LEN                  64

/*******************************************************************************
 * Telemetry Ratio Enumeration
 * 
 * Citation: ELRS common.h - TLM_RATIO enum
 * 
 * Defines the ratio of telemetry packets to total packets.
 * E.g., TLM_RATIO_1_64 means 1 telemetry packet every 64 packets.
 ******************************************************************************/

typedef enum {
    TLM_RATIO_NO_TLM    = 0,
    TLM_RATIO_1_2       = 2,
    TLM_RATIO_1_4       = 4,
    TLM_RATIO_1_8       = 8,
    TLM_RATIO_1_16      = 16,
    TLM_RATIO_1_32      = 32,
    TLM_RATIO_1_64      = 64,
    TLM_RATIO_1_128     = 128,
} elrs_tlm_ratio_e;

#ifdef __cplusplus
}
#endif

#endif /* ELRS_TELEMETRY_PROTOCOL_H */
