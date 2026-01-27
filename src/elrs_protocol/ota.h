/**
 * @file ota.h
 * @brief ELRS Over-The-Air Packet Structures and Functions
 * 
 * Ported from ELRS 4.0 src/lib/OTA/OTA.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * This defines the OTA packet formats used for RF communication between
 * TX and RX. Two packet sizes are supported:
 *   - OTA4 (8 bytes): Standard packets, 14-bit CRC
 *   - OTA8 (13 bytes): Full-resolution packets, 16-bit CRC
 * 
 * CRITICAL: These structures must be bit-for-bit identical to ELRS.
 * Any deviation will cause packet rejection.
 */

#ifndef ELRS_OTA_H
#define ELRS_OTA_H

#include "elrs_platform.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "telemetry_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * OTA Packet Size Constants
 * 
 * Citation: ELRS OTA.h lines 15-18
 ******************************************************************************/

#define OTA4_PACKET_SIZE        8U
#define OTA4_CRC_CALC_LEN       6U      /* offsetof(OTA_Packet4_s, crcLow) */
#define OTA8_PACKET_SIZE        13U
#define OTA8_CRC_CALC_LEN       11U     /* offsetof(OTA_Packet8_s, crc) */

/*******************************************************************************
 * Packet Type Identifiers
 * 
 * Citation: ELRS OTA.h lines 20-26
 * These are stored in the low 2 bits of the first byte
 ******************************************************************************/

#define PACKET_TYPE_DATA        0b01    /* Generic data packet */
#define PACKET_TYPE_RCDATA      0b00    /* RC channel data (uplink) */
#define PACKET_TYPE_SYNC        0b10    /* Sync packet (uplink) */
#define PACKET_TYPE_LINKSTATS   0b00    /* Link stats (downlink) */

/**
 * Model match mask - XORed with ModelId in SYNC packet
 * Citation: ELRS OTA.h line 29
 */
#define MODELMATCH_MASK         0x3F

/*******************************************************************************
 * OTA Sync Packet Structure
 * 
 * Citation: ELRS OTA.h lines 31-42
 * Sent periodically by TX to synchronize RX
 ******************************************************************************/

typedef struct {
    uint8_t fhssIndex;              /* Current FHSS sequence index */
    uint8_t nonce;                  /* Packet nonce for CRC */
    uint8_t rfRateEnum;             /* RF rate enumeration */
    uint8_t switchEncMode   : 1;    /* Switch encoding mode */
    uint8_t newTlmRatio     : 3;    /* Telemetry ratio */
    uint8_t geminiMode      : 1;    /* Gemini (dual antenna) mode */
    uint8_t otaProtocol     : 2;    /* OTA protocol version */
    uint8_t free            : 1;    /* Reserved */
    uint8_t UID4;                   /* UID byte 4 (for binding verification) */
    uint8_t UID5;                   /* UID byte 5 */
} PACKED OTA_Sync_s;

/*******************************************************************************
 * OTA Link Statistics Structure
 * 
 * Citation: ELRS OTA.h lines 44-52
 * Sent from RX to TX in telemetry packets
 ******************************************************************************/

typedef struct {
    uint8_t uplink_RSSI_1   : 7;    /* RSSI antenna 1 (dBm * -1) */
    uint8_t antenna         : 1;    /* Active antenna */
    uint8_t uplink_RSSI_2   : 7;    /* RSSI antenna 2 */
    uint8_t modelMatch      : 1;    /* Model match status */
    uint8_t lq              : 7;    /* Link quality % */
    uint8_t trueDiversityAvailable : 1;
    int8_t  SNR;                    /* Signal-to-noise ratio */
} PACKED OTA_LinkStats_s;

/*******************************************************************************
 * 4x10-bit Channel Data Structure
 * 
 * Citation: ELRS OTA.h lines 54-56
 * Packs 4 channels into 5 bytes (4 * 10 bits = 40 bits)
 ******************************************************************************/

typedef struct {
    uint8_t raw[5];
} PACKED OTA_Channels_4x10;

/*******************************************************************************
 * Standard 4-byte OTA Packet (OTA4)
 * 
 * Citation: ELRS OTA.h lines 58-103
 * 8 bytes total, 14-bit CRC
 ******************************************************************************/

typedef struct {
    /* First byte: type in low 2 bits, high 6 bits of CRC */
    uint8_t type    : 2;
    uint8_t crcHigh : 6;
    
    union {
        /** PACKET_TYPE_RCDATA - RC channel data **/
        struct {
            OTA_Channels_4x10 ch;           /* 4x 10-bit channels */
            uint8_t switches    : 7;        /* Switch data + stubbornAck */
            uint8_t isArmed     : 1;        /* Arm status */
        } rc;
        
        /** PACKET_TYPE_SYNC - Sync packet **/
        OTA_Sync_s sync;
        
        /** PACKET_TYPE_DATA uplink - Data to RX **/
        struct {
            uint8_t packageIndex : 7;
            uint8_t stubbornAck  : 1;
            uint8_t payload[ELRS4_DATA_UL_BYTES_PER_CALL];
        } data_ul;
        
        /** PACKET_TYPE_DATA/LINKSTATS downlink - Data from RX **/
        struct {
            uint8_t packageIndex : 7;
            uint8_t stubbornAck  : 1;
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t payload[ELRS4_DATA_DL_BYTES_PER_CALL - sizeof(OTA_LinkStats_s)];
                } PACKED ul_link_stats;
                uint8_t payload[ELRS4_DATA_DL_BYTES_PER_CALL];
            };
        } data_dl;
        
        /** Airport mode data **/
        struct {
            uint8_t free  : 2;
            uint8_t count : 6;
            uint8_t payload[ELRS4_DATA_DL_BYTES_PER_CALL];
        } PACKED airport;
    };
    
    uint8_t crcLow;                         /* Low 8 bits of CRC */
} PACKED OTA_Packet4_s;

/*******************************************************************************
 * Full-Resolution 8-byte OTA Packet (OTA8)
 * 
 * Citation: ELRS OTA.h lines 105-161
 * 13 bytes total, 16-bit CRC
 ******************************************************************************/

typedef struct {
    union {
        /** PACKET_TYPE_RCDATA - Full resolution RC data **/
        struct {
            uint8_t packetType  : 2;
            uint8_t stubbornAck : 1;
            uint8_t uplinkPower : 3;        /* TX power level - 1 */
            uint8_t isHighAux   : 1;        /* true = AUX6-9, false = AUX2-5 */
            uint8_t isArmed     : 1;
            OTA_Channels_4x10 chLow;        /* CH0-CH3 */
            OTA_Channels_4x10 chHigh;       /* AUX channels */
        } PACKED rc;
        
        /** PACKET_TYPE_SYNC **/
        struct {
            uint8_t packetType;             /* Only low 2 bits used */
            OTA_Sync_s sync;
            uint8_t free[ELRS8_DATA_DL_BYTES_PER_CALL - sizeof(OTA_Sync_s)];
        } PACKED sync;
        
        /** PACKET_TYPE_DATA uplink **/
        struct {
            uint8_t packetType   : 2;
            uint8_t stubbornAck  : 1;
            uint8_t packageIndex : 5;
            uint8_t payload[ELRS8_DATA_UL_BYTES_PER_CALL];
        } data_ul;
        
        /** PACKET_TYPE_DATA/LINKSTATS downlink **/
        struct {
            uint8_t packetType   : 2;
            uint8_t stubbornAck  : 1;
            uint8_t packageIndex : 5;
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t payload[ELRS8_DATA_DL_BYTES_PER_CALL - sizeof(OTA_LinkStats_s)];
                } PACKED ul_link_stats;
                uint8_t payload[ELRS8_DATA_DL_BYTES_PER_CALL];
            };
        } PACKED data_dl;
        
        /** Airport mode data **/
        struct {
            uint8_t packetType : 2;
            uint8_t free       : 1;
            uint8_t count      : 5;
            uint8_t payload[ELRS8_DATA_DL_BYTES_PER_CALL];
        } PACKED airport;
    };
    
    uint16_t crc;                           /* 16-bit CRC, little-endian */
} PACKED OTA_Packet8_s;

/*******************************************************************************
 * Unified OTA Packet Union
 * 
 * Citation: ELRS OTA.h lines 163-168
 ******************************************************************************/

typedef union {
    OTA_Packet4_s std;                      /* Standard 4-byte packet */
    OTA_Packet8_s full;                     /* Full-resolution 8-byte packet */
    uint8_t raw[OTA8_PACKET_SIZE];          /* Raw byte access */
} PACKED OTA_Packet_s;

/*******************************************************************************
 * Switch Modes
 * 
 * Citation: ELRS OTA.h line 175
 ******************************************************************************/

typedef enum {
    smWideOr8ch     = 0,    /* Wide switches or 8-channel mode */
    smHybridOr16ch  = 1,    /* Hybrid switches or 16-channel mode */
    sm12ch          = 2,    /* 12-channel mode */
} OtaSwitchMode_e;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/** True if using full-resolution (8-byte) packets */
extern bool OtaIsFullRes;

/** Current packet nonce */
extern volatile uint8_t OtaNonce;

/** CRC initializer derived from UID */
extern uint16_t OtaCrcInitializer;

/** Current switch mode */
extern OtaSwitchMode_e OtaSwitchModeCurrent;

/** Arm status (RX only) */
#if defined(TARGET_RX)
extern bool isArmed;
#endif

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/

/**
 * @brief Update CRC initializer from UID
 * 
 * Must be called after UID is set. Combines UID bytes 4-5 with
 * OTA_VERSION_ID to create unique CRC seed.
 * 
 * Citation: ELRS OTA.cpp lines 28-39
 */
void OtaUpdateCrcInitFromUid(void);

/**
 * @brief Initialize OTA serializers for given mode and packet size
 * 
 * Sets up the appropriate pack/unpack functions and CRC calculator
 * based on switch mode and whether using standard or full-res packets.
 * 
 * Citation: ELRS OTA.cpp lines 500-549
 * 
 * @param switchMode Switch encoding mode
 * @param packetSize OTA4_PACKET_SIZE or OTA8_PACKET_SIZE
 */
void OtaUpdateSerializers(OtaSwitchMode_e switchMode, uint8_t packetSize);

/*******************************************************************************
 * CRC Function Pointer Types - Match ELRS exactly
 ******************************************************************************/

typedef bool (*ValidatePacketCrc_t)(OTA_Packet_s *otaPktPtr);
typedef void (*GeneratePacketCrc_t)(OTA_Packet_s *otaPktPtr);

/** CRC validation function pointer - set by OtaUpdateSerializers */
extern ValidatePacketCrc_t OtaValidatePacketCrc;

/** CRC generation function pointer - set by OtaUpdateSerializers */
extern GeneratePacketCrc_t OtaGeneratePacketCrc;

/*******************************************************************************
 * Channel Unpacking (RX only) - Match ELRS exactly
 ******************************************************************************/

#if defined(TARGET_RX)
typedef bool (*UnpackChannelData_t)(OTA_Packet_s const *otaPktPtr, uint32_t *channelData);

/** Channel unpacking function pointer - set by OtaUpdateSerializers */
extern UnpackChannelData_t OtaUnpackChannelData;
#endif

#ifdef __cplusplus
}
#endif

#endif /* ELRS_OTA_H */
