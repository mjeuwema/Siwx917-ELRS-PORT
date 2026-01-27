/**
 * @file crsf_protocol.h
 * @brief CRSF (Crossfire) Protocol Definitions
 * 
 * Ported from ELRS 4.0 src/lib/CrsfProtocol/crsf_protocol.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * This defines the CRSF protocol used for:
 *   - FC <-> RX serial communication
 *   - Telemetry frame formats
 *   - Channel data encoding
 */

#ifndef ELRS_CRSF_PROTOCOL_H
#define ELRS_CRSF_PROTOCOL_H

#include "elrs_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * CRSF Protocol Constants
 ******************************************************************************/

/** CRSF CRC polynomial */
#define CRSF_CRC_POLY                   0xD5

/** CRSF sync byte - start of every frame */
#define CRSF_SYNC_BYTE                  0xC8

/** Packet length limits */
#define CRSF_MIN_PACKET_LEN             4
#define CRSF_MAX_PACKET_LEN             64
#define CRSF_PAYLOAD_SIZE_MAX           62
#define CRSF_FRAME_NOT_COUNTED_BYTES    2
#define CRSF_FRAME_SIZE(payload_size)   ((payload_size) + 2)
#define CRSF_EXT_FRAME_SIZE(payload)    (CRSF_FRAME_SIZE(payload) + 2)
#define CRSF_FRAME_SIZE_MAX             (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_NOT_COUNTED_BYTES)
#define CRSF_FRAME_CRC_SIZE             1
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC  4

/*******************************************************************************
 * Channel Value Definitions
 * 
 * Citation: ELRS crsf_protocol.h lines 11-24
 ******************************************************************************/

/* Failsafe microsecond values */
#define CHANNEL_VALUE_FS_US_MIN         476     /* Stretch failsafe min */
#define CHANNEL_VALUE_FS_US_ELIMITS_MIN 880     /* E.Limits failsafe min */
#define CHANNEL_VALUE_FS_US_MID         1500    /* Center */
#define CHANNEL_VALUE_FS_US_ELIMITS_MAX 2120    /* E.Limits failsafe max */
#define CHANNEL_VALUE_FS_US_MAX         2523    /* Stretch failsafe max */

/* CRSF channel values (11-bit, 0-2047 range) */
#define CRSF_CHANNEL_VALUE_EXT_MIN      0       /* 880us with E.Limits (-121.1%) */
#define CRSF_CHANNEL_VALUE_MIN          172     /* 987us */
#define CRSF_CHANNEL_VALUE_1000         191     /* 1000us */
#define CRSF_CHANNEL_VALUE_MID          992     /* 1500us center */
#define CRSF_CHANNEL_VALUE_2000         1792    /* 2000us */
#define CRSF_CHANNEL_VALUE_MAX          1811    /* 2012us */
#define CRSF_CHANNEL_VALUE_EXT_MAX      1984    /* 2120us with E.Limits (+121.1%) */
#define CRSF_CHANNEL_VALUE_UNSET        0xFFFF  /* Internal: no value received */

/*******************************************************************************
 * CRSF Frame Types
 * 
 * Citation: ELRS crsf_protocol.h lines 52-87
 ******************************************************************************/

typedef enum {
    /* Standard frames */
    CRSF_FRAMETYPE_GPS                      = 0x02,
    CRSF_FRAMETYPE_VARIO                    = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR           = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE            = 0x09,
    CRSF_FRAMETYPE_AIRSPEED                 = 0x0A,
    CRSF_FRAMETYPE_HEARTBEAT                = 0x0B,
    CRSF_FRAMETYPE_RPM                      = 0x0C,
    CRSF_FRAMETYPE_TEMP                     = 0x0D,
    CRSF_FRAMETYPE_CELLS                    = 0x0E,
    CRSF_FRAMETYPE_LINK_STATISTICS          = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED       = 0x16,
    CRSF_FRAMETYPE_ATTITUDE                 = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE              = 0x21,
    
    /* Extended header frames (0x28 to 0x96) */
    CRSF_FRAMETYPE_DEVICE_PING              = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO              = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ           = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE          = 0x2D,
    CRSF_FRAMETYPE_ELRS_STATUS              = 0x2E,
    CRSF_FRAMETYPE_COMMAND                  = 0x32,
    CRSF_FRAMETYPE_HANDSET                  = 0x3A,
    
    /* KISS frames */
    CRSF_FRAMETYPE_KISS_REQ                 = 0x78,
    CRSF_FRAMETYPE_KISS_RESP                = 0x79,
    
    /* MSP frames */
    CRSF_FRAMETYPE_MSP_REQ                  = 0x7A,
    CRSF_FRAMETYPE_MSP_RESP                 = 0x7B,
    CRSF_FRAMETYPE_MSP_WRITE                = 0x7C,
    
    /* Ardupilot */
    CRSF_FRAMETYPE_ARDUPILOT_RESP           = 0x80,
} crsf_frame_type_e;

/*******************************************************************************
 * CRSF Addresses
 * 
 * Citation: ELRS crsf_protocol.h lines 105-122
 ******************************************************************************/

typedef enum {
    CRSF_ADDRESS_BROADCAST              = 0x00,
    CRSF_ADDRESS_USB                    = 0x10,
    CRSF_ADDRESS_BLUETOOTH_WIFI         = 0x12,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO       = 0x80,
    CRSF_ADDRESS_RESERVED1              = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR         = 0xC0,
    CRSF_ADDRESS_GPS                    = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX           = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER      = 0xC8,
    CRSF_ADDRESS_RESERVED2              = 0xCA,
    CRSF_ADDRESS_RACE_TAG               = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER      = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER          = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER       = 0xEE,
    CRSF_ADDRESS_ELRS_LUA               = 0xEF,
} crsf_addr_e;

/*******************************************************************************
 * CRSF Commands
 ******************************************************************************/

typedef enum {
    CRSF_COMMAND_SUBCMD_RX              = 0x10,
} crsf_command_e;

typedef enum {
    CRSF_COMMAND_SUBCMD_RX_BIND         = 0x01,
    CRSF_COMMAND_MODEL_SELECT_ID        = 0x05,
    CRSF_HANDSET_SUBCMD_TIMING          = 0x10,
} crsf_subcommand_e;

/*******************************************************************************
 * CRSF Frame Structures
 ******************************************************************************/

/**
 * Standard CRSF header
 * Citation: ELRS crsf_protocol.h lines 154-160
 */
typedef struct {
    uint8_t sync_byte;          /* Always CRSF_SYNC_BYTE (0xC8) */
    uint8_t frame_size;         /* Payload size + 2 (type + CRC) */
    uint8_t type;               /* Frame type */
    uint8_t payload[0];         /* Variable length payload */
} PACKED crsf_header_t;

/**
 * Extended CRSF header (for types 0x28-0x96)
 * Citation: ELRS crsf_protocol.h lines 166-176
 */
typedef struct {
    uint8_t device_addr;        /* Device address (sync byte position) */
    uint8_t frame_size;
    uint8_t type;
    uint8_t dest_addr;          /* Destination address */
    uint8_t orig_addr;          /* Origin address */
    uint8_t payload[0];
} PACKED crsf_ext_header_t;

/**
 * Packed channel data - 16 channels x 11 bits = 22 bytes
 * Citation: ELRS crsf_protocol.h lines 181-199
 */
typedef struct {
    unsigned ch0  : 11;
    unsigned ch1  : 11;
    unsigned ch2  : 11;
    unsigned ch3  : 11;
    unsigned ch4  : 11;
    unsigned ch5  : 11;
    unsigned ch6  : 11;
    unsigned ch7  : 11;
    unsigned ch8  : 11;
    unsigned ch9  : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

/**
 * Link statistics payload
 * Citation: ELRS crsf_protocol.h lines 379-391
 */
typedef struct {
    uint8_t uplink_RSSI_1;          /* dBm * -1 */
    uint8_t uplink_RSSI_2;          /* dBm * -1 */
    uint8_t uplink_Link_quality;    /* % */
    int8_t  uplink_SNR;             /* dB */
    uint8_t active_antenna;         /* 0 = ant1, 1 = ant2 */
    uint8_t rf_Mode;                /* Rate enum */
    uint8_t uplink_TX_Power;        /* Power enum */
    uint8_t downlink_RSSI_1;        /* dBm * -1 */
    uint8_t downlink_Link_quality;  /* % */
    int8_t  downlink_SNR;           /* dB */
} PACKED crsf_link_stats_t;

/**
 * Battery sensor payload
 * Citation: ELRS crsf_protocol.h lines 271-277
 */
typedef struct {
    unsigned voltage   : 16;    /* mV * 100 BigEndian */
    unsigned current   : 16;    /* mA * 100 BigEndian */
    unsigned capacity  : 24;    /* mAh */
    unsigned remaining : 8;     /* % */
} PACKED crsf_sensor_battery_t;

/**
 * GPS sensor payload
 * Citation: ELRS crsf_protocol.h lines 338-346
 */
typedef struct {
    int32_t  latitude;          /* degree / 10,000,000 */
    int32_t  longitude;         /* degree / 10,000,000 */
    uint16_t groundspeed;       /* km/h / 10 */
    uint16_t gps_heading;       /* degree / 100 */
    uint16_t altitude;          /* meter - 1000m offset */
    uint8_t  satellites;        /* count */
} PACKED crsf_sensor_gps_t;

/**
 * Attitude sensor payload
 * Citation: ELRS crsf_protocol.h lines 349-354
 */
typedef struct {
    int16_t pitch;              /* radians * 10000 */
    int16_t roll;               /* radians * 10000 */
    int16_t yaw;                /* radians * 10000 */
} PACKED crsf_sensor_attitude_t;

/*******************************************************************************
 * Channel Conversion Functions
 * 
 * Citation: ELRS crsf_protocol.h lines 398-482
 ******************************************************************************/

/**
 * @brief Map value from one range to another
 */
static inline uint16_t fmap(uint16_t x, uint16_t in_min, uint16_t in_max, 
                            uint16_t out_min, uint16_t out_max)
{
    int32_t result = ((int32_t)(x - in_min) * (out_max - out_min) * 2 / 
                      (in_max - in_min) + out_min * 2 + 1) / 2;
    return result < 0 ? 0 : (result > 65535 ? 65535 : result);
}

/**
 * @brief Convert CRSF value to microseconds (988-2012)
 */
static inline uint16_t CRSF_to_US(uint16_t val)
{
    return fmap(val, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 988, 2012);
}

/**
 * @brief Convert 10-bit value to CRSF
 */
static inline uint16_t UINT10_to_CRSF(uint16_t val)
{
    return fmap(val, 0, 1023, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

/**
 * @brief Convert CRSF to 10-bit value
 */
static inline uint16_t CRSF_to_UINT10(uint16_t val)
{
    return fmap(val, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 1023);
}

/**
 * @brief Convert 0-max to CRSF 1000-2000us range
 */
static inline uint16_t N_to_CRSF(uint16_t val, uint16_t max)
{
    return val * (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000) / max + 
           CRSF_CHANNEL_VALUE_1000;
}

/**
 * @brief Convert CRSF to 0-(cnt-1) range
 */
static inline uint16_t CRSF_to_N(uint16_t val, uint16_t cnt)
{
    if (val <= CRSF_CHANNEL_VALUE_1000)
        return 0;
    if (val >= CRSF_CHANNEL_VALUE_2000)
        return cnt - 1;
    return (val - CRSF_CHANNEL_VALUE_1000) * cnt / 
           (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000 + 1);
}

/**
 * @brief Convert CRSF to 3-bit switch value (0-5, 7=center)
 */
static inline uint8_t CRSF_to_SWITCH3b(uint16_t ch)
{
    const uint16_t CHANNEL_BIN_COUNT = 6;
    const uint16_t CHANNEL_BIN_SIZE = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / 
                                       CHANNEL_BIN_COUNT;
    
    /* Middle position returns special value 7 */
    if (ch < (CRSF_CHANNEL_VALUE_MID - CHANNEL_BIN_SIZE/4) ||
        ch > (CRSF_CHANNEL_VALUE_MID + CHANNEL_BIN_SIZE/4))
    {
        return CRSF_to_N(ch, CHANNEL_BIN_COUNT);
    }
    return 7;
}

/**
 * @brief Convert 3-bit switch value to CRSF
 */
static inline uint16_t SWITCH3b_to_CRSF(uint16_t val)
{
    switch (val)
    {
        case 0:  return CRSF_CHANNEL_VALUE_1000;
        case 5:  return CRSF_CHANNEL_VALUE_2000;
        case 6:
        case 7:  return CRSF_CHANNEL_VALUE_MID;
        default: return val * 240 + 391;  /* 150us spacing from 1275us */
    }
}

/**
 * @brief Convert CRSF to binary (0/1)
 */
static inline uint8_t CRSF_to_BIT(uint16_t val)
{
    return (val > CRSF_CHANNEL_VALUE_MID) ? 1 : 0;
}

/**
 * @brief Convert binary to CRSF
 */
static inline uint16_t BIT_to_CRSF(uint8_t val)
{
    return val ? CRSF_CHANNEL_VALUE_2000 : CRSF_CHANNEL_VALUE_1000;
}

#ifdef __cplusplus
}
#endif

#endif /* ELRS_CRSF_PROTOCOL_H */
