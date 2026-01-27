/***************************************************************************/ /**
 * @file elrs_config.h
 * @brief ELRS Configuration Storage - Persistent settings via NVM3
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Persistent configuration storage for ExpressLRS settings using NVM3.
 * Settings survive power cycles and firmware updates.
 *
 * Citation: Silicon Labs NVM3 API Documentation
 * - https://docs.silabs.com/platform-security/latest/nvm3
 * - nvm3_writeData/nvm3_readData for arbitrary data storage
 * - NVM3 default instance auto-initialized by SDK
 *
 * Citation: ExpressLRS src/lib/ELRS_CONFIG
 * - UID: 6-byte binding phrase hash
 * - Regulatory domain: FCC_915, ISM_2400, etc.
 * - Rate index, power level, telemetry ratio
 *
 ******************************************************************************/

#ifndef ELRS_CONFIG_H
#define ELRS_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration Version
 * 
 * Increment this when the config structure changes to trigger migration/reset.
 ******************************************************************************/
#define ELRS_CONFIG_VERSION      16  /* v16: Use TX UID BE:93:67:27:D6:9C for "matthew" build flag method */

/*******************************************************************************
 * NVM3 Key Definitions
 * 
 * Citation: NVM3 Documentation
 * - Keys 0x00000 to 0x0FFFF are reserved for user application
 * - Each key stores one NVM3 object
 ******************************************************************************/
#define NVM3_KEY_ELRS_CONFIG     0x00001   /* Main configuration structure */
#define NVM3_KEY_ELRS_UID        0x00002   /* Binding UID (6 bytes) */
#define NVM3_KEY_ELRS_WIFI       0x00003   /* WiFi settings */
#define NVM3_KEY_ELRS_RADIO      0x00004   /* Radio parameters */

/*******************************************************************************
 * Serial Protocol Options
 * 
 * Citation: ExpressLRS common.h - SERIAL_PROTOCOL enum
 ******************************************************************************/
typedef enum {
  ELRS_SERIAL_CRSF        = 0,   /* Crossfire protocol (default) */
  ELRS_SERIAL_INVERTED    = 1,   /* Inverted CRSF */
  ELRS_SERIAL_SBUS        = 2,   /* SBUS output */
  ELRS_SERIAL_SUMD        = 3,   /* SUMD output */
  ELRS_SERIAL_DJI_RS2_PRO = 4,   /* DJI RS2 Pro gimbal */
  ELRS_SERIAL_HOTT_TLM    = 5,   /* HoTT telemetry */
  ELRS_SERIAL_MAVLINK     = 6,   /* MAVLink */
} elrs_serial_protocol_t;

/*******************************************************************************
 * Failsafe Mode Options
 * 
 * Citation: ExpressLRS config.h - SBUS_FAILSAFE enum
 ******************************************************************************/
typedef enum {
  ELRS_FAILSAFE_NO_PULSES = 0,   /* Stop sending pulses */
  ELRS_FAILSAFE_LAST      = 1,   /* Hold last received values */
  ELRS_FAILSAFE_SET       = 2,   /* Use configured failsafe values */
} elrs_failsafe_mode_t;

/*******************************************************************************
 * Regulatory Domain Options
 * 
 * Citation: ExpressLRS common.h - RADIO_DOMAIN enum
 ******************************************************************************/
typedef enum {
  ELRS_DOMAIN_AU_915       = 0,   /* Australia 915 MHz */
  ELRS_DOMAIN_EU_868       = 1,   /* Europe 868 MHz */
  ELRS_DOMAIN_IN_866       = 2,   /* India 866 MHz */
  ELRS_DOMAIN_AU_433       = 3,   /* Australia 433 MHz */
  ELRS_DOMAIN_EU_433       = 4,   /* Europe 433 MHz */
  ELRS_DOMAIN_FCC_915      = 5,   /* FCC 915 MHz (USA) */
  ELRS_DOMAIN_ISM_2400     = 6,   /* 2.4 GHz ISM band */
  ELRS_DOMAIN_CE_2400      = 7,   /* 2.4 GHz CE (Europe) */
} elrs_reg_domain_t;

/*******************************************************************************
 * Main ELRS Configuration Structure
 * 
 * This structure is stored in NVM3 and persists across power cycles.
 * Total size must be <= NVM3_DEFAULT_MAX_OBJECT_SIZE (254 bytes)
 ******************************************************************************/
typedef struct __attribute__((packed)) {
  /* Header */
  uint8_t  version;              /* Config version for migration */
  uint8_t  flags;                /* Status flags */
  
  /* Binding UID - 6 bytes derived from binding phrase */
  uint8_t  uid[6];               /* Binding UID */
  
  /* Serial/Output Settings */
  uint8_t  serial_protocol;      /* elrs_serial_protocol_t */
  uint8_t  failsafe_mode;        /* elrs_failsafe_mode_t */
  
  /* Model Match */
  uint8_t  model_id;             /* Model match ID (0-63, 255=disabled) */
  
  /* Telemetry */
  uint8_t  force_tlm;            /* Force telemetry on */
  uint8_t  tlm_interval;         /* Telemetry interval ratio */
  
  /* Voltage-based binding */
  uint8_t  vbind;                /* Voltage binding threshold */
  
  /* Radio Settings */
  uint8_t  reg_domain_low;       /* Low-band regulatory domain */
  uint8_t  reg_domain_high;      /* High-band regulatory domain */
  int8_t   tx_power;             /* TX power in dBm */
  uint8_t  rate_index;           /* Packet rate index */
  
  /* WiFi Settings */
  char     wifi_ssid[33];        /* WiFi AP SSID (max 32 chars + null) */
  char     wifi_password[65];    /* WiFi password (max 64 chars + null) */
  uint8_t  wifi_channel;         /* WiFi channel (1-13) */
  
  /* Reserved for future use */
  uint8_t  reserved[32];
  
  /* CRC for validation */
  uint16_t crc;                  /* CRC-16 of config data */
  
} elrs_config_t;

/* Static assert to ensure config fits in NVM3 max object size */
_Static_assert(sizeof(elrs_config_t) <= 254, "elrs_config_t exceeds NVM3 max object size");

/*******************************************************************************
 * Configuration Flags
 ******************************************************************************/
#define ELRS_CONFIG_FLAG_VALID       (1 << 0)  /* Config has been initialized */
#define ELRS_CONFIG_FLAG_BOUND       (1 << 1)  /* Device is bound */
#define ELRS_CONFIG_FLAG_WIFI_CUSTOM (1 << 2)  /* Custom WiFi settings */

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize the ELRS configuration module
 * 
 * Opens NVM3 default instance and loads saved configuration.
 * If no valid config exists, initializes with defaults.
 * 
 * @return 0 on success, negative error code on failure
 */
int elrs_config_init(void);

/**
 * @brief Get pointer to the current configuration
 * 
 * Returns pointer to RAM copy of configuration.
 * Changes to this structure are NOT automatically saved.
 * Call elrs_config_save() to persist changes.
 * 
 * @return Pointer to current configuration
 */
elrs_config_t* elrs_config_get(void);

/**
 * @brief Save current configuration to NVM3
 * 
 * Writes the RAM configuration to flash storage.
 * Automatically updates CRC before saving.
 * 
 * @return 0 on success, negative error code on failure
 */
int elrs_config_save(void);

/**
 * @brief Reset configuration to factory defaults
 * 
 * Erases saved configuration and reinitializes with defaults.
 * Automatically saves to NVM3.
 * 
 * @return 0 on success, negative error code on failure
 */
int elrs_config_reset(void);

/**
 * @brief Set binding UID
 * 
 * @param uid 6-byte UID array
 * @return 0 on success, negative error code on failure
 */
int elrs_config_set_uid(const uint8_t uid[6]);

/**
 * @brief Get binding UID
 * 
 * @param uid_out 6-byte buffer to receive UID
 * @return 0 on success, negative error code on failure
 */
int elrs_config_get_uid(uint8_t uid_out[6]);

/**
 * @brief Check if device is bound
 * 
 * @return true if bound, false otherwise
 */
bool elrs_config_is_bound(void);

/**
 * @brief Set WiFi credentials
 * 
 * @param ssid WiFi SSID (max 32 chars)
 * @param password WiFi password (max 64 chars)
 * @param channel WiFi channel (1-13, 0 for auto)
 * @return 0 on success, negative error code on failure
 */
int elrs_config_set_wifi(const char* ssid, const char* password, uint8_t channel);

/**
 * @brief Export configuration as JSON string
 * 
 * Generates JSON representation suitable for HTTP /config endpoint.
 * 
 * @param buffer Output buffer for JSON string
 * @param buffer_size Size of output buffer
 * @return Length of JSON string, or negative error code
 */
int elrs_config_to_json(char* buffer, size_t buffer_size);

/**
 * @brief Import configuration from JSON string
 * 
 * Parses JSON and updates configuration.
 * Does NOT automatically save - call elrs_config_save() after.
 * 
 * @param json JSON string to parse
 * @param json_len Length of JSON string
 * @return 0 on success, negative error code on failure
 */
int elrs_config_from_json(const char* json, size_t json_len);

/**
 * @brief Print current configuration to debug output
 */
void elrs_config_print(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_CONFIG_H */
