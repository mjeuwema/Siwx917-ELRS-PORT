/***************************************************************************/ /**
 * @file elrs_config.c
 * @brief ELRS Configuration Storage - NVM3 Implementation
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Implementation of persistent ELRS configuration using NVM3.
 *
 * Citation: Silicon Labs NVM3 API Documentation
 * - nvm3_initDefault(): Initialize default NVM3 instance
 * - nvm3_readData(): Read object from NVM3
 * - nvm3_writeData(): Write object to NVM3
 * - nvm3_deleteObject(): Delete object from NVM3
 *
 ******************************************************************************/

#include "elrs_config.h"
#include "rsi_debug.h"

/* NVM3 includes */
#include "nvm3.h"
#include "nvm3_default.h"

/* MD5 for binding phrase to UID conversion
 * Citation: ExpressLRS uses MD5 hash of binding phrase, first 6 bytes = UID
 */
#include "md5.h"

#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/* RAM copy of configuration */
static elrs_config_t g_config;

/* Initialization flag */
static bool g_initialized = false;

/* Debug: NVM3 init status for diagnostics (exposed via /config endpoint) */
volatile int32_t g_nvm3_debug_status = -999;  /* -999 = not yet run */
volatile int32_t g_nvm3_read_status = -999;
volatile uint32_t g_nvm3_obj_len = 0;
volatile uint16_t g_nvm3_stored_crc = 0;
volatile uint16_t g_nvm3_calc_crc = 0;

/*******************************************************************************
 * Default Configuration Values
 * 
 * Citation: ExpressLRS default values from config.h
 ******************************************************************************/
static const elrs_config_t DEFAULT_CONFIG = {
  .version          = ELRS_CONFIG_VERSION,
  .flags            = 0,  /* Not valid until explicitly set */
  
  /* UID for binding phrase "matthew" - BUILD FLAG METHOD
   * 
   * Citation: ExpressLRS build system hashes the ENTIRE build flag string:
   *   - MD5("-DMY_BINDING_PHRASE=\"matthew\"") = BE 93 67 27 D6 9C ...
   *   - First 6 bytes = UID = [190, 147, 103, 39, 214, 156]
   * 
   * This matches the TX which shows: Binding UID 190,147,103,39,214,156
   * 
   * OtaCrcInitializer = (UID[4] << 8 | UID[5]) ^ (OTA_VERSION_ID << 8)
   * LUA/WIFI METHOD (used by Configurator app):
   *   Hash "matthew" → E6:A5:BA:08:42:A5
   *   CRC init = (0x42 << 8 | 0xA5) ^ (4 << 8)
   *                   = 0x42A5 ^ 0x0400 = 0x46A5
   */
  .uid              = { 0xBE, 0x93, 0x67, 0x27, 0xD6, 0x9C },  /* TX UID for "matthew" build flag method */
  
  /* Serial settings */
  .serial_protocol  = ELRS_SERIAL_CRSF,
  .failsafe_mode    = ELRS_FAILSAFE_NO_PULSES,
  
  /* Model match disabled by default */
  .model_id         = 255,
  
  /* Telemetry */
  .force_tlm        = 0,
  .tlm_interval     = 0,
  
  /* Voltage binding disabled */
  .vbind            = 0,
  
  /* Radio defaults - dual band support for LR1121 */
  .reg_domain_low   = ELRS_DOMAIN_FCC_915,
  .reg_domain_high  = ELRS_DOMAIN_ISM_2400,
  .tx_power         = 20,   /* 20 dBm default */
  .rate_index       = 8,    /* 50Hz rate (RATE_LORA_900_50HZ) */
  
  /* WiFi defaults */
  .wifi_ssid        = "ELRS_TEST_AP",
  .wifi_password    = "elrs1234",
  .wifi_channel     = 6,
  
  /* Reserved zeros */
  .reserved         = { 0 },
  
  /* CRC will be calculated on save */
  .crc              = 0
};

/*******************************************************************************
 * CRC-16 Calculation (CCITT polynomial)
 * 
 * Citation: Standard CRC-16-CCITT implementation
 ******************************************************************************/
static uint16_t calc_crc16(const uint8_t* data, size_t len)
{
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return crc;
}

/*******************************************************************************
 * Validate Configuration CRC
 ******************************************************************************/
static bool validate_config(const elrs_config_t* config)
{
  /* Check version */
  if (config->version != ELRS_CONFIG_VERSION) {
    DEBUGOUT("[Config] Version mismatch: stored=%d, expected=%d\n",
             config->version, ELRS_CONFIG_VERSION);
    return false;
  }
  
  /* Calculate CRC over config (excluding CRC field itself) */
  size_t crc_len = sizeof(elrs_config_t) - sizeof(uint16_t);
  uint16_t calc_crc = calc_crc16((const uint8_t*)config, crc_len);
  
  if (calc_crc != config->crc) {
    DEBUGOUT("[Config] CRC mismatch: stored=0x%04X, calculated=0x%04X\n",
             config->crc, calc_crc);
    return false;
  }
  
  return true;
}

/*******************************************************************************
 * Update Configuration CRC
 ******************************************************************************/
static void update_config_crc(elrs_config_t* config)
{
  size_t crc_len = sizeof(elrs_config_t) - sizeof(uint16_t);
  config->crc = calc_crc16((const uint8_t*)config, crc_len);
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int elrs_config_init(void)
{
  Ecode_t status;
  elrs_config_t loaded_config;
  uint32_t obj_type;
  size_t obj_len;
  
  /*
   * ELRS Configuration Initialization with NVM3
   * 
   * Citation: sl_si91x_nvm3_common_flash/readme.md
   *   "This example performs wireless initialization before using NVM3 APIs 
   *    using sl_net_init(). This is done to set up NWP-M4 communication."
   * 
   * IMPORTANT: The caller (elrs_main_task) MUST call sl_net_init() BEFORE
   * calling this function. This is required for NVM3 to access the common
   * flash on SiWx917.
   * 
   * Flow:
   *   1. Initialize NVM3 default instance
   *   2. Try to read saved configuration
   *   3. Validate CRC and version
   *   4. Use saved config or fall back to defaults
   */
  
  DEBUGOUT("[Config] Initializing ELRS configuration from NVM3...\n");
  
  /***************************************************************************
   * Step 1: Initialize NVM3 default instance
   * 
   * Citation: Silicon Labs NVM3 API Documentation
   *   nvm3_initDefault() - Initialize the default NVM3 instance
   *   Must be called before any other NVM3 operations
   **************************************************************************/
  status = nvm3_initDefault();
  g_nvm3_debug_status = (int32_t)status;
  
  if (status != ECODE_NVM3_OK) {
    DEBUGOUT("[Config] WARNING: nvm3_initDefault() failed: 0x%lX\n", (unsigned long)status);
    DEBUGOUT("[Config]   Using hardcoded defaults instead.\n");
    goto use_defaults;
  }
  DEBUGOUT("[Config] NVM3 initialized successfully\n");
  
  /***************************************************************************
   * Step 2: Check if config object exists
   * 
   * Citation: Silicon Labs NVM3 API Documentation
   *   nvm3_getObjectInfo() - Get object info (type and length)
   **************************************************************************/
  status = nvm3_getObjectInfo(nvm3_defaultHandle, NVM3_KEY_ELRS_CONFIG, &obj_type, &obj_len);
  
  if (status == ECODE_NVM3_ERR_KEY_NOT_FOUND) {
    DEBUGOUT("[Config] No saved configuration found in NVM3 (first boot)\n");
    DEBUGOUT("[Config]   Will save defaults to NVM3.\n");
    goto save_and_use_defaults;
  }
  
  if (status != ECODE_NVM3_OK) {
    DEBUGOUT("[Config] WARNING: nvm3_getObjectInfo() failed: 0x%lX\n", (unsigned long)status);
    goto use_defaults;
  }
  
  if (obj_type != NVM3_OBJECTTYPE_DATA) {
    DEBUGOUT("[Config] WARNING: NVM3 object is not data type (type=%lu)\n", (unsigned long)obj_type);
    goto save_and_use_defaults;
  }
  
  g_nvm3_obj_len = (uint32_t)obj_len;
  
  if (obj_len != sizeof(elrs_config_t)) {
    DEBUGOUT("[Config] WARNING: Config size mismatch (stored=%lu, expected=%u)\n", 
             (unsigned long)obj_len, (unsigned int)sizeof(elrs_config_t));
    DEBUGOUT("[Config]   Configuration format may have changed. Resetting to defaults.\n");
    goto save_and_use_defaults;
  }
  
  /***************************************************************************
   * Step 3: Read configuration from NVM3
   * 
   * Citation: Silicon Labs NVM3 API Documentation
   *   nvm3_readData() - Read object from NVM3
   **************************************************************************/
  status = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_ELRS_CONFIG,
                         &loaded_config, sizeof(loaded_config));
  g_nvm3_read_status = (int32_t)status;
  
  if (status != ECODE_NVM3_OK) {
    DEBUGOUT("[Config] WARNING: nvm3_readData() failed: 0x%lX\n", (unsigned long)status);
    goto save_and_use_defaults;
  }
  
  /***************************************************************************
   * Step 4: Validate loaded configuration
   **************************************************************************/
  g_nvm3_stored_crc = loaded_config.crc;
  size_t crc_len = sizeof(elrs_config_t) - sizeof(uint16_t);
  g_nvm3_calc_crc = calc_crc16((const uint8_t*)&loaded_config, crc_len);
  
  if (!validate_config(&loaded_config)) {
    DEBUGOUT("[Config] WARNING: Loaded config validation failed!\n");
    goto save_and_use_defaults;
  }
  
  /***************************************************************************
   * Step 5: Use loaded configuration - SUCCESS!
   **************************************************************************/
  memcpy(&g_config, &loaded_config, sizeof(g_config));
  g_initialized = true;
  
  DEBUGOUT("[Config] Successfully loaded configuration from NVM3!\n");
  elrs_config_print();
  
  return 0;
  
  /***************************************************************************
   * Error paths: Save and/or use defaults
   **************************************************************************/
save_and_use_defaults:
  DEBUGOUT("[Config] Saving defaults to NVM3...\n");
  memcpy(&g_config, &DEFAULT_CONFIG, sizeof(g_config));
  g_config.flags |= ELRS_CONFIG_FLAG_VALID;
  update_config_crc(&g_config);
  
  /* Try to save defaults to NVM3 for next boot */
  status = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_ELRS_CONFIG,
                          &g_config, sizeof(g_config));
  if (status == ECODE_NVM3_OK) {
    DEBUGOUT("[Config] Defaults saved to NVM3 successfully\n");
  } else {
    DEBUGOUT("[Config] WARNING: Failed to save defaults: 0x%lX\n", (unsigned long)status);
  }
  
  g_initialized = true;
  DEBUGOUT("[Config] Using default configuration:\n");
  elrs_config_print();
  return 0;
  
use_defaults:
  DEBUGOUT("[Config] Using hardcoded defaults (NVM3 unavailable)\n");
  memcpy(&g_config, &DEFAULT_CONFIG, sizeof(g_config));
  g_config.flags |= ELRS_CONFIG_FLAG_VALID;
  update_config_crc(&g_config);
  g_initialized = true;
  elrs_config_print();
  return 0;
}

elrs_config_t* elrs_config_get(void)
{
  if (!g_initialized) {
    DEBUGOUT("[Config] WARNING: Config not initialized, returning defaults\n");
    memcpy(&g_config, &DEFAULT_CONFIG, sizeof(g_config));
  }
  return &g_config;
}

int elrs_config_save(void)
{
  Ecode_t status;
  
  DEBUGOUT("[Config] elrs_config_save() called\n");
  fflush(stdout);
  
  if (!g_initialized) {
    DEBUGOUT("[Config] ERROR: Config not initialized\n");
    fflush(stdout);
    return -1;
  }
  DEBUGOUT("[Config] g_initialized=true, proceeding...\n");
  fflush(stdout);
  
  /* Update CRC before saving */
  DEBUGOUT("[Config] Updating CRC...\n");
  update_config_crc(&g_config);
  DEBUGOUT("[Config] CRC updated to 0x%04X\n", g_config.crc);
  
  /* Write to NVM3 
   * 
   * Citation: Silicon Labs NVM3 API - nvm3_writeData()
   * WARNING: This can hang if NWP is busy or flash access conflicts occur!
   * 
   * Add small delay to let WiFi stack settle before flash access.
   * This helps avoid NWP/M4 flash access contention.
   */
  DEBUGOUT("[Config] Calling nvm3_writeData() - handle=%p, key=0x%lX, size=%u...\n",
           (void*)nvm3_defaultHandle, (unsigned long)NVM3_KEY_ELRS_CONFIG, 
           (unsigned int)sizeof(g_config));
  fflush(stdout);
  
  /* Small delay to let WiFi/NWP settle before flash write */
  extern void osDelay(uint32_t ticks);
  osDelay(50);  /* 50ms delay */
  
  DEBUGOUT("[Config] Executing nvm3_writeData()...\n");
  fflush(stdout);
  
  status = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_ELRS_CONFIG,
                          &g_config, sizeof(g_config));
  
  DEBUGOUT("[Config] nvm3_writeData() returned: 0x%lX\n", (unsigned long)status);
  fflush(stdout);
  
  if (status != ECODE_NVM3_OK) {
    DEBUGOUT("[Config] ERROR: nvm3_writeData failed: 0x%lX\n", (unsigned long)status);
    return -2;
  }
  
  DEBUGOUT("[Config] Configuration saved to NVM3 (CRC=0x%04X)\n", g_config.crc);
  
  /* Trigger repack if needed */
  DEBUGOUT("[Config] Checking if repack needed...\n");
  if (nvm3_repackNeeded(nvm3_defaultHandle)) {
    DEBUGOUT("[Config] NVM3 repack needed, repacking...\n");
    status = nvm3_repack(nvm3_defaultHandle);
    if (status != ECODE_NVM3_OK) {
      DEBUGOUT("[Config] WARNING: nvm3_repack failed: 0x%lX\n", (unsigned long)status);
    }
  }
  DEBUGOUT("[Config] elrs_config_save() complete, returning 0\n");
  
  return 0;
}

int elrs_config_reset(void)
{
  Ecode_t status;
  
  DEBUGOUT("[Config] Resetting to factory defaults...\n");
  
  /* Delete existing config */
  status = nvm3_deleteObject(nvm3_defaultHandle, NVM3_KEY_ELRS_CONFIG);
  if (status != ECODE_NVM3_OK && status != ECODE_NVM3_ERR_KEY_NOT_FOUND) {
    DEBUGOUT("[Config] WARNING: Failed to delete old config: 0x%lX\n", (unsigned long)status);
  }
  
  /* Reset to defaults */
  memcpy(&g_config, &DEFAULT_CONFIG, sizeof(g_config));
  g_config.flags |= ELRS_CONFIG_FLAG_VALID;
  
  /* Save defaults */
  return elrs_config_save();
}

int elrs_config_set_uid(const uint8_t uid[6])
{
  if (uid == NULL) {
    return -1;
  }
  
  memcpy(g_config.uid, uid, 6);
  
  /* Check if UID is non-zero (bound) */
  bool is_bound = false;
  for (int i = 0; i < 6; i++) {
    if (uid[i] != 0) {
      is_bound = true;
      break;
    }
  }
  
  if (is_bound) {
    g_config.flags |= ELRS_CONFIG_FLAG_BOUND;
  } else {
    g_config.flags &= ~ELRS_CONFIG_FLAG_BOUND;
  }
  
  DEBUGOUT("[Config] UID set to: %02X:%02X:%02X:%02X:%02X:%02X (bound=%d)\n",
           uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], is_bound);
  
  return 0;
}

int elrs_config_get_uid(uint8_t uid_out[6])
{
  if (uid_out == NULL) {
    return -1;
  }
  
  memcpy(uid_out, g_config.uid, 6);
  return 0;
}

bool elrs_config_is_bound(void)
{
  return (g_config.flags & ELRS_CONFIG_FLAG_BOUND) != 0;
}

int elrs_config_set_wifi(const char* ssid, const char* password, uint8_t channel)
{
  if (ssid == NULL || password == NULL) {
    return -1;
  }
  
  /* Validate lengths */
  size_t ssid_len = strlen(ssid);
  size_t pass_len = strlen(password);
  
  if (ssid_len > 32 || pass_len > 64) {
    DEBUGOUT("[Config] ERROR: WiFi credentials too long\n");
    return -2;
  }
  
  /* Copy with null termination */
  memset(g_config.wifi_ssid, 0, sizeof(g_config.wifi_ssid));
  memset(g_config.wifi_password, 0, sizeof(g_config.wifi_password));
  strncpy(g_config.wifi_ssid, ssid, 32);
  strncpy(g_config.wifi_password, password, 64);
  g_config.wifi_channel = channel;
  
  g_config.flags |= ELRS_CONFIG_FLAG_WIFI_CUSTOM;
  
  DEBUGOUT("[Config] WiFi set: SSID=%s, channel=%d\n", g_config.wifi_ssid, channel);
  
  return 0;
}

int elrs_config_to_json(char* buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size < 512) {
    return -1;
  }
  
  elrs_config_t* cfg = &g_config;
  
  /* Generate ELRS-compatible JSON
   * 
   * Citation: ExpressLRS devWIFI.cpp GetConfiguration()
   * - config: runtime binding/serial settings
   * - settings: device info and capabilities
   * 
   * Note: Added nvm3_debug section for diagnosing NVM read issues
   */
  int len = snprintf(buffer, buffer_size,
    "{"
      "\"config\":{"
        "\"uid\":[%u,%u,%u,%u,%u,%u],"
        "\"serial-protocol\":%u,"
        "\"sbus-failsafe\":%u,"
        "\"modelid\":%u,"
        "\"force-tlm\":%s,"
        "\"vbind\":%u"
      "},"
      "\"settings\":{"
        "\"product_name\":\"ELRS SiWx917 LR1121 RX\","
        "\"lua_name\":\"SiWx917 RX\","
        "\"uidtype\":\"%s\","
        "\"ssid\":\"%s\","
        "\"mode\":\"AP\","
        "\"custom_hardware\":false,"
        "\"target\":\"SIWG917Y_LR1121\","
        "\"version\":\"4.0.0-SiWx917\","
        "\"git-commit\":\"siwx917\","
        "\"module-type\":\"RX\","
        "\"radio-type\":\"LR1121\","
        "\"has_low_band\":true,"
        "\"has_high_band\":true,"
        "\"reg_domain_low\":\"%s\","
        "\"reg_domain_high\":\"%s\""
      "},"
      "\"nvm3_debug\":{"
        "\"init_status\":%ld,"
        "\"read_status\":%ld,"
        "\"obj_len\":%lu,"
        "\"expected_len\":%u,"
        "\"stored_crc\":\"0x%04X\","
        "\"calc_crc\":\"0x%04X\""
      "},"
      "\"options\":{}"
    "}",
    cfg->uid[0], cfg->uid[1], cfg->uid[2], cfg->uid[3], cfg->uid[4], cfg->uid[5],
    cfg->serial_protocol,
    cfg->failsafe_mode,
    cfg->model_id,
    cfg->force_tlm ? "true" : "false",
    cfg->vbind,
    elrs_config_is_bound() ? "Bound" : "Not Bound",
    cfg->wifi_ssid,
    (cfg->reg_domain_low == ELRS_DOMAIN_FCC_915) ? "FCC_915" : "EU_868",
    (cfg->reg_domain_high == ELRS_DOMAIN_ISM_2400) ? "ISM_2400" : "CE_2400",
    (long)g_nvm3_debug_status,
    (long)g_nvm3_read_status,
    (unsigned long)g_nvm3_obj_len,
    (unsigned int)sizeof(elrs_config_t),
    g_nvm3_stored_crc,
    g_nvm3_calc_crc
  );
  
  return len;
}

int elrs_config_from_json(const char* json, size_t json_len)
{
  if (json == NULL || json_len == 0) {
    return -1;
  }
  
  /* 
   * Parse binding phrase if present (takes priority over raw UID)
   * 
   * Citation: ExpressLRS Configurator binding phrase handling
   * - User enters binding phrase in web UI
   * - MD5 hash computed, first 6 bytes = UID
   * - Both TX and RX must use identical binding phrase
   * 
   * JSON format: {"bindingPhrase":"my secret phrase"}
   */
  const char* phrase_pos = strstr(json, "\"bindingPhrase\":");
  if (phrase_pos != NULL) {
    /* Find the opening quote of the value */
    const char* start = phrase_pos + 16;  /* strlen("\"bindingPhrase\":") */
    while (*start == ' ' || *start == '\t') start++;
    
    if (*start == '"') {
      start++;  /* Skip opening quote */
      const char* end = start;
      
      /* Find closing quote, handling escape sequences */
      while (*end != '\0' && *end != '"') {
        if (*end == '\\' && *(end + 1) != '\0') {
          end += 2;  /* Skip escaped character */
        } else {
          end++;
        }
      }
      
      /* Extract and convert binding phrase */
      size_t phrase_len = end - start;
      if (phrase_len > 0 && phrase_len < 128) {
        char phrase_buf[128];
        memcpy(phrase_buf, start, phrase_len);
        phrase_buf[phrase_len] = '\0';
        
        DEBUGOUT("[Config] Binding phrase received: \"%s\" (%u chars)\n", 
                 phrase_buf, (unsigned int)phrase_len);
        
        /* Convert binding phrase to UID using MD5 (ELRS-compatible)
         * Citation: ExpressLRS - MD5 hash of phrase, first 6 bytes = UID
         */
        uint8_t uid[6];
        elrs_md5_uid_from_phrase(phrase_buf, uid);
        
        DEBUGOUT("[Config] MD5-derived UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
        
        elrs_config_set_uid(uid);
        
        /* Skip parsing raw UID array if phrase was provided */
        goto parse_other_fields;
      }
    }
  }
  
  /* Parse UID array manually (fallback if no binding phrase) */
  const char* uid_pos = strstr(json, "\"uid\":[");
  if (uid_pos != NULL) {
    const char* bracket = uid_pos;
    while (*bracket != '\0' && *bracket != '[') bracket++;
    
    if (*bracket == '[') {
      const char* p = bracket + 1;
      uint8_t uid[6] = {0};
      int count = 0;
      
      while (count < 6 && *p != '\0' && *p != ']') {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ']' || *p == '\0') break;
        
        if (*p >= '0' && *p <= '9') {
          int val = 0;
          while (*p >= '0' && *p <= '9') {
            val = val * 10 + (*p - '0');
            p++;
          }
          if (val <= 255) {
            uid[count++] = (uint8_t)val;
          }
        } else {
          p++;
        }
        while (*p == ',' || *p == ' ' || *p == '\t') p++;
      }
      
      if (count >= 6) {
        /* Require all 6 UID bytes 
         * Citation: ELRS UID is always 6 bytes (48 bits)
         */
        elrs_config_set_uid(uid);
      } else if (count == 5) {
        /* Web UI bug: only sent 5 bytes, pad with 0 and warn */
        DEBUGOUT("[Config] WARNING: UID only has %d bytes (expected 6), padding with 0x00\n", count);
        uid[5] = 0x00;
        elrs_config_set_uid(uid);
      } else {
        DEBUGOUT("[Config] ERROR: UID has only %d bytes (need at least 5)\n", count);
      }
    }
  }

parse_other_fields:
  
  /* Parse serial-protocol */
  const char* serial_start = strstr(json, "\"serial-protocol\":");
  if (serial_start != NULL) {
    serial_start += 18;
    int serial_val;
    if (sscanf(serial_start, "%d", &serial_val) == 1) {
      g_config.serial_protocol = (uint8_t)serial_val;
    }
  }
  
  /* Parse sbus-failsafe */
  const char* failsafe_start = strstr(json, "\"sbus-failsafe\":");
  if (failsafe_start != NULL) {
    failsafe_start += 16;
  } else {
    failsafe_start = strstr(json, "\"sb-failsafe\":");
    if (failsafe_start != NULL) {
      failsafe_start += 14;
    }
  }
  if (failsafe_start != NULL) {
    int failsafe_val;
    if (sscanf(failsafe_start, "%d", &failsafe_val) == 1) {
      g_config.failsafe_mode = (uint8_t)failsafe_val;
    }
  }
  
  /* Parse modelid */
  const char* model_start = strstr(json, "\"modelid\":");
  if (model_start != NULL) {
    model_start += 10;
    int model_val;
    if (sscanf(model_start, "%d", &model_val) == 1) {
      g_config.model_id = (uint8_t)model_val;
    }
  }
  
  /* Parse force-tlm */
  const char* tlm_start = strstr(json, "\"force-tlm\":");
  if (tlm_start != NULL) {
    tlm_start += 12;
    while (*tlm_start == ' ') tlm_start++;
    
    if (strncmp(tlm_start, "true", 4) == 0) {
      g_config.force_tlm = 1;
    } else if (strncmp(tlm_start, "false", 5) == 0) {
      g_config.force_tlm = 0;
    } else {
      int tlm_val;
      if (sscanf(tlm_start, "%d", &tlm_val) == 1) {
        g_config.force_tlm = (uint8_t)tlm_val;
      }
    }
  }
  
  /* Parse vbind */
  const char* vbind_start = strstr(json, "\"force-vbind\":");
  if (vbind_start != NULL) {
    vbind_start += 14;
  } else {
    vbind_start = strstr(json, "\"vbind\":");
    if (vbind_start != NULL) {
      vbind_start += 8;
    }
  }
  if (vbind_start != NULL) {
    int vbind_val;
    if (sscanf(vbind_start, "%d", &vbind_val) == 1) {
      g_config.vbind = (uint8_t)vbind_val;
    }
  }
  
  /* Mark as valid */
  g_config.flags |= ELRS_CONFIG_FLAG_VALID;
  
  return 0;
}

void elrs_config_print(void)
{
  elrs_config_t* cfg = &g_config;
  
  DEBUGOUT("\n");
  DEBUGOUT("=== ELRS Configuration ===\n");
  DEBUGOUT("  Version:    %d\n", cfg->version);
  DEBUGOUT("  Flags:      0x%02X (valid=%d, bound=%d, wifi_custom=%d)\n",
           cfg->flags,
           (cfg->flags & ELRS_CONFIG_FLAG_VALID) ? 1 : 0,
           (cfg->flags & ELRS_CONFIG_FLAG_BOUND) ? 1 : 0,
           (cfg->flags & ELRS_CONFIG_FLAG_WIFI_CUSTOM) ? 1 : 0);
  DEBUGOUT("  UID:        %02X:%02X:%02X:%02X:%02X:%02X\n",
           cfg->uid[0], cfg->uid[1], cfg->uid[2], cfg->uid[3], cfg->uid[4], cfg->uid[5]);
  DEBUGOUT("  Serial:     %d\n", cfg->serial_protocol);
  DEBUGOUT("  Failsafe:   %d\n", cfg->failsafe_mode);
  DEBUGOUT("  Model ID:   %d\n", cfg->model_id);
  DEBUGOUT("  Force TLM:  %d\n", cfg->force_tlm);
  DEBUGOUT("  TX Power:   %d dBm\n", cfg->tx_power);
  DEBUGOUT("  Rate Index: %d\n", cfg->rate_index);
  DEBUGOUT("  WiFi SSID:  %s\n", cfg->wifi_ssid);
  DEBUGOUT("  WiFi Ch:    %d\n", cfg->wifi_channel);
  DEBUGOUT("  CRC:        0x%04X\n", cfg->crc);
  DEBUGOUT("==========================\n");
  DEBUGOUT("\n");
}
