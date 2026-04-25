/**
 * @file options.cpp
 * @brief Firmware options implementation for ELRS on SiW917
 *
 * Provides configuration storage and initialization for the receiver.
 */

#include "options.h"
#include "logging.h"
#include <string.h>

// NVM3 persistent config integration
extern "C" {
#include "elrs_config.h"
}

// Frequency domain enum (matches upstream OTA.h)
// Defined here before use in firmwareOptions initializer
enum : uint8_t {
    AU915 = 0,
    FCC915 = 1,
    EU868 = 2,
    IN866 = 3,
    AU433 = 4,
    EU433 = 5,
    US433 = 6,
    US433W = 7,
    ISM2G4 = 8,
    CE2G4 = 9,
};

// Target identification
const unsigned char target_name[] = "SiW917_ELRS_RX";
const uint8_t target_name_size = sizeof(target_name) - 1;
const char commit[] = "dev";
const char version[] = "4.0.0";

// WiFi configuration defaults (not used on SiW917 in same way as ESP)
const char *wifi_hostname = "elrs_siw917";
const char *wifi_ap_ssid = "ExpressLRS RX";
const char *wifi_ap_password = "expresslrs";
const char *wifi_ap_address = "10.0.0.1";

// Device name storage
char device_name[ELRSOPTS_DEVICENAME_SIZE] = "SiW917 RX";
char product_name[ELRSOPTS_PRODUCTNAME_SIZE] = "ExpressLRS SiW917";
uint32_t logo_image = 0;

// Firmware options with defaults
firmware_options_t firmwareOptions = {
    ._magic_ = {'E', 'L', 'R', 'S', 'O', 'P', 'T', 'S'},
    ._version_ = 1,
    .domain = ISM2G4,  // Force 2.4GHz startup for SiW917/LR1121 bring-up
    .hasUID = 0,
    .uid = {0, 0, 0, 0, 0, 0},
    .flash_discriminator = 0,
    .fan_min_runtime = 0,
    .wifi_auto_on_interval = -1,  // Disabled
    .home_wifi_ssid = "",
    .home_wifi_password = "",
#if defined(TARGET_RX)
    .uart_baud = 420000,
    ._unused1 = false,
    .lock_on_first_connection = true,
    .dji_permanently_armed = false,
    .is_airport = false,
#endif
};

bool options_init()
{
    // Keep runtime behavior on 2.4GHz for bring-up so we avoid the 900MHz path.
    firmwareOptions.domain = ISM2G4;

    // Load UID from NVM3 persistent config
    // elrs_config_init() must be called before this function
    elrs_config_t* cfg = elrs_config_get();
    
    if (cfg != nullptr) {
        // Copy UID from persistent config
        memcpy(firmwareOptions.uid, cfg->uid, 6);
        firmwareOptions.hasUID = 1;
        
        DBGLN("Loaded UID from NVM3: %02X:%02X:%02X:%02X:%02X:%02X",
              firmwareOptions.uid[0], firmwareOptions.uid[1],
              firmwareOptions.uid[2], firmwareOptions.uid[3],
              firmwareOptions.uid[4], firmwareOptions.uid[5]);
    } else {
        // Fallback to hardcoded UID if config not available
        DBGLN("WARNING: Config not available, using hardcoded UID");
        firmwareOptions.uid[0] = 0xBE;
        firmwareOptions.uid[1] = 0x93;
        firmwareOptions.uid[2] = 0x67;
        firmwareOptions.uid[3] = 0x27;
        firmwareOptions.uid[4] = 0xD6;
        firmwareOptions.uid[5] = 0x9C;
        firmwareOptions.hasUID = 1;
    }

    DBGLN("Options initialized - domain=%d, baud=%lu",
          firmwareOptions.domain, firmwareOptions.uart_baud);

    return true;
}

// Placeholder implementations for flash storage (not fully implemented yet)
static char optionsJson[ELRSOPTS_OPTIONS_SIZE] = "{}";
static char hardwareJson[ELRSOPTS_HARDWARE_SIZE] = "{}";

String optionsString(optionsJson);
String hardwareString(hardwareJson);

String& getOptions()
{
    return optionsString;
}

String& getHardware()
{
    return hardwareString;
}

void saveOptions()
{
    // Save options to NVM3 persistent storage
    elrs_config_t* cfg = elrs_config_get();
    if (cfg) {
        // Copy UID to config
        memcpy(cfg->uid, firmwareOptions.uid, 6);
        
        // Save to flash
        int result = elrs_config_save();
        if (result == 0) {
            DBGLN("Options saved to NVM3");
        } else {
            DBGLN("ERROR: Failed to save options to NVM3");
        }
    } else {
        DBGLN("ERROR: Config not available for save");
    }
}

void setOptions(String &options)
{
    strncpy(optionsJson, options.c_str(), sizeof(optionsJson) - 1);
    optionsJson[sizeof(optionsJson) - 1] = '\0';
}

void options_SetTrueDefaults()
{
    // Reset to compiled defaults
    firmwareOptions.domain = ISM2G4;
    firmwareOptions.hasUID = 0;
    memset(firmwareOptions.uid, 0, sizeof(firmwareOptions.uid));
    firmwareOptions.flash_discriminator = 0;
    firmwareOptions.fan_min_runtime = 0;
    firmwareOptions.wifi_auto_on_interval = -1;
    memset(firmwareOptions.home_wifi_ssid, 0, sizeof(firmwareOptions.home_wifi_ssid));
    memset(firmwareOptions.home_wifi_password, 0, sizeof(firmwareOptions.home_wifi_password));
#if defined(TARGET_RX)
    firmwareOptions.uart_baud = 420000;
    firmwareOptions._unused1 = false;
    firmwareOptions.lock_on_first_connection = true;
    firmwareOptions.dji_permanently_armed = false;
    firmwareOptions.is_airport = false;
#endif
}
