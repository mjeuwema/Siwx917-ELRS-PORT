/**
 * @file common.cpp
 * @brief ELRS common functionality - rate tables and global state
 *
 * This file contains the LR1121 rate tables and global accessor functions.
 * Adapted from upstream ExpressLRS/src/src/common.cpp
 */

#include "common.h"
#include "OTA.h"
#include "LR1121Driver.h"
#include "LR1121_Regs.h"
#include "crsf_protocol.h"
#include "options.h"

// Global Radio instance
LR1121Driver Radio;

// Global link statistics
elrsLinkStatistics_t linkStats = {};

// LR1121 Air Rate Configuration Table
// Format: {index, radio_type, enum_rate, BW, SF, CR, PreambleLen, BW_2, SF_2, CR_2, PreambleLen_2, TLMinterval, FHSShopInterval, interval, PayloadLength, numOfSends}
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // 900MHz modes (primary for FCC915 domain)
    {0,  RADIO_TYPE_LR1121_GFSK_900,  RATE_FSK_900_1000HZ_8CH,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA8_PACKET_SIZE, 1},
    {1,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_250HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_8,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {2,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_200HZ_8CH,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA8_PACKET_SIZE, 1},
    {3,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_200HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA4_PACKET_SIZE, 1},
    {4,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_100HZ_8CH,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {5,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_100HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_32,  4, 10000, OTA4_PACKET_SIZE, 1},
    {6,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_50HZ,       LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_4_7,    10, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_16,  4, 20000, OTA4_PACKET_SIZE, 1},
    {7,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_25HZ,       LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF9,       LR11XX_RADIO_LORA_CR_4_7,    10, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF9,       LR11XX_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_8,   2, 40000, OTA4_PACKET_SIZE, 1},
    {8,  RADIO_TYPE_LR1121_LORA_900,  RATE_LORA_900_50HZ_DVDA,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  2,  5000, OTA4_PACKET_SIZE, 4},
    // 2.4GHz modes
    {9,  RADIO_TYPE_LR1121_GFSK_2G4,  RATE_FSK_2G4_1000HZ,      LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 1},
    {10, RADIO_TYPE_LR1121_GFSK_2G4,  RATE_FSK_2G4_500HZ_DVDA,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 2},
    {11, RADIO_TYPE_LR1121_GFSK_2G4,  RATE_FSK_2G4_250HZ_DVDA,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 4},
    {12, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_500HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_128, 4,  2000, OTA4_PACKET_SIZE, 1},
    {13, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_333HZ_8CH,  LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_128, 4,  3003, OTA8_PACKET_SIZE, 1},
    {14, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_250HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {15, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_150HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {16, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_100HZ_8CH,  LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {17, RADIO_TYPE_LR1121_LORA_2G4,  RATE_LORA_2G4_50HZ,       LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_16,  2, 20000, OTA4_PACKET_SIZE, 1},
    // Dual band modes (not supported on single LR1121)
    {18, RADIO_TYPE_LR1121_LORA_DUAL, RATE_LORA_DUAL_150HZ,     LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,    12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {19, RADIO_TYPE_LR1121_LORA_DUAL, RATE_LORA_DUAL_100HZ_8CH, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,    18, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1}
};

// LR1121 RF Performance Parameters Table
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0,  -101,   658, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {1,  -111,  3216, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {2,  -111,  4240, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {3,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {4,  -112,  6690, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {5,  -117,  8770, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(2.5)},
    {6,  -120, 18560, 4000, 2500, 600,  5000, SNR_SCALE(-1), SNR_SCALE(1.5)},
    {7,  -123, 29950, 6000, 4000, 600,  5000, SNR_SCALE(-3), SNR_SCALE(0.5)},
    {8,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {9,  -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {10, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {11, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {12, -105,  1507, 2500, 2500,   3,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {13, -105,  2374, 2500, 2500,   4,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {14, -108,  3300, 3000, 2500,   6,  5000, SNR_SCALE( 3), SNR_SCALE(9.5)},
    {15, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {16, -112,  7605, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {17, -115, 10798, 4000, 2500,   0,  5000, SNR_SCALE(-1), SNR_SCALE(6.5)},
    {18, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {19, -112,  7456, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)}
};

expresslrs_mod_settings_s *get_elrs_airRateConfig(uint8_t index)
{
    if (RATE_MAX <= index)
    {
        index = RATE_MAX - 1;
    }
    return &ExpressLRS_AirRateConfig[index];
}

expresslrs_rf_pref_params_s *get_elrs_RFperfParams(uint8_t index)
{
    if (RATE_MAX <= index)
    {
        index = RATE_MAX - 1;
    }
    return &ExpressLRS_AirRateRFperf[index];
}

uint8_t get_elrs_HandsetRate_max(uint8_t rateIndex, uint32_t minInterval)
{
    while (rateIndex < RATE_MAX)
    {
        expresslrs_mod_settings_s const * const ModParams = &ExpressLRS_AirRateConfig[rateIndex];
        uint32_t handsetInterval = ModParams->interval * ModParams->numOfSends;
        if (handsetInterval >= minInterval && isSupportedRFRate(rateIndex))
            break;
        ++rateIndex;
    }
    return rateIndex;
}

uint8_t ICACHE_RAM_ATTR enumRatetoIndex(expresslrs_RFrates_e const eRate)
{
    expresslrs_mod_settings_s const * ModParams;
    for (uint8_t i = 0; i < RATE_MAX; i++)
    {
        ModParams = get_elrs_airRateConfig(i);
        if (ModParams->enum_rate == eRate)
        {
            return i;
        }
    }
    return 0;  // Not found, return first rate
}

uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(expresslrs_tlm_ratio_e const enumval)
{
    // TLM_RATIO_STD/TLM_RATIO_DISARMED should be converted by the caller
    if (enumval == TLM_RATIO_NO_TLM)
        return 1;

    // 1 << (8 - (enumval - TLM_RATIO_NO_TLM))
    // 1_128 = 128, 1_64 = 64, 1_32 = 32, etc
    return 1 << (8 + TLM_RATIO_NO_TLM - enumval);
}

uint8_t TLMBurstMaxForRateRatio(uint16_t const rateHz, uint8_t const ratioDiv)
{
    constexpr uint32_t TELEM_MIN_LINK_INTERVAL_MS = 512U;
    unsigned retVal = TELEM_MIN_LINK_INTERVAL_MS * rateHz / ratioDiv / 1000U;
    if (retVal > 1)
        --retVal;
    else
        retVal = 1;
    return retVal;
}

bool isSupportedRFRate(uint8_t index)
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);

    // Dual Band modes not supported for hardware with only a single LR1121
    if (!isDualRadio() && ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL)
    {
        return false;
    }

    // For 900MHz domain (FCC915, AU915, EU868, etc.), skip 2.4GHz-only rates
    // For 2.4GHz domain, skip 900MHz-only rates
    // This ensures RX scans the correct band for the configured domain
    if (firmwareOptions.domain <= 7) // 900MHz domains (AU915, FCC915, EU868, IN866, AU433, EU433, US433, US433W)
    {
        // Skip 2.4GHz-only rates (9-17)
        if (ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4 ||
            ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4)
        {
            return false;
        }
    }

    return true;
}
