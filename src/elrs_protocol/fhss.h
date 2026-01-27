/**
 * @file fhss.h
 * @brief Frequency Hopping Spread Spectrum for ELRS
 * 
 * Ported from ELRS 4.0 src/lib/FHSS/FHSS.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * The FHSS sequence is deterministically generated from the UID seed.
 * Both TX and RX use the same algorithm to hop through frequencies in sync.
 * 
 * Key concepts:
 *   - Sequence length is 256 entries (FHSS_SEQUENCE_LEN)
 *   - Sync channel is freq_count/2 (middle of the band)
 *   - Every freq_count hops, we return to sync channel
 *   - Frequencies are stored directly in Hz for LR1121
 */

#ifndef ELRS_FHSS_H
#define ELRS_FHSS_H

#include "elrs_platform.h"
#include "random.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/**
 * FHSS sequence length
 * Citation: ELRS FHSS.h line 23
 */
#define FHSS_SEQUENCE_LEN   256

/**
 * For LR1121, frequencies are stored directly in Hz
 * Citation: ELRS FHSS.h lines 16-17
 */
#define FREQ_HZ_TO_REG_VAL(freq)    (freq)
#define FREQ_SPREAD_SCALE           1

/*******************************************************************************
 * Regulatory Domain Configuration
 * 
 * Citation: ELRS FHSS.cpp lines 14-23
 * Each domain defines frequency range and number of channels
 ******************************************************************************/

typedef struct {
    const char  *domain;        /* Domain name string */
    uint32_t    freq_start;     /* Start frequency in Hz */
    uint32_t    freq_stop;      /* Stop frequency in Hz */
    uint32_t    freq_count;     /* Number of channels */
    uint32_t    freq_center;    /* Center frequency for display */
} fhss_config_t;

/**
 * Regulatory domain index enumeration
 * Citation: ELRS FHSS.cpp lines 14-23
 */
typedef enum {
    DOMAIN_AU915 = 0,   /* 915.5 - 926.9 MHz, 20 channels */
    DOMAIN_FCC915,      /* 903.5 - 926.9 MHz, 40 channels */
    DOMAIN_EU868,       /* 863.275 - 869.575 MHz, 13 channels */
    DOMAIN_IN866,       /* 865.375 - 866.950 MHz, 4 channels */
    DOMAIN_AU433,       /* 433.42 - 434.42 MHz, 3 channels */
    DOMAIN_EU433,       /* 433.1 - 434.45 MHz, 3 channels */
    DOMAIN_US433,       /* 433.25 - 438.0 MHz, 8 channels */
    DOMAIN_US433W,      /* 423.5 - 438.0 MHz, 20 channels (wide) */
    DOMAIN_COUNT
} fhss_domain_e;

/**
 * 2.4 GHz domain for dual-band LR1121
 * Citation: ELRS FHSS.cpp lines 26-34
 */
typedef enum {
    DOMAIN_ISM2G4 = 0,  /* 2400.4 - 2479.4 MHz, 80 channels */
    DOMAIN_2G4_COUNT
} fhss_domain_2g4_e;

/*******************************************************************************
 * Global Variables (extern declarations)
 ******************************************************************************/

/* Current domain configuration pointers */
extern const fhss_config_t *FHSSconfig;
extern const fhss_config_t *FHSSconfigDualBand;

/* FHSS sequence arrays */
extern uint8_t FHSSsequence[FHSS_SEQUENCE_LEN];
extern uint8_t FHSSsequence_DualBand[FHSS_SEQUENCE_LEN];

/* Current position in sequence */
extern volatile uint8_t FHSSptr;

/* Sync channel indices */
extern uint8_t sync_channel;
extern uint8_t sync_channel_DualBand;

/* Frequency correction (AFC) - register units */
extern int32_t FreqCorrection;
extern int32_t FreqCorrection_2;

/* Frequency hop spacing */
extern uint32_t freq_spread;
extern uint32_t freq_spread_DualBand;

/* Dual band control */
extern bool FHSSusePrimaryFreqBand;
extern bool FHSSuseDualBand;

/* Band counts for sequence limiting */
extern uint16_t primaryBandCount;
extern uint16_t secondaryBandCount;

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize FHSS sequence from seed
 * 
 * Generates the frequency hopping sequence deterministically from the seed.
 * Must be called on both TX and RX with the same seed (derived from UID).
 * 
 * Citation: ELRS FHSS.cpp lines 81-106
 * 
 * @param seed 32-bit seed value (typically derived from UID)
 */
void FHSSrandomiseFHSSsequence(uint32_t seed);

/**
 * @brief Build FHSS sequence for a specific band
 * 
 * Internal function used by FHSSrandomiseFHSSsequence.
 * 
 * Citation: ELRS FHSS.cpp lines 121-162
 * 
 * @param seed Seed value
 * @param freqCount Number of frequencies in the band
 * @param syncChannel Index of sync channel
 * @param sequence Output sequence array
 */
void FHSSrandomiseFHSSsequenceBuild(uint32_t seed, uint32_t freqCount, 
                                     uint8_t syncChannel, uint8_t *sequence);

/**
 * @brief Check if current domain is EU868
 * Citation: ELRS FHSS.cpp lines 164-167
 */
bool isDomain868(void);

/**
 * @brief Check if using primary frequency band
 * Citation: ELRS FHSS.cpp lines 169-172
 */
bool isUsingPrimaryFreqBand(void);

/*******************************************************************************
 * Inline Functions
 * 
 * These are performance-critical and kept inline for timing
 ******************************************************************************/

/**
 * @brief Get minimum frequency in current band
 * Citation: ELRS FHSS.h lines 57-60
 */
static inline uint32_t FHSSgetMinimumFreq(void)
{
    return FHSSconfig->freq_start;
}

/**
 * @brief Get maximum frequency in current band
 * Citation: ELRS FHSS.h lines 62-65
 */
static inline uint32_t FHSSgetMaximumFreq(void)
{
    return FHSSconfig->freq_stop;
}

/**
 * @brief Get number of channels in current band
 * Citation: ELRS FHSS.h lines 68-78
 */
static inline uint32_t FHSSgetChannelCount(void)
{
    if (FHSSusePrimaryFreqBand)
    {
        return FHSSconfig->freq_count;
    }
    else
    {
        return FHSSconfigDualBand->freq_count;
    }
}

/**
 * @brief Get number of entries in FHSS sequence
 * Citation: ELRS FHSS.h lines 81-103
 */
static inline uint16_t FHSSgetSequenceCount(void)
{
    if (FHSSuseDualBand)
    {
        /* Use smaller of 2 bands */
        if (primaryBandCount < secondaryBandCount)
        {
            return primaryBandCount;
        }
        else
        {
            return secondaryBandCount;
        }
    }

    if (FHSSusePrimaryFreqBand)
    {
        return primaryBandCount;
    }
    else
    {
        return secondaryBandCount;
    }
}

/**
 * @brief Get initial/sync frequency
 * Citation: ELRS FHSS.h lines 106-116
 */
static inline uint32_t FHSSgetInitialFreq(void)
{
    if (FHSSusePrimaryFreqBand)
    {
        return FHSSconfig->freq_start + 
               (sync_channel * freq_spread / FREQ_SPREAD_SCALE) - FreqCorrection;
    }
    else
    {
        return FHSSconfigDualBand->freq_start + 
               (sync_channel_DualBand * freq_spread_DualBand / FREQ_SPREAD_SCALE);
    }
}

/**
 * @brief Get current sequence index
 * Citation: ELRS FHSS.h lines 119-122
 */
static inline uint8_t FHSSgetCurrIndex(void)
{
    return FHSSptr;
}

/**
 * @brief Check if currently on sync channel
 * Citation: ELRS FHSS.h lines 125-135
 */
static inline uint8_t FHSSonSyncChannel(void)
{
    if (FHSSusePrimaryFreqBand)
    {
        return FHSSsequence[FHSSptr] == sync_channel;
    }
    else
    {
        return FHSSsequence_DualBand[FHSSptr] == sync_channel_DualBand;
    }
}

/**
 * @brief Set current sequence index (used by RX on SYNC)
 * Citation: ELRS FHSS.h lines 138-141
 */
static inline void FHSSsetCurrIndex(uint8_t value)
{
    FHSSptr = value % FHSSgetSequenceCount();
}

/**
 * @brief Advance to next frequency and return it
 * Citation: ELRS FHSS.h lines 144-156
 */
static inline uint32_t FHSSgetNextFreq(void)
{
    FHSSptr = (FHSSptr + 1) % FHSSgetSequenceCount();

    if (FHSSusePrimaryFreqBand)
    {
        return FHSSconfig->freq_start + 
               (freq_spread * FHSSsequence[FHSSptr] / FREQ_SPREAD_SCALE) - FreqCorrection;
    }
    else
    {
        return FHSSconfigDualBand->freq_start + 
               (freq_spread_DualBand * FHSSsequence_DualBand[FHSSptr] / FREQ_SPREAD_SCALE);
    }
}

/**
 * @brief Get regulatory domain name string
 * Citation: ELRS FHSS.h lines 158-168
 */
static inline const char *FHSSgetRegulatoryDomain(void)
{
    if (FHSSusePrimaryFreqBand)
    {
        return FHSSconfig->domain;
    }
    else
    {
        return FHSSconfigDualBand->domain;
    }
}

/**
 * @brief Get Gemini offset frequency for antenna diversity
 * Citation: ELRS FHSS.h lines 171-187
 */
static inline uint32_t FHSSGeminiFreq(uint8_t FHSSsequenceIdx)
{
    uint32_t freq;
    uint32_t numfhss = FHSSgetChannelCount();
    uint8_t offSetIdx = (FHSSsequenceIdx + (numfhss / 2)) % numfhss;

    if (FHSSusePrimaryFreqBand)
    {
        freq = FHSSconfig->freq_start + 
               (freq_spread * offSetIdx / FREQ_SPREAD_SCALE) - FreqCorrection_2;
    }
    else
    {
        freq = FHSSconfigDualBand->freq_start + 
               (freq_spread_DualBand * offSetIdx / FREQ_SPREAD_SCALE);
    }

    return freq;
}

/**
 * @brief Get current Gemini frequency
 * Citation: ELRS FHSS.h lines 189-207
 */
static inline uint32_t FHSSgetGeminiFreq(void)
{
    if (FHSSuseDualBand)
    {
        return FHSSconfigDualBand->freq_start + 
               (FHSSsequence_DualBand[FHSSptr] * freq_spread_DualBand / FREQ_SPREAD_SCALE);
    }
    else
    {
        if (FHSSusePrimaryFreqBand)
        {
            return FHSSGeminiFreq(FHSSsequence[FHSSgetCurrIndex()]);
        }
        else
        {
            return FHSSGeminiFreq(FHSSsequence_DualBand[FHSSgetCurrIndex()]);
        }
    }
}

/**
 * @brief Get initial Gemini frequency
 * Citation: ELRS FHSS.h lines 209-226
 */
static inline uint32_t FHSSgetInitialGeminiFreq(void)
{
    if (FHSSuseDualBand)
    {
        return FHSSconfigDualBand->freq_start + 
               (sync_channel_DualBand * freq_spread_DualBand / FREQ_SPREAD_SCALE);
    }
    else
    {
        if (FHSSusePrimaryFreqBand)
        {
            return FHSSGeminiFreq(sync_channel);
        }
        else
        {
            return FHSSGeminiFreq(sync_channel_DualBand);
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* ELRS_FHSS_H */
