/**
 * @file fhss.c
 * @brief Frequency Hopping Spread Spectrum for ELRS
 * 
 * Ported from ELRS 4.0 src/lib/FHSS/FHSS.cpp
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * CRITICAL: The FHSS sequence algorithm must be bit-for-bit identical
 * to the ESP32 version. Any deviation will cause TX/RX desync.
 */

#include "fhss.h"
#include <string.h>

/*******************************************************************************
 * Regulatory Domain Tables
 * 
 * Citation: ELRS FHSS.cpp lines 14-23
 * Frequencies are in Hz for LR1121
 ******************************************************************************/

static const fhss_config_t domains[] = {
    /* AU915:  915.5 - 926.9 MHz, 20 channels */
    {"AU915",  FREQ_HZ_TO_REG_VAL(915500000), FREQ_HZ_TO_REG_VAL(926900000), 20, 921000000},
    
    /* FCC915: 903.5 - 926.9 MHz, 40 channels (USA) */
    {"FCC915", FREQ_HZ_TO_REG_VAL(903500000), FREQ_HZ_TO_REG_VAL(926900000), 40, 915000000},
    
    /* EU868:  863.275 - 869.575 MHz, 13 channels (Europe) */
    {"EU868",  FREQ_HZ_TO_REG_VAL(863275000), FREQ_HZ_TO_REG_VAL(869575000), 13, 868000000},
    
    /* IN866:  865.375 - 866.950 MHz, 4 channels (India) */
    {"IN866",  FREQ_HZ_TO_REG_VAL(865375000), FREQ_HZ_TO_REG_VAL(866950000), 4, 866000000},
    
    /* AU433:  433.42 - 434.42 MHz, 3 channels */
    {"AU433",  FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 3, 434000000},
    
    /* EU433:  433.1 - 434.45 MHz, 3 channels */
    {"EU433",  FREQ_HZ_TO_REG_VAL(433100000), FREQ_HZ_TO_REG_VAL(434450000), 3, 434000000},
    
    /* US433:  433.25 - 438.0 MHz, 8 channels */
    {"US433",  FREQ_HZ_TO_REG_VAL(433250000), FREQ_HZ_TO_REG_VAL(438000000), 8, 434000000},
    
    /* US433W: 423.5 - 438.0 MHz, 20 channels (wide) */
    {"US433W", FREQ_HZ_TO_REG_VAL(423500000), FREQ_HZ_TO_REG_VAL(438000000), 20, 434000000},
};

/**
 * 2.4 GHz domain for dual-band LR1121
 * Citation: ELRS FHSS.cpp lines 26-34
 */
static const fhss_config_t domainsDualBand[] = {
    /* ISM2G4: 2400.4 - 2479.4 MHz, 80 channels */
    {"ISM2G4", FREQ_HZ_TO_REG_VAL(2400400000UL), FREQ_HZ_TO_REG_VAL(2479400000UL), 80, 2440000000UL}
};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* Current domain configuration */
const fhss_config_t *FHSSconfig = NULL;
const fhss_config_t *FHSSconfigDualBand = NULL;

/* FHSS sequences */
uint8_t FHSSsequence[FHSS_SEQUENCE_LEN];
uint8_t FHSSsequence_DualBand[FHSS_SEQUENCE_LEN];

/* Current position in sequence */
volatile uint8_t FHSSptr = 0;

/* Sync channel indices */
uint8_t sync_channel = 0;
uint8_t sync_channel_DualBand = 0;

/* Frequency correction (AFC) */
int32_t FreqCorrection = 0;
int32_t FreqCorrection_2 = 0;

/* Frequency hop spacing */
uint32_t freq_spread = 0;
uint32_t freq_spread_DualBand = 0;

/* Dual band control */
bool FHSSusePrimaryFreqBand = true;
bool FHSSuseDualBand = false;

/* Band counts */
uint16_t primaryBandCount = 0;
uint16_t secondaryBandCount = 0;

/*******************************************************************************
 * Function Implementations
 ******************************************************************************/

/**
 * @brief Initialize FHSS sequence from seed
 * 
 * Citation: ELRS FHSS.cpp lines 81-106
 * 
 * Requirements for the sequence (from ELRS comments):
 * 1. Sync channel (0) every freq_count hops
 * 2. No two repeated channels in a row
 * 3. Equal occurrence of each channel (or as even as possible)
 * 4. Pseudorandom distribution
 */
void FHSSrandomiseFHSSsequence(uint32_t seed)
{
    /* Set primary domain from firmware options */
    FHSSconfig = &domains[firmwareOptions.domain];
    
    /* Sync channel is middle of the band */
    sync_channel = FHSSconfig->freq_count / 2;
    
    /* Calculate frequency spacing */
    freq_spread = (FHSSconfig->freq_stop - FHSSconfig->freq_start) * 
                  FREQ_SPREAD_SCALE / (FHSSconfig->freq_count - 1);
    
    /* Calculate how many complete cycles fit in sequence */
    primaryBandCount = (FHSS_SEQUENCE_LEN / FHSSconfig->freq_count) * 
                       FHSSconfig->freq_count;

    DBGLN("Primary Domain %s, %u channels, sync=%u",
        FHSSconfig->domain, (unsigned)FHSSconfig->freq_count, (unsigned)sync_channel);

    /* Build primary band sequence */
    FHSSrandomiseFHSSsequenceBuild(seed, FHSSconfig->freq_count, 
                                    sync_channel, FHSSsequence);

    /* For LR1121, also set up 2.4 GHz dual band */
#if defined(RADIO_LR1121)
    FHSSconfigDualBand = &domainsDualBand[0];
    sync_channel_DualBand = FHSSconfigDualBand->freq_count / 2;
    freq_spread_DualBand = (FHSSconfigDualBand->freq_stop - FHSSconfigDualBand->freq_start) * 
                           FREQ_SPREAD_SCALE / (FHSSconfigDualBand->freq_count - 1);
    secondaryBandCount = (FHSS_SEQUENCE_LEN / FHSSconfigDualBand->freq_count) * 
                         FHSSconfigDualBand->freq_count;

    DBGLN("Dual Domain %s, %u channels, sync=%u",
        FHSSconfigDualBand->domain, (unsigned)FHSSconfigDualBand->freq_count, 
        (unsigned)sync_channel_DualBand);

    /* Build dual band sequence */
    FHSSusePrimaryFreqBand = false;
    FHSSrandomiseFHSSsequenceBuild(seed, FHSSconfigDualBand->freq_count, 
                                    sync_channel_DualBand, FHSSsequence_DualBand);
    FHSSusePrimaryFreqBand = true;
#endif
}

/**
 * @brief Build FHSS sequence for a specific band
 * 
 * Citation: ELRS FHSS.cpp lines 108-162
 * 
 * Algorithm:
 * 1. Fill array with sync channel every freq_count entries
 * 2. Fill remaining entries with sequential channel numbers
 * 3. Shuffle entries within each block (excluding sync channel)
 */
void FHSSrandomiseFHSSsequenceBuild(uint32_t seed, uint32_t freqCount, 
                                     uint8_t syncChannel, uint8_t *inSequence)
{
    /* Reset pointer for consistent test results */
    FHSSptr = 0;
    
    /* Seed the PRNG */
    rngSeed(seed);

    /* Initialize the sequence array */
    for (uint16_t i = 0; i < FHSSgetSequenceCount(); i++)
    {
        if (i % freqCount == 0) 
        {
            /* Sync channel at start of each block */
            inSequence[i] = syncChannel;
        } 
        else if (i % freqCount == syncChannel) 
        {
            /* Move channel 0 to where sync channel was */
            inSequence[i] = 0;
        } 
        else 
        {
            /* Sequential channel number */
            inSequence[i] = i % freqCount;
        }
    }

    /* Shuffle entries within each block */
    for (uint16_t i = 0; i < FHSSgetSequenceCount(); i++)
    {
        /* Skip sync channel positions */
        if (i % freqCount != 0)
        {
            uint8_t offset = (i / freqCount) * freqCount;  /* Start of current block */
            uint8_t rand = rngN(freqCount - 1) + 1;         /* Random 1 to freqCount-1 */

            /* Swap this entry with random entry in same block */
            uint8_t temp = inSequence[i];
            inSequence[i] = inSequence[offset + rand];
            inSequence[offset + rand] = temp;
        }
    }

    /* Debug: output FHSS sequence */
    /*
    for (uint16_t i = 0; i < FHSSgetSequenceCount(); i++)
    {
        DBG("%u ", inSequence[i]);
        if (i % 10 == 9)
            DBGCR;
    }
    DBGCR;
    */
}

/**
 * @brief Check if current domain is EU868
 * Citation: ELRS FHSS.cpp lines 164-167
 */
bool isDomain868(void)
{
    return strcmp(FHSSconfig->domain, "EU868") == 0;
}

/**
 * @brief Check if using primary frequency band
 * Citation: ELRS FHSS.cpp lines 169-172
 */
bool isUsingPrimaryFreqBand(void)
{
    return FHSSusePrimaryFreqBand;
}
