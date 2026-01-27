/**
 * @file random.c
 * @brief Pseudo-random number generator for ELRS FHSS
 * 
 * Ported from ELRS 4.0 src/lib/FHSS/random.cpp
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * Linear Congruential Generator (LCG) implementation.
 * This is bit-for-bit compatible with the ELRS ESP32 implementation.
 * 
 * CRITICAL: Do not modify the constants or algorithm - this must produce
 * the exact same sequence as the TX to maintain frequency sync.
 */

#include "random.h"

/* Static seed - shared across all calls */
static uint32_t seed = 0;

/**
 * @brief Generate next random number (0 to 0x7FFF)
 * 
 * LCG Formula: seed = (a * seed + c) % m
 *   a = 214013
 *   c = 2531011  
 *   m = 2147483648 (2^31)
 * 
 * Citation: ELRS random.cpp lines 8-15
 * Note: rngN depends on this output range (0-0x7FFF), so if we change
 * the behavior, rngN will need updating.
 */
uint16_t rng(void)
{
    const uint32_t m = 2147483648;   /* 2^31 */
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    
    seed = (a * seed + c) % m;
    return seed >> 16;
}

/**
 * @brief Set the PRNG seed
 * 
 * Citation: ELRS random.cpp lines 17-20
 */
void rngSeed(const uint32_t newSeed)
{
    seed = newSeed;
}

/**
 * @brief Generate random number in range [0, max)
 * 
 * Citation: ELRS random.cpp lines 23-26
 * Returns 0 <= x < max where max < 256
 */
uint8_t rngN(const uint8_t max)
{
    return rng() % max;
}

/**
 * @brief Generate 8-bit random number (0-255)
 * 
 * Citation: ELRS random.cpp lines 29-32
 */
uint8_t rng8Bit(void)
{
    return rng() & 0xFF;
}

/**
 * @brief Generate 5-bit random number (0-31)
 * 
 * Citation: ELRS random.cpp lines 35-38
 */
uint8_t rng5Bit(void)
{
    return rng() & 0x1F;
}
