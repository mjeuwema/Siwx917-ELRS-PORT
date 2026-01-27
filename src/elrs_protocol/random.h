/**
 * @file random.h
 * @brief Pseudo-random number generator for ELRS FHSS
 * 
 * Ported from ELRS 4.0 src/lib/FHSS/random.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * CRITICAL: This PRNG must produce the EXACT same sequence as the ESP32 version.
 * The TX and RX use the same seed (derived from UID) to generate identical
 * frequency hopping sequences. Any deviation will cause loss of sync.
 * 
 * Algorithm: Linear Congruential Generator (LCG)
 *   seed = (214013 * seed + 2531011) % 2147483648
 *   output = seed >> 16 (returns 0 to 0x7FFF)
 */

#ifndef ELRS_RANDOM_H
#define ELRS_RANDOM_H

#include "elrs_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Maximum value returned by rng()
 * Citation: ELRS random.h line 6
 */
#define RNG_MAX 0x7FFF

/**
 * @brief Set the PRNG seed
 * 
 * Must be called with the same seed on TX and RX to generate identical
 * FHSS sequences. Seed is typically derived from UID (binding phrase).
 * 
 * @param newSeed 32-bit seed value
 */
void rngSeed(uint32_t newSeed);

/**
 * @brief Generate next random number
 * 
 * Citation: ELRS random.cpp lines 8-15
 * LCG parameters: a=214013, c=2531011, m=2147483648
 * 
 * @return Random value between 0 and RNG_MAX (0x7FFF)
 */
uint16_t rng(void);

/**
 * @brief Generate random number in range [0, upper)
 * 
 * Citation: ELRS random.cpp lines 23-26
 * 
 * @param upper Upper bound (exclusive), must be < 256
 * @return Random value between 0 and upper-1
 */
uint8_t rngN(uint8_t upper);

/**
 * @brief Generate 8-bit random number
 * 
 * Citation: ELRS random.cpp lines 29-32
 * 
 * @return Random value between 0 and 255
 */
uint8_t rng8Bit(void);

/**
 * @brief Generate 5-bit random number
 * 
 * Citation: ELRS random.cpp lines 35-38
 * 
 * @return Random value between 0 and 31
 */
uint8_t rng5Bit(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_RANDOM_H */
