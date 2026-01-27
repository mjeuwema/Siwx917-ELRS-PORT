/**
 * @file crc.h
 * @brief CRC calculation for ELRS OTA packets
 * 
 * Ported from ELRS 4.0 src/lib/CRC/crc.h
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * CRC polynomials used by ELRS (Koopman notation in comments):
 *   ELRS_CRC_POLY     0x07   (0x83)   - 8-bit CRC for CRSF frames
 *   ELRS_CRC14_POLY   0x2E57 (0x372B) - 14-bit CRC for 4-byte OTA packets
 *   ELRS_CRC16_POLY   0x3D65 (0x9EB2) - 16-bit CRC for 8-byte OTA packets
 * 
 * Citation: https://users.ece.cmu.edu/~koopman/crc/
 */

#ifndef ELRS_CRC_H
#define ELRS_CRC_H

#include "elrs_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * CRC Polynomial Definitions
 * 
 * Citation: ELRS OTA.h lines 185-187
 * Value is implicit leading 1, comment is Koopman formatting (implicit trailing 1)
 ******************************************************************************/
#define ELRS_CRC_POLY       0x07    /* 0x83 - 8-bit for CRSF */
#define ELRS_CRC14_POLY     0x2E57  /* 0x372B - 14-bit for OTA4 */
#define ELRS_CRC16_POLY     0x3D65  /* 0x9EB2 - 16-bit for OTA8 */

#define CRC_TABLE_LEN       256

/*******************************************************************************
 * 8-bit CRC (GENERIC_CRC8)
 * 
 * Used for CRSF frame CRC validation
 ******************************************************************************/

/**
 * @brief 8-bit CRC context structure
 */
typedef struct {
    uint8_t crc8tab[CRC_TABLE_LEN];
    uint8_t crcpoly;
} elrs_crc8_t;

/**
 * @brief Initialize 8-bit CRC lookup table
 * @param ctx Pointer to CRC context
 * @param poly CRC polynomial (e.g., ELRS_CRC_POLY)
 */
void elrs_crc8_init(elrs_crc8_t *ctx, uint8_t poly);

/**
 * @brief Calculate CRC of a single byte
 * @param ctx Pointer to initialized CRC context
 * @param data Single byte to calculate CRC for
 * @return CRC value
 */
uint8_t elrs_crc8_calc_byte(const elrs_crc8_t *ctx, uint8_t data);

/**
 * @brief Calculate CRC of a byte array
 * @param ctx Pointer to initialized CRC context
 * @param data Pointer to data buffer
 * @param len Length of data buffer
 * @param crc Initial CRC value (usually 0)
 * @return Calculated CRC value
 */
uint8_t elrs_crc8_calc(const elrs_crc8_t *ctx, const uint8_t *data, uint16_t len, uint8_t crc);

/*******************************************************************************
 * 14/16-bit CRC (Crc2Byte)
 * 
 * Used for OTA packet CRC validation
 *   - 14-bit for standard 4-byte packets (OTA4)
 *   - 16-bit for full-resolution 8-byte packets (OTA8)
 ******************************************************************************/

/**
 * @brief 14/16-bit CRC context structure
 */
typedef struct {
    uint16_t crctab[CRC_TABLE_LEN];
    uint8_t  bits;
    uint16_t bitmask;
    uint16_t poly;
} elrs_crc16_t;

/**
 * @brief Initialize 14/16-bit CRC lookup table
 * @param ctx Pointer to CRC context
 * @param bits Number of CRC bits (14 or 16)
 * @param poly CRC polynomial (ELRS_CRC14_POLY or ELRS_CRC16_POLY)
 */
void elrs_crc16_init(elrs_crc16_t *ctx, uint8_t bits, uint16_t poly);

/**
 * @brief Calculate CRC of a byte array
 * @param ctx Pointer to initialized CRC context
 * @param data Pointer to data buffer
 * @param len Length of data buffer
 * @param crc Initial CRC value (usually OtaCrcInitializer ^ nonce)
 * @return Calculated CRC value (masked to bit width)
 */
uint16_t elrs_crc16_calc(const elrs_crc16_t *ctx, uint8_t *data, uint8_t len, uint16_t crc);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_CRC_H */
