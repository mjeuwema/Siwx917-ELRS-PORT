/**
 * @file crc.c
 * @brief CRC calculation for ELRS OTA packets
 * 
 * Ported from ELRS 4.0 src/lib/CRC/crc.cpp
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * This implementation is bit-for-bit compatible with ELRS ESP32 code.
 * The CRC algorithms are critical for OTA packet validation - any deviation
 * will cause packet rejection.
 */

#include "crc.h"

/*******************************************************************************
 * 8-bit CRC Implementation
 ******************************************************************************/

void elrs_crc8_init(elrs_crc8_t *ctx, uint8_t poly)
{
    uint8_t crc;
    
    ctx->crcpoly = poly;
    
    /* Build lookup table for fast CRC calculation */
    for (uint16_t i = 0; i < CRC_TABLE_LEN; i++)
    {
        crc = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        ctx->crc8tab[i] = crc & 0xFF;
    }
}

uint8_t elrs_crc8_calc_byte(const elrs_crc8_t *ctx, uint8_t data)
{
    return ctx->crc8tab[data];
}

uint8_t elrs_crc8_calc(const elrs_crc8_t *ctx, const uint8_t *data, uint16_t len, uint8_t crc)
{
    while (len--)
    {
        crc = ctx->crc8tab[crc ^ *data++];
    }
    return crc;
}

/*******************************************************************************
 * 14/16-bit CRC Implementation
 ******************************************************************************/

void elrs_crc16_init(elrs_crc16_t *ctx, uint8_t bits, uint16_t poly)
{
    /* Skip if already initialized with same parameters */
    if (bits == ctx->bits && poly == ctx->poly)
        return;
    
    ctx->poly = poly;
    ctx->bits = bits;
    ctx->bitmask = (1 << bits) - 1;
    
    uint16_t highbit = 1 << (bits - 1);
    uint16_t crc;
    
    /* Build lookup table */
    for (uint16_t i = 0; i < CRC_TABLE_LEN; i++)
    {
        crc = i << (bits - 8);
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & highbit) ? poly : 0);
        }
        ctx->crctab[i] = crc;
    }
}

uint16_t elrs_crc16_calc(const elrs_crc16_t *ctx, uint8_t *data, uint8_t len, uint16_t crc)
{
    while (len--)
    {
        crc = (crc << 8) ^ ctx->crctab[((crc >> (ctx->bits - 8)) ^ (uint16_t)*data++) & 0x00FF];
    }
    return crc & ctx->bitmask;
}
