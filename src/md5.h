/***************************************************************************/ /**
 * @file md5.h
 * @brief MD5 Message-Digest Algorithm for ELRS Binding Phrase
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * MD5 implementation for converting ELRS binding phrases to UIDs.
 * 
 * Citation: RFC 1321 - The MD5 Message-Digest Algorithm
 * Citation: ExpressLRS - Uses MD5 hash of binding phrase, first 6 bytes = UID
 *
 * This is a compact, embedded-friendly MD5 implementation optimized for
 * single-use hashing of short binding phrases (typically < 128 bytes).
 *
 ******************************************************************************/

#ifndef ELRS_MD5_H
#define ELRS_MD5_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief MD5 context structure
 * 
 * Citation: RFC 1321 Section 3.3
 * - A[0] through A[3]: 4x 32-bit state registers
 * - count: number of bits processed (64-bit)
 * - buffer: input buffer for partial blocks
 */
typedef struct {
    uint32_t state[4];    /**< State (ABCD) */
    uint32_t count[2];    /**< Number of bits, modulo 2^64 (lsb first) */
    uint8_t  buffer[64];  /**< Input buffer */
} md5_context_t;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize MD5 context
 * 
 * Citation: RFC 1321 Section 3.3 - Initial values for A, B, C, D
 * A = 0x67452301, B = 0xefcdab89, C = 0x98badcfe, D = 0x10325476
 *
 * @param ctx Pointer to MD5 context to initialize
 */
void md5_init(md5_context_t *ctx);

/**
 * @brief Update MD5 hash with input data
 * 
 * Citation: RFC 1321 Section 3.4 - Process Message in 16-Word Blocks
 *
 * @param ctx Pointer to MD5 context
 * @param input Input data to hash
 * @param len Length of input data in bytes
 */
void md5_update(md5_context_t *ctx, const uint8_t *input, size_t len);

/**
 * @brief Finalize MD5 hash and output digest
 * 
 * Citation: RFC 1321 Section 3.5 - Padding and finalization
 *
 * @param ctx Pointer to MD5 context
 * @param digest Output buffer for 16-byte (128-bit) MD5 digest
 */
void md5_final(md5_context_t *ctx, uint8_t digest[16]);

/**
 * @brief Compute MD5 hash of data in one call
 *
 * Convenience function that combines init, update, and final.
 *
 * @param data Input data to hash
 * @param len Length of input data
 * @param digest Output buffer for 16-byte MD5 digest
 */
void md5_hash(const uint8_t *data, size_t len, uint8_t digest[16]);

/**
 * @brief Convert ELRS binding phrase to 6-byte UID using MD5
 * 
 * Citation: ExpressLRS config.cpp - Binding phrase handling
 * - Computes MD5 hash of the binding phrase string
 * - Takes first 6 bytes of the 16-byte MD5 digest as UID
 * - This ensures TX and RX with same phrase get identical UIDs
 *
 * @param phrase Null-terminated binding phrase string
 * @param uid_out Output buffer for 6-byte UID
 */
void elrs_md5_uid_from_phrase(const char *phrase, uint8_t uid_out[6]);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_MD5_H */
