/***************************************************************************/ /**
 * @file md5.c
 * @brief MD5 Message-Digest Algorithm Implementation
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * RFC 1321 compliant MD5 implementation for ELRS binding phrase to UID.
 * 
 * Citation: RFC 1321 - The MD5 Message-Digest Algorithm (April 1992)
 * Citation: ExpressLRS - Uses MD5 hash of binding phrase for UID generation
 *
 * This implementation is optimized for embedded systems:
 * - No dynamic memory allocation
 * - Compact code size (~1KB)
 * - Single-pass hashing for binding phrases
 *
 ******************************************************************************/

#include "md5.h"
#include <string.h>
#include <stdio.h>   /* For snprintf in elrs_md5_uid_from_phrase */

/*******************************************************************************
 * Constants
 * 
 * Citation: RFC 1321 Section 3.4 - Table T[1..64]
 * T[i] = floor(4294967296 * abs(sin(i))), i in radians
 ******************************************************************************/

/**
 * @brief MD5 transformation constants (T table from RFC 1321)
 * 
 * Citation: RFC 1321 Section 3.4
 * These constants are derived from the sine function to provide
 * a "random" bit pattern for the compression function.
 */
static const uint32_t K[64] = {
    /* Round 1 (F function) */
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
    0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
    0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    
    /* Round 2 (G function) */
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
    0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
    0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    
    /* Round 3 (H function) */
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
    0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
    0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    
    /* Round 4 (I function) */
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
    0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
    0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391
};

/**
 * @brief Per-round shift amounts
 * 
 * Citation: RFC 1321 Section 3.4
 * Shift amounts for each of the 64 operations.
 */
static const uint8_t S[64] = {
    /* Round 1 */
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    /* Round 2 */
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    /* Round 3 */
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    /* Round 4 */
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21
};

/*******************************************************************************
 * Macros
 * 
 * Citation: RFC 1321 Section 3.4 - Auxiliary functions F, G, H, I
 ******************************************************************************/

/**
 * @brief Left rotate 32-bit value
 */
#define ROTL32(x, n) (((x) << (n)) | ((x) >> (32 - (n))))

/**
 * @brief MD5 round functions (RFC 1321 Section 3.4)
 * 
 * F(X,Y,Z) = XY v not(X) Z  (Round 1)
 * G(X,Y,Z) = XZ v Y not(Z)  (Round 2)
 * H(X,Y,Z) = X xor Y xor Z  (Round 3)
 * I(X,Y,Z) = Y xor (X v not(Z))  (Round 4)
 */
#define F(x, y, z) (((x) & (y)) | ((~(x)) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~(z))))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~(z))))

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

/**
 * @brief Decode little-endian bytes to 32-bit words
 * 
 * Citation: RFC 1321 - MD5 uses little-endian byte ordering
 */
static void decode(uint32_t *output, const uint8_t *input, size_t len)
{
    for (size_t i = 0, j = 0; j < len; i++, j += 4) {
        output[i] = ((uint32_t)input[j]) |
                    (((uint32_t)input[j + 1]) << 8) |
                    (((uint32_t)input[j + 2]) << 16) |
                    (((uint32_t)input[j + 3]) << 24);
    }
}

/**
 * @brief Encode 32-bit words to little-endian bytes
 * 
 * Citation: RFC 1321 - MD5 uses little-endian byte ordering
 */
static void encode(uint8_t *output, const uint32_t *input, size_t len)
{
    for (size_t i = 0, j = 0; j < len; i++, j += 4) {
        output[j]     = (uint8_t)(input[i] & 0xff);
        output[j + 1] = (uint8_t)((input[i] >> 8) & 0xff);
        output[j + 2] = (uint8_t)((input[i] >> 16) & 0xff);
        output[j + 3] = (uint8_t)((input[i] >> 24) & 0xff);
    }
}

/**
 * @brief MD5 basic transformation - Process one 64-byte block
 * 
 * Citation: RFC 1321 Section 3.4 - Process Message in 16-Word Blocks
 * 
 * The transformation processes a 512-bit (64-byte) block through
 * 4 rounds of 16 operations each (64 total operations).
 */
static void md5_transform(uint32_t state[4], const uint8_t block[64])
{
    uint32_t a = state[0];
    uint32_t b = state[1];
    uint32_t c = state[2];
    uint32_t d = state[3];
    uint32_t x[16];
    
    /* Decode block to 16 32-bit words */
    decode(x, block, 64);
    
    /* 64 operations in 4 rounds */
    for (int i = 0; i < 64; i++) {
        uint32_t f, g;
        
        if (i < 16) {
            /* Round 1: F function
             * Citation: RFC 1321 Section 3.4 Round 1
             */
            f = F(b, c, d);
            g = i;
        } else if (i < 32) {
            /* Round 2: G function
             * Citation: RFC 1321 Section 3.4 Round 2
             */
            f = G(b, c, d);
            g = (5 * i + 1) % 16;
        } else if (i < 48) {
            /* Round 3: H function
             * Citation: RFC 1321 Section 3.4 Round 3
             */
            f = H(b, c, d);
            g = (3 * i + 5) % 16;
        } else {
            /* Round 4: I function
             * Citation: RFC 1321 Section 3.4 Round 4
             */
            f = I(b, c, d);
            g = (7 * i) % 16;
        }
        
        /* Update working variables
         * Citation: RFC 1321 - a = b + ((a + F(b,c,d) + X[k] + T[i]) <<< s)
         */
        uint32_t temp = d;
        d = c;
        c = b;
        b = b + ROTL32((a + f + K[i] + x[g]), S[i]);
        a = temp;
    }
    
    /* Add this block's hash to result so far
     * Citation: RFC 1321 Section 3.4 - Output is A+AA, B+BB, C+CC, D+DD
     */
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    
    /* Clear sensitive data */
    memset(x, 0, sizeof(x));
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void md5_init(md5_context_t *ctx)
{
    /**
     * Citation: RFC 1321 Section 3.3 - Step 3. Initialize MD Buffer
     * 
     * word A: 01 23 45 67  (little-endian: 0x67452301)
     * word B: 89 ab cd ef  (little-endian: 0xefcdab89)
     * word C: fe dc ba 98  (little-endian: 0x98badcfe)
     * word D: 76 54 32 10  (little-endian: 0x10325476)
     */
    ctx->count[0] = 0;
    ctx->count[1] = 0;
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xefcdab89;
    ctx->state[2] = 0x98badcfe;
    ctx->state[3] = 0x10325476;
}

void md5_update(md5_context_t *ctx, const uint8_t *input, size_t len)
{
    /* Compute number of bytes mod 64 */
    size_t index = (ctx->count[0] >> 3) & 0x3F;
    
    /* Update bit count
     * Citation: RFC 1321 - count is in bits, not bytes
     */
    ctx->count[0] += (uint32_t)(len << 3);
    if (ctx->count[0] < (uint32_t)(len << 3)) {
        ctx->count[1]++;  /* Overflow */
    }
    ctx->count[1] += (uint32_t)(len >> 29);
    
    size_t part_len = 64 - index;
    size_t i = 0;
    
    /* Transform as many times as possible */
    if (len >= part_len) {
        memcpy(&ctx->buffer[index], input, part_len);
        md5_transform(ctx->state, ctx->buffer);
        
        for (i = part_len; i + 63 < len; i += 64) {
            md5_transform(ctx->state, &input[i]);
        }
        
        index = 0;
    }
    
    /* Buffer remaining input */
    memcpy(&ctx->buffer[index], &input[i], len - i);
}

void md5_final(md5_context_t *ctx, uint8_t digest[16])
{
    /**
     * Citation: RFC 1321 Section 3.1 - Step 1. Append Padding Bits
     * 
     * The message is "padded" (extended) so that its length (in bits)
     * is congruent to 448, modulo 512. Padding is always performed,
     * even if the length is already congruent to 448, modulo 512.
     * 
     * Padding is a single "1" bit followed by zeros.
     */
    static const uint8_t padding[64] = {
        0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    
    uint8_t bits[8];
    
    /* Save number of bits
     * Citation: RFC 1321 Section 3.2 - Step 2. Append Length
     * A 64-bit representation of the length in bits is appended.
     */
    encode(bits, ctx->count, 8);
    
    /* Pad out to 56 mod 64 */
    size_t index = (ctx->count[0] >> 3) & 0x3F;
    size_t pad_len = (index < 56) ? (56 - index) : (120 - index);
    md5_update(ctx, padding, pad_len);
    
    /* Append length (before padding) */
    md5_update(ctx, bits, 8);
    
    /* Store state in digest
     * Citation: RFC 1321 Section 3.5 - Step 5. Output
     * The message digest produced as output is A, B, C, D starting
     * with the low-order byte of A, and ending with the high-order
     * byte of D (little-endian).
     */
    encode(digest, ctx->state, 16);
    
    /* Clear sensitive information */
    memset(ctx, 0, sizeof(*ctx));
}

void md5_hash(const uint8_t *data, size_t len, uint8_t digest[16])
{
    md5_context_t ctx;
    md5_init(&ctx);
    md5_update(&ctx, data, len);
    md5_final(&ctx, digest);
}

void elrs_md5_uid_from_phrase(const char *phrase, uint8_t uid_out[6])
{
    /**
     * Citation: ExpressLRS UID generation - CONFIGURATOR METHOD
     * 
     * ExpressLRS Configurator (web UI) hashes the BINDING PHRASE DIRECTLY.
     * This is the standard method used by most users who flash via the web interface.
     * 
     * Example:
     *   User enters: "matthew"
     *   String hashed: "matthew" (just the phrase, no wrapper!)
     *   MD5 result: E6 A5 BA 08 42 A5 ... (first 6 bytes = UID)
     *   UID: [230, 165, 186, 8, 66, 165]
     * 
     * NOTE: There is also an older "build flag" method used when compiling with
     * MY_BINDING_PHRASE in user_defines.txt, which hashes the entire flag string:
     *   String hashed: -DMY_BINDING_PHRASE="matthew"
     *   MD5 result: BE 93 67 27 D6 9C ...
     *   UID: [190, 147, 103, 39, 214, 156]
     * 
     * We use the CONFIGURATOR METHOD (direct phrase hash) because:
     *   1. Most users flash TX via ExpressLRS Configurator web UI
     *   2. WiFi binding phrase input expects this method
     *   3. This matches the ExpressLRS rainbow table lookups
     * 
     * Citation: ExpressLRS Configurator source - uses direct MD5 of phrase
     * Citation: https://github.com/wetheredge/expresslrs-uid-lookup
     *   Shows "expresslrs" -> 65,245,33,230,58,226 which is MD5("expresslrs")
     */
    uint8_t md5_digest[16];
    
    if (phrase == NULL || uid_out == NULL) {
        /* Safety: return zero UID on invalid input */
        if (uid_out != NULL) {
            memset(uid_out, 0, 6);
        }
        return;
    }
    
    /* Compute MD5 hash of the binding phrase DIRECTLY
     * This matches ExpressLRS Configurator behavior.
     */
    size_t phrase_len = strlen(phrase);
    md5_hash((const uint8_t *)phrase, phrase_len, md5_digest);
    
    /* Extract first 6 bytes as UID
     * Citation: ExpressLRS uses first 6 bytes of MD5 for UID
     * 
     * MD5 digest layout:
     * [0][1][2][3][4][5][6][7][8][9][10][11][12][13][14][15]
     *  └─────────────────┘
     *      UID bytes 0-5
     */
    memcpy(uid_out, md5_digest, 6);
}
