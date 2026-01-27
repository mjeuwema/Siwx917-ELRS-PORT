/**
 * @file elrs_protocol.h
 * @brief Master include for ELRS Protocol on SiW917
 * 
 * Include this single header to get access to all ELRS protocol functionality.
 * 
 * Ported from ExpressLRS 4.0
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * This port provides bit-for-bit compatibility with ESP32 ELRS devices,
 * allowing SiW917-based receivers to communicate with standard ELRS transmitters.
 * 
 * Usage:
 *   1. Include this header
 *   2. Set UID from binding phrase (or use binding process)
 *   3. Call OtaUpdateCrcInitFromUid() after setting UID
 *   4. Call FHSSrandomiseFHSSsequence(seed) with UID-derived seed
 *   5. Call OtaUpdateSerializers() with initial switch mode and packet size
 *   6. Use OtaValidatePacketCrc() to validate received packets
 *   7. Use OtaUnpackChannelData() to decode channel data
 *   8. Use FHSSgetNextFreq() to hop frequencies
 */

#ifndef ELRS_PROTOCOL_H
#define ELRS_PROTOCOL_H

/* Platform compatibility layer */
#include "elrs_platform.h"

/* CRC calculation */
#include "crc.h"

/* Pseudo-random number generator */
#include "random.h"

/* Frequency hopping */
#include "fhss.h"

/* CRSF protocol definitions */
#include "crsf_protocol.h"

/* Telemetry constants */
#include "telemetry_protocol.h"

/* OTA packet encoding/decoding */
#include "ota.h"

/* LR1121 register definitions and HAL */
#include "lr1121_regs.h"
#include "lr1121_hal.h"

/* MD5 for binding phrase compatibility */
#include "md5.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize ELRS protocol stack
 * 
 * Call this once at startup after setting configuration.
 * 
 * @param domain Regulatory domain (DOMAIN_FCC915, DOMAIN_EU868, etc.)
 * @param uid 6-byte UID array (from binding phrase)
 * @return 0 on success, negative on error
 */
static inline int elrs_protocol_init(uint8_t domain, const uint8_t *uid)
{
    /* Set firmware options */
    firmwareOptions.domain = domain;
    
    /* Copy UID */
    for (int i = 0; i < UID_LEN; i++)
    {
        UID[i] = uid[i];
        firmwareOptions.uid[i] = uid[i];
    }
    firmwareOptions.hasUID = 1;
    
    /* Initialize CRC from UID */
    OtaUpdateCrcInitFromUid();
    
    /* Generate FHSS sequence */
    /* Seed is derived from UID - same calculation as ELRS */
    uint32_t seed = ((uint32_t)UID[2] << 24) | ((uint32_t)UID[3] << 16) |
                    ((uint32_t)UID[4] << 8) | (uint32_t)UID[5];
    FHSSrandomiseFHSSsequence(seed);
    
    /* Default to standard mode */
    OtaUpdateSerializers(smWideOr8ch, OTA4_PACKET_SIZE);
    
    return 0;
}

/**
 * @brief Convert binding phrase to UID
 * 
 * Uses the same MD5 algorithm as official ELRS to derive 6-byte UID from phrase.
 * This ensures bit-for-bit compatibility with ELRS transmitters.
 * 
 * Algorithm (matching ExpressLRS exactly):
 *   UID[0:5] = MD5(binding_phrase)[0:5]
 * 
 * Example:
 *   Phrase "expresslrs" -> UID: 65,245,33,230,58,226 (0x41,0xF5,0x21,0xE6,0x3A,0xE2)
 * 
 * @param phrase Null-terminated binding phrase string
 * @param uid Output 6-byte UID array
 */
static inline void elrs_uid_from_phrase(const char *phrase, uint8_t *uid)
{
    /* Use MD5 hash for full ELRS compatibility */
    elrs_md5_uid_from_phrase(phrase, uid);
}

#ifdef __cplusplus
}
#endif

#endif /* ELRS_PROTOCOL_H */
