/**
 * @file ota.c
 * @brief ELRS Over-The-Air Packet Encoding/Decoding
 * 
 * EXACT PORT from ELRS 4.0 src/lib/OTA/OTA.cpp
 * Original: https://github.com/ExpressLRS/ExpressLRS
 * 
 * This file is intentionally kept as close to the original C++ as possible
 * to ensure bit-for-bit compatibility. Only minimal changes for C compilation.
 */

#include "ota.h"
#include "crc.h"
#include <string.h>

/*******************************************************************************
 * Global Variables - Match ELRS exactly
 ******************************************************************************/

bool OtaIsFullRes;
volatile uint8_t OtaNonce;
uint16_t OtaCrcInitializer;
OtaSwitchMode_e OtaSwitchModeCurrent;

#if defined(TARGET_RX)
bool isArmed;
#endif

/*******************************************************************************
 * Static CRC Context
 ******************************************************************************/

static elrs_crc16_t ota_crc;

/*******************************************************************************
 * Function Pointers - Match ELRS exactly
 ******************************************************************************/

static bool ValidatePacketCrcFull(OTA_Packet_s *otaPktPtr);
static bool ValidatePacketCrcStd(OTA_Packet_s *otaPktPtr);
static void GeneratePacketCrcFull(OTA_Packet_s *otaPktPtr);
static void GeneratePacketCrcStd(OTA_Packet_s *otaPktPtr);

ValidatePacketCrc_t OtaValidatePacketCrc;
GeneratePacketCrc_t OtaGeneratePacketCrc;

#if defined(TARGET_RX)
static bool UnpackChannelDataHybridSwitch8(OTA_Packet_s const *otaPktPtr, uint32_t *channelData);
static bool UnpackChannelDataHybridWide(OTA_Packet_s const *otaPktPtr, uint32_t *channelData);
static bool UnpackChannelData8ch(OTA_Packet_s const *otaPktPtr, uint32_t *channelData);
UnpackChannelData_t OtaUnpackChannelData;
#endif

/*******************************************************************************
 * OtaUpdateCrcInitFromUid - EXACT copy from ELRS OTA.cpp lines 28-39
 ******************************************************************************/

void OtaUpdateCrcInitFromUid(void)
{
#if OTA_VERSION_ID > 15
#error "OTA version can't be > 15"
#endif

    OtaCrcInitializer = (UID[4] << 8) | UID[5];

    // shift OTA_VERSION_ID to the high byte to leave room for
    // xor-ing in the nonce in the GenerateCRC and ValidateCRC function
    OtaCrcInitializer ^= (uint16_t)OTA_VERSION_ID << 8;
    
    /* Debug: Print UID and CRC init for verification */
    extern int printf(const char *, ...);
    printf("[OTA] UID = %02X:%02X:%02X:%02X:%02X:%02X\n",
           UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
    printf("[OTA] CRC Init = 0x%04X (UID[4:5]=0x%02X%02X ^ OTA_VER=%d)\n",
           OtaCrcInitializer, UID[4], UID[5], OTA_VERSION_ID);
}

/*******************************************************************************
 * HybridWideNonceToSwitchIndex - EXACT copy from ELRS OTA.cpp lines 41-51
 ******************************************************************************/

static inline uint8_t HybridWideNonceToSwitchIndex(uint8_t nonce)
{
    // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
    // 0, 1, 2, 3, 4, 5, 6, 7,
    // 1, 2, 3, 4, 5, 6, 7, 0
    // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
    // this makes sure each of the 8 values is sent at least once every 16 packets
    // regardless of the TLM ratio
    // Index 7 also can never fall on a telemetry slot
    return ((nonce & 0b111) + ((nonce >> 3) & 0b1)) % 8;
}

/*******************************************************************************
 * RX Channel Unpacking - EXACT copy from ELRS OTA.cpp
 ******************************************************************************/

#if defined(TARGET_RX)

static void UnpackChannels4x10ToUInt11(OTA_Channels_4x10 const *srcChannels4x10, uint32_t *dest)
{
    uint8_t const *payload = (uint8_t const *)srcChannels4x10;
    const unsigned numOfChannels = 4;
    const unsigned srcBits = 10;
    const unsigned dstBits = 11;
    const unsigned inputChannelMask = (1 << srcBits) - 1;
    const unsigned precisionShift = dstBits - srcBits;

    // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    unsigned readByteIndex = 0;
    for (uint8_t n = 0; n < numOfChannels; n++)
    {
        while (bitsMerged < srcBits)
        {
            uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t)readByte) << bitsMerged;
            bitsMerged += 8;
        }
        dest[n] = (readValue & inputChannelMask) << precisionShift;
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }
}

static void UnpackChannelDataHybridCommon(OTA_Packet4_s const *ota4, uint32_t *channelData)
{
    isArmed = ota4->rc.isArmed;

    // The analog channels, encoded as 10bit where 0 = 998us and 1023 = 2012us
    uint32_t rawChannelData[4];
    UnpackChannels4x10ToUInt11(&ota4->rc.ch, rawChannelData);
    // The unpacker simply does a << 1 to convert 10 to 11bit, but Hybrid/Wide modes
    // only pack a subset of the full range CRSF data, so properly expand it
    for (unsigned ch = 0; ch < 4; ++ch)
    {
        channelData[ch] = UINT10_to_CRSF(rawChannelData[ch] >> 1);
    }
    channelData[4] = BIT_to_CRSF(isArmed);
    // Copy the armed flag into CH14/AUX10 for consistency with fullres
    channelData[13] = channelData[4];
}

/**
 * Hybrid switches decoding - EXACT copy from ELRS OTA.cpp lines 361-384
 */
static bool UnpackChannelDataHybridSwitch8(OTA_Packet_s const *otaPktPtr, uint32_t *channelData)
{
    OTA_Packet4_s const *ota4 = (OTA_Packet4_s const *)otaPktPtr;
    UnpackChannelDataHybridCommon(ota4, channelData);

    // The round-robin switch, switchIndex is actually index-1
    const uint8_t switchByte = ota4->rc.switches;
    uint8_t switchIndex = (switchByte & 0b111000) >> 3;
    if (switchIndex >= 6)
    {
        // AUX1 (index 0) is low latency, low bit can be used as data
        channelData[11] = N_to_CRSF(switchByte & 0b1111, 15);
    }
    else
    {
        channelData[5 + switchIndex] = SWITCH3b_to_CRSF(switchByte & 0b111);
    }

    // stubbornAck bit
    return switchByte & (1 << 6);
}

/**
 * HybridWide switches decoding - EXACT copy from ELRS OTA.cpp lines 398-418
 */
static bool UnpackChannelDataHybridWide(OTA_Packet_s const *otaPktPtr, uint32_t *channelData)
{
    OTA_Packet4_s const *ota4 = (OTA_Packet4_s const *)otaPktPtr;
    UnpackChannelDataHybridCommon(ota4, channelData);

    // The round-robin switch, 6-7 bits with the switch index implied by the nonce
    const uint8_t switchByte = ota4->rc.switches;
    uint8_t switchIndex = HybridWideNonceToSwitchIndex(OtaNonce);
    bool stubbornAck = (switchByte & 0b01000000) >> 6;
    if (switchIndex == 7)
    {
        // TX power slot - store in linkStats (handled externally)
    }
    else
    {
        uint16_t switchValue = switchByte & 0b111111; // 6-bit
        channelData[5 + switchIndex] = N_to_CRSF(switchValue, 63);
    }

    return stubbornAck;
}

/**
 * 8ch/12ch/16ch channel decoding - EXACT copy from ELRS OTA.cpp lines 420-461
 */
static bool UnpackChannelData8ch(OTA_Packet_s const *otaPktPtr, uint32_t *channelData)
{
    OTA_Packet8_s const *ota8 = (OTA_Packet8_s const *)otaPktPtr;

    isArmed = ota8->rc.isArmed;

    uint8_t chDstLow;
    uint8_t chDstHigh;
    if (OtaSwitchModeCurrent == smHybridOr16ch)
    {
        if (ota8->rc.isHighAux)
        {
            chDstLow = 8;
            chDstHigh = 12;
        }
        else
        {
            chDstLow = 0;
            chDstHigh = 4;
        }
    }
    else
    {
        chDstLow = 0;
        chDstHigh = (ota8->rc.isHighAux) ? 8 : 4;
        // For 8ch and 12ch mode, Arm status is placed in CH14/AUX10
        channelData[13] = BIT_to_CRSF(isArmed);
    }

    // Analog channels packed 10bit covering the entire CRSF extended range
    UnpackChannels4x10ToUInt11(&ota8->rc.chLow, &channelData[chDstLow]);
    UnpackChannels4x10ToUInt11(&ota8->rc.chHigh, &channelData[chDstHigh]);

    return ota8->rc.stubbornAck;
}

#endif /* TARGET_RX */

/*******************************************************************************
 * CRC Validation - EXACT copy from ELRS OTA.cpp lines 464-484
 ******************************************************************************/

static bool ValidatePacketCrcFull(OTA_Packet_s *otaPktPtr)
{
    uint16_t nonceValidator = (otaPktPtr->std.type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    uint16_t calculatedCRC = elrs_crc16_calc(&ota_crc, 
                                              (uint8_t*)otaPktPtr, 
                                              OTA8_CRC_CALC_LEN, 
                                              OtaCrcInitializer ^ nonceValidator);
    return otaPktPtr->full.crc == calculatedCRC;
}

static bool ValidatePacketCrcStd(OTA_Packet_s *otaPktPtr)
{
    uint16_t inCRC = ((uint16_t)otaPktPtr->std.crcHigh << 8) + otaPktPtr->std.crcLow;

    // Zero the crcHigh bits, as the CRC is calculated before it is ORed in
    otaPktPtr->std.crcHigh = 0;

    uint16_t nonceValidator = (otaPktPtr->std.type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    uint16_t calculatedCRC = elrs_crc16_calc(&ota_crc, 
                                              (uint8_t*)otaPktPtr, 
                                              OTA4_CRC_CALC_LEN, 
                                              OtaCrcInitializer ^ nonceValidator);

    return inCRC == calculatedCRC;
}

/*******************************************************************************
 * CRC Generation - EXACT copy from ELRS OTA.cpp lines 486-498
 ******************************************************************************/

static void GeneratePacketCrcFull(OTA_Packet_s *otaPktPtr)
{
    uint16_t nonceValidator = (otaPktPtr->std.type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    otaPktPtr->full.crc = elrs_crc16_calc(&ota_crc, 
                                           (uint8_t*)otaPktPtr, 
                                           OTA8_CRC_CALC_LEN, 
                                           OtaCrcInitializer ^ nonceValidator);
}

static void GeneratePacketCrcStd(OTA_Packet_s *otaPktPtr)
{
    uint16_t nonceValidator = (otaPktPtr->std.type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    uint16_t crc = elrs_crc16_calc(&ota_crc, 
                                    (uint8_t*)otaPktPtr, 
                                    OTA4_CRC_CALC_LEN, 
                                    OtaCrcInitializer ^ nonceValidator);
    otaPktPtr->std.crcHigh = (crc >> 8);
    otaPktPtr->std.crcLow = crc;
}

/*******************************************************************************
 * OtaUpdateSerializers - EXACT copy from ELRS OTA.cpp lines 500-549
 ******************************************************************************/

void OtaUpdateSerializers(OtaSwitchMode_e switchMode, uint8_t packetSize)
{
    OtaIsFullRes = (packetSize == OTA8_PACKET_SIZE);

    if (OtaIsFullRes)
    {
        OtaValidatePacketCrc = &ValidatePacketCrcFull;
        OtaGeneratePacketCrc = &GeneratePacketCrcFull;
        elrs_crc16_init(&ota_crc, 16, ELRS_CRC16_POLY);

#if defined(TARGET_RX)
        OtaUnpackChannelData = &UnpackChannelData8ch;
#endif
    }
    else
    {
        OtaValidatePacketCrc = &ValidatePacketCrcStd;
        OtaGeneratePacketCrc = &GeneratePacketCrcStd;
        elrs_crc16_init(&ota_crc, 14, ELRS_CRC14_POLY);

        if (switchMode == smWideOr8ch)
        {
#if defined(TARGET_RX)
            OtaUnpackChannelData = &UnpackChannelDataHybridWide;
#endif
        }
        else
        {
#if defined(TARGET_RX)
            OtaUnpackChannelData = &UnpackChannelDataHybridSwitch8;
#endif
        }
    }

    OtaSwitchModeCurrent = switchMode;
}
