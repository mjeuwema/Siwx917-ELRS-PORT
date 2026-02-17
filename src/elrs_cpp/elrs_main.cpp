/**
 * @file elrs_main.cpp
 * @brief Main ELRS RX application for SiW917
 *
 * This is a working RX implementation adapted from upstream ELRS 4.0
 * for the SiW917 platform with LR1121 radio.
 * 
 * Key differences from upstream:
 * - No CRSF serial output (handled separately)
 * - No device management (LEDs, buttons handled separately)
 * - Simplified for single LR1121 radio (no Gemini/dual radio)
 */

#include "elrs_main.h"

// ELRS includes
#include "common.h"
#include "FHSS.h"
#include "LR1121Driver.h"
#include "LR1121.h"
#include "OTA.h"
#include "LQCALC.h"
#include "PFD.h"
#include "LowPassFilter.h"
#include "hwTimer.h"
#include "options.h"
#include "logging.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "stubborn_sender.h"
#include "telemetry_protocol.h"

// Standard includes
#include <string.h>
#include <algorithm>  // for std::max

// Status LED, bind button, persistent config, WiFi integration, and CRSF output
extern "C" {
#include "status_led.h"
#include "bind_button.h"
#include "elrs_config.h"
#include "wifi_http_test.h"
#include "crsf_serial.h"
}

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define PACKET_TO_TOCK_SLACK 200  // Desired buffer time between Packet ISR and Tock ISR
#define CRSF_RC_OUTPUT_INTERVAL 4   // Output RC channels every 4ms (~250Hz)
///////////////////

// Model match ID (0xFF = disabled, 0-63 = specific model)
static uint8_t modelMatchId = 0xFF;

// Link statistics (for external API)
static elrs_link_stats_t currentLinkStats = {};

// Channel callback
static elrs_channel_callback_t channelCallback = nullptr;

// Antenna tracking
uint8_t antenna = 0;
uint8_t geminiMode = 0;

// Connection timing
static const uint32_t ConsiderConnGoodMillis = 1000;

//=============================================================================
// Global state variables (required by ELRS protocol)
//=============================================================================
connectionState_e connectionState = disconnected;
uint8_t UID[UID_LEN] = {0};
bool connectionHasModelMatch = false;
bool teamraceHasModelMatch = true;  // Always true for basic RX
bool InBindingMode = false;
bool InWiFiMode = false;  // WiFi configuration mode
uint8_t ExpressLRS_currTlmDenom = 1;

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams = nullptr;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams = nullptr;

uint32_t ChannelData[CRSF_NUM_CHANNELS];

// Radio driver instance (defined in common.cpp)
extern LR1121Driver Radio;

// Link stats (defined in common.cpp)
extern elrsLinkStatistics_t linkStats;

// CRC calculator
Crc2Byte ota_crc;

// PFD (Phase Frequency Detector) for timing synchronization
PFD PFDloop;

// LQ calculation
LQCALC<100> LQCalc;
uint8_t uplinkLQ = 0;

// Low pass filters
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);
LPF LPF_UplinkRSSI0(5);
LPF LPF_UplinkRSSI1(5);

// Timing variables
uint32_t LastValidPacket = 0;
uint32_t LastSyncPacket = 0;
static uint32_t RFmodeLastCycled = 0;
static uint8_t RFmodeCycleMultiplier = 1;
static bool LockRFmode = false;

// Rate/mode scanning
static uint8_t scanIndex = 0;
uint8_t ExpressLRS_nextAirRateIndex = 0;

// Timer state
RXtimerState_e RXtimerState = tim_disconnected;
static bool doStartTimer = false;

// Connection tracking
uint32_t GotConnectionMillis = 0;

// Phase lock
int32_t PfdPrevRawOffset = 0;

// Cycle interval for rate scanning
uint32_t cycleInterval = 0;

// TLM state
static bool alreadyTLMresp = false;

// Telemetry TX (downlink to TX module)
StubbornSender TelemetrySender;
static uint8_t TelemetryBuffer[ELRS_DATA_UL_BUFFER];
static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static uint8_t telemetryBurstCount = 0;
static uint8_t telemetryBurstMax = 1;

// SNR accumulator for telemetry (simplified - just use last value)
static int8_t lastSnrRaw = 0;

// Forward declarations
static void SetRFLinkRate(uint8_t index, bool bindMode);
static void ICACHE_RAM_ATTR HWtimerCallbackTick();
static void ICACHE_RAM_ATTR HWtimerCallbackTock();
static void ICACHE_RAM_ATTR HandleFHSS();
static void ICACHE_RAM_ATTR updatePhaseLock();
static void ICACHE_RAM_ATTR getRFlinkInfo();
static uint8_t minLqForChaos();
static void LostConnection(bool resumeRx);
static void TentativeConnection(unsigned long now);
static void GotConnection(unsigned long now);
static bool ICACHE_RAM_ATTR HandleSendTelemetryResponse();
static void LinkStatsToOta(OTA_LinkStats_s * const ls);

//=============================================================================
// Channel data initialization
//=============================================================================
void ChannelDataReset()
{
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
    }
}

//=============================================================================
// UID/MAC seed helpers
//=============================================================================
uint32_t uidMacSeedGet()
{
    return ((uint32_t)UID[2] << 24) | ((uint32_t)UID[3] << 16) |
           ((uint32_t)UID[4] << 8) | UID[5];
}

//=============================================================================
// Chaos detection (noise floor)
//=============================================================================
static uint8_t minLqForChaos()
{
    const uint32_t numfhss = FHSSgetChannelCount();
    const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

//=============================================================================
// Link statistics collection
//=============================================================================
static void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int32_t rssiDBM = Radio.LastPacketRSSI;

    // Single radio - no dual antenna handling
    rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
    if (rssiDBM > 0) rssiDBM = 0;

    // BetaFlight/iNav expect positive values for -dBm
    linkStats.uplink_RSSI_1 = -rssiDBM;
    linkStats.uplink_RSSI_2 = -rssiDBM;  // Same for single radio

    // Track raw SNR for telemetry
    lastSnrRaw = Radio.LastPacketSNRRaw;

    linkStats.active_antenna = antenna;
    linkStats.uplink_SNR = Radio.LastPacketSNRRaw / RADIO_SNR_SCALE;
    linkStats.rf_Mode = ExpressLRS_currAirRate_Modparams->enum_rate;
}

//=============================================================================
// FHSS Frequency Hopping
//=============================================================================
static void ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresultFHSS = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || 
        InBindingMode || 
        (modresultFHSS != 0) || 
        (connectionState == disconnected))
    {
        return;
    }

    // Hop to next frequency
    Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, false);
}

//=============================================================================
// Telemetry TX (downlink to TX module)
//=============================================================================

/**
 * @brief Fill in link stats for telemetry response
 * Citation: ExpressLRS rx_main.cpp LinkStatsToOta()
 */
static void LinkStatsToOta(OTA_LinkStats_s * const ls)
{
    // Values are "positivized" (inverted polarity) - TX must invert back
    ls->uplink_RSSI_1 = linkStats.uplink_RSSI_1;
    ls->uplink_RSSI_2 = linkStats.uplink_RSSI_2;
    ls->antenna = antenna;
    ls->modelMatch = connectionHasModelMatch;
    ls->lq = linkStats.uplink_Link_quality;
    ls->trueDiversityAvailable = false;  // Single radio
    ls->SNR = lastSnrRaw;  // Raw SNR value
}

/**
 * @brief Send telemetry response to TX
 * 
 * Called from HWtimerCallbackTock when it's time to send telemetry.
 * Sends link statistics and any queued data back to the TX module.
 * 
 * Citation: ExpressLRS rx_main.cpp HandleSendTelemetryResponse()
 * 
 * @return true if telemetry was sent, false otherwise
 */
static bool ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    uint8_t modresult = OtaNonce % ExpressLRS_currTlmDenom;

    // Don't send telemetry if:
    // - Disconnected
    // - TLM is disabled (denom == 1)
    // - Already sent this period
    // - Not the right time slot
    // - Model doesn't match (teamraceHasModelMatch)
    if ((connectionState == disconnected) || 
        (ExpressLRS_currTlmDenom == 1) || 
        (alreadyTLMresp == true) || 
        (modresult != 0) ||
        !teamraceHasModelMatch)
    {
        return false;
    }

    // ESP requires word aligned buffer
    WORD_ALIGNED_ATTR OTA_Packet_s otaPkt;
    memset(&otaPkt, 0, sizeof(otaPkt));
    alreadyTLMresp = true;

    // Always send LINKSTATS first, then DATA if available
    if (NextTelemetryType == PACKET_TYPE_LINKSTATS || !TelemetrySender.IsActive())
    {
        // Send link statistics packet
        otaPkt.std.type = PACKET_TYPE_LINKSTATS;
        
        if (OtaIsFullRes)
        {
            // Full resolution mode (8-byte packets)
            otaPkt.full.data_dl.stubbornAck = 0;  // No uplink data receiver for now
            otaPkt.full.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
                otaPkt.full.data_dl.ul_link_stats.payload,
                ELRS8_DATA_DL_BYTES_PER_CALL - sizeof(OTA_LinkStats_s));
            LinkStatsToOta(&otaPkt.full.data_dl.ul_link_stats.stats);
        }
        else
        {
            // Standard mode (4-byte packets)
            otaPkt.std.data_dl.stubbornAck = 0;
            otaPkt.std.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
                otaPkt.std.data_dl.ul_link_stats.payload,
                ELRS4_DATA_DL_BYTES_PER_CALL - sizeof(OTA_LinkStats_s));
            LinkStatsToOta(&otaPkt.std.data_dl.ul_link_stats.stats);
        }

        NextTelemetryType = PACKET_TYPE_DATA;
        telemetryBurstCount = 1;
    }
    else
    {
        // Send data packet (if we had MSP data to send - future feature)
        otaPkt.std.type = PACKET_TYPE_DATA;
        
        if (OtaIsFullRes)
        {
            otaPkt.full.data_dl.stubbornAck = 0;
            otaPkt.full.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
                otaPkt.full.data_dl.payload,
                ELRS8_DATA_DL_BYTES_PER_CALL);
        }
        else
        {
            otaPkt.std.data_dl.stubbornAck = 0;
            otaPkt.std.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
                otaPkt.std.data_dl.payload,
                ELRS4_DATA_DL_BYTES_PER_CALL);
        }

        telemetryBurstCount++;
        if (telemetryBurstCount >= telemetryBurstMax)
        {
            NextTelemetryType = PACKET_TYPE_LINKSTATS;
        }
    }

    // Generate CRC for the packet
    OtaGeneratePacketCrc(&otaPkt);

    // Transmit the telemetry packet
    // TXnb signature: (data, sendGeminiBuffer, dataGemini, radioNumber)
    // For single radio, we don't use Gemini mode
    Radio.TXnb((uint8_t*)&otaPkt, false, nullptr, SX12XX_Radio_1);

    return true;
}

//=============================================================================
// Phase lock / PFD update
//=============================================================================
static void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (connectionState != disconnected && PFDloop.hasResult())
    {
        int32_t RawOffset = PFDloop.calcResult();
        int32_t Offset = LPF_Offset.update(RawOffset);
        int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
        PfdPrevRawOffset = RawOffset;

        if (RXtimerState == tim_locked)
        {
            // Limit rate of freq offset adjustment
            if (OtaNonce % 8 == 0)
            {
                if (Offset > 0) {
                    hwTimer::incFreqOffset();
                } else if (Offset < 0) {
                    hwTimer::decFreqOffset();
                }
            }
        }

        if (connectionState != connected) {
            hwTimer::phaseShift(RawOffset >> 1);
        } else {
            hwTimer::phaseShift(Offset >> 2);
        }

        (void)OffsetDx;  // Suppress unused warning
    }

    PFDloop.reset();
}

//=============================================================================
// Timer Tick callback - mid-packet timing
//=============================================================================
static void ICACHE_RAM_ATTR HWtimerCallbackTick()
{
    if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        // Save the LQ value before the inc() reduces it by 1
        uplinkLQ = LQCalc.getLQ();
    }

    linkStats.uplink_Link_quality = uplinkLQ;
    
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp)
        LQCalc.inc();

    alreadyTLMresp = false;
}

//=============================================================================
// Timer Tock callback - end of packet interval
//=============================================================================
static void ICACHE_RAM_ATTR HWtimerCallbackTock()
{
    PFDloop.intEvent(micros());  // Our internal osc just fired

    // Check if packet was missed
    if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        if (!LQCalc.currentIsSet())
        {
            // Packet missed - could notify FC here
        }
    }

    OtaNonce++;

    // Send telemetry response to TX (if it's time)
    // This must happen BEFORE HandleFHSS so the TX receives telemetry on the current frequency
    HandleSendTelemetryResponse();

    HandleFHSS();
    updatePhaseLock();
}

//=============================================================================
// Connection state management
//=============================================================================
static void LostConnection(bool resumeRx)
{
    DBGLN("lost conn fc=%d fo=%d", 0, hwTimer::FreqOffset);

    setConnectionState(disconnected);
    RXtimerState = tim_disconnected;
    hwTimer::resetFreqOffset();
    PfdPrevRawOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    LQCalc.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;

    if (!InBindingMode)
    {
        if (hwTimer::running)
        {
            hwTimer::stop();
        }
        SetRFLinkRate(ExpressLRS_nextAirRateIndex, false);
        if (resumeRx)
        {
            Radio.RXnb();
        }
    }
}

static void TentativeConnection(unsigned long now)
{
    PFDloop.reset();
    setConnectionState(tentative);
    connectionHasModelMatch = false;
    RXtimerState = tim_disconnected;
    DBGLN("tentative conn");
    PfdPrevRawOffset = 0;
    LPF_Offset.init(0);
    RFmodeLastCycled = now;  // Give another cycle for lock to occur
}

static void GotConnection(unsigned long now)
{
    if (connectionState == connected)
    {
        return;  // Already connected
    }

    LockRFmode = firmwareOptions.lock_on_first_connection;

    setConnectionState(connected);
    RXtimerState = tim_tentative;
    GotConnectionMillis = now;

    DBGLN("got conn");
}

//=============================================================================
// Process SYNC packet
//=============================================================================
static bool ICACHE_RAM_ATTR ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const * const otaSync)
{
    // Verify the binding ID
    if (otaSync->UID4 != UID[4])
        return false;

    if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
        return false;

    LastSyncPacket = now;

    // Will change the packet air rate in loop() if this changes
    ExpressLRS_nextAirRateIndex = enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);

    // Update TLM ratio
    expresslrs_tlm_ratio_e TLMrateIn = (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio + (uint8_t)TLM_RATIO_NO_TLM);
    uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
    if (ExpressLRS_currTlmDenom != TlmDenom)
    {
        DBGLN("New TLMrate 1:%u", TlmDenom);
        ExpressLRS_currTlmDenom = TlmDenom;
    }

    // Model match check
    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    // Must use bitwise NOT (~) to match upstream ELRS behavior
    uint8_t modelXor = (~modelMatchId) & MODELMATCH_MASK;
    bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);

    if (connectionState == disconnected
        || OtaNonce != otaSync->nonce
        || FHSSgetCurrIndex() != otaSync->fhssIndex
        || connectionHasModelMatch != modelMatched)
    {
        FHSSsetCurrIndex(otaSync->fhssIndex);
        OtaNonce = otaSync->nonce;
        TentativeConnection(now);
        connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

//=============================================================================
// Process RC data packet
//=============================================================================
static void ICACHE_RAM_ATTR ProcessRfPacket_RC(OTA_Packet_s const * const otaPktPtr)
{
    // Must be fully connected to process RC packets
    if (connectionState != connected)
        return;

    bool telemetryConfirmValue = OtaUnpackChannelData(otaPktPtr, ChannelData);
    
    // Confirm telemetry was received by TX (for stubborn sender reliability)
    TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);

    // Notify callback if registered
    if (connectionHasModelMatch && channelCallback)
    {
        channelCallback(ChannelData, CRSF_NUM_CHANNELS);
    }
}

//=============================================================================
// Main packet processing ISR
//=============================================================================

// Debug counter for CRC failures
static uint32_t crcFailCount = 0;
static uint32_t crcPassCount = 0;
static uint32_t lastCrcDebugTime = 0;

static bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
    {
        return false;
    }

    uint32_t const now = millis();

    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;

    // Validate CRC
    if (!OtaValidatePacketCrc(otaPktPtr))
    {
        crcFailCount++;
        // Print CRC debug info periodically (not every packet - too spammy)
        if ((now - lastCrcDebugTime) > 2000) {
            lastCrcDebugTime = now;
            DBGLN("CRC: %lu pass, %lu fail. Last pkt: %02X %02X %02X %02X %02X %02X",
                  crcPassCount, crcFailCount,
                  Radio.RXdataBuffer[0], Radio.RXdataBuffer[1], 
                  Radio.RXdataBuffer[2], Radio.RXdataBuffer[3],
                  Radio.RXdataBuffer[4], Radio.RXdataBuffer[5]);
            DBGLN("  OtaCrcInit=0x%04X, Nonce=%d, FullRes=%d", 
                  OtaCrcInitializer, OtaNonce, OtaIsFullRes);
        }
        return false;
    }
    
    crcPassCount++;

    // Record PFD timing - capture timestamp early for accuracy
    uint32_t const beginProcessing = micros();

    // The extEvent defines where TOCK timer ISR is to be synced to, i.e. where the packet period begins.
    // For rates where the TOA is longer than half the packet period schedule the TOCK for roughly 1x TOA before
    // the TX's end of the period so telemetry is received by the TX in the correct period. For all others,
    // schedule TOCK to be PACKET_TO_TOCK_SLACK (us) after RX packet reception.
    int32_t slack = std::max(ExpressLRS_currAirRate_Modparams->interval - 2 * ExpressLRS_currAirRate_RFperfParams->TOA, (int32_t)PACKET_TO_TOCK_SLACK);
    PFDloop.extEvent(beginProcessing + slack);

    // Get packet type (low 2 bits of first byte)
    uint8_t const type = cycleInterval? otaPktPtr->std.type : otaPktPtr->full.rc.packetType;

    // Update link quality and RSSI
    LQCalc.add();
    getRFlinkInfo();

    // Record valid packet time
    LastValidPacket = now;

    // Handle packet based on type
    switch (type)
    {
        case PACKET_TYPE_RCDATA:
            ProcessRfPacket_RC(otaPktPtr);
            break;

        case PACKET_TYPE_SYNC:
        {
            OTA_Sync_s const * const sync = OtaIsFullRes ? 
                &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync;
            bool newConnection = ProcessRfPacket_SYNC(now, sync);
            if (newConnection)
            {
                // Start the timer for phase lock
                hwTimer::resume();
            }
            break;
        }

        case PACKET_TYPE_DATA:
            // MSP/Data packets - not implemented for basic RX
            break;
    }

    // Connection state transitions
    if (connectionState == disconnected)
    {
        // Ignore non-sync packets when disconnected
    }
    else if (connectionState == tentative)
    {
        // Check for connection promotion
        if ((now - GotConnectionMillis) > ConsiderConnGoodMillis)
        {
            if (LQCalc.getLQ() > minLqForChaos())
            {
                GotConnection(now);
            }
        }
    }
    else if (connectionState == connected)
    {
        // Update timer state if needed
        if (RXtimerState == tim_tentative)
        {
            RXtimerState = tim_locked;
        }
    }

    return true;
}

//=============================================================================
// Radio ISR callbacks
//=============================================================================
static bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status rxStatus)
{
    // Already received a packet this period, don't process again (DVDA/multi-send guard)
    if (LQCalc.currentIsSet() && connectionState == connected)
    {
        return false;
    }
    return ProcessRFPacket(rxStatus);
}

static void ICACHE_RAM_ATTR TXdoneISR()
{
    // After sending telemetry, switch back to RX
    Radio.RXnb();
}

//=============================================================================
// RF Link rate setting
//=============================================================================
static void SetRFLinkRate(uint8_t index, bool bindMode)
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    if (!ModParams || !RFperf) {
        DBGLN("Invalid rate index: %d", index);
        return;
    }

    // Binding always uses invertIQ
    bool invertIQ = bindMode || (UID[5] & 0x01);

    uint32_t interval = ModParams->interval;
    hwTimer::updateInterval(interval);

    // Configure FHSS band selection
    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && 
                             !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = false;  // No dual band support

    // Configure radio
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength,
                 ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 || 
                 ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4,
                 (uint8_t)UID[5], (uint8_t)UID[4]);

    // Update OTA serializers
    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);

    // Update telemetry sender for new packet size
    TelemetrySender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES : ELRS4_DATA_DL_MAX_PACKAGES);

    // Calculate cycle interval for rate scanning
    cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index;

    // Reset telemetry state on rate change
    NextTelemetryType = PACKET_TYPE_LINKSTATS;
    telemetryBurstCount = 0;

    DBGLN("Set RF rate index %d, interval %lu us", index, interval);
}

//=============================================================================
// Rate cycling for connection scanning
//=============================================================================
static void cycleRfMode()
{
    if (LockRFmode) return;

    uint32_t now = millis();

    if ((now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier)) {
        RFmodeLastCycled = now;

        // Find next valid rate
        do {
            scanIndex = (scanIndex + 1) % RATE_MAX;
        } while (!isSupportedRFRate(scanIndex));

        SetRFLinkRate(scanIndex, InBindingMode);
        Radio.RXnb();

        DBGLN("Cycling to rate index %d", scanIndex);
    }
}

//=============================================================================
// Bind Button Callback
//=============================================================================

// External function to request WiFi mode (defined in gspi_example.c)
// This sets a flag that's checked in the main task loop
extern "C" void elrs_cpp_request_wifi_mode(void);

/**
 * @brief Callback when bind button is pressed/released
 * @param long_press true if button was held >= 3 seconds
 * 
 * Button behavior:
 *   Short press: Request WiFi configuration mode (handled in main task)
 *   Long press (3+ seconds): Toggle binding mode
 * 
 * Note: WiFi mode must be started from main task context (not callback)
 * to ensure osDelay() works properly. This matches the working C implementation.
 */
static void onBindButtonEvent(bool long_press)
{
    if (long_press) {
        // Long press = toggle binding mode
        DBGLN("Bind button long press - toggling binding mode");
        if (!InBindingMode) {
            elrs_enter_binding_mode();
            status_led_set_mode(LED_MODE_BINDING);
        } else {
            elrs_exit_binding_mode();
            status_led_set_mode(LED_MODE_DISCONNECTED);
        }
    } else {
        // Short press = request WiFi mode (handled in main task loop)
        DBGLN("Bind button short press - requesting WiFi mode");
        if (!InWiFiMode) {
            // Set flag - main task will handle starting WiFi
            elrs_cpp_request_wifi_mode();
        }
    }
}

//=============================================================================
// Public API implementation
//=============================================================================

extern "C" {

bool elrs_init(void)
{
    DBGLN("ELRS RX init starting...");

    // Initialize status LEDs first for visual feedback
    status_led_init();
    status_led_set_mode(LED_MODE_DISCONNECTED);

    // Initialize bind button
    bind_button_init();
    bind_button_set_callback(onBindButtonEvent);

    // Initialize persistent config from NVM3 (must be before options_init)
    // Note: sl_net_init() must be called before this for NVM3 access
    if (elrs_config_init() != 0) {
        DBGLN("WARNING: Config init failed, using defaults");
    }

    // Initialize options/configuration (loads UID from elrs_config)
    if (!options_init()) {
        DBGLN("Options init failed!");
        return false;
    }

    // Copy UID from firmware options
    memcpy(UID, firmwareOptions.uid, UID_LEN);
    DBGLN("UID: %02X:%02X:%02X:%02X:%02X:%02X",
          UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

    // Initialize channel data
    ChannelDataReset();

    // Initialize OTA CRC
    OtaUpdateCrcInitFromUid();

    // Initialize telemetry sender
    TelemetrySender.ResetState();
    TelemetrySender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES : ELRS4_DATA_DL_MAX_PACKAGES);
    
    // Initialize FHSS
    FHSSrandomiseFHSSsequence(uidMacSeedGet());
    DBGLN("FHSS initialized with %d channels", FHSSgetChannelCount());

    // Initialize radio with frequency range from FHSS config
    bool radioOk = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    if (!radioOk) {
        DBGLN("Radio init failed!");
        connectionState = radioFailed;
        return false;
    }

    // Set up radio callbacks
    Radio.RXdoneCallback = RXdoneISR;
    Radio.TXdoneCallback = TXdoneISR;

    // Set initial rate (start with binding rate for scan - slowest for best range)
    scanIndex = RATE_BINDING;
    SetRFLinkRate(scanIndex, false);

    // Start receiving
    Radio.RXnb();
    
    // Initialize hardware timer
    hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);

    // Record start time for rate cycling
    RFmodeLastCycled = millis();

    // Initialize CRSF serial output to flight controller
    // Uses USART0 at 420000 baud (ELRS standard)
    if (crsf_serial_init(420000) != 0) {
        DBGLN("WARNING: CRSF serial init failed - no FC output");
    } else {
        DBGLN("CRSF serial output initialized");
    }

    // Load model match ID from config (0xFF = disabled)
    elrs_config_t* cfg = elrs_config_get();
    if (cfg) {
        modelMatchId = cfg->model_id;
        DBGLN("Model match ID: %d (%s)", modelMatchId, 
              modelMatchId == 0xFF ? "disabled" : "enabled");
    }

    DBGLN("ELRS RX init complete");
    return true;
}

void elrs_loop(void)
{
    unsigned long now = millis();

    // Rate cycling when disconnected
    if (connectionState == disconnected) {
        cycleRfMode();
    }

    // Connection timeout check
    if (connectionState != disconnected)
    {
        uint32_t disconnectTimeout = ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs;
        if ((now - LastValidPacket) > disconnectTimeout)
        {
            LostConnection(true);
        }
    }

    // Rate change handling
    if (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index)
    {
        LostConnection(false);
        SetRFLinkRate(ExpressLRS_nextAirRateIndex, false);
        Radio.RXnb();
    }

    // Send CRSF RC channels to flight controller when connected
    static uint32_t lastRcOutput = 0;
    if ((connectionState == connected) && connectionHasModelMatch && 
        (now - lastRcOutput) >= CRSF_RC_OUTPUT_INTERVAL) {
        lastRcOutput = now;
        
        if (crsf_serial_is_ready()) {
            crsf_serial_send_channels(ChannelData);
        }
    }

    // Update external link stats and send to FC periodically
    static uint32_t lastLinkStatsUpdate = 0;
    if ((now - lastLinkStatsUpdate) > SEND_LINK_STATS_TO_FC_INTERVAL) {
        lastLinkStatsUpdate = now;

        currentLinkStats.rssi_1 = linkStats.uplink_RSSI_1;
        currentLinkStats.rssi_2 = linkStats.uplink_RSSI_2;
        currentLinkStats.snr = linkStats.uplink_SNR;
        currentLinkStats.lq = uplinkLQ;
        currentLinkStats.active_ant = antenna;
        if (ExpressLRS_currAirRate_Modparams) {
            currentLinkStats.rf_mode = ExpressLRS_currAirRate_Modparams->index;
        }
        
        // Send link stats to FC via CRSF
        if (crsf_serial_is_ready() && connectionState == connected) {
            crsf_link_stats_t crsfStats;
            crsfStats.uplink_rssi_1 = linkStats.uplink_RSSI_1;
            crsfStats.uplink_rssi_2 = linkStats.uplink_RSSI_2;
            crsfStats.uplink_lq = uplinkLQ;
            crsfStats.uplink_snr = linkStats.uplink_SNR;
            crsfStats.active_antenna = antenna;
            crsfStats.rf_mode = ExpressLRS_currAirRate_Modparams ? 
                                ExpressLRS_currAirRate_Modparams->index : 0;
            crsfStats.uplink_tx_power = linkStats.uplink_TX_Power;
            crsfStats.downlink_rssi = 0;  // RX doesn't have downlink stats
            crsfStats.downlink_lq = 0;
            crsfStats.downlink_snr = 0;
            crsf_serial_send_link_stats(&crsfStats);
        }
    }

    // Update status LED based on connection state
    static connectionState_e lastLedState = disconnected;
    if (connectionState != lastLedState) {
        lastLedState = connectionState;
        switch (connectionState) {
            case connected:
                status_led_set_mode(LED_MODE_CONNECTED);
                break;
            case tentative:
                status_led_set_mode(LED_MODE_TENTATIVE);
                break;
            case disconnected:
                status_led_set_mode(LED_MODE_DISCONNECTED);
                break;
            case radioFailed:
                status_led_set_mode(LED_MODE_ERROR);
                break;
            default:
                break;
        }
    }

    // Update LED blinking patterns
    status_led_update();

    // Poll bind button for long-press detection
    bind_button_poll();
    
    // Print ISR stats periodically for debugging
    static uint32_t lastIsrStatsPrint = 0;
    if ((now - lastIsrStatsPrint) > 5000) {
        lastIsrStatsPrint = now;
        uint32_t isr_count, rx_count, tx_count, other_count, last_irq;
        void lr1121_get_isr_stats(uint32_t*, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
        lr1121_get_isr_stats(&isr_count, &rx_count, &tx_count, &other_count, &last_irq);
        DBGLN("ISR stats: calls=%lu, RX=%lu, TX=%lu, other=%lu, lastIRQ=0x%04lX",
              isr_count, rx_count, tx_count, other_count, last_irq);
        
        // Debug: Check DIO1 pin state and poll IRQ status
        int lr1121_dio1_read(void);
        int dio1_pin = lr1121_dio1_read();
        uint32_t polled_irq = Radio.GetIrqStatus(SX12XX_Radio_1);
        DBGLN("DIO1 pin=%d, Polled IRQ=0x%08lX", dio1_pin, (unsigned long)polled_irq);
        
        // If IRQ is set but ISR didn't fire, we have a DIO1 routing problem
        if (polled_irq != 0 && isr_count == 0) {
            DBGLN("WARNING: IRQ set but ISR never fired - DIO1 not connected?");
        }
    }
}

elrs_connection_state_t elrs_get_connection_state(void)
{
    switch (connectionState) {
        case connected:     return ELRS_CONNECTED;
        case tentative:     return ELRS_TENTATIVE;
        case disconnected:  return ELRS_DISCONNECTED;
        case radioFailed:   return ELRS_RADIO_FAILED;
        default:            return ELRS_DISCONNECTED;
    }
}

bool elrs_is_connected(void)
{
    return (connectionState == connected || connectionState == tentative);
}

uint8_t elrs_get_channels(uint32_t *channels)
{
    if (channels) {
        memcpy(channels, ChannelData, sizeof(ChannelData));
    }
    return CRSF_NUM_CHANNELS;
}

void elrs_get_link_stats(elrs_link_stats_t *stats)
{
    if (stats) {
        *stats = currentLinkStats;
    }
}

void elrs_set_channel_callback(elrs_channel_callback_t callback)
{
    channelCallback = callback;
}

void elrs_enter_binding_mode(void)
{
    InBindingMode = true;
    SetRFLinkRate(RATE_BINDING, true);
    DBGLN("Entering binding mode");
}

void elrs_exit_binding_mode(void)
{
    InBindingMode = false;
    SetRFLinkRate(scanIndex, false);
    DBGLN("Exiting binding mode");
}

void elrs_enter_wifi_mode(void)
{
    if (InWiFiMode) {
        DBGLN("Already in WiFi mode");
        return;
    }
    
    DBGLN("Entering WiFi configuration mode...");
    InWiFiMode = true;
    
    // Stop the radio to free up resources
    hwTimer::stop();
    
    // Set LED to WiFi mode (fast blink LED1)
    status_led_set_mode(LED_MODE_WIFI);
    
    // Start WiFi AP and HTTP server
    // Note: This function blocks and handles HTTP requests in a loop
    // Device will reset when user triggers reboot from web UI
    wifi_http_test_run();
    
    // If we ever return from WiFi mode (shouldn't normally happen)
    // Note: Normally device reboots from WiFi web UI, so this code path 
    // is only reached if wifi_http_test_run() returns unexpectedly
    InWiFiMode = false;
    status_led_set_mode(LED_MODE_DISCONNECTED);
    DBGLN("Exited WiFi mode - recommend rebooting to restore full functionality");
}

bool elrs_is_wifi_mode(void)
{
    return InWiFiMode;
}

bool elrs_set_uid(const uint8_t new_uid[6])
{
    if (new_uid == nullptr) {
        return false;
    }
    
    // Update runtime UID
    memcpy(UID, new_uid, UID_LEN);
    memcpy(firmwareOptions.uid, new_uid, UID_LEN);
    
    // Save to persistent storage
    elrs_config_set_uid(new_uid);
    int result = elrs_config_save();
    
    if (result == 0) {
        DBGLN("New UID saved: %02X:%02X:%02X:%02X:%02X:%02X",
              UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
        
        // Re-initialize FHSS with new UID
        OtaUpdateCrcInitFromUid();
        FHSSrandomiseFHSSsequence(uidMacSeedGet());
        
        return true;
    } else {
        DBGLN("ERROR: Failed to save UID to NVM3");
        return false;
    }
}

uint8_t elrs_get_lq(void)
{
    return uplinkLQ;
}

const char* elrs_get_rate_name(void)
{
    if (!ExpressLRS_currAirRate_Modparams) {
        return "Unknown";
    }

    switch (ExpressLRS_currAirRate_Modparams->enum_rate) {
        case RATE_LORA_900_50HZ:   return "900M 50Hz";
        case RATE_LORA_900_100HZ:  return "900M 100Hz";
        case RATE_LORA_900_200HZ:  return "900M 200Hz";
        case RATE_LORA_900_250HZ:  return "900M 250Hz";
        case RATE_LORA_2G4_50HZ:   return "2.4G 50Hz";
        case RATE_LORA_2G4_150HZ:  return "2.4G 150Hz";
        case RATE_LORA_2G4_250HZ:  return "2.4G 250Hz";
        case RATE_LORA_2G4_500HZ:  return "2.4G 500Hz";
        default:                   return "Custom";
    }
}

//=============================================================================
// Legacy C API wrappers for gspi_example.c compatibility
//=============================================================================

void elrs_rx_test(void)
{
    DBGLN("=== ELRS C++ Integration Test ===");
    DBGLN("Platform: SiW917");
    DBGLN("Radio: LR1121 (Waveshare Core1121-HF)");
    DBGLN("Mode: 900MHz Sub-GHz");
    DBGLN("UID: 0x%02X%02X%02X%02X%02X%02X",
          UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
    DBGLN("=== End Integration Test ===");
}

void elrs_rx_init(void)
{
    elrs_init();
}

void elrs_rx_start(void)
{
    DBGLN("ELRS RX Started");
}

void elrs_rx_stop(void)
{
    DBGLN("ELRS RX Stopped");
    hwTimer::stop();
}

void elrs_rx_loop(void)
{
    elrs_loop();
}

} // extern "C"
