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
#include "FHSS.h"
#include "LQCALC.h"
#include "LR1121.h"
#include "LR1121Driver.h"
#include "LowPassFilter.h"
#include "OTA.h"
#include "PFD.h"
#include "common.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "hwTimer.h"
#include "logging.h"
#include "options.h"
#include "stubborn_sender.h"
#include "telemetry_protocol.h"

// Standard includes
#include <string.h>

// Status LED, bind button, persistent config, WiFi integration, and CRSF output
extern "C" {
#include "bind_button.h"
#include "crsf_serial.h"
#include "elrs_config.h"
#include "status_led.h"
#include "wifi_http_test.h"
}

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define PACKET_TO_TOCK_SLACK                                                   \
  200 // Desired buffer time between Packet ISR and Tock ISR
#define CRSF_RC_OUTPUT_INTERVAL 4 // Output RC channels every 4ms (~250Hz)
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
bool teamraceHasModelMatch = true; // Always true for basic RX
bool InBindingMode = false;
bool InWiFiMode = false; // WiFi configuration mode
uint8_t ExpressLRS_currTlmDenom = 1;

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams = nullptr;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams = nullptr;

uint32_t ChannelData[CRSF_NUM_CHANNELS];

// Radio driver instance (defined in common.cpp)
extern LR1121Driver Radio;

// Hardware Abstraction Layer instance
static LR1121Hal radioHal;

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

// Telemetry TX (downlink to TX module)
StubbornSender TelemetrySender;
static uint8_t TelemetryBuffer[ELRS_DATA_UL_BUFFER];
static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static uint8_t telemetryBurstCount = 0;
static uint8_t telemetryBurstMax = 1;

// SNR accumulator for telemetry (simplified - just use last value)
static int8_t lastSnrRaw = 0;

// --- PACKET CAPTURE BUFFER ---
#define PKT_CAPTURE_SIZE 16
volatile uint8_t pkt_capture_buf[PKT_CAPTURE_SIZE][8];
volatile uint8_t pkt_capture_len[PKT_CAPTURE_SIZE];
volatile uint32_t pkt_capture_idx = 0;
volatile uint32_t pkt_capture_count = 0;
volatile bool do_packet_dump = false;

extern "C" void trigger_packet_dump() { do_packet_dump = true; }

extern "C" void dump_capture_buffer() {
  DBGLN("=========================================");
  DBGLN("  PACKET CAPTURE BUFFER DUMP");
  DBGLN("=========================================");
  uint32_t total = pkt_capture_count;
  uint32_t max_print = total > PKT_CAPTURE_SIZE ? PKT_CAPTURE_SIZE : total;
  uint32_t start_idx =
      total > PKT_CAPTURE_SIZE ? (pkt_capture_idx % PKT_CAPTURE_SIZE) : 0;

  DBGLN("Total packets captured: %lu", total);

  for (uint32_t i = 0; i < max_print; i++) {
    uint32_t idx = (start_idx + i) % PKT_CAPTURE_SIZE;
    DBGLN("  [%02lu] RAW: %02X %02X %02X %02X %02X %02X %02X %02X",
          (unsigned long)i, pkt_capture_buf[idx][0], pkt_capture_buf[idx][1],
          pkt_capture_buf[idx][2], pkt_capture_buf[idx][3],
          pkt_capture_buf[idx][4], pkt_capture_buf[idx][5],
          pkt_capture_buf[idx][6], pkt_capture_buf[idx][7]);
  }
  DBGLN("=========================================");
}
// -----------------------------

// Forward declarations
void SetMode(connectionState_e NewMode);
static void SetRFLinkRate(uint8_t index, bool bindMode);

// DMA debug test removed
extern bool SerialrxUpdatePacketComplete; // This seems to be a new declaration

// Hardware interrupt flags from LR1121_hal.cpp
extern volatile uint32_t isr_1_pending_count;
extern volatile uint32_t isr_2_pending_count;

static void ICACHE_RAM_ATTR getRFlinkInfo();

//=============================================================================
// Channel data initialization
//=============================================================================
void ChannelDataReset() {
  for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
    ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
  }
}

//=============================================================================
// UID/MAC seed helpers
//=============================================================================
uint32_t uidMacSeedGet() {
  return ((uint32_t)UID[2] << 24) | ((uint32_t)UID[3] << 16) |
         ((uint32_t)UID[4] << 8) | UID[5];
}

//=============================================================================
// Chaos detection (noise floor)
//=============================================================================

//=============================================================================
// Link statistics collection
//=============================================================================
static void ICACHE_RAM_ATTR getRFlinkInfo() {
  int32_t rssiDBM = Radio.LastPacketRSSI;

  // Single radio - no dual antenna handling
  rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
  if (rssiDBM > 0)
    rssiDBM = 0;

  // BetaFlight/iNav expect positive values for -dBm
  linkStats.uplink_RSSI_1 = -rssiDBM;
  linkStats.uplink_RSSI_2 = -rssiDBM; // Same for single radio

  // Track raw SNR for telemetry
  lastSnrRaw = Radio.LastPacketSNRRaw;

  linkStats.active_antenna = antenna;
  linkStats.uplink_SNR = Radio.LastPacketSNRRaw / RADIO_SNR_SCALE;
  linkStats.rf_Mode = ExpressLRS_currAirRate_Modparams->enum_rate;
}

//=============================================================================
// Connection state management
//=============================================================================

//=============================================================================
// Process SYNC packet
//=============================================================================
static bool ICACHE_RAM_ATTR
ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const *const otaSync) {
  // Verify the binding ID
  if (otaSync->UID4 != UID[4])
    return false;

  if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
    return false;

  LastSyncPacket = now;

  // Will change the packet air rate in loop() if this changes
  ExpressLRS_nextAirRateIndex =
      enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);

  // Update TLM ratio
  expresslrs_tlm_ratio_e TLMrateIn =
      (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio +
                               (uint8_t)TLM_RATIO_NO_TLM);
  uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
  if (ExpressLRS_currTlmDenom != TlmDenom) {
    DBGLN("New TLMrate 1:%u", TlmDenom);
    ExpressLRS_currTlmDenom = TlmDenom;
  }

  // Model match check
  // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in
  // that case Must use bitwise NOT (~) to match upstream ELRS behavior
  uint8_t modelXor = (~modelMatchId) & MODELMATCH_MASK;
  bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);

  if (connectionState == disconnected || OtaNonce != otaSync->nonce ||
      FHSSgetCurrIndex() != otaSync->fhssIndex ||
      connectionHasModelMatch != modelMatched) {
    FHSSsetCurrIndex(otaSync->fhssIndex);
    OtaNonce = otaSync->nonce;
    connectionHasModelMatch = modelMatched;
    return true;
  }

  return false;
}

//=============================================================================
// Process RC data packet
//=============================================================================
static void ICACHE_RAM_ATTR
ProcessRfPacket_RC(OTA_Packet_s const *const otaPktPtr) {
  // Notify callback if registered
  if (connectionHasModelMatch && channelCallback) {
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

static bool ICACHE_RAM_ATTR
ProcessRFPacket(SX12xxDriverCommon::rx_status const status) {
  if (status != SX12xxDriverCommon::SX12XX_RX_OK) {
    return false;
  }

  uint32_t const now = millis();

  OTA_Packet_s *const otaPktPtr = (OTA_Packet_s *const)Radio.RXdataBuffer;

  // Validate CRC
  if (!OtaValidatePacketCrc(otaPktPtr)) {
    crcFailCount++;
    // Print comprehensive CRC debug info
    uint8_t type = Radio.RXdataBuffer[0] & 0x03;
    uint8_t crcHigh = (Radio.RXdataBuffer[0] >> 2) & 0x3F;
    uint8_t crcLow = Radio.RXdataBuffer[7];
    uint16_t inCRC = ((uint16_t)crcHigh << 8) | crcLow;
    DBGLN("CRC FAIL #%lu: type=%d inCRC=0x%04X (H=0x%02X L=0x%02X)",
          crcFailCount, type, inCRC, crcHigh, crcLow);
    DBGLN("  RAW[8]: %02X %02X %02X %02X %02X %02X %02X %02X",
          Radio.RXdataBuffer[0], Radio.RXdataBuffer[1], Radio.RXdataBuffer[2],
          Radio.RXdataBuffer[3], Radio.RXdataBuffer[4], Radio.RXdataBuffer[5],
          Radio.RXdataBuffer[6], Radio.RXdataBuffer[7]);
    DBGLN("  OtaCrcInit=0x%04X, Nonce=%d, FullRes=%d", OtaCrcInitializer,
          OtaNonce, OtaIsFullRes);
    return false;
  }

  crcPassCount++;

  // Get packet type (low 2 bits of first byte)
  uint8_t const type = Radio.RXdataBuffer[0] & 0x03;

  // Capture this packet
  uint32_t cidx = pkt_capture_idx % PKT_CAPTURE_SIZE;
  memcpy((void *)pkt_capture_buf[cidx], (void *)Radio.RXdataBuffer, 8);
  pkt_capture_len[cidx] = 8;
  pkt_capture_idx++;
  pkt_capture_count++;

  // Update link quality and RSSI
  LQCalc.add();
  getRFlinkInfo();

  // Record valid packet time
  LastValidPacket = now;

  // Handle packet based on type
  switch (type) {
  case PACKET_TYPE_RCDATA:
    ProcessRfPacket_RC(otaPktPtr);
    break;

  case PACKET_TYPE_SYNC: {
    OTA_Sync_s const *const sync =
        OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync;
    ProcessRfPacket_SYNC(now, sync);
    break;
  }

  case PACKET_TYPE_DATA:
    // MSP/Data packets - not implemented for basic RX
    break;
  }

  return true;
}

//=============================================================================
// Radio ISR callbacks
//=============================================================================
static bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status rxStatus) {
  return ProcessRFPacket(rxStatus);
}

static void ICACHE_RAM_ATTR TXdoneISR() {
  // After sending telemetry, switch back to RX
  Radio.RXnb();
}

//=============================================================================
// RF Link rate setting
//=============================================================================
static void SetRFLinkRate(uint8_t index, bool bindMode) {
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
  FHSSusePrimaryFreqBand =
      !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) &&
      !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  FHSSuseDualBand = false; // No dual band support

  uint32_t initFreq = FHSSgetInitialFreq();
  DBGLN("SetRFLinkRate: index=%d, radio_type=%d, primaryBand=%d, initFreq=%u",
        index, ModParams->radio_type, FHSSusePrimaryFreqBand,
        (unsigned int)initFreq);

  // Configure radio
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, initFreq,
               ModParams->PreambleLen, invertIQ, ModParams->PayloadLength,
               ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 ||
                   ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4,
               (uint8_t)UID[5], (uint8_t)UID[4]);

  // Update OTA serializers
  OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);

  // Calculate cycle interval for rate scanning
  cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() *
                   ModParams->FHSShopInterval * interval) /
                  (10U * 1000U);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;
  ExpressLRS_nextAirRateIndex = index;

  DBGLN("Set RF rate index %d, interval %lu us", index, interval);
}

//=============================================================================
// Rate cycling for connection scanning
//=============================================================================
static void cycleRfMode() {
  if (LockRFmode)
    return;

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
 * to ensure osDelay() works properly. This matches the working C
 * implementation.
 */
static void onBindButtonEvent(bool long_press) {
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

bool elrs_init(void) {
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
  DBGLN("UID: %02X:%02X:%02X:%02X:%02X:%02X", UID[0], UID[1], UID[2], UID[3],
        UID[4], UID[5]);

  // Initialize channel data
  ChannelDataReset();

  // NOTE: Do NOT call radioHal.init() here!
  // Radio.Begin() below calls hal.init() which does the full init sequence.
  // Calling init() twice causes sl_si91x_gspi_init() to reinitialize the
  // GSPI, corrupting DMA channel state and causing ARM_DRIVER_ERROR on all
  // subsequent SPI transfers.

  // Initialize OTA CRC
  OtaUpdateCrcInitFromUid();

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

  // CRITICAL: Bind the hardware interrupt callbacks to the HAL
  // Without this, the C++ HAL explicitly ignores the hardware DIO interrupts!
  if (LR1121Hal::instance) {
    LR1121Hal::instance->IsrCallback_1 = LR1121Driver::IsrCallback_1;
    LR1121Hal::instance->IsrCallback_2 = LR1121Driver::IsrCallback_2;
  }

  // Set initial rate (start with binding rate for scan - slowest for best
  // range)
  scanIndex = RATE_BINDING;
  SetRFLinkRate(scanIndex, false);

  // Start receiving
  Radio.RXnb();

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
  elrs_config_t *cfg = elrs_config_get();
  if (cfg) {
    modelMatchId = cfg->model_id;
    DBGLN("Model match ID: %d (%s)", modelMatchId,
          modelMatchId == 0xFF ? "disabled" : "enabled");
  }

  DBGLN("ELRS RX init complete");
  return true;
}

void elrs_loop(void) {
  // Process Deferred Hardware Interrupts early
  while (isr_1_pending_count > 0) {
    isr_1_pending_count--;
    Radio.IsrCallback_1();
  }
  while (isr_2_pending_count > 0) {
    isr_2_pending_count--;
    Radio.IsrCallback_2();
  }

  // Automatically dump packets once we have collected enough
  if (pkt_capture_count == 15 && !do_packet_dump) {
    do_packet_dump = true;
  }

  if (do_packet_dump) {
    do_packet_dump = false;
    dump_capture_buffer();
  }

  static uint32_t lastDiag = 0;
  if (millis() - lastDiag > 2000) {
    lastDiag = millis();
    DBGLN("ISR:%lu RXdone:%lu CRCfail:%lu conn:%d rate:%d freq:%lu",
          isr_1_pending_count + isr_2_pending_count,
          pkt_capture_count, // Using pkt_capture_count as proxy for rxDoneCount
          crcFailCount, connectionState,
          ExpressLRS_currAirRate_Modparams
              ? ExpressLRS_currAirRate_Modparams->index
              : 0,
          Radio.currFreq);
  }

  unsigned long now = millis();

  // Rate cycling when disconnected
  if (connectionState == disconnected) {
    cycleRfMode();
  }

  // Connection timeout check
  if (connectionState != disconnected) {
    uint32_t disconnectTimeout =
        ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs;
    if ((now - LastValidPacket) > disconnectTimeout) {
      setConnectionState(disconnected);
    }
  }

  // Rate change handling
  if (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index) {
    setConnectionState(disconnected);
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
      crsfStats.rf_mode = ExpressLRS_currAirRate_Modparams
                              ? ExpressLRS_currAirRate_Modparams->index
                              : 0;
      crsfStats.uplink_tx_power = linkStats.uplink_TX_Power;
      crsfStats.downlink_rssi = 0; // RX doesn't have downlink stats
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

  //=========================================================================
  // HP GPIO DIO1 Interrupt Mode
  //
  // DIO1 is connected to GPIO_46 (HP domain) on the SiW917. The interrupt
  // is configured in LR1121_hal.cpp init() via lr1121_dio1_init().
  //
  // The interrupt callback chain is:
  //   DIO1 rising edge -> NVIC IRQ -> dio1_gpio_interrupt_callback()
  //   -> dioISR_1() -> IsrCallback_1() -> Radio.IsrCallback() -> packet
  //   processing
  //
  // No polling needed - hardware interrupts handle packet reception.
  //=========================================================================

  // NOTE: Periodic DBGLN printing of ISR stats removed.
  // DBGLN calls printf which uses UART interrupts/DMA. That was taking too
  // long and starving the GSPI DMA completion callback when hwTimer frequency
  // hopped!
}

elrs_connection_state_t elrs_get_connection_state(void) {
  switch (connectionState) {
  case connected:
    return ELRS_CONNECTED;
  case tentative:
    return ELRS_TENTATIVE;
  case disconnected:
    return ELRS_DISCONNECTED;
  case radioFailed:
    return ELRS_RADIO_FAILED;
  default:
    return ELRS_DISCONNECTED;
  }
}

bool elrs_is_connected(void) {
  return (connectionState == connected || connectionState == tentative);
}

uint8_t elrs_get_channels(uint32_t *channels) {
  if (channels) {
    memcpy(channels, ChannelData, sizeof(ChannelData));
  }
  return CRSF_NUM_CHANNELS;
}

void elrs_get_link_stats(elrs_link_stats_t *stats) {
  if (stats) {
    *stats = currentLinkStats;
  }
}

void elrs_set_channel_callback(elrs_channel_callback_t callback) {
  channelCallback = callback;
}

void elrs_enter_binding_mode(void) {
  InBindingMode = true;
  SetRFLinkRate(RATE_BINDING, true);
  DBGLN("Entering binding mode");
}

void elrs_exit_binding_mode(void) {
  InBindingMode = false;
  SetRFLinkRate(scanIndex, false);
  DBGLN("Exiting binding mode");
}

void elrs_enter_wifi_mode(void) {
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

bool elrs_is_wifi_mode(void) { return InWiFiMode; }

bool elrs_set_uid(const uint8_t new_uid[6]) {
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
    DBGLN("New UID saved: %02X:%02X:%02X:%02X:%02X:%02X", UID[0], UID[1],
          UID[2], UID[3], UID[4], UID[5]);

    // Re-initialize FHSS with new UID
    OtaUpdateCrcInitFromUid();
    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    return true;
  } else {
    DBGLN("ERROR: Failed to save UID to NVM3");
    return false;
  }
}

uint8_t elrs_get_lq(void) { return uplinkLQ; }

const char *elrs_get_rate_name(void) {
  if (!ExpressLRS_currAirRate_Modparams) {
    return "Unknown";
  }

  switch (ExpressLRS_currAirRate_Modparams->enum_rate) {
  case RATE_LORA_900_50HZ:
    return "900M 50Hz";
  case RATE_LORA_900_100HZ:
    return "900M 100Hz";
  case RATE_LORA_900_200HZ:
    return "900M 200Hz";
  case RATE_LORA_900_250HZ:
    return "900M 250Hz";
  case RATE_LORA_2G4_50HZ:
    return "2.4G 50Hz";
  case RATE_LORA_2G4_150HZ:
    return "2.4G 150Hz";
  case RATE_LORA_2G4_250HZ:
    return "2.4G 250Hz";
  case RATE_LORA_2G4_500HZ:
    return "2.4G 500Hz";
  default:
    return "Custom";
  }
}

//=============================================================================
// Legacy C API wrappers for gspi_example.c compatibility
//=============================================================================

void elrs_rx_test(void) {
  DBGLN("=== ELRS C++ Integration Test ===");
  DBGLN("Platform: SiW917");
  DBGLN("Radio: LR1121 (Waveshare Core1121-HF)");
  DBGLN("Mode: 900MHz Sub-GHz");
  DBGLN("UID: 0x%02X%02X%02X%02X%02X%02X", UID[0], UID[1], UID[2], UID[3],
        UID[4], UID[5]);
  DBGLN("=== End Integration Test ===");
}

void elrs_rx_init(void) { elrs_init(); }

void elrs_rx_start(void) { DBGLN("ELRS RX Started"); }

void elrs_rx_stop(void) {
  DBGLN("ELRS RX Stopped");
  hwTimer::stop();
}

void elrs_rx_loop(void) { elrs_loop(); }

} // extern "C"
