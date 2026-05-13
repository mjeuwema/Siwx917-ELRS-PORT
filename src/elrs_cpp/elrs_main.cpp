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
#include "LR1121_hal.h"
#include "LowPassFilter.h"
#include "OTA.h"
#include "PFD.h"
#include "common.h"
#include "crc.h"
#include "crsf_protocol.h"
#include "hwTimer.h"
#include "logging.h"
#include "options.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"
#include "telemetry_protocol.h"
#include "msptypes.h"

// Standard includes
#include <stdio.h>
#include <string.h>

// Status LED, bind button, persistent config, WiFi integration, and CRSF output
extern "C" {
#include "bind_button.h"
#include "crsf_serial.h"
#include "elrs_config.h"
#include "status_led.h"
#include "wifi_http_test.h"
void elrs_cpp_request_wifi_mode(void);
int lr1121_dio1_read(void);
uint32_t lr1121_hal_get_last_dio1_edge_us(void);
uint32_t lr1121_hal_get_last_deferred_us(void);
uint32_t lr1121_get_last_rxnbisr_entry_us(void);
uint32_t lr1121_get_last_packet_ready_us(void);
void lr1121_get_isr_stats(uint32_t *isr_count, uint32_t *rx_count,
                          uint32_t *tx_count, uint32_t *other_count,
                          uint32_t *last_irq);
bool lr1121_get_status(uint8_t *stat1, uint8_t *stat2, uint8_t *irq_status);
void elrs_enter_binding_mode(void);
}

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
// Desired buffer time between Packet ISR and Tock ISR. Keep this aligned with
// upstream ELRS; platform latency belongs in the HAL/timer adapter, not here.
#define PACKET_TO_TOCK_SLACK 200
#define CRSF_RC_OUTPUT_INTERVAL 4 // Output RC channels every 4ms (~250Hz)
#define RFmodeCycleMultiplierSlow 10
#define BindingRateChangeCyclePeriodMs 125U
#define ELRS_DIAG_DISABLE_DOWNLINK_TLM 0
#define ELRS_DIAG_DISABLE_CRSF_SERIAL 1
#define ELRS_DIAG_USE_DIO_PFD_TIMESTAMP 0
#define ELRS_DIAG_CRC_NONCE_WINDOW 0
#define ELRS_DIAG_RX_LUA_UL 1
#define ELRS_DIAG_RX_LUA_UL_PRINT_LIMIT 24
#define ELRS_DIAG_RX_LUA_UL_PRINT_EVERY 128
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
static int8_t SwitchModePending = 0;
static tx_transmission_mode_e TxOtaProtocol = TX_NORMAL_MODE;
static bool warnedUnsupportedTxProtocol = false;

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
static volatile int32_t pfdLastRawOffset = 0;
static volatile int32_t pfdLastOffset = 0;
static volatile int32_t pfdLastOffsetDx = 0;
static volatile int32_t pfdLastPhaseShift = 0;
static volatile uint32_t pfdResultCount = 0;

enum DisconnectReason : uint8_t {
  DISC_NONE = 0,
  DISC_RX_LOCK_TIMEOUT,
  DISC_PACKET_TIMEOUT,
  DISC_RATE_CHANGE,
  DISC_EXTERNAL
};

static volatile DisconnectReason lastDisconnectReason = DISC_NONE;

// Cycle interval for rate scanning
uint32_t cycleInterval = 0;

// TLM state

// Telemetry TX (downlink to TX module)
StubbornSender TelemetrySender;
StubbornReceiver DataUlReceiver;
static uint8_t TelemetryBuffer[ELRS_DATA_UL_BUFFER];
static uint8_t DataUlBuffer[ELRS_DATA_UL_BUFFER];
static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static uint8_t telemetryBurstCount = 0;
static uint8_t telemetryBurstMax = 1;
static bool telemBurstValid = false;
static bool alreadyTLMresp = false;
static volatile uint32_t telemetryTxCount = 0;
static volatile uint32_t telemetrySuppressedCount = 0;
static volatile bool dataUlReady = false;
static volatile bool uidSavePending = false;
static uint8_t pendingUid[UID_LEN] = {0};

// SNR accumulator for telemetry (simplified - just use last value)
static int8_t lastSnrRaw = 0;

//=============================================================================
// RX Lua / CRSF-over-OTA bridge
//=============================================================================
//
// Downstream ESP32 routes uplink DATA frames through CRSFRouter/RXEndpoint and
// feeds RXOTAConnector into StubbornSender. This is the same protocol shape, but
// kept deliberately small for the SiW917 bring-up so the TX Lua can discover the
// receiver and read/write a safe starter set of parameters.

enum RxLuaCommandStep : uint8_t {
  RX_LUA_CMD_IDLE = 0,
  RX_LUA_CMD_CLICK = 1,
  RX_LUA_CMD_EXECUTING = 2,
  RX_LUA_CMD_ASK_CONFIRM = 3,
  RX_LUA_CMD_CONFIRMED = 4,
  RX_LUA_CMD_CANCEL = 5,
  RX_LUA_CMD_QUERY = 6,
};

enum RxLuaParamId : uint8_t {
  RX_LUA_PARAM_ROOT = 0,
  RX_LUA_PARAM_PROTOCOL = 1,
  RX_LUA_PARAM_FAILSAFE = 2,
  RX_LUA_PARAM_FORCE_TLM = 3,
  RX_LUA_PARAM_WIFI = 4,
  RX_LUA_PARAM_BIND = 5,
  RX_LUA_PARAM_MODEL_ID = 6,
  RX_LUA_PARAM_TLM_RATIO = 7,
  RX_LUA_PARAM_VERSION = 8,
  RX_LUA_PARAM_COUNT = RX_LUA_PARAM_VERSION,
};

static char rxLuaModelIdValue[8] = "Off";
static char rxLuaTlmRatioValue[8] = "1:1";
static uint8_t rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
static uint8_t rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
static const char *rxLuaWifiCommandInfo = "";
static const char *rxLuaBindCommandInfo = "";
static bool rxLuaWifiPending = false;
static bool rxLuaBindPending = false;
static uint32_t rxLuaPendingActionAtMs = 0;
static volatile uint32_t rxLuaUlChunkCount = 0;
static volatile uint32_t rxLuaUlCompleteCount = 0;
static volatile uint32_t rxLuaCrsfHandledCount = 0;
static volatile uint32_t rxLuaCrsfRejectCount = 0;
static volatile uint32_t rxLuaCrsfIgnoredCount = 0;
static volatile uint8_t rxLuaLastUlPackageIndex = 0;

#define RX_LUA_QUEUE_DEPTH 4
static uint8_t rxLuaQueue[RX_LUA_QUEUE_DEPTH][CRSF_FRAME_SIZE_MAX];
static uint8_t rxLuaQueueLen[RX_LUA_QUEUE_DEPTH] = {};
static uint8_t rxLuaQueueHead = 0;
static uint8_t rxLuaQueueTail = 0;
static uint8_t rxLuaQueueCount = 0;

#if ELRS_DIAG_RX_LUA_UL
static bool rxLuaShouldPrintDiag(uint32_t count) {
  return count <= ELRS_DIAG_RX_LUA_UL_PRINT_LIMIT ||
         (ELRS_DIAG_RX_LUA_UL_PRINT_EVERY != 0 &&
          (count % ELRS_DIAG_RX_LUA_UL_PRINT_EVERY) == 0);
}

static void rxLuaDiagReject(const char *reason, const uint8_t *frame) {
  const uint32_t count = ++rxLuaCrsfRejectCount;
  if (!rxLuaShouldPrintDiag(count)) {
    return;
  }

  if (frame == nullptr) {
    DBGLN("[RX_LUA] CRSF reject #%lu %s null",
          (unsigned long)count, reason);
    return;
  }

  DBGLN("[RX_LUA] CRSF reject #%lu %s bytes:%02X %02X %02X %02X %02X %02X %02X %02X",
        (unsigned long)count, reason, frame[0], frame[1], frame[2], frame[3],
        frame[4], frame[5], frame[6], frame[7]);
}

static void rxLuaDiagIgnore(const char *reason, const uint8_t *frame) {
  const uint32_t count = ++rxLuaCrsfIgnoredCount;
  if (!rxLuaShouldPrintDiag(count)) {
    return;
  }

  DBGLN("[RX_LUA] CRSF ignore #%lu %s type=0x%02X dest=0x%02X orig=0x%02X",
        (unsigned long)count, reason, frame[CRSF_TELEMETRY_TYPE_INDEX],
        frame[3], frame[4]);
}
#endif

static uint8_t *appendCString(uint8_t *dst, const char *src) {
  if (src == nullptr) {
    src = "";
  }

  do {
    *dst++ = (uint8_t)*src;
  } while (*src++ != '\0');

  return dst;
}

static uint8_t selectionOptionMax(const char *options) {
  uint8_t max = 0;
  if (options == nullptr) {
    return 0;
  }

  while (*options != '\0') {
    if (*options++ == ';') {
      max++;
    }
  }
  return max;
}

static uint32_t versionStringToU32(const char *verStr) {
  uint32_t retVal = 0;
  uint8_t accumulator = 0;
  bool trailingData = false;

  while (verStr != nullptr && *verStr != '\0') {
    const char c = *verStr++;
    if (c == '.') {
      retVal = (retVal << 8) | accumulator;
      accumulator = 0;
      trailingData = false;
    } else if (c >= '0' && c <= '9') {
      accumulator = (uint8_t)((accumulator * 10) + (c - '0'));
      trailingData = true;
    } else {
      break;
    }
  }

  if (trailingData) {
    retVal = (retVal << 8) | accumulator;
  }
  return retVal < 0x010000 ? ((uint32_t)OTA_VERSION_ID << 16) : retVal;
}

static uint8_t crsfCalcFrameCrc(const uint8_t *data, uint8_t len);

static void setCrsfHeaderAndCrc(uint8_t *frame, crsf_frame_type_e frameType,
                                uint8_t frameSize) {
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = frameSize;
  frame[2] = frameType;
  frame[frameSize + CRSF_FRAME_NOT_COUNTED_BYTES - 1] =
      crsfCalcFrameCrc(frame + CRSF_FRAME_NOT_COUNTED_BYTES, frameSize - 1);
}

static void setCrsfExtendedHeaderAndCrc(uint8_t *frame,
                                        crsf_frame_type_e frameType,
                                        uint8_t frameSize,
                                        crsf_addr_e destAddr,
                                        crsf_addr_e origAddr) {
  frame[3] = destAddr;
  frame[4] = origAddr;
  setCrsfHeaderAndCrc(frame, frameType, frameSize);
}

static uint8_t crsfCalcFrameCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ CRSF_CRC_POLY)
                                  : (crc << 1));
    }
  }
  return crc;
}

static bool rxLuaIsKnownCrsfFrameAddress(uint8_t address) {
  return address == CRSF_SYNC_BYTE ||
         address == CRSF_ADDRESS_RADIO_TRANSMITTER ||
         address == CRSF_ADDRESS_CRSF_RECEIVER ||
         address == CRSF_ADDRESS_CRSF_TRANSMITTER ||
         address == CRSF_ADDRESS_ELRS_LUA;
}

static bool validateCrsfFrame(const uint8_t *frame, uint8_t *frameLen) {
  if (frame == nullptr) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("null", frame);
#endif
    return false;
  }

  // CRSF byte 0 is the bus/device address, not always CRSF_SYNC_BYTE. The TX
  // Lua "Other Devices" ping arrives via OTA as 0xEE, matching upstream routing.
  if (!rxLuaIsKnownCrsfFrameAddress(frame[0])) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("addr", frame);
#endif
    return false;
  }

  const uint8_t frameSize = frame[CRSF_TELEMETRY_LENGTH_INDEX];
  if (frameSize < 2 ||
      frameSize > (CRSF_FRAME_SIZE_MAX - CRSF_FRAME_NOT_COUNTED_BYTES)) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("len", frame);
#endif
    return false;
  }

  const uint8_t totalLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  if (totalLen > ELRS_DATA_UL_BUFFER) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("buffer", frame);
#endif
    return false;
  }

  const uint8_t crc =
      crsfCalcFrameCrc(frame + CRSF_FRAME_NOT_COUNTED_BYTES, frameSize - 1);
  if (crc != frame[totalLen - 1]) {
    DBGLN("[RX_LUA] Dropping CRSF frame with bad CRC type=0x%02X calc=0x%02X got=0x%02X",
          frame[CRSF_TELEMETRY_TYPE_INDEX], crc, frame[totalLen - 1]);
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagReject("crc", frame);
#endif
    return false;
  }

  if (frameLen != nullptr) {
    *frameLen = totalLen;
  }
  return true;
}

static void rxLuaQueueFrame(const uint8_t *frame, uint8_t len) {
  if (frame == nullptr || len == 0 || len > CRSF_FRAME_SIZE_MAX) {
    return;
  }

  if (rxLuaQueueCount >= RX_LUA_QUEUE_DEPTH) {
    // Keep latest UI responses moving; stale parameter entries are worse than
    // dropping the oldest one during a burst of Lua polling.
    rxLuaQueueHead = (uint8_t)((rxLuaQueueHead + 1) % RX_LUA_QUEUE_DEPTH);
    rxLuaQueueCount--;
  }

  memcpy(rxLuaQueue[rxLuaQueueTail], frame, len);
  rxLuaQueueLen[rxLuaQueueTail] = len;
  rxLuaQueueTail = (uint8_t)((rxLuaQueueTail + 1) % RX_LUA_QUEUE_DEPTH);
  rxLuaQueueCount++;
#if ELRS_DIAG_RX_LUA_UL
  DBGLN("[RX_LUA] queued response type=0x%02X len=%u q=%u",
        frame[CRSF_TELEMETRY_TYPE_INDEX], len, rxLuaQueueCount);
#endif
}

static void rxLuaPumpTelemetry() {
  if (TelemetrySender.IsActive() || rxLuaQueueCount == 0) {
    return;
  }

  const uint8_t len = rxLuaQueueLen[rxLuaQueueHead];
  memcpy(TelemetryBuffer, rxLuaQueue[rxLuaQueueHead], len);
  rxLuaQueueHead = (uint8_t)((rxLuaQueueHead + 1) % RX_LUA_QUEUE_DEPTH);
  rxLuaQueueCount--;
  TelemetrySender.SetDataToTransmit(TelemetryBuffer, len);
}

static uint8_t getProtocolSelectionFromConfig(const elrs_config_t *cfg) {
  if (cfg == nullptr) {
    return 0;
  }

  switch ((elrs_serial_protocol_t)cfg->serial_protocol) {
  case ELRS_SERIAL_SBUS:
    return 1;
  case ELRS_SERIAL_MAVLINK:
    return 2;
  case ELRS_SERIAL_CRSF:
  default:
    return 0;
  }
}

static void rxLuaRefreshDynamicValues() {
  elrs_config_t *cfg = elrs_config_get();
  const uint8_t modelId = cfg != nullptr ? cfg->model_id : modelMatchId;

  if (modelId == 0xFF) {
    strncpy(rxLuaModelIdValue, "Off", sizeof(rxLuaModelIdValue));
  } else {
    snprintf(rxLuaModelIdValue, sizeof(rxLuaModelIdValue), "%u", modelId);
  }
  rxLuaModelIdValue[sizeof(rxLuaModelIdValue) - 1] = '\0';

  snprintf(rxLuaTlmRatioValue, sizeof(rxLuaTlmRatioValue), "1:%u",
           ExpressLRS_currTlmDenom);
  rxLuaTlmRatioValue[sizeof(rxLuaTlmRatioValue) - 1] = '\0';
}

static bool rxLuaBuildDeviceInfo(crsf_addr_e destAddr, uint8_t *frame,
                                 uint8_t *frameLen) {
  const uint8_t nameLen = (uint8_t)(strlen(device_name) + 1);
  const uint8_t payloadLen = (uint8_t)(nameLen + sizeof(deviceInformationPacket_t));
  const uint8_t frameSize = CRSF_EXT_FRAME_SIZE(payloadLen);

  if ((frameSize + CRSF_FRAME_NOT_COUNTED_BYTES) > CRSF_FRAME_SIZE_MAX) {
    return false;
  }

  memcpy(frame + sizeof(crsf_ext_header_t), device_name, nameLen);
  auto *device = reinterpret_cast<deviceInformationPacket_t *>(
      frame + sizeof(crsf_ext_header_t) + nameLen);
  device->serialNo = htobe32(0x454C5253); // "ELRS"
  device->hardwareVer = 0;
  device->softwareVer = htobe32(versionStringToU32(version));
  device->fieldCnt = RX_LUA_PARAM_COUNT;
  device->parameterVersion = 0;

  setCrsfExtendedHeaderAndCrc(frame, CRSF_FRAMETYPE_DEVICE_INFO, frameSize,
                              destAddr, CRSF_ADDRESS_CRSF_RECEIVER);
  *frameLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  return true;
}

static bool rxLuaBuildParameter(crsf_addr_e destAddr, uint8_t parameterIndex,
                                uint8_t fieldChunk, uint8_t *frame,
                                uint8_t *frameLen) {
  rxLuaRefreshDynamicValues();

  uint8_t chunkBuffer[128] = {};
  uint8_t paramType = 0;
  const char *name = "";
  const char *value = "";
  const char *options = "";
  const char *units = "";
  uint8_t selectionValue = 0;
  uint8_t selectionMin = 0;
  uint8_t selectionMax = 0;
  uint8_t commandStep = RX_LUA_CMD_IDLE;
  const char *commandInfo = "";
  bool isFolder = false;

  elrs_config_t *cfg = elrs_config_get();

  switch (parameterIndex) {
  case RX_LUA_PARAM_ROOT:
    isFolder = true;
    paramType = CRSF_FOLDER;
    name = device_name;
    break;
  case RX_LUA_PARAM_PROTOCOL:
    paramType = CRSF_TEXT_SELECTION;
    name = "Protocol";
    options = "CRSF;SBUS;MAVLink";
    selectionValue = getProtocolSelectionFromConfig(cfg);
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_FAILSAFE:
    paramType = CRSF_TEXT_SELECTION;
    name = "SBUS failsafe";
    options = "No Pulses;Last Pos;Set Pos";
    selectionValue =
        cfg != nullptr && cfg->failsafe_mode <= ELRS_FAILSAFE_SET
            ? cfg->failsafe_mode
            : (uint8_t)ELRS_FAILSAFE_NO_PULSES;
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_FORCE_TLM:
    paramType = CRSF_TEXT_SELECTION;
    name = "Force Tlm";
    options = "Off;On";
    selectionValue = (cfg != nullptr && cfg->force_tlm != 0) ? 1 : 0;
    selectionMax = selectionOptionMax(options);
    break;
  case RX_LUA_PARAM_WIFI:
    paramType = CRSF_COMMAND;
    name = "WiFi Mode";
    commandStep = rxLuaWifiCommandStep;
    commandInfo = rxLuaWifiCommandInfo;
    break;
  case RX_LUA_PARAM_BIND:
    paramType = CRSF_COMMAND;
    name = "Enter Bind Mode";
    commandStep = rxLuaBindCommandStep;
    commandInfo = rxLuaBindCommandInfo;
    break;
  case RX_LUA_PARAM_MODEL_ID:
    paramType = CRSF_INFO;
    name = "Model Id";
    value = rxLuaModelIdValue;
    break;
  case RX_LUA_PARAM_TLM_RATIO:
    paramType = CRSF_INFO;
    name = "Tlm Ratio";
    value = rxLuaTlmRatioValue;
    break;
  case RX_LUA_PARAM_VERSION:
    paramType = CRSF_INFO;
    name = version;
    value = commit;
    break;
  default:
    return false;
  }

  chunkBuffer[2] = 0; // parent
  chunkBuffer[3] = paramType;

  uint8_t *next = appendCString(&chunkBuffer[4], name);
  switch (paramType & CRSF_FIELD_TYPE_MASK) {
  case CRSF_FOLDER:
    if (isFolder) {
      next = appendCString(&chunkBuffer[4], name);
      for (uint8_t i = RX_LUA_PARAM_PROTOCOL; i <= RX_LUA_PARAM_COUNT; i++) {
        *next++ = i;
      }
      *next++ = 0xFF;
    }
    break;
  case CRSF_TEXT_SELECTION:
    next = appendCString(next, options);
    *next++ = selectionValue;
    *next++ = selectionMin;
    *next++ = selectionMax;
    *next++ = 0; // default
    next = appendCString(next, units);
    break;
  case CRSF_COMMAND:
    *next++ = commandStep;
    *next++ = 200; // timeout in 10 ms units, matching upstream
    next = appendCString(next, commandInfo);
    break;
  case CRSF_INFO:
  case CRSF_STRING:
    next = appendCString(next, value);
    break;
  default:
    return false;
  }

  const uint8_t dataSize = (uint8_t)(next - &chunkBuffer[2]);
  const uint8_t chunkMax = CRSF_MAX_PACKET_LEN - 6 - 2;
  const uint8_t chunkCnt = (uint8_t)((dataSize + chunkMax - 1) / chunkMax);
  if (fieldChunk >= chunkCnt) {
    return false;
  }

  const uint8_t chunkOffset = (uint8_t)(fieldChunk * chunkMax);
  const uint8_t chunkSize =
      (uint8_t)((dataSize - chunkOffset) > chunkMax ? chunkMax
                                                    : (dataSize - chunkOffset));
  const uint8_t payloadLen = (uint8_t)(chunkSize + 2);
  const uint8_t frameSize = CRSF_EXT_FRAME_SIZE(payloadLen);

  uint8_t *payload = frame + sizeof(crsf_ext_header_t);
  payload[0] = parameterIndex;
  payload[1] = (uint8_t)(chunkCnt - (fieldChunk + 1));
  memcpy(payload + 2, &chunkBuffer[2 + chunkOffset], chunkSize);

  setCrsfExtendedHeaderAndCrc(frame, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                              frameSize, destAddr,
                              CRSF_ADDRESS_CRSF_RECEIVER);
  *frameLen = frameSize + CRSF_FRAME_NOT_COUNTED_BYTES;
  return true;
}

static void rxLuaQueueDeviceInfo(crsf_addr_e destAddr) {
  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
  uint8_t frameLen = 0;
  if (rxLuaBuildDeviceInfo(destAddr, frame, &frameLen)) {
    rxLuaQueueFrame(frame, frameLen);
  }
}

static void rxLuaQueueParameter(crsf_addr_e destAddr, uint8_t parameterIndex,
                                uint8_t fieldChunk) {
  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {};
  uint8_t frameLen = 0;
  if (rxLuaBuildParameter(destAddr, parameterIndex, fieldChunk, frame,
                          &frameLen)) {
    rxLuaQueueFrame(frame, frameLen);
  }
}

static void rxLuaSaveConfig(void) {
  if (elrs_config_save() != 0) {
    DBGLN("[RX_LUA] Config save failed");
  }
}

static void rxLuaHandleParameterWrite(crsf_addr_e origin, uint8_t parameterIndex,
                                      uint8_t arg) {
  elrs_config_t *cfg = elrs_config_get();

  switch (parameterIndex) {
  case RX_LUA_PARAM_PROTOCOL:
    if (cfg != nullptr) {
      cfg->serial_protocol =
          (arg == 1) ? ELRS_SERIAL_SBUS
                     : ((arg == 2) ? ELRS_SERIAL_MAVLINK : ELRS_SERIAL_CRSF);
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_FAILSAFE:
    if (cfg != nullptr) {
      cfg->failsafe_mode =
          arg <= ELRS_FAILSAFE_SET ? arg : (uint8_t)ELRS_FAILSAFE_NO_PULSES;
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_FORCE_TLM:
    if (cfg != nullptr) {
      cfg->force_tlm = arg ? 1 : 0;
      rxLuaSaveConfig();
      rxLuaQueueParameter(origin, parameterIndex, 0);
    }
    break;
  case RX_LUA_PARAM_WIFI:
    rxLuaWifiCommandStep = RX_LUA_CMD_EXECUTING;
    rxLuaWifiCommandInfo = "Entering...";
    rxLuaWifiPending = (arg == RX_LUA_CMD_CLICK || arg == RX_LUA_CMD_CONFIRMED ||
                        arg == RX_LUA_CMD_QUERY);
    rxLuaPendingActionAtMs = millis();
    rxLuaQueueParameter(origin, parameterIndex, 0);
    break;
  case RX_LUA_PARAM_BIND:
    rxLuaBindCommandStep = RX_LUA_CMD_EXECUTING;
    rxLuaBindCommandInfo = "Entering...";
    rxLuaBindPending = (arg == RX_LUA_CMD_CLICK || arg == RX_LUA_CMD_CONFIRMED ||
                        arg == RX_LUA_CMD_QUERY);
    rxLuaPendingActionAtMs = millis();
    rxLuaQueueParameter(origin, parameterIndex, 0);
    break;
  default:
    break;
  }
}

static bool rxLuaHandleCrsfFrame(const uint8_t *frame) {
  uint8_t frameLen = 0;
  if (!validateCrsfFrame(frame, &frameLen)) {
    return false;
  }

  const crsf_frame_type_e frameType =
      (crsf_frame_type_e)frame[CRSF_TELEMETRY_TYPE_INDEX];
  if (frameType < CRSF_FRAMETYPE_DEVICE_PING) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("legacy", frame);
#endif
    return false;
  }

  const auto *ext = reinterpret_cast<const crsf_ext_header_t *>(frame);
  if (ext->dest_addr != CRSF_ADDRESS_CRSF_RECEIVER &&
      ext->dest_addr != CRSF_ADDRESS_BROADCAST) {
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("dest", frame);
#endif
    return false;
  }

  const crsf_addr_e origin = ext->orig_addr;
  const uint8_t *payload = frame + sizeof(crsf_ext_header_t);

  switch (frameType) {
  case CRSF_FRAMETYPE_DEVICE_PING:
    rxLuaCrsfHandledCount++;
    DBGLN("[RX_LUA] DEVICE_PING from 0x%02X", origin);
    rxLuaQueueDeviceInfo(origin);
    return true;
  case CRSF_FRAMETYPE_PARAMETER_READ:
    rxLuaCrsfHandledCount++;
    if (frameLen >= sizeof(crsf_ext_header_t) + 2 + CRSF_FRAME_CRC_SIZE) {
      const uint8_t parameterIndex = payload[0];
      const uint8_t fieldChunk = payload[1];
      DBGLN("[RX_LUA] PARAM_READ id=%u chunk=%u from=0x%02X",
            parameterIndex, fieldChunk, origin);
      if ((parameterIndex == RX_LUA_PARAM_WIFI) && fieldChunk == 0) {
        rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
        rxLuaWifiCommandInfo = "";
      } else if ((parameterIndex == RX_LUA_PARAM_BIND) && fieldChunk == 0) {
        rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
        rxLuaBindCommandInfo = "";
      }
      rxLuaQueueParameter(origin, parameterIndex, fieldChunk);
    }
    return true;
  case CRSF_FRAMETYPE_PARAMETER_WRITE:
    rxLuaCrsfHandledCount++;
    if (frameLen >= sizeof(crsf_ext_header_t) + 2 + CRSF_FRAME_CRC_SIZE) {
      DBGLN("[RX_LUA] PARAM_WRITE id=%u arg=%u", payload[0], payload[1]);
      rxLuaHandleParameterWrite(origin, payload[0], payload[1]);
    }
    return true;
  default:
#if ELRS_DIAG_RX_LUA_UL
    rxLuaDiagIgnore("type", frame);
#endif
    return false;
  }
}

static void rxLuaProcessPendingActions() {
  if (!rxLuaWifiPending && !rxLuaBindPending) {
    return;
  }

  const uint32_t elapsedMs = millis() - rxLuaPendingActionAtMs;
  if ((rxLuaQueueCount != 0 || TelemetrySender.IsActive()) && elapsedMs < 5000) {
    return;
  }

  if (rxLuaWifiPending) {
    rxLuaWifiPending = false;
    rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
    rxLuaWifiCommandInfo = "";
    DBGLN("[RX_LUA] Requesting WiFi mode");
    elrs_cpp_request_wifi_mode();
  }

  if (rxLuaBindPending) {
    rxLuaBindPending = false;
    rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
    rxLuaBindCommandInfo = "";
    DBGLN("[RX_LUA] Entering bind mode");
    elrs_enter_binding_mode();
  }
}

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
extern volatile uint32_t isr_1_total_count;

static void ICACHE_RAM_ATTR getRFlinkInfo();
static void ICACHE_RAM_ATTR HWtimerCallbackTick();
static void ICACHE_RAM_ATTR HWtimerCallbackTock();
static void ICACHE_RAM_ATTR HandleFHSS();
static void ICACHE_RAM_ATTR updatePhaseLock();
static void ICACHE_RAM_ATTR TentativeConnection(unsigned long now);
static void GotConnection(unsigned long now);
static void LostConnection(bool resumeRx);
static uint8_t minLqForChaos();
static void updateBindingMode(unsigned long now);
static void LinkStatsToOta(OTA_LinkStats_s *ls);
static bool ICACHE_RAM_ATTR HandleSendDataDl();
static void updateTelemetryBurst();
static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode);
static void updateSwitchMode();
static void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t *newUid4);
static void ICACHE_RAM_ATTR ProcessRfPacket_DataUl(
    OTA_Packet_s const *const otaPktPtr);
static void DataUlReceiveComplete();

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
         ((uint32_t)UID[4] << 8) | (UID[5] ^ OTA_VERSION_ID);
}

static bool use2G4Domain(void) { return firmwareOptions.domain >= 8; }

static uint8_t getStartupOrBindingRateIndex(void) {
  if (!InBindingMode) {
    elrs_config_t *cfg = elrs_config_get();
    if (cfg != nullptr && cfg->rate_index < RATE_MAX &&
        isSupportedRFRate(cfg->rate_index)) {
      return cfg->rate_index;
    }
  }

  return enumRatetoIndex(use2G4Domain() ? RATE_LORA_2G4_50HZ : RATE_BINDING);
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

static uint8_t minLqForChaos() {
  const uint32_t numfhss = FHSSgetChannelCount();
  const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

static void LinkStatsToOta(OTA_LinkStats_s *ls) {
  if (ls == nullptr) {
    return;
  }

  ls->uplink_RSSI_1 = linkStats.uplink_RSSI_1;
  ls->uplink_RSSI_2 = linkStats.uplink_RSSI_2;
  ls->antenna = antenna;
  ls->modelMatch = connectionHasModelMatch;
  ls->lq = uplinkLQ;
  ls->trueDiversityAvailable = 0;
  ls->SNR = lastSnrRaw;
}

static void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t *newUid4) {
  UID[0] = 0;
  UID[1] = 0;
  for (unsigned i = 0; i < 4; i++) {
    UID[i + 2] = newUid4[i];
  }

  memcpy(firmwareOptions.uid, UID, UID_LEN);
  memcpy(pendingUid, UID, UID_LEN);
  uidSavePending = true;

  DBGLN("New UID from MSP bind = %u,%u,%u,%u,%u,%u", UID[0], UID[1], UID[2],
        UID[3], UID[4], UID[5]);
}

static void ICACHE_RAM_ATTR updatePhaseLock() {
  if (connectionState != disconnected && PFDloop.hasResult()) {
    int32_t rawOffset = PFDloop.calcResult();
    int32_t offset = LPF_Offset.update(rawOffset);
    int32_t offsetDx = LPF_OffsetDx.update(rawOffset - PfdPrevRawOffset);
    PfdPrevRawOffset = rawOffset;
    pfdLastRawOffset = rawOffset;
    pfdLastOffset = offset;
    pfdLastOffsetDx = offsetDx;
    pfdResultCount++;

    if (RXtimerState == tim_locked && (OtaNonce % 8 == 0)) {
      if (offset > 0) {
        hwTimer::incFreqOffset();
      } else if (offset < 0) {
        hwTimer::decFreqOffset();
      }
    }

    int32_t phaseShift = 0;
    if (connectionState != connected) {
      phaseShift = rawOffset >> 1;
    } else {
      phaseShift = offset >> 2;
    }
    pfdLastPhaseShift = phaseShift;
    hwTimer::phaseShift(phaseShift);

    (void)offsetDx;
  }

  PFDloop.reset();
}

static void ICACHE_RAM_ATTR HandleFHSS() {
  if (ExpressLRS_currAirRate_Modparams == nullptr) {
    return;
  }

  uint8_t modresultFHSS =
      OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) ||
      InBindingMode || (modresultFHSS != 0) ||
      (connectionState == disconnected)) {
    return;
  }

  // Match upstream RX core: platform-specific fused retune/RX handling belongs
  // behind the LR1121 HAL boundary.
  Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, false);
}

static void ICACHE_RAM_ATTR HWtimerCallbackTick() {
  uplinkLQ = LQCalc.getLQ();
  linkStats.uplink_Link_quality = uplinkLQ;
  if (!alreadyTLMresp) {
    LQCalc.inc();
  }
  alreadyTLMresp = false;
}

static void ICACHE_RAM_ATTR HWtimerCallbackTock() {
  PFDloop.intEvent(hwTimer::eventMicros());
  OtaNonce++;
  HandleFHSS();
  if (HandleSendDataDl()) {
    telemetryTxCount++;
  }
  updatePhaseLock();
}

static void ICACHE_RAM_ATTR TentativeConnection(unsigned long now) {
  PFDloop.reset();
  setConnectionState(tentative);
  connectionHasModelMatch = false;
  lastDisconnectReason = DISC_NONE;
  RXtimerState = tim_disconnected;
  PfdPrevRawOffset = 0;
  GotConnectionMillis = 0;
  LPF_Offset.init(0);
  LPF_OffsetDx.init(0);
  alreadyTLMresp = false;
  RFmodeLastCycled = now;
}

static void GotConnection(unsigned long now) {
  if (connectionState == connected) {
    return;
  }

  LockRFmode = firmwareOptions.lock_on_first_connection;
  setConnectionState(connected);
  RXtimerState = tim_tentative;
  GotConnectionMillis = now;
}

static void LostConnection(bool resumeRx) {
  if (lastDisconnectReason == DISC_NONE) {
    lastDisconnectReason = DISC_EXTERNAL;
  }

  DBGLN("LostConnection reason=%u age=%lu pfd raw=%ld off=%ld dx=%ld phase=%ld "
        "fo=%ld nonce=%u fhss=%u",
        (unsigned)lastDisconnectReason,
        (unsigned long)(millis() - LastValidPacket),
        (long)pfdLastRawOffset, (long)pfdLastOffset, (long)pfdLastOffsetDx,
        (long)pfdLastPhaseShift, (long)hwTimer::FreqOffset, OtaNonce,
        FHSSgetCurrIndex());

  setConnectionState(disconnected);
  RXtimerState = tim_disconnected;
  PfdPrevRawOffset = 0;
  GotConnectionMillis = 0;
  uplinkLQ = 0;
  connectionHasModelMatch = false;
  LQCalc.reset();
  LPF_Offset.init(0);
  LPF_OffsetDx.init(0);
  alreadyTLMresp = false;
  SwitchModePending = 0;
  dataUlReady = false;
  DataUlReceiver.ResetState();

  if (hwTimer::isRunning()) {
    hwTimer::stop();
  }
  hwTimer::resetFreqOffset();

  if (!InBindingMode) {
    SetRFLinkRate(ExpressLRS_nextAirRateIndex, false);
    if (resumeRx) {
      Radio.RXnb();
    }
  }
}

//=============================================================================
// Connection state management
//=============================================================================

//=============================================================================
// Process SYNC packet
//=============================================================================
static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode) {
  if (OtaSwitchModeCurrent == newSwitchMode) {
    SwitchModePending = 0;
    return;
  }

  int8_t newSwitchModePending = -(int8_t)newSwitchMode - 1;
  if ((connectionState == disconnected) ||
      (SwitchModePending == newSwitchModePending)) {
    SwitchModePending = (int8_t)newSwitchMode + 1;
  } else {
    SwitchModePending = newSwitchModePending;
  }
}

static bool ICACHE_RAM_ATTR
ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const *const otaSync) {
  // Verify the binding ID
  if (otaSync->UID4 != UID[4])
    return false;

  if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
    return false;

  LastSyncPacket = now;

  TxOtaProtocol = (otaSync->otaProtocol == TX_MAVLINK_MODE)
                      ? TX_MAVLINK_MODE
                      : TX_NORMAL_MODE;
  if ((TxOtaProtocol == TX_MAVLINK_MODE) && !warnedUnsupportedTxProtocol) {
    warnedUnsupportedTxProtocol = true;
    DBGLN("TX requested MAVLink OTA mode; CRSF RC output remains unsupported");
  } else if ((TxOtaProtocol == TX_NORMAL_MODE) && warnedUnsupportedTxProtocol) {
    warnedUnsupportedTxProtocol = false;
    DBGLN("TX returned to normal OTA mode");
  }

  geminiMode = otaSync->geminiMode;

  // Will change the packet air rate in loop() if this changes
  ExpressLRS_nextAirRateIndex =
      enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);
  updateSwitchModePendingFromOta(otaSync->switchEncMode);

  // Update TLM ratio
  expresslrs_tlm_ratio_e TLMrateIn =
      (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio +
                               (uint8_t)TLM_RATIO_NO_TLM);
  uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
  if (ExpressLRS_currTlmDenom != TlmDenom) {
    DBGLN("New TLMrate 1:%u", TlmDenom);
    ExpressLRS_currTlmDenom = TlmDenom;
    telemBurstValid = false;
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
    TentativeConnection(now);
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
  if ((connectionState != connected) || SwitchModePending) {
    return;
  }

  bool telemetryConfirmValue = OtaUnpackChannelData(otaPktPtr, ChannelData);
  TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);

  // Notify callback if registered
  if (connectionHasModelMatch && channelCallback) {
    channelCallback(ChannelData, CRSF_NUM_CHANNELS);
  }
}

static void ICACHE_RAM_ATTR
ProcessRfPacket_DataUl(OTA_Packet_s const *const otaPktPtr) {
  uint8_t packageIndex;
  uint8_t const *payload;
  uint8_t dataLen;

  if (OtaIsFullRes) {
    packageIndex = otaPktPtr->full.data_ul.packageIndex;
    payload = otaPktPtr->full.data_ul.payload;
    dataLen = sizeof(otaPktPtr->full.data_ul.payload);
  } else {
    packageIndex = otaPktPtr->std.data_ul.packageIndex;
    payload = otaPktPtr->std.data_ul.payload;
    dataLen = sizeof(otaPktPtr->std.data_ul.payload);
  }

  if (InBindingMode && packageIndex == 1 && payload[0] == MSP_ELRS_BIND) {
    OnELRSBindMSP((uint8_t *)&payload[1]);
    return;
  }

  if (connectionState != connected) {
    return;
  }

#if ELRS_DIAG_RX_LUA_UL
  const uint32_t ulChunkCount = ++rxLuaUlChunkCount;
  rxLuaLastUlPackageIndex = packageIndex;
  if (rxLuaShouldPrintDiag(ulChunkCount)) {
    DBGLN("[RX_LUA] UL chunk #%lu pkg=%u len=%u bytes:%02X %02X %02X %02X %02X %02X",
          (unsigned long)ulChunkCount, packageIndex, dataLen, payload[0],
          dataLen > 1 ? payload[1] : 0, dataLen > 2 ? payload[2] : 0,
          dataLen > 3 ? payload[3] : 0, dataLen > 4 ? payload[4] : 0,
          dataLen > 5 ? payload[5] : 0);
  }
#endif

  DataUlReceiver.ReceiveData(packageIndex, payload, dataLen);
  if (DataUlReceiver.HasFinishedData()) {
    dataUlReady = true;
#if ELRS_DIAG_RX_LUA_UL
    DBGLN("[RX_LUA] UL complete pending after pkg=%u first:%02X %02X %02X %02X %02X %02X %02X %02X",
          packageIndex, DataUlBuffer[0], DataUlBuffer[1], DataUlBuffer[2],
          DataUlBuffer[3], DataUlBuffer[4], DataUlBuffer[5], DataUlBuffer[6],
          DataUlBuffer[7]);
#endif
  }
}

//=============================================================================
// Main packet processing ISR
//=============================================================================

// Debug counter for CRC failures
static uint32_t crcFailCount = 0;
static uint32_t crcPassCount = 0;
static uint32_t lastCrcDebugTime = 0;
static volatile uint32_t crcFailTypeCount[4] = {};
static volatile uint32_t crcPassTypeCount[4] = {};
static volatile uint8_t lastValidPacketType = 0xFF;
static volatile uint8_t lastValidExpectedNonce = 0;
static volatile uint8_t lastValidExpectedFhss = 0;
static volatile uint8_t lastValidSyncNonce = 0xFF;
static volatile uint8_t lastValidSyncFhss = 0xFF;
static volatile uint32_t lastValidFreq = 0;
static volatile uint32_t lastRxDoneLatencyUs = 0;
static volatile uint32_t lastDioToDeferredUs = 0;
static volatile uint32_t lastDeferredToRxIsrUs = 0;
static volatile uint32_t lastRxIsrToPacketUs = 0;
static volatile uint32_t lastPacketToCallbackUs = 0;
static volatile int32_t lastPfdSlackUs = PACKET_TO_TOCK_SLACK;
static volatile uint8_t lastPfdUsedEdgeTimestamp = 0;
static volatile uint8_t lastRadioStat1 = 0;
static volatile uint8_t lastRadioStat2 = 0;
static volatile uint8_t lastRadioIrqByte = 0;
static volatile uint8_t lastRadioStatusOk = 0;
static volatile uint32_t pfdSkippedNoTimestampCount = 0;
static volatile int8_t lastCrcNonceDelta = 127;
static volatile uint8_t lastCrcNonceType = 0xFF;
static volatile uint8_t lastCrcNonceExpected = 0;
static volatile uint8_t lastCrcNonceMatched = 0xFF;
static volatile uint32_t crcNonceDiagHitCount = 0;
static volatile uint32_t crcNonceDiagMissCount = 0;

static inline int32_t ICACHE_RAM_ATTR absI32(int32_t value) {
  return value < 0 ? -value : value;
}

static inline int32_t ICACHE_RAM_ATTR maxI32(int32_t lhs, int32_t rhs) {
  return lhs > rhs ? lhs : rhs;
}

static void ICACHE_RAM_ATTR
diagnoseCrcFailureNonce(OTA_Packet_s const *const otaPktPtr,
                        uint8_t const type) {
#if ELRS_DIAG_CRC_NONCE_WINDOW > 0
  if (type == PACKET_TYPE_SYNC) {
    return;
  }

  const uint8_t expectedNonce = OtaNonce;
  lastCrcNonceType = type;
  lastCrcNonceExpected = expectedNonce;

  for (int8_t delta = -ELRS_DIAG_CRC_NONCE_WINDOW;
       delta <= ELRS_DIAG_CRC_NONCE_WINDOW; delta++) {
    OTA_Packet_s packetCopy;
    memcpy(&packetCopy, otaPktPtr, sizeof(packetCopy));
    const uint8_t candidateNonce = (uint8_t)(expectedNonce + delta);
    if (OtaValidatePacketCrcForNonce(&packetCopy, candidateNonce)) {
      lastCrcNonceDelta = delta;
      lastCrcNonceMatched = candidateNonce;
      crcNonceDiagHitCount++;
      return;
    }
  }

  lastCrcNonceDelta = 127;
  lastCrcNonceMatched = 0xFF;
  crcNonceDiagMissCount++;
#else
  (void)otaPktPtr;
  (void)type;
#endif
}

static bool ICACHE_RAM_ATTR
ProcessRFPacket(SX12xxDriverCommon::rx_status const status) {
  if (status != SX12xxDriverCommon::SX12XX_RX_OK) {
    return false;
  }

  uint32_t const beginProcessing = micros();
  uint32_t const dio1EdgeUs = lr1121_hal_get_last_dio1_edge_us();
  uint32_t const deferredUs = lr1121_hal_get_last_deferred_us();
  uint32_t const rxIsrEntryUs = lr1121_get_last_rxnbisr_entry_us();
  uint32_t const packetReadyUs = lr1121_get_last_packet_ready_us();
  const bool timerRunningForPfd = hwTimer::isRunning();
  uint32_t packetTimeUs = beginProcessing;
  uint8_t pfdSourceForPacket = 0;
  if (dio1EdgeUs != 0U) {
    const uint32_t irqLatencyUs = beginProcessing - dio1EdgeUs;
    if (irqLatencyUs < 10000U) {
      lastRxDoneLatencyUs = irqLatencyUs;
#if ELRS_DIAG_USE_DIO_PFD_TIMESTAMP
      packetTimeUs = dio1EdgeUs;
      if (timerRunningForPfd) {
        pfdSourceForPacket = 1;
      }
#endif
    }
    const uint32_t dioToDeferred = deferredUs - dio1EdgeUs;
    if (deferredUs != 0U && dioToDeferred < 10000U) {
      lastDioToDeferredUs = dioToDeferred;
    }
  }
  if (deferredUs != 0U && rxIsrEntryUs != 0U) {
    const uint32_t deferredToRxIsr = rxIsrEntryUs - deferredUs;
    if (deferredToRxIsr < 10000U) {
      lastDeferredToRxIsrUs = deferredToRxIsr;
    }
  }
  if (rxIsrEntryUs != 0U && packetReadyUs != 0U) {
    const uint32_t rxIsrToPacket = packetReadyUs - rxIsrEntryUs;
    if (rxIsrToPacket < 10000U) {
      lastRxIsrToPacketUs = rxIsrToPacket;
    }
  }
  if (packetReadyUs != 0U) {
    const uint32_t packetToCallback = beginProcessing - packetReadyUs;
    if (packetToCallback < 10000U) {
      lastPacketToCallbackUs = packetToCallback;
    }
  }
  uint32_t const now = millis();

  OTA_Packet_s *const otaPktPtr = (OTA_Packet_s *const)Radio.RXdataBuffer;
  uint8_t const type = Radio.RXdataBuffer[0] & 0x03;

  // Validate CRC
  if (!OtaValidatePacketCrc(otaPktPtr)) {
    crcFailCount++;
    crcFailTypeCount[type]++;
    diagnoseCrcFailureNonce(otaPktPtr, type);
    return false;
  }

  crcPassCount++;
  crcPassTypeCount[type]++;
  lastValidPacketType = type;
  lastValidExpectedNonce = OtaNonce;
  lastValidExpectedFhss = FHSSgetCurrIndex();
  lastValidFreq = Radio.currFreq;
  lastPfdUsedEdgeTimestamp = pfdSourceForPacket;

  if (ExpressLRS_currAirRate_Modparams != nullptr &&
      ExpressLRS_currAirRate_RFperfParams != nullptr) {
    const int32_t slack = maxI32(
        (int32_t)ExpressLRS_currAirRate_Modparams->interval -
            2 * (int32_t)ExpressLRS_currAirRate_RFperfParams->TOA,
        (int32_t)PACKET_TO_TOCK_SLACK);
    lastPfdSlackUs = slack;
    PFDloop.extEvent(packetTimeUs + slack);
  }

  // Capture this packet
  uint32_t cidx = pkt_capture_idx % PKT_CAPTURE_SIZE;
  memcpy((void *)pkt_capture_buf[cidx], (void *)Radio.RXdataBuffer, 8);
  pkt_capture_len[cidx] = 8;
  pkt_capture_idx++;
  pkt_capture_count++;

  // Record valid packet time
  LastValidPacket = now;

  doStartTimer = false;

  // Handle packet based on type
  switch (type) {
  case PACKET_TYPE_RCDATA:
    ProcessRfPacket_RC(otaPktPtr);
    break;

  case PACKET_TYPE_SYNC: {
    OTA_Sync_s const *const sync =
        OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync;
    lastValidSyncNonce = sync->nonce;
    lastValidSyncFhss = sync->fhssIndex;
    doStartTimer = ProcessRfPacket_SYNC(now, sync) && !InBindingMode;
    break;
  }

  case PACKET_TYPE_DATA:
    ProcessRfPacket_DataUl(otaPktPtr);
    break;
  }

  // Match downstream RX core ordering: packet contents are handled before the
  // current LQ period is marked as consumed.
  Radio.GetLastPacketStats();
  getRFlinkInfo();
  LQCalc.add();
  RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

  return true;
}

//=============================================================================
// Radio ISR callbacks
//=============================================================================
static bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status rxStatus) {
  if (LQCalc.currentIsSet() && connectionState == connected) {
    return false;
  }

  if (ProcessRFPacket(rxStatus)) {
    if (doStartTimer) {
      doStartTimer = false;
      hwTimer::resume();
    }
    return true;
  }

  return false;
}

static void ICACHE_RAM_ATTR TXdoneISR() {
  // After sending telemetry, switch back to RX
  Radio.RXnb();
}

//=============================================================================
// RF Link rate setting
//=============================================================================
static void SetRFLinkRate(uint8_t index, bool bindMode) {
  DBGLN("SetRFLinkRate begin: index=%d bind=%d", index, bindMode ? 1 : 0);

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
  DBGLN("SetRFLinkRate timer updated: interval=%lu",
        (unsigned long)interval);

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
  DBGLN("SetRFLinkRate radio configured");

  // Update OTA serializers
  OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
  DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  TelemetrySender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES
                                                  : ELRS4_DATA_DL_MAX_PACKAGES);

  // Calculate cycle interval for rate scanning
  cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() *
                   ModParams->FHSShopInterval * interval) /
                  (10U * 1000U);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;
  ExpressLRS_nextAirRateIndex = index;
  telemBurstValid = false;

  DBGLN("Set RF rate index %d, interval %lu us", index, interval);
}

static bool ICACHE_RAM_ATTR HandleSendDataDl() {
  if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) ||
      alreadyTLMresp || !teamraceHasModelMatch ||
      ((OtaNonce % ExpressLRS_currTlmDenom) != 0)) {
    return false;
  }

#if ELRS_DIAG_DISABLE_DOWNLINK_TLM
  // Isolation test: reserve the telemetry slot without keying the LR1121 TX.
  // This preserves LQ timing while proving whether TX/TX_DONE/RX return causes
  // the post-connect packet loss.
  alreadyTLMresp = true;
  telemetrySuppressedCount++;
  return false;
#endif

  WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {};
  alreadyTLMresp = true;

  bool tlmQueued = TelemetrySender.IsActive();
  if ((NextTelemetryType == PACKET_TYPE_LINKSTATS) || !tlmQueued) {
    otaPkt.std.type = PACKET_TYPE_LINKSTATS;

    if (OtaIsFullRes) {
      otaPkt.full.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      otaPkt.full.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
          otaPkt.full.data_dl.ul_link_stats.payload,
          sizeof(otaPkt.full.data_dl.ul_link_stats.payload));
      LinkStatsToOta(&otaPkt.full.data_dl.ul_link_stats.stats);
    } else {
      otaPkt.std.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      otaPkt.std.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
          otaPkt.std.data_dl.ul_link_stats.payload,
          sizeof(otaPkt.std.data_dl.ul_link_stats.payload));
      LinkStatsToOta(&otaPkt.std.data_dl.ul_link_stats.stats);
    }

    NextTelemetryType = PACKET_TYPE_DATA;
    telemetryBurstCount = 1;
  } else {
    if (telemetryBurstCount < telemetryBurstMax) {
      telemetryBurstCount++;
    } else {
      NextTelemetryType = PACKET_TYPE_LINKSTATS;
    }

    otaPkt.std.type = PACKET_TYPE_DATA;
    if (OtaIsFullRes) {
      otaPkt.full.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      otaPkt.full.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
          otaPkt.full.data_dl.payload, sizeof(otaPkt.full.data_dl.payload));
    } else {
      otaPkt.std.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm();
      otaPkt.std.data_dl.packageIndex = TelemetrySender.GetCurrentPayload(
          otaPkt.std.data_dl.payload, sizeof(otaPkt.std.data_dl.payload));
    }
  }

  OtaGeneratePacketCrc(&otaPkt);
  Radio.TXnb((uint8_t *)&otaPkt, false, nullptr, SX12XX_Radio_All);
  return true;
}

static void updateTelemetryBurst() {
  if (telemBurstValid || (ExpressLRS_currAirRate_Modparams == nullptr) ||
      (ExpressLRS_currTlmDenom == 0)) {
    return;
  }

  telemBurstValid = true;
  uint16_t hz = 1000000 / ExpressLRS_currAirRate_Modparams->interval;
  telemetryBurstMax = TLMBurstMaxForRateRatio(hz, ExpressLRS_currTlmDenom);
  TelemetrySender.UpdateTelemetryRate(hz, ExpressLRS_currTlmDenom,
                                      telemetryBurstMax);
}

static void updateSwitchMode() {
  if ((SwitchModePending <= 0) || (ExpressLRS_currAirRate_Modparams == nullptr)) {
    return;
  }

  OtaUpdateSerializers((OtaSwitchMode_e)(SwitchModePending - 1),
                       ExpressLRS_currAirRate_Modparams->PayloadLength);
  SwitchModePending = 0;
}

static void DataUlReceiveComplete() {
#if ELRS_DIAG_RX_LUA_UL
  const uint32_t completeCount = ++rxLuaUlCompleteCount;
  if (rxLuaShouldPrintDiag(completeCount)) {
    DBGLN("[RX_LUA] UL complete #%lu first:%02X %02X %02X %02X %02X %02X %02X %02X",
          (unsigned long)completeCount, DataUlBuffer[0], DataUlBuffer[1],
          DataUlBuffer[2], DataUlBuffer[3], DataUlBuffer[4], DataUlBuffer[5],
          DataUlBuffer[6], DataUlBuffer[7]);
  }
#endif

  switch (DataUlBuffer[0]) {
  case MSP_ELRS_SET_RX_WIFI_MODE:
    elrs_cpp_request_wifi_mode();
    break;
  case MSP_ELRS_BIND:
    if (InBindingMode) {
      OnELRSBindMSP(&DataUlBuffer[1]);
    }
    break;
  case MSP_ELRS_MAVLINK_TLM:
    if (TxOtaProtocol == TX_MAVLINK_MODE) {
      uint32_t mavlinkLen = DataUlBuffer[1];
      if ((mavlinkLen > 0) &&
          ((mavlinkLen + CRSF_FRAME_NOT_COUNTED_BYTES) <= sizeof(DataUlBuffer)) &&
          crsf_serial_is_ready()) {
        (void)crsf_serial_send_frame(&DataUlBuffer[CRSF_FRAME_NOT_COUNTED_BYTES],
                                     mavlinkLen);
      }
    }
    break;
  default:
    if (rxLuaHandleCrsfFrame(DataUlBuffer)) {
      break;
    }

    if ((TxOtaProtocol == TX_NORMAL_MODE) && crsf_serial_is_ready()) {
      const crsf_header_t *receivedHeader =
          reinterpret_cast<const crsf_header_t *>(DataUlBuffer);
      uint32_t frameLen =
          receivedHeader->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES;
      if (frameLen >= CRSF_MIN_PACKET_LEN && frameLen <= CRSF_MAX_PACKET_LEN) {
        (void)crsf_serial_send_frame(DataUlBuffer, frameLen);
      } else {
        DBGLN("Ignoring malformed UL CRSF frame len=%lu",
              (unsigned long)frameLen);
      }
    }
    break;
  }

  DataUlReceiver.Unlock();
  dataUlReady = false;
}

//=============================================================================
// Rate cycling for connection scanning
//=============================================================================
static bool rateCyclingStarted = false;

static void cycleRfMode() {
  if (LockRFmode || InBindingMode)
    return;

  uint32_t now = millis();

  if ((now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier)) {
    RFmodeLastCycled = now;
    LastSyncPacket = now;

    const uint8_t currentScanIndex = scanIndex % RATE_MAX;
    DBGLN("cycleRfMode begin: scan=%u interval=%lu multiplier=%u",
          currentScanIndex, (unsigned long)cycleInterval,
          (unsigned)RFmodeCycleMultiplier);
    SetRFLinkRate(currentScanIndex, false);
    LQCalc.reset100();

    do {
      scanIndex = (scanIndex + 1) % RATE_MAX;
    } while (!isSupportedRFRate(scanIndex));

    Radio.RXnb();
    DBGLN("cycleRfMode RX re-armed");
    RFmodeCycleMultiplier = 1;

    DBGLN("Cycling to rate index %d", currentScanIndex);

    // Enable per-iteration diagnostics in elrs_loop to catch hang
    rateCyclingStarted = true;
  }
}

static void updateBindingMode(unsigned long now) {
  static uint32_t bindingRateChangeMs = 0;

  if (!InBindingMode || !ExpressLRS_currAirRate_Modparams) {
    return;
  }

  if (use2G4Domain()) {
    return;
  }

  if ((now - bindingRateChangeMs) > BindingRateChangeCyclePeriodMs) {
    bindingRateChangeMs = now;

    uint8_t bindingIndex = enumRatetoIndex(RATE_BINDING);
    uint8_t dualBandBindingIndex = enumRatetoIndex(RATE_DUALBAND_BINDING);

    if (ExpressLRS_currAirRate_Modparams->enum_rate == RATE_DUALBAND_BINDING) {
      SetRFLinkRate(bindingIndex, true);
    } else {
      SetRFLinkRate(dualBandBindingIndex, true);
    }

    Radio.RXnb();
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
  memset(DataUlBuffer, 0, sizeof(DataUlBuffer));
  memset(TelemetryBuffer, 0, sizeof(TelemetryBuffer));
  rxLuaQueueHead = 0;
  rxLuaQueueTail = 0;
  rxLuaQueueCount = 0;
  rxLuaWifiPending = false;
  rxLuaBindPending = false;
  rxLuaWifiCommandStep = RX_LUA_CMD_IDLE;
  rxLuaBindCommandStep = RX_LUA_CMD_IDLE;
  rxLuaWifiCommandInfo = "";
  rxLuaBindCommandInfo = "";
  rxLuaUlChunkCount = 0;
  rxLuaUlCompleteCount = 0;
  rxLuaCrsfHandledCount = 0;
  rxLuaCrsfRejectCount = 0;
  rxLuaCrsfIgnoredCount = 0;
  rxLuaLastUlPackageIndex = 0;
  DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  DataUlReceiver.SetDataToReceive(DataUlBuffer, sizeof(DataUlBuffer));

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

  hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);

  // Start on the configured rate, matching upstream's scan-start behavior.
  scanIndex = getStartupOrBindingRateIndex();
  SetRFLinkRate(scanIndex, false);
  RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;

  // Start receiving
  Radio.RXnb();
  DBGLN("Radio.RXnb() called - LR1121 should be in continuous RX mode");

  // Record start time for rate cycling
  RFmodeLastCycled = millis();

#if ELRS_DIAG_DISABLE_CRSF_SERIAL
  DBGLN("CRSF serial output disabled for standalone RF timing test");
#else
  // Initialize CRSF serial output to flight controller
  // Uses USART0 at 420000 baud (ELRS standard)
  if (crsf_serial_init(420000) != 0) {
    DBGLN("WARNING: CRSF serial init failed - no FC output");
  } else {
    DBGLN("CRSF serial output initialized");
  }
#endif

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
  // CRITICAL: Process deferred DIO1 interrupts in main-loop context.
  // The ISR only sets a flag (no SPI). We process it here where SPI is safe.
  LR1121Hal::handleDeferredISR();
  hwTimer::service();

  unsigned long now = millis();

  if (do_packet_dump) {
    do_packet_dump = false;
    dump_capture_buffer();
  }

  static uint32_t lastDiag = 0;
  if (millis() - lastDiag > 2000) {
    lastDiag = millis();

    uint32_t isrCount = 0;
    uint32_t rxIrqCount = 0;
    uint32_t txIrqCount = 0;
    uint32_t otherIrqCount = 0;
    uint32_t lastIrq = 0;
    lr1121_get_isr_stats(&isrCount, &rxIrqCount, &txIrqCount, &otherIrqCount,
                         &lastIrq);
    if (isrCount == 0 && connectionState == disconnected) {
      uint8_t stat1 = 0;
      uint8_t stat2 = 0;
      uint8_t irq = 0;
      lastRadioStatusOk = lr1121_get_status(&stat1, &stat2, &irq) ? 1 : 0;
      lastRadioStat1 = stat1;
      lastRadioStat2 = stat2;
      lastRadioIrqByte = irq;
    }
    DBGLN("IRQ isr:%lu rx:%lu tx:%lu other:%lu dio:%d irq:0x%08lX "
          "rxok:%lu crcfail:%lu okT:%lu/%lu/%lu failT:%lu/%lu/%lu "
          "stat:%u/%02X/%02X/%02X",
          isrCount, rxIrqCount, txIrqCount, otherIrqCount, lr1121_dio1_read(),
          lastIrq, pkt_capture_count, crcFailCount,
          (unsigned long)crcPassTypeCount[PACKET_TYPE_RCDATA],
          (unsigned long)crcPassTypeCount[PACKET_TYPE_DATA],
          (unsigned long)crcPassTypeCount[PACKET_TYPE_SYNC],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_RCDATA],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_DATA],
          (unsigned long)crcFailTypeCount[PACKET_TYPE_SYNC],
          lastRadioStatusOk, lastRadioStat1, lastRadioStat2, lastRadioIrqByte);
    hwTimer::service();
    DBGLN("LINK conn:%d rxst:%d rate:%d freq:%lu age:%lu nonce:%u fhss:%u "
          "pfd:%ld/%ld/%ld ph:%ld fo:%ld lq:%u/%u rssi:%d snr:%d disc:%u",
          connectionState, RXtimerState,
          ExpressLRS_currAirRate_Modparams
              ? ExpressLRS_currAirRate_Modparams->index
              : 0,
          Radio.currFreq, (unsigned long)(now - LastValidPacket), OtaNonce,
          FHSSgetCurrIndex(),
          (long)pfdLastRawOffset, (long)pfdLastOffset,
          (long)pfdLastOffsetDx, (long)pfdLastPhaseShift,
          (long)hwTimer::FreqOffset, LQCalc.getLQRaw(), LQCalc.getCount(),
          Radio.LastPacketRSSI, Radio.LastPacketSNRRaw,
          (unsigned)lastDisconnectReason);
    hwTimer::service();
#if ELRS_DIAG_CRC_NONCE_WINDOW > 0
    DBGLN("TIM lat:%lu e2loop:%lu loop2rx:%lu rx2pkt:%lu pkt2cb:%lu "
          "slack:%ld pfdsrc:%u pfdskip:%lu last:%u exp:%u/%u sync:%u/%u "
          "vf:%lu crcN:%d/%u/%u/%lu/%lu tlm:%lu/%lu den:%u "
          "lua:%lu/%lu/%lu/%lu/%lu/%u/%u "
          "tmr:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu",
          (unsigned long)lastRxDoneLatencyUs,
          (unsigned long)lastDioToDeferredUs,
          (unsigned long)lastDeferredToRxIsrUs,
          (unsigned long)lastRxIsrToPacketUs,
          (unsigned long)lastPacketToCallbackUs, (long)lastPfdSlackUs,
          lastPfdUsedEdgeTimestamp,
          (unsigned long)pfdSkippedNoTimestampCount, lastValidPacketType,
          lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
          lastValidSyncFhss, (unsigned long)lastValidFreq,
          (int)lastCrcNonceDelta, lastCrcNonceExpected,
          lastCrcNonceMatched, (unsigned long)crcNonceDiagHitCount,
          (unsigned long)crcNonceDiagMissCount,
          (unsigned long)telemetryTxCount,
          (unsigned long)telemetrySuppressedCount, ExpressLRS_currTlmDenom,
          (unsigned long)rxLuaUlChunkCount,
          (unsigned long)rxLuaUlCompleteCount,
          (unsigned long)rxLuaCrsfHandledCount,
          (unsigned long)rxLuaCrsfRejectCount,
          (unsigned long)rxLuaCrsfIgnoredCount, rxLuaQueueCount,
          rxLuaLastUlPackageIndex,
          (unsigned long)hwTimer::getHardwareHalfTicks(),
          (unsigned long)hwTimer::getHardwareCount(),
          (unsigned long)hwTimer::getHardwareMatch(),
          (unsigned long)hwTimer::getHardwareFreqHz(),
          (unsigned long)hwTimer::getQueuedTickCount(),
          (unsigned long)hwTimer::getQueuedTockCount(),
          (unsigned long)hwTimer::getProcessedTickCount(),
          (unsigned long)hwTimer::getProcessedTockCount(),
          (unsigned long)hwTimer::getQueueOverflowCount(),
          (unsigned long)hwTimer::getImmediateTockDeliveredCount());
#else
    DBGLN("TIM lat:%lu e2loop:%lu loop2rx:%lu rx2pkt:%lu pkt2cb:%lu "
          "slack:%ld pfdsrc:%u pfdskip:%lu last:%u exp:%u/%u sync:%u/%u "
          "vf:%lu tlm:%lu/%lu den:%u "
          "lua:%lu/%lu/%lu/%lu/%lu/%u/%u "
          "tmr:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu",
          (unsigned long)lastRxDoneLatencyUs,
          (unsigned long)lastDioToDeferredUs,
          (unsigned long)lastDeferredToRxIsrUs,
          (unsigned long)lastRxIsrToPacketUs,
          (unsigned long)lastPacketToCallbackUs, (long)lastPfdSlackUs,
          lastPfdUsedEdgeTimestamp,
          (unsigned long)pfdSkippedNoTimestampCount, lastValidPacketType,
          lastValidExpectedNonce, lastValidExpectedFhss, lastValidSyncNonce,
          lastValidSyncFhss, (unsigned long)lastValidFreq,
          (unsigned long)telemetryTxCount,
          (unsigned long)telemetrySuppressedCount, ExpressLRS_currTlmDenom,
          (unsigned long)rxLuaUlChunkCount,
          (unsigned long)rxLuaUlCompleteCount,
          (unsigned long)rxLuaCrsfHandledCount,
          (unsigned long)rxLuaCrsfRejectCount,
          (unsigned long)rxLuaCrsfIgnoredCount, rxLuaQueueCount,
          rxLuaLastUlPackageIndex,
          (unsigned long)hwTimer::getHardwareHalfTicks(),
          (unsigned long)hwTimer::getHardwareCount(),
          (unsigned long)hwTimer::getHardwareMatch(),
          (unsigned long)hwTimer::getHardwareFreqHz(),
          (unsigned long)hwTimer::getQueuedTickCount(),
          (unsigned long)hwTimer::getQueuedTockCount(),
          (unsigned long)hwTimer::getProcessedTickCount(),
          (unsigned long)hwTimer::getProcessedTockCount(),
          (unsigned long)hwTimer::getQueueOverflowCount(),
          (unsigned long)hwTimer::getImmediateTockDeliveredCount());
#endif
  }

  updateTelemetryBurst();

  if (uidSavePending) {
    uidSavePending = false;
    (void)elrs_config_set_uid(pendingUid);
    (void)elrs_config_save();
    OtaUpdateCrcInitFromUid();
    DBGLN("Persisted UID from MSP bind");
  }

  if (dataUlReady) {
    DataUlReceiveComplete();
  }

  rxLuaPumpTelemetry();
  rxLuaProcessPendingActions();

  updateSwitchMode();

  // Rate cycling when disconnected
  if (connectionState == disconnected) {
    cycleRfMode();
  }

  if ((connectionState == tentative) &&
      ((now - LastSyncPacket) >
       ExpressLRS_currAirRate_RFperfParams->RxLockTimeoutMs)) {
    lastDisconnectReason = DISC_RX_LOCK_TIMEOUT;
    LostConnection(true);
    RFmodeLastCycled = now;
    LastSyncPacket = now;
  }

  // Connection timeout check
  if (connectionState == connected) {
    uint32_t disconnectTimeout =
        ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs;
    if ((now - LastValidPacket) > disconnectTimeout) {
      lastDisconnectReason = DISC_PACKET_TIMEOUT;
      LostConnection(true);
    }
  }

  // Rate change handling
  if ((connectionState != disconnected) &&
      (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index)) {
    lastDisconnectReason = DISC_RATE_CHANGE;
    LostConnection(true);
    LastSyncPacket = now;
    RFmodeLastCycled = now;
  }

  if (connectionState == tentative) {
    if ((absI32(LPF_OffsetDx.value()) <= 10) && (LPF_Offset.value() < 100) &&
        (LQCalc.getLQRaw() > minLqForChaos())) {
      GotConnection(now);
    }
  }

  if ((RXtimerState == tim_tentative) &&
      ((now - GotConnectionMillis) > ConsiderConnGoodMillis) &&
      (absI32(LPF_OffsetDx.value()) <= 5)) {
    RXtimerState = tim_locked;
  }

  // Send CRSF RC channels to flight controller when connected
  static uint32_t lastRcOutput = 0;
  if ((connectionState == connected) && connectionHasModelMatch &&
      (TxOtaProtocol == TX_NORMAL_MODE) &&
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
    if (crsf_serial_is_ready() && (connectionState == connected) &&
        (TxOtaProtocol == TX_NORMAL_MODE)) {
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

  updateBindingMode(now);

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
  hwTimer::service();
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
  if (InBindingMode) {
    return;
  }

  OtaCrcInitializer = OTA_VERSION_ID;
  OtaNonce = 0;
  InBindingMode = true;
  scanIndex = getStartupOrBindingRateIndex();
  ExpressLRS_nextAirRateIndex = scanIndex;
  SetRFLinkRate(scanIndex, true);
  Radio.RXnb();
  DBGLN("Entering binding mode");
}

void elrs_exit_binding_mode(void) {
  if (!InBindingMode) {
    return;
  }

  InBindingMode = false;
  OtaUpdateCrcInitFromUid();
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  LockRFmode = false;
  scanIndex = getStartupOrBindingRateIndex();
  ExpressLRS_nextAirRateIndex = scanIndex;
  LostConnection(true);
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
