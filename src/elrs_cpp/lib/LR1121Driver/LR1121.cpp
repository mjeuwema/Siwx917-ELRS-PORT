#include "Arduino.h"
#include "LR1121.h"
#include "LR1121_hal.h"
#include "logging.h"
#include "lr1121_transceiver_F30104.h"

// C functions from lr1121_driver.c
extern "C" {
#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
}

// LittleFS is ESP32-specific, not needed for SiW917
#if defined(PLATFORM_ESP32)
#include <LittleFS.h>
#endif
#include <SPIEx.h>

#define LR1121_FIRMWARE_TYPE 0xF3

LR1121Hal hal;
LR1121Driver *LR1121Driver::instance = NULL;
static volatile uint8_t siw917_last_payload_length = 8;
static volatile uint32_t siw917_rxnbisr_entry_us = 0;
static volatile uint32_t siw917_packet_ready_us = 0;

extern "C" uint32_t lr1121_get_last_rxnbisr_entry_us(void) {
  return siw917_rxnbisr_entry_us;
}

extern "C" uint32_t lr1121_get_last_packet_ready_us(void) {
  return siw917_packet_ready_us;
}

static int UpdateFirmwareWithCDriver(
    const SX12XX_Radio_Number_t radioNumber) {
  if (radioNumber != SX12XX_Radio_1) {
    DBGLN("LR1121 OTA helper only supports Radio_1 on SiW917");
    return -1;
  }

  if (lr1121_begin_update(sizeof(lr11xx_firmware_image)) != 0) {
    return -1;
  }

  WORD_ALIGNED_ATTR uint8_t dest[256];
  for (uint32_t pos = 0; pos < sizeof(lr11xx_firmware_image) / 4; pos += 64) {
    uint32_t size = 256;
    if (pos + 63 > sizeof(lr11xx_firmware_image) / 4) {
      size = sizeof(lr11xx_firmware_image) % 256;
    }

    memcpy(dest, lr11xx_firmware_image + pos, size);
    for (size_t i = 0; i < size; i += 4) {
      const auto ptr = reinterpret_cast<uint32_t *>(&dest[i]);
      *ptr = __builtin_bswap32(*ptr);
    }

    if (lr1121_write_update_bytes(dest, size) != 0) {
      return -1;
    }
  }

  return lr1121_end_update();
}

// DEBUG_LR1121_OTA_TIMING

#if defined(DEBUG_LR1121_OTA_TIMING)
static uint32_t beginTX;
static uint32_t endTX;
#endif

class FECCodec final : public BufferCodec {
public:
  void encode(uint8_t *out, uint8_t *in, uint32_t len) override;
  void decode(uint8_t *out, uint8_t *in, uint32_t len) override;
} fecCodec;

class CopyCodec final : public BufferCodec {
public:
  void encode(uint8_t *out, uint8_t *in, uint32_t len) override;
  void decode(uint8_t *out, uint8_t *in, uint32_t len) override;
} copyCodec;

void ICACHE_RAM_ATTR FECCodec::encode(uint8_t *out, uint8_t *in, uint32_t len) {
  memset(out, 0, len); // ensure that the buffer is zeroed to start
  FECEncode(in, out);
}

void ICACHE_RAM_ATTR FECCodec::decode(uint8_t *out, uint8_t *in, uint32_t len) {
  FECDecode(in, out);
}

void ICACHE_RAM_ATTR CopyCodec::encode(uint8_t *out, uint8_t *in,
                                       const uint32_t len) {
  memcpy(out, in, len);
}
void ICACHE_RAM_ATTR CopyCodec::decode(uint8_t *out, uint8_t *in,
                                       const uint32_t len) {
  memcpy(out, in, len);
}

LR1121Driver::LR1121Driver() : SX12xxDriverCommon() {
  useFSK = false;
  rxContinuousActive = false;
  instance = this;
  strongestReceivingRadio = SX12XX_Radio_1;
  fallBackMode = LR1121_MODE_FS;
  codec = &copyCodec;
}

void LR1121Driver::End() {
  SetMode(LR1121_MODE_SLEEP, SX12XX_Radio_All);
  hal.end();
  RemoveCallbacks();
}

bool LR1121Driver::CheckVersion(const SX12XX_Radio_Number_t radioNumber) {
  firmware_version_t version = GetFirmwareVersion(radioNumber);
  // On ESP32, LittleFS can be used to skip firmware check. On other platforms,
  // always check.
#if defined(PLATFORM_ESP32)
  bool skipUpdate = LittleFS.exists("/lr1121.txt");
#else
  bool skipUpdate = false;
#endif
  version = GetFirmwareVersion(radioNumber);
  if (!skipUpdate && (version.type != LR1121_FIRMWARE_TYPE ||
                      version.version != LR11XX_FIRMWARE_VERSION)) {
    DBGLN("LR1121 #%d version mismatch: expected Type 0x%02X / FW 0x%04X, got "
          "Type 0x%02X / FW 0x%04X",
          radioNumber, LR1121_FIRMWARE_TYPE, LR11XX_FIRMWARE_VERSION,
          version.type, version.version);
    DBGLN("Upgrading radio #%d", radioNumber);
    if (UpdateFirmwareWithCDriver(radioNumber) != 0)
      return false;

    // Match the working Web UI flow: tear down and fully reinitialize the HAL
    // before trusting the post-update version probe.
    hal.end();
    delay(10);
    hal.init();
    delay(50);

    version = GetFirmwareVersion(radioNumber);
    if (version.type != LR1121_FIRMWARE_TYPE ||
        version.version != LR11XX_FIRMWARE_VERSION) {
      DBGLN("LR1121 #%d failed to be detected or upgraded. Type: 0x%02X, "
            "Version: 0x%04X",
            radioNumber, version.type, version.version);
      return false;
    }
  }
  DBGLN("LR1121 #%d Ready", radioNumber);
  return true;
}

bool LR1121Driver::Begin(uint32_t minimumFrequency, uint32_t maximumFrequency) {
  hal.init();
  // NOTE: Do NOT call hal.reset() here!
  // hal.init() already does full initialization including TCXO setup.
  // A hardware reset would undo the TCXO configuration and cause
  // calibration errors (IMG_CALIB_ERR / 0x0200).

  // Validate that the LR1121(s) are working.
  if (!CheckVersion(SX12XX_Radio_1))
    return false;
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    if (!CheckVersion(SX12XX_Radio_2))
      return false;
  }

  hal.IsrCallback_1 = &LR1121Driver::IsrCallback_1;
  hal.IsrCallback_2 = &LR1121Driver::IsrCallback_2;

  // Clear Errors
  hal.WriteCommand(LR11XX_SYSTEM_CLEAR_ERRORS_OC,
                   SX12XX_Radio_All); // Remove later?  Might not be required???

  /*
  Do not enable for dual radio TX.
  When AUTOFS is set and tlm received by only 1 of the 2 radios,  that radio
  will go into FS mode and the other into Standby mode.  After the following SPI
  command for tx mode, busy will go high for differing periods of time because 1
  is transitioning from FS mode and the other from Standby mode. This causes the
  tx done dio of the 2 radios to occur at very different times.
  */
  // 7.2.5 SetRxTxFallbackMode
  uint8_t FBbuf[1] = {LR11XX_RADIO_FALLBACK_FS};
  fallBackMode = LR1121_MODE_FS;
  hal.WriteCommand(LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC, FBbuf,
                   sizeof(FBbuf), SX12XX_Radio_All);

  // 7.2.12 SetRxBoosted
  uint8_t abuf[1] = {1};
  hal.WriteCommand(LR11XX_RADIO_SET_RX_BOOSTED_OC, abuf, sizeof(abuf),
                   SX12XX_Radio_All);

  SetDioAsRfSwitch();
  SetDioIrqParams();

  if (OPT_USE_HARDWARE_DCDC) {
    // 5.1.1 SetRegMode
    uint8_t RegMode[1] = {1};
    hal.WriteCommand(LR11XX_SYSTEM_SET_REGMODE_OC, RegMode, sizeof(RegMode),
                     SX12XX_Radio_All); // Enable DCDC converter instead of LDO
  }

  // CalibrateImage must match the actual operating band. The generic
  // Waveshare bring-up sequence performs a baseline calibration during init,
  // but the active runtime band may differ (for example, forced 2.4 GHz
  // bring-up on Core1121-HF). Re-apply the ELRS band-specific image
  // calibration here using the real min/max frequencies before entering RX.
  if (!lr1121_elrs_calib_image(minimumFrequency, maximumFrequency)) {
    DBGLN("CalibImage failed for runtime band %lu-%lu",
          (unsigned long)minimumFrequency, (unsigned long)maximumFrequency);
    return false;
  }

  return true;
}

// 12.2.1 SetTxCw
void LR1121Driver::startCWTest(uint32_t freq,
                               SX12XX_Radio_Number_t radioNumber) {
  // Set a basic Config that can be used for both 2.4G and SubGHz bands.
  Config(LR11XX_RADIO_LORA_BW_62, LR11XX_RADIO_LORA_SF6,
         LR11XX_RADIO_LORA_CR_4_8, freq, 12, false, 8, false, 0, 0,
         radioNumber);
  hal.WriteCommand(LR11XX_RADIO_SET_TX_CW_OC, radioNumber);
}

void LR1121Driver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t regfreq,
                          uint8_t PreambleLength, bool InvertIQ,
                          uint8_t _PayloadLength, bool setFSKModulation,
                          uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                          SX12XX_Radio_Number_t radioNumber) {
  DBGLN("Config: freq=%u, bw=%d, sf=%d, cr=%d, FSK=%d, Pre=%d, InvIQ=%d, "
        "Payload=%d",
        (unsigned int)regfreq, (int)bw, (int)sf, (int)cr, (int)setFSKModulation,
        (int)PreambleLength, (int)InvertIQ, (int)_PayloadLength);
  PayloadLength = _PayloadLength;

  bool isSubGHz = regfreq < 1000000000;

  if (radioNumber & SX12XX_Radio_1)
    radio1isSubGHz = isSubGHz;

  if (radioNumber & SX12XX_Radio_2)
    radio2isSubGHz = isSubGHz;

  IQinverted = InvertIQ;
  lr11xx_radio_lora_iq_t inverted =
      InvertIQ ? LR11XX_RADIO_LORA_IQ_INVERTED : LR11XX_RADIO_LORA_IQ_STANDARD;
  // IQinverted is always STANDARD for 900
  if (isSubGHz) {
    inverted = LR11XX_RADIO_LORA_IQ_STANDARD;
  }

  // Use STDBY_XOSC for TCXO-based modules (Core1121/Waveshare)
  // Using STDBY_RC on a TCXO module can cause calibration errors
  // because the RC oscillator is less stable than the TCXO
  SetMode(LR1121_MODE_STDBY_XOSC, radioNumber);

  // CRITICAL: When reconfiguring the radio, the TCXO needs time to stabilize.
  // The SiW917 SPI is so fast that if we immediately send `SetPacketType`
  // (0x020E), the radio asserts BUSY and times out. Add a 5ms delay to prevent
  // `WriteCommand BUSY timeout`.
  delay(5);

  useFSK = setFSKModulation;

  // 8.1.1 SetPacketType
  uint8_t buf[1] = {useFSK ? LR11XX_RADIO_PKT_TYPE_GFSK
                           : LR11XX_RADIO_PKT_TYPE_LORA};
  hal.WriteCommand(LR11XX_RADIO_SET_PKT_TYPE_OC, buf, sizeof(buf), radioNumber);

  codec = &copyCodec;
  if (useFSK) {
    DBGLN("Config FSK");
    uint32_t bitrate = (uint32_t)bw * 10000;
    uint8_t bwf = sf;
    uint32_t fdev = (uint32_t)cr * 1000;
    ConfigModParamsFSK(bitrate, bwf, fdev, radioNumber);

    // Increase packet length for FEC used only on 1000Hz 2.5GHz.
    if (!isSubGHz) {
      codec = &fecCodec;
      PayloadLength = 14;
    }

    SetPacketParamsFSK(PreambleLength, PayloadLength, radioNumber);
    SetFSKSyncWord(fskSyncWord1, fskSyncWord2, radioNumber);
  } else {
    DBGLN("Config LoRa");
    ConfigModParamsLoRa(bw, sf, cr, radioNumber);

#if defined(DEBUG_FREQ_CORRECTION) // TODO Check if this available with the
                                   // LR1121?
    lr11xx_RadioLoRaPacketLengthsModes_t packetLengthType =
        LR1121_LORA_PACKET_VARIABLE_LENGTH;
#else
    lr11xx_RadioLoRaPacketLengthsModes_t packetLengthType =
        LR1121_LORA_PACKET_FIXED_LENGTH;
#endif

    SetPacketParamsLoRa(PreambleLength, packetLengthType, PayloadLength,
                        inverted, radioNumber);
  }

  SetFrequencyReg(regfreq, radioNumber, false);

  ClearIrqStatus(radioNumber);

  SetPaConfig(isSubGHz,
              radioNumber); // Must be called after changing rf modes between
                            // subG and 2.4G.  This sets the correct rf amps,
                            // and txen pins to be used.
  if (PayloadLength > 0) {
    siw917_last_payload_length = PayloadLength;
  }
  pwrForceUpdate =
      true; // force an update of the output power because the band may have
            // changed, and we need to configure the power for the band.
  CommitOutputPower();
  DBGLN("Config complete");
}

void LR1121Driver::ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF,
                                      uint32_t Fdev,
                                      SX12XX_Radio_Number_t radioNumber) {
  // 8.5.1 SetModulationParams
  uint8_t buf[10];
  buf[0] = Bitrate >> 24;
  buf[1] = Bitrate >> 16;
  buf[2] = Bitrate >> 8;
  buf[3] = Bitrate >> 0;
  buf[4] = LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF; // Pulse Shape - 0x00: No filter
                                              // applied
  buf[5] = BWF;
  buf[6] = Fdev >> 24;
  buf[7] = Fdev >> 16;
  buf[8] = Fdev >> 8;
  buf[9] = Fdev >> 0;
  hal.WriteCommand(LR11XX_RADIO_SET_MODULATION_PARAM_OC, buf, sizeof(buf),
                   radioNumber);
}

void LR1121Driver::SetPacketParamsFSK(uint8_t PreambleLength,
                                      uint8_t PayloadLength,
                                      SX12XX_Radio_Number_t radioNumber) {
  // 8.5.2 SetPacketParams
  uint8_t buf[9];
  buf[0] = 0; // MSB PbLengthTX defines the length of the LoRa packet preamble.
              // Minimum of 12 with SF5 and SF6, and of 8 for other SF advised;
  buf[1] = PreambleLength; // LSB PbLengthTX defines the length of the LoRa
                           // packet preamble. Minimum of 12 with SF5 and SF6,
                           // and of 8 for other SF advised;
  buf[2] = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS; // Pbl Detect - 0x04:
                                                          // Preamble detector
                                                          // length 8 bits
  buf[3] = 16; // SyncWordLen defines the length of the Syncword in bits. By
               // default, the Syncword is set to 0x9723522556536564
  buf[4] =
      LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE; // Addr Comp - 0x00: Address
                                                   // Filtering Disabled
  buf[5] = LR11XX_RADIO_GFSK_PKT_FIX_LEN; // PacketType - 0x00: Packet length is
                                          // known on both sides
  buf[6] = PayloadLength;                 // PayloadLen
  buf[7] = LR11XX_RADIO_GFSK_CRC_OFF;     // CrcType - 0x01: CRC_OFF (No CRC).
  buf[8] =
      LR11XX_RADIO_GFSK_DC_FREE_WHITENING; // DcFree - 0x01:
                                           // SX127x/SX126x/LR11xx compatible
                                           // whitening enable. 0x03: SX128x
                                           // compatible whitening enable
  hal.WriteCommand(LR11XX_RADIO_SET_PKT_PARAM_OC, buf, sizeof(buf),
                   radioNumber);
}

void LR1121Driver::SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2,
                                  SX12XX_Radio_Number_t radioNumber) {
  // 8.5.3 SetGfskSyncWord
  // SyncWordLen is 16 bits as set in SetPacketParamsFSK().  Fill the rest with
  // preamble bytes.
  uint8_t synbuf[8] = {fskSyncWord1, fskSyncWord2, 0x55, 0x55,
                       0x55,         0x55,         0x55, 0x55};
  hal.WriteCommand(LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC, synbuf, sizeof(synbuf),
                   radioNumber);
}

void LR1121Driver::SetDioAsRfSwitch() {
  // 4.2.1 SetDioAsRfSwitch
#if LR1121_BAND_24GHZ
  // Match the proven SiW917/LR1121 bring-up path: the 2.4 GHz RFIO_HF path
  // does not use the external PE4259 switch, so leave DIO5/DIO6 idle.
  DBGLN("SetDioAsRfSwitch skipped for 2.4GHz RFIO_HF path");
  return;
#else
  uint8_t switchbuf[8];
#if defined(LR1121_RFSW_CTRL) && (LR1121_RFSW_CTRL_COUNT == 8)
  switchbuf[0] = LR1121_RFSW_CTRL[0]; // RfswEnable
  switchbuf[1] = LR1121_RFSW_CTRL[1]; // RfSwStbyCfg
  switchbuf[2] = LR1121_RFSW_CTRL[2]; // RfSwRxCfg
  switchbuf[3] = LR1121_RFSW_CTRL[3]; // RfSwTxCfg
  switchbuf[4] = LR1121_RFSW_CTRL[4]; // RfSwTxHPCfg
  switchbuf[5] = LR1121_RFSW_CTRL[5]; // RfSwTxHfCfg
  switchbuf[6] = LR1121_RFSW_CTRL[6]; // Unused
  switchbuf[7] = LR1121_RFSW_CTRL[7]; // RfSwWifiCfg
#else
  // RF switch configuration for Waveshare Core1121-HF with PE4259
  // Citation: Core1121_XF_Sch.pdf - PE4259 truth table:
  //   V1=0, V2=1 → TX (RF1 to ANT)
  //   V1=1, V2=0 → RX (RF2 to ANT)
  // DIO5 → V1, DIO6 → V2
  // So: RX mode = DIO5=1, DIO6=0 = 0x01
  //     TX mode = DIO5=0, DIO6=1 = 0x02
  switchbuf[0] = 0x03; // RfswEnable: DIO5 + DIO6
  switchbuf[1] = 0x00; // RfSwStbyCfg: both off (isolated)
  switchbuf[2] = 0x01; // RfSwRxCfg: DIO5=1, DIO6=0 → RX
  switchbuf[3] = 0x02; // RfSwTxCfg: DIO5=0, DIO6=1 → TX
  switchbuf[4] = 0x02; // RfSwTxHPCfg: same as TX
  switchbuf[5] = 0x00; // RfSwTxHfCfg: 2.4GHz uses RFIO_HF (no switch)
  switchbuf[6] = 0x00; // Unused/GNSS
  switchbuf[7] = 0x00; // RfSwWifiCfg
#endif
  hal.WriteCommand(LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC, switchbuf,
                   sizeof(switchbuf), SX12XX_Radio_All);
#endif
}

void LR1121Driver::CorrectRegisterForSF6(uint8_t sf,
                                         SX12XX_Radio_Number_t radioNumber) {
  // 8.3.1 SetModulationParams
  // - SF6 can be made compatible with the SX127x family in implicit mode via a
  // register setting.
  // - Set bit 18 of register at address 0xf20414 to 1
  // - Set bit 23 of register at address 0xf20414 to 0.  This information is
  // from Semtech in an email. 3.7.3 WriteRegMemMask32

  if ((lr11xx_radio_lora_sf_t)sf == LR11XX_RADIO_LORA_SF6) {
    uint8_t wrbuf[12];
    // Address
    wrbuf[0] = 0x00; // MSB
    wrbuf[1] = 0xf2;
    wrbuf[2] = 0x04;
    wrbuf[3] = 0x14;
    // Mask
    wrbuf[4] = 0x00;       // MSB
    wrbuf[5] = 0b10000100; // bit18=1 and bit23=0
    wrbuf[6] = 0x00;
    wrbuf[7] = 0x00;
    // Data
    wrbuf[8] = 0x00;       // MSB
    wrbuf[9] = 0b00000100; // bit18=1 and bit23=0
    wrbuf[10] = 0x00;
    wrbuf[11] = 0x00;
    hal.WriteCommand(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, sizeof(wrbuf),
                     radioNumber);
  }
}

/***
 * @brief: Schedule an output power change after the next transmit
 ***/
void LR1121Driver::SetOutputPower(int8_t power, bool isSubGHz) {
  uint8_t pwrNew;

  if (isSubGHz) {
    if (OPT_USE_SX1276_RFO_HF) {
      pwrNew = constrain(power, LR1121_POWER_MIN_LP_PA, LR1121_POWER_MAX_LP_PA);
    } else {
      pwrNew = constrain(power, LR1121_POWER_MIN_HP_PA, LR1121_POWER_MAX_HP_PA);
    }

    if ((pwrPendingLF == PWRPENDING_NONE && pwrCurrentLF != pwrNew) ||
        pwrPendingLF != pwrNew) {
      pwrPendingLF = pwrNew;
    }
  } else {
    pwrNew = constrain(power, LR1121_POWER_MIN_HF_PA, LR1121_POWER_MAX_HF_PA);

    if ((pwrPendingHF == PWRPENDING_NONE && pwrCurrentHF != pwrNew) ||
        pwrPendingHF != pwrNew) {
      pwrPendingHF = pwrNew;
    }
  }
}

void ICACHE_RAM_ATTR LR1121Driver::CommitOutputPower() {
  if (pwrPendingLF != PWRPENDING_NONE) {
    pwrCurrentLF = pwrPendingLF;
    pwrPendingLF = PWRPENDING_NONE;
    pwrForceUpdate = true;
  }

  if (pwrPendingHF != PWRPENDING_NONE) {
    pwrCurrentHF = pwrPendingHF;
    pwrPendingHF = PWRPENDING_NONE;
    pwrForceUpdate = true;
  }

  if (pwrForceUpdate) {
    WriteOutputPower(radio1isSubGHz ? pwrCurrentLF : pwrCurrentHF,
                     radio1isSubGHz, SX12XX_Radio_1);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
      WriteOutputPower(radio2isSubGHz ? pwrCurrentLF : pwrCurrentHF,
                       radio2isSubGHz, SX12XX_Radio_2);
    }
    pwrForceUpdate = false;
  }
}

void ICACHE_RAM_ATTR LR1121Driver::WriteOutputPower(
    uint8_t power, bool isSubGHz, SX12XX_Radio_Number_t radioNumber) {
  uint8_t Txbuf[2] = {power, LR11XX_RADIO_RAMP_48_US};

  // 9.5.2 SetTxParams
  hal.WriteCommand(LR11XX_RADIO_SET_TX_PARAMS_OC, Txbuf, sizeof(Txbuf),
                   radioNumber);
}

void ICACHE_RAM_ATTR
LR1121Driver::SetPaConfig(bool isSubGHz, SX12XX_Radio_Number_t radioNumber) {
  uint8_t Pabuf[4] = {0};

  // 9.5.1 SetPaConfig
  if (isSubGHz) {
    // 900M low power RF Amp
    // Table 9-1: Optimized Settings for LP PA with Same Matching Network
    // -17dBm (0xEF) to +14dBm (0x0E) by steps of 1dB if the low power PA is
    // selected
    if (OPT_USE_SX1276_RFO_HF) {
      Pabuf[0] =
          LR11XX_RADIO_PA_SEL_LP; // PaSel - 0x00: Selects the low power PA
      Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VREG; // RegPaSupply - 0x01: Powers
                                                  // the PA from VBAT. The user
                                                  // must use RegPaSupply = 0x01
                                                  // whenever TxPower > 14
      Pabuf[2] = 0x07;                            // PaDutyCycle
    }
    // 900M high power RF Amp
    // Table 9-2: Optimized Settings for HP PA with Same Matching Network
    // -9dBm (0xF7) to +22dBm (0x16) by steps of 1dB if the high power PA is
    // selected
    else {
      Pabuf[0] =
          LR11XX_RADIO_PA_SEL_HP; // PaSel - 0x01: Selects the high power PA
      Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VBAT; // RegPaSupply - 0x01: Powers
                                                  // the PA from VBAT. The user
                                                  // must use RegPaSupply = 0x01
                                                  // whenever TxPower > 14
      Pabuf[2] = 0x04;                            // PaDutyCycle
      Pabuf[3] = 0x07; // PaHPSel - In order to reach +22dBm output power,
                       // PaHPSel must be set to 7. PaHPSel has no impact on
                       // either the low power PA or the high frequency PA.
    }
  }
  // 2.4G RF Amp
  // Table 9-3: Optimized Settings for HF PA with Same Matching Network
  // -18dBm (0xEE) to +13dBm (0x0D) by steps of 1dB if the high frequency PA is
  // selected
  else {
    Pabuf[0] =
        LR11XX_RADIO_PA_SEL_HF; // PaSel - 0x02: Selects the high frequency PA
    Pabuf[1] = LR11XX_RADIO_PA_REG_SUPPLY_VREG; // RegPaSupply - 0x01: Powers
                                                // the PA from VBAT. The user
                                                // must use RegPaSupply = 0x01
                                                // whenever TxPower > 14
    Pabuf[2] = 0x00;                            // PaDutyCycle
    Pabuf[3] = 0x00; // PaHPSel - In order to reach +22dBm output power, PaHPSel
                     // must be set to 7. PaHPSel has no impact on either the
                     // low power PA or the high frequency PA.
  }

  hal.WriteCommand(LR11XX_RADIO_SET_PA_CFG_OC, Pabuf, sizeof(Pabuf),
                   radioNumber);
}

void LR1121Driver::SetMode(lr11xx_RadioOperatingModes_t OPmode,
                           SX12XX_Radio_Number_t radioNumber) {
  WORD_ALIGNED_ATTR uint8_t buf[5] = {0};

  switch (OPmode) {
  case LR1121_MODE_SLEEP:
    // 2.1.5.1 SetSleep
    rxContinuousActive = false;
    hal.WriteCommand(LR11XX_SYSTEM_SET_SLEEP_OC, buf, 5, radioNumber);
    break;

  case LR1121_MODE_STDBY_RC:
    // 2.1.2.1 SetStandby
    rxContinuousActive = false;
    buf[0] = 0x00;
    hal.WriteCommand(LR11XX_SYSTEM_SET_STANDBY_OC, buf, 1, radioNumber);
    break;

  case LR1121_MODE_STDBY_XOSC:
    // 2.1.2.1 SetStandby
    rxContinuousActive = false;
    buf[0] = 0x01;
    hal.WriteCommand(LR11XX_SYSTEM_SET_STANDBY_OC, buf, 1, radioNumber);
    break;

  case LR1121_MODE_FS:
    // 2.1.9.1 SetFs
    rxContinuousActive = false;
    hal.WriteCommand(LR11XX_SYSTEM_SET_FS_OC, radioNumber);
    break;

  case LR1121_MODE_RX_CONT: {
    // 7.2.2 SetRx - Continuous RX mode (0xFFFFFF = no timeout)
    static bool firstRx = true;
    if (firstRx) {
      DBGLN("SetRx(0xFFFFFF) - Entering continuous RX mode");
      firstRx = false;
    }
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF; // Continuous RX
    hal.WriteCommand(LR11XX_RADIO_SET_RX_OC, buf, 3, radioNumber);
    rxContinuousActive = true;
    break;
  }

  case LR1121_MODE_TX:
    // Table 7-3: SetTx Command
    rxContinuousActive = false;
    hal.WriteCommand(LR11XX_RADIO_SET_TX_OC, buf, 3, radioNumber);
    break;

  case LR1121_MODE_CAD:
    break;

  default:
    break;
  }
}

void LR1121Driver::ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr,
                                       SX12XX_Radio_Number_t radioNumber) {
  // 8.3.1 SetModulationParams
  uint8_t buf[4];
  buf[0] = sf;
  buf[1] = bw;
  buf[2] = cr;
  buf[3] = 0x00; // 0x00: LowDataRateOptimize off
  hal.WriteCommand(LR11XX_RADIO_SET_MODULATION_PARAM_OC, buf, sizeof(buf),
                   radioNumber);

  if (radioNumber & SX12XX_Radio_1 && radio1isSubGHz)
    CorrectRegisterForSF6(sf, SX12XX_Radio_1);

  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    if (radioNumber & SX12XX_Radio_2 && radio2isSubGHz)
      CorrectRegisterForSF6(sf, SX12XX_Radio_2);
  }
}

void LR1121Driver::SetPacketParamsLoRa(
    uint8_t PreambleLength, lr11xx_RadioLoRaPacketLengthsModes_t HeaderType,
    uint8_t PayloadLength, uint8_t InvertIQ,
    SX12XX_Radio_Number_t radioNumber) {
  // 8.3.2 SetPacketParams
  uint8_t buf[6];
  buf[0] = 0; // MSB PbLengthTX defines the length of the LoRa packet preamble.
              // Minimum of 12 with SF5 and SF6, and of 8 for other SF advised;
  buf[1] = PreambleLength; // LSB PbLengthTX defines the length of the LoRa
                           // packet preamble. Minimum of 12 with SF5 and SF6,
                           // and of 8 for other SF advised;
  buf[2] =
      HeaderType; // HeaderType defines if the header is explicit or implicit
  buf[3] = PayloadLength; // PayloadLen defines the size of the payload
  buf[4] = LR11XX_RADIO_LORA_CRC_OFF;
  buf[5] = InvertIQ;
  hal.WriteCommand(LR11XX_RADIO_SET_PKT_PARAM_OC, buf, sizeof(buf),
                   radioNumber);
}

void ICACHE_RAM_ATTR
LR1121Driver::SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber,
                              bool doRx, uint32_t rxTime) {
  const uint32_t timeout = rxTime != 0 ? rxTime : 0xFFFFFFU;
  uint8_t buf[7] = {
      (uint8_t)(freq >> 24),
      (uint8_t)(freq >> 16),
      (uint8_t)(freq >> 8),
      (uint8_t)(freq),
      (uint8_t)(timeout >> 16),
      (uint8_t)(timeout >> 8),
      (uint8_t)(timeout),
  };
  if (doRx) {
    // SetRfFrequency_SetRX
    hal.WriteCommand(LR11XX_RADIO_SET_FREQ_SET_RX, buf, sizeof(buf),
                     radioNumber);
    rxContinuousActive = true;
  } else {
    // 7.2.1 SetRfFrequency
    hal.WriteCommand(LR11XX_RADIO_SET_RF_FREQUENCY_OC, buf, 4, radioNumber);
  }

  currFreq = freq;
}

// 4.1.1 SetDioIrqParams
// SetDioIrqParams command (opcode 0x0113) takes 8 bytes:
//   - Bytes 0-3:  IRQ enable mask (which IRQs to enable) - big-endian 32-bit
//   - Bytes 4-7:  Dio1Mask (which IRQs route to DIO1 pin) - big-endian 32-bit
//
// Physical wiring (Waveshare Core1121-HF):
//   LR1121 DIO1 → SiW917 GPIO_46 (HP domain, rising-edge interrupt)
void LR1121Driver::SetDioIrqParams() {
  uint8_t buf[8] = {0};

  const uint32_t irqMask = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;

  // Bytes 0-3: IRQ enable mask
  buf[0] = (irqMask >> 24) & 0xFF;
  buf[1] = (irqMask >> 16) & 0xFF;
  buf[2] = (irqMask >> 8) & 0xFF;
  buf[3] = irqMask & 0xFF;

  // Bytes 4-7: Dio2Mask - ZERO to ensure no hardware routing conflict
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  buf[7] = 0;

  hal.WriteCommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, buf, sizeof(buf),
                   SX12XX_Radio_All);

  DBGLN("SetDioIrqParams: TX_DONE|RX_DONE routed to DIO1 (via Dio1Mask)");
}

uint32_t ICACHE_RAM_ATTR
LR1121Driver::GetIrqStatus(SX12XX_Radio_Number_t radioNumber) {
  uint8_t status[6] = {0};

  // Upstream ELRS hack: Send ClearIrq (0x0114) with mask 0xFFFFFFFF.
  // The LR1121 has NO separate GetIrqStatus command. Instead, the ClearIrq
  // command returns the current IRQ status in the response bytes while
  // simultaneously clearing all IRQs. This is an atomic get+clear.
  // Citation: upstream ELRS LR1121.cpp GetIrqStatus()
  status[0] = LR11XX_SYSTEM_CLEAR_IRQ_OC >> 8;
  status[1] = LR11XX_SYSTEM_CLEAR_IRQ_OC & 0xFF;
  status[2] = 0xFF;
  status[3] = 0xFF;
  status[4] = 0xFF;
  status[5] = 0xFF;

  hal.ReadCommand(status, sizeof(status), radioNumber);

  // IRQ flags are returned Big-Endian (MSB first at status[2])
  return (uint32_t)status[2] << 24 | (uint32_t)status[3] << 16 |
         (uint32_t)status[4] << 8 | (uint32_t)status[5];
}

void ICACHE_RAM_ATTR
LR1121Driver::ClearIrqStatus(SX12XX_Radio_Number_t radioNumber) {
  // Clear IRQ status command (0x0114) takes 4 bytes of masks
  uint8_t buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  hal.WriteCommand(LR11XX_SYSTEM_CLEAR_IRQ_OC, buf, sizeof(buf), radioNumber);
}

void ICACHE_RAM_ATTR LR1121Driver::TXnbISR() {
#ifdef DEBUG_LR1121_OTA_TIMING
  endTX = micros();
  DBGLN("TOA: %d", endTX - beginTX);
#endif
  CommitOutputPower();
  TXdoneCallback();
}

void ICACHE_RAM_ATTR LR1121Driver::TXnb(
    uint8_t *data, const bool sendGeminiBuffer, uint8_t *dataGemini,
    const SX12XX_Radio_Number_t radioNumber) {
  transmittingRadio = radioNumber;

  // //catch TX timeout
  // if (currOpmode == SX1280_MODE_TX)
  // {
  //     DBGLN("Timeout!");
  //     SetMode(SX1280_MODE_FS, SX12XX_Radio_All);
  //     ClearIrqStatus(SX1280_IRQ_RADIO_ALL, SX12XX_Radio_All);
  //     TXnbISR();
  //     return;
  // }

  if (radioNumber == SX12XX_Radio_NONE) {
    SetMode(fallBackMode, SX12XX_Radio_All);
    return;
  }

  rxContinuousActive = false;

#if defined(DEBUG_RCVR_SIGNAL_STATS)
  if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_1) {
    rxSignalStats[0].telem_count++;
  }
  if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_2) {
    rxSignalStats[1].telem_count++;
  }
#endif

  // Normal diversity mode
  if (GPIO_PIN_NSS_2 != UNDEF_PIN && radioNumber != SX12XX_Radio_All) {
    // Make sure the unused radio is in FS mode and will not receive the tx
    // packet.
    if (radioNumber == SX12XX_Radio_1) {
      SetMode(fallBackMode, SX12XX_Radio_2);
    } else {
      SetMode(fallBackMode, SX12XX_Radio_1);
    }
  }

  WORD_ALIGNED_ATTR uint8_t outBuffer[32] = {0};
  const uint8_t length =
      PayloadLength + 3; // 3 extra zero bytes for the 24-bit timeout
  codec->encode(outBuffer, data, PayloadLength);
  if (sendGeminiBuffer) {
    hal.WriteCommand(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length,
                     SX12XX_Radio_1);
    codec->encode(outBuffer, dataGemini, PayloadLength);
    hal.WriteCommand(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length,
                     SX12XX_Radio_2);
  } else {
    hal.WriteCommand(LR11XX_RADIO_WRITE_BUFFER8_SET_TX, outBuffer, length,
                     radioNumber);
  }
#ifdef DEBUG_LLCC68_OTA_TIMING
  beginTX = micros();
#endif
}

inline void ICACHE_RAM_ATTR LR1121Driver::DecodeRssiSnr(
    SX12XX_Radio_Number_t radioNumber, const uint8_t *buf) {
  // RssiPkt defines the average RSSI over the last packet received. RSSI value
  // in dBm is –RssiPkt/2.
  const int8_t rssi = -(int8_t)(buf[useFSK ? 3 : 5] / 2);

  // SignalRssiPkt is an estimation of RSSI of the LoRa signal (after
  // despreading) on last packet received, in two’s complement format [negated,
  // dBm, fixdt(0,8,1)]. Actual RSSI in dB is -SignalRssiPkt/2. rssi[i =
  // -(int8_t)(status[3] / 2); // SignalRssiPkt

  // If radio # is 0, update LastPacketRSSI, otherwise LastPacketRSSI2
  radioNumber == SX12XX_Radio_1 ? LastPacketRSSI = rssi
                                : LastPacketRSSI2 = rssi;

  // Update whatever SNRs we have
  LastPacketSNRRaw = useFSK ? 0 : (int8_t)buf[4];

#if defined(DEBUG_RCVR_SIGNAL_STATS)
  // stat updates
  int i = radioNumber == SX12XX_Radio_1 ? 0 : 1;
  rxSignalStats[i].irq_count++;
  rxSignalStats[i].rssi_sum += rssi;
  rxSignalStats[i].snr_sum += LastPacketSNRRaw;
  if (LastPacketSNRRaw > rxSignalStats[i].snr_max) {
    rxSignalStats[i].snr_max = LastPacketSNRRaw;
  }
#endif
}

bool ICACHE_RAM_ATTR LR1121Driver::RXnbISR(SX12XX_Radio_Number_t radioNumber) {
  siw917_rxnbisr_entry_us = micros();
  const uint8_t effectivePayloadLength =
      PayloadLength != 0 ? PayloadLength : siw917_last_payload_length;
  bool packetAccepted = false;

  // GetPacket
  hal.WriteCommand(LR11XX_RADIO_GET_PACKET, radioNumber);
  hal.ReadCommand(rx_buf, effectivePayloadLength + 6, radioNumber);
  siw917_packet_ready_us = micros();

  codec->decode(RXdataBuffer, rx_buf + 6, effectivePayloadLength);
  packetAccepted = RXdoneCallback(SX12XX_RX_OK);
  if (!packetAccepted) {
#if defined(DEBUG_RCVR_SIGNAL_STATS)
    rxSignalStats[radioNumber == SX12XX_Radio_1 ? 0 : 1].fail_count++;
#endif
  }

  return packetAccepted;
}

void ICACHE_RAM_ATTR LR1121Driver::RXnb() {
  // HOT PATH - No debug output or extra SPI commands here!
  // This is called from ISR context and must be fast.
  // Citation: LR1121 User Manual Section 7.2.2 SetRx
  // "The command SetRx can be issued only in STDBY_RC or STDBY_XOSC modes"
  SetMode(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_All);
  SetMode(LR1121_MODE_RX_CONT, SX12XX_Radio_All);
}

bool ICACHE_RAM_ATTR
LR1121Driver::GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber) {
  return false;
}

// 7.2.8 GetRssiInst
void ICACHE_RAM_ATTR
LR1121Driver::StartRssiInst(SX12XX_Radio_Number_t radioNumber) {
  hal.WriteCommand(LR11XX_RADIO_GET_RSSI_INST_OC, radioNumber);
}

int8_t ICACHE_RAM_ATTR
LR1121Driver::GetRssiInst(SX12XX_Radio_Number_t radioNumber) {
  uint8_t status[2] = {0};
  hal.ReadCommand(status, sizeof(status), radioNumber);
  return -(int8_t)(status[1] / 2);
}

void ICACHE_RAM_ATTR LR1121Driver::CheckForSecondPacket() {
  hasSecondRadioGotData = false;
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    constexpr SX12XX_Radio_Number_t radio[2] = {SX12XX_Radio_1, SX12XX_Radio_2};
    const uint8_t processingRadioIdx =
        (instance->processingPacketRadio == SX12XX_Radio_1) ? 0 : 1;
    const uint8_t secondRadioIdx = !processingRadioIdx;
    const uint32_t secondIrqStatus =
        instance->GetIrqStatus(radio[secondRadioIdx]);
    if (secondIrqStatus & LR1121_IRQ_RX_DONE) {
      hal.WriteCommand(LR11XX_RADIO_GET_PACKET, radio[secondRadioIdx]);
      hal.ReadCommand(rx2_buf, PayloadLength + 6, radio[secondRadioIdx]);
      codec->decode(RXdataBufferSecond, rx2_buf + 6, PayloadLength);
      hasSecondRadioGotData = true;
    }
  }
}

void ICACHE_RAM_ATTR LR1121Driver::GetLastPacketStats() {
  const SX12XX_Radio_Number_t radioNumber =
      processingPacketRadio == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;

  // by default, set the strongest receiving radio to be the current processing
  // radio (which got a successful packet)
  strongestReceivingRadio = processingPacketRadio;
  DecodeRssiSnr(processingPacketRadio, rx_buf);
#if defined(DEBUG_RCVR_SIGNAL_STATS)
  irq_count_or++;
#endif

  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    // when both radio got the packet, use the better RSSI one
    if (hasSecondRadioGotData) {
      const int8_t firstSNR = LastPacketSNRRaw;
      DecodeRssiSnr(radioNumber, rx2_buf);
      LastPacketSNRRaw =
          fuzzy_snr(LastPacketSNRRaw, firstSNR, FuzzySNRThreshold);
      // Update the strongest receiving radio to be the one with better signal
      // strength
      strongestReceivingRadio =
          LastPacketRSSI > LastPacketRSSI2 ? SX12XX_Radio_1 : SX12XX_Radio_2;
#if defined(DEBUG_RCVR_SIGNAL_STATS)
      irq_count_both++;
    } else {
      rxSignalStats[radioNumber == SX12XX_Radio_1 ? 0 : 1].fail_count++;
#endif
    }
  }
}

void LR1121Driver::IsrCallback_1() { IsrCallback(SX12XX_Radio_1); }

void LR1121Driver::IsrCallback_2() { IsrCallback(SX12XX_Radio_2); }

// Debug counters for ISR tracking
static volatile uint32_t isrCallCount = 0;
volatile uint32_t rxDoneCount = 0;
static volatile uint32_t txDoneCount = 0;
static volatile uint32_t otherIrqCount = 0;
volatile uint32_t lastIrqStatus = 0;

extern "C" {
void lr1121_cs_assert(void);
void lr1121_cs_deassert(void);
bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data,
                         uint16_t length);
}

void LR1121Driver::IsrCallback(SX12XX_Radio_Number_t radioNumber) {
  uint32_t irqStatus = instance->GetIrqStatus(radioNumber);

  // Delegate to the version that takes pre-read status
  IsrCallbackWithStatus(radioNumber, irqStatus);
}

void LR1121Driver::IsrCallbackWithStatus(SX12XX_Radio_Number_t radioNumber,
                                         uint32_t irqStatus) {
  isrCallCount++;
  instance->processingPacketRadio = radioNumber;
  const SX12XX_Radio_Number_t otherRadioNumber =
      radioNumber == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;

  lastIrqStatus = irqStatus;

  // HOT PATH - No debug output here! Printf kills timing.

  if (irqStatus & LR1121_IRQ_TX_DONE) {
    txDoneCount++;
    instance->TXnbISR();
    // Note: GetIrqStatus already cleared the IRQ atomically, no need to clear
    // again But we still clear in case of dual radio setup
    if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
      instance->ClearIrqStatus(otherRadioNumber);
    }
  } else if (irqStatus & LR1121_IRQ_RX_DONE) {
    rxDoneCount++;
    instance->RXnbISR(radioNumber);
    // Note: GetIrqStatus already cleared the IRQ atomically
  } else if (irqStatus & LR1121_IRQ_TIMEOUT) {
    // RX timeout - re-arm receiver
    otherIrqCount++;  // Count as "other" for stats
    instance->RXnb(); // Re-enter RX mode
  } else if (irqStatus != 0) {
    otherIrqCount++;
  }
}

// Function to get ISR debug stats
extern "C" void lr1121_get_isr_stats(uint32_t *isr_count, uint32_t *rx_count,
                                     uint32_t *tx_count, uint32_t *other_count,
                                     uint32_t *last_irq) {
  if (isr_count)
    *isr_count = isrCallCount;
  if (rx_count)
    *rx_count = rxDoneCount;
  if (tx_count)
    *tx_count = txDoneCount;
  if (other_count)
    *other_count = otherIrqCount;
  if (last_irq)
    *last_irq = lastIrqStatus;
}

struct lr1121UpdateState_s {
  size_t expectedFilesize;
  size_t totalSize;
  SX12XX_Radio_Number_t updatingRadio;
  size_t left_over;
  struct {
    uint8_t header[6];
    uint8_t buffer[256];
  } __attribute__((packed)) packet;
};

static lr1121UpdateState_s *lr1121UpdateState;
static constexpr uint32_t LR1121BootloaderWriteHeaderSize = 6;

// The Semtech Bootloader strictly requires exactly 256-byte payload chunks for all
// transfers except the last one.
// We previously limited this to 240/250 due to a 256-byte GSPI DMA limit on SiW917.
// Now that the SiW917 SPI driver DMA buffer has been increased to 512 bytes,
// we can safely use the official 256-byte payload sizes.
static constexpr uint32_t LR1121BootloaderMaxWritePayload = 256;

firmware_version_t
LR1121Driver::GetFirmwareVersion(const SX12XX_Radio_Number_t radioNumber,
                                 const uint16_t command) {
  uint8_t buffer[5] = {};
  hal.WriteCommand(command, radioNumber);
  hal.ReadCommand(buffer, sizeof(buffer), radioNumber);
  hal.WaitOnBusy(radioNumber);

  DBGLN("GetFirmwareVersion raw: [%02X %02X %02X %02X %02X] -> HW=0x%02X Type=0x%02X FW=0x%04X",
        buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
        buffer[1], buffer[2], (uint16_t)(buffer[3] << 8 | buffer[4]));

  return {.hardware = buffer[1],
          .type = buffer[2],
          .version = (uint16_t)(buffer[3] << 8 | buffer[4])};
}

int LR1121Driver::BeginUpdate(const SX12XX_Radio_Number_t radioNumber,
                              const uint32_t expectedSize) {
  lr1121UpdateState = new lr1121UpdateState_s;
  lr1121UpdateState->expectedFilesize = expectedSize;
  lr1121UpdateState->updatingRadio = radioNumber;
  lr1121UpdateState->totalSize = 0;
  lr1121UpdateState->left_over = 0;

  // Reboot to BL mode
  DBGLN("Reboot 1121 to bootloader mode");
  uint8_t mode = 3;
  hal.WriteCommand(LR11XX_SYSTEM_REBOOT_OC, &mode, 1, radioNumber);
  while (!hal.WaitOnBusy(radioNumber)) {
    DBGLN("Waiting...");
    delay(10);
  }

  // Ensure we're in BL mode
  DBGLN("Ensure BL mode");
  const firmware_version_t version =
      GetFirmwareVersion(radioNumber, LR11XX_BL_GET_VERSION_OC);
  if (version.type != 0xDF) {
    DBGLN("%x", version);
    return -1; // Not in bootloader mode
  }

  // Erase flash
  DBGLN("Erasing");
  hal.WriteCommand(LR11XX_BL_ERASE_FLASH_OC, radioNumber);
  while (!hal.WaitOnBusy(radioNumber)) {
    DBGLN("Waiting...");
    delay(100);
  }
  DBGLN("Erased");

  lr1121UpdateState->left_over = 0;
  SPIEx.setHwCs(false);

  pinMode(radioNumber == SX12XX_Radio_1 ? GPIO_PIN_NSS : GPIO_PIN_NSS_2,
          OUTPUT);
  digitalWrite(radioNumber == SX12XX_Radio_1 ? GPIO_PIN_NSS : GPIO_PIN_NSS_2,
               HIGH);
  return 0;
}

static void writeBytes(const uint8_t *data, const uint32_t data_size) {
  lr1121UpdateState->packet.header[0] =
      (uint8_t)(LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 8);
  lr1121UpdateState->packet.header[1] =
      (uint8_t)(LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC);
  lr1121UpdateState->packet.header[2] =
      (uint8_t)(lr1121UpdateState->totalSize >> 24);
  lr1121UpdateState->packet.header[3] =
      (uint8_t)(lr1121UpdateState->totalSize >> 16);
  lr1121UpdateState->packet.header[4] =
      (uint8_t)(lr1121UpdateState->totalSize >> 8);
  lr1121UpdateState->packet.header[5] = (uint8_t)(lr1121UpdateState->totalSize);

  uint32_t write_size = lr1121UpdateState->left_over;
  if (data != nullptr) {
    DBGLN("Left %d, new %d", lr1121UpdateState->left_over, data_size);
    memcpy(lr1121UpdateState->packet.header + 6 + lr1121UpdateState->left_over,
           data, data_size);
    write_size += data_size;
  }
  
  if (write_size == 0) {
    return; // Don't send empty firmware update packets
  }

  if (write_size > LR1121BootloaderMaxWritePayload) {
    DBGLN("LR1121 update chunk too large: %u > %u", (unsigned)write_size,
          (unsigned)LR1121BootloaderMaxWritePayload);
    return;
  }
  DBGLN("Flashing %d at %x", write_size, lr1121UpdateState->totalSize);

  // The SiW917 GSPI helper currently supports up to 256 bytes per transfer,
  // including the 6-byte bootloader header. Keep the payload capped so the
  // bootloader write stays within that proven transport limit.
  digitalWrite(lr1121UpdateState->updatingRadio == SX12XX_Radio_1
                   ? GPIO_PIN_NSS
                   : GPIO_PIN_NSS_2,
                LOW);
  // Use transferBytes with nullptr rx to do a write-only transfer.
  // This is critical: SPIEx.transfer() overwrites the source buffer in-place,
  // which corrupts the firmware payload. transferBytes(tx, nullptr, n) uses
  // a separate dummy rx buffer to preserve the firmware data.
  SPIEx.transferBytes(lr1121UpdateState->packet.header, nullptr,
                      6 + write_size);
  digitalWrite(lr1121UpdateState->updatingRadio == SX12XX_Radio_1
                   ? GPIO_PIN_NSS
                   : GPIO_PIN_NSS_2,
               HIGH);

  while (!hal.WaitOnBusy(lr1121UpdateState->updatingRadio)) {
    delay(1);
  }
  lr1121UpdateState->totalSize += write_size;
  lr1121UpdateState->left_over = 0;
  DBGLN("Flashed");
}

int LR1121Driver::WriteUpdateBytes(const uint8_t *bytes, uint32_t size) {
  while (size >= LR1121BootloaderMaxWritePayload - lr1121UpdateState->left_over) {
    const uint32_t chunk_size =
        size > LR1121BootloaderMaxWritePayload - lr1121UpdateState->left_over
            ? LR1121BootloaderMaxWritePayload -
                  lr1121UpdateState->left_over
                                    : size;
    writeBytes(bytes, chunk_size);
    size -= chunk_size;
    bytes += chunk_size;
  }
  memcpy(lr1121UpdateState->packet.header + 6 + lr1121UpdateState->left_over, bytes,
         size);
  lr1121UpdateState->left_over += size;
  DBGLN("Left-over %d", lr1121UpdateState->left_over);
  return 0;
}

int LR1121Driver::EndUpdate() {
  int retCode = 0;
  writeBytes(nullptr, 0);

  SPIEx.setHwCs(true);
#if defined(PLATFORM_ESP32)
  if (GPIO_PIN_NSS_2 != UNDEF_PIN) {
    spiAttachSS(SPIEx.bus(), 1, GPIO_PIN_NSS_2);
  }
#endif

  if (lr1121UpdateState->totalSize == lr1121UpdateState->expectedFilesize) {
    DBGLN("Reboot LR1121");
    uint8_t buf = 0;
    hal.WriteCommand(LR11XX_BL_REBOOT_OC, &buf, 1,
                     lr1121UpdateState->updatingRadio);
    while (!hal.WaitOnBusy(lr1121UpdateState->updatingRadio)) {
      delay(1);
    }
    
    // The LR1121 requires hundreds of milliseconds to fully reboot from Bootloader to Application mode.
    // Waiting 300ms is necessary to prevent querying the bootloader instead of the application.
    delay(300);

    DBGLN("Check not in BL mode");
    const firmware_version_t version = GetFirmwareVersion(
        lr1121UpdateState->updatingRadio, LR11XX_SYSTEM_GET_VERSION_OC);
    DBGLN("Hardware %x", version.hardware >> 24);
    DBGLN("Type %x", version.type);
    DBGLN("Firmware %x", version.version & 0xFFFF);
    retCode = version.type == 0xDF ? -2 : 0; // still in bootloader mode?
  } else {
    DBGLN("Finished expected %d, total %d", lr1121UpdateState->expectedFilesize,
          lr1121UpdateState->totalSize);
    retCode = -1; // Not enough bytes uploaded
  }
  delete lr1121UpdateState;
  lr1121UpdateState = nullptr;
  return retCode;
}
