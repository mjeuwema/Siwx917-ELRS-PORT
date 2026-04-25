/**
 * @file lr1121_rx_test.c
 * @brief Standalone LR1121 RX test - pure C, no ELRS C++ stack
 *
 * This test proves whether the LR1121 can receive radio packets by
 * bypassing the entire ELRS C++ driver and using the proven C driver
 * functions directly. It tests 3 independent layers:
 *
 *   Layer 1: Does the chip enter RX mode? (GetStatus SPI poll)
 *   Layer 2: Does IRQ status show RX_DONE? (SPI IRQ register poll)
 *   Layer 3: Does GPIO_46 go HIGH? (Direct pin read of DIO9)
 *
 * Band selection — controlled by LR1121_BAND_24GHZ in lr1121_elrs_init.h:
 *   0: sub-GHz 915.5 MHz, uses PE4259 external RF switch via DIO5/DIO6
 *   1: 2.4 GHz 2450 MHz, uses LR1121 internal RFIO_HF switching
 *      (external switch bypassed — useful for isolating the IRQ chain
 *       from sub-GHz switch/antenna issues)
 *
 * LoRa Configuration:
 *   - 2.4 GHz (default): ELRS 50 Hz profile
 *       SF8/BW800/CR_LI_4-8, implicit header, 8B payload, CRC off, IQ inverted
 *   - Sub-GHz, or LR1121_RX_TEST_WIDE_OPEN=1: wide-open fallback
 *       SF9/BW500/CR4/5, explicit header, 255B max payload, CRC on, std IQ
 *   - Continuous RX (no timeout)
 *
 * Hardware: SiWG917Y (BRD2708A) + LR1121 on mikroBUS
 * Citations:
 *   - LR1121 User Manual Section 7-9 (Radio Commands)
 *   - LR1121 Datasheet Table 5-2 (SetDioIrqParams)
 */

/* Band selection is defined in lr1121_elrs_init.h as LR1121_BAND_24GHZ.
 * Both this test and lr1121_waveshare_init() (CalibrateImage) use it so
 * frequency, image calibration, and RF switch config stay consistent.
 */

#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
#include "rsi_debug.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * Test A: Wide-open RX debug mode
 *
 * When LR1121_RX_TEST_WIDE_OPEN = 1 the 2.4 GHz branch of STEP 6 / STEP 7
 * ignores the ELRS 50 Hz profile and programs a permissive LoRa RX:
 *   SF9, BW500, CR 4/5, LDRO off
 *   Preamble 12, Explicit header, 255B max payload, CRC on, Standard IQ
 *
 * This rules out TX/RX config mismatch as the cause of "IRQ stuck at 0".
 * Any LoRa broadcast on 2440 MHz matching the above should fire at least
 * PREAMBLE_DETECTED / HEADER_VALID / RX_DONE.
 *
 * Set to 0 to restore the ELRS 2.4 GHz 50 Hz config (SF8/BW800/CR_LI_4-8).
 ******************************************************************************/
#ifndef LR1121_RX_TEST_WIDE_OPEN
#define LR1121_RX_TEST_WIDE_OPEN 0
#endif

/*******************************************************************************
 * LR1121 Command Opcodes (subset needed for RX test)
 ******************************************************************************/
#define CMD_GET_STATUS 0x0100
#define CMD_GET_ERRORS 0x010D
#define CMD_CLEAR_ERRORS 0x010E
#define CMD_SET_DIO_IRQ_PARAMS 0x0113
#define CMD_CLEAR_IRQ 0x0114
#define CMD_SET_STANDBY 0x011C

#define CMD_SET_PKT_TYPE 0x020E
#define CMD_SET_MODULATION_PARAM 0x020F
#define CMD_SET_PKT_PARAM 0x0210
#define CMD_SET_RF_FREQUENCY 0x020B
#define CMD_SET_RX 0x0209
#define CMD_SET_RX_BOOSTED 0x0227
#define CMD_GET_RX_BUFFER_STATUS 0x0203
#define CMD_READ_BUFFER8 0x010A
#define CMD_GET_PACKET 0x0702 /* Fused: fetch last packet + metadata */
#define CMD_SET_DIO_AS_RF_SWITCH 0x0112
#define CMD_GET_PKT_STATUS 0x0204
#define CMD_GET_RSSI_INST 0x0205 /* Instantaneous RSSI while in RX */

/* IRQ bits */
#define IRQ_TX_DONE 0x00000004   /* Bit 2 */
#define IRQ_RX_DONE 0x00000008   /* Bit 3 */
#define IRQ_TIMEOUT 0x00000200   /* Bit 9 */
#define IRQ_CRC_ERROR 0x00000040 /* Bit 6 - Preamble/Header/CRC error */

/*******************************************************************************
 * External C driver functions
 ******************************************************************************/
extern bool lr1121_wait_busy_timeout(uint32_t timeout_ms);
extern bool lr1121_send_command(uint16_t opcode, const uint8_t *params,
                                uint16_t param_len);
extern bool lr1121_read_response(uint8_t *response, uint16_t response_len);
extern void lr1121_cs_assert(void);
extern void lr1121_cs_deassert(void);
extern bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data,
                                uint16_t length);
extern int lr1121_dio1_read(void);
extern uint32_t lr1121_dio1_get_isr_count(void);

/*******************************************************************************
 * Helper: delay
 ******************************************************************************/
static void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    for (volatile uint32_t j = 0; j < 10000; j++) {
    }
  }
}

/*******************************************************************************
 * Helper: Send command and optionally read response
 ******************************************************************************/
static bool cmd(uint16_t opcode, const uint8_t *params, uint16_t param_len) {
  if (!lr1121_wait_busy_timeout(100))
    return false;
  return lr1121_send_command(opcode, params, param_len);
}

static bool cmd_read(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                     uint8_t *resp, uint16_t resp_len) {
  if (!cmd(opcode, params, param_len))
    return false;
  if (!lr1121_wait_busy_timeout(100))
    return false;
  return lr1121_read_response(resp, resp_len);
}

/*******************************************************************************
 * Helper: Get chip mode via GetStatus (proven 2-phase SPI)
 ******************************************************************************/
static int get_chip_mode(void) {
  uint8_t resp[6] = {0};
  if (!cmd_read(CMD_GET_STATUS, NULL, 0, resp, 6))
    return -1;
  /* resp[0] = stat1 (during DMA: may be offset)
   * For the proven C driver, lr1121_read_response handles it.
   * stat2 bits [3:1] = chip_mode */
  return (resp[1] >> 1) & 0x07;
}

/*******************************************************************************
 * Helper: Read IRQ status via SPI (raw single-phase transfer)
 *
 * The LR1121 returns IRQ during the GetStatus response.
 * But for a cleaner approach, we use the proven 2-phase SPI:
 *   Phase 1: Send GetStatus (0x0100)
 *   Phase 2: Read 6 bytes [stat1, stat2, irq3, irq2, irq1, irq0]
 *
 * Note: The DMA offset issue affects lr1121_read_response differently than
 * the raw lr1121_spi_transfer used in IsrCallback. The C driver's
 * lr1121_read_response has its own handling.
 ******************************************************************************/
static uint32_t read_irq_status(void) {
  uint8_t resp[6] = {0};
  if (!cmd_read(CMD_GET_STATUS, NULL, 0, resp, 6))
    return 0xFFFFFFFF;

  /* Debug: dump raw bytes so we can see exactly what's coming back */
  DEBUGOUT("  RAW GetStatus resp: [%02X %02X %02X %02X %02X %02X]\n", resp[0],
           resp[1], resp[2], resp[3], resp[4], resp[5]);

  /* IRQ is in bytes 2-5, big-endian (MSB first) */
  uint32_t irq = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
                 ((uint32_t)resp[4] << 8) | ((uint32_t)resp[5]);
  return irq;
}

/*******************************************************************************
 * Helper: Read instantaneous RSSI while in RX mode
 *
 * GetRssiInst (0x0205) returns [stat1, rssi_raw].
 * Conversion per LR1121 UM: RSSI_dBm = -rssi_raw / 2
 *
 * A floating / terminated antenna with no ambient energy typically reads
 * ~ -110 to -120 dBm. If the TX is nearby and on-channel we'd expect
 * spikes up toward -40 .. -70 dBm at the moment a preamble hits.
 ******************************************************************************/
static int read_rssi_inst(void) {
  uint8_t resp[2] = {0};
  if (!cmd_read(CMD_GET_RSSI_INST, NULL, 0, resp, 2))
    return 0;
  /* resp[0] = stat1, resp[1] = rssi_raw */
  return -(int)(resp[1]) / 2;
}

/*******************************************************************************
 * Helper: Clear all IRQ flags
 ******************************************************************************/
static void clear_irq(void) {
  uint8_t buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  cmd(CMD_CLEAR_IRQ, buf, 4);
}

/*******************************************************************************
 * Helper: Read received packet using GetPacket (0x0702)
 *
 * This is the upstream-ELRS path (LR1121.cpp::RXnbISR). It's a fused
 * "fetch the last received packet + its metadata in one SPI read" command:
 *
 *   TX: <opcode 0x0702>
 *   RX: [stat1][stat2][metadata 4B][payload N bytes]
 *        <-- 6 bytes header ---><------ payload ------>
 *
 * Upstream reads PayloadLength + 6 bytes then takes `rx_buf + 6` as payload.
 * We know PayloadLength = 8 for ELRS OTA4, but we accept it as arg for safety.
 *
 * This replaces the older GetRxBufferStatus + ReadBuffer8 pair, which was
 * returning zero payloads — likely because ReadBuffer8 actually wants two
 * param bytes (offset + length) and we were only sending offset.
 ******************************************************************************/
static bool read_rx_packet(uint8_t *payload, uint8_t *length) {
  /* For ELRS OTA4 packets the payload is fixed at 8 bytes (implicit header).
   * If we ever generalise, sniff payload_len via GetRxBufferStatus first. */
  const uint8_t payload_len = 8;
  uint8_t rx_buf[14] = {0}; /* 6 header + 8 payload */

  if (!cmd_read(CMD_GET_PACKET, NULL, 0, rx_buf, payload_len + 6)) {
    return false;
  }

  DEBUGOUT("  GetPacket hdr: [%02X %02X %02X %02X %02X %02X]\n", rx_buf[0],
           rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]);

  *length = payload_len;
  memcpy(payload, &rx_buf[6], payload_len);
  return true;
}

/*******************************************************************************
 * Main standalone RX test
 ******************************************************************************/
void lr1121_rx_test_run(void) {
  DEBUGOUT("\n");
  DEBUGOUT("╔═══════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║        STANDALONE LR1121 RX TEST (Pure C Driver)        ║\n");
  DEBUGOUT("║  Bypasses ELRS C++ stack - tests radio directly         ║\n");
  DEBUGOUT("╚═══════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");

  /* =========================================================================
   * STEP 1: Verify chip is alive and in STDBY_XOSC
   * (waveshare init should have already been called)
   * =========================================================================
   */
  DEBUGOUT("=== STEP 1: Verify chip status ===\n");
  int mode = get_chip_mode();
  DEBUGOUT("  Chip mode: %d (expect 2=STDBY_XOSC)\n", mode);
  if (mode != 2) {
    DEBUGOUT("  WARNING: Chip not in STDBY_XOSC! Init may have failed.\n");
  }

  /* =========================================================================
   * STEP 2: Configure RF switch (PE4259) — sub-GHz only.
   * On 2.4 GHz the LR1121 uses RFIO_HF with internal TX/RX switching,
   * so no external switch configuration is needed (DIO5/DIO6 stay idle).
   * =========================================================================
   */
#if LR1121_BAND_24GHZ
  DEBUGOUT("\n=== STEP 2: SetDioAsRfSwitch SKIPPED (2.4 GHz, no ext switch) ===\n");
#else
  DEBUGOUT("\n=== STEP 2: SetDioAsRfSwitch (PE4259) ===\n");
  uint8_t rf_switch[8] = {
      0x03, /* enable: DIO5+DIO6 */
      0x00, /* standby: both off */
      0x01, /* rx: DIO5=1, DIO6=0 → RX */
      0x02, /* tx: DIO5=0, DIO6=1 → TX */
      0x02, /* tx_hp: same as tx */
      0x00, /* tx_hf */
      0x00, /* gnss */
      0x00  /* wifi */
  };
  if (!cmd(CMD_SET_DIO_AS_RF_SWITCH, rf_switch, 8)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");
#endif

  /* =========================================================================
   * STEP 3: Clear errors and IRQs
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 3: Clear errors and IRQs ===\n");
  cmd(CMD_CLEAR_ERRORS, NULL, 0);
  clear_irq();
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 4: Set packet type to LoRa (0x02)
   * Citation: LR1121 User Manual Section 8.2.1 SetPacketType
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 4: SetPacketType(LoRa) ===\n");
  uint8_t pkt_type[1] = {0x02}; /* LoRa */
  if (!cmd(CMD_SET_PKT_TYPE, pkt_type, 1)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 5: Set RF frequency. LR1121 auto-selects the RF path by frequency:
   *   sub-GHz (150–960 MHz) → RFI_LF / RFO_LF_*  (external switch required)
   *   2.4 GHz (2.4–2.5 GHz) → RFIO_HF            (internal switch)
   * Citation: LR1121 User Manual Section 7.2.3 SetRfFrequency
   * =========================================================================
   */
#if LR1121_BAND_24GHZ
  /* ELRS 2.4 GHz ISM sync/center channel. TX hops 2400.4–2479.4 MHz over 80
   * channels at 50 Hz; fixed on 2440 MHz we catch the ~1-in-80 hops that
   * land here (~0.625 pkt/s). */
  uint32_t freq = 2440000000u;
#else
  uint32_t freq = 915500000u; /* Middle of FCC915 band */
#endif
  DEBUGOUT("\n=== STEP 5: SetRfFrequency(%lu Hz) ===\n", (unsigned long)freq);
  uint8_t freq_buf[4] = {(uint8_t)(freq >> 24), (uint8_t)(freq >> 16),
                         (uint8_t)(freq >> 8), (uint8_t)(freq)};
  if (!cmd(CMD_SET_RF_FREQUENCY, freq_buf, 4)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK: %lu Hz\n", (unsigned long)freq);

  /* =========================================================================
   * STEP 6: Set LoRa modulation parameters
   * Citation: LR1121 User Manual Section 8.3.1 SetModulationParams
   *
   * ELRS 2.4 GHz 50 Hz (from common.cpp row 17):
   *   SF8, BW800, CR 4/8 LongInterleaver  -- hard-coded to match the TX.
   * Sub-GHz path keeps the old wide settings.
   * =========================================================================
   */
#if LR1121_BAND_24GHZ && !LR1121_RX_TEST_WIDE_OPEN
  DEBUGOUT("\n=== STEP 6: SetModulationParams(SF8, BW800, CR_LI_4/8) "
           "ELRS 2G4 50Hz ===\n");
  uint8_t mod_params[4] = {
      0x08, /* SF8 */
      0x0F, /* BW800 (812 kHz, 2G4 only) */
      0x07, /* CR 4/8 Long Interleaver */
      0x00  /* LowDataRateOptimize off */
  };
#else
  /* Wide-open params: work for both sub-GHz and 2.4 GHz Test-A debug */
  DEBUGOUT("\n=== STEP 6: SetModulationParams(SF9, BW500, CR4/5) "
           "[wide-open] ===\n");
  uint8_t mod_params[4] = {
      0x09, /* SF9 */
      0x06, /* BW500 */
      0x01, /* CR 4/5 */
      0x00  /* LowDataRateOptimize off */
  };
#endif
  if (!cmd(CMD_SET_MODULATION_PARAM, mod_params, 4)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 7: Set LoRa packet parameters
   * Citation: LR1121 User Manual Section 8.3.2 SetPacketParams
   *
   * ELRS 2.4 GHz 50 Hz packet: implicit header, 8-byte payload
   * (OTA4_PACKET_SIZE), CRC off (ELRS does its own), IQ inverted.
   * Sub-GHz path keeps explicit/255B/CRC-on to catch any generic LoRa.
   * =========================================================================
   */
#if LR1121_BAND_24GHZ && !LR1121_RX_TEST_WIDE_OPEN
  DEBUGOUT("\n=== STEP 7: SetPacketParams(Preamble=12, Implicit, 8B, "
           "CRC off, IQ inverted) ELRS 2G4 ===\n");
  uint8_t pkt_params[6] = {
      0x00, 0x0C, /* Preamble length = 12 symbols (MSB, LSB) */
      0x01,       /* Implicit header (fixed length) */
      0x08,       /* OTA4_PACKET_SIZE = 8 bytes */
      0x00,       /* CRC off */
      0x01        /* IQ inverted */
  };
#else
  /* Wide-open packet params: catches any standard LoRa broadcast */
  DEBUGOUT("\n=== STEP 7: SetPacketParams(Preamble=12, Explicit, 255B, "
           "CRC on, Standard IQ) [wide-open] ===\n");
  uint8_t pkt_params[6] = {
      0x00, 0x0C, /* Preamble length = 12 symbols (MSB, LSB) */
      0x00,       /* Explicit header (variable length) */
      0xFF,       /* Max payload length = 255 */
      0x01,       /* CRC on */
      0x00        /* Standard IQ (not inverted) */
  };
#endif
  if (!cmd(CMD_SET_PKT_PARAM, pkt_params, 6)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 8: Enable RX boosted mode for better sensitivity
   * Citation: LR1121 User Manual Section 7.2.12 SetRxBoosted
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 8: SetRxBoosted(on) ===\n");
  uint8_t rx_boosted[1] = {0x01};
  if (!cmd(CMD_SET_RX_BOOSTED, rx_boosted, 1)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 9: Set DIO IRQ params - route all IRQs to DIO1 pin (= GPIO_46)
   * Citation: elrs_cpp LR1121.cpp:680-708 and LR1121 User Manual 4.1.1
   *   Command 0x0113 takes exactly 8 bytes:
   *     Bytes 0-3: IRQ enable mask  (which IRQs latch into the register)
   *     Bytes 4-7: Dio1Mask         (which of those route to DIO1 pin)
   *   Physical wiring: LR1121 DIO1 → SiW917 GPIO_46.
   *   The variable `lr1121_dio1_read()` reads this same pin -- the old
   *   "DIO9 via Dio2Mask" labelling was incorrect.
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 9: SetDioIrqParams (ALL -> DIO1 pin / GPIO_46) ===\n");
  uint8_t irq_params[8] = {
      0xFF, 0xFF, 0xFF, 0xFF, /* IrqMask: enable every IRQ */
      0xFF, 0xFF, 0xFF, 0xFF  /* Dio1Mask: route every enabled IRQ to DIO1 */
  };
  if (!cmd(CMD_SET_DIO_IRQ_PARAMS, irq_params, 8)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK: all IRQs enabled and routed to DIO1 pin via Dio1Mask\n");

  /* =========================================================================
   * STEP 10: Clear IRQs again before entering RX
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 10: Clear IRQs before RX ===\n");
  clear_irq();
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 11: Read baseline GPIO_46 and ISR count
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 11: Baseline readings ===\n");
  int dio9_pin = lr1121_dio1_read();
  uint32_t isr_count = lr1121_dio1_get_isr_count();
  DEBUGOUT("  GPIO_46 (DIO9) pin state: %d\n", dio9_pin);
  DEBUGOUT("  ISR count: %lu\n", (unsigned long)isr_count);

  /* =========================================================================
   * STEP 12: Enter continuous RX mode
   * Citation: LR1121 User Manual Section 7.2.2 SetRx
   *   Timeout = 0xFFFFFF → continuous RX
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 12: SetRx(continuous) ===\n");
  uint8_t rx_params[3] = {0xFF, 0xFF, 0xFF}; /* Continuous RX */
  if (!cmd(CMD_SET_RX, rx_params, 3)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  delay_ms(5); /* Let it settle */

  /* Verify we're in RX mode */
  mode = get_chip_mode();
  DEBUGOUT("  Chip mode after SetRx: %d (expect 4=RX)\n", mode);
  if (mode != 4) {
    DEBUGOUT("  ERROR: Did not enter RX mode! Mode=%d\n", mode);
    DEBUGOUT("  Possible causes:\n");
    DEBUGOUT("    - TCXO not stable\n");
    DEBUGOUT("    - Calibration error\n");
    DEBUGOUT("    - RF switch misconfigured\n");
    return;
  }
  DEBUGOUT("  SUCCESS: Radio is in RX mode!\n");

  /* =========================================================================
   * STEP 13: Poll for packets (30 seconds)
   *
   * We poll THREE things every 500ms:
   *   1. GPIO_46 pin state (physical DIO9 wire)
   *   2. IRQ status register (SPI read)
   *   3. ISR callback count (interrupt handler)
   *
   * This tells us EXACTLY where the break is:
   *   - IRQ set but GPIO low → SetDioIrqParams broken
   *   - GPIO high but ISR=0 → GPIO interrupt config broken
   *   - Nothing set → no packets received / radio config wrong
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 13: Polling for packets (30 seconds) ===\n");
#if LR1121_BAND_24GHZ && !LR1121_RX_TEST_WIDE_OPEN
  DEBUGOUT("  ELRS 2.4 GHz 50 Hz on 2440 MHz (SF8/BW800/CR_LI_4-8, implicit\n");
  DEBUGOUT("  8B, CRC off). IQ *alternates* every 5 s because ELRS picks IQ\n");
  DEBUGOUT("  from UID[5]&1 — one of the two polarities will match your TX.\n");
#elif LR1121_BAND_24GHZ
  DEBUGOUT("  Wide-open 2.4 GHz (SF9/BW500/CR4/5/Explicit/CRC/IQ std).\n");
#else
  DEBUGOUT("  Turn on your ELRS TX now! Any 915 MHz LoRa signal will do.\n");
#endif
  DEBUGOUT("  Polling every 100ms (300 polls = 30s)...\n\n");

  uint32_t rx_done_count = 0;
  uint32_t poll_count = 0;

#if LR1121_BAND_24GHZ && !LR1121_RX_TEST_WIDE_OPEN
  /* Current IQ setting for the alternator. Start with INVERTED because
   * that's what half the UIDs use + binding mode always uses it. */
  uint8_t cur_iq = 0x01;
  DEBUGOUT("  >>> IQ window 1: INVERTED\n");
#endif

  for (int i = 0; i < 300; i++) { /* 300 x 100ms = 30 seconds */
    delay_ms(100);
    poll_count++;

#if LR1121_BAND_24GHZ && !LR1121_RX_TEST_WIDE_OPEN
    /* Every 50 polls (~5 s) flip IQ and re-arm RX. This makes one 30 s
     * run cover both possible ELRS IQ polarities without reflashing. */
    if ((poll_count % 50) == 0 && poll_count < 300) {
      cur_iq ^= 0x01;
      DEBUGOUT("  >>> Switching IQ to %s, re-arming RX\n",
               cur_iq ? "INVERTED" : "STANDARD");
      /* SetPacketParams requires STDBY. Go STDBY_XOSC, reprogram, re-RX. */
      uint8_t stdby_xosc[1] = {0x01};
      cmd(CMD_SET_STANDBY, stdby_xosc, 1);
      uint8_t pkt_params_alt[6] = {
          0x00, 0x0C, /* Preamble length = 12 symbols */
          0x01,       /* Implicit header */
          0x08,       /* 8 byte payload (OTA4) */
          0x00,       /* CRC off */
          cur_iq      /* IQ: alternated */
      };
      cmd(CMD_SET_PKT_PARAM, pkt_params_alt, 6);
      clear_irq();
      uint8_t rx_cont[3] = {0xFF, 0xFF, 0xFF};
      cmd(CMD_SET_RX, rx_cont, 3);
    }
#endif

    /* Read all 3 layers */
    dio9_pin = lr1121_dio1_read();
    isr_count = lr1121_dio1_get_isr_count();

    /* Read IRQ status via SPI - use the 2-phase proven protocol */
    uint32_t irq = read_irq_status();

    /* Check chip mode - should still be RX (4) */
    mode = get_chip_mode();

    /* Instantaneous RSSI — valid while in RX. Gives a visibility window
     * into the RF floor vs. actual signal even when no packet decodes. */
    int rssi_inst = read_rssi_inst();

    /* Print status line */
    DEBUGOUT("[%02lu] mode=%d DIO9=%d ISR=%lu IRQ=0x%08lX RSSI=%d",
             (unsigned long)poll_count, mode, dio9_pin,
             (unsigned long)isr_count, (unsigned long)irq, rssi_inst);

    /* Decode IRQ flags */
    if (irq & IRQ_RX_DONE) {
      rx_done_count++;
      DEBUGOUT(" << RX_DONE!");

      /* Read the packet */
      uint8_t payload[256] = {0};
      uint8_t pkt_len = 0;
      if (read_rx_packet(payload, &pkt_len)) {
        DEBUGOUT("\n  PACKET RECEIVED! len=%d data:", pkt_len);
        for (int j = 0; j < pkt_len && j < 32; j++) {
          DEBUGOUT(" %02X", payload[j]);
        }
      }

      /* Read packet status (RSSI, SNR) */
      uint8_t pkt_status[4] = {0};
      if (cmd_read(CMD_GET_PKT_STATUS, NULL, 0, pkt_status, 4)) {
        int8_t rssi = -(int8_t)(pkt_status[1] / 2);
        int8_t snr = (int8_t)pkt_status[2] / 4;
        DEBUGOUT("\n  RSSI=%d dBm, SNR=%d dB", rssi, snr);
      }

      /* Clear IRQ so we can detect the next one */
      clear_irq();

      /* Re-enter RX mode in case it fell back to standby */
      cmd(CMD_SET_RX, rx_params, 3);
    }
    if (irq & IRQ_TIMEOUT)
      DEBUGOUT(" TIMEOUT");
    if (irq & IRQ_CRC_ERROR)
      DEBUGOUT(" CRC_ERR");
    if (irq & IRQ_TX_DONE)
      DEBUGOUT(" TX_DONE(?!)");

    DEBUGOUT("\n");

    /* Clear any non-RX_DONE latched IRQs so fresh events register on
     * the next poll instead of us staring at the same 0x50 forever. */
    if (irq && !(irq & IRQ_RX_DONE)) {
      clear_irq();
      /* Re-arm RX in case the radio fell back to standby. */
      cmd(CMD_SET_RX, rx_params, 3);
    }

    /* If DIO9 is stuck high, try clearing IRQ */
    if (dio9_pin == 1 && !(irq & IRQ_RX_DONE)) {
      DEBUGOUT("  NOTE: DIO9 HIGH but no RX_DONE in IRQ. Clearing...\n");
      clear_irq();
    }
  }

  /* =========================================================================
   * RESULTS SUMMARY
   * =========================================================================
   */
  DEBUGOUT("\n");
  DEBUGOUT("╔═══════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║                    TEST RESULTS                         ║\n");
  DEBUGOUT("╚═══════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("  Total polls:    %lu\n", (unsigned long)poll_count);
  DEBUGOUT("  RX_DONE count:  %lu\n", (unsigned long)rx_done_count);
  DEBUGOUT("  ISR count:      %lu\n",
           (unsigned long)lr1121_dio1_get_isr_count());
  DEBUGOUT("  Final DIO9:     %d\n", lr1121_dio1_read());
  DEBUGOUT("  Final mode:     %d\n", get_chip_mode());
  DEBUGOUT("\n");

  if (rx_done_count > 0) {
    DEBUGOUT("  ✓ RADIO IS RECEIVING PACKETS!\n");
    DEBUGOUT("    The LR1121 hardware and SPI are working.\n");
    if (lr1121_dio1_get_isr_count() > 0) {
      DEBUGOUT("  ✓ GPIO_46 INTERRUPT IS WORKING!\n");
      DEBUGOUT("    The problem is in the ELRS C++ driver layer.\n");
    } else {
      DEBUGOUT("  ✗ GPIO_46 INTERRUPT NOT FIRING\n");
      DEBUGOUT("    DIO9 → GPIO_46 wiring or interrupt config is broken.\n");
    }
  } else {
    DEBUGOUT("  ✗ NO PACKETS RECEIVED\n");
    DEBUGOUT("    Possible causes:\n");
#if LR1121_BAND_24GHZ
    DEBUGOUT("    1. No 2.4 GHz LoRa transmitter active at 2450 MHz\n");
    DEBUGOUT("    2. RFIO_HF pin not wired to antenna on this board variant\n");
    DEBUGOUT("    3. Frequency mismatch (check TX freq)\n");
    DEBUGOUT("    4. Modulation mismatch (SF/BW/CR)\n");
    DEBUGOUT("    Note: chip mode=RX and SPI responses prove the command\n");
    DEBUGOUT("    path works even if no RF energy is present.\n");
#else
    DEBUGOUT("    1. No transmitter active on 915.5 MHz\n");
    DEBUGOUT("    2. RF switch (PE4259) not working\n");
    DEBUGOUT("    3. Antenna not connected\n");
    DEBUGOUT("    4. Frequency mismatch (check TX freq)\n");
    DEBUGOUT("    5. Modulation mismatch (SF/BW/CR)\n");
#endif
  }
  DEBUGOUT("\n");
}
