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
 * LoRa Configuration:
 *   - Frequency: 915.5 MHz (middle of FCC915 band)
 *   - SF9, BW500, CR4/7 (wide/sensitive for max chance of catching packets)
 *   - Packet type: LoRa, implicit header, 8-byte payload
 *   - Continuous RX (no timeout)
 *
 * Hardware: SiWG917Y (BRD2708A) + LR1121 on mikroBUS
 * Citations:
 *   - LR1121 User Manual Section 7-9 (Radio Commands)
 *   - LR1121 Datasheet Table 5-2 (SetDioIrqParams)
 */

#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
#include "rsi_debug.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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
#define CMD_SET_DIO_AS_RF_SWITCH 0x0112
#define CMD_GET_PKT_STATUS 0x0204

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
 * Helper: Clear all IRQ flags
 ******************************************************************************/
static void clear_irq(void) {
  uint8_t buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  cmd(CMD_CLEAR_IRQ, buf, 4);
}

/*******************************************************************************
 * Helper: Read received packet from RX buffer
 ******************************************************************************/
static bool read_rx_packet(uint8_t *payload, uint8_t *length) {
  /* GetRxBufferStatus: returns [payloadLen, rxStartBufPtr] */
  uint8_t buf_status[4] = {0};
  if (!cmd_read(CMD_GET_RX_BUFFER_STATUS, NULL, 0, buf_status, 4)) {
    return false;
  }

  DEBUGOUT("  RxBufStatus: [%02X %02X %02X %02X]\n", buf_status[0],
           buf_status[1], buf_status[2], buf_status[3]);

  uint8_t payload_len = buf_status[1];
  uint8_t start_ptr = buf_status[2];

  if (payload_len == 0) {
    *length = 0;
    return false;
  }

  *length = payload_len;

  /* ReadBuffer8: offset, then read payload_len bytes */
  uint8_t offset[1] = {start_ptr};
  uint8_t read_buf[258] = {0}; /* stat1 + up to 256 bytes */
  if (!cmd_read(CMD_READ_BUFFER8, offset, 1, read_buf, payload_len + 1)) {
    return false;
  }

  /* Skip stat1 byte */
  memcpy(payload, &read_buf[1], payload_len);
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
   * STEP 2: Configure RF switch (PE4259)
   * =========================================================================
   */
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
   * STEP 5: Set RF frequency to 915.5 MHz
   * Citation: LR1121 User Manual Section 7.2.3 SetRfFrequency
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 5: SetRfFrequency(915.5 MHz) ===\n");
  uint32_t freq = 915500000;
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
   *   SF9, BW500, CR4/5, LowDataRateOptimize=off
   * Using wide/sensitive settings for maximum receive chance
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 6: SetModulationParams(SF9, BW500, CR4/5) ===\n");
  uint8_t mod_params[4] = {
      0x09, /* SF9 */
      0x06, /* BW500 */
      0x01, /* CR 4/5 */
      0x00  /* LowDataRateOptimize off */
  };
  if (!cmd(CMD_SET_MODULATION_PARAM, mod_params, 4)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK\n");

  /* =========================================================================
   * STEP 7: Set LoRa packet parameters
   * Citation: LR1121 User Manual Section 8.3.2 SetPacketParams
   *   Preamble=12, Explicit header, Payload=255 (max), CRC on, standard IQ
   * Using EXPLICIT header + CRC so any LoRa packet on this frequency is caught
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 7: SetPacketParams(Preamble=12, Explicit, 255B, CRC on) "
           "===\n");
  uint8_t pkt_params[6] = {
      0x00, 0x0C, /* Preamble length = 12 symbols (MSB, LSB) */
      0x00,       /* Explicit header (variable length) */
      0xFF,       /* Max payload length = 255 */
      0x01,       /* CRC on */
      0x00        /* Standard IQ (not inverted) */
  };
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
   * STEP 9: Set DIO IRQ params - route ALL IRQs to DIO9 (Dio2Mask)
   * Citation: LR1121 Datasheet Table 5-2
   *   Bytes 0-3:  IrqMask  = 0xFFFFFFFF (all IRQs)
   *   Bytes 4-7:  Dio1Mask = 0x00000000 (DIO1 is NSS, skip)
   *   Bytes 8-11: Dio2Mask = 0xFFFFFFFF (route all to DIO9)
   * =========================================================================
   */
  DEBUGOUT("\n=== STEP 9: SetDioIrqParams (ALL → DIO9 via Dio2Mask) ===\n");
  uint8_t irq_params[12] = {0};
  /* IrqMask: all IRQs */
  irq_params[0] = 0xFF;
  irq_params[1] = 0xFF;
  irq_params[2] = 0xFF;
  irq_params[3] = 0xFF;
  /* Dio1Mask: skip (DIO1 = NSS) */
  /* Dio2Mask: all IRQs → DIO9 */
  irq_params[8] = 0xFF;
  irq_params[9] = 0xFF;
  irq_params[10] = 0xFF;
  irq_params[11] = 0xFF;
  if (!cmd(CMD_SET_DIO_IRQ_PARAMS, irq_params, 12)) {
    DEBUGOUT("  FAILED!\n");
    return;
  }
  DEBUGOUT("  OK: All IRQs enabled, routed to DIO9 via 12-byte Dio2Mask\n");

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
  DEBUGOUT("  Turn on your ELRS TX now! Any 915 MHz LoRa signal will do.\n");
  DEBUGOUT("  Polling every 500ms...\n\n");

  uint32_t rx_done_count = 0;
  uint32_t poll_count = 0;

  for (int i = 0; i < 60; i++) { /* 60 x 500ms = 30 seconds */
    delay_ms(500);
    poll_count++;

    /* Read all 3 layers */
    dio9_pin = lr1121_dio1_read();
    isr_count = lr1121_dio1_get_isr_count();

    /* Read IRQ status via SPI - use the 2-phase proven protocol */
    uint32_t irq = read_irq_status();

    /* Check chip mode - should still be RX (4) */
    mode = get_chip_mode();

    /* Print status line */
    DEBUGOUT("[%02lu] mode=%d DIO9=%d ISR=%lu IRQ=0x%08lX",
             (unsigned long)poll_count, mode, dio9_pin,
             (unsigned long)isr_count, (unsigned long)irq);

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
    DEBUGOUT("    1. No transmitter active on 915.5 MHz\n");
    DEBUGOUT("    2. RF switch (PE4259) not working\n");
    DEBUGOUT("    3. Antenna not connected\n");
    DEBUGOUT("    4. Frequency mismatch (check TX freq)\n");
    DEBUGOUT("    5. Modulation mismatch (SF/BW/CR)\n");
  }
  DEBUGOUT("\n");
}
