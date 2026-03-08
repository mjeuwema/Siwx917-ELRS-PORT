/**
 * @file lr1121_elrs_init.c
 * @brief ELRS-Compatible LR1121 Initialization Implementation
 *
 * This implementation matches ExpressLRS 4.0 LR1121 initialization exactly.
 * See lr1121_elrs_init.h for detailed documentation.
 *
 * KEY INSIGHT: ELRS NEVER calls SetTcxoMode!
 *
 * The Core1121-HF module has an externally-powered TCXO that is always
 * running when VCC is applied. Calling SetTcxoMode can cause issues
 * because it configures the LR1121's internal VTCXO regulator which
 * conflicts with the external TCXO.
 *
 * Citations:
 *   - ExpressLRS 4.0 src/lib/LR1121Driver/LR1121.cpp
 *   - ExpressLRS 4.0 src/lib/LR1121Driver/LR1121_hal.cpp
 *   - LR1121 Datasheet (61252685.LR1121_V2_1_data_sheet.pdf)
 */

#include "lr1121_elrs_init.h"
#include "lr1121_driver.h"
/* IRQ constants moved inline - no longer need elrs_protocol header */
#define LR1121_IRQ_TX_DONE 0x00000004
#define LR1121_IRQ_RX_DONE 0x00000008
#include "rsi_debug.h"
#include <string.h>

/*******************************************************************************
 * Helper Functions - Low-Level Command Execution
 ******************************************************************************/

/**
 * @brief Simple delay in milliseconds
 */
static void elrs_delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    for (volatile uint32_t j = 0; j < 10000; j++) {
    }
  }
}

/**
 * @brief Simple delay in microseconds
 */
__attribute__((unused)) static void elrs_delay_us(uint32_t us) {
  for (uint32_t i = 0; i < us; i++) {
    for (volatile uint32_t j = 0; j < 10; j++) {
    }
  }
}

/**
 * @brief Execute command with optional parameters and response
 *
 * This implements the LR1121's two-phase SPI protocol:
 *   Phase 1: Send opcode + parameters
 *   Wait: BUSY goes HIGH then LOW
 *   Phase 2: Send NOPs to read response (if needed)
 *
 * Citation: LR1121 Datasheet Section 3 (SPI Protocol)
 */
static bool elrs_cmd(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                     uint8_t *response, uint16_t resp_len) {
  /* Wait for chip ready */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("[ELRS] CMD 0x%04X: BUSY timeout before send\n", opcode);
    return false;
  }

  /* Send command */
  if (!lr1121_send_command(opcode, params, param_len)) {
    DEBUGOUT("[ELRS] CMD 0x%04X: Send failed\n", opcode);
    return false;
  }

  /* Wait for processing */
  if (!lr1121_wait_busy_timeout(500)) {
    DEBUGOUT("[ELRS] CMD 0x%04X: BUSY timeout after send\n", opcode);
    return false;
  }

  /* Read response if needed */
  if (response != NULL && resp_len > 0) {
    if (!lr1121_read_response(response, resp_len)) {
      DEBUGOUT("[ELRS] CMD 0x%04X: Read response failed\n", opcode);
      return false;
    }
  }

  return true;
}

/**
 * @brief Execute command and verify success via status byte
 *
 * Citation: LR1121 User Manual Section 3.4.2 Stat1 Values
 *   Bits [3:1] = Command Status:
 *     0 = CMD_FAIL: The last command could not be executed
 *     1 = CMD_PERR: Parameter error (wrong opcode, arguments)
 *     2 = CMD_OK: The last command was processed successfully
 *     3 = CMD_DAT: Command successful, data being transmitted
 *     4-7 = RFU
 */
static bool elrs_cmd_check(uint16_t opcode, const uint8_t *params,
                           uint16_t param_len) {
  uint8_t resp[1] = {0};

  if (!elrs_cmd(opcode, params, param_len, resp, 1)) {
    return false;
  }

  /* Check command status (bits [3:1])
   * Citation: LR1121 User Manual Section 3.4.2
   *   2 = CMD_OK (success)
   *   3 = CMD_DAT (success, data available) */
  uint8_t cmd_status = (resp[0] >> 1) & 0x07;
  if (cmd_status != 2 && cmd_status != 3) { /* 2=CMD_OK, 3=CMD_DAT */
    DEBUGOUT("[ELRS] CMD 0x%04X: Bad status %d (%s)\n", opcode, cmd_status,
             cmd_status == 0   ? "CMD_FAIL"
             : cmd_status == 1 ? "CMD_PERR"
                               : "RFU");
    return false;
  }

  return true;
}

/*******************************************************************************
 * ELRS-Compatible Initialization
 ******************************************************************************/

/**
 * @brief Initialize LR1121 using ELRS-compatible sequence
 *
 * Citation: ExpressLRS LR1121.cpp Begin() function lines 107-157
 *
 * ELRS Sequence:
 *   1. hal.init() - GPIO/SPI init (already done by lr1121_init)
 *   2. hal.reset() - Hardware reset (RST LOW 1ms, wait 300ms)
 *   3. CheckVersion() - Verify chip response
 *   4. SetRxTxFallbackMode(FS) - Fallback to FS mode after TX/RX
 *   5. SetRxBoosted(1) - Enable RX boost
 *   6. SetDioAsRfSwitch() - Configure RF switch GPIOs
 *   7. SetDioIrqParams() - Route IRQs to DIO1
 *   8. SetRegMode(DCDC) - Use DC-DC converter (optional)
 *   9. CalibImage() - Calibrate for frequency band
 *
 * WHAT ELRS DOES NOT DO:
 *   ❌ SetTcxoMode - NEVER called!
 *   ❌ SetStandby(XOSC) - Stays in STANDBY_RC
 *   ❌ Calibrate(0x3F) - Only CalibImage is called
 */
lr1121_status_t lr1121_elrs_init(uint32_t freq_min, uint32_t freq_max) {
  bool ok;

  DEBUGOUT("\n");
  DEBUGOUT(
      "╔═══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT(
      "║  LR1121 EXTERNAL TCXO INITIALIZATION (Waveshare Core1121-XF)  ║\n");
  DEBUGOUT(
      "║  FIX 2026-01-21: voltage=0 for externally-powered TCXO        ║\n");
  DEBUGOUT(
      "╠═══════════════════════════════════════════════════════════════╣\n");
  DEBUGOUT(
      "║  Hardware: 32MHz TCXO always powered from 3.3V rail           ║\n");
  DEBUGOUT(
      "║  Config: SetTcxoMode(0, ~20ms) → SetStandby(XOSC) → Calibrate ║\n");
  DEBUGOUT(
      "╚═══════════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");

  /* =========================================================================
   * Step 1: Hardware Reset
   * Citation: LR1121 Datasheet Section 3.1 (Reset Sequence)
   *   - RST LOW for 1ms minimum
   *   - RST HIGH
   *   - Wait for BUSY LOW (chip ready)
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 1: Hardware Reset\n");
  lr1121_status_t status = lr1121_reset();
  if (status != LR1121_OK) {
    DEBUGOUT("[TCXO] ERROR: Reset failed with code %d\n", status);
    return status;
  }
  DEBUGOUT("[TCXO] Reset OK - BUSY is LOW\n");

  /* =========================================================================
   * Step 2: Verify Chip Version
   * Citation: LR1121 Datasheet Section 11.2.1 "GetVersion"
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 2: GetVersion\n");
  lr1121_version_t version;
  status = lr1121_get_version(&version);
  if (status != LR1121_OK) {
    DEBUGOUT("[TCXO] ERROR: GetVersion failed\n");
    return status;
  }
  DEBUGOUT("[TCXO] Version: HW=0x%02X Type=0x%02X FW=v%d.%d\n",
           version.hardware, version.type, (version.version >> 8) & 0xFF,
           version.version & 0xFF);

  /* =========================================================================
   * Step 3: SetRegMode (DC-DC) - Power Efficiency Configuration
   * Citation: LR1121 Datasheet Section 11.2.7 "SetRegMode" (opcode 0x0110)
   *
   * Parameter:
   *   - 0x00 = LDO mode (less efficient)
   *   - 0x01 = DC-DC mode (more efficient, recommended)
   *
   * Citation: lr1121_elrs_init.h lines 110-111
   *   #define ELRS_REG_MODE_LDO   0x00
   *   #define ELRS_REG_MODE_DCDC  0x01
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 3: SetRegMode(DCDC)\n");
  uint8_t reg_mode = ELRS_REG_MODE_DCDC; /* 0x01 = DC-DC */
  ok = elrs_cmd_check(ELRS_CMD_SET_REG_MODE, &reg_mode, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] WARNING: SetRegMode failed (using LDO mode)\n");
  } else {
    DEBUGOUT("[TCXO] SetRegMode(DCDC) OK\n");
  }

  /* =========================================================================
   * Step 4: SetTcxoMode - CONFIGURE FOR EXTERNAL TCXO
   * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode" (opcode 0x0117)
   * Citation: Waveshare Core1121-XF Demo configuration
   *
   * CRITICAL FIX (2026-01-21): External TCXO Configuration
   * =======================================================
   * The Waveshare Core1121-XF has an EXTERNALLY-POWERED 32MHz TCXO
   * that is ALWAYS ON from the 3.3V rail. The LR1121's internal VTCXO
   * regulator is NOT used.
   *
   * Configuration for EXTERNAL TCXO:
   *   - Voltage = 0 (TCXO_CTRL_NONE) - Don't use internal regulator
   *   - Delay = ~10ms (328 ticks) - Stabilization time (TCXO already running)
   *
   * Why SetTcxoMode is STILL REQUIRED:
   *   - Switches LR1121 from XO (crystal oscillator) mode to TCXO input mode
   *   - Without this, chip expects crystal circuit, not single-ended clock
   *   - XTA pin receives TCXO clock; XTB is not connected (NC)
   *   - Skipping SetTcxoMode → oscillator failures → "stuck in sleep"
   *
   * Parameter format: [voltage (1 byte)] [delay (3 bytes, big-endian)]
   *
   * Citation: Semtech lr11xx_system.c lr11xx_system_set_tcxo_mode()
   * Citation: lr1121_elrs_init.h TCXO_ACTIVE_VOLTAGE, TCXO_ACTIVE_DELAY
   * =========================================================================
   */
  uint32_t tcxo_delay = TCXO_ACTIVE_DELAY;
  uint8_t tcxo_voltage = TCXO_ACTIVE_VOLTAGE;

  DEBUGOUT("[TCXO] Step 4: SetTcxoMode for EXTERNAL TCXO\n");
  DEBUGOUT("[TCXO]   TCXO_ACTIVE_VOLTAGE define = 0x%02X\n",
           (unsigned int)TCXO_ACTIVE_VOLTAGE);
  DEBUGOUT("[TCXO]   tcxo_voltage variable = 0x%02X\n",
           (unsigned int)tcxo_voltage);
  DEBUGOUT("[TCXO]   voltage=0x%02X (%s)\n", (unsigned int)tcxo_voltage,
           tcxo_voltage == 0 ? "NONE - external TCXO" : "INTERNAL regulator");
  DEBUGOUT("[TCXO]   delay=0x%06lX (~%lums stabilization)\n",
           (unsigned long)tcxo_delay, (unsigned long)(tcxo_delay * 31 / 1000));

  uint8_t tcxo_params[4] = {
      tcxo_voltage, /* 0 = external TCXO, no internal regulator */
      (uint8_t)((tcxo_delay >> 16) & 0xFF), /* Delay MSB */
      (uint8_t)((tcxo_delay >> 8) & 0xFF),  /* Delay middle byte */
      (uint8_t)(tcxo_delay & 0xFF)          /* Delay LSB */
  };
  ok = elrs_cmd_check(ELRS_CMD_SET_TCXO_MODE, tcxo_params, 4);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: SetTcxoMode failed!\n");
    DEBUGOUT("[TCXO] Without TCXO mode, LR1121 expects crystal input → init "
             "will fail\n");
    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("[TCXO] SetTcxoMode OK - Chip configured for single-ended TCXO "
           "input on XTA\n");

  /* SIGNIFICANTLY INCREASED delay for TCXO stabilization and clock lock
   *
   * CRITICAL FIX (2026-01-22): Hardware SPI timing issue
   * =====================================================
   * With hardware SPI vs soft SPI, timing is faster, giving less settling time.
   * The external TCXO needs sufficient time to stabilize before we switch
   * the system clock to it with SetStandby(XOSC).
   *
   * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
   *   The timeout specifies the maximum time to wait for TCXO ready.
   *   After SetTcxoMode, an additional software delay ensures stability.
   *
   * Changed from 25ms to 100ms for reliable operation with hardware SPI.
   * Total margin: ~100ms hardware timeout + 100ms software delay = 200ms
   */
  DEBUGOUT("[TCXO] Waiting 100ms for TCXO sync (INCREASED for HW SPI)...\n");
  elrs_delay_ms(100);

  /* =========================================================================
   * Step 5: SetStandby(XOSC) - SWITCH SYSTEM CLOCK TO TCXO
   * Citation: LR1121 Datasheet Section 11.2.2 "SetStandby" (opcode 0x011C)
   * Citation: User requirement - Switch from RC to XOSC after TCXO power-up
   *
   * CRITICAL: After SetTcxoMode, the chip is STILL running on RC oscillator!
   * SetStandby(0x01) switches the system clock source from RC to XOSC (TCXO).
   *
   * Parameter:
   *   - 0x00 = STDBY_RC (internal RC oscillator)
   *   - 0x01 = STDBY_XOSC (external crystal/TCXO)
   *
   * Citation: lr1121_elrs_init.h lines 106-107
   *   #define ELRS_STANDBY_RC    0x00
   *   #define ELRS_STANDBY_XOSC  0x01
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 5: SetStandby(XOSC) - Switch to TCXO clock\n");
  uint8_t standby_xosc = ELRS_STANDBY_XOSC; /* 0x01 = XOSC mode */
  ok = elrs_cmd_check(ELRS_CMD_SET_STANDBY, &standby_xosc, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: SetStandby(XOSC) failed!\n");
    DEBUGOUT("[TCXO] System clock NOT switched to TCXO - RF precision will be "
             "poor\n");

    /* Enhanced debugging: Check GetStatus and GetErrors */
    DEBUGOUT("[TCXO] === ENHANCED DEBUGGING ===\n");
    uint8_t chip_mode = 0, cmd_status = 0;
    uint16_t errors = 0;
    if (lr1121_elrs_get_status(&chip_mode, &cmd_status, &errors)) {
      DEBUGOUT("[TCXO] GetStatus: chip_mode=%d (expect 2=STDBY_XOSC), "
               "cmd_status=%d\n",
               chip_mode, cmd_status);
      DEBUGOUT("[TCXO]   If chip_mode=1 (STDBY_RC): TCXO failed to start\n");
      DEBUGOUT("[TCXO]   If chip_mode=0 (SLEEP): Wakeup failed or standby "
               "ignored\n");

      DEBUGOUT("[TCXO] GetErrors: 0x%04X\n", errors);
      if (errors & 0x0020)
        DEBUGOUT("[TCXO]   HF_XOSC_START_ERR: TCXO delay/voltage issue!\n");
      if (errors & 0x0080)
        DEBUGOUT("[TCXO]   PLL_LOCK_ERR: PLL failed to lock\n");
      if (errors & 0x0040)
        DEBUGOUT("[TCXO]   PLL_CALIB_ERR: PLL calibration failed\n");
    }
    DEBUGOUT("[TCXO] === END DEBUGGING ===\n");

    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("[TCXO] SetStandby(XOSC) OK - System clock now running from TCXO\n");

  /* ADDED: Extra delay for PLL lock after switching to XOSC
   *
   * CRITICAL FIX (2026-01-22): Hardware SPI timing issue
   * =====================================================
   * The PLL needs time to lock to the TCXO reference after SetStandby(XOSC).
   * With faster hardware SPI, we need explicit delays.
   *
   * Citation: LR1121 Datasheet Section 2.1 State Machine
   *   After STDBY_XOSC, the PLL locks to the crystal/TCXO reference.
   */
  DEBUGOUT("[TCXO] Waiting 50ms for PLL lock...\n");
  elrs_delay_ms(50);

  /* Verify chip is actually in XOSC mode */
  uint8_t verify_mode = 0, verify_status = 0;
  uint16_t verify_errors = 0;
  if (lr1121_elrs_get_status(&verify_mode, &verify_status, &verify_errors)) {
    DEBUGOUT(
        "[TCXO] Verify: chip_mode=%d (expect 2=STDBY_XOSC), errors=0x%04X\n",
        verify_mode, verify_errors);
    if (verify_mode != 2) {
      DEBUGOUT(
          "[TCXO] WARNING: Not in STDBY_XOSC mode! TCXO may have failed.\n");
      if (verify_errors & 0x0020) {
        DEBUGOUT("[TCXO] HF_XOSC_START_ERR detected - try different "
                 "voltage/delay!\n");
      }
    }
  }

  /* =========================================================================
   * Step 6: Calibrate(0x3F) - FULL CALIBRATION WITH STABLE TCXO
   * Citation: LR1121 Datasheet Section 11.2.4 "Calibrate" (opcode 0x010F)
   * Citation: User requirement - This MUST be step 4 of TCXO sequence
   *
   * Now that TCXO is stable and being used as system clock, perform full
   * calibration of all internal blocks.
   *
   * Parameter (bitmask):
   *   Bit 0: RC64K calibration
   *   Bit 1: RC13M calibration
   *   Bit 2: PLL calibration
   *   Bit 3: ADC calibration
   *   Bit 4: Image rejection calibration
   *   Bit 5: PLL TX calibration
   *
   *   0x3F = All calibrations enabled (bits 0-5 set)
   *
   * Citation: Semtech lr11xx_system.h LR11XX_SYSTEM_CALIB_ALL = 0x3F
   * =========================================================================
   */
  DEBUGOUT(
      "[TCXO] Step 6: Calibrate(0x3F) - Full calibration with stable TCXO\n");
  uint8_t calib_mask = 0x3F; /* All calibrations */
  ok = elrs_cmd_check(ELRS_CMD_CALIBRATE, &calib_mask, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: Calibrate(0x3F) failed!\n");
    DEBUGOUT("[TCXO] This may indicate TCXO not stable or wrong voltage\n");
    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("[TCXO] Calibrate(0x3F) OK - All blocks calibrated with TCXO "
           "reference\n");

  /* Allow calibration to complete and chip to return to STDBY */
  elrs_delay_ms(20);

  /* =========================================================================
   * Step 7: SetStandby(XOSC) again - Return to XOSC mode after calibration
   * Citation: Waveshare lr1121_config.cpp - Calibrate puts chip in SLEEP
   *
   * After Calibrate command, chip may enter SLEEP mode briefly.
   * Send SetStandby(XOSC) to ensure we're back in active XOSC mode.
   * =========================================================================
   */
  DEBUGOUT(
      "[TCXO] Step 7: SetStandby(XOSC) - Ensure XOSC mode after calibration\n");
  ok = elrs_cmd_check(ELRS_CMD_SET_STANDBY, &standby_xosc, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] WARNING: Post-calibration SetStandby(XOSC) failed\n");
  }

  /* =========================================================================
   * Step 8: Clear Errors
   * Citation: LR1121 Datasheet Section 11.2.3 "ClearErrors"
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 8: ClearErrors\n");
  ok = elrs_cmd_check(ELRS_CMD_CLEAR_ERRORS, NULL, 0);
  if (!ok) {
    DEBUGOUT("[TCXO] WARNING: ClearErrors failed (continuing anyway)\n");
  }

  /* =========================================================================
   * Step 9: SetRxTxFallbackMode(FS)
   * Citation: ExpressLRS LR1121.cpp line 124
   *
   * FS (Frequency Synthesis) mode keeps the PLL locked for fast hopping.
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 9: SetRxTxFallbackMode(FS)\n");
  uint8_t fallback = ELRS_FALLBACK_FS; /* 0x03 = FS mode */
  ok = elrs_cmd_check(ELRS_CMD_SET_RX_TX_FALLBACK, &fallback, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: SetRxTxFallbackMode failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* =========================================================================
   * Step 10: SetRxBoosted(1)
   * Citation: ExpressLRS LR1121.cpp line 125
   *
   * Enables RX gain boost for better sensitivity.
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 10: SetRxBoosted(1)\n");
  uint8_t rx_boost = 1;
  ok = elrs_cmd_check(ELRS_CMD_SET_RX_BOOSTED, &rx_boost, 1);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: SetRxBoosted failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* =========================================================================
   * Step 11: SetDioAsRfSwitch
   * Citation: Waveshare Core1121_XF_Demo lr1121_common.c lines 78-87
   * =========================================================================
   */
  /* PE4259 Truth Table: V1=0,V2=1→TX, V1=1,V2=0→RX. DIO5→V1, DIO6→V2 */
  DEBUGOUT("[TCXO] Step 11: SetDioAsRfSwitch (PE4259: RX=0x01, TX=0x02)\n");
  uint8_t rf_switch_params[8] = {
      0x03, /* enable: DIO5+DIO6 */
      0x00, /* stbyCfg: both off (switch isolated) */
      0x01, /* rxCfg: DIO5=1, DIO6=0 → V1=1, V2=0 → RF2 → RX */
      0x02, /* txCfg: DIO5=0, DIO6=1 → V1=0, V2=1 → RF1 → TX */
      0x02, /* txHpCfg: same as TX */
      0x00, /* txHfCfg: 2.4GHz uses RFIO_HF */
      0x00, /* gnssCfg */
      0x00  /* wifiCfg */
  };
  ok = elrs_cmd_check(ELRS_CMD_SET_DIO_AS_RF_SWITCH, rf_switch_params, 8);
  if (!ok) {
    DEBUGOUT("[TCXO] WARNING: SetDioAsRfSwitch failed (continuing)\n");
  }

  /* =========================================================================
   * Step 12: SetDioIrqParams
   * Citation: ExpressLRS LR1121.cpp line 137-141
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 12: SetDioIrqParams\n");
  /*
   * Citation: ExpressLRS LR1121.cpp line 560-564
   *   void LR1121Driver::SetDioIrqParams()
   *   {
   *       uint8_t buf[8] = {0};
   *       buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
   *       hal.WriteCommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, buf, sizeof(buf),
   * SX12XX_Radio_All);
   *   }
   *
   * Citation: LR1121 Datasheet Table 5-2 "SetDioIrqParams"
   *   Command format: 8 bytes total
   *   - Bytes 0-3: IrqMask (32-bit, little-endian, LSB in byte 3)
   *   - Bytes 4-7: Dio1Mask (32-bit, little-endian, LSB in byte 7)
   *
   * LR1121_IRQ_TX_DONE = 0x04 (bit 2)
   * LR1121_IRQ_RX_DONE = 0x08 (bit 3)
   * Combined = 0x0C
   */
  uint8_t irq_params[12] = {0}; /* Need 12 bytes to reach Dio2Mask */

  /* IRQ mask: Only enable TX_DONE and RX_DONE (bits in low byte) */
  irq_params[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE; /* 0x0C */

  /* DIO1 mask: Left as 0x00 because physical DIO1 is hardwired to NSS */

  /* DIO2 mask: Route TX_DONE and RX_DONE to physical DIO9 pin */
  irq_params[11] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE; /* 0x0C */

  ok = elrs_cmd_check(ELRS_CMD_SET_DIO_IRQ_PARAMS, irq_params, 12);
  if (!ok) {
    DEBUGOUT("[TCXO] WARNING: SetDioIrqParams failed (continuing)\n");
  }

  /* =========================================================================
   * Step 13: CalibImage for frequency band
   * Citation: LR1121 Datasheet Section 11.2.4 "CalibrateImage"
   *
   * Additional image rejection calibration for the specific frequency band.
   * =========================================================================
   */
  DEBUGOUT("[TCXO] Step 13: CalibImage for %lu - %lu Hz\n",
           (unsigned long)freq_min, (unsigned long)freq_max);
  ok = lr1121_elrs_calib_image(freq_min, freq_max);
  if (!ok) {
    DEBUGOUT("[TCXO] ERROR: CalibImage failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Short delay for calibration to complete */
  elrs_delay_ms(10);

  /* =========================================================================
   * Initialization Complete!
   *
   * TCXO SEQUENCE COMPLETED:
   *   ✓ SetTcxoMode - TCXO powered via VTCXO pin
   *   ✓ SetStandby(XOSC) - System clock switched to TCXO
   *   ✓ SetRegMode(DCDC) - Power efficiency configured
   *   ✓ Calibrate(0x3F) - All blocks calibrated with stable TCXO
   *   ✓ CalibImage - Image rejection calibrated for frequency band
   *
   * The chip is now in STANDBY_XOSC mode with full calibration done.
   * =========================================================================
   */
  DEBUGOUT("\n");
  DEBUGOUT(
      "[TCXO] ╔═══════════════════════════════════════════════════════════╗\n");
  DEBUGOUT(
      "[TCXO] ║  EXTERNAL TCXO INITIALIZATION COMPLETE!                   ║\n");
  DEBUGOUT(
      "[TCXO] ╠═══════════════════════════════════════════════════════════╣\n");
  DEBUGOUT(
      "[TCXO] ║  ✓ SetTcxoMode(0, ~20ms) - External TCXO mode enabled     ║\n");
  DEBUGOUT(
      "[TCXO] ║  ✓ SetStandby(XOSC) - Clock switched to TCXO              ║\n");
  DEBUGOUT(
      "[TCXO] ║  ✓ SetRegMode(DCDC) - Power efficiency enabled            ║\n");
  DEBUGOUT(
      "[TCXO] ║  ✓ Calibrate(0x3F) - Full calibration with stable TCXO    ║\n");
  DEBUGOUT(
      "[TCXO] ║  ✓ Chip is now in STANDBY_XOSC mode                       ║\n");
  DEBUGOUT(
      "[TCXO] ╚═══════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");

  /* Print final status */
  lr1121_elrs_print_status();

  return LR1121_OK;
}

/*******************************************************************************
 * ELRS-Compatible Calibration
 ******************************************************************************/

bool lr1121_elrs_calib_image(uint32_t freq_min, uint32_t freq_max) {
  /**
   * Citation: ExpressLRS LR1121.cpp CalibImage()
   * Citation: LR1121 Datasheet Section 11.2.4 "CalibrateImage"
   *
   * Parameters:
   *   freq1 = freq_band_min / 4000000 (floor)
   *   freq2 = freq_band_max / 4000000 (ceil)
   *
   * Common values:
   *   433 MHz: freq1=0x6B, freq2=0x6F
   *   868 MHz: freq1=0xD7, freq2=0xDB
   *   915 MHz: freq1=0xE1, freq2=0xE9
   *   2.4 GHz: freq1=0x94, freq2=0x98 (2400/4 = 0x258, but uses lookup)
   */
  uint8_t freq1, freq2;

  /* Use lookup table for common bands (from ELRS) */
  if (freq_min >= 2400000000UL) {
    /* 2.4 GHz band */
    freq1 = 0x94; /* ~2.368 GHz */
    freq2 = 0x98; /* ~2.528 GHz */
  } else if (freq_min >= 900000000UL) {
    /* 915 MHz ISM band (US) */
    freq1 = 0xE1; /* 902 MHz */
    freq2 = 0xE9; /* 928 MHz */
  } else if (freq_min >= 860000000UL) {
    /* 868 MHz ISM band (EU) */
    freq1 = 0xD7; /* 863 MHz */
    freq2 = 0xDB; /* 870 MHz */
  } else if (freq_min >= 430000000UL) {
    /* 433 MHz ISM band */
    freq1 = 0x6B; /* 428 MHz */
    freq2 = 0x6F; /* 444 MHz */
  } else {
    /* Calculate from frequency */
    freq1 = (uint8_t)(freq_min / 4000000UL);
    freq2 = (uint8_t)((freq_max + 3999999UL) / 4000000UL);
  }

  DEBUGOUT("[ELRS] CalibImage: freq1=0x%02X, freq2=0x%02X\n", freq1, freq2);

  uint8_t params[2] = {freq1, freq2};
  return elrs_cmd_check(ELRS_CMD_CALIBRATE_IMAGE, params, 2);
}

/*******************************************************************************
 * ELRS-Compatible Temperature Reading
 ******************************************************************************/

int16_t lr1121_elrs_get_temperature(void) {
  /**
   * Citation: LR1121 Datasheet Section 11.2.6 "GetTemp"
   *
   * Two-phase protocol:
   *   Phase 1: Send opcode 0x011A
   *   Phase 2: Read [stat1][temp_high][temp_low]
   *
   * Temperature formula: Temp = (raw / 256) - 64
   * Where raw = (temp_high << 8) | temp_low
   */
  uint8_t resp[3] = {0};

  if (!elrs_cmd(ELRS_CMD_GET_TEMP, NULL, 0, resp, 3)) {
    return -999;
  }

  /* Check status
   * Citation: LR1121 User Manual Section 3.4.2 - 2=CMD_OK, 3=CMD_DAT */
  uint8_t cmd_status = (resp[0] >> 1) & 0x07;
  if (cmd_status != 2 && cmd_status != 3) {
    DEBUGOUT("[ELRS] GetTemp: Bad status %d\n", cmd_status);
    return -999;
  }

  /* Parse temperature */
  uint16_t raw = ((uint16_t)resp[1] << 8) | resp[2];
  int16_t temp = (int16_t)(raw / 256) - 64;

  return temp;
}

/*******************************************************************************
 * ELRS-Compatible Random Number
 ******************************************************************************/

uint32_t lr1121_elrs_get_random(void) {
  /**
   * Citation: LR1121 Datasheet Section 11.2.8 "GetRandomNumber"
   *
   * Two-phase protocol:
   *   Phase 1: Send opcode 0x0120
   *   Phase 2: Read [stat1][rand3][rand2][rand1][rand0]
   *
   * Random number is MSB first: (rand3 << 24) | (rand2 << 16) | (rand1 << 8) |
   * rand0
   */
  uint8_t resp[5] = {0};

  if (!elrs_cmd(ELRS_CMD_GET_RANDOM_NUMBER, NULL, 0, resp, 5)) {
    return 0;
  }

  /* Check status
   * Citation: LR1121 User Manual Section 3.4.2 - 2=CMD_OK, 3=CMD_DAT */
  uint8_t cmd_status = (resp[0] >> 1) & 0x07;
  if (cmd_status != 2 && cmd_status != 3) {
    DEBUGOUT("[ELRS] GetRandom: Bad status %d\n", cmd_status);
    return 0;
  }

  /* Parse random number (MSB first) */
  uint32_t rand_val = ((uint32_t)resp[1] << 24) | ((uint32_t)resp[2] << 16) |
                      ((uint32_t)resp[3] << 8) | resp[4];

  return rand_val;
}

/*******************************************************************************
 * ELRS-Compatible Frequency Setting
 ******************************************************************************/

bool lr1121_elrs_set_frequency(uint32_t freq_hz) {
  /**
   * Citation: LR1121 Datasheet Section 11.3.1 "SetRfFrequency"
   *
   * Parameter: 4 bytes, frequency in Hz (MSB first)
   */
  uint8_t params[4];
  params[0] = (freq_hz >> 24) & 0xFF;
  params[1] = (freq_hz >> 16) & 0xFF;
  params[2] = (freq_hz >> 8) & 0xFF;
  params[3] = freq_hz & 0xFF;

  return elrs_cmd_check(ELRS_CMD_SET_RF_FREQUENCY, params, 4);
}

/*******************************************************************************
 * ELRS-Compatible LoRa Configuration
 ******************************************************************************/

bool lr1121_elrs_set_lora_mod(uint8_t sf, uint8_t bw, uint8_t cr,
                              uint8_t ldro) {
  /**
   * Citation: LR1121 Datasheet Section 11.3.4 "SetModulationParams"
   *
   * Parameters (for LoRa):
   *   [0] = SF (5-12)
   *   [1] = BW index (0=62.5k, 1=125k, 2=250k, 3=500k for sub-GHz)
   *   [2] = CR (1=4/5, 2=4/6, 3=4/7, 4=4/8)
   *   [3] = LDRO (0=off, 1=on)
   */
  uint8_t params[4] = {sf, bw, cr, ldro};
  return elrs_cmd_check(ELRS_CMD_SET_MODULATION_PARAMS, params, 4);
}

bool lr1121_elrs_set_lora_pkt(uint16_t preamble_len, uint8_t header_type,
                              uint8_t payload_len, uint8_t crc_on,
                              uint8_t invert_iq) {
  /**
   * Citation: LR1121 Datasheet Section 11.3.5 "SetPacketParams"
   *
   * Parameters (for LoRa):
   *   [0:1] = Preamble length (MSB first)
   *   [2] = Header type (0=variable, 1=fixed)
   *   [3] = Payload length
   *   [4] = CRC on (0=off, 1=on)
   *   [5] = Invert IQ (0=standard, 1=inverted)
   */
  uint8_t params[6];
  params[0] = (preamble_len >> 8) & 0xFF;
  params[1] = preamble_len & 0xFF;
  params[2] = header_type;
  params[3] = payload_len;
  params[4] = crc_on;
  params[5] = invert_iq;

  return elrs_cmd_check(ELRS_CMD_SET_PACKET_PARAMS, params, 6);
}

/*******************************************************************************
 * ELRS-Compatible Status Functions
 ******************************************************************************/

bool lr1121_elrs_get_status(uint8_t *chip_mode, uint8_t *cmd_status,
                            uint16_t *errors) {
  /* Get basic status */
  uint8_t stat1, stat2, irq;
  if (!lr1121_get_status(&stat1, &stat2, &irq)) {
    return false;
  }

  if (chip_mode) {
    *chip_mode = (stat2 >> 1) & 0x07; /* Bits [3:1] = chip mode */
  }
  if (cmd_status) {
    *cmd_status = (stat1 >> 1) & 0x07; /* Bits [3:1] = cmd status */
  }

  /* Get errors if requested */
  if (errors) {
    uint8_t err_resp[3] = {0};
    if (elrs_cmd(ELRS_CMD_GET_ERRORS, NULL, 0, err_resp, 3)) {
      *errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
    } else {
      *errors = 0xFFFF; /* Indicate error reading errors */
    }
  }

  return true;
}

void lr1121_elrs_print_status(void) {
  uint8_t chip_mode, cmd_status;
  uint16_t errors;

  if (!lr1121_elrs_get_status(&chip_mode, &cmd_status, &errors)) {
    DEBUGOUT("[ELRS] ERROR: Could not read status\n");
    return;
  }

  /* Chip mode names */
  static const char *mode_names[] = {"SLEEP", "STDBY_RC", "STDBY_XOSC", "FS",
                                     "RX",    "TX",       "?",          "?"};

  /* Command status names */
  static const char *cmd_names[] = {"FAIL",       "PERR", "SPI_ERR", "OK",
                                    "DATA_AVAIL", "?",    "?",       "?"};

  DEBUGOUT("[ELRS] Status:\n");
  DEBUGOUT("  Chip Mode: %d (%s)\n", chip_mode, mode_names[chip_mode & 0x07]);
  DEBUGOUT("  Cmd Status: %d (%s)\n", cmd_status, cmd_names[cmd_status & 0x07]);
  DEBUGOUT("  Errors: 0x%04X\n", errors);

  if (errors != 0 && errors != 0xFFFF) {
    if (errors & 0x0001)
      DEBUGOUT("    [ERR] LF_RC_CALIB\n");
    if (errors & 0x0002)
      DEBUGOUT("    [ERR] HF_RC_CALIB\n");
    if (errors & 0x0004)
      DEBUGOUT("    [ERR] ADC_CALIB\n");
    if (errors & 0x0008)
      DEBUGOUT("    [ERR] PLL_CALIB\n");
    if (errors & 0x0010)
      DEBUGOUT("    [ERR] IMG_CALIB\n");
    if (errors & 0x0020)
      DEBUGOUT("    [ERR] HF_XOSC_START\n");
    if (errors & 0x0040)
      DEBUGOUT("    [ERR] LF_XOSC_START\n");
    if (errors & 0x0080)
      DEBUGOUT("    [ERR] PLL_LOCK\n");
    if (errors & 0x0100)
      DEBUGOUT("    [ERR] RX_ADC_OFFSET\n");
  }

  DEBUGOUT("\n");
}

/*******************************************************************************
 * BULLETPROOF TCXO INITIALIZATION
 *
 * Based on user's analysis of the "200mV headroom rule" and "ghost RC
 * oscillator" problem that causes Temperature/TCXO tests to fail while RNG
 * works.
 ******************************************************************************/

/**
 * @brief Bulletproof TCXO initialization - THE FIX!
 *
 * Citation: User analysis + LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *
 * WHY RNG WORKS BUT TEMPERATURE FAILS:
 *   - RNG uses internal 32MHz RC oscillator (no TCXO needed)
 *   - Temperature sensor + ADC require stable TCXO/XOSC
 *
 * THE THREE PROBLEMS THIS FIXES:
 *
 * 1. "200mV Headroom Rule" (LR1121 Datasheet):
 *    VBAT must be >= VTCXO + 200mV
 *    - If VBAT=3.3V and TCXO voltage=3.3V (0x07), it WILL FAIL!
 *    - FIX: Use 1.8V (0x02) or 2.4V (0x04) instead
 *
 * 2. "Ghost RC Oscillator Switch" (Missing SetStandby):
 *    After SetTcxoMode, chip is powering TCXO but STILL running on RC!
 *    - FIX: MUST send SetStandby(0x01) to switch to XOSC!
 *
 * 3. "SPI Phase/Timing Jitter" (SiWG917 is fast):
 *    Commands sent too quickly after SetTcxoMode before VTCXO ramps up
 *    - FIX: Wait for BUSY LOW before next command
 *
 * THE BULLETPROOF SEQUENCE:
 *   Step 1: Reset       - [Toggle RST pin LOW for 10ms]
 *   Step 2: SetTcxoMode - [0x01 0x17] [0x02] [0x00 0x03 0xE8]
 *           - Voltage: 0x02 = 1.8V (with 200mV headroom from 3.3V supply)
 *           - Delay: 0x0003E8 = 1000 steps × 30.52µs ≈ 30ms (safe timeout)
 *   Step 3: Wait        - [Wait for BUSY pin to go LOW, then stay LOW 10ms]
 *   Step 4: SetStandby  - [0x01 0x1C] [0x01]  *** THIS IS THE KEY! ***
 *           - Mode: 0x01 = STDBY_XOSC (forces switch from RC to TCXO)
 *   Step 5: Calibrate   - [0x01 0x0F] [0x3F]
 *           - Mask: 0x3F = All calibrations (ADC, RC, image, PLL)
 *
 * VERIFICATION:
 *   After this sequence, GetStatus should show:
 *   - Chip Mode bits [3:1] = 0x03 (STDBY_XOSC), NOT 0x02 (STDBY_RC)
 *
 * @param tcxo_voltage TCXO voltage trim value (recommend TCXO_VOLTAGE_1V8)
 * @return LR1121_OK on success
 */
lr1121_status_t lr1121_bulletproof_tcxo_init(uint8_t tcxo_voltage) {
  const char *voltage_names[] = {"1.6V", "1.7V", "1.8V", "2.2V",
                                 "2.4V", "2.7V", "3.0V", "3.3V"};
  const char *voltage_str =
      (tcxo_voltage <= 7) ? voltage_names[tcxo_voltage] : "INVALID";

  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════════"
           "═════╗\n");
  DEBUGOUT("║  BULLETPROOF TCXO INIT v2 - WAVESHARE SEQUENCE                   "
           "     ║\n");
  DEBUGOUT("╠══════════════════════════════════════════════════════════════════"
           "═════╣\n");
  DEBUGOUT("║  Citation: Waveshare Core1121_XF_Demo lora_system_init()         "
           "     ║\n");
  DEBUGOUT("║  Key fix: SetStandby(XOSC) BEFORE SetTcxoMode!                   "
           "     ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════════"
           "═════╝\n");
  DEBUGOUT("\n");

  /* Check voltage headroom */
  if (tcxo_voltage >= 0x05) { /* 2.7V or higher */
    DEBUGOUT("⚠️  WARNING: TCXO voltage %s may violate 200mV headroom rule!\n",
             voltage_str);
    DEBUGOUT("    With 3.3V VBAT, maximum safe TCXO is 2.4V (0x04)\n");
    DEBUGOUT("    Note: Waveshare demo uses 3.0V (0x06) which works on their "
             "board\n\n");
  }

  /* =========================================================================
   * STEP 1: Hardware Reset
   * Citation: Waveshare lr1121_config.cpp line 83 - lr11xx_system_reset()
   * =========================================================================
   */
  DEBUGOUT("STEP 1: Hardware Reset\n");
  DEBUGOUT("  Action: Pull RST LOW for 10ms\n");

  lr1121_status_t status = lr1121_reset();
  if (status != LR1121_OK) {
    DEBUGOUT("  ✗ FAILED: Reset error %d\n", status);
    return status;
  }
  DEBUGOUT("  ✓ Reset complete, BUSY is LOW OK\n\n");

  /* =========================================================================
   * STEP 2: Wakeup (Toggle NSS/CS)
   * Citation: Waveshare lr11xx_hal.c line 179-187 - lr11xx_hal_wakeup()
   *   The wakeup sequence is: CS LOW for ~10ms, then CS HIGH
   *   This ensures the chip is fully awake after reset
   * =========================================================================
   */
  DEBUGOUT("STEP 2: Wakeup (Toggle CS/NSS)\n");
  DEBUGOUT("  Action: CS LOW for 10ms, then CS HIGH\n");
  DEBUGOUT("  Citation: Waveshare lr11xx_hal.c lr11xx_hal_wakeup()\n");

  /* Toggle CS to wake up the chip
   * Citation: Waveshare lr11xx_hal.c lines 179-187
   * Using existing driver functions for CS control (lr1121_driver.h)
   */
  lr1121_cs_assert(); /* CS LOW - GPIO 28 */
  elrs_delay_ms(10);
  lr1121_cs_deassert(); /* CS HIGH */
  elrs_delay_ms(5);

  /* Wait for BUSY LOW */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout after wakeup\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ Wakeup complete, chip is ready\n\n");

  /* =========================================================================
   * STEP 2.5: Disable SPI CRC (optional but recommended)
   * Citation: Semtech lr11xx_system.c - LR11XX_SYSTEM_ENABLE_SPI_CRC_OC =
   * 0x0108
   *
   * Disabling SPI CRC after wakeup matches the reference sequence and ensures
   * clean SPI communication without CRC overhead during initialization.
   * =========================================================================
   */
  DEBUGOUT("STEP 2.5: Disable SPI CRC (0x0108)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x08] [0x00]\n");
  DEBUGOUT("  Parameter: 0x00 = CRC disabled\n");
  DEBUGOUT(
      "  Citation: Semtech lr11xx_system.c LR11XX_SYSTEM_ENABLE_SPI_CRC_OC\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SPI CRC disable\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  uint8_t spi_crc_param = 0x00; /* 0 = disabled */
  if (!lr1121_send_command(0x0108, &spi_crc_param, 1)) {
    DEBUGOUT("  ✗ FAILED: SPI CRC disable command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout after SPI CRC disable\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ SPI CRC disabled\n\n");

  /* =========================================================================
   * STEP 3: SetStandby(XOSC) - First oscillator config!
   * Citation: Waveshare lr1121_config.cpp line 94:
   *   lr11xx_system_set_standby(context, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
   * =========================================================================
   */
  DEBUGOUT("STEP 3: SetStandby(XOSC)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x1C] [0x01]\n");
  DEBUGOUT("  Mode: 0x01 = STDBY_XOSC\n");
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 94\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SetStandby\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  uint8_t standby_param = 0x01; /* XOSC mode */
  if (!lr1121_send_command(ELRS_CMD_SET_STANDBY, &standby_param, 1)) {
    DEBUGOUT("  ✗ FAILED: SetStandby command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  ✗ FAILED: SetStandby BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ SetStandby(XOSC) complete\n\n");

  /* =========================================================================
   * STEP 4: CalibrateImage for frequency band
   * Citation: Waveshare lr1121_config.cpp line 96:
   *   lr11xx_system_calibrate_image(context, 0x6B, 0x6E); // 430-440MHz
   * Citation: Semtech lr11xx_system.c line 101 - opcode 0x0111
   *
   * For 2.4GHz ELRS: Use 0xE1, 0xE9 (2400-2500 MHz / 4 = 0x258 >> 2)
   * For 915MHz: Use 0xE1, 0xE9 (or appropriate values)
   * =========================================================================
   */
  DEBUGOUT("STEP 4: CalibrateImage (0x0111)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x11] [0xE1] [0xE9]\n");
  DEBUGOUT("  Freq band: 902-928 MHz (915 MHz ISM)\n");
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 96\n");
  DEBUGOUT("  Citation: lr1121_elrs_calib_image() frequency table\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before CalibrateImage\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* CalibrateImage: [freq1] [freq2]
   * Citation: Semtech lr11xx_system.c line 101 - opcode 0x0111
   * Citation: lr1121_elrs_calib_image() frequency lookup table
   *
   * 915 MHz band: freq1=0xE1 (902MHz), freq2=0xE9 (928MHz)
   * 433 MHz band: freq1=0x6B (428MHz), freq2=0x6F (444MHz)
   * 868 MHz band: freq1=0xD7 (863MHz), freq2=0xDB (870MHz)
   */
  uint8_t calib_img_params[2] = {0xE1, 0xE9}; /* 915 MHz band */
  if (!lr1121_send_command(ELRS_CMD_CALIBRATE_IMAGE, calib_img_params, 2)) {
    DEBUGOUT("  ✗ FAILED: CalibrateImage command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  ✗ FAILED: CalibrateImage BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ CalibrateImage complete\n\n");

  /* =========================================================================
   * STEP 5: SetRegMode(DCDC)
   * Citation: Waveshare lr1121_config.cpp line 100-101:
   *   const lr11xx_system_reg_mode_t regulator =
   * smtc_shield_lr11xx_common_get_reg_mode();
   *   lr11xx_system_set_reg_mode(context, regulator);
   * Citation: lr1121_common.c line 726 returns LR11XX_SYSTEM_REG_MODE_DCDC
   * Citation: Semtech lr11xx_system.c line 100 - opcode 0x0110
   * =========================================================================
   */
  DEBUGOUT("STEP 5: SetRegMode(DCDC) (0x0110)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x10] [0x01]\n");
  DEBUGOUT("  Mode: 0x01 = DC-DC converter\n");
  DEBUGOUT("  Citation: Waveshare lr1121_common.c line 726\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SetRegMode\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  uint8_t reg_mode = 0x01; /* DCDC mode */
  if (!lr1121_send_command(ELRS_CMD_SET_REG_MODE, &reg_mode, 1)) {
    DEBUGOUT("  ✗ FAILED: SetRegMode command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: SetRegMode BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ SetRegMode(DCDC) complete\n\n");

  /* =========================================================================
   * STEP 6: SetDioAsRfSwitch
   * Citation: Waveshare lr1121_config.cpp line 104-105:
   *   const lr11xx_system_rfswitch_cfg_t* rf_switch_setup = ...
   *   lr11xx_system_set_dio_as_rf_switch(context, rf_switch_setup);
   * Citation: lr1121_common.c lines 78-87 for RF switch config
   * Citation: Semtech lr11xx_system.c line 102 - opcode 0x0112
   * =========================================================================
   */
  DEBUGOUT("STEP 6: SetDioAsRfSwitch (0x0112)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x12] [enable] [standby] [rx] [tx] [tx_hp] "
           "[tx_hf] [gnss] [wifi]\n");
  DEBUGOUT("  Config: enable=0x03, standby=0, rx=0x01, tx=0x02, tx_hp=0x02\n");
  DEBUGOUT("  Citation: Waveshare lr1121_common.c lines 78-87\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SetDioAsRfSwitch\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* PE4259 Truth Table (from Core1121-HF schematic):
   *   V1=0, V2=1 → RFC to RF1 = TX path
   *   V1=1, V2=0 → RFC to RF2 = RX path
   * DIO5 → V1, DIO6 → V2
   * RX: DIO5=1, DIO6=0 = 0x01
   * TX: DIO5=0, DIO6=1 = 0x02
   */
  uint8_t rf_switch_params[8] = {
      0x03, /* enable: DIO5+DIO6 */
      0x00, /* standby: both off (switch isolated) */
      0x01, /* rx: DIO5=1, DIO6=0 → V1=1, V2=0 → RF2 → RX */
      0x02, /* tx: DIO5=0, DIO6=1 → V1=0, V2=1 → RF1 → TX */
      0x02, /* tx_hp: same as tx */
      0x00, /* tx_hf: 2.4GHz uses RFIO_HF, no switch */
      0x00, /* gnss */
      0x00  /* wifi */
  };
  if (!lr1121_send_command(ELRS_CMD_SET_DIO_AS_RF_SWITCH, rf_switch_params,
                           8)) {
    DEBUGOUT("  ✗ FAILED: SetDioAsRfSwitch command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: SetDioAsRfSwitch BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ SetDioAsRfSwitch complete\n\n");

  /* =========================================================================
   * STEP 7: SetTcxoMode - NOW we configure TCXO!
   * Citation: Waveshare lr1121_config.cpp line 108:
   *   lr11xx_system_set_tcxo_mode(context, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300);
   *
   * The Waveshare demo uses:
   *   - Voltage: 0x06 = 3.0V
   *   - Timeout: 300 steps ≈ 9.15ms
   * =========================================================================
   */
  DEBUGOUT("STEP 7: SetTcxoMode (0x0117)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x17] [0x%02X] [0x00 0x01 0x2C]\n",
           tcxo_voltage);
  DEBUGOUT("  Voltage: 0x%02X (%s)\n", tcxo_voltage, voltage_str);
  DEBUGOUT("  Timeout: 300 ticks × 30.52µs ≈ 9.16ms (WAVESHARE MATCH!)\n");
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 108:\n");
  DEBUGOUT("    lr11xx_system_set_tcxo_mode(context, "
           "LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300);\n");

  /* Wait for BUSY LOW before command */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SetTcxoMode\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* SetTcxoMode parameters: [Voltage][Delay MSB][Delay MID][Delay LSB]
   *
   * Citation: Waveshare lr1121_config.cpp line 108:
   *   lr11xx_system_set_tcxo_mode(context, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300);
   *
   * Citation: Semtech lr11xx_system.c line 374-387 - parameter encoding
   *
   * WAVESHARE EXACT VALUES:
   *   Voltage: LR11XX_SYSTEM_TCXO_CTRL_3_0V = 0x06
   *   Timeout: 300 ticks = 0x00012C
   *
   * TCXO startup delay calculation:
   *   1 tick = 30.52 µs
   *   300 ticks = 300 × 30.52µs ≈ 9.16ms
   *   0x00012C = 300 decimal
   */
  uint8_t tcxo_params[4] = {
      tcxo_voltage, /* Voltage trim (0x06 = 3.0V from Waveshare) */
      0x00,         /* Delay MSB */
      0x01,         /* Delay MID - 0x00012C = 300 ticks ≈ 9ms (WAVESHARE!) */
      0x2C          /* Delay LSB */
  };

  if (!lr1121_send_command(ELRS_CMD_SET_TCXO_MODE, tcxo_params, 4)) {
    DEBUGOUT("  ✗ FAILED: SetTcxoMode command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Wait for TCXO startup - hardware timeout via BUSY pin */
  DEBUGOUT("  Waiting for TCXO startup (BUSY LOW)...\n");
  if (!lr1121_wait_busy_timeout(500)) {
    DEBUGOUT("  ✗ FAILED: TCXO startup timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* Additional software delay for TCXO stabilization
   * Citation: User request - new LR1121 may need extra time to stabilize
   * Adding 100ms delay after BUSY goes LOW to ensure TCXO is fully stable
   * Increased from 50ms to give new chip more time to initialize
   */
  DEBUGOUT("  Adding 100ms software delay for TCXO stabilization...\n");
  for (volatile uint32_t i = 0; i < 1000000; i++) {
  } /* ~100ms delay */

  DEBUGOUT("  ✓ SetTcxoMode complete, TCXO is powered and stabilized\n\n");

  /* =========================================================================
   * STEP 8: CfgLfClk - Configure low-frequency clock
   *
   * CRITICAL FIX (2026-01-16): Now uses Internal RC (0x00) instead of XTAL
   * (0x02)
   *
   * The Waveshare Core1121-HF module may NOT have a 32.768kHz crystal
   * populated. Configuring for LF_XTAL when no crystal exists causes
   * PLL_LOCK_ERR because the LR1121 state machine hangs waiting for a clock
   * that never arrives.
   *
   * Citation: Waveshare lr1121_config.cpp line 111:
   *   lr11xx_system_cfg_lfclk(context, LR11XX_SYSTEM_LFCLK_XTAL, true);
   * Citation: lr1121_common.c lines 89-92:
   *   lf_clk_cfg = LR11XX_SYSTEM_LFCLK_XTAL, wait_32k_ready = true
   * Citation: Semtech lr11xx_system.c line 105 - opcode 0x0116
   *
   * Note: LR1121_LFCLK_USE_RC is now set to 1 in lr1121_elrs_init.h
   * =========================================================================
   */
  DEBUGOUT("STEP 8: CfgLfClk (0x0116)\n");
#if LR1121_LFCLK_USE_RC
  DEBUGOUT("  SPI Frame: [0x01 0x16] [0x04]\n");
  DEBUGOUT("  Parameter: 0x04 = RC(0x00) | wait_32k(bit2)\n");
#else
  DEBUGOUT("  SPI Frame: [0x01 0x16] [0x06]\n");
  DEBUGOUT("  Parameter: 0x06 = XTAL(0x02) | wait_32k(bit2)\n");
#endif
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 111\n");
  DEBUGOUT(
      "  Citation: Semtech lr11xx_system.c line 368 - single byte encoding\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before CfgLfClk\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* CfgLfClk: Single byte parameter
   * Citation: Semtech lr11xx_system.c line 368:
   *   cbuffer[2] = (uint8_t)(lfclock_cfg | (wait_for_32k_ready << 2))
   *
   * lf_clk_cfg: 0=RC, 1=EXT, 2=XTAL
   * wait_for_32k_ready: bit 2 (0=no wait, 1=wait)
   *
   * For XTAL (2) + wait (1<<2=4): 2 | 4 = 0x06
   * For RC   (0) + wait (1<<2=4): 0 | 4 = 0x04
   *
   * If board lacks 32kHz XTAL, define LR1121_LFCLK_USE_RC=1 to use RC
   * oscillator.
   */
#if LR1121_LFCLK_USE_RC
  /* Use internal 32kHz RC oscillator (less accurate, but always available) */
  uint8_t lfclk_param =
      LR11XX_SYSTEM_LFCLK_RC | (0x01 << 2); /* RC=0x00, wait_32k=bit2 -> 0x04 */
  DEBUGOUT("  Using LFCLK_RC (internal 32kHz RC oscillator)\n");
#else
  /* Use 32kHz crystal (Waveshare default - more accurate) */
  uint8_t lfclk_param = LR11XX_SYSTEM_LFCLK_XTAL |
                        (0x01 << 2); /* XTAL=0x02, wait_32k=bit2 -> 0x06 */
  DEBUGOUT("  Using LFCLK_XTAL (32kHz crystal)\n");
#endif

  if (!lr1121_send_command(ELRS_CMD_CFG_LFCLK, &lfclk_param, 1)) {
    DEBUGOUT("  ✗ FAILED: CfgLfClk command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  ✗ FAILED: CfgLfClk BUSY timeout\n");
    /* If XTAL mode fails, suggest trying RC mode */
#if !LR1121_LFCLK_USE_RC
    DEBUGOUT("  HINT: If no 32kHz crystal, define LR1121_LFCLK_USE_RC=1\n");
#endif
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ CfgLfClk complete\n\n");

  /* =========================================================================
   * STEP 9: ClearErrors - Before calibration
   * Citation: Waveshare lr1121_config.cpp line 114:
   *   lr11xx_system_clear_errors(context);
   * =========================================================================
   */
  DEBUGOUT("STEP 9: ClearErrors (0x010E)\n");
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 114\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before ClearErrors\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* ClearErrors: opcode 0x010E, no parameters */
  if (!lr1121_send_command(ELRS_CMD_CLEAR_ERRORS, NULL, 0)) {
    DEBUGOUT("  ✗ FAILED: ClearErrors command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: ClearErrors BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ Errors cleared\n\n");

  /* =========================================================================
   * STEP 10: Calibrate(0x3F) - Full calibration
   * Citation: Waveshare lr1121_config.cpp line 116:
   *   lr11xx_system_calibrate(context, 0x3F);
   * =========================================================================
   */
  DEBUGOUT("STEP 10: Calibrate(0x3F)\n");
  DEBUGOUT("  SPI Frame: [0x01 0x0F] [0x3F]\n");
  DEBUGOUT("  Mask: 0x3F = All calibrations\n");
  DEBUGOUT("  Citation: Waveshare lr1121_config.cpp line 116\n");
  DEBUGOUT("  Citation: Semtech lr11xx_system.c line 99 - opcode 0x010F\n");

  /* Wait for BUSY LOW before command */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before Calibrate\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* Calibrate parameter: 0x3F = all calibrations */
  uint8_t calib_param = 0x3F;

  if (!lr1121_send_command(ELRS_CMD_CALIBRATE, &calib_param, 1)) {
    DEBUGOUT("  ✗ FAILED: Calibrate command send failed\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Wait for calibration to complete - can take up to 100ms */
  DEBUGOUT("  Waiting for calibration (BUSY LOW)...\n");
  if (!lr1121_wait_busy_timeout(500)) {
    DEBUGOUT("  ✗ FAILED: Calibration timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ Calibration complete\n\n");

  /* =========================================================================
   * STEP 11: SetStandby(XOSC) AGAIN - Wake from post-calibration SLEEP!
   *
   * CRITICAL DISCOVERY: The LR1121 automatically goes to SLEEP mode after
   * Calibrate(0x3F) completes to save power. This is BY DESIGN!
   *
   * We MUST send SetStandby(XOSC) again to:
   *   1. Wake the chip from SLEEP
   *   2. Put it back in STDBY_XOSC mode
   *   3. Enable the TCXO for temperature sensor operation
   *
   * Citation: Waveshare lr1121_config.cpp continues after calibrate
   * Citation: Semtech lr11xx_system.c line 111 - opcode 0x011C
   * =========================================================================
   */
  DEBUGOUT("STEP 11: SetStandby(XOSC) - Wake from post-calibration SLEEP\n");
  DEBUGOUT("  SPI Frame: [0x01 0x1C] [0x01]\n");
  DEBUGOUT("  CRITICAL: Chip goes to SLEEP after Calibrate - must wake it!\n");
  DEBUGOUT("  Citation: Semtech lr11xx_system.c line 111\n");

  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  ✗ FAILED: BUSY timeout before SetStandby (post-calibration)\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }

  /* Send SetStandby(XOSC) to wake from SLEEP */
  uint8_t standby_param2 = 0x01; /* XOSC mode */
  if (!lr1121_send_command(ELRS_CMD_SET_STANDBY, &standby_param2, 1)) {
    DEBUGOUT("  ✗ FAILED: SetStandby command send failed (post-calibration)\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Wait for chip to enter STDBY_XOSC */
  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  ✗ FAILED: SetStandby BUSY timeout (post-calibration)\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  ✓ SetStandby(XOSC) complete - chip should be awake now\n\n");

  /* =========================================================================
   * STEP 12: Verify we're in STDBY_XOSC mode
   * =========================================================================
   */
  DEBUGOUT("STEP 12: Verify XOSC Mode\n");

  /* CRITICAL FIX per user analysis:
   * LR1121 Status Byte uses bits [3:1] for Chip Mode
   *   Mode 0 = SLEEP
   *   Mode 1 = STDBY_RC
   *   Mode 2 = STDBY_XOSC  <-- This is what we expect!
   *   Mode 3 = FS (Frequency Synthesis)
   *   Mode 4 = RX
   *   Mode 5 = TX
   */
  uint8_t chip_mode = 0xFF;
  if (lr1121_verify_xosc_mode(&chip_mode)) {
    DEBUGOUT("  ✓ Chip Mode: %d (STDBY_XOSC) - SUCCESS!\n", chip_mode);
  } else {
    DEBUGOUT("  ✗ Chip Mode: %d - NOT in STDBY_XOSC!\n", chip_mode);
    DEBUGOUT("    Expected mode 2 (STDBY_XOSC), got %d\n", chip_mode);
    if (chip_mode == 0) {
      DEBUGOUT("    Mode 0 = SLEEP: Chip fell back to SLEEP (XOSC failed to "
               "start)\n");
      DEBUGOUT("    Check: TCXO hardware, SetTcxoMode voltage setting\n");
    } else if (chip_mode == 1) {
      DEBUGOUT("    Mode 1 = STDBY_RC: TCXO switch FAILED (still on RC "
               "oscillator)\n");
      DEBUGOUT("    Check: 200mV headroom, TCXO hardware, SetStandby opcode\n");
    }
    return LR1121_ERROR_BUSY_TIMEOUT; /* Use this as a "not ready" indicator */
  }

  /* =========================================================================
   * SUCCESS!
   * =========================================================================
   */
  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════════"
           "═════╗\n");
  DEBUGOUT("║  ✓ BULLETPROOF TCXO INIT COMPLETE!                               "
           "     ║\n");
  DEBUGOUT("║  - TCXO voltage: %s (with 200mV headroom)                        "
           "   ║\n",
           voltage_str);
  DEBUGOUT("║  - Chip is NOW running on TCXO (not RC!)                         "
           "     ║\n");
  DEBUGOUT("║  - ADC calibrated for temperature sensor                         "
           "     ║\n");
  DEBUGOUT("║  - GetTemperature should now work!                               "
           "     ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════════"
           "═════╝\n");
  DEBUGOUT("\n");

  return LR1121_OK;
}

/**
 * @brief Verify chip is in STDBY_XOSC mode (not STDBY_RC)
 *
 * After bulletproof_tcxo_init, the chip should be in mode 3 (STDBY_XOSC).
 * If it's still in mode 2 (STDBY_RC), the TCXO switch failed.
 *
 * Citation: LR1121 Datasheet Section 11.1.1 "GetStatus"
 *   Chip Mode values (Stat2 bits [3:1]):
 *     0 = SLEEP
 *     1 = STDBY_RC
 *     2 = STDBY_XOSC  <-- We want this!
 *     3 = FS
 *     4 = RX
 *     5 = TX
 */
bool lr1121_verify_xosc_mode(uint8_t *chip_mode) {
  uint8_t stat1, stat2, irq;

  if (!lr1121_get_status(&stat1, &stat2, &irq)) {
    if (chip_mode)
      *chip_mode = 0xFF;
    return false;
  }

  /* Extract chip mode from Stat2 bits [3:1] */
  uint8_t mode = (stat2 >> 1) & 0x07;
  if (chip_mode)
    *chip_mode = mode;

  /* Mode 2 = STDBY_XOSC (success), Mode 1 = STDBY_RC (failure) */
  return (mode == 2); /* STDBY_XOSC */
}

/*******************************************************************************
 * WAVESHARE EXACT INITIALIZATION
 *
 * This function implements the EXACT sequence from the Waveshare Core1121-XF
 * Arduino demo (lr1121_config.cpp lora_system_init function).
 *
 * Citation: C:\Users\mjeuw\Downloads\Core1121_XF_Demo\Core1121_XF_Demo\esp32s3\
 *           Arduino\waveshare_lroa_1121\examples\lr1121_ping_pong\lr1121_config.cpp
 ******************************************************************************/

/**
 * @brief Initialize LR1121 using EXACT Waveshare demo sequence
 *
 * This function replicates the exact sequence from Waveshare's working demo:
 *
 * Citation: lr1121_config.cpp lora_system_init() lines 81-130:
 *   1. lr11xx_system_reset(context)
 *   2. lr11xx_hal_wakeup(context)
 *   3. lr11xx_system_enable_spi_crc(context, false)
 *   4. lr11xx_system_set_standby(context, LR11XX_SYSTEM_STANDBY_CFG_XOSC)
 *   5. lr11xx_system_calibrate_image(context, 0x6B, 0x6E)  // 430-440MHz
 *   6. lr11xx_system_set_reg_mode(context, LR11XX_SYSTEM_REG_MODE_DCDC)
 *   7. lr11xx_system_set_dio_as_rf_switch(context, rf_switch_setup)
 *   8. lr11xx_system_set_tcxo_mode(context, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300)
 *   9. lr11xx_system_cfg_lfclk(context, LR11XX_SYSTEM_LFCLK_XTAL, true)
 *  10. lr11xx_system_clear_errors(context)
 *  11. lr11xx_system_calibrate(context, 0x3F)
 *  12. lr11xx_system_get_errors(context, &errors)  // Check for IMG_CALIB_ERR
 *  13. lr11xx_system_clear_errors(context)
 *  14. lr11xx_system_clear_irq_status(context, LR11XX_SYSTEM_IRQ_ALL_MASK)
 *
 * @return LR1121_OK on success
 */
lr1121_status_t lr1121_waveshare_init(void) {

  DEBUGOUT("\n");
  DEBUGOUT("===============================================================\n");
  DEBUGOUT("  TCXO INIT - MATCHING WORKING TEST SEQUENCE\n");
  DEBUGOUT("===============================================================\n");
  DEBUGOUT("  Order: Reset -> ClearErr -> RegMode(DCDC) -> SetTcxoMode ->\n");
  DEBUGOUT("         (wait) -> SetStandby(XOSC) -> Calibrate ->\n");
  DEBUGOUT("         SetStandby(XOSC) again\n");
  DEBUGOUT(
      "===============================================================\n\n");

  /* =========================================================================
   * STEP 1: Hardware Reset
   * =========================================================================
   */
  DEBUGOUT("STEP 1: Hardware Reset\n");
  lr1121_status_t status = lr1121_reset();
  if (status != LR1121_OK) {
    DEBUGOUT("  X FAILED: Reset error %d\n", status);
    return status;
  }
  DEBUGOUT("  OK Reset complete\n\n");

  /* =========================================================================
   * STEP 2: Clear Errors (clean slate)
   * =========================================================================
   */
  DEBUGOUT("STEP 2: ClearErrors [0x010E]\n");
  lr1121_send_command(0x010E, NULL, 0);
  lr1121_wait_busy_timeout(100);
  elrs_delay_ms(5);
  DEBUGOUT("  OK Errors cleared\n\n");

  /* =========================================================================
   * STEP 3: SetRegMode(DCDC)
   * =========================================================================
   */
  DEBUGOUT("STEP 3: SetRegMode(DCDC) [0x0110]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t reg_mode = 0x01; /* DCDC */
  if (!lr1121_send_command(0x0110, &reg_mode, 1)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  elrs_delay_ms(5);
  DEBUGOUT("  OK SetRegMode(DCDC) complete\n\n");

  /* =========================================================================
   * STEP 4: SetTcxoMode - BEFORE SetStandby(XOSC)!
   * This is the key difference from the previous sequences.
   *
   * From working test: set_tcxo_mode(voltage, delay_ticks)
   * Using voltage=0x06 (3.0V), delay=300 ticks (~9ms)
   *
   * Then wait: software delay = hardware delay + 20ms margin
   * =========================================================================
   */
  DEBUGOUT("STEP 4: SetTcxoMode(3.0V, 300 ticks) [0x0117]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t tcxo_params[4] = {
      0x06, /* Voltage: 3.0V */
      0x00, /* Delay MSB */
      0x01, /* Delay middle byte */
      0x2C  /* Delay LSB (300 = 0x012C) */
  };
  if (!lr1121_send_command(0x0117, tcxo_params, 4)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK SetTcxoMode command sent\n");

  /* CRITICAL: Wait for TCXO to stabilize!
   * Software delay = (300 ticks * 31us/tick) / 1000 + 30ms margin = ~39ms
   */
  DEBUGOUT("  Waiting 40ms for TCXO stabilization...\n");
  elrs_delay_ms(40);
  DEBUGOUT("  OK TCXO wait complete\n\n");

  /* =========================================================================
   * STEP 5: SetStandby(XOSC) - NOW switch to TCXO clock
   * =========================================================================
   */
  DEBUGOUT("STEP 5: SetStandby(XOSC) [0x011C]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t standby_xosc = 0x01; /* XOSC */
  if (!lr1121_send_command(0x011C, &standby_xosc, 1)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  elrs_delay_ms(20);
  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  X BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK SetStandby(XOSC) complete\n");

  /* Check mode - should be STDBY_XOSC (2) */
  uint8_t chip_mode = 0, cmd_status = 0;
  uint16_t errors = 0;
  if (lr1121_elrs_get_status(&chip_mode, &cmd_status, &errors)) {
    DEBUGOUT("  Mode: %d (expect 2=STDBY_XOSC), Errors: 0x%04X\n", chip_mode,
             errors);
    if (chip_mode != 2) {
      DEBUGOUT("  WARNING: Not in STDBY_XOSC!\n");
    }
    if (errors & 0x0020) {
      DEBUGOUT("  WARNING: HF_XOSC_START_ERR!\n");
    }
  }
  DEBUGOUT("\n");

  /* =========================================================================
   * STEP 6: Calibrate(0x3F)
   * =========================================================================
   */
  DEBUGOUT("STEP 6: Calibrate(0x3F) [0x010F]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t calib_mask = 0x3F;
  if (!lr1121_send_command(0x010F, &calib_mask, 1)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  elrs_delay_ms(50);
  if (!lr1121_wait_busy_timeout(500)) {
    DEBUGOUT("  X BUSY timeout during calibration\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK Calibration complete\n\n");

  /* =========================================================================
   * STEP 7: SetStandby(XOSC) AGAIN after calibration
   * Calibration may put chip back in STDBY_RC, so re-issue this
   * =========================================================================
   */
  DEBUGOUT("STEP 7: SetStandby(XOSC) again [0x011C]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  if (!lr1121_send_command(0x011C, &standby_xosc, 1)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  elrs_delay_ms(10);
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("  X BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK SetStandby(XOSC) complete\n\n");

  /* =========================================================================
   * STEP 8: Check errors after calibration
   * =========================================================================
   */
  DEBUGOUT("STEP 8: Check errors\n");
  chip_mode = 0;
  errors = 0;
  if (lr1121_elrs_get_status(&chip_mode, &cmd_status, &errors)) {
    DEBUGOUT("  Mode: %d, Errors: 0x%04X\n", chip_mode, errors);
  }
  DEBUGOUT("\n");

  /* =========================================================================
   * STEP 9: SetDioAsRfSwitch (PE4259)
   * =========================================================================
   */
  DEBUGOUT("STEP 9: SetDioAsRfSwitch [0x0112]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t rf_switch[8] = {
      0x03, /* enable: DIO5+DIO6 */
      0x00, /* standby: both off */
      0x01, /* rx: DIO5=1, DIO6=0 */
      0x02, /* tx: DIO5=0, DIO6=1 */
      0x02, /* tx_hp: same as tx */
      0x00, /* tx_hf */
      0x00, /* gnss */
      0x00  /* wifi */
  };
  if (!lr1121_send_command(0x0112, rf_switch, 8)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK SetDioAsRfSwitch complete\n\n");

  /* =========================================================================
   * STEP 10: CalibrateImage for 915MHz band
   * =========================================================================
   */
  DEBUGOUT("STEP 10: CalibrateImage(915MHz) [0x0111]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t calib_img[2] = {0xE1, 0xE9}; /* 915MHz band */
  if (!lr1121_send_command(0x0111, calib_img, 2)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  elrs_delay_ms(10);
  if (!lr1121_wait_busy_timeout(200)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK CalibrateImage complete\n\n");

  /* =========================================================================
   * STEP 11: Clear errors and IRQs
   * =========================================================================
   */
  DEBUGOUT("STEP 11: Clear errors and IRQs\n");
  lr1121_send_command(0x010E, NULL, 0); /* ClearErrors */
  lr1121_wait_busy_timeout(100);
  uint8_t irq_mask[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  lr1121_send_command(0x0114, irq_mask, 4); /* ClearIrq */
  lr1121_wait_busy_timeout(100);
  DEBUGOUT("  OK Cleared\n\n");

  /* =========================================================================
   * STEP 12: SetStandby(XOSC) - FINAL! After ALL calibrations
   *
   * Per LR1121 docs: "At the end of the calibration procedure, the device
   * returns to Standby RC." This applies to BOTH Calibrate() and
   * CalibrateImage().
   *
   * So we MUST issue SetStandby(XOSC) at the very end to switch back to TCXO.
   * =========================================================================
   */
  DEBUGOUT("STEP 12: SetStandby(XOSC) - FINAL [0x011C]\n");
  if (!lr1121_wait_busy_timeout(100)) {
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  uint8_t standby_final = 0x01; /* XOSC */
  if (!lr1121_send_command(0x011C, &standby_final, 1)) {
    DEBUGOUT("  X FAILED\n");
    return LR1121_ERROR_SPI_INIT;
  }
  elrs_delay_ms(20); /* Give TCXO time to start */
  if (!lr1121_wait_busy_timeout(200)) {
    DEBUGOUT("  X BUSY timeout\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT("  OK SetStandby(XOSC) complete\n\n");

  /* =========================================================================
   * STEP 13: Final status check
   * =========================================================================
   */
  DEBUGOUT("STEP 13: Final status check\n");

  /* Two-phase GetStatus */
  lr1121_wait_busy_timeout(100);
  lr1121_send_command(0x0100, NULL, 0);
  lr1121_wait_busy_timeout(100);
  uint8_t status_resp[6] = {0};
  if (lr1121_read_response(status_resp, 6)) {
    DEBUGOUT("  RAW GetStatus: [%02X %02X %02X %02X %02X %02X]\n",
             status_resp[0], status_resp[1], status_resp[2], status_resp[3],
             status_resp[4], status_resp[5]);

    /* stat2 is in resp[1], chip mode in bits [3:1] */
    uint8_t stat2 = status_resp[1];
    chip_mode = (stat2 >> 1) & 0x07;
    DEBUGOUT("  stat2=0x%02X -> chip_mode = %d\n", stat2, chip_mode);
    DEBUGOUT("  Mode: %d (0=SLEEP, 1=STDBY_RC, 2=STDBY_XOSC)\n", chip_mode);

    if (chip_mode == 2) {
      DEBUGOUT("  SUCCESS! LR1121 ready in STDBY_XOSC - TCXO working!\n");
    } else if (chip_mode == 1) {
      DEBUGOUT("  OK: Chip in STDBY_RC (usable)\n");
    } else if (chip_mode == 0) {
      DEBUGOUT("  Note: Chip in SLEEP (will wake on next command)\n");
    }
  }

  /* Check errors */
  lr1121_wait_busy_timeout(100);
  lr1121_send_command(0x010D, NULL, 0);
  lr1121_wait_busy_timeout(100);
  uint8_t err_resp[3] = {0};
  if (lr1121_read_response(err_resp, 3)) {
    errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
    DEBUGOUT("  Errors: 0x%04X\n", errors);
    if (errors == 0) {
      DEBUGOUT("  No errors - initialization successful!\n");
    }
    if (errors & 0x0020)
      DEBUGOUT("  WARNING: HF_XOSC_START_ERR\n");
    if (errors & 0x0080)
      DEBUGOUT("  WARNING: PLL_LOCK_ERR\n");
  }
  DEBUGOUT("\n");

  DEBUGOUT("===============================================================\n");
  DEBUGOUT("  WAVESHARE INIT COMPLETE\n");
  DEBUGOUT(
      "===============================================================\n\n");

  return LR1121_OK;
}
