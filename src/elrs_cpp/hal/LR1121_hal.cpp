/**
 * @file LR1121_hal.cpp
 * @brief SiW917-specific LR1121 HAL implementation
 *
 * This replaces the upstream LR1121_hal.cpp with SiW917-specific setup code
 * and the proven two-phase GSPI command/response path from lr1121_driver.c.
 *
 * IMPORTANT: Core1121/Waveshare module requires TCXO initialization that
 * standard ESP32 ELRS targets don't need. This is handled in
 * lr1121_waveshare_init().
 *
 * Key functions from C driver:
 * - lr1121_send_command() - handles command writes
 * - lr1121_read_response() - handles command responses
 * - lr1121_wait_busy_timeout() - handles BUSY pin polling
 * - lr1121_dio1_*() - handles DIO1 interrupt
 */

#include "../lib/LR1121Driver/LR1121_hal.h"
#include "../lib/LR1121Driver/LR1121_Regs.h"
#include "Arduino.h"
#include "logging.h"
#include "targets.h"

#include <string.h>

// Include our proven C driver implementation
extern "C" {
#include "hw_timer.h"
#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
}

volatile uint32_t isr_1_pending_count = 0;
volatile uint32_t isr_1_total_count = 0;
volatile uint32_t isr_2_pending_count = 0;
volatile bool isr_1_pending = false;
volatile bool isr_2_pending = false;
volatile uint32_t busy_timeout_count = 0;
static volatile uint16_t last_command_opcode = 0;
static volatile bool rx_continuous_active = false;
static volatile bool pending_rx_retune = false;

// Diagnostic toggle: keep the ELRS LR1121 fused SetFreq+Rx helper enabled by
// default. Disabling it made phase error much worse on SiW917.
#define ELRS_DIAG_DISABLE_FUSED_RX_RETUNE 0

extern LR1121Driver Radio;

// Static instance pointer
LR1121Hal *LR1121Hal::instance = nullptr;

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------

LR1121Hal::LR1121Hal() {
  instance = this;
  IsrCallback_1 = nullptr;
  IsrCallback_2 = nullptr;
}

//-----------------------------------------------------------------------------
// Initialization
// Note: Core1121/Waveshare requires TCXO setup not needed on standard ESP32
// targets
//-----------------------------------------------------------------------------

void LR1121Hal::init() {
  instance = this;
  DBGLN("Hal Init");

  // Initialize the underlying C driver (GPIO, SPI, etc.)
  lr1121_status_t status = lr1121_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 C driver init failed: %d", (int)status);
    return;
  }

  // Core1121/Waveshare TCXO initialization sequence:
  //   1. Hardware reset
  //   2. Wakeup (CS toggle)
  //   3. Disable SPI CRC
  //   4. SetStandby(XOSC) - External TCXO is already running
  //   5. CalibrateImage (915MHz band)
  //   6. SetRegMode(DCDC)
  //   7. SetDioAsRfSwitch
  //   8. SetTcxoMode(3.0V, 300 ticks)
  //   9. CfgLfClk
  //  10. Calibrate(0x3F)
  //  11. ClearErrors
  //  12. ClearIrq
  status = lr1121_waveshare_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 TCXO init failed: %d", (int)status);
    return;
  }

  // Configure LR1121 DIO9 interrupt (SiW917 GPIO_46)
  // NOTE: Function named "dio1" for ELRS legacy compatibility, but this is
  // LR1121 DIO9! LR1121 DIO1 is NSS (chip select), DIO9 is the IRQ line.
  // Matches ESP32: attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1),
  // dioISR_1, RISING)
  lr1121_dio1_init();
  lr1121_dio1_set_callback(dioISR_1);
  lr1121_dio1_enable();

  DBGLN("LR1121Hal initialized");
}

void LR1121Hal::end() {
  DBGLN("LR1121Hal::end()");

  // Disable interrupts
  lr1121_dio1_disable();
  IsrCallback_1 = nullptr;
  IsrCallback_2 = nullptr;

  // Deinitialize the C driver
  lr1121_deinit();
}

//-----------------------------------------------------------------------------
// Reset
//-----------------------------------------------------------------------------

void LR1121Hal::reset(bool bootloader) {
  DBGLN("LR1121Hal::reset(bootloader=%d)", bootloader);
  (void)bootloader; // Not used - no bootloader mode support

  // Perform hardware reset AND full TCXO init via C driver
  // CRITICAL: A raw lr1121_reset() kills the TCXO. We MUST re-run the full
  // waveshare TCXO init sequence every time the chip is hardware reset!
  lr1121_status_t status = lr1121_waveshare_init();
  if (status != LR1121_OK) {
    DBGLN("LR1121 reset/init failed: %d", (int)status);
  }
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode only)
// Use the same two-phase C-driver command path as the standalone RX test.
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode,
                             SX12XX_Radio_Number_t radioNumber) {
  last_command_opcode = opcode;

  if (opcode == LR11XX_RADIO_GET_PACKET) {
    // SiW917 drains this ELRS firmware helper from ReadCommand() using the
    // port-layer soft response path. Do not send it twice.
    return;
  }

  if (pending_rx_retune && opcode != LR11XX_RADIO_SET_RF_FREQUENCY_OC &&
      opcode != LR11XX_SYSTEM_SET_STANDBY_OC) {
    pending_rx_retune = false;
  }

  if (!WaitOnBusy(radioNumber)) {
    busy_timeout_count++;
    DBGLN("WriteCommand BUSY timeout (opcode=0x%04X)", opcode);
    return;
  }

  if (!lr1121_send_command(opcode, nullptr, 0)) {
    DBGLN("WriteCommand failed (opcode=0x%04X)", opcode);
  }
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode + data)
// Use the same two-phase C-driver command path as the standalone RX test.
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size,
                             SX12XX_Radio_Number_t radioNumber) {
  last_command_opcode = opcode;

  if (opcode == LR11XX_RADIO_GET_PACKET) {
    // See opcode-only overload: ReadCommand() owns this SiW917 hot-path command.
    return;
  }

  if (!ELRS_DIAG_DISABLE_FUSED_RX_RETUNE &&
      opcode == LR11XX_RADIO_SET_RF_FREQUENCY_OC && pending_rx_retune &&
      buffer != nullptr && size >= 4) {
    const uint32_t freq_hz = ((uint32_t)buffer[0] << 24) |
                             ((uint32_t)buffer[1] << 16) |
                             ((uint32_t)buffer[2] << 8) |
                             (uint32_t)buffer[3];
    pending_rx_retune = false;
    if (lr1121_elrs_set_freq_set_rx(freq_hz, true)) {
      rx_continuous_active = true;
      return;
    }
    DBGLN("SetFreq_SetRx failed, falling back to SetRfFrequency");
  }

  uint8_t patched_buffer[16];
  uint8_t *tx_buffer = buffer;
  if (opcode == LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC && buffer != nullptr &&
      size == 8 && size <= sizeof(patched_buffer)) {
    memcpy(patched_buffer, buffer, size);
    const bool dio1_mask_empty = patched_buffer[4] == 0 &&
                                 patched_buffer[5] == 0 &&
                                 patched_buffer[6] == 0 &&
                                 patched_buffer[7] == 0;
    if (dio1_mask_empty) {
      // Upstream-style code builds the enable mask first. On this board the IRQ
      // line is LR1121 DIO1, so mirror that enable mask into Dio1Mask here.
      patched_buffer[4] = patched_buffer[0];
      patched_buffer[5] = patched_buffer[1];
      patched_buffer[6] = patched_buffer[2];
      patched_buffer[7] = patched_buffer[3];
      tx_buffer = patched_buffer;
    }
  }

  if (pending_rx_retune && opcode != LR11XX_RADIO_SET_RF_FREQUENCY_OC &&
      opcode != LR11XX_SYSTEM_SET_STANDBY_OC) {
    pending_rx_retune = false;
  }

  if (!WaitOnBusy(radioNumber)) {
    busy_timeout_count++;
    DBGLN("WriteCommand BUSY timeout (opcode=0x%04X)", opcode);
    return;
  }

  if (!lr1121_send_command(opcode, tx_buffer, size)) {
    DBGLN("WriteCommand failed (opcode=0x%04X size=%u)", opcode, size);
  }

  if (opcode == LR11XX_SYSTEM_SET_STANDBY_OC && buffer != nullptr &&
      size >= 1) {
    if (rx_continuous_active) {
      pending_rx_retune = true;
    }
    rx_continuous_active = false;
  } else if (opcode == LR11XX_RADIO_SET_RX_OC) {
    rx_continuous_active = true;
    pending_rx_retune = false;
  } else if (opcode == LR11XX_RADIO_SET_TX_OC ||
             opcode == LR11XX_SYSTEM_SET_SLEEP_OC ||
             opcode == LR11XX_SYSTEM_SET_FS_OC) {
    rx_continuous_active = false;
    pending_rx_retune = false;
  }
}

//-----------------------------------------------------------------------------
// SPI Commands - ReadCommand
// ELRS SINGLE-PHASE FULL-DUPLEX (matching upstream ESP32 behavior)
//
// Upstream ESP32 behavior (SPIEx.read):
//   1. Full-duplex transfer: sends buffer contents while receiving
//   2. Response overwrites the same buffer
//-----------------------------------------------------------------------------

void LR1121Hal::ReadCommand(uint8_t *buffer, uint8_t size,
                            SX12XX_Radio_Number_t radioNumber) {
  if (buffer != nullptr && size >= 2) {
    const uint16_t inline_opcode =
        ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    if (inline_opcode == LR11XX_SYSTEM_CLEAR_IRQ_OC) {
      uint8_t tx_buffer[32];
      if (size > sizeof(tx_buffer)) {
        DBGLN("ReadCommand inline opcode too large (opcode=0x%04X size=%u)",
              inline_opcode, size);
        return;
      }
      memcpy(tx_buffer, buffer, size);
      if (!WaitOnBusy(radioNumber)) {
        busy_timeout_count++;
        DBGLN("ReadCommand BUSY timeout before inline opcode=0x%04X size=%u",
              inline_opcode, size);
        return;
      }
      lr1121_cs_assert();
      const bool ok = lr1121_spi_transfer(tx_buffer, buffer, size);
      lr1121_cs_deassert();
      if (!ok) {
        DBGLN("ReadCommand inline transfer failed opcode=0x%04X size=%u",
              inline_opcode, size);
      }
      last_command_opcode = inline_opcode;
      return;
    }
  }

  if (last_command_opcode == LR11XX_RADIO_GET_PACKET) {
    const bool ok = (buffer != nullptr && size > 0) &&
                    lr1121_elrs_get_packet(buffer, size, true);
    last_command_opcode = 0;
    if (!ok) {
      DBGLN("ReadCommand GET_PACKET failed size=%u", size);
    }
    return;
  }

  if (!WaitOnBusy(radioNumber)) {
    busy_timeout_count++;
    DBGLN("ReadCommand BUSY timeout after opcode=0x%04X size=%u",
          last_command_opcode, size);
    return;
  }

  if (size > 0 && buffer != nullptr &&
      !lr1121_read_response(buffer, size)) {
    DBGLN("ReadCommand failed after opcode=0x%04X size=%u",
          last_command_opcode, size);
  }
}

//-----------------------------------------------------------------------------
// BUSY Pin - uses proven lr1121_wait_busy_timeout() from C driver
//-----------------------------------------------------------------------------

bool LR1121Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Upstream ESP targets poll for roughly 2 ms, but the Core1121-HF TCXO/XOSC
  // transitions on this SiW917 board can legitimately hold BUSY longer. A too
  // short timeout silently skips critical config commands such as SetPacketType.
  return lr1121_wait_busy_timeout(100);
}

//-----------------------------------------------------------------------------
// Hardware Interrupt Handlers (ISRs)
// These bridge to the C++ callbacks from the C driver's ISR
//-----------------------------------------------------------------------------

// DEFERRED ISR: SiW917 GSPI transactions are not re-entrant. Keep the GPIO
// callback small, mask DIO1 while the IRQ line is asserted, and do all SPI work
// from elrs_loop().
static volatile bool dio1_isr_pending = false;
static volatile uint32_t dio1_level_requeue_count = 0;
void LR1121Hal::dioISR_1() {
  isr_1_total_count++;
  dio1_isr_pending = true;
  isr_1_pending = true;

  // DIO1 stays high until the LR1121 IRQ is cleared over SPI. Mask it here so
  // the GPIO interrupt cannot repeatedly fire before the deferred handler runs.
  lr1121_dio1_pause_isr();
}

// Called from elrs_loop() to process deferred DIO1 interrupts safely
void LR1121Hal::handleDeferredISR() {
  if (!dio1_isr_pending && lr1121_dio1_read() != 0) {
    // DIO1 is level-high until the LR1121 IRQ is cleared. If a new IRQ arrives
    // while the GPIO edge is masked, there may be no fresh rising edge to wake
    // us, so synthesize one from the level.
    dio1_level_requeue_count++;
    dio1_isr_pending = true;
    isr_1_pending = true;
    lr1121_dio1_pause_isr();
  }

  if (dio1_isr_pending) {
    dio1_isr_pending = false;
    isr_1_pending = false;

    // Call the ISR callback from task context. On SiW917, explicitly clear and
    // re-arm after RX-side IRQs so DIO1 cannot remain asserted and starve the
    // ELRS loop after the first real packet.
    if (instance && instance->IsrCallback_1) {
      LR1121Driver::instance = &Radio;
      uint32_t irqStatus = Radio.GetIrqStatus(SX12XX_Radio_1);
      LR1121Driver::IsrCallbackWithStatus(SX12XX_Radio_1, irqStatus);
      if (irqStatus != 0 && !(irqStatus & (LR1121_IRQ_TX_DONE |
                                           LR1121_IRQ_RX_DONE |
                                           LR1121_IRQ_TIMEOUT))) {
        Radio.ClearIrqStatus(SX12XX_Radio_1);
      }
    }

    lr1121_dio1_resume_isr();
    if (lr1121_dio1_read() != 0) {
      dio1_level_requeue_count++;
      dio1_isr_pending = true;
      isr_1_pending = true;
      lr1121_dio1_pause_isr();
    }
  }
}

void LR1121Hal::dioISR_2() {
  if (instance && instance->IsrCallback_2) {
    instance->IsrCallback_2();
  }
}
