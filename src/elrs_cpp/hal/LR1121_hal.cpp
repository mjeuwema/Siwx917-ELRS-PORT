/**
 * @file LR1121_hal.cpp
 * @brief SiW917-specific LR1121 HAL implementation
 *
 * This replaces the upstream LR1121_hal.cpp with SiW917-specific code.
 * Uses our proven lr1121_driver.c functions for hardware access.
 *
 * IMPORTANT: Core1121/Waveshare module requires TCXO initialization that
 * standard ESP32 ELRS targets don't need. This is handled in
 * lr1121_waveshare_init().
 *
 * Key functions from C driver:
 * - lr1121_send_command() - handles full command protocol
 * - lr1121_read_response() - handles response reading
 * - lr1121_wait_busy_timeout() - handles BUSY pin polling
 * - lr1121_dio1_*() - handles DIO1 interrupt
 */

#include "../lib/LR1121Driver/LR1121_hal.h"
#include "../lib/LR1121Driver/LR1121_Regs.h"
#include "logging.h"
#include "targets.h"

// Include our proven C driver implementation
extern "C" {
#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
}

volatile uint32_t isr_1_pending_count = 0;
volatile uint32_t isr_1_total_count = 0;
volatile uint32_t isr_2_pending_count = 0;
volatile uint32_t busy_timeout_count = 0;

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
// Uses proven lr1121_send_command() from C driver
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode,
                             SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Wait for BUSY before sending command
  if (!lr1121_wait_busy_timeout(10)) {
    busy_timeout_count++;
    DBGLN("WriteCommand BUSY timeout (opcode=0x%04X)", opcode);
    return;
  }

  // Send command using proven C driver function
  lr1121_send_command(opcode, nullptr, 0);
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode + data)
// Uses proven lr1121_send_command() from C driver
//-----------------------------------------------------------------------------

void LR1121Hal::WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size,
                             SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Wait for BUSY before sending command
  if (!lr1121_wait_busy_timeout(10)) {
    busy_timeout_count++;
    DBGLN("WriteCommand BUSY timeout (opcode=0x%04X)", opcode);
    return;
  }

  // Send command with data using proven C driver function
  lr1121_send_command(opcode, buffer, size);
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
  (void)radioNumber;

  // Wait for BUSY before Phase 2 transfer
  // Citation: UserManual_LR1121_v1_2.pdf Section 3.2.3 Read Command
  if (!lr1121_wait_busy_timeout(10)) {
    busy_timeout_count++;
    DBGLN("ReadCommand BUSY timeout");
    return;
  }

  // ELRS Full-Duplex Hack: Output the caller's buffer on MOSI while
  // capturing the response on MISO. This perfectly mimics the ESP32
  // SPIEx.transferBytes behavior, which upstream ELRS relies on to pack
  // opcodes for instantaneous commands (like GetIrqStatus) into the buffer.
  lr1121_cs_assert();

  if (size > 0) {
    // spi_transfer correctly handles tx_data and rx_data pointing to the
    // exact same memory because it uses statically allocated internal DMA buffers.
    lr1121_spi_transfer(buffer, buffer, size);
  }

  lr1121_cs_deassert();
}

//-----------------------------------------------------------------------------
// BUSY Pin - uses proven lr1121_wait_busy_timeout() from C driver
//-----------------------------------------------------------------------------

bool LR1121Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Use 2ms timeout as per upstream ELRS
  return lr1121_wait_busy_timeout(2);
}

//-----------------------------------------------------------------------------
// Hardware Interrupt Handlers (ISRs)
// These bridge to the C++ callbacks from the C driver's ISR
//-----------------------------------------------------------------------------

// DEFERRED ISR: The SiW917 GSPI DMA is NOT re-entrant.
// If the ISR fires while the main loop is mid-SPI transfer, both try to
// use the same DMA buffers/CS pin, causing a hard fault.
//
// ADDITIONAL HAZARD: The LR1121 holds DIO1 HIGH until IRQs are cleared via
// SPI. If the edge detector re-fires or the interrupt is treated as level,
// the ISR will loop infinitely, starving the main loop. Solution: disable
// the interrupt in the ISR, re-enable after processing in the main loop.
static volatile bool dio1_isr_pending = false;

void LR1121Hal::dioISR_1() {
  isr_1_total_count++;
  dio1_isr_pending = true;

  // CRITICAL: Disable DIO1 interrupt to prevent infinite re-fire.
  // DIO1 stays HIGH until IRQ is cleared via SPI in handleDeferredISR().
  // We re-enable after the IRQ is cleared and DIO1 goes LOW.
  lr1121_dio1_pause_isr();
}

// Called from elrs_loop() to process deferred DIO1 interrupts safely
void LR1121Hal::handleDeferredISR() {
  if (dio1_isr_pending) {
    dio1_isr_pending = false;

    // Call the IsrCallback which does SPI (GetIrqStatus clears IRQ, DIO1→LOW)
    if (instance && instance->IsrCallback_1) {
      uint32_t irqStatus = LR1121Driver::instance->GetIrqStatus(SX12XX_Radio_1);
      LR1121Driver::IsrCallbackWithStatus(SX12XX_Radio_1, irqStatus);
    }

    // Now DIO1 should be LOW (IRQ cleared). Re-enable the interrupt
    // so the next rising edge fires.
    lr1121_dio1_resume_isr();
  }
}

void LR1121Hal::dioISR_2() {
  if (instance && instance->IsrCallback_2) {
    instance->IsrCallback_2();
  }
}
