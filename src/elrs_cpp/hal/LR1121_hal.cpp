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

  // Perform hardware reset via C driver
  lr1121_status_t status = lr1121_reset();
  if (status != LR1121_OK) {
    DBGLN("LR1121 reset failed: %d", (int)status);
  }

  // Wait for BUSY to go LOW after reset (can take up to 300ms)
  if (!lr1121_wait_busy_timeout(500)) {
    DBGLN("LR1121 post-reset BUSY timeout");
  }
}

//-----------------------------------------------------------------------------
// SPI Commands - WriteCommand (opcode only)
// Uses proven lr1121_send_command() from C driver
//-----------------------------------------------------------------------------

void ICACHE_RAM_ATTR
LR1121Hal::WriteCommand(uint16_t opcode, SX12XX_Radio_Number_t radioNumber) {
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

void ICACHE_RAM_ATTR
LR1121Hal::WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size,
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

void ICACHE_RAM_ATTR LR1121Hal::ReadCommand(uint8_t *buffer, uint8_t size,
                                            SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Wait for BUSY before transfer
  if (!lr1121_wait_busy_timeout(10)) {
    busy_timeout_count++;
    DBGLN("ReadCommand BUSY timeout");
    return;
  }

  // Full-duplex transfer: send extended tx_buf, receive into rx_buf
  uint8_t tx_buf[256];
  uint8_t rx_buf[256];

  // We transfer exactly size bytes
  uint16_t transfer_size = size;

  for (uint16_t i = 0; i < transfer_size; i++) {
    tx_buf[i] = buffer[i];
  }

  // Assert CS, perform full-duplex transfer, deassert CS
  lr1121_cs_assert();
  lr1121_spi_transfer(tx_buf, rx_buf, transfer_size);
  lr1121_cs_deassert();

  for (uint8_t i = 0; i < size; i++) {
    buffer[i] = rx_buf[i];
  }
}

//-----------------------------------------------------------------------------
// BUSY Pin - uses proven lr1121_wait_busy_timeout() from C driver
//-----------------------------------------------------------------------------

bool ICACHE_RAM_ATTR LR1121Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber) {
  // We only support single radio (Radio_1)
  (void)radioNumber;

  // Use 2ms timeout as per upstream ELRS
  return lr1121_wait_busy_timeout(2);
}

//-----------------------------------------------------------------------------
// Hardware Interrupt Handlers (ISRs)
// These bridge to the C++ callbacks from the C driver's ISR
//-----------------------------------------------------------------------------

void ICACHE_RAM_ATTR LR1121Hal::dioISR_1() {
  if (instance && instance->IsrCallback_1) {
    isr_1_pending_count++;
  }
}

void ICACHE_RAM_ATTR LR1121Hal::dioISR_2() {
  if (instance && instance->IsrCallback_2) {
    isr_2_pending_count++;
  }
}
