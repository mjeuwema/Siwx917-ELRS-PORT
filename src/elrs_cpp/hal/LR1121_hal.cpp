/**
 * @file LR1121_hal.cpp
 * @brief SiW917-specific LR1121 HAL implementation
 * 
 * This replaces the upstream LR1121_hal.cpp with SiW917-specific code.
 * Uses our proven lr1121_driver.c functions for hardware access.
 * 
 * IMPORTANT: Core1121/Waveshare module requires TCXO initialization that
 * standard ESP32 ELRS targets don't need. This is handled in lr1121_waveshare_init().
 * 
 * Key functions from C driver:
 * - lr1121_send_command() - handles full command protocol
 * - lr1121_read_response() - handles response reading
 * - lr1121_wait_busy_timeout() - handles BUSY pin polling
 * - lr1121_dio1_*() - handles DIO1 interrupt
 */

#include "targets.h"
#include "../lib/LR1121Driver/LR1121_hal.h"
#include "../lib/LR1121Driver/LR1121_Regs.h"
#include "logging.h"

// Include our proven C driver implementation
extern "C" {
#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"
}

// Static instance pointer
LR1121Hal *LR1121Hal::instance = nullptr;

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------

LR1121Hal::LR1121Hal()
{
    instance = this;
    IsrCallback_1 = nullptr;
    IsrCallback_2 = nullptr;
}

//-----------------------------------------------------------------------------
// Initialization
// Note: Core1121/Waveshare requires TCXO setup not needed on standard ESP32 targets
//-----------------------------------------------------------------------------

void LR1121Hal::init()
{
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
    
    // Configure DIO1 as input and attach interrupt
    // Matches ESP32: attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1), dioISR_1, RISING)
    lr1121_dio1_init();
    lr1121_dio1_set_callback(dioISR_1);
    lr1121_dio1_enable();
    
    DBGLN("LR1121Hal initialized");
}

void LR1121Hal::end()
{
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

void LR1121Hal::reset(bool bootloader)
{
    DBGLN("LR1121Hal::reset(bootloader=%d)", bootloader);
    (void)bootloader;  // Not used - no bootloader mode support
    
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

void ICACHE_RAM_ATTR LR1121Hal::WriteCommand(uint16_t opcode, SX12XX_Radio_Number_t radioNumber)
{
    // We only support single radio (Radio_1)
    (void)radioNumber;
    
    // Wait for BUSY before sending command
    if (!lr1121_wait_busy_timeout(10)) {
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

void ICACHE_RAM_ATTR LR1121Hal::WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    // We only support single radio (Radio_1)
    (void)radioNumber;
    
    // Wait for BUSY before sending command
    if (!lr1121_wait_busy_timeout(10)) {
        DBGLN("WriteCommand BUSY timeout (opcode=0x%04X)", opcode);
        return;
    }
    
    // Send command with data using proven C driver function
    lr1121_send_command(opcode, buffer, size);
}

//-----------------------------------------------------------------------------
// SPI Commands - ReadCommand
// CRITICAL: This must match upstream ELRS behavior!
// Upstream sends buffer contents AND receives response in one SPI transaction.
// The buffer contains a command (e.g. ClearIrq opcode + args) and gets overwritten
// with the response.
//-----------------------------------------------------------------------------

void ICACHE_RAM_ATTR LR1121Hal::ReadCommand(uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    // We only support single radio (Radio_1)
    (void)radioNumber;
    
    // Wait for BUSY
    if (!lr1121_wait_busy_timeout(10)) {
        DBGLN("ReadCommand BUSY timeout");
        return;
    }
    
    // Upstream pattern: buffer contains command to send, gets overwritten with response
    // This is a full-duplex SPI transfer WITH CS handling
    lr1121_cs_assert();
    lr1121_spi_transfer(buffer, buffer, size);
    lr1121_cs_deassert();
}

//-----------------------------------------------------------------------------
// BUSY Pin - uses proven lr1121_wait_busy_timeout() from C driver
//-----------------------------------------------------------------------------

bool ICACHE_RAM_ATTR LR1121Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber)
{
    // We only support single radio (Radio_1)
    (void)radioNumber;
    
    // Use 2ms timeout as per upstream ELRS
    return lr1121_wait_busy_timeout(2);
}

//-----------------------------------------------------------------------------
// DIO1 Interrupt Handlers
// These bridge to the C++ callbacks from the C driver's ISR
//-----------------------------------------------------------------------------

void ICACHE_RAM_ATTR LR1121Hal::dioISR_1()
{
    if (instance && instance->IsrCallback_1) {
        instance->IsrCallback_1();
    }
}

void ICACHE_RAM_ATTR LR1121Hal::dioISR_2()
{
    if (instance && instance->IsrCallback_2) {
        instance->IsrCallback_2();
    }
}
