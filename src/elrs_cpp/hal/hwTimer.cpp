/**
 * @file hwTimer.cpp
 * @brief ELRS hwTimer C++ class implementation for SiW917
 *
 * This file provides the ELRS hwTimer C++ class interface by wrapping
 * the existing C implementation in hw_timer.c which uses the SiW917
 * Configurable Timer (CT) peripheral.
 *
 * The CT provides:
 * - SOC_PLL clock (180MHz, crystal accuracy) - ~5.5ns resolution
 * - 32-bit mode for long intervals
 * - Buffer register for glitch-free interval updates
 *
 * Citation: ExpressLRS 4.0 lib/HWTIMER/hwTimer.h - Class interface
 * Citation: Our hw_timer.c - CT-based implementation
 */

#include "hwTimer.h"
#include "logging.h"

// Include our C timer implementation
extern "C" {
#include "hw_timer.h"
}

// Static member definitions
hwTimerCallback_t hwTimer::callbackTick = nullptr;
hwTimerCallback_t hwTimer::callbackTock = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = 20000; // Default 20ms (50Hz)
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

//-----------------------------------------------------------------------------
// Internal C callback bridge
//-----------------------------------------------------------------------------

// These functions are called by hw_timer.c ISR and bridge to our C++ callbacks
static void hwTimerTickBridge(void) {
  hwTimer::isTick = true;
  if (hwTimer::running && hwTimer::callbackTick) {
    hwTimer::callbackTick();
  }
}

static void hwTimerTockBridge(void) {
  hwTimer::isTick = false;
  if (hwTimer::running && hwTimer::callbackTock) {
    hwTimer::callbackTock();
  }
}

//-----------------------------------------------------------------------------
// hwTimer Class Implementation
//-----------------------------------------------------------------------------

void hwTimer::init(hwTimerCallback_t cbTick, hwTimerCallback_t cbTock) {
  printf(">>> hwTimer::init ENTRY (interval=%lu) <<<\n",
         (unsigned long)HWtimerInterval);

  callbackTick = cbTick;
  callbackTock = cbTock;
  running = false;
  isTick = false;
  PhaseShift = 0;
  FreqOffset = 0;

  // Bypass removed - hwTimer now enabled

  // Initialize the underlying C timer with current interval
  printf("hwTimer::init - calling hw_timer_init...\n");
  sl_status_t status = hw_timer_init(HWtimerInterval);
  printf("hwTimer::init - hw_timer_init returned 0x%04lX\n",
         (unsigned long)status);

  if (status != SL_STATUS_OK) {
    printf("hwTimer::init FAILED!\n");
    DBGLN("hwTimer init failed: 0x%04X", (unsigned)status);
    return;
  }

  // Register our bridge callbacks with the C implementation
  printf("hwTimer::init - setting callbacks...\n");
  hw_timer_set_tick_callback(hwTimerTickBridge);
  hw_timer_set_tock_callback(hwTimerTockBridge);

  printf("hwTimer::init COMPLETE OK\n");
  DBGLN("hwTimer initialized (CT-based)");
}

void hwTimer::stop() {
  if (!running)
    return;

  running = false;
  hw_timer_stop();
  DBGLN("hwTimer stopped");
}

void hwTimer::resume() {
  if (running)
    return;

  // Start fresh - tock should fire first
  isTick = false;
  PhaseShift = 0;

  // Start the CT timer
  hw_timer_start();
  running = true;

  DBGLN("hwTimer resumed, interval=%lu us", HWtimerInterval);
}

void hwTimer::updateInterval(uint32_t newTimerInterval) {
  HWtimerInterval = newTimerInterval;

  // Update the C timer's interval
  hw_timer_set_interval(newTimerInterval);

  DBGLN("hwTimer interval: %lu us", (unsigned long)newTimerInterval);
}

void hwTimer::resetFreqOffset() {
  FreqOffset = 0;
  hw_timer_reset_freq_offset();
}

void hwTimer::phaseShift(int32_t newPhaseShift) {
  // Clamp to reasonable range (±1/4 of interval)
  int32_t maxShift = (int32_t)(HWtimerInterval >> 2);
  if (newPhaseShift > maxShift)
    newPhaseShift = maxShift;
  if (newPhaseShift < -maxShift)
    newPhaseShift = -maxShift;

  PhaseShift = newPhaseShift;

  // Apply to underlying C timer
  hw_timer_phase_shift(newPhaseShift);
}

void hwTimer::incFreqOffset() {
  FreqOffset++;
  hw_timer_inc_freq_offset(1);
}

void hwTimer::decFreqOffset() {
  FreqOffset--;
  hw_timer_inc_freq_offset(-1);
}

// Note: handleISR() is not needed - the C timer calls our bridge functions
// directly
void hwTimer::handleISR() {
  // ISR handling is done by hw_timer.c callback mechanism
  // This function exists for API compatibility but is unused
}
