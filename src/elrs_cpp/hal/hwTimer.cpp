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
 * - 16-bit Counter 0 mode for ELRS half-intervals
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

static volatile bool immediateTockPending = false;
static volatile uint32_t activeEventMicros = 0;

enum TimerEventType : uint8_t {
  TIMER_EVENT_TICK = 1,
  TIMER_EVENT_TOCK = 2,
};

struct TimerEvent {
  uint8_t type;
  uint32_t timestampUs;
};

static constexpr uint16_t TIMER_EVENT_QUEUE_SIZE = 128;
static TimerEvent timerEventQueue[TIMER_EVENT_QUEUE_SIZE] = {};
static volatile uint16_t timerEventHead = 0;
static volatile uint16_t timerEventTail = 0;
static volatile uint32_t timerEventOverflowCount = 0;
static volatile uint32_t queuedTickCount = 0;
static volatile uint32_t queuedTockCount = 0;
static volatile uint32_t processedTickCount = 0;
static volatile uint32_t processedTockCount = 0;
static volatile uint32_t immediateTockDeliveredCount = 0;

static void resetTimerEventQueue() {
  immediateTockPending = false;
  timerEventHead = 0;
  timerEventTail = 0;
}

static void resetTimerEventStats() {
  timerEventOverflowCount = 0;
  queuedTickCount = 0;
  queuedTockCount = 0;
  processedTickCount = 0;
  processedTockCount = 0;
  immediateTockDeliveredCount = 0;
}

static void enqueueTimerEvent(uint8_t type) {
  const uint16_t head = timerEventHead;
  const uint16_t nextHead =
      (uint16_t)((head + 1U) % TIMER_EVENT_QUEUE_SIZE);

  if (nextHead == timerEventTail) {
    // Single-producer/single-consumer queue: the ISR owns head only, while the
    // task owns tail. Dropping the newest event is safer than moving tail here.
    timerEventOverflowCount++;
    return;
  }

  timerEventQueue[head].type = type;
  timerEventQueue[head].timestampUs = micros();
  timerEventHead = nextHead;

  if (type == TIMER_EVENT_TICK) {
    queuedTickCount++;
  } else if (type == TIMER_EVENT_TOCK) {
    queuedTockCount++;
  }
}

static bool dequeueTimerEvent(TimerEvent *event) {
  if (timerEventTail == timerEventHead) {
    return false;
  }

  *event = timerEventQueue[timerEventTail];
  timerEventTail =
      (uint16_t)((timerEventTail + 1U) % TIMER_EVENT_QUEUE_SIZE);
  return true;
}

//-----------------------------------------------------------------------------
// Internal C callback bridge
//-----------------------------------------------------------------------------

// These functions are called by hw_timer.c ISR. Queue the work so ELRS core
// callbacks can do LR1121 SPI from task context instead of timer interrupt
// context; the queued timestamp preserves PFD timing.
static void hwTimerTickBridge(void) {
  if (hwTimer::running) {
    enqueueTimerEvent(TIMER_EVENT_TICK);
  }
}

static void hwTimerTockBridge(void) {
  if (hwTimer::running) {
    enqueueTimerEvent(TIMER_EVENT_TOCK);
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
  activeEventMicros = 0;
  resetTimerEventQueue();
  resetTimerEventStats();

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
  resetTimerEventQueue();
  hw_timer_stop();
  DBGLN("hwTimer stopped");
}

void hwTimer::resume() {
  if (running)
    return;

  // Match upstream RX behavior: enabling the timer schedules an immediate TOCK,
  // but that TOCK does not interrupt the currently running radio callback.
  isTick = false;
  PhaseShift = 0;
  activeEventMicros = 0;
  resetTimerEventQueue();
  resetTimerEventStats();

  sl_status_t status = hw_timer_start();
  if (status != SL_STATUS_OK) {
    running = false;
    DBGLN("hwTimer resume failed: 0x%04X", (unsigned)status);
    return;
  }

  running = true;
  isTick = false;
  hw_timer_note_immediate_tock();
  immediateTockPending = true;

  DBGLN("hwTimer resumed, interval=%lu us", HWtimerInterval);
}

void hwTimer::service() {
  if (!running) {
    return;
  }

  if (immediateTockPending) {
    immediateTockPending = false;
    isTick = false;
    activeEventMicros = micros();
    immediateTockDeliveredCount++;
    processedTockCount++;

    if (callbackTock) {
      callbackTock();
    }

    activeEventMicros = 0;
    isTick = true;
  }

  TimerEvent event;
  while (running && dequeueTimerEvent(&event)) {
    activeEventMicros = event.timestampUs;
    if (event.type == TIMER_EVENT_TICK) {
      isTick = true;
      processedTickCount++;
      if (callbackTick) {
        callbackTick();
      }
      isTick = false;
    } else if (event.type == TIMER_EVENT_TOCK) {
      isTick = false;
      processedTockCount++;
      if (callbackTock) {
        callbackTock();
      }
      isTick = true;
    }
    activeEventMicros = 0;
  }
}

uint32_t hwTimer::eventMicros() {
  const uint32_t eventTime = activeEventMicros;
  return eventTime != 0 ? eventTime : micros();
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
  // Clamp to reasonable range (+/- 1/4 of interval)
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
  hw_timer_inc_freq_offset(1);
  FreqOffset = hw_timer_get_freq_offset();
}

void hwTimer::decFreqOffset() {
  hw_timer_inc_freq_offset(-1);
  FreqOffset = hw_timer_get_freq_offset();
}

// Note: handleISR() is not needed - the C timer calls our bridge functions
// directly
void hwTimer::handleISR() {
  // ISR handling is done by hw_timer.c callback mechanism
  // This function exists for API compatibility but is unused
}

uint32_t hwTimer::getHardwareHalfTicks() {
  return hw_timer_get_total_half_ticks();
}

uint32_t hwTimer::getHardwareCount() { return hw_timer_get_current_count(); }

uint32_t hwTimer::getHardwareMatch() { return hw_timer_get_match_value(); }

uint32_t hwTimer::getHardwareFreqHz() { return hw_timer_get_ct_freq_hz(); }

uint32_t hwTimer::getQueuedTickCount() { return queuedTickCount; }

uint32_t hwTimer::getQueuedTockCount() { return queuedTockCount; }

uint32_t hwTimer::getProcessedTickCount() { return processedTickCount; }

uint32_t hwTimer::getProcessedTockCount() { return processedTockCount; }

uint32_t hwTimer::getQueueOverflowCount() {
  return timerEventOverflowCount;
}

uint32_t hwTimer::getImmediateTockDeliveredCount() {
  return immediateTockDeliveredCount;
}
