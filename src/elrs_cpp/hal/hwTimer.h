/**
 * @file hwTimer.h
 * @brief Hardware Timer abstraction for ELRS on SiW917
 *
 * Provides precise timing for the ELRS protocol packet timing.
 * Uses SiW917's hardware timer peripheral for microsecond precision.
 */
#pragma once

#include "targets.h"
#include <stdint.h>

// Timer callback function type
typedef void (*hwTimerCallback_t)(void);

class hwTimer
{
public:
    /**
     * @brief Initialize the hardware timer
     * @param callbackTick Function called on tick interrupt
     * @param callbackTock Function called on tock interrupt
     */
    static void init(hwTimerCallback_t callbackTick, hwTimerCallback_t callbackTock);

    /**
     * @brief Stop the timer
     */
    static void stop();

    /**
     * @brief Resume the timer
     */
    static void resume();

    /**
     * @brief Update the timer interval
     * @param newTimerInterval New interval in microseconds
     */
    static void updateInterval(uint32_t newTimerInterval);

    /**
     * @brief Reset the frequency offset applied to timer
     */
    static void resetFreqOffset();

    /**
     * @brief Phase shift the timer by the given amount
     * @param newPhaseShift Amount to shift in microseconds (can be negative)
     */
    static void phaseShift(int32_t newPhaseShift);

    /**
     * @brief Increment the frequency offset
     * @param offset Amount to adjust frequency
     */
    static void incFreqOffset();

    /**
     * @brief Decrement the frequency offset
     */
    static void decFreqOffset();

    /**
     * @brief Get current timer interval
     * @return Current interval in microseconds
     */
    static uint32_t getInterval() { return HWtimerInterval; }

    /**
     * @brief Check if timer is running
     * @return true if running
     */
    static bool isRunning() { return running; }

    // Timer state - public for ISR access
    static volatile bool running;
    static volatile bool isTick;
    static volatile uint32_t HWtimerInterval;
    static volatile int32_t PhaseShift;
    static volatile int32_t FreqOffset;

    // Callbacks
    static hwTimerCallback_t callbackTick;
    static hwTimerCallback_t callbackTock;

    // ISR handler - exists for API compatibility (actual ISR in hw_timer.c)
    static void IRAM_ATTR handleISR();
};

// Macro for ISR attribute (platform specific)
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
