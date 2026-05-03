/**
 * @file Arduino.cpp
 * @brief Arduino API implementation for SiW917
 *
 * Implements Arduino-compatible functions using SiW917 SDK.
 */

#include "targets.h"

// Suppress missing-field-initializers warnings from SDK headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// Use the same GPIO headers as the working lr1121_driver.c
#include "rsi_egpio.h"
#include "rsi_rom_egpio.h"
#include "rsi_rom_clks.h"

#pragma GCC diagnostic pop

#include "cmsis_os2.h"
#include <cstdio>
#include <cstdlib>

// Include our C driver for GPIO operations
extern "C" {
#include "lr1121_driver.h"
}

//=============================================================================
// Serial Output
//=============================================================================

HardwareSerial Serial;
Stream *BackpackOrLogStrm = &Serial;

size_t HardwareSerial::write(uint8_t c) {
    // Use SiW917 debug UART via printf
    putchar(c);
    return 1;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        putchar(buffer[i]);
    }
    return size;
}

//=============================================================================
// Timing Functions
//=============================================================================

extern "C" {

uint32_t millis(void) {
    return osKernelGetTickCount();
}

uint32_t micros(void) {
    // Combine the RTOS tick counter with the live SysTick down-counter to
    // preserve sub-millisecond timing. ELRS phase locking depends on this.
    uint32_t tick_before;
    uint32_t tick_after;
    uint32_t systick_val;
    const uint32_t systick_reload = SysTick->LOAD + 1U;

    do {
        tick_before = osKernelGetTickCount();
        systick_val = SysTick->VAL;
        tick_after = osKernelGetTickCount();
    } while (tick_before != tick_after);

    uint32_t elapsed_cycles = systick_reload - systick_val;
    uint32_t sub_ms_us =
        (uint32_t)(((uint64_t)elapsed_cycles * 1000ULL) / systick_reload);

    return (tick_before * 1000U) + sub_ms_us;
}

void delay(uint32_t ms) {
    osDelay(ms);
}

void delayMicroseconds(uint32_t us) {
    if (us == 0) {
        return;
    }

    uint32_t start = micros();
    while ((micros() - start) < us) {
        __NOP();
    }
}

void yield(void) {
    osThreadYield();
}

//=============================================================================
// GPIO Functions  
// Using RSI EGPIO API (same as working lr1121_driver.c)
//=============================================================================

void pinMode(int pin, int mode) {
    if (pin == UNDEF_PIN) return;
    
    // Enable EGPIO clock if not already enabled
    RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);
    
    // Configure pad selection for the pin
    RSI_EGPIO_PadSelectionEnable(pin / 16 + 1);  // PAD 1-4 based on pin range
    
    // Set pin MUX to GPIO mode (mode 0)
    RSI_EGPIO_SetPinMux(EGPIO, 0, pin, 0);  // Port 0, GPIO mode
    
    if (mode == OUTPUT) {
        RSI_EGPIO_SetDir(EGPIO, 0, pin, 0);  // 0 = output
    } else {
        RSI_EGPIO_SetDir(EGPIO, 0, pin, 1);  // 1 = input
    }
}

void digitalWrite(int pin, int value) {
    if (pin == UNDEF_PIN) return;
    
    if (value) {
        RSI_EGPIO_SetPin(EGPIO, 0, pin, 1);
    } else {
        RSI_EGPIO_SetPin(EGPIO, 0, pin, 0);
    }
}

int digitalRead(int pin) {
    if (pin == UNDEF_PIN) return LOW;
    
    return RSI_EGPIO_GetPin(EGPIO, 0, pin) ? HIGH : LOW;
}

// Interrupt callback storage
static void (*gpio_callbacks[64])(void) = {nullptr};

void attachInterrupt(int pin, void (*callback)(void), int mode) {
    if (pin == UNDEF_PIN || pin >= 64) return;
    
    gpio_callbacks[pin] = callback;
    
    // Note: Actual interrupt setup is handled by lr1121_driver for DIO1
    // This Arduino shim just stores the callback
    // For DIO1, lr1121_driver.c configures the UULP GPIO interrupt directly
    (void)mode;  // Mode handled in driver setup
}

void detachInterrupt(int pin) {
    if (pin == UNDEF_PIN || pin >= 64) return;
    gpio_callbacks[pin] = nullptr;
}

} // extern "C"

//=============================================================================
// C++ Runtime Support (for baremetal embedded)
//=============================================================================

// These are required when using C++ classes with virtual destructors
// but not using the full C++ standard library
void* operator new(size_t size) {
    void* ptr = malloc(size);
    if (ptr == nullptr) {
        std::fputs("fatal: operator new failed\n", stderr);
        std::abort();
    }
    return ptr;
}

void* operator new[](size_t size) {
    void* ptr = malloc(size);
    if (ptr == nullptr) {
        std::fputs("fatal: operator new[] failed\n", stderr);
        std::abort();
    }
    return ptr;
}

void operator delete(void* ptr) noexcept {
    free(ptr);
}

void operator delete[](void* ptr) noexcept {
    free(ptr);
}

void operator delete(void* ptr, size_t) noexcept {
    free(ptr);
}

void operator delete[](void* ptr, size_t) noexcept {
    free(ptr);
}
