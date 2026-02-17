/**
 * @file device.cpp
 * @brief Device abstraction stubs for SiW917 platform
 *
 * ELRS uses a device abstraction layer for buttons, LEDs, etc.
 * This integrates with the SiW917 status LED.
 */

#include <stdint.h>

// Include the C status LED API
extern "C" {
#include "status_led.h"
}

// Device event trigger - called when state changes (connection, bind mode, etc.)
void devicesTriggerEvent(uint32_t events)
{
    (void)events;  // Events not used directly - state is tracked via elrs_main
}

// No-arg version for compatibility
void devicesTriggerEvent()
{
    // No-op
}
