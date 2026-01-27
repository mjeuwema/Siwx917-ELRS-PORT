/**
 * @file status_led.h
 * @brief Status LED driver for ELRS receiver mode indication
 *
 * Provides visual feedback via LED0 and LED1 on BRD2708A board.
 *
 * Hardware Documentation Citations:
 * - ug590-brd2708a-user-guide.pdf Section 3.4 "Push Buttons and LEDs":
 *   "The kit also features two yellow LEDs, marked LED0 and LED1, that are
 *    controlled by GPIO pins on the SiWG917Y Module. The LEDs are connected
 *    to pin GPIO_10 and ULP_GPIO_2, respectively, in an active-high configuration."
 *
 * LED Modes:
 *   - DISCONNECTED: LED0 slow blink (1Hz)
 *   - TENTATIVE:    LED0 fast blink (4Hz)
 *   - CONNECTED:    LED0 solid ON
 *   - BINDING:      Both LEDs alternating fast (4Hz)
 *   - WIFI:         LED1 fast blink (4Hz), LED0 off
 *   - ERROR:        Both LEDs rapid flash (8Hz)
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * LED Mode Definitions
 ******************************************************************************/

/**
 * @brief Status LED indication modes
 */
typedef enum {
    LED_MODE_OFF = 0,       /**< All LEDs off */
    LED_MODE_DISCONNECTED,  /**< Waiting for TX - slow blink LED0 */
    LED_MODE_TENTATIVE,     /**< Sync in progress - fast blink LED0 */
    LED_MODE_CONNECTED,     /**< Connected - solid LED0 */
    LED_MODE_BINDING,       /**< Binding mode - alternating blink both */
    LED_MODE_WIFI,          /**< WiFi mode - fast blink LED1 */
    LED_MODE_ERROR,         /**< Error state - rapid flash both */
    LED_MODE_BOOT,          /**< Boot sequence - sweep pattern */
} status_led_mode_t;

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/

/**
 * @brief LED0 GPIO pin number
 * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
 */
#define STATUS_LED0_GPIO_PIN    10

/**
 * @brief LED1 GPIO pin number (ULP domain)
 * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
 */
#define STATUS_LED1_ULP_GPIO_PIN    2

/**
 * @brief Blink intervals in milliseconds
 */
#define LED_BLINK_SLOW_MS       500     /**< 1Hz blink (500ms period) */
#define LED_BLINK_FAST_MS       125     /**< 4Hz blink (125ms period) */
#define LED_BLINK_RAPID_MS      62      /**< 8Hz blink (62ms period) */

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize the status LEDs
 *
 * Configures GPIO_10 (LED0) and ULP_GPIO_2 (LED1) as outputs:
 * - Sets MCU control for both pins
 * - Configures pad for output with drive strength
 * - Sets GPIO mode for direct control
 * - Initially turns both LEDs off
 *
 * Register Configuration Citations:
 * - siw917x-family-rm.pdf Section 11.4.3 MCUHP_PAD_SELECTION:
 *   Bit 5 = GPIO_10 MCU control
 * - siw917x-family-rm.pdf Section 11.4.4 MCUHP_PAD_SELECTION_1:
 *   Bit 2 = ULP_GPIO_2 MCU control
 * - siw917x-family-rm.pdf Section 11.6.1 PAD_CONFIG_REG_x:
 *   Drive strength, slew rate configuration
 * - siw917x-family-rm.pdf Section 11.12.9 GPIO_CONFIG_REG_x:
 *   MODE = 0 for GPIO, DIRECTION = 0 for output
 *
 * @return 0 on success, negative error code on failure
 */
int status_led_init(void);

/**
 * @brief Set the LED indication mode
 *
 * Changes the current LED pattern based on receiver state.
 *
 * @param mode The new LED mode to display
 */
void status_led_set_mode(status_led_mode_t mode);

/**
 * @brief Get the current LED mode
 *
 * @return Current LED indication mode
 */
status_led_mode_t status_led_get_mode(void);

/**
 * @brief Update LED state (call periodically)
 *
 * This function handles the blinking patterns. Call from main loop
 * or a timer at least every 50ms for smooth patterns.
 *
 * Uses osKernelGetTickCount() for timing.
 */
void status_led_update(void);

/**
 * @brief Directly control LED0
 *
 * Low-level control, bypasses mode state machine.
 *
 * @param on true = LED on, false = LED off
 */
void status_led0_set(bool on);

/**
 * @brief Directly control LED1
 *
 * Low-level control, bypasses mode state machine.
 *
 * @param on true = LED on, false = LED off
 */
void status_led1_set(bool on);

/**
 * @brief Toggle LED0
 */
void status_led0_toggle(void);

/**
 * @brief Toggle LED1
 */
void status_led1_toggle(void);

/**
 * @brief Run boot LED animation
 *
 * Blocking function that shows a startup sequence.
 * Call once during initialization to indicate successful boot.
 *
 * Pattern: Both LEDs flash 3 times
 */
void status_led_boot_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_LED_H */
