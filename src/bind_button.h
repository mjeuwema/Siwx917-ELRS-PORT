/**
 * @file bind_button.h
 * @brief Binding mode button driver for ELRS receiver
 *
 * Uses BTN1 (GPIO_11) on BRD2708A to trigger binding mode.
 * BTN0 (UULP_VBAT_GPIO_2) is NOT used as it conflicts with LR1121 DIO1.
 *
 * Hardware Documentation Citations:
 * - ug590-brd2708a-user-guide.pdf Section 3.4 "Push Buttons and LEDs":
 *   "BTN1 is connected to GPIO_11"
 *   "The logic state of a button is high while that button is not being pressed,
 *    and low when it is pressed."
 *   "debounced by an RC filter with a time constant of 1 ms"
 *
 * - siw917x-family-rm.pdf Section 11.4.3 "MCUHP_PAD_SELECTION":
 *   Bit 6 = NWP_MCUHP_GPIO_CTRL1_6 = Control select for GPIO_11
 *   Set to '1' for MCU control
 *
 * - siw917x-family-rm.pdf Section 11.6.1 "PAD_CONFIG_REG_x":
 *   REN (bit 4) = Receiver Enable
 *   P (bits 7:6) = Driver disabled state (01 = pull-up)
 *
 * @note BTN1 has hardware debouncing (1ms RC filter) on the board
 */

#ifndef BIND_BUTTON_H
#define BIND_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/

/**
 * @brief GPIO pin number for binding button (BTN1)
 * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
 */
#define BIND_BUTTON_GPIO_PIN    11

/**
 * @brief Long press duration to trigger binding mode (milliseconds)
 * Prevents accidental binding mode entry
 */
#define BIND_BUTTON_LONG_PRESS_MS   3000

/**
 * @brief Debounce time in milliseconds (software debounce)
 * Note: Hardware already has 1ms RC debounce, but we add software debounce for reliability
 */
#define BIND_BUTTON_DEBOUNCE_MS     50

/*******************************************************************************
 * Types
 ******************************************************************************/

/**
 * @brief Button event callback type
 * @param long_press true if long press detected, false for short press
 */
typedef void (*bind_button_callback_t)(bool long_press);

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize the binding button GPIO
 *
 * Configures GPIO_11 as input with pull-up:
 * - Sets MCU control (not NWP)
 * - Enables receiver
 * - Configures pull-up resistor
 * - Sets GPIO mode for direct control
 *
 * @return 0 on success, negative error code on failure
 */
int bind_button_init(void);

/**
 * @brief Check if binding button is currently pressed
 *
 * Reads the raw GPIO state. Button is active-low (pressed = LOW).
 *
 * @return true if button is pressed (GPIO LOW), false otherwise
 */
bool bind_button_is_pressed(void);

/**
 * @brief Register callback for button events
 *
 * The callback is invoked on button release, with long_press indicating
 * whether the button was held for >= BIND_BUTTON_LONG_PRESS_MS.
 *
 * @param callback Function to call on button events (NULL to disable)
 */
void bind_button_set_callback(bind_button_callback_t callback);

/**
 * @brief Poll button state and handle press detection
 *
 * Call this periodically (e.g., every 10-50ms) from your main loop or timer.
 * Handles software debouncing and long-press detection.
 *
 * When a button release is detected after a long press (>=3 seconds),
 * the registered callback is invoked with long_press=true.
 */
void bind_button_poll(void);

/**
 * @brief Get press duration if button is currently held
 *
 * @return Press duration in milliseconds, or 0 if not pressed
 */
uint32_t bind_button_get_press_duration(void);

/**
 * @brief Print diagnostic information about button GPIO state
 * 
 * Useful for debugging button issues - prints all relevant register values
 * and current button state.
 */
void bind_button_print_diagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* BIND_BUTTON_H */
