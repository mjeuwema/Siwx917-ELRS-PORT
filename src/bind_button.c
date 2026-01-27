/**
 * @file bind_button.c
 * @brief Binding mode button driver implementation
 *
 * Configures GPIO_11 (BTN1) for binding mode trigger.
 *
 * Register Configuration Citations:
 * - siw917x-family-rm.pdf Section 11.3 and 11.4.3 "MCUHP_PAD_SELECTION" (offset 0x610):
 *   Base address: 0x4130_0000 (GPIO Pad Control Selection)
 *   Offset: 0x610
 *   Bit 6 = NWP_MCUHP_GPIO_CTRL1_6 = GPIO_11 control
 *   Write '1' for MCU control
 *
 * - siw917x-family-rm.pdf Section 11.5/11.6.1 "PAD_CONFIG_REG_x":
 *   Base address: 0x4600_4000
 *   Offset: 0x000 + (0x04 * pin_number) = 0x02C for GPIO_11
 *   Bits 7:6 (P) = 01 for pull-up
 *   Bit 5 (SR) = slew rate
 *   Bit 4 (REN) = 1 for receiver enable
 *   Bit 3 (SMT) = Schmitt trigger
 *
 * - siw917x-family-rm.pdf Section 11.12.9 "GPIO_CONFIG_REG_x":
 *   Base address: 0x4604_6000 (EGPIO)
 *   Offset: 0x000 + (0x04 * pin_number) = 0x02C for GPIO_11
 *   Bits 3:0 (MODE) = 0 for GPIO mode
 *   Bit 8 (DIRECTION) = 1 for input
 */

#include "bind_button.h"
#include <stdio.h>
#include <stdint.h>
#include "cmsis_os2.h"

/*******************************************************************************
 * Debug Output
 ******************************************************************************/
#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define BTN_DBG(fmt, ...) DEBUGOUT("[BIND_BTN] " fmt, ##__VA_ARGS__)

/*******************************************************************************
 * Register Definitions
 *
 * Citation: siw917x-family-rm.pdf Chapter 11 "GPIO"
 ******************************************************************************/

/**
 * SoC and ULP GPIO Pad Control Selection Register Base
 * Citation: siw917x-family-rm.pdf Section 11.3 "SoC and ULP GPIO Pad Control Selection Register Map"
 *   Base address: 0x4130_0000
 * 
 * NOTE: Previous code used 0x46001000 which was INCORRECT and caused
 *       the bind button to not work. The correct base is 0x41300000.
 */
#define GPIO_PAD_CTRL_BASE          0x41300000UL

/**
 * MCUHP_PAD_SELECTION register - MCU/NWP control selection for GPIO_6-57
 * Citation: siw917x-family-rm.pdf Section 11.4.3 "MCUHP_PAD_SELECTION"
 *   Offset: 0x610 from GPIO Pad Control base
 *   Controls GPIO_6-12, GPIO_15, GPIO_31-34, GPIO_46-57
 *   (except GPIO_25-30 which use MEM_GPIO_ACCESS_CTRL_SET/CLEAR)
 */
#define MCUHP_PAD_SELECTION         (*(volatile uint32_t *)(GPIO_PAD_CTRL_BASE + 0x610))

/**
 * Bit mask for GPIO_11 MCU control
 * Citation: siw917x-family-rm.pdf Section 11.4.3
 * Bit 6 = NWP_MCUHP_GPIO_CTRL1_6 = GPIO_11
 */
#define MCUHP_GPIO11_CTRL_BIT       (1UL << 6)

/**
 * Pad Configuration Register base
 * Citation: siw917x-family-rm.pdf Section 11.5
 */
#define PAD_CONFIG_REG_BASE         0x46004000UL

/**
 * Pad Configuration for GPIO_11
 * Citation: siw917x-family-rm.pdf Section 11.6.1
 * Offset: 0x000 + (0x04 * 11) = 0x02C
 */
#define PAD_CONFIG_REG_11           (*(volatile uint32_t *)(PAD_CONFIG_REG_BASE + 0x02C))

/**
 * Pad configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.6.1 "PAD_CONFIG_REG_x"
 */
#define PAD_P_HIZ                   (0UL << 6)   /* High-Z */
#define PAD_P_PULLUP                (1UL << 6)   /* Pull-up */
#define PAD_P_PULLDOWN              (2UL << 6)   /* Pull-down */
#define PAD_REN_ENABLE              (1UL << 4)   /* Receiver enable */
#define PAD_SMT_ENABLE              (1UL << 3)   /* Schmitt trigger */

/**
 * EGPIO Register Base
 * Citation: siw917x-family-rm.pdf Section 11.11 "EGPIO Register Map"
 *   Base address for EGPIO instance: 0x4613_0000
 */
#define EGPIO_BASE                  0x46130000UL

/**
 * GPIO Configuration Register for GPIO_11
 * Citation: siw917x-family-rm.pdf Section 11.11 "EGPIO Register Map"
 *   Offset: 0x000 + (0x10 * x) for GPIO_CONFIG_REG_x
 *   For GPIO_11: 0x000 + (0x10 * 11) = 0x0B0
 */
#define GPIO_CONFIG_REG_11          (*(volatile uint32_t *)(EGPIO_BASE + 0x000 + (0x10 * 11)))

/**
 * GPIO configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.12.1 "GPIO_CONFIG_REG_x"
 *   Bits 5:2 (MODE) = GPIO Pin Mode, selects muxing group/mode
 *   Bit 0 (DIRECTION) = 0 for output, 1 for input
 */
#define GPIO_MODE_MASK              (0x0FUL << 2)  /* Bits 5:2 */
#define GPIO_MODE_GPIO              (0x00UL << 2)  /* Direct GPIO mode (mode 0) */
#define GPIO_DIRECTION_BIT          (1UL << 0)     /* 0=output, 1=input */

/**
 * BIT_LOAD register for reading GPIO pin
 * Citation: siw917x-family-rm.pdf Section 11.11 "EGPIO Register Map"
 *   Base: 0x4613_0000
 *   Offset: 0x004 + (0x10 * x) for BIT_LOAD_REG_x
 *   For GPIO_11: 0x004 + (0x10 * 11) = 0x0B4
 */
#define BIT_LOAD_REG_11             (*(volatile uint32_t *)(EGPIO_BASE + 0x004 + (0x10 * 11)))

/*******************************************************************************
 * Module State
 ******************************************************************************/

static bind_button_callback_t button_callback = NULL;
static uint32_t press_start_tick = 0;
static bool was_pressed = false;
static bool initialized = false;
static uint32_t init_tick = 0;

/* Debounce time in ms after init before accepting button presses */
#define INIT_DEBOUNCE_MS  500

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize the binding button GPIO
 */
int bind_button_init(void)
{
    BTN_DBG("Initializing BTN1 (GPIO_11) for binding mode...\n");
    
    /* Step 1: Select MCU control for GPIO_11
     * Citation: siw917x-family-rm.pdf Section 11.4.3
     * Set bit 6 to '1' for MCU control of GPIO_11
     */
    MCUHP_PAD_SELECTION |= MCUHP_GPIO11_CTRL_BIT;
    BTN_DBG("  MCUHP_PAD_SELECTION = 0x%08lX (MCU control enabled)\n", 
            (unsigned long)MCUHP_PAD_SELECTION);
    
    /* Step 2: Configure pad for input with pull-up
     * Citation: siw917x-family-rm.pdf Section 11.6.1
     * - P = 01 (pull-up) - button is active low
     * - REN = 1 (receiver enabled)
     * - SMT = 1 (Schmitt trigger for noise immunity)
     */
    PAD_CONFIG_REG_11 = PAD_P_PULLUP | PAD_REN_ENABLE | PAD_SMT_ENABLE;
    BTN_DBG("  PAD_CONFIG_REG_11 = 0x%08lX (pull-up, REN, SMT)\n", 
            (unsigned long)PAD_CONFIG_REG_11);
    
    /* Step 3: Configure GPIO for direct control, input direction
     * Citation: siw917x-family-rm.pdf Section 11.12.9
     * - MODE = 0 (GPIO mode)
     * - DIRECTION = 1 (input)
     */
    uint32_t cfg = GPIO_CONFIG_REG_11;
    cfg &= ~GPIO_MODE_MASK;              /* Clear MODE bits */
    cfg |= GPIO_MODE_GPIO;               /* Set MODE = 0 (GPIO) */
    cfg |= GPIO_DIRECTION_BIT;           /* Set direction = input */
    GPIO_CONFIG_REG_11 = cfg;
    BTN_DBG("  GPIO_CONFIG_REG_11 = 0x%08lX (GPIO mode, input)\n", 
            (unsigned long)GPIO_CONFIG_REG_11);
    
    /* Allow GPIO to settle before reading */
    for (volatile int i = 0; i < 100000; i++) { /* ~1ms delay */ }
    
    /* Read initial state and initialize was_pressed to match
     * This prevents false trigger on first poll if button reads as pressed
     */
    initialized = true;  /* Must set before calling is_pressed */
    bool pressed = bind_button_is_pressed();
    was_pressed = pressed;  /* Initialize to current state to prevent false edge detection */
    BTN_DBG("  Initial state: %s\n", pressed ? "PRESSED" : "released");
    
    /* Record init time for debounce */
    init_tick = osKernelGetTickCount();
    
    BTN_DBG("BTN1 initialization complete\n");
    BTN_DBG("  Hold BTN1 for %d seconds to enter binding mode\n", 
            BIND_BUTTON_LONG_PRESS_MS / 1000);
    
    return 0;
}

/**
 * @brief Check if binding button is currently pressed
 */
bool bind_button_is_pressed(void)
{
    if (!initialized) {
        return false;
    }
    
    /* Read GPIO_11 state via BIT_LOAD register
     * Citation: siw917x-family-rm.pdf Section 11.12.4
     * Bit 0 contains the pin logic level
     *
     * Button is active-low (pressed = LOW = 0)
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
     */
    uint32_t pin_state = BIT_LOAD_REG_11 & 0x01;
    return (pin_state == 0);  /* Active low: pressed when LOW */
}

/**
 * @brief Print diagnostic information about button GPIO state
 * 
 * Call this to debug button issues - prints all relevant register values.
 */
void bind_button_print_diagnostics(void)
{
    BTN_DBG("=== BTN1 GPIO_11 Diagnostics ===\n");
    BTN_DBG("  MCUHP_PAD_SELECTION   = 0x%08lX (bit6 should be 1 for MCU ctrl)\n", 
            (unsigned long)MCUHP_PAD_SELECTION);
    BTN_DBG("  PAD_CONFIG_REG_11     = 0x%08lX (expect 0x58: P=01, REN=1, SMT=1)\n", 
            (unsigned long)PAD_CONFIG_REG_11);
    BTN_DBG("  GPIO_CONFIG_REG_11    = 0x%08lX (expect 0x01: MODE=0, DIR=input)\n", 
            (unsigned long)GPIO_CONFIG_REG_11);
    BTN_DBG("  BIT_LOAD_REG_11       = 0x%08lX (bit0 = pin state)\n", 
            (unsigned long)BIT_LOAD_REG_11);
    BTN_DBG("  Button state: %s\n", bind_button_is_pressed() ? "PRESSED" : "RELEASED");
    BTN_DBG("  Callback registered: %s\n", button_callback ? "YES" : "NO");
    BTN_DBG("  Initialized: %s\n", initialized ? "YES" : "NO");
    if (was_pressed) {
        uint32_t duration = bind_button_get_press_duration();
        BTN_DBG("  Currently held for: %lu ms\n", (unsigned long)duration);
    }
}

/**
 * @brief Register callback for button events
 */
void bind_button_set_callback(bind_button_callback_t callback)
{
    button_callback = callback;
    BTN_DBG("Callback %s\n", callback ? "registered" : "unregistered");
}

/**
 * @brief Poll button state and handle press detection
 */
void bind_button_poll(void)
{
    if (!initialized) {
        return;
    }
    
    uint32_t now = osKernelGetTickCount();
    
    /* Ignore button events during post-init debounce period
     * This prevents false triggers from GPIO settling noise
     */
    if ((now - init_tick) < INIT_DEBOUNCE_MS) {
        /* Still in debounce period - just track state without triggering */
        was_pressed = bind_button_is_pressed();
        return;
    }
    
    bool is_pressed = bind_button_is_pressed();
    
    if (is_pressed && !was_pressed) {
        /* Button just pressed - start timing */
        press_start_tick = now;
        was_pressed = true;
        BTN_DBG("Button pressed (starting timer)\n");
    }
    else if (!is_pressed && was_pressed) {
        /* Button just released - check duration */
        uint32_t duration = now - press_start_tick;
        was_pressed = false;
        
        bool is_long_press = (duration >= BIND_BUTTON_LONG_PRESS_MS);
        
        BTN_DBG("Button released after %lu ms (%s press)\n", 
                (unsigned long)duration,
                is_long_press ? "LONG" : "short");
        
        /* Invoke callback */
        if (button_callback != NULL) {
            button_callback(is_long_press);
        }
    }
}

/**
 * @brief Get press duration if button is currently held
 */
uint32_t bind_button_get_press_duration(void)
{
    if (!was_pressed) {
        return 0;
    }
    
    uint32_t now = osKernelGetTickCount();
    return now - press_start_tick;
}
