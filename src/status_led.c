/**
 * @file status_led.c
 * @brief Status LED driver implementation for BRD2708A
 *
 * Uses only LED0 (GPIO_10) for receiver status indication.
 * NOTE: LED1 (ULP_GPIO_2) is NOT used because it conflicts with the
 *       LR1121 DIO1 interrupt pin (UULP_VBAT_GPIO_2).
 *
 * LED Modes (single LED with different patterns):
 *   - OFF:          LED off
 *   - DISCONNECTED: Slow blink (1Hz, 500ms on/off)
 *   - TENTATIVE:    Medium blink (2Hz, 250ms on/off)
 *   - CONNECTED:    Solid ON
 *   - BINDING:      Very fast blink (10Hz, 50ms on/off)
 *   - WIFI:         Double-blink pattern (blink-blink-pause)
 *   - ERROR:        Rapid triple-blink
 *
 * Register Configuration Citations:
 * - siw917x-family-rm.pdf Section 11.3 and 11.4.3 MCUHP_PAD_SELECTION (offset 0x610):
 *   Base address: 0x4130_0000 (GPIO Pad Control Selection)
 *   Bit 5 = NWP_MCUHP_GPIO_CTRL1_5 = GPIO_10 control
 *   Write '1' for MCU control
 *
 * - siw917x-family-rm.pdf Section 11.6.1 PAD_CONFIG_REG_x:
 *   Base address: 0x4600_4000
 *   Offset: 0x000 + (0x04 * pin_number)
 *   Bits 1:0 (E) = drive strength (01 = 4mA)
 *
 * - siw917x-family-rm.pdf Section 11.11 EGPIO Register Map:
 *   EGPIO base: 0x4613_0000
 *   GPIO_CONFIG_REG offset: 0x000 + (0x10 * pin)
 *   BIT_LOAD_REG offset: 0x004 + (0x10 * pin)
 */

#include "status_led.h"
#include <stdio.h>
#include <stdint.h>
#include "cmsis_os2.h"

/*******************************************************************************
 * Debug Output
 ******************************************************************************/
#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define LED_DBG(fmt, ...) DEBUGOUT("[STATUS_LED] " fmt, ##__VA_ARGS__)

/*******************************************************************************
 * Register Definitions - SoC GPIO (LED0 on GPIO_10)
 *
 * Citation: siw917x-family-rm.pdf Chapter 11 "GPIO"
 ******************************************************************************/

/**
 * EGPIO clocks are disabled at reset on SiWx917. The radio driver also enables
 * them later for DIO1, but the status LED is initialized before the radio.
 */
#define M4CLK_BASE                  0x46000000UL
#define CLK_ENABLE_SET_REG2         (*(volatile uint32_t *)(M4CLK_BASE + 0x008))
#define CLK_ENABLE_SET_REG3         (*(volatile uint32_t *)(M4CLK_BASE + 0x010))
#define EGPIO_PCLK_ENABLE_BIT       (1UL << 21)
#define EGPIO_CLK_ENABLE_BIT        (1UL << 16)

/**
 * SoC and ULP GPIO Pad Control Selection Register Base
 * Citation: siw917x-family-rm.pdf Section 11.3 "SoC and ULP GPIO Pad Control Selection Register Map"
 *   Base address: 0x4130_0000
 */
#define GPIO_PAD_CTRL_BASE          0x41300000UL

/**
 * MCUHP_PAD_SELECTION register - MCU/NWP control selection for SoC GPIO
 * Citation: siw917x-family-rm.pdf Section 11.4.3 "MCUHP_PAD_SELECTION"
 *   Offset: 0x610 from GPIO Pad Control base
 */
#define MCUHP_PAD_SELECTION         (*(volatile uint32_t *)(GPIO_PAD_CTRL_BASE + 0x610))

/**
 * Bit mask for GPIO_10 MCU control
 * Citation: siw917x-family-rm.pdf Section 11.4.3
 * Bit 5 = NWP_MCUHP_GPIO_CTRL1_5 = GPIO_10
 */
#define MCUHP_GPIO10_CTRL_BIT       (1UL << 5)

/**
 * Pad Configuration Register base for SoC GPIO
 * Citation: siw917x-family-rm.pdf Section 11.5
 */
#define PAD_CONFIG_REG_BASE         0x46004000UL

/**
 * Pad Configuration for GPIO_10
 * Citation: siw917x-family-rm.pdf Section 11.5
 * Offset: 0x000 + (0x04 * 10) = 0x028
 */
#define PAD_CONFIG_REG_10           (*(volatile uint32_t *)(PAD_CONFIG_REG_BASE + 0x028))

/**
 * Pad configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.6.1 "PAD_CONFIG_REG_x"
 */
#define PAD_E_4MA                   (1UL << 0)   /* 4mA drive strength */
#define PAD_SR_HIGH                 (1UL << 5)   /* High slew rate */

/**
 * EGPIO Register Base (SoC GPIO)
 * Citation: siw917x-family-rm.pdf Section 11.11
 */
#define EGPIO_BASE                  0x46130000UL

/**
 * GPIO Configuration Register for GPIO_10
 * Citation: siw917x-family-rm.pdf Section 11.11
 * Offset: 0x000 + (0x10 * 10) = 0x0A0
 */
#define GPIO_CONFIG_REG_10          (*(volatile uint32_t *)(EGPIO_BASE + 0x0A0))

/**
 * BIT_LOAD register for GPIO_10
 * Citation: siw917x-family-rm.pdf Section 11.11
 * Offset: 0x004 + (0x10 * 10) = 0x0A4
 */
#define BIT_LOAD_REG_10             (*(volatile uint32_t *)(EGPIO_BASE + 0x0A4))

/**
 * GPIO configuration bit definitions
 * Citation: siw917x-family-rm.pdf Section 11.12.9 "GPIO_CONFIG_REG_x"
 */
#define GPIO_MODE_MASK              0x3CUL       /* Bits 5:2 */
#define GPIO_MODE_GPIO              0x00UL       /* MODE = 0 for GPIO */
#define GPIO_DIRECTION_BIT          (1UL << 0)   /* 0=output, 1=input */

/*******************************************************************************
 * Blink Pattern Definitions (single LED patterns)
 ******************************************************************************/

/* Slow blink for disconnected: 500ms on, 500ms off (1Hz) */
#define BLINK_SLOW_MS               500

/* Medium blink for tentative: 250ms on, 250ms off (2Hz) */
#define BLINK_MEDIUM_MS             250

/* Fast blink for binding: 50ms on, 50ms off (10Hz) */
#define BLINK_FAST_MS               50

/* Double-blink pattern for WiFi mode */
#define WIFI_BLINK_ON_MS            100
#define WIFI_BLINK_OFF_MS           100
#define WIFI_PAUSE_MS               600

/* Rapid blink for error */
#define BLINK_RAPID_MS              75

/*******************************************************************************
 * Module State
 ******************************************************************************/

static status_led_mode_t current_mode = LED_MODE_OFF;
static uint32_t last_toggle_tick = 0;
static bool led_state = false;
static bool initialized = false;

/* State machine for complex patterns (WiFi double-blink) */
typedef enum {
    PATTERN_BLINK1_ON,
    PATTERN_BLINK1_OFF,
    PATTERN_BLINK2_ON,
    PATTERN_BLINK2_OFF,
    PATTERN_PAUSE
} pattern_state_t;

static pattern_state_t wifi_pattern_state = PATTERN_BLINK1_ON;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Write to LED0 (GPIO_10)
 */
static void write_led(bool on)
{
    /* Citation: siw917x-family-rm.pdf Section 11.12.4
     * BIT_LOAD_REG bit 0 = output value
     * LEDs are active-high per ug590-brd2708a-user-guide.pdf Section 3.4
     */
    BIT_LOAD_REG_10 = on ? 1UL : 0UL;
    led_state = on;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize the status LED
 */
int status_led_init(void)
{
    LED_DBG("Initializing status LED (LED0 only)...\n");
    LED_DBG("  NOTE: LED1 (ULP_GPIO_2) not used - conflicts with LR1121 DIO1\n");

    /* Enable EGPIO clocks before touching GPIO config/output registers. */
    CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;
    CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;
    LED_DBG("  EGPIO clocks enabled for LED0\n");
    
    /***************************************************************************
     * LED0 Configuration (GPIO_10)
     **************************************************************************/
    
    /* Step 1: Select MCU control for GPIO_10
     * Citation: siw917x-family-rm.pdf Section 11.4.3
     * Set bit 5 to '1' for MCU control of GPIO_10
     */
    MCUHP_PAD_SELECTION |= MCUHP_GPIO10_CTRL_BIT;
    LED_DBG("  MCUHP_PAD_SELECTION = 0x%08lX (GPIO_10 MCU control)\n",
            (unsigned long)MCUHP_PAD_SELECTION);
    
    /* Step 2: Configure pad for output with drive strength
     * Citation: siw917x-family-rm.pdf Section 11.6.1
     * - E = 01 (4mA drive strength - adequate for LED)
     * - SR = 1 (high slew rate for clean edges)
     */
    PAD_CONFIG_REG_10 = PAD_E_4MA | PAD_SR_HIGH;
    LED_DBG("  PAD_CONFIG_REG_10 = 0x%08lX (4mA, high slew)\n",
            (unsigned long)PAD_CONFIG_REG_10);
    
    /* Step 3: Configure GPIO for output
     * Citation: siw917x-family-rm.pdf Section 11.12.9
     * - MODE = 0 (GPIO mode)
     * - DIRECTION = 0 (output)
     */
    uint32_t cfg = GPIO_CONFIG_REG_10;
    cfg &= ~GPIO_MODE_MASK;          /* Clear MODE bits */
    cfg &= ~GPIO_DIRECTION_BIT;      /* Clear direction bit = output */
    GPIO_CONFIG_REG_10 = cfg;
    LED_DBG("  GPIO_CONFIG_REG_10 = 0x%08lX (GPIO mode, output)\n",
            (unsigned long)GPIO_CONFIG_REG_10);
    
    /***************************************************************************
     * Initialize LED state
     **************************************************************************/
    
    /* Turn LED off initially */
    write_led(false);
    
    current_mode = LED_MODE_OFF;
    last_toggle_tick = osKernelGetTickCount();
    initialized = true;
    
    LED_DBG("Status LED initialization complete\n");
    LED_DBG("  LED0: GPIO_10 (yellow) - active high\n");
    
    return 0;
}

/**
 * @brief Set the LED indication mode
 */
void status_led_set_mode(status_led_mode_t mode)
{
    if (!initialized) {
        return;
    }
    
    if (mode == current_mode) {
        return;  /* No change */
    }
    
    current_mode = mode;
    last_toggle_tick = osKernelGetTickCount();
    wifi_pattern_state = PATTERN_BLINK1_ON;  /* Reset pattern state */
    
    /* Set initial state for the new mode */
    switch (mode) {
        case LED_MODE_OFF:
            write_led(false);
            LED_DBG("Mode: OFF\n");
            break;
            
        case LED_MODE_DISCONNECTED:
            write_led(true);   /* Start ON for slow blink */
            LED_DBG("Mode: DISCONNECTED (slow blink 1Hz)\n");
            break;
            
        case LED_MODE_TENTATIVE:
            write_led(true);   /* Start ON for medium blink */
            LED_DBG("Mode: TENTATIVE (medium blink 2Hz)\n");
            break;
            
        case LED_MODE_CONNECTED:
            write_led(true);   /* Solid ON */
            LED_DBG("Mode: CONNECTED (solid ON)\n");
            break;
            
        case LED_MODE_BINDING:
            write_led(true);   /* Start fast blink */
            LED_DBG("Mode: BINDING (fast blink 10Hz)\n");
            break;
            
        case LED_MODE_WIFI:
            write_led(true);   /* Start double-blink pattern */
            LED_DBG("Mode: WIFI (double-blink pattern)\n");
            break;
            
        case LED_MODE_ERROR:
            write_led(true);   /* Rapid blink */
            LED_DBG("Mode: ERROR (rapid blink)\n");
            break;
            
        case LED_MODE_BOOT:
            write_led(false);
            LED_DBG("Mode: BOOT\n");
            break;
            
        default:
            break;
    }
}

/**
 * @brief Get the current LED mode
 */
status_led_mode_t status_led_get_mode(void)
{
    return current_mode;
}

/**
 * @brief Update LED state (call periodically from main loop)
 */
void status_led_update(void)
{
    if (!initialized) {
        return;
    }
    
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - last_toggle_tick;
    
    switch (current_mode) {
        case LED_MODE_OFF:
        case LED_MODE_CONNECTED:
            /* No blinking needed - static states */
            return;
            
        case LED_MODE_DISCONNECTED:
            /* Slow blink: 500ms on, 500ms off */
            if (elapsed >= BLINK_SLOW_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_TENTATIVE:
            /* Medium blink: 250ms on, 250ms off */
            if (elapsed >= BLINK_MEDIUM_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_BINDING:
            /* Fast blink: 50ms on, 50ms off */
            if (elapsed >= BLINK_FAST_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_WIFI:
            /* Double-blink pattern: blink-blink-pause */
            switch (wifi_pattern_state) {
                case PATTERN_BLINK1_ON:
                    if (elapsed >= WIFI_BLINK_ON_MS) {
                        last_toggle_tick = now;
                        write_led(false);
                        wifi_pattern_state = PATTERN_BLINK1_OFF;
                    }
                    break;
                case PATTERN_BLINK1_OFF:
                    if (elapsed >= WIFI_BLINK_OFF_MS) {
                        last_toggle_tick = now;
                        write_led(true);
                        wifi_pattern_state = PATTERN_BLINK2_ON;
                    }
                    break;
                case PATTERN_BLINK2_ON:
                    if (elapsed >= WIFI_BLINK_ON_MS) {
                        last_toggle_tick = now;
                        write_led(false);
                        wifi_pattern_state = PATTERN_BLINK2_OFF;
                    }
                    break;
                case PATTERN_BLINK2_OFF:
                    if (elapsed >= WIFI_PAUSE_MS) {
                        last_toggle_tick = now;
                        write_led(true);
                        wifi_pattern_state = PATTERN_BLINK1_ON;
                    }
                    break;
                default:
                    wifi_pattern_state = PATTERN_BLINK1_ON;
                    break;
            }
            break;
            
        case LED_MODE_ERROR:
            /* Rapid blink: 75ms on, 75ms off */
            if (elapsed >= BLINK_RAPID_MS) {
                last_toggle_tick = now;
                led_state = !led_state;
                write_led(led_state);
            }
            break;
            
        case LED_MODE_BOOT:
            /* Boot sequence handled separately */
            break;
            
        default:
            break;
    }
}

/**
 * @brief Directly control LED0
 */
void status_led0_set(bool on)
{
    if (!initialized) {
        return;
    }
    write_led(on);
}

/**
 * @brief LED1 control - DISABLED (pin used by DIO1 interrupt)
 */
void status_led1_set(bool on)
{
    (void)on;  /* Unused - LED1 conflicts with LR1121 DIO1 */
}

/**
 * @brief Toggle LED0
 */
void status_led0_toggle(void)
{
    if (!initialized) {
        return;
    }
    led_state = !led_state;
    write_led(led_state);
}

/**
 * @brief Toggle LED1 - DISABLED
 */
void status_led1_toggle(void)
{
    /* Unused - LED1 conflicts with LR1121 DIO1 */
}

/**
 * @brief Run boot LED animation (single LED version)
 */
void status_led_boot_sequence(void)
{
    if (!initialized) {
        return;
    }
    
    LED_DBG("Running boot sequence...\n");
    
    /* Flash LED0 3 times rapidly */
    for (int i = 0; i < 3; i++) {
        write_led(true);
        osDelay(100);
        write_led(false);
        osDelay(100);
    }
    
    LED_DBG("Boot sequence complete\n");
}
