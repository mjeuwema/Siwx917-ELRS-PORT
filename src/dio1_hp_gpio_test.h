/**
 * @file dio1_hp_gpio_test.h
 * @brief Test suite for DIO1 HP GPIO interrupt (GPIO_46)
 * 
 * Tests the new HP GPIO pin interrupt implementation for LR1121 DIO1.
 * Uses GPIO_46 (Port C, Pin 14) with rising-edge interrupt on channel 0.
 */

#ifndef DIO1_HP_GPIO_TEST_H
#define DIO1_HP_GPIO_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run complete HP GPIO DIO1 interrupt test suite
 * 
 * Tests:
 * 1. GPIO initialization and pin read
 * 2. Callback registration
 * 3. Interrupt enable/disable via NVIC
 * 4. LR1121 IRQ trigger (RX timeout) to verify DIO1 fires
 * 5. ISR count verification
 * 6. Multiple interrupt cycle test
 */
void dio1_hp_gpio_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* DIO1_HP_GPIO_TEST_H */
