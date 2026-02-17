/**
 * @file hardware.h
 * @brief Hardware pin definitions for SiW917 + LR1121 (Waveshare Core1121-HF)
 * 
 * Pin mapping for SiW917 DevKit with Waveshare Core1121-HF module:
 * 
 * GSPI (Radio SPI):
 *   GPIO_25 = SCK  (SPI Clock)
 *   GPIO_26 = MISO (SPI Data from LR1121)
 *   GPIO_27 = MOSI (SPI Data to LR1121)
 *   GPIO_28 = NSS  (Chip Select)
 *
 * LR1121 Control:
 *   GPIO_29 = BUSY (LR1121 busy indicator) 
 *   GPIO_6  = DIO1 (LR1121 interrupt) - HP GPIO for reliable interrupts
 *   GPIO_30 = NRESET (LR1121 reset)
 *
 * RF Switch (PE4259):
 *   DIO5 = Power enable (active high)
 *   DIO6 = TX/RX select (LOW=TX, HIGH=RX)
 */

#pragma once

#include "Arduino.h"

//=============================================================================
// SPI Pins - GSPI peripheral
//=============================================================================
#define GPIO_PIN_SCK    25
#define GPIO_PIN_MISO   26
#define GPIO_PIN_MOSI   27
#define GPIO_PIN_NSS    28

//=============================================================================
// LR1121 Control Pins
//=============================================================================
#define GPIO_PIN_BUSY   29   // BUSY pin (GPIO_29)
#define GPIO_PIN_DIO1   46   // DIO1 interrupt (GPIO_46, HP domain, breakout pad)
#define GPIO_PIN_RST    30   // Hardware reset (active low, GPIO_30)

//=============================================================================
// Secondary Radio (Gemini/Dual mode) - Not used
//=============================================================================
#define GPIO_PIN_NSS_2      UNDEF_PIN
#define GPIO_PIN_BUSY_2     UNDEF_PIN
#define GPIO_PIN_DIO1_2     UNDEF_PIN
#define GPIO_PIN_RST_2      UNDEF_PIN

//=============================================================================
// LED Pins
//=============================================================================
#define GPIO_PIN_LED        11    // SiW917 DevKit onboard LED
#define GPIO_PIN_LED_RED    UNDEF_PIN
#define GPIO_PIN_LED_GREEN  GPIO_PIN_LED
#define GPIO_PIN_LED_BLUE   UNDEF_PIN

//=============================================================================
// Button Pin
//=============================================================================
#define GPIO_PIN_BUTTON     11

//=============================================================================
// Serial Pins (CRSF output)
//=============================================================================
#define GPIO_PIN_RCSIGNAL_RX    UNDEF_PIN
#define GPIO_PIN_RCSIGNAL_TX    UNDEF_PIN

//=============================================================================
// PWM Outputs - Not used
//=============================================================================
#define GPIO_PIN_PWM_OUTPUTS_COUNT  0

//=============================================================================
// Antenna Switch - Not used (PE4259 controlled via DIO5/DIO6)
//=============================================================================
#define GPIO_PIN_ANT_CTRL       UNDEF_PIN
#define GPIO_PIN_ANT_CTRL_COMPL UNDEF_PIN

//=============================================================================
// Power Management
//=============================================================================
#define GPIO_PIN_PA_ENABLE      UNDEF_PIN
#define GPIO_PIN_RX_ENABLE      UNDEF_PIN
#define GPIO_PIN_TX_ENABLE      UNDEF_PIN
#define GPIO_PIN_TCXO_ENABLE    UNDEF_PIN

//=============================================================================
// Debug Settings
//=============================================================================
#define DEBUG_LOG 1
// #define DEBUG_LOG_VERBOSE 1

//=============================================================================
// Feature Flags
//=============================================================================
#define OPT_HAS_VTX_SPI         false
#define OPT_HAS_SERVO_OUTPUT    false
#define OPT_HAS_GSENSOR         false
#define OPT_HAS_THERMAL         false
#define OPT_USE_BACKPACK        false

//=============================================================================
// LR1121 Options
//=============================================================================
#define OPT_USE_HARDWARE_DCDC   false  // Use internal LDO, not external DC-DC
#define OPT_USE_SX1276_RFO_HF   false  // Use high-power PA, not low-power

//=============================================================================
// RF Switch Control (LR1121 DIO pins for external switch)
// Waveshare Core1121-HF uses PE4259 RF switch controlled by DIO5/DIO6
// If LR1121_RFSW_CTRL_COUNT is not 8, default switch settings are used
//=============================================================================
#define LR1121_RFSW_CTRL_COUNT  0  // Use default RF switch settings

//=============================================================================
// Helper Functions
//=============================================================================

// Single radio configuration (no Gemini mode)
static inline bool isDualRadio() { return false; }
