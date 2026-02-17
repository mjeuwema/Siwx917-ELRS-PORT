/***************************************************************************/ /**
 * @file sl_si91x_config_timer_config.h
 * @brief Config Timer configuration file for ELRS hw_timer
 * 
 * This configures the Configurable Timer (CT) for use as the ELRS hardware
 * timer, providing precise timing for packet synchronization.
 * 
 * The CT provides crystal-accurate timing (16MHz from PLL) vs the ULP Timer's
 * RC oscillator which has ±1-2% tolerance.
 ******************************************************************************/

#ifndef SL_SI91X_CONFIG_TIMER_CONFIG_H
#define SL_SI91X_CONFIG_TIMER_CONFIG_H

#include "sl_si91x_config_timer.h"

// <<< Use Configuration Wizard in Context Menu >>>

/******************************* CT Configuration **************************/
// <h> CT Configuration for ELRS hwTimer

/* Use 16-bit mode - we only need Counter 0 for ELRS */
#define SL_CT_MODE_32BIT_ENABLE_MACRO SL_COUNTER_16BIT

//  <e>Config Timer UC Configuration
//  <i> Enable: Use configuration from this file
//  <i> Default: 1
#define CONFIG_TIMER_UC 1

// Counter-0 Direction: Up Counter for ELRS timing
// We count up from 0 to match value, then interrupt fires
#define SL_COUNTER0_DIRECTION_MACRO SL_COUNTER0_UP

// Counter-1 Direction (not used for ELRS, but must be defined)
#define SL_COUNTER1_DIRECTION_MACRO SL_COUNTER1_UP

// Counter0: Enable Periodic mode - CRITICAL for ELRS tick/tock
// Timer auto-reloads after reaching match value
#define SL_COUNTER0_PERIODIC_ENABLE_MACRO 1

// Counter1: Not used for ELRS
#define SL_COUNTER1_PERIODIC_ENABLE_MACRO 0

// Counter0: Disable sync trigger - we use software trigger
#define SL_COUNTER0_SYNC_TRIGGER_ENABLE_MACRO 0

// Counter1: Disable sync trigger
#define SL_COUNTER1_SYNC_TRIGGER_ENABLE_MACRO 0

// </e>

// </h>
// <<< end of configuration section >>>

/* CT Configuration structure for ELRS timing */
sl_config_timer_config_t ct_configuration = {
  .is_counter_mode_32bit_enabled    = SL_CT_MODE_32BIT_ENABLE_MACRO,
  .is_counter0_soft_reset_enabled   = false,
  .is_counter0_periodic_enabled     = SL_COUNTER0_PERIODIC_ENABLE_MACRO,
  .is_counter0_trigger_enabled      = false,  /* We use software trigger */
  .is_counter0_sync_trigger_enabled = SL_COUNTER0_SYNC_TRIGGER_ENABLE_MACRO,
  .is_counter0_buffer_enabled       = true,   /* Enable buffer for glitch-free updates */
  .is_counter1_soft_reset_enabled   = false,
  .is_counter1_periodic_enabled     = SL_COUNTER1_PERIODIC_ENABLE_MACRO,
  .is_counter1_trigger_enabled      = false,
  .is_counter1_sync_trigger_enabled = SL_COUNTER1_SYNC_TRIGGER_ENABLE_MACRO,
  .is_counter1_buffer_enabled       = false,
  .counter0_direction               = SL_COUNTER0_DIRECTION_MACRO,
  .counter1_direction               = SL_COUNTER1_DIRECTION_MACRO,
};

/* 
 * Pin configuration - CT doesn't need GPIO pins for ELRS timing
 * (we only use the internal interrupt, not PWM outputs)
 * These are placeholder definitions to satisfy the SDK
 */

// <<< sl:start pin_tool >>>
// <sct signal=IN0,(OUT0),(OUT1)> SL_SCT
// $[SCT_SL_SCT]
#ifndef SL_SCT_PERIPHERAL                       
#define SL_SCT_PERIPHERAL                        SCT
#endif

// SCT IN0 on GPIO_25
#ifndef SL_SCT_IN0_PORT                         
#define SL_SCT_IN0_PORT                          HP
#endif
#ifndef SL_SCT_IN0_PIN                          
#define SL_SCT_IN0_PIN                           25
#endif
#ifndef SL_SCT_IN0_LOC                          
#define SL_SCT_IN0_LOC                           0
#endif

// SCT OUT0 on GPIO_29
#ifndef SL_SCT_OUT0_PORT                        
#define SL_SCT_OUT0_PORT                         HP
#endif
#ifndef SL_SCT_OUT0_PIN                         
#define SL_SCT_OUT0_PIN                          29
#endif
#ifndef SL_SCT_OUT0_LOC                         
#define SL_SCT_OUT0_LOC                          10
#endif

// SCT OUT1 on GPIO_30
#ifndef SL_SCT_OUT1_PORT                        
#define SL_SCT_OUT1_PORT                         HP
#endif
#ifndef SL_SCT_OUT1_PIN                         
#define SL_SCT_OUT1_PIN                          30
#endif
#ifndef SL_SCT_OUT1_LOC                         
#define SL_SCT_OUT1_LOC                          12
#endif
// [SCT_SL_SCT]$
// <<< sl:end pin_tool >>>

#endif /* SL_SI91X_CONFIG_TIMER_CONFIG_H */
