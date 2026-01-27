/**
 * @file busy_monitor_test.c
 * @brief LR1121 BUSY Pin Monitor Test
 *
 * This standalone test monitors the LR1121 BUSY pin (GPIO_29) to determine
 * if the LR1121 chip boots and shows any activity. This is a diagnostic test
 * to verify basic hardware connectivity before attempting SPI communication.
 *
 * Test Logic:
 * 1. Configure GPIO_29 (BUSY) as input with receiver enabled
 * 2. Sample BUSY pin continuously, logging any transitions
 * 3. Perform hardware reset and observe BUSY behavior
 *
 * Expected Behavior if LR1121 is Working:
 * - After reset: BUSY goes HIGH for ~230ms, then LOW
 * - Citation: LR1121 Datasheet Section 4.2.1 "Reset Timing"
 *
 * Expected Behavior if LR1121 is NOT Working:
 * - BUSY stays permanently LOW (no power/not connected)
 * - BUSY stays permanently HIGH (stuck in reset)
 *
 * Hardware Connections (BRD2708A mikroBUS socket):
 * - GPIO_29: BUSY (Input - Active High when busy)
 * - GPIO_30: RST  (Output - Active Low reset)
 *
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
 * Citation: siw917x-family-rm.pdf Section 11 "GPIO"
 */

#include "rsi_debug.h"
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * GPIO Pin Definitions
 ******************************************************************************/
#define LR1121_PIN_BUSY 29 /* BUSY signal (Active High when busy) */
#define LR1121_PIN_RST  30 /* Reset signal (Active Low) */

/*******************************************************************************
 * Register Definitions for Direct GPIO Access
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.3, p.98
 *   CLK_ENABLE_SET_REG2: Bit 21 = EGPIO_PCLK_ENABLE (APB clock)
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.5, p.102
 *   CLK_ENABLE_SET_REG3: Bit 16 = EGPIO_CLK_ENABLE (controller clock)
 ******************************************************************************/
#define M4CLK_BASE 0x46000000UL
#define CLK_ENABLE_SET_REG2  (*(volatile uint32_t *)(M4CLK_BASE + 0x008))
#define CLK_ENABLE_SET_REG3  (*(volatile uint32_t *)(M4CLK_BASE + 0x010))
#define EGPIO_PCLK_ENABLE_BIT  (1UL << 21)
#define EGPIO_CLK_ENABLE_BIT   (1UL << 16)

/* GPIO Access Control - Take MCU control from NWP
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.4.1, p.309
 */
#define GPIO_PAD_CTRL_BASE 0x41300000UL
#define MEM_GPIO_ACCESS_CTRL_SET (*(volatile uint32_t *)(GPIO_PAD_CTRL_BASE + 0x000))
#define NWP_MCUHP_GPIO_CTRL2_BIT (1UL << 5)

/* EGPIO Register Base
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.11, p.320
 */
#define EGPIO_BASE 0x46130000UL

/* Per-Pin GPIO_CONFIG_REG for direction control
 * Offset = 0x000 + (0x10 * pin_number)
 * Bit 0 = DIRECTION (0=output, 1=input)
 */
#define EGPIO_GPIO_CONFIG_REG(pin) (*(volatile uint32_t *)(EGPIO_BASE + (0x10 * (pin))))

/* Per-Pin BIT_LOAD_REG for individual pin read
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.2, p.323
 */
#define EGPIO_BIT_LOAD_REG(pin) (*(volatile uint32_t *)(EGPIO_BASE + 0x004 + (0x10 * (pin))))

/* PORT 1 registers for GPIO_25-30
 * Port 1 base offset = 0x1000 + (0x40 * 1) = 0x1040
 */
#define EGPIO_PORT1_BASE (EGPIO_BASE + 0x1040)
#define EGPIO_PORT1_SET_REG    (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x04))
#define EGPIO_PORT1_CLR_REG    (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x08))

/* Bit position in PORT 1 for each GPIO pin */
#define HP_GPIO_PORT1_BIT(pin) (1UL << ((pin) - 25 + 9))

/* PAD Configuration
 * Citation: siw917x-family-rm.pdf Section 11.6.1
 */
#define PAD_CONFIG_BASE 0x46004000UL
#define PAD_CONFIG_REG(pin) (*(volatile uint32_t *)(PAD_CONFIG_BASE + (4 * (pin))))
#define PADCONFIG_REN_BIT (1UL << 4) /* Receiver Enable */
#define PADCONFIG_SMT_BIT (1UL << 3) /* Schmitt Trigger */

/*******************************************************************************
 * GPIO Helper Macros
 ******************************************************************************/
#define HP_GPIO_SET_OUTPUT(pin) (EGPIO_GPIO_CONFIG_REG(pin) &= ~(1UL << 0))
#define HP_GPIO_SET_INPUT(pin)  (EGPIO_GPIO_CONFIG_REG(pin) |=  (1UL << 0))
#define HP_GPIO_SET_HIGH(pin)   (EGPIO_PORT1_SET_REG = HP_GPIO_PORT1_BIT(pin))
#define HP_GPIO_SET_LOW(pin)    (EGPIO_PORT1_CLR_REG = HP_GPIO_PORT1_BIT(pin))
#define HP_GPIO_READ(pin)       (EGPIO_BIT_LOAD_REG(pin) & 1)

/*******************************************************************************
 * Timing Helpers
 ******************************************************************************/

/**
 * @brief Simple delay in milliseconds (approximate)
 */
static void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    for (volatile uint32_t j = 0; j < 10000; j++) { }
  }
}

/**
 * @brief Get approximate millisecond timestamp (simple counter)
 */
static volatile uint32_t g_tick_counter = 0;

__attribute__((unused))
static uint32_t get_ms_tick(void) {
  return g_tick_counter;
}

static void tick_increment(uint32_t ms) {
  g_tick_counter += ms;
}

/*******************************************************************************
 * GPIO Initialization
 ******************************************************************************/

/**
 * @brief Initialize GPIO for BUSY pin monitoring
 *
 * Configures:
 * - GPIO_29 (BUSY) as input with receiver enabled
 * - GPIO_30 (RST) as output for reset control
 */
static void init_gpio_for_busy_test(void) {
  DEBUGOUT("\n=== Initializing GPIO for BUSY Pin Monitor ===\n");
  
  /* Step 1: Enable EGPIO clocks - CRITICAL!
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.3, p.98
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.5, p.102
   */
  DEBUGOUT("Enabling EGPIO clocks...\n");
  CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;  /* APB clock */
  CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;   /* Controller clock */
  for (volatile int i = 0; i < 1000; i++) { }
  
  /* Step 2: Take MCU control of GPIO_25-30 from NWP
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.4.1, p.309
   */
  DEBUGOUT("Taking MCU control of GPIO pins...\n");
  MEM_GPIO_ACCESS_CTRL_SET = NWP_MCUHP_GPIO_CTRL2_BIT;
  for (volatile int i = 0; i < 100; i++) { }
  
  /* Step 3: Enable receiver on BUSY pin (GPIO_29)
   * Citation: siw917x-family-rm.pdf Section 11.6.1
   * Bit 4 (REN): Receiver Enable - MUST be 1 for input
   * Bit 3 (SMT): Schmitt Trigger - improves noise immunity
   */
  DEBUGOUT("Configuring BUSY pin (GPIO_%d) PAD...\n", LR1121_PIN_BUSY);
  PAD_CONFIG_REG(LR1121_PIN_BUSY) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  for (volatile int i = 0; i < 100; i++) { }
  
  /* Step 4: Set BUSY (GPIO_29) to GPIO mode and INPUT direction */
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_BUSY) &= ~(0xF << 2);  /* GPIO mode */
  HP_GPIO_SET_INPUT(LR1121_PIN_BUSY);
  
  /* Step 5: Set RST (GPIO_30) to GPIO mode and OUTPUT direction */
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_RST) &= ~(0xF << 2);  /* GPIO mode */
  HP_GPIO_SET_OUTPUT(LR1121_PIN_RST);
  HP_GPIO_SET_HIGH(LR1121_PIN_RST);  /* Start with RST HIGH (not in reset) */
  
  DEBUGOUT("GPIO Configuration Complete:\n");
  DEBUGOUT("  GPIO_%d (BUSY): Input, PAD=0x%08lX\n", 
           LR1121_PIN_BUSY, (unsigned long)PAD_CONFIG_REG(LR1121_PIN_BUSY));
  DEBUGOUT("  GPIO_%d (RST):  Output, HIGH\n", LR1121_PIN_RST);
}

/*******************************************************************************
 * BUSY Pin Monitor Test Functions
 ******************************************************************************/

/**
 * @brief Continuous BUSY pin sampling for specified duration
 *
 * @param duration_ms How long to monitor (milliseconds)
 * @param sample_interval_ms Time between samples (milliseconds)
 * @return Number of times BUSY was observed HIGH
 *
 * @note This function is kept for potential manual testing - suppress unused warning
 */
__attribute__((unused))
static uint32_t monitor_busy_pin(uint32_t duration_ms, uint32_t sample_interval_ms) {
  uint32_t samples = duration_ms / sample_interval_ms;
  uint32_t high_count = 0;
  uint32_t low_count = 0;
  uint32_t transition_count = 0;
  int last_state = -1;  /* Unknown initial state */
  
  DEBUGOUT("\nMonitoring BUSY pin for %lu ms (sampling every %lu ms)...\n",
           (unsigned long)duration_ms, (unsigned long)sample_interval_ms);
  DEBUGOUT("Legend: H=HIGH, L=LOW, ^=LOW->HIGH, v=HIGH->LOW\n");
  DEBUGOUT("[");
  
  for (uint32_t i = 0; i < samples; i++) {
    int current_state = HP_GPIO_READ(LR1121_PIN_BUSY);
    
    if (current_state) {
      high_count++;
    } else {
      low_count++;
    }
    
    /* Detect and print transitions */
    if (last_state >= 0 && current_state != last_state) {
      transition_count++;
      if (current_state) {
        DEBUGOUT("^");  /* LOW to HIGH */
      } else {
        DEBUGOUT("v");  /* HIGH to LOW */
      }
    } else {
      /* Print current state */
      DEBUGOUT("%c", current_state ? 'H' : 'L');
    }
    
    /* Print progress every 20 samples */
    if ((i + 1) % 50 == 0) {
      DEBUGOUT("]\n[");
    }
    
    last_state = current_state;
    delay_ms(sample_interval_ms);
    tick_increment(sample_interval_ms);
  }
  
  DEBUGOUT("]\n\n");
  DEBUGOUT("Results:\n");
  DEBUGOUT("  Total samples:    %lu\n", (unsigned long)samples);
  DEBUGOUT("  HIGH samples:     %lu (%.1f%%)\n", 
           (unsigned long)high_count, (100.0f * high_count / samples));
  DEBUGOUT("  LOW samples:      %lu (%.1f%%)\n", 
           (unsigned long)low_count, (100.0f * low_count / samples));
  DEBUGOUT("  Transitions:      %lu\n", (unsigned long)transition_count);
  
  return high_count;
}

/**
 * @brief Perform hardware reset and monitor BUSY response
 *
 * Reset Sequence (Citation: LR1121 Datasheet Section 4.2.1):
 * 1. Drive RST LOW for >= 100µs (using 5ms for safety)
 * 2. Drive RST HIGH
 * 3. Monitor BUSY - should go HIGH then LOW within ~230ms
 */
static void reset_and_monitor_busy(void) {
  DEBUGOUT("\n=== Hardware Reset Test ===\n");
  DEBUGOUT("Citation: LR1121 Datasheet Section 4.2.1 'Reset Timing'\n");
  DEBUGOUT("Expected: BUSY goes HIGH for ~230ms after reset, then LOW\n\n");
  
  /* Read initial BUSY state */
  int initial_busy = HP_GPIO_READ(LR1121_PIN_BUSY);
  DEBUGOUT("Initial BUSY state: %s\n", initial_busy ? "HIGH" : "LOW");
  
  /* Assert RST (drive LOW) */
  DEBUGOUT("Asserting RST (LOW)...\n");
  HP_GPIO_SET_LOW(LR1121_PIN_RST);
  delay_ms(5);  /* 5ms reset pulse (min 100µs required) */
  tick_increment(5);
  
  /* Read BUSY during reset */
  int busy_during_reset = HP_GPIO_READ(LR1121_PIN_BUSY);
  DEBUGOUT("BUSY during reset: %s\n", busy_during_reset ? "HIGH" : "LOW");
  
  /* Release RST (drive HIGH) */
  DEBUGOUT("Releasing RST (HIGH) - monitoring BUSY for 500ms...\n");
  HP_GPIO_SET_HIGH(LR1121_PIN_RST);
  
  /* Monitor BUSY with fine granularity during boot */
  uint32_t busy_high_time_ms = 0;
  uint32_t first_high_at = 0;
  uint32_t went_low_at = 0;
  bool saw_high = false;
  bool saw_transition_to_low = false;
  
  DEBUGOUT("Time(ms) | BUSY | Event\n");
  DEBUGOUT("---------+------+------------------\n");
  
  for (uint32_t t = 0; t < 500; t += 5) {
    int busy = HP_GPIO_READ(LR1121_PIN_BUSY);
    
    if (busy) {
      busy_high_time_ms += 5;
      if (!saw_high) {
        saw_high = true;
        first_high_at = t;
        DEBUGOUT("  %5lu  | HIGH | *** BUSY went HIGH! ***\n", (unsigned long)t);
      }
    } else if (saw_high && !saw_transition_to_low) {
      saw_transition_to_low = true;
      went_low_at = t;
      DEBUGOUT("  %5lu  | LOW  | *** BUSY went LOW (ready) ***\n", (unsigned long)t);
    }
    
    /* Print state every 50ms */
    if (t % 50 == 0 && t > 0) {
      if (!saw_high && !busy) {
        DEBUGOUT("  %5lu  | LOW  | (still waiting for HIGH)\n", (unsigned long)t);
      } else if (saw_high && busy) {
        DEBUGOUT("  %5lu  | HIGH | (processing...)\n", (unsigned long)t);
      }
    }
    
    delay_ms(5);
    tick_increment(5);
  }
  
  /* Print summary */
  DEBUGOUT("\n=== Reset Test Summary ===\n");
  if (saw_high) {
    DEBUGOUT("*** LR1121 SHOWS ACTIVITY! ***\n");
    DEBUGOUT("  BUSY went HIGH at:       %lu ms after reset\n", (unsigned long)first_high_at);
    if (saw_transition_to_low) {
      DEBUGOUT("  BUSY went LOW (ready) at: %lu ms after reset\n", (unsigned long)went_low_at);
      DEBUGOUT("  Total boot time:         %lu ms\n", (unsigned long)(went_low_at - first_high_at));
      DEBUGOUT("  RESULT: LR1121 BOOTED SUCCESSFULLY!\n");
    } else {
      DEBUGOUT("  BUSY never went LOW - chip may be stuck\n");
      DEBUGOUT("  RESULT: LR1121 may be stuck in boot\n");
    }
  } else {
    DEBUGOUT("*** NO ACTIVITY DETECTED ***\n");
    DEBUGOUT("  BUSY never went HIGH after reset\n");
    DEBUGOUT("  Possible causes:\n");
    DEBUGOUT("    - LR1121 not powered (check VDDIO, VDDRF)\n");
    DEBUGOUT("    - RST pin not connected\n");
    DEBUGOUT("    - BUSY pin not connected\n");
    DEBUGOUT("    - LR1121 chip defective\n");
  }
}

/**
 * @brief Quick BUSY pin read test (no reset)
 *
 * Just reads the BUSY pin state multiple times to check basic GPIO functionality.
 */
static void quick_busy_read_test(void) {
  DEBUGOUT("\n=== Quick BUSY Pin Read Test ===\n");
  DEBUGOUT("Reading BUSY (GPIO_%d) 20 times with 50ms intervals...\n\n", LR1121_PIN_BUSY);
  
  uint32_t high_count = 0;
  
  DEBUGOUT("Sample: ");
  for (int i = 0; i < 20; i++) {
    int state = HP_GPIO_READ(LR1121_PIN_BUSY);
    DEBUGOUT("%d ", state);
    if (state) high_count++;
    delay_ms(50);
    tick_increment(50);
  }
  DEBUGOUT("\n\n");
  
  DEBUGOUT("Result: %lu HIGH, %lu LOW out of 20 samples\n", 
           (unsigned long)high_count, (unsigned long)(20 - high_count));
  
  if (high_count == 0) {
    DEBUGOUT("  -> BUSY is always LOW (chip may be idle, unpowered, or disconnected)\n");
  } else if (high_count == 20) {
    DEBUGOUT("  -> BUSY is always HIGH (chip may be stuck or processing)\n");
  } else {
    DEBUGOUT("  -> BUSY is transitioning - LR1121 appears active!\n");
  }
}

/*******************************************************************************
 * Boot Loop Detection Test
 * Detects if the LR1121 is repeatedly resetting by measuring BUSY HIGH durations
 * Citation: LR1121 Datasheet Section 4.2.1 - BUSY HIGH ~230ms after reset
 ******************************************************************************/
static void boot_loop_detection_test(void)
{
  DEBUGOUT("\r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("           BOOT LOOP DETECTION TEST                                   \r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("\r\n");
  DEBUGOUT("Citation: LR1121 Datasheet Section 4.2.1 'Reset Timing'\r\n");
  DEBUGOUT("  - Normal boot: BUSY HIGH for ~230ms after reset, then stays LOW\r\n");
  DEBUGOUT("  - Boot loop: Repeated BUSY HIGH pulses of ~230ms each\r\n");
  DEBUGOUT("\r\n");
  DEBUGOUT("Monitoring for 10 seconds with 10ms resolution...\r\n");
  DEBUGOUT("Will detect and measure each BUSY HIGH pulse duration.\r\n");
  DEBUGOUT("\r\n");
  
  #define BOOT_TEST_DURATION_MS  10000
  #define BOOT_TEST_SAMPLE_MS    10
  #define MAX_PULSES             20
  
  uint32_t pulse_durations[MAX_PULSES];
  uint32_t pulse_count = 0;
  uint32_t current_pulse_duration = 0;
  int last_state = HP_GPIO_READ(LR1121_PIN_BUSY);
  int in_pulse = last_state;  /* If starting HIGH, we're in a pulse */
  
  uint32_t total_high_time = 0;
  uint32_t total_low_time = 0;
  uint32_t longest_low_streak = 0;
  uint32_t current_low_streak = 0;
  
  DEBUGOUT("Initial BUSY state: %s\r\n", last_state ? "HIGH" : "LOW");
  DEBUGOUT("\r\n");
  DEBUGOUT("Time(ms) | Event\r\n");
  DEBUGOUT("---------+--------------------------------------------------\r\n");
  
  for (uint32_t elapsed = 0; elapsed < BOOT_TEST_DURATION_MS; elapsed += BOOT_TEST_SAMPLE_MS) {
    int current = HP_GPIO_READ(LR1121_PIN_BUSY);
    
    if (current) {
      total_high_time += BOOT_TEST_SAMPLE_MS;
      current_low_streak = 0;
      
      if (!last_state) {
        /* Rising edge - start of new pulse */
        in_pulse = 1;
        current_pulse_duration = BOOT_TEST_SAMPLE_MS;
        DEBUGOUT("%-8lu | >> BUSY went HIGH (pulse #%lu starting)\r\n", 
                 (unsigned long)elapsed, (unsigned long)(pulse_count + 1));
      } else if (in_pulse) {
        current_pulse_duration += BOOT_TEST_SAMPLE_MS;
      }
    } else {
      total_low_time += BOOT_TEST_SAMPLE_MS;
      current_low_streak += BOOT_TEST_SAMPLE_MS;
      if (current_low_streak > longest_low_streak) {
        longest_low_streak = current_low_streak;
      }
      
      if (last_state && in_pulse) {
        /* Falling edge - end of pulse */
        if (pulse_count < MAX_PULSES) {
          pulse_durations[pulse_count] = current_pulse_duration;
          DEBUGOUT("%-8lu | << BUSY went LOW (pulse #%lu duration: %lu ms)\r\n", 
                   (unsigned long)elapsed, (unsigned long)(pulse_count + 1), 
                   (unsigned long)current_pulse_duration);
          
          /* Check if this looks like a boot pulse */
          if (current_pulse_duration >= 180 && current_pulse_duration <= 300) {
            DEBUGOUT("         |    ^^^ This matches expected boot duration (~230ms)!\r\n");
          } else if (current_pulse_duration > 300) {
            DEBUGOUT("         |    ^^^ LONGER than expected boot time\r\n");
          } else {
            DEBUGOUT("         |    ^^^ SHORTER than expected boot time\r\n");
          }
          
          pulse_count++;
        }
        in_pulse = 0;
        current_pulse_duration = 0;
      }
    }
    
    last_state = current;
    delay_ms(BOOT_TEST_SAMPLE_MS);
    tick_increment(BOOT_TEST_SAMPLE_MS);
  }
  
  /* Handle case where we ended while still in a pulse */
  if (in_pulse && pulse_count < MAX_PULSES) {
    pulse_durations[pulse_count] = current_pulse_duration;
    DEBUGOUT("%-8lu | (test ended while BUSY HIGH, pulse #%lu duration: %lu+ ms)\r\n",
             (unsigned long)BOOT_TEST_DURATION_MS, (unsigned long)(pulse_count + 1), 
             (unsigned long)current_pulse_duration);
    pulse_count++;
  }
  
  DEBUGOUT("\r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("           BOOT LOOP ANALYSIS RESULTS                                 \r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("\r\n");
  DEBUGOUT("Timing Summary:\r\n");
  DEBUGOUT("  Total HIGH time: %lu ms (%.1f%%)\r\n", 
           (unsigned long)total_high_time, (total_high_time * 100.0f) / BOOT_TEST_DURATION_MS);
  DEBUGOUT("  Total LOW time:  %lu ms (%.1f%%)\r\n", 
           (unsigned long)total_low_time, (total_low_time * 100.0f) / BOOT_TEST_DURATION_MS);
  DEBUGOUT("  Longest LOW streak: %lu ms\r\n", (unsigned long)longest_low_streak);
  DEBUGOUT("\r\n");
  DEBUGOUT("Pulse Analysis:\r\n");
  DEBUGOUT("  Total BUSY HIGH pulses detected: %lu\r\n", (unsigned long)pulse_count);
  
  if (pulse_count > 0) {
    DEBUGOUT("  Pulse durations: ");
    uint32_t sum = 0;
    for (uint32_t i = 0; i < pulse_count; i++) {
      DEBUGOUT("%lu", (unsigned long)pulse_durations[i]);
      if (i < pulse_count - 1) DEBUGOUT(", ");
      sum += pulse_durations[i];
    }
    DEBUGOUT(" ms\r\n");
    DEBUGOUT("  Average pulse duration: %lu ms\r\n", (unsigned long)(sum / pulse_count));
  }
  
  DEBUGOUT("\r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("           DIAGNOSIS                                                  \r\n");
  DEBUGOUT("======================================================================\r\n");
  DEBUGOUT("\r\n");
  
  if (pulse_count == 0 && total_high_time == 0) {
    DEBUGOUT("[X] BUSY never went HIGH\r\n");
    DEBUGOUT("    -> LR1121 is NOT booting or BUSY pin not connected\r\n");
    DEBUGOUT("    -> Check power supply and pin connections\r\n");
  } else if (pulse_count == 0 && total_high_time == BOOT_TEST_DURATION_MS) {
    DEBUGOUT("[X] BUSY stuck HIGH for entire test\r\n");
    DEBUGOUT("    -> LR1121 may be stuck in reset or not receiving commands\r\n");
    DEBUGOUT("    -> Check: RST pin, crystal/oscillator, power supply stability\r\n");
  } else if (pulse_count == 1 && longest_low_streak > 5000) {
    DEBUGOUT("[OK] SINGLE boot pulse detected, chip appears STABLE\r\n");
    DEBUGOUT("    -> LR1121 booted successfully and is idle\r\n");
    DEBUGOUT("    -> Ready for SPI communication\r\n");
  } else if (pulse_count >= 2) {
    /* Check if pulses are ~230ms (boot pulses) */
    uint32_t boot_like_pulses = 0;
    for (uint32_t i = 0; i < pulse_count; i++) {
      if (pulse_durations[i] >= 150 && pulse_durations[i] <= 350) {
        boot_like_pulses++;
      }
    }
    
    if (boot_like_pulses >= 2) {
      DEBUGOUT("[!!] BOOT LOOP DETECTED!\r\n");
      DEBUGOUT("    -> %lu pulses match boot timing (~230ms)\r\n", (unsigned long)boot_like_pulses);
      DEBUGOUT("    -> LR1121 is repeatedly resetting!\r\n");
      DEBUGOUT("\r\n");
      DEBUGOUT("    Possible causes:\r\n");
      DEBUGOUT("    1. Power supply unstable (brown-out resets)\r\n");
      DEBUGOUT("    2. Watchdog triggering (if enabled in NVM)\r\n");
      DEBUGOUT("    3. Crystal/oscillator not starting reliably\r\n");
      DEBUGOUT("    4. Corrupted NVM configuration\r\n");
      DEBUGOUT("\r\n");
      DEBUGOUT("    Recommended actions:\r\n");
      DEBUGOUT("    1. Check power supply with oscilloscope for dips\r\n");
      DEBUGOUT("    2. Add capacitors (100nF + 10uF) close to LR1121\r\n");
      DEBUGOUT("    3. Verify crystal connections and load capacitors\r\n");
    } else {
      DEBUGOUT("[!] IRREGULAR BUSY activity detected\r\n");
      DEBUGOUT("    -> %lu pulses detected but timing doesn't match boot\r\n", (unsigned long)pulse_count);
      DEBUGOUT("    -> Chip may be responding to noise or commands\r\n");
      DEBUGOUT("    -> Check for floating pins or EMI issues\r\n");
    }
  } else {
    DEBUGOUT("[!] INTERMITTENT BUSY activity\r\n");
    DEBUGOUT("    -> BUSY is toggling but pattern is unclear\r\n");
    DEBUGOUT("    -> Longest stable LOW period: %lu ms\r\n", (unsigned long)longest_low_streak);
    if (longest_low_streak < 1000) {
      DEBUGOUT("    -> Chip is NOT staying idle - possible instability\r\n");
    }
  }
  
  DEBUGOUT("\r\n");
}

/*******************************************************************************
 * Main Test Entry Point
 ******************************************************************************/

/**
 * @brief Run the complete BUSY pin monitor test suite
 *
 * Called from gspi_example_init() when BUSY monitor test is enabled.
 */
void busy_monitor_test_run(void) {
  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║        LR1121 BUSY PIN MONITOR TEST                          ║\n");
  DEBUGOUT("║        Testing if LR1121 boots and shows activity            ║\n");
  DEBUGOUT("╠══════════════════════════════════════════════════════════════╣\n");
  DEBUGOUT("║  Hardware: BRD2708A + LR1121 on mikroBUS socket              ║\n");
  DEBUGOUT("║  BUSY Pin: GPIO_29                                           ║\n");
  DEBUGOUT("║  RST Pin:  GPIO_30                                           ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
  
  /* Initialize GPIO */
  init_gpio_for_busy_test();
  
  /* Test 1: Quick BUSY read test (no reset) */
  quick_busy_read_test();
  
  /* Test 2: Reset and monitor BUSY response */
  reset_and_monitor_busy();
  
  /* Test 3: Boot loop detection (10 seconds) */
  boot_loop_detection_test();
  
  /* Final summary */
  DEBUGOUT("\n");
  DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
  DEBUGOUT("║                    TEST COMPLETE                             ║\n");
  DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
  DEBUGOUT("\n");
  DEBUGOUT("If BUSY never went HIGH, check:\n");
  DEBUGOUT("  1. Is the LR1121 module inserted correctly?\n");
  DEBUGOUT("  2. Is power supplied to VDDIO (1.8V-3.6V) and VDDRF?\n");
  DEBUGOUT("  3. Are the RST and BUSY pins connected correctly?\n");
  DEBUGOUT("  4. Try measuring voltages with a multimeter\n");
  DEBUGOUT("\n");
  DEBUGOUT("If BUSY went HIGH but never LOW:\n");
  DEBUGOUT("  1. The LR1121 may be stuck - try longer timeout\n");
  DEBUGOUT("  2. Check for proper crystal/oscillator connection\n");
  DEBUGOUT("\n");
  DEBUGOUT("If BUSY transitions HIGH then LOW after reset:\n");
  DEBUGOUT("  -> LR1121 is booting correctly! Proceed with SPI communication.\n");
}
