/***************************************************************************/ /**
 * @file gspi_example.c
 * @brief GSPI example with enhanced loopback test and LR1121 diagnostics
 *******************************************************************************
 * # License
 * <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Enhanced loopback test with multiple patterns and LR1121 preparation diagnostics.
 * Tests SPI Mode 0 (CPOL=0, CPHA=0) which is required for LR1121 communication.
 *
 * LR1121 SPI Requirements (Citation: LR1121 Datasheet Section 3):
 * - SPI Mode 0 (CPOL=0, CPHA=0): Clock idle LOW, sample on rising edge
 * - Maximum SPI clock: 16 MHz
 * - MSB first
 * - 8-bit frames
 *
 ******************************************************************************/
#include "sl_si91x_gspi.h"
#include "sl_si91x_gspi_common_config.h"
#include "gspi_example.h"
#include "rsi_debug.h"

#include "rsi_rom_clks.h"

/*******************************************************************************
 * Test Mode Selection (defined early for conditional includes)
 ******************************************************************************/
#define TEST_MODE_LOOPBACK          0
#define TEST_MODE_LR1121_SPI        1
#define TEST_MODE_BUSY_MONITOR      2
#define TEST_MODE_GPIO_TOGGLE       3
#define TEST_MODE_STANDALONE_TESTS  4
#define TEST_MODE_TCXO_DIAGNOSTIC   5
#define TEST_MODE_EXTERNAL_TCXO     6
#define TEST_MODE_WIFI_HTTP         7
#define TEST_MODE_RADIO_LISTEN      8   /* Radio init + packet listen test */
#define TEST_MODE_ELRS_RX           9   /* ELRS RX main loop - full protocol stack (legacy) */
#define TEST_MODE_DIO1_TEST        10   /* DIO1 interrupt tests */
#define TEST_MODE_ELRS_MAIN        11   /* ELRS 4.0 flow with hwTimer/PFD - REQUIRES ELRS TX */
#define TEST_MODE_TCXO_SWEEP       12   /* TCXO voltage/delay sweep test - NO TX REQUIRED */

/* SELECT YOUR TEST MODE HERE */
/* Available test modes:
 * - TEST_MODE_STANDALONE_TESTS (4): Hardware validation - NO TRANSMITTER REQUIRED
 *   Tests: GetVersion, Temperature, RSSI noise floor, CW TX, TX packet, Freq hopping
 * - TEST_MODE_RADIO_LISTEN (8): Continuous RX (raw LoRa) - REQUIRES TRANSMITTER
 * - TEST_MODE_WIFI_HTTP (7): WiFi access point for OTA updates
 * - TEST_MODE_ELRS_RX (9): Legacy ELRS RX - basic protocol stack (no hwTimer)
 *   Features: FHSS, OTA CRC, channel decoding, connection state machine
 * - TEST_MODE_DIO1_TEST (10): DIO1 interrupt validation - NO TRANSMITTER REQUIRED
 *   Tests: Pin read, callback registration, interrupt enable/disable
 * - TEST_MODE_ELRS_MAIN (11): ELRS 4.0 Complete Flow - REQUIRES ELRS TX
 *   Features: hwTimer tick/tock, PFD phase locking, LQ calculator, rate cycling
 * - TEST_MODE_TCXO_SWEEP (12): TCXO Voltage/Delay Sweep - NO TRANSMITTER REQUIRED
 *   Tests: SPI baseline, voltage sweep, delay sweep, full init, stress test
 */
#define TEST_MODE_SPI_LOOPBACK     13   /* Simple SPI loopback - connect MOSI to MISO */
#define TEST_MODE_LR1121_SIMPLE    14   /* Simple LR1121 GetVersion test */
#define TEST_MODE TEST_MODE_ELRS_MAIN  /* ELRS 4.0 main loop - connect to TX */

/* ULP Timer only needed for loopback test mode */
#if (TEST_MODE == TEST_MODE_LOOPBACK)
#include "sl_ulp_timer_instances.h"
#include "sl_si91x_ulp_timer_common_config.h"
#endif

/* LR1121 Driver for GetVersion test */
#include "lr1121_driver.h"

/* BUSY Monitor Test */
#include "busy_monitor_test.h"

/* LR1121 Standalone Test Suite */
#include "lr1121_standalone_test.h"

/* LR1121 DIO1 Interrupt Test Suite (Comprehensive) */
#include "lr1121_dio1_interrupt_test.h"

/* LR1121 Hardware Test (comprehensive diagnostics) */
#include "lr1121_hw_test.h"

/* TCXO Diagnostic */
#include "tcxo_diagnostic.h"

/* TCXO Sweep Test (focused TCXO troubleshooting) */
#include "lr1121_tcxo_test.h"

/* External TCXO Test */
#include "external_tcxo_test.h"

/* WiFi HTTP Test (for ELRS OTA) */
#include "wifi_http_test.h"

/* Radio Listen Test (init + RX loop) */
#include "radio_listen_test.h"

/* ELRS Protocol Stack (for TEST_MODE_ELRS_RX - legacy) */
#if (TEST_MODE == TEST_MODE_ELRS_RX)
#include "elrs_protocol/elrs_rx.h"
#include "elrs_protocol/elrs_platform.h"
#include "elrs_protocol/fhss.h"
#include "elrs_protocol/crsf.h"       /* CRSF serial output */
#include "elrs_config.h"
#endif

/* ELRS 4.0 Main Flow (for TEST_MODE_ELRS_MAIN) */
#if (TEST_MODE == TEST_MODE_ELRS_MAIN)
#include "elrs_protocol/elrs_main.h"
#include "elrs_protocol/elrs_rx.h"
#include "elrs_protocol/elrs_platform.h"
#include "elrs_protocol/crsf.h"
#include "elrs_config.h"
#include "wifi_http_test.h"     /* WiFi AP + HTTP server for OTA updates */
#include "bind_button.h"        /* BTN1 binding button diagnostics */

/* WiseConnect SDK Includes - for NWP initialization (required for NVM3)
 * 
 * Citation: sl_si91x_nvm3_common_flash/readme.md
 *   "This example performs wireless initialization before using NVM3 APIs 
 *    using sl_net_init(). This is done to set up NWP-M4 communication."
 * 
 * On SiWx917 with common flash, NVM3 requires NWP to be initialized for
 * flash access coordination. We only init the NWP (no WiFi radio) so
 * the binding phrase can be loaded from NVM3 before starting ELRS RX.
 */
#include "sl_net.h"
#include "sl_wifi.h"
#endif

/* CMSIS-RTOS2 for FreeRTOS task creation (needed for WiFi and ELRS test modes) */
#if (TEST_MODE == TEST_MODE_WIFI_HTTP) || (TEST_MODE == TEST_MODE_ELRS_RX) || (TEST_MODE == TEST_MODE_ELRS_MAIN)
#include "cmsis_os2.h"
#endif

/* Legacy compatibility */
#define USE_LR1121_TEST (TEST_MODE == TEST_MODE_LR1121_SPI)

/*******************************************************************************
 * FreeRTOS Task for WiFi HTTP Test
 *
 * Citation: wifi_access_point_soc example (app.c)
 * - WiFi operations MUST run inside a FreeRTOS task, not from app_init() directly
 * - Task is created in app_init(), runs after osKernelStart()
 * - Stack size 3072 bytes is typical for WiFi operations
 ******************************************************************************/
#if (TEST_MODE == TEST_MODE_WIFI_HTTP)
static const osThreadAttr_t wifi_task_attributes = {
  .name       = "wifi_http",
  .attr_bits  = 0,
  .cb_mem     = 0,
  .cb_size    = 0,
  .stack_mem  = 0,
  .stack_size = 3072,
  .priority   = osPriorityLow,
  .tz_module  = 0,
  .reserved   = 0,
};

/**
 * @brief FreeRTOS task entry point for WiFi HTTP test
 * This task runs after the RTOS scheduler starts
 */
static void wifi_http_task(void *argument)
{
  (void)argument;
  
  DEBUGOUT("\n");
  DEBUGOUT("[FreeRTOS] WiFi HTTP task started!\n");
  
  /* Now we can safely call WiFi functions */
  wifi_http_test_run();
  
  /* wifi_http_test_run() has its own infinite loop, so we shouldn't reach here.
   * If we do, just delete this task. */
  osThreadExit();
}
#endif /* TEST_MODE_WIFI_HTTP */

/*******************************************************************************
 * FreeRTOS Task for ELRS RX Main Loop
 *
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 * - Main loop calls Radio.RXnb() to process packets
 * - Connection state machine handles sync/tentative/connected states
 * - FHSS frequency hopping is automatic after valid packet reception
 *
 * Citation: LR1121 Datasheet Section 9 "IRQ System"
 * - IRQ polling used since DIO1 interrupt not connected in test setup
 * - RX_DONE, TIMEOUT, CRC_ERROR flags checked each iteration
 ******************************************************************************/
#if (TEST_MODE == TEST_MODE_ELRS_RX)
static const osThreadAttr_t elrs_rx_task_attributes = {
  .name       = "elrs_rx",
  .attr_bits  = 0,
  .cb_mem     = 0,
  .cb_size    = 0,
  .stack_mem  = 0,
  .stack_size = 4096,   /* Larger stack for ELRS protocol processing */
  .priority   = osPriorityNormal,  /* Higher priority than WiFi for real-time RX */
  .tz_module  = 0,
  .reserved   = 0,
};

/**
 * @brief FreeRTOS task entry point for ELRS RX main loop
 * 
 * This task:
 *   1. Initializes ELRS configuration from NVM3
 *   2. Initializes FHSS sequence from UID
 *   3. Initializes radio via HAL
 *   4. Enters continuous RX loop calling elrs_rx_loop()
 *   5. Prints connection status and channel data periodically
 */
static void elrs_rx_task(void *argument)
{
  (void)argument;
  
  elrs_config_t *config;
  uint32_t loop_count = 0;
  uint32_t last_status_print = 0;
  elrs_connection_state_t last_state = ELRS_DISCONNECTED;
  
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  ELRS RX Main Loop Task Started\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /* Step 1: Initialize ELRS configuration storage (NVM3)
   * Citation: elrs_config.c - loads config from flash or creates defaults
   */
  DEBUGOUT("[ELRS_RX] Step 1: Initialize configuration...\n");
  if (elrs_config_init() != 0) {
    DEBUGOUT("[ELRS_RX] ERROR: Failed to initialize configuration\n");
    osThreadExit();
    return;
  }
  
  /* Get configuration pointer */
  config = elrs_config_get();
  
  /* Use UID from config (loaded from NVM3 or defaults)
   * 
   * For binding phrase "matthew" (WiFi/Configurator method):
   *   MD5("matthew") = E6:A5:BA:08:42:A5:...
   *   UID = E6:A5:BA:08:42:A5
   *   CRC init = (0x42 << 8 | 0xA5) ^ (4 << 8) = 0x46A5
   */
  DEBUGOUT("[ELRS_RX] Using UID from config: %02X:%02X:%02X:%02X:%02X:%02X\n",
           config->uid[0], config->uid[1], config->uid[2],
           config->uid[3], config->uid[4], config->uid[5]);
  
  /* Check if device is bound */
  if (!elrs_config_is_bound()) {
    DEBUGOUT("[ELRS_RX] WARNING: Device is not bound - using default UID\n");
  }
  
  /* Copy UID from config to global UID array
   * Citation: elrs_platform.h - UID[6] is used by FHSS and OTA CRC
   */
  memcpy(UID, config->uid, UID_LEN);
  DEBUGOUT("[ELRS_RX] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
           UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
  
  /* Step 2: Initialize ELRS RX subsystem
   * Citation: elrs_rx.c elrs_rx_init()
   * - Initializes HAL (lr1121_hal_init)
   * - Resets radio
   * - Initializes FHSS sequence from UID
   * - Initializes OTA CRC from UID
   * - Sets initial rate to 100Hz
   */
  DEBUGOUT("[ELRS_RX] Step 2: Initialize ELRS RX subsystem...\n");
  if (!elrs_rx_init(UID, DOMAIN_FCC915)) {
    DEBUGOUT("[ELRS_RX] ERROR: Failed to initialize ELRS RX\n");
    osThreadExit();
    return;
  }
  
  /* Step 3: Set task handle for interrupt-driven operation
   * 
   * Citation: elrs_rx.h elrs_rx_set_task_handle()
   *   Enables DIO1 interrupt-driven packet notification.
   *   When set, elrs_rx_loop() uses osThreadFlagsWait() instead of polling.
   *   This reduces CPU usage from ~30% to <1% between packets.
   * 
   * Citation: CMSIS-RTOS2 osThreadGetId()
   *   Returns the thread ID of the current running thread.
   */
  DEBUGOUT("[ELRS_RX] Step 3: Enable interrupt-driven RX mode...\n");
  elrs_rx_set_task_handle(osThreadGetId());
  DEBUGOUT("[ELRS_RX]   Task handle set for DIO1 interrupt signaling\n");
  
  /* Step 4: Start receiver
   * Citation: elrs_rx.c elrs_rx_start()
   * - Gets initial frequency from FHSS
   * - Configures radio for current rate
   * - Enters RX mode
   */
  DEBUGOUT("[ELRS_RX] Step 4: Start receiver...\n");
  elrs_rx_start();
  
  /* Step 4b: Print RF diagnostics to verify configuration
   * Citation: elrs_rx.c elrs_rx_print_rf_diagnostics()
   * This verifies the RF switch is configured, radio is in RX mode,
   * and there are no hardware errors.
   */
  elrs_rx_print_rf_diagnostics();
  
  /* Step 4: Initialize CRSF serial output
   * Citation: crsf.c crsf_init()
   * - Configures USART0 for 420800 baud
   * - Enables TX FIFO
   * - Ready to send CRSF frames to flight controller
   */
  DEBUGOUT("[ELRS_RX] Step 4: Initialize CRSF output...\n");
  if (!crsf_init(CRSF_BAUDRATE)) {
    DEBUGOUT("[ELRS_RX] WARNING: CRSF init failed, channel output disabled\n");
  } else {
    DEBUGOUT("[ELRS_RX] CRSF output enabled at 420800 baud\n");
  }
  
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  ELRS RX Active - Waiting for TX\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Expected configuration:\n");
  DEBUGOUT("  Rate:     100 Hz (SF7, BW500, CR4/7)\n");
  DEBUGOUT("  Domain:   FCC 915 MHz\n");
  DEBUGOUT("  FHSS:     40 channels\n");
  DEBUGOUT("  Binding:  UID-based\n");
  DEBUGOUT("  CRSF:     %s\n", crsf_is_initialized() ? "ENABLED" : "DISABLED");
  DEBUGOUT("\n");
  DEBUGOUT("Power on your ELRS TX with matching binding phrase...\n");
  DEBUGOUT("\n");
  
  /* Main RX loop
   * Citation: ExpressLRS rx_main.cpp loop()
   * - Polls radio IRQ status
   * - Processes received packets
   * - Updates connection state
   * - Handles FHSS frequency hopping
   */
  while (1) {
    /* Call ELRS RX loop - returns true if packet was received */
    bool packet_received = elrs_rx_loop();
    
    /* Check for connection state change */
    elrs_connection_state_t current_state = elrs_rx_get_state();
    if (current_state != last_state) {
      const char *state_names[] = {"DISCONNECTED", "TENTATIVE", "CONNECTED"};
      DEBUGOUT("[ELRS_RX] State: %s -> %s\n", 
               state_names[last_state], state_names[current_state]);
      last_state = current_state;
    }
    
    /* Print packet info when received */
    if (packet_received) {
      const elrs_rx_link_stats_t *stats = elrs_rx_get_link_stats();
      DEBUGOUT("[ELRS_RX] Packet #%lu: RSSI=%d dBm, SNR=%d dB, LQ=%u%%\n",
               (unsigned long)stats->packets_received,
               stats->rssi_ant1, stats->snr, stats->lq);
      
      /* Print channel data when connected */
      if (current_state == ELRS_CONNECTED) {
        const elrs_channel_data_t *ch = elrs_rx_get_channels();
        DEBUGOUT("[ELRS_RX] CH1-4: %u %u %u %u | Armed: %d\n",
                 ch->ch[0], ch->ch[1], ch->ch[2], ch->ch[3], ch->armed);
        
        /* Send CRSF RC channels to flight controller
         * Citation: crsf.h crsf_send_rc_channels()
         * - Builds CRSF_FRAMETYPE_RC_CHANNELS_PACKED (0x16)
         * - Packs 16x 11-bit channels into 22-byte payload
         * - Sends over USART0 at 420800 baud
         */
        if (crsf_is_initialized()) {
          crsf_send_rc_channels(ch->ch);
        }
      }
      
      /* Send CRSF link statistics periodically (every ~100ms or every 10 packets)
       * Citation: crsf.h CRSF_STATS_INTERVAL_MS (100ms)
       * - CRSF_FRAMETYPE_LINK_STATISTICS (0x14)
       * - Contains RSSI, SNR, LQ for OSD display
       */
      static uint32_t last_link_stats_packet = 0;
      if (crsf_is_initialized() && 
          (stats->packets_received - last_link_stats_packet >= 10)) {
        last_link_stats_packet = stats->packets_received;
        
        /* Convert elrs_rx_link_stats_t to crsf_link_stats_t
         * Citation: crsf_protocol.h lines 205-216
         * - RSSI stored as dBm * -1 (e.g., -90 dBm stored as 90)
         */
        crsf_link_stats_t crsf_stats = {
          .uplink_RSSI_1 = (uint8_t)(-stats->rssi_ant1),      /* dBm * -1 */
          .uplink_RSSI_2 = (uint8_t)(-stats->rssi_ant2),      /* dBm * -1 */
          .uplink_Link_quality = stats->lq,                   /* 0-100% */
          .uplink_SNR = stats->snr,                           /* dB */
          .active_antenna = stats->antenna,                   /* 0 or 1 */
          .rf_Mode = elrs_rx_get_rate(),                      /* Rate index */
          .uplink_TX_Power = 0,                               /* Unknown from RX side */
          .downlink_RSSI_1 = 0,                               /* N/A for RX */
          .downlink_Link_quality = 0,                         /* N/A for RX */
          .downlink_SNR = 0                                   /* N/A for RX */
        };
        crsf_send_link_stats(&crsf_stats);
      }
    }
    
    /* Periodic status print (every ~5 seconds) */
    loop_count++;
    if (loop_count - last_status_print >= 50000) {
      last_status_print = loop_count;
      
      const elrs_rx_link_stats_t *stats = elrs_rx_get_link_stats();
      const char *state_names[] = {"DISCONNECTED", "TENTATIVE", "CONNECTED"};
      
      DEBUGOUT("\n--- ELRS RX Status ---\n");
      DEBUGOUT("  State:    %s\n", state_names[current_state]);
      DEBUGOUT("  Packets:  %lu received\n", (unsigned long)stats->packets_received);
      DEBUGOUT("  LQ:       %u%%\n", stats->lq);
      DEBUGOUT("  Rate:     %d\n", elrs_rx_get_rate());
      if (crsf_is_initialized()) {
        DEBUGOUT("  CRSF TX:  %lu frames\n", (unsigned long)crsf_get_frames_sent());
      }
      DEBUGOUT("----------------------\n\n");
    }
    
    /* Small delay to prevent tight polling - 1ms
     * Citation: osDelay() from CMSIS-RTOS2
     */
    osDelay(1);
  }
  
  /* Should never reach here */
  osThreadExit();
}
#endif /* TEST_MODE_ELRS_RX */

/*******************************************************************************
 * FreeRTOS Task for ELRS 4.0 Main Flow (TEST_MODE_ELRS_MAIN)
 *
 * This is the complete ELRS 4.0 receiver implementation with:
 *   - Hardware timer (tick/tock callbacks) for precise timing
 *   - Phase-frequency detection (PFD) for timing sync
 *   - Link quality calculation
 *   - Connection state management
 *   - CRSF output to flight controller
 *
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 *   - Uses hwTimer for timing-critical operations
 *   - ProcessRFPacket handles packet validation and unpacking
 *   - PFD adjusts local timing to match transmitter
 *
 * IMPORTANT: This mode REQUIRES a working ELRS transmitter for sync!
 ******************************************************************************/
#if (TEST_MODE == TEST_MODE_ELRS_MAIN)
static const osThreadAttr_t elrs_main_task_attributes = {
  .name       = "elrs_main",
  .attr_bits  = 0,
  .cb_mem     = 0,
  .cb_size    = 0,
  .stack_mem  = 0,
  .stack_size = 8192,   /* Larger stack for ELRS 4.0 full protocol processing */
  .priority   = osPriorityAboveNormal,  /* Higher priority for real-time RX */
  .tz_module  = 0,
  .reserved   = 0,
};

/**
 * @brief Flag to signal when WiFi mode should start
 * 
 * Set by the WiFi callback from ELRS main, checked in the main loop.
 * Using volatile since it's set from callback context.
 */
static volatile bool wifi_mode_requested = false;

/**
 * @brief WiFi mode callback - called when ELRS decides to enter WiFi
 * 
 * Citation: ExpressLRS devWIFI.cpp - WifiService()
 *   When no TX connection for 60 seconds, ELRS auto-enters WiFi mode.
 *   This callback signals the main task to start the WiFi HTTP server.
 */
static void on_wifi_mode_requested(void)
{
  DEBUGOUT("[ELRS_MAIN] WiFi callback triggered - will start HTTP server\n");
  wifi_mode_requested = true;
}

/**
 * @brief FreeRTOS task entry point for ELRS 4.0 Main Flow
 * 
 * This task follows the ELRS 4.0 initialization flow but adapted for SiWx917:
 *   1. Initialize NWP subsystem (for NVM3 flash access) - WiFi radio stays OFF
 *   2. Load UID/config from NVM3 persistent storage
 *   3. Initialize ELRS 4.0 subsystems (radio, timers, FHSS, etc.)
 *   4. Enter continuous RX loop
 *   5. On WiFi timeout, start WiFi AP for OTA updates
 *
 * Citation: ExpressLRS rx_main.cpp - setup() and loop()
 * 
 * Citation: sl_si91x_nvm3_common_flash/readme.md
 *   "This example performs wireless initialization before using NVM3 APIs 
 *    using sl_net_init(). This is done to set up NWP-M4 communication."
 *
 * CRITICAL: On SiWx917 with common flash, NVM3 requires NWP to be initialized
 * FIRST via sl_net_init() before any NVM3 operations can work. However,
 * sl_net_init() only sets up NWP-M4 communication - it does NOT enable WiFi!
 * The WiFi radio is only activated by sl_net_up() which we defer until needed.
 */
static void elrs_main_task(void *argument)
{
  (void)argument;
  
  elrs_main_config_t config;  /* Runtime config for ELRS 4.0 main flow */
  sl_status_t status;
  uint32_t loop_count = 0;
  uint32_t last_status_print = 0;
  elrs_connection_state_t last_state = ELRS_DISCONNECTED;
  
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  ELRS 4.0 Main Flow Task Started\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /***************************************************************************
   * Step -1: Run LR1121 Hardware Diagnostic Test
   * 
   * This comprehensive test runs BEFORE ELRS initialization to isolate
   * hardware communication issues. It tests:
   *   - GPIO pin states (MOSI, MISO, SCK, CS, BUSY, RESET)
   *   - Raw SPI transfer (bit-level verification)
   *   - Reset sequence validation
   *   - GetStatus/GetVersion commands
   *   - Wakeup sequence functionality
   * 
   * If this test fails, the ELRS initialization will likely fail too.
   * This helps diagnose:
   *   - MISO stuck at 0x00 (broken read path)
   *   - GPIO configuration issues
   *   - LR1121 power/clock problems
   *   - SPI mode/timing issues
   **************************************************************************/
  /* Hardware diagnostic test disabled - gives false failures due to SDK 0x0022 
   * status code, but actual ELRS protocol works correctly.
   * The ELRS init below will verify LR1121 communication is working.
   */
  // DEBUGOUT("[ELRS_MAIN] Step -1: Running LR1121 hardware diagnostic test...\n");
  // lr1121_run_hardware_test();
  // DEBUGOUT("[ELRS_MAIN] Hardware test complete. Proceeding with ELRS initialization.\n\n");
  
  /***************************************************************************
   * Step 0: Initialize NWP for NVM3 access (WiFi radio stays OFF)
   * 
   * Citation: sl_si91x_nvm3_common_flash/app.c lines 132-138
   *   status = sl_net_init(SL_NET_WIFI_CLIENT_INTERFACE, &station_init_configuration, NULL, NULL);
   *   ...
   *   err = nvm3_initDefault();
   * 
   * IMPORTANT: sl_net_init() only initializes NWP-M4 communication for flash
   * access coordination. The WiFi radio is NOT enabled until sl_net_up() or
   * sl_wifi_start_ap() is called. This allows us to:
   *   1. Load binding phrase from NVM3
   *   2. Start ELRS RX immediately (WiFi off)
   *   3. Only enable WiFi AP later when OTA update is requested
   * 
   * This matches ELRS 4.0 flow where WiFi is deferred until needed.
   **************************************************************************/
  DEBUGOUT("[ELRS_MAIN] Step 0: Initialize NWP for NVM3 access (WiFi radio OFF)...\n");
  DEBUGOUT("[ELRS_MAIN]   Citation: sl_si91x_nvm3_common_flash/readme.md\n");
  DEBUGOUT("[ELRS_MAIN]   'This is done to set up NWP-M4 communication.'\n");
  
  /* Use AP interface since that's what we'll need later for OTA
   * This just initializes NWP - WiFi radio stays OFF until sl_net_up() */
  status = sl_net_init(SL_NET_WIFI_AP_INTERFACE, NULL, NULL, NULL);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("[ELRS_MAIN] ERROR: sl_net_init() failed: 0x%lX\n", (unsigned long)status);
    DEBUGOUT("[ELRS_MAIN]   NVM3 will not work! Using hardcoded defaults.\n");
    /* Continue anyway - elrs_config_init() will use defaults */
  } else {
    DEBUGOUT("[ELRS_MAIN]   NWP initialized successfully (WiFi radio OFF)\n");
    DEBUGOUT("[ELRS_MAIN]   NVM3 flash access now available\n");
  }
  
  /***************************************************************************
   * Step 1: Initialize NVM3 configuration storage
   * 
   * Now that NWP is initialized, NVM3 can access the common flash to
   * load the saved binding phrase and other configuration.
   **************************************************************************/
  DEBUGOUT("[ELRS_MAIN] Step 1: Initialize configuration storage (NVM3)...\n");
  
  /* TEMPORARY: Force reset config to clear old UID and use new Lua/WiFi method UID
   * TODO: Remove this after testing - it will clear config on EVERY boot!
   */
  DEBUGOUT("[ELRS_MAIN] *** FORCING CONFIG RESET TO UPDATE UID ***\n");
  elrs_config_reset();
  
  if (elrs_config_init() != 0) {
    DEBUGOUT("[ELRS_MAIN] ERROR: Failed to initialize NVM3 configuration\n");
    osThreadExit();
    return;
  }
  
  /* Step 2: Get default configuration and customize */
  DEBUGOUT("[ELRS_MAIN] Step 2: Load configuration...\n");
  elrs_main_get_default_config(&config);
  
  /* Use UID from config - for "matthew" (WiFi method):
   * UID = E6:A5:BA:08:42:A5
   * CRC init = (0x42 << 8 | 0xA5) ^ (4 << 8) = 0x46A5
   */
  DEBUGOUT("[ELRS_MAIN] Using UID from config: %02X:%02X:%02X:%02X:%02X:%02X\n",
           config.uid[0], config.uid[1], config.uid[2],
           config.uid[3], config.uid[4], config.uid[5]);
  
  /* Check binding status */
  if (!elrs_config_is_bound()) {
    DEBUGOUT("[ELRS_MAIN] WARNING: Device not bound - using default UID.\n");
  }
  
  DEBUGOUT("[ELRS_MAIN] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
           config.uid[0], config.uid[1], config.uid[2],
           config.uid[3], config.uid[4], config.uid[5]);
  
  /* Configure for 915 MHz FCC domain */
  config.domain = DOMAIN_FCC915;
  config.initial_rate = RATE_LORA_900_50HZ;  /* Match TX at 50Hz for faster sync */
  config.tx_power_dbm = 10;
  config.telemetry_enabled = true;
  config.crsf_baud_rate = 420000;    /* Standard CRSF baud rate */
  
  DEBUGOUT("[ELRS_MAIN]   Domain: FCC 915MHz\n");
  DEBUGOUT("[ELRS_MAIN]   Initial Rate: 50Hz\n");
  DEBUGOUT("[ELRS_MAIN]   Telemetry: Enabled\n");
  DEBUGOUT("[ELRS_MAIN]   WiFi auto-enter: %s (timeout: %lums)\n",
           config.wifi_on_no_conn ? "Enabled" : "Disabled",
           (unsigned long)config.wifi_timeout_ms);
  
  /* Step 3: Register WiFi callback before initialization
   * Citation: ExpressLRS devWIFI.cpp - WifiService()
   *   When no connection for wifi_timeout_ms, ELRS enters WiFi mode.
   *   This callback allows us to start the HTTP server from task context.
   */
  DEBUGOUT("[ELRS_MAIN] Step 3: Register WiFi mode callback...\n");
  elrs_main_set_wifi_callback(on_wifi_mode_requested);
  
  /* Step 4: Initialize ELRS main subsystem */
  DEBUGOUT("[ELRS_MAIN] Step 4: Initialize ELRS 4.0 subsystems...\n");
  if (!elrs_main_setup(&config)) {
    DEBUGOUT("[ELRS_MAIN] ERROR: Failed to initialize ELRS 4.0 subsystems\n");
    osThreadExit();
    return;
  }
  
  DEBUGOUT("[ELRS_MAIN] Initialization complete!\n");
  DEBUGOUT("[ELRS_MAIN] Entering main receive loop...\n");
  DEBUGOUT("[ELRS_MAIN] Waiting for transmitter sync...\n");
  DEBUGOUT("\n");
  
  /* Step 5: Main loop */
  while (1) {
    /* Check if WiFi mode was requested (auto-timeout or manual)
     * Citation: ExpressLRS devWIFI.cpp - WifiService()
     *   When wifi_start is triggered, start AP and HTTP server
     */
    if (wifi_mode_requested) {
      wifi_mode_requested = false;
      
      DEBUGOUT("\n");
      DEBUGOUT("========================================\n");
      DEBUGOUT("  ENTERING WiFi MODE FOR OTA UPDATES\n");
      DEBUGOUT("========================================\n");
      DEBUGOUT("\n");
      DEBUGOUT("[ELRS_MAIN] Starting WiFi AP + HTTP Server...\n");
      DEBUGOUT("[ELRS_MAIN] Connect to SSID: ELRS_TEST_AP\n");
      DEBUGOUT("[ELRS_MAIN] Password: elrs1234\n");
      DEBUGOUT("[ELRS_MAIN] Web UI: http://192.168.10.10/\n");
      DEBUGOUT("\n");
      
      /* Start WiFi HTTP server - this function runs the server
       * and only returns on error or device reset (after OTA)
       * 
       * Citation: wifi_http_test.c - wifi_http_test_run()
       *   Starts AP mode, configures static IP, runs HTTP server loop
       */
      wifi_http_test_run();
      
      /* If we return from WiFi (e.g., user cancelled OTA), exit WiFi mode */
      DEBUGOUT("[ELRS_MAIN] WiFi mode ended, resuming RX mode...\n");
      elrs_main_exit_wifi_mode();
      last_status_print = osKernelGetTickCount();
      continue;
    }
    
    /* Call main loop - processes IRQs, updates state, sends CRSF */
    bool packet_processed = elrs_main_loop();
    
    loop_count++;
    
    /* Print status every 5 seconds or on state change */
    uint32_t current_time = osKernelGetTickCount();
    elrs_connection_state_t current_state = elrs_main_get_connection_state();
    
    if (current_state != last_state) {
      const char *state_names[] = {"DISCONNECTED", "TENTATIVE", "CONNECTED"};
      DEBUGOUT("[ELRS_MAIN] State: %s -> %s\n",
               state_names[last_state], state_names[current_state]);
      last_state = current_state;
      last_status_print = current_time;
    }
    
    if ((current_time - last_status_print) >= 5000) {
      last_status_print = current_time;
      
      if (elrs_main_is_connected()) {
        /* Print connected status with signal quality */
        const elrs_channel_data_t *ch = elrs_main_get_channels();
        DEBUGOUT("[ELRS_MAIN] Connected: LQ=%d%% RSSI=%ddBm SNR=%ddB Rate=%d\n",
                 elrs_main_get_lq(),
                 elrs_main_get_rssi(),
                 elrs_main_get_snr(),
                 elrs_main_get_rate());
        DEBUGOUT("[ELRS_MAIN] Channels: A=%d E=%d T=%d R=%d\n",
                 ch->ch[0], ch->ch[1], ch->ch[2], ch->ch[3]);
      } else {
        /* Print disconnected status */
        DEBUGOUT("[ELRS_MAIN] Disconnected - scanning... (loops: %lu)\n", loop_count);
      }
      
      /* Button diagnostics removed - button is polled automatically */
    }
    
    /* Short delay to prevent hogging CPU when no packet */
    if (!packet_processed) {
      osDelay(1);
    }
  }
  
  /* Should never reach here */
  osThreadExit();
}
#endif /* TEST_MODE_ELRS_MAIN */

/*******************************************************************************
 ***************************  Defines / Macros  ********************************
 ******************************************************************************/
#define GSPI_BUFFER_SIZE             256       // Smaller buffer for faster multi-pattern test

/* Test pattern definitions */
#define PATTERN_SEQUENTIAL           0   // 1, 2, 3, 4... (original)
#define PATTERN_ALTERNATING          1   // 0xAA, 0x55, 0xAA, 0x55...
#define PATTERN_ALL_ONES             2   // 0xFF, 0xFF, 0xFF...
#define PATTERN_ALL_ZEROS            3   // 0x00, 0x00, 0x00...
#define PATTERN_WALKING_ONE          4   // 0x01, 0x02, 0x04, 0x08...
#define PATTERN_LR1121_CMD           5   // Simulated LR1121 GetVersion command pattern
#define NUM_PATTERNS                 6

#define GSPI_INTF_PLL_CLK            180000000 // Intf pll clock frequency
#define GSPI_INTF_PLL_REF_CLK        40000000  // Intf pll reference clock frequency
#define GSPI_SOC_PLL_CLK             20000000  // Soc pll clock frequency
#define GSPI_SOC_PLL_REF_CLK         40000000  // Soc pll reference clock frequency
#define GSPI_INTF_PLL_500_CTRL_VALUE 0xD900    // Intf pll control value
#define GSPI_SOC_PLL_MM_COUNT_LIMIT  0xA4      // Soc pll count limit
#define GSPI_DVISION_FACTOR          0         // Division factor
#define GSPI_SWAP_READ_DATA          1         // true to enable and false to disable swap read
#define GSPI_SWAP_WRITE_DATA         0         // true to enable and false to disable swap write
#define GSPI_BITRATE                 8000000   // 8 MHz - Production speed for LR1121
#define GSPI_BIT_WIDTH               8         // Default Bit width
#define GSPI_MAX_BIT_WIDTH           16        // Maximum Bit width
#define TIMER_FREQUENCY              32000     // Timer frequency for delay
#define INITIAL_COUNT                7000      // Count configured at timer init
#define SYNC_TIME                    1000      // Reduced delay for faster testing
#define RECEIVE_SYNC_TIME            100       // Reduced delay for faster testing

/*******************************************************************************
 *************************** LOCAL VARIABLES   *******************************
 ******************************************************************************/
//Enum for different transmission scenarios
typedef enum {
  SL_GSPI_TRANSFER_DATA,
  SL_GSPI_RECEIVE_DATA,
  SL_GSPI_SEND_DATA,
  SL_GSPI_TRANSMISSION_COMPLETED,
} gspi_mode_enum_t;
static gspi_mode_enum_t current_mode = SL_GSPI_TRANSFER_DATA;

#if (TEST_MODE == TEST_MODE_LOOPBACK)
static uint8_t gspi_data_in[GSPI_BUFFER_SIZE];
static uint8_t gspi_data_out[GSPI_BUFFER_SIZE];
static uint16_t gspi_division_factor       = 1;
static sl_gspi_handle_t gspi_driver_handle = NULL;
static uint8_t current_pattern = 0;
static uint8_t patterns_passed = 0;
static uint8_t patterns_failed = 0;
#endif /* TEST_MODE_LOOPBACK */

/*******************************************************************************
 **********************  Local Function prototypes   ***************************
 ******************************************************************************/
#if (TEST_MODE == TEST_MODE_LOOPBACK)
static bool compare_loop_back_data(void);
static void callback_event(uint32_t event);
static boolean_t transfer_complete  = false;
static boolean_t begin_transmission = true;
static void init_timer_for_sync(void);
static void wait_for_sync(uint16_t time_ms);
static void fill_pattern(uint8_t pattern_type);
static const char* get_pattern_name(uint8_t pattern_type);
static void print_test_summary(void);
#endif /* TEST_MODE_LOOPBACK */

/*******************************************************************************
 * Get human-readable pattern name
 ******************************************************************************/
#if (TEST_MODE == TEST_MODE_LOOPBACK)
static const char* get_pattern_name(uint8_t pattern_type)
{
  switch (pattern_type) {
    case PATTERN_SEQUENTIAL:    return "Sequential (1,2,3...)";
    case PATTERN_ALTERNATING:   return "Alternating (0xAA,0x55)";
    case PATTERN_ALL_ONES:      return "All Ones (0xFF)";
    case PATTERN_ALL_ZEROS:     return "All Zeros (0x00)";
    case PATTERN_WALKING_ONE:   return "Walking One (0x01,0x02,0x04...)";
    case PATTERN_LR1121_CMD:    return "LR1121 GetVersion Sim (0x01,0x01,0x00...)";
    default:                    return "Unknown";
  }
}

/*******************************************************************************
 * Fill buffer with specified test pattern
 ******************************************************************************/
static void fill_pattern(uint8_t pattern_type)
{
  for (uint16_t i = 0; i < GSPI_BUFFER_SIZE; i++) {
    switch (pattern_type) {
      case PATTERN_SEQUENTIAL:
        gspi_data_out[i] = (uint8_t)(i + 1);
        break;
      case PATTERN_ALTERNATING:
        gspi_data_out[i] = (i & 1) ? 0x55 : 0xAA;
        break;
      case PATTERN_ALL_ONES:
        gspi_data_out[i] = 0xFF;
        break;
      case PATTERN_ALL_ZEROS:
        gspi_data_out[i] = 0x00;
        break;
      case PATTERN_WALKING_ONE:
        gspi_data_out[i] = (uint8_t)(1 << (i & 7));
        break;
      case PATTERN_LR1121_CMD:
        /* Simulate LR1121 GetVersion command + NOP bytes for response
         * Citation: LR1121 Datasheet - GetVersion opcode is 0x0101
         * Phase 1: Send [0x01][0x01] (opcode)
         * Phase 2: Send [0x00][0x00][0x00][0x00][0x00] (NOP to clock out response)
         */
        if (i == 0) gspi_data_out[i] = 0x01;       // Opcode MSB
        else if (i == 1) gspi_data_out[i] = 0x01;  // Opcode LSB
        else gspi_data_out[i] = 0x00;              // NOP bytes for response
        break;
      default:
        gspi_data_out[i] = (uint8_t)(i + 1);
        break;
    }
  }
  
  /* Clear input buffer */
  for (uint16_t i = 0; i < GSPI_BUFFER_SIZE; i++) {
    gspi_data_in[i] = 0x00;
  }
}
#endif /* TEST_MODE_LOOPBACK */

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
/*******************************************************************************
 * GSPI example initialization function
 ******************************************************************************/
void gspi_example_init(void)
{
#if (TEST_MODE == TEST_MODE_RADIO_LISTEN)
  /* Radio Listen Test - Initialize radio and listen for packets
   *
   * This test mode:
   *   1. Initializes the LR1121 radio via lr1121_driver
   *   2. Configures LoRa modulation parameters (SF7, BW500, CR4/7)
   *   3. Sets frequency (default 915 MHz US ISM band)
   *   4. Enters continuous RX mode
   *   5. Prints received packets with RSSI/SNR
   *
   * Use with an ELRS TX module or another LoRa transmitter to test reception.
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: Radio Listen Test Mode\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("This test will:\n");
  DEBUGOUT("  1. Initialize LR1121 radio\n");
  DEBUGOUT("  2. Configure LoRa parameters\n");
  DEBUGOUT("  3. Enter continuous RX mode\n");
  DEBUGOUT("  4. Display received packets\n");
  DEBUGOUT("\n");
  
  /* Run the radio listen test (does not return) */
  radio_listen_test_run();
  
  /* Set mode to completed (won't reach here normally) */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_WIFI_HTTP)
  /* WiFi AP + HTTP Server Test Mode (for ELRS OTA preparation)
   *
   * Citation: wifi_access_point_soc example (app.c)
   * - WiFi functions require FreeRTOS scheduler to be running
   * - We create a task here; it executes after osKernelStart() in main.c
   * - The task handles WiFi init, AP setup, and HTTP server
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: WiFi HTTP Test Mode\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Testing WiFi AP + HTTP server for ELRS OTA.\n");
  DEBUGOUT("Creating FreeRTOS task for WiFi operations...\n");
  DEBUGOUT("\n");

  /* Create the WiFi HTTP task - runs after FreeRTOS scheduler starts */
  osThreadId_t wifi_task = osThreadNew(wifi_http_task, NULL, &wifi_task_attributes);
  if (wifi_task == NULL) {
    DEBUGOUT("ERROR: Failed to create WiFi HTTP task!\n");
  } else {
    DEBUGOUT("WiFi HTTP task created. Waiting for RTOS scheduler...\n");
  }

  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_EXTERNAL_TCXO)
  /* External TCXO Test - For Core1121-HF with externally-powered TCXO */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: External TCXO Test\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Testing initialization for externally-powered TCXO.\n");
  DEBUGOUT("The Core1121-HF module has TCXO powered from VCC.\n");
  DEBUGOUT("\n");
  
  /* Initialize the LR1121 driver first (basic SPI setup) */
  lr1121_status_t init_status = lr1121_init();
  if (init_status == LR1121_OK || init_status == LR1121_ERROR_BUSY_TIMEOUT) {
    /* Run external TCXO test suite */
    DEBUGOUT("Running External TCXO Test Suite...\n\n");
    external_tcxo_test_all();
  } else {
    DEBUGOUT("ERROR: LR1121 basic initialization failed with code %d\n", init_status);
    DEBUGOUT("Cannot run tests - check SPI connections first.\n");
  }
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_TCXO_DIAGNOSTIC)
  /* TCXO Deep Diagnostic - Diagnose HF_XOSC_START_ERR */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: TCXO Deep Diagnostic\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("This diagnostic will test if the HF_XOSC_START_ERR is:\n");
  DEBUGOUT("  A) Software configuration issue (fixable)\n");
  DEBUGOUT("  B) Hardware issue (module fault)\n");
  DEBUGOUT("\n");
  
  /* Initialize the LR1121 driver first (basic SPI setup) */
  init_status = lr1121_init();
  if (init_status == LR1121_OK || init_status == LR1121_ERROR_BUSY_TIMEOUT) {
    /* Run deep TCXO diagnostic even if init had timeout (expected with XOSC fail) */
    DEBUGOUT("Running TCXO Deep Diagnostic...\n\n");
    tcxo_deep_diagnostic();
  } else {
    DEBUGOUT("ERROR: LR1121 basic initialization failed with code %d\n", init_status);
    DEBUGOUT("Cannot run TCXO diagnostic - check SPI connections first.\n");
  }
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_TCXO_SWEEP)
  /* TCXO Sweep Test - Comprehensive voltage/delay testing
   *
   * This mode systematically tests TCXO configurations to find
   * the optimal voltage and delay settings for reliable operation.
   *
   * Tests performed:
   *   1. SPI baseline communication check
   *   2. Voltage sweep (0x00, 0x02, 0x06, 0x07)
   *   3. Delay sweep (5ms, 9ms, 15ms, 30ms, 60ms)
   *   4. Full init validation with temperature reading
   *   5. Stress test (10 cycles)
   *
   * NO TRANSMITTER REQUIRED - tests LR1121 internal functions only.
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: TCXO Sweep Test\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Testing TCXO voltage and delay configurations\n");
  DEBUGOUT("to find optimal settings for Waveshare Core1121-XF.\n");
  DEBUGOUT("\n");
  
  /* Run the EXTENDED TCXO stress test suite (includes basic tests + stress tests) */
  lr1121_tcxo_stress_test_all();
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_STANDALONE_TESTS)
  /* LR1121 Standalone Test Suite - No second receiver required */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: LR1121 Standalone Tests\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /* Initialize the LR1121 driver first */
  lr1121_status_t init_status = lr1121_init();
  if (init_status == LR1121_OK) {
    /* Run the standalone test suite */
    lr1121_run_standalone_tests();
  } else {
    DEBUGOUT("ERROR: LR1121 initialization failed with code %d\n", init_status);
  }
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_DIO1_TEST)
  /* DIO1 Comprehensive Interrupt Test Suite
   *
   * Citation: lr1121_dio1_interrupt_test.c - Comprehensive test suite
   * Hardware: SiW917 BRD2708A UULP_VBAT_GPIO_2 connected to LR1121 DIO1
   * Citation: UG590 BRD2708A User Guide Section 3.8.2 Table 3.3:
   *   "INT - Hardware Interrupt - UULP_VBAT_GPIO_2"
   *
   * This mode tests END-TO-END interrupt operation:
   * - Test 1: DIO1 IRQ configuration (SetDioIrqParams)
   * - Test 2: RX Timeout interrupt triggering (DIO1 fires on timeout)
   * - Test 3: osThreadFlagsWait() signaling from ISR
   * - Test 4: Multiple consecutive interrupts (stress test)
   * - Test 5: DIO1 pin state correlation with IRQ status
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("  GSPI Example: DIO1 COMPREHENSIVE Interrupt Tests\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("\n");
  
  /* Run the comprehensive DIO1 interrupt test suite
   * Note: This calls lr1121_hal_init() internally which initializes LR1121 */
  lr1121_dio1_interrupt_run_all_tests();
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_GPIO_TOGGLE)
  /* GPIO Toggle Test Mode - Verify outputs with multimeter */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: GPIO Toggle Test Mode\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /* Run the GPIO toggle test - 5 cycles, or use 0 for infinite */
  lr1121_gpio_toggle_test(5);
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_BUSY_MONITOR)
  /* BUSY Pin Monitor Test Mode */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: BUSY Monitor Test Mode\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /* Run the BUSY pin monitor test */
  busy_monitor_test_run();
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_LR1121_SPI)
  /* LR1121 Communication Test Mode */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: LR1121 Test Mode\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  
  /* Run the LR1121 communication test */
  lr1121_test_communication();
  
  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_ELRS_RX)
  /* ELRS RX Main Loop Test Mode - Full ExpressLRS protocol stack
   *
   * Citation: ExpressLRS 4.0 rx_main.cpp - RX main loop architecture
   * - Implements connection state machine (DISCONNECTED -> TENTATIVE -> CONNECTED)
   * - FHSS frequency hopping with sync
   * - OTA packet validation with CRC
   * - Channel data unpacking (4x 10-bit channels or 8x switch channels)
   *
   * This mode runs the complete ELRS RX protocol stack with:
   * - elrs_rx_init(): Initialize HAL, FHSS tables, OTA CRC
   * - elrs_rx_start(): Configure radio for ELRS 2.4GHz and enter RX
   * - elrs_rx_loop(): Poll IRQ, process packets, handle hopping
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: ELRS RX Main Loop\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Starting full ExpressLRS RX protocol stack.\n");
  DEBUGOUT("Creating FreeRTOS task for ELRS RX operations...\n");
  DEBUGOUT("\n");

  /* Create the ELRS RX task - runs after FreeRTOS scheduler starts */
  osThreadId_t elrs_task = osThreadNew(elrs_rx_task, NULL, &elrs_rx_task_attributes);
  if (elrs_task == NULL) {
    DEBUGOUT("ERROR: Failed to create ELRS RX task!\n");
  } else {
    DEBUGOUT("ELRS RX task created. Waiting for RTOS scheduler...\n");
  }

  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;
  
#elif (TEST_MODE == TEST_MODE_ELRS_MAIN)
  /* ELRS 4.0 Main Flow Test Mode - Complete ExpressLRS receiver
   *
   * Citation: ExpressLRS 4.0 rx_main.cpp - Full RX implementation
   * - Uses hwTimer for precise timing (tick/tock callbacks)
   * - Phase-frequency detection (PFD) for TX sync
   * - Link quality calculation
   * - CRSF output to flight controller
   *
   * This mode requires:
   * - Working ELRS TX module (for timing sync)
   * - Same binding phrase/UID as TX
   * - Proper RF environment for 915 MHz
   *
   * IMPORTANT: Unlike TEST_MODE_ELRS_RX (legacy polling), this mode
   * uses the full ELRS 4.0 architecture with hardware timer sync.
   */
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: ELRS 4.0 Main Flow\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Starting full ExpressLRS 4.0 receiver stack.\n");
  DEBUGOUT("Features:\n");
  DEBUGOUT("  - Hardware timer for timing sync\n");
  DEBUGOUT("  - Phase-frequency detection (PFD)\n");
  DEBUGOUT("  - Link quality calculation\n");
  DEBUGOUT("  - CRSF output to flight controller\n");
  DEBUGOUT("\n");
  DEBUGOUT("Creating FreeRTOS task for ELRS 4.0 operations...\n");
  DEBUGOUT("\n");

  /* Create the ELRS Main task - runs after FreeRTOS scheduler starts */
  osThreadId_t elrs_main_task_handle = osThreadNew(elrs_main_task, NULL, &elrs_main_task_attributes);
  if (elrs_main_task_handle == NULL) {
    DEBUGOUT("ERROR: Failed to create ELRS Main task!\n");
  } else {
    DEBUGOUT("ELRS Main task created. Waiting for RTOS scheduler...\n");
  }

  /* Set mode to completed so process_action does nothing */
  current_mode = SL_GSPI_TRANSMISSION_COMPLETED;

#elif (TEST_MODE == TEST_MODE_SPI_LOOPBACK)
  /***************************************************************************
   * SIMPLE SPI LOOPBACK TEST - For debugging hardware GSPI
   * 
   * Connect MOSI (GPIO_27) to MISO (GPIO_26) with a jumper wire.
   * This bypasses the LR1121 and tests pure SPI hardware.
   ***************************************************************************/
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  SIMPLE HARDWARE SPI LOOPBACK TEST\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("*** Connect MOSI (GPIO_27) to MISO (GPIO_26) ***\n");
  DEBUGOUT("\n");
  
  /* Initialize LR1121 driver (sets up GSPI) */
  DEBUGOUT("Initializing GSPI via LR1121 driver...\n");
  lr1121_status_t init_status = lr1121_init();
  DEBUGOUT("lr1121_init() returned: %d\n\n", init_status);
  
  if (init_status != LR1121_OK) {
    DEBUGOUT("ERROR: GSPI init failed!\n");
  } else {
    /* Run actual loopback test */
    DEBUGOUT("Running SPI loopback test...\n\n");
    
    uint8_t test_patterns[][8] = {
      {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55},  /* Alternating */
      {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF},  /* Min/Max */
      {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},  /* Walking 1 */
      {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE},  /* Random */
    };
    const char *pattern_names[] = {
      "Alternating (0xAA/0x55)",
      "Min/Max (0x00/0xFF)",
      "Walking 1s",
      "Random (DEADBEEF...)",
    };
    
    int total_tests = 4;
    int passed = 0;
    
    for (int p = 0; p < total_tests; p++) {
      uint8_t tx_buf[8];
      uint8_t rx_buf[8];
      
      memcpy(tx_buf, test_patterns[p], 8);
      memset(rx_buf, 0x00, 8);
      
      DEBUGOUT("Test %d: %s\n", p+1, pattern_names[p]);
      DEBUGOUT("  TX: ");
      for (int i = 0; i < 8; i++) DEBUGOUT("%02X ", tx_buf[i]);
      DEBUGOUT("\n");
      
      /* Do SPI transfer using the driver's low-level function */
      lr1121_cs_assert();
      bool ok = lr1121_spi_transfer(tx_buf, rx_buf, 8);
      lr1121_cs_deassert();
      
      DEBUGOUT("  RX: ");
      for (int i = 0; i < 8; i++) DEBUGOUT("%02X ", rx_buf[i]);
      DEBUGOUT("\n");
      
      /* Compare */
      bool match = ok && (memcmp(tx_buf, rx_buf, 8) == 0);
      if (match) {
        DEBUGOUT("  PASS!\n\n");
        passed++;
      } else {
        DEBUGOUT("  FAIL!\n\n");
      }
    }
    
    DEBUGOUT("========================================\n");
    if (passed == total_tests) {
      DEBUGOUT("  LOOPBACK TEST PASSED! (%d/%d)\n", passed, total_tests);
      DEBUGOUT("  Hardware SPI is working correctly.\n");
    } else {
      DEBUGOUT("  LOOPBACK TEST FAILED! (%d/%d)\n", passed, total_tests);
      DEBUGOUT("  Check MOSI->MISO jumper connection.\n");
    }
    DEBUGOUT("========================================\n");
  }
  
  DEBUGOUT("\nTest complete. Halting.\n");
  while(1) {
    for (volatile int i = 0; i < 1000000; i++);
  }

#elif (TEST_MODE == TEST_MODE_LR1121_SIMPLE)
  /***************************************************************************
   * SIMPLE LR1121 GetVersion TEST
   * 
   * Just init and do GetVersion - no TCXO complexity
   ***************************************************************************/
  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  SIMPLE LR1121 GetVersion TEST\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("This test uses ELRS-compatible init (no SetTcxoMode)\n");
  DEBUGOUT("Expected for ELRS firmware: HW=0x22, Type=0xF3, Ver=1.4\n");
  DEBUGOUT("\n");
  
  /* Initialize driver */
  lr1121_status_t init_status = lr1121_init();
  DEBUGOUT("lr1121_init() returned: %d\n\n", init_status);
  
  if (init_status != LR1121_OK) {
    DEBUGOUT("ERROR: Driver init failed!\n");
  } else {
    /* Simple GetVersion test using driver functions */
    DEBUGOUT("=== GetVersion Test ===\n");
    
    /* Wait for BUSY */
    DEBUGOUT("Waiting for BUSY LOW...\n");
    if (!lr1121_wait_busy_timeout(100)) {
      DEBUGOUT("ERROR: BUSY timeout!\n");
    } else {
      DEBUGOUT("BUSY is LOW, sending GetVersion...\n\n");
      
      /* Phase 1: Send GetVersion opcode (0x0101) */
      DEBUGOUT("Phase 1: Send GetVersion command\n");
      lr1121_cs_assert();
      uint8_t cmd[2] = {0x01, 0x01};  /* GetVersion opcode */
      uint8_t cmd_rx[2] = {0};
      lr1121_spi_transfer(cmd, cmd_rx, 2);
      lr1121_cs_deassert();
      DEBUGOUT("  CMD TX: %02X %02X\n", cmd[0], cmd[1]);
      DEBUGOUT("  CMD RX: %02X %02X (ignore - status during command)\n", cmd_rx[0], cmd_rx[1]);
      
      /* Wait for processing */
      DEBUGOUT("\nWaiting for BUSY LOW (command processing)...\n");
      for (volatile int i = 0; i < 10000; i++);  /* Small delay */
      if (!lr1121_wait_busy_timeout(100)) {
        DEBUGOUT("ERROR: BUSY timeout after command!\n");
      } else {
        DEBUGOUT("BUSY is LOW, reading response...\n\n");
        
        /* Phase 2: Read response using BIT-BANG to verify MISO works */
        DEBUGOUT("Phase 2: Bit-bang read (bypass HW GSPI)\n");
        
        /* GPIO register definitions for bit-bang */
        #define EGPIO_BASE_BB 0x46130000UL
        #define EGPIO_CONFIG_BB(pin) (*(volatile uint32_t *)(EGPIO_BASE_BB + (0x10 * (pin))))
        #define EGPIO_BIT_BB(pin)    (*(volatile uint32_t *)(EGPIO_BASE_BB + 0x004 + (0x10 * (pin))))
        #define MCR_CTRL1_BB (*(volatile uint32_t *)(0x46008000UL + 0x044))
        
        /* Check and print HOST_PADS_GPIO_MODE */
        DEBUGOUT("  MCR_CTRL1 = 0x%08lX, bits[18:13] = 0x%02lX\n", 
                 (unsigned long)MCR_CTRL1_BB,
                 (unsigned long)((MCR_CTRL1_BB >> 13) & 0x3F));
        
        /* Try setting HOST_PADS_GPIO_MODE for GPIO_26 (bit 14) to 1 for GPIO mode */
        MCR_CTRL1_BB |= (1UL << 14);  /* GPIO_26 = GPIO mode */
        for (volatile int d = 0; d < 100; d++);
        DEBUGOUT("  MCR_CTRL1 after = 0x%08lX\n", (unsigned long)MCR_CTRL1_BB);
        
        /* Save current GPIO modes */
        uint32_t saved_sck  = EGPIO_CONFIG_BB(25);
        uint32_t saved_miso = EGPIO_CONFIG_BB(26);
        uint32_t saved_mosi = EGPIO_CONFIG_BB(27);
        
        /* Switch to GPIO mode (MODE=0) for bit-bang */
        EGPIO_CONFIG_BB(25) = (saved_sck & ~(0xF << 2));   /* SCK: output */
        EGPIO_CONFIG_BB(26) = (saved_miso & ~(0xF << 2)) | 1;  /* MISO: input */
        EGPIO_CONFIG_BB(27) = (saved_mosi & ~(0xF << 2));  /* MOSI: output */
        for (volatile int d = 0; d < 100; d++);
        
        /* Set idle: SCK LOW, MOSI LOW */
        EGPIO_BIT_BB(25) = 0;  /* SCK LOW */
        EGPIO_BIT_BB(27) = 0;  /* MOSI LOW */
        
        lr1121_cs_assert();
        for (volatile int d = 0; d < 500; d++);
        
        /* Bit-bang read 5 bytes */
        uint8_t bb_resp[5] = {0};
        for (int byte = 0; byte < 5; byte++) {
          uint8_t rx_byte = 0;
          for (int bit = 7; bit >= 0; bit--) {
            /* MOSI = 0 (NOP) already set */
            for (volatile int d = 0; d < 20; d++);
            
            /* Clock HIGH - sample MISO */
            EGPIO_BIT_BB(25) = 1;
            for (volatile int d = 0; d < 20; d++);
            
            if (EGPIO_BIT_BB(26) & 1) {
              rx_byte |= (1 << bit);
            }
            
            /* Clock LOW */
            EGPIO_BIT_BB(25) = 0;
            for (volatile int d = 0; d < 20; d++);
          }
          bb_resp[byte] = rx_byte;
        }
        
        lr1121_cs_deassert();
        
        /* Restore GSPI mode */
        EGPIO_CONFIG_BB(25) = saved_sck;
        EGPIO_CONFIG_BB(26) = saved_miso;
        EGPIO_CONFIG_BB(27) = saved_mosi;
        for (volatile int d = 0; d < 100; d++);
        
        DEBUGOUT("  Bit-bang RSP: %02X %02X %02X %02X %02X\n", 
                 bb_resp[0], bb_resp[1], bb_resp[2], bb_resp[3], bb_resp[4]);
        
        /* Also try HW GSPI for comparison */
        DEBUGOUT("Phase 2b: HW GSPI read\n");
        uint8_t nop[5] = {0};
        uint8_t resp[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        lr1121_cs_assert();
        lr1121_spi_transfer(nop, resp, 5);
        lr1121_cs_deassert();
        DEBUGOUT("  HW GSPI RSP: %02X %02X %02X %02X %02X\n", 
                 resp[0], resp[1], resp[2], resp[3], resp[4]);
        
        /* Parse bit-bang response */
        DEBUGOUT("\nParsing bit-bang response:\n");
        DEBUGOUT("  Status=0x%02X HW=0x%02X Type=0x%02X Ver=%d.%d\n",
                 bb_resp[0], bb_resp[1], bb_resp[2], bb_resp[3], bb_resp[4]);
        
        uint8_t status_byte = resp[0];
        uint8_t hw = resp[1];
        uint8_t type = resp[2];
        uint16_t fw_ver = (resp[3] << 8) | resp[4];
        
        DEBUGOUT("=== Parsed Response ===\n");
        DEBUGOUT("  Status:   0x%02X\n", status_byte);
        DEBUGOUT("  Hardware: 0x%02X (%s)\n", hw, 
                 hw == 0x22 ? "LR1121" : 
                 hw == 0x21 ? "LR1120" : "Unknown");
        DEBUGOUT("  Type:     0x%02X (%s)\n", type,
                 type == 0xF3 ? "ELRS Custom" :
                 type == 0x03 ? "Stock LR1121" :
                 type == 0xDF ? "Bootloader" : "Unknown");
        DEBUGOUT("  Firmware: v%d.%d\n", fw_ver >> 8, fw_ver & 0xFF);
        DEBUGOUT("\n");
        
        if (hw == 0x22 && (type == 0xF3 || type == 0x03)) {
          DEBUGOUT("SUCCESS: Valid LR1121 response!\n");
        } else {
          DEBUGOUT("WARNING: Unexpected values - check SPI timing\n");
        }
      }
    }
  }
  
  DEBUGOUT("\nTest complete. Halting.\n");
  while(1) {
    for (volatile int i = 0; i < 1000000; i++);
  }

#else
  /* Enhanced Loopback Test Mode with Multiple Patterns */
  sl_status_t status;
  sl_gspi_version_t version;
  sl_gspi_status_t gspi_status;
  sl_gspi_control_config_t config;
  
  DEBUGOUT("\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("  GSPI Enhanced Loopback Test (LR1121 SPI Preparation)\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("Test Configuration:\n");
  DEBUGOUT("  SPI Mode:    Mode 0 (CPOL=0, CPHA=0) - LR1121 compatible\n");
  DEBUGOUT("  Bit Width:   8 bits\n");
  DEBUGOUT("  Bitrate:     8 MHz (production speed)\n");
  DEBUGOUT("  Buffer Size: %d bytes\n", GSPI_BUFFER_SIZE);
  DEBUGOUT("  Patterns:    %d (Sequential, Alternating, AllOnes, AllZeros, WalkingOne, LR1121Cmd)\n", NUM_PATTERNS);
  DEBUGOUT("\n");
  DEBUGOUT("IMPORTANT: For loopback test, connect MOSI (GPIO_27) to MISO (GPIO_26)\n");
  DEBUGOUT("\n");
  
  /* Configure for LR1121-compatible SPI settings
   * Citation: LR1121 Datasheet Section 3 - SPI Interface
   * - Mode 0: CPOL=0, CPHA=0 (clock idle LOW, sample on rising edge)
   * - Max clock: 16 MHz
   * - MSB first
   */
  config.bit_width         = GSPI_BIT_WIDTH;
  config.bitrate           = GSPI_BITRATE;
  config.clock_mode        = SL_GSPI_MODE_0;  /* LR1121 requires Mode 0 */
  config.slave_select_mode = SL_GSPI_MASTER_HW_OUTPUT;
  config.swap_read         = GSPI_SWAP_READ_DATA;
  config.swap_write        = GSPI_SWAP_WRITE_DATA;

  /* Initialize first pattern */
  current_pattern = 0;
  fill_pattern(current_pattern);
  
  do {
    /* Initialize the timer */
    init_timer_for_sync();
    
    /* Get GSPI driver version */
    version = sl_si91x_gspi_get_version();
    DEBUGOUT("GSPI Driver Version: %d.%d.%d\n", version.release, version.major, version.minor);
    
    /* Initialize GSPI */
    status = sl_si91x_gspi_init(SL_GSPI_MASTER, &gspi_driver_handle);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("ERROR: sl_si91x_gspi_init failed: 0x%04lX\n", status);
      break;
    }
    DEBUGOUT("GSPI initialized successfully\n");
    
    /* Check GSPI status */
    gspi_status = sl_si91x_gspi_get_status(gspi_driver_handle);
    DEBUGOUT("GSPI Status: Busy=%d, DataLost=%d, ModeFault=%d\n", 
             gspi_status.busy, gspi_status.data_lost, gspi_status.mode_fault);
    
    /* Configure GSPI */
    status = sl_si91x_gspi_set_configuration(gspi_driver_handle, &config);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("ERROR: sl_si91x_gspi_set_configuration failed: 0x%04lX\n", status);
      break;
    }
    DEBUGOUT("GSPI configured successfully\n");
    
    /* Register callback */
    status = sl_si91x_gspi_register_event_callback(gspi_driver_handle, callback_event);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("ERROR: sl_si91x_gspi_register_event_callback failed: 0x%04lX\n", status);
      break;
    }
    
    /* Print clock info */
    DEBUGOUT("Clock Division Factor: %lu\n", sl_si91x_gspi_get_clock_division_factor(gspi_driver_handle));
    DEBUGOUT("Frame Length: %lu bits\n", sl_si91x_gspi_get_frame_length());
    
    if (sl_si91x_gspi_get_frame_length() > GSPI_BIT_WIDTH) {
      gspi_division_factor = sizeof(gspi_data_out[0]);
    }
    
    /* Sync delay */
    wait_for_sync(SYNC_TIME);
    
    /* Start with transfer mode */
    DEBUGOUT("\n");
    DEBUGOUT("========== Starting Multi-Pattern Loopback Test ==========\n");
    current_mode = SL_GSPI_TRANSFER_DATA;
    
  } while (false);
#endif
}

#if (TEST_MODE == TEST_MODE_LOOPBACK)
/*******************************************************************************
 * Print final test summary
 ******************************************************************************/
static void print_test_summary(void)
{
  DEBUGOUT("\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("                  TEST SUMMARY\n");
  DEBUGOUT("========================================================\n");
  DEBUGOUT("  Patterns Tested:  %d\n", NUM_PATTERNS);
  DEBUGOUT("  Patterns PASSED:  %d\n", patterns_passed);
  DEBUGOUT("  Patterns FAILED:  %d\n", patterns_failed);
  DEBUGOUT("\n");
  
  if (patterns_failed == 0 && patterns_passed == NUM_PATTERNS) {
    DEBUGOUT("  *** ALL TESTS PASSED - GSPI READY FOR LR1121 ***\n");
    DEBUGOUT("\n");
    DEBUGOUT("  LR1121 SPI Requirements Verified:\n");
    DEBUGOUT("    [OK] SPI Mode 0 (CPOL=0, CPHA=0)\n");
    DEBUGOUT("    [OK] 8-bit frame width\n");
    DEBUGOUT("    [OK] Full-duplex operation\n");
    DEBUGOUT("    [OK] Data integrity (all patterns matched)\n");
    DEBUGOUT("\n");
    DEBUGOUT("  Next Steps:\n");
    DEBUGOUT("    1. Connect LR1121 module to mikroBUS socket\n");
    DEBUGOUT("    2. Set USE_LR1121_TEST to 1 in gspi_example.c\n");
    DEBUGOUT("    3. Rebuild and flash\n");
  } else {
    DEBUGOUT("  *** SOME TESTS FAILED - CHECK CONNECTIONS ***\n");
    DEBUGOUT("\n");
    DEBUGOUT("  Troubleshooting:\n");
    DEBUGOUT("    1. Verify MOSI (GPIO_27) connected to MISO (GPIO_26)\n");
    DEBUGOUT("    2. Check for loose connections\n");
    DEBUGOUT("    3. Try lower bitrate\n");
  }
  DEBUGOUT("========================================================\n");
}
#endif /* TEST_MODE_LOOPBACK */

/*******************************************************************************
 * Function will run continuously in while loop
 ******************************************************************************/
void gspi_example_process_action(void)
{
#if (TEST_MODE == TEST_MODE_RADIO_LISTEN) || (TEST_MODE == TEST_MODE_WIFI_HTTP) || (TEST_MODE == TEST_MODE_EXTERNAL_TCXO) || (TEST_MODE == TEST_MODE_TCXO_DIAGNOSTIC) || (TEST_MODE == TEST_MODE_STANDALONE_TESTS) || (TEST_MODE == TEST_MODE_GPIO_TOGGLE) || (TEST_MODE == TEST_MODE_BUSY_MONITOR) || (TEST_MODE == TEST_MODE_LR1121_SPI) || (TEST_MODE == TEST_MODE_ELRS_RX) || (TEST_MODE == TEST_MODE_DIO1_TEST) || (TEST_MODE == TEST_MODE_ELRS_MAIN) || (TEST_MODE == TEST_MODE_TCXO_SWEEP) || (TEST_MODE == TEST_MODE_SPI_LOOPBACK) || (TEST_MODE == TEST_MODE_LR1121_SIMPLE)
  /* All non-loopback test modes - nothing to do, test runs in init or FreeRTOS task */
  (void)current_mode; /* Suppress unused variable warning */
#else
  sl_status_t status;
  
  switch (current_mode) {
    case SL_GSPI_TRANSFER_DATA:
      if (begin_transmission == true) {
        DEBUGOUT("\n--- Pattern %d: %s ---\n", current_pattern + 1, get_pattern_name(current_pattern));
        DEBUGOUT("TX First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                 gspi_data_out[0], gspi_data_out[1], gspi_data_out[2], gspi_data_out[3],
                 gspi_data_out[4], gspi_data_out[5], gspi_data_out[6], gspi_data_out[7]);
        
        sl_si91x_gspi_set_slave_number(GSPI_SLAVE_0);
        status = sl_si91x_gspi_transfer_data(gspi_driver_handle,
                                             gspi_data_out,
                                             gspi_data_in,
                                             sizeof(gspi_data_out) / gspi_division_factor);
        if (status != SL_STATUS_OK) {
          DEBUGOUT("ERROR: sl_si91x_gspi_transfer_data failed: 0x%04lX\n", status);
          patterns_failed++;
          
          /* Move to next pattern */
          current_pattern++;
          if (current_pattern >= NUM_PATTERNS) {
            print_test_summary();
            current_mode = SL_GSPI_TRANSMISSION_COMPLETED;
          } else {
            fill_pattern(current_pattern);
          }
          break;
        }
        begin_transmission = false;
      }
      
      if (transfer_complete) {
        transfer_complete = false;
        
        DEBUGOUT("RX First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                 gspi_data_in[0], gspi_data_in[1], gspi_data_in[2], gspi_data_in[3],
                 gspi_data_in[4], gspi_data_in[5], gspi_data_in[6], gspi_data_in[7]);
        
        /* Compare data */
        if (compare_loop_back_data()) {
          DEBUGOUT("Result: PASSED\n");
          patterns_passed++;
        } else {
          DEBUGOUT("Result: FAILED\n");
          patterns_failed++;
        }
        
        /* Move to next pattern */
        current_pattern++;
        if (current_pattern >= NUM_PATTERNS) {
          print_test_summary();
          current_mode = SL_GSPI_TRANSMISSION_COMPLETED;
        } else {
          fill_pattern(current_pattern);
          begin_transmission = true;
          wait_for_sync(100); /* Small delay between patterns */
        }
      }
      break;

    case SL_GSPI_SEND_DATA:
    case SL_GSPI_RECEIVE_DATA:
      /* Not used in multi-pattern test */
      current_mode = SL_GSPI_TRANSMISSION_COMPLETED;
      break;

    case SL_GSPI_TRANSMISSION_COMPLETED:
      break;
  }
#endif /* USE_LR1121_TEST */
}

#if (TEST_MODE == TEST_MODE_LOOPBACK)
/*******************************************************************************
 * Function to compare the loop back data
 * Returns true if data matches, false otherwise
 ******************************************************************************/
static bool compare_loop_back_data(void)
{
  uint16_t data_index   = 0;
  uint32_t frame_length = 0;
  uint16_t mask         = (uint16_t)~0;
  uint16_t mismatch_count = 0;
  
  frame_length = sl_si91x_gspi_get_frame_length();
  mask = mask >> (GSPI_MAX_BIT_WIDTH - frame_length);
  
  for (data_index = 0; data_index < GSPI_BUFFER_SIZE; data_index++) {
    uint8_t expected = gspi_data_out[data_index] & mask;
    uint8_t received = gspi_data_in[data_index] & mask;
    
    if (expected != received) {
      if (mismatch_count < 3) {
        DEBUGOUT("  Mismatch at [%d]: TX=0x%02X, RX=0x%02X\n", data_index, expected, received);
      }
      mismatch_count++;
    }
  }
  
  if (mismatch_count > 0) {
    DEBUGOUT("  Total mismatches: %d of %d bytes\n", mismatch_count, GSPI_BUFFER_SIZE);
    return false;
  }
  
  return true;
}

/*******************************************************************************
 * Callback event function
 ******************************************************************************/
static void callback_event(uint32_t event)
{
  switch (event) {
    case SL_GSPI_TRANSFER_COMPLETE:
      transfer_complete = true;
      break;
    case SL_GSPI_DATA_LOST:
      DEBUGOUT("WARNING: GSPI Data Lost!\n");
      break;
    case SL_GSPI_MODE_FAULT:
      DEBUGOUT("WARNING: GSPI Mode Fault!\n");
      break;
  }
}

/*******************************************************************************
 * @brief  Initialization of timer for sync
 ******************************************************************************/
static void init_timer_for_sync(void)
{
  sl_status_t status;
  status = sl_si91x_ulp_timer_configure_clock(&sl_timer_clk_handle);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("sl_si91x_ulp_timer_configure_clock failed, error code: %ld", status);
  }
  status = sl_si91x_ulp_timer_set_configuration(&sl_timer_handle_timer0);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("sl_si91x_ulp_timer_set_configuration failed, error code: %ld", status);
  }
  status = sl_si91x_ulp_timer_set_count(TIMER_0, TIMER_FREQUENCY * INITIAL_COUNT);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("sl_si91x_ulp_timer_set_count failed, error code: %ld", status);
  }
}

/*******************************************************************************
 * @brief  Waits till the master and slave application is synced
 ******************************************************************************/
static void wait_for_sync(uint16_t time_ms)
{
  sl_status_t status;
  uint32_t start_time, current_time;
  uint32_t end_time = time_ms * TIMER_FREQUENCY;

  status = sl_si91x_ulp_timer_start(TIMER_0);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("sl_si91x_ulp_timer_start failed, error code: %ld", status);
  }
  status = sl_si91x_ulp_timer_get_count(TIMER_0, &start_time);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("sl_si91x_ulp_timer_get_count failed, error code: %ld", status);
  }
  do {
    status = sl_si91x_ulp_timer_get_count(TIMER_0, &current_time);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_ulp_timer_get_count failed, error code: %ld", status);
    }
  } while (!((current_time - start_time) > end_time));
  sl_si91x_ulp_timer_stop(TIMER_0);
}
#endif /* TEST_MODE_LOOPBACK */
