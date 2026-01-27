/**
 * @file elrs_main.c
 * @brief ELRS 4.0 Main Flow Controller Implementation
 * 
 * Implements the complete ELRS 4.0 receiver flow.
 * 
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 *   - Complete receiver state machine
 *   - hwTimer integration
 *   - PFD phase locking
 */

#include "elrs_main.h"
#include "elrs_rx.h"
#include "hw_timer.h"
#include "pfd.h"
#include "lq_calc.h"
#include "lpf.h"
#include "fhss.h"
#include "ota.h"
#include "crsf.h"
#include "lr1121_hal.h"
#include "rsi_debug.h"
#include "cmsis_os2.h"
#include "bind_button.h"
#include "status_led.h"

#include <string.h>
#include <inttypes.h>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define ELRS_MAIN_DEBUG     1

#if ELRS_MAIN_DEBUG
    #define MAIN_DBG(fmt, ...)    DEBUGOUT("[ELRS_MAIN] " fmt, ##__VA_ARGS__)
#else
    #define MAIN_DBG(fmt, ...)    ((void)0)
#endif

/* Connection timeouts
 * Citation: ExpressLRS rx_main.cpp
 */
#define CONN_LOST_TIMEOUT_MS        1500    /* Lost connection if no packets */
#define CRSF_SEND_INTERVAL_MS       4       /* ~250Hz CRSF output */
#define LINK_STATS_INTERVAL_MS      100     /* 10Hz link stats */

/**
 * ConsiderConnGoodMillis - Time in connected state before timer becomes "locked"
 * 
 * Citation: ExpressLRS rx_main.cpp line 164
 *   const uint32_t ConsiderConnGoodMillis = 1000;
 * 
 * After this time, if PFD is stable (LPF_OffsetDx <= 5), the timer transitions
 * from tim_tentative to tim_locked state.
 */
#define CONSIDER_CONN_GOOD_MS       1000

/**
 * Rate Cycling Constants
 * 
 * Citation: ExpressLRS rx_main.cpp lines 187-191
 *   #define RFmodeCycleMultiplierSlow 10
 *   uint32_t cycleInterval; // in ms
 *   uint8_t RFmodeCycleMultiplier;
 *   bool LockRFmode = false;
 * 
 * The cycle interval is calculated dynamically based on the current rate's
 * FHSS parameters. The multiplier starts high (slow cycling) and drops to 1
 * (fast cycling) after first attempting the initial rate.
 */
#define RFMODE_CYCLE_MULTIPLIER_SLOW    10  /* Initial slow multiplier (5x faster = 10/2 = 5 on start) */

/* WiFi entry timeout when disconnected */
#define WIFI_AUTO_ENTER_MS          60000   /* Enter WiFi after 60s no conn */

/*******************************************************************************
 * Module State
 ******************************************************************************/

/* RSSI/SNR filters */
static lpf_t rssi_filter;
static lpf_t snr_filter;

/**
 * PFD Phase Stability Filters
 * 
 * Citation: ExpressLRS rx_main.cpp lines 146-147
 *   LPF LPF_Offset(2);     // Filtered phase offset
 *   LPF LPF_OffsetDx(4);   // Rate of change of phase offset (derivative)
 * 
 * These filters are used in the GotConnection() criteria to ensure
 * we have a stable phase lock before declaring connected:
 *   - abs(LPF_OffsetDx) <= 10 (phase is stable)
 *   - LPF_Offset < 100 (phase offset is small)
 * 
 * Citation: ExpressLRS rx_main.cpp line 161
 *   int32_t PfdPrevRawOffset;  // Previous raw offset for derivative calculation
 */
static lpf_t lpf_offset;           /* Filtered phase offset (beta=2) */
static lpf_t lpf_offset_dx;        /* Filtered offset derivative (beta=4) */
static int32_t pfd_prev_raw_offset;  /* Previous raw offset for dx calculation */

/* Main state */
static struct {
    /* Configuration */
    elrs_main_config_t   config;
    
    /* Operating state */
    elrs_mode_t     mode;
    elrs_connection_state_t conn_state;
    
    /* Timing */
    uint32_t        last_packet_us;     /* Micros timestamp of last packet */
    uint32_t        last_packet_ms;     /* Millis timestamp of last packet */
    uint32_t        last_sync_ms;       /* Millis timestamp of last SYNC packet */
    uint32_t        last_crsf_ms;       /* Last CRSF frame sent */
    uint32_t        last_stats_ms;      /* Last link stats sent */
    uint32_t        connected_ms;       /* When we became tentative */
    uint32_t        got_connection_ms;  /* When we got full connection (ELRS 4.0) */
    uint32_t        disconnected_ms;    /* When we became disconnected */
    
    /* Rate cycling state
     * Citation: ExpressLRS rx_main.cpp lines 157, 187-191, 354
     */
    uint32_t        rf_mode_last_cycled;    /* When we last cycled rate */
    uint32_t        cycle_interval_ms;      /* Current cycle interval (dynamic) */
    uint8_t         rf_mode_cycle_mult;     /* Multiplier for cycle interval */
    uint8_t         scan_index;             /* Current rate scan position */
    bool            lock_rf_mode;           /* Lock to current rate on connect */
    
    /* Packet processing flags (set by ISR, cleared by loop) */
    volatile bool   packet_pending;
    volatile bool   packet_valid;
    volatile int32_t packet_offset_us;  /* Phase offset of received packet */
    
    /* hwTimer synchronization */
    volatile bool   tock_fired;         /* TOCK just fired */
    volatile bool   expecting_packet;   /* Currently in RX window */
    
    /* Radio state */
    uint32_t        current_freq_hz;
    uint8_t         fhss_index;
    
    /* Telemetry */
    bool            tlm_confirm_pending;
    uint8_t         tlm_ratio_counter;
    
    /* Statistics */
    uint32_t        packets_received;
    uint32_t        packets_lost;
    
    /* Callbacks */
    elrs_connect_callback_t     connect_cb;
    elrs_channels_callback_t    channels_cb;
    elrs_wifi_mode_callback_t   wifi_cb;
    
    /* Initialization flag */
    bool            initialized;
    
} elrs = {0};

/*******************************************************************************
 * Forward Declarations
 ******************************************************************************/

static uint32_t get_millis(void);
static void update_connection_state(bool packet_received);
static void handle_connection_lost(bool resume_rx);
static void tentative_connection(uint32_t now);
static void got_connection(uint32_t now);
static void send_crsf_output(void);
static void send_link_stats(void);
static void cycle_rf_mode(uint32_t now);
static void update_phase_lock(void);
static uint8_t min_lq_for_chaos(void);
static void on_packet_received(uint32_t timestamp_us, uint8_t packet_type);
static void on_bind_button_event(bool long_press);

/* Public function forward declarations (for internal use before definition) */
bool elrs_main_is_binding_mode(void);

/*******************************************************************************
 * Binding Button Callback
 * 
 * Called when BTN1 (GPIO_11) is released after being pressed.
 * Long press (>=3 seconds) triggers binding mode entry.
 * 
 * Citation: ExpressLRS rx_main.cpp - EnterBindingMode()
 *   Binding mode uses a well-known UID {0,1,2,3,4,5} to pair with any TX.
 ******************************************************************************/

static void on_bind_button_event(bool long_press)
{
    if (long_press) {
        MAIN_DBG("BTN1 LONG PRESS - Entering binding mode!\n");
        
        /* Toggle binding mode */
        if (elrs_main_is_binding_mode()) {
            MAIN_DBG("Already in binding mode - exiting\n");
            elrs_main_exit_binding_mode();
        } else {
            MAIN_DBG("Entering binding mode...\n");
            elrs_main_enter_binding_mode();
        }
    } else {
        /* Short press - just log for now, could be used for other actions */
        MAIN_DBG("BTN1 short press (ignored)\n");
    }
}

/*******************************************************************************
 * Packet Callback Handler
 * 
 * Called by elrs_rx when a valid packet is received.
 * Sets the packet_pending flag and records timing for PFD with proper slack.
 * 
 * Citation: ExpressLRS rx_main.cpp lines 1112-1113
 *   int32_t slack = std::max(
 *       ExpressLRS_currAirRate_Modparams->interval - 2 * ExpressLRS_currAirRate_RFperfParams->TOA,
 *       (int32_t)PACKET_TO_TOCK_SLACK
 *   );
 *   PFDloop.extEvent(beginProcessing + slack);
 ******************************************************************************/

static void on_packet_received(uint32_t timestamp_us, uint8_t packet_type)
{
    (void)packet_type;  /* May use later for different handling */
    
    /* Calculate slack for PFD timing
     * 
     * Slack is the time between packet RX complete and when TOCK should fire.
     * This accounts for the Time On Air (TOA) of the packet.
     * 
     * Citation: ExpressLRS rx_main.cpp - PFDloop.extEvent() call
     *   slack = max(interval - 2*TOA, PACKET_TO_TOCK_SLACK)
     */
    int32_t slack = PACKET_TO_TOCK_SLACK;  /* Default minimum slack */
    
    if (ELRS_RX.rf_params != NULL) {
        int32_t calculated_slack = (int32_t)ELRS_RX.rf_params->interval_us - 
                                   2 * (int32_t)ELRS_RX.rf_params->toa_us;
        if (calculated_slack > slack) {
            slack = calculated_slack;
        }
    }
    
    /* Store packet time with slack for PFD */
    uint32_t pfd_time = timestamp_us + slack;
    
    /* Set flags for main loop processing */
    elrs.packet_pending = true;
    elrs.packet_valid = true;
    elrs.packet_offset_us = (int32_t)(pfd_time - elrs.last_packet_us);
    
    /* Also call pfd_ext_event directly for proper timing
     * This uses the reference TOCK time recorded in hw_timer_tock()
     */
    pfd_ext_event(pfd_time);
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void elrs_main_get_default_config(elrs_main_config_t *config)
{
    if (config == NULL) return;
    
    memset(config, 0, sizeof(*config));
    
    /* UID for binding phrase "matthew" - BUILD FLAG METHOD
     * 
     * TX Web UI shows: Binding UID 190,147,103,39,214,156
     * This corresponds to MD5("-DMY_BINDING_PHRASE=\"matthew\"")
     *   UID: BE:93:67:27:D6:9C
     *   CRC init = (0xD6 << 8 | 0x9C) ^ (4 << 8) = 0xD69C ^ 0x0400 = 0xD29C
     */
    config->uid[0] = 0xBE;
    config->uid[1] = 0x93;
    config->uid[2] = 0x67;
    config->uid[3] = 0x27;
    config->uid[4] = 0xD6;
    config->uid[5] = 0x9C;
    
    /* Default RF settings */
    config->domain = DOMAIN_FCC915;
    config->initial_rate = RATE_LORA_900_50HZ;  /* Match TX at 50Hz */
    config->tx_power_dbm = 20;
    
    /* Features */
    config->telemetry_enabled = true;
    config->model_match_enabled = false;
    config->model_id = 0;
    config->wifi_on_no_conn = true;
    config->wifi_timeout_ms = WIFI_AUTO_ENTER_MS;
    
    /* Rate switching behavior 
     * Citation: ExpressLRS options.cpp - default lock_on_first_connection = true
     */
    config->lock_on_first_connection = true;
    
    /* CRSF */
    config->crsf_baud_rate = CRSF_BAUDRATE;
}

bool elrs_main_setup(const elrs_main_config_t *config)
{
    MAIN_DBG("=== ELRS Main Setup ===\n");
    
    if (config == NULL) {
        MAIN_DBG("ERROR: NULL config\n");
        return false;
    }
    
    /* Store configuration */
    memcpy(&elrs.config, config, sizeof(elrs.config));
    
    /* Initialize filters */
    lpf_init(&rssi_filter, LPF_BETA_RSSI);
    lpf_init(&snr_filter, LPF_BETA_SNR);
    
    /* Initialize PFD stability tracking filters
     * Citation: ExpressLRS rx_main.cpp lines 146-147
     *   LPF LPF_Offset(2);     // beta=2
     *   LPF LPF_OffsetDx(4);   // beta=4
     */
    lpf_init(&lpf_offset, 2);
    lpf_init(&lpf_offset_dx, 4);
    pfd_prev_raw_offset = 0;
    
    /* Initialize LQ calculator */
    lq_calc_init();
    
    /* Initialize PFD with default interval (will be updated by rate)
     * Citation: ExpressLRS rx_main.cpp - PFDloop initialized at startup
     */
    pfd_init(10000);  /* 100Hz default */
    
    /* Initialize hardware timer
     * Citation: ExpressLRS rx_main.cpp line 2037
     *   hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);
     * 
     * API Note: New API uses separate init and callback registration.
     * Initial interval 10000µs (100Hz) - will be updated when rate is known.
     */
    MAIN_DBG("Initializing hardware timer...\n");
    if (hw_timer_init(10000) != SL_STATUS_OK) {
        MAIN_DBG("ERROR: hwTimer init failed\n");
        return false;
    }
    hw_timer_set_tick_callback(elrs_main_hw_timer_tick);
    hw_timer_set_tock_callback(elrs_main_hw_timer_tock);
    
    /* Initialize ELRS RX subsystem */
    MAIN_DBG("Initializing ELRS RX...\n");
    if (!elrs_rx_init(config->uid, config->domain)) {
        MAIN_DBG("ERROR: ELRS RX init failed\n");
        return false;
    }
    
    /* Register packet callback for timing integration
     * 
     * Citation: ExpressLRS rx_main.cpp - ProcessRFPacket()
     *   Packet arrival timing is used by PFD for phase locking.
     */
    elrs_rx_set_packet_callback(on_packet_received);
    
    /* Set model match if enabled */
    if (config->model_match_enabled) {
        elrs_rx_set_model_match(config->model_id);
    }
    
    /* Initialize CRSF output */
    MAIN_DBG("Initializing CRSF output...\n");
    if (!crsf_init(config->crsf_baud_rate)) {
        MAIN_DBG("WARNING: CRSF init failed (continuing anyway)\n");
        /* Continue anyway - CRSF might not be connected */
    }
    
    /* Initialize status LEDs (LED0 = GPIO_10, LED1 = ULP_GPIO_2)
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
     *   "The LEDs are connected to pin GPIO_10 and ULP_GPIO_2, respectively,
     *    in an active-high configuration."
     * 
     * LED patterns indicate receiver state:
     *   - Disconnected: LED0 slow blink (1Hz)
     *   - Tentative: LED0 fast blink (4Hz)
     *   - Connected: LED0 solid ON
     *   - Binding: Both LEDs alternating (4Hz)
     *   - WiFi: LED1 fast blink (4Hz)
     */
    MAIN_DBG("Initializing status LEDs...\n");
    if (status_led_init() != 0) {
        MAIN_DBG("WARNING: Status LED init failed (continuing anyway)\n");
    } else {
        /* Run boot sequence to indicate successful startup */
        status_led_boot_sequence();
        /* Set initial mode to disconnected */
        status_led_set_mode(LED_MODE_DISCONNECTED);
    }
    
    /* Initialize binding button (BTN1 / GPIO_11)
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
     *   BTN1 is connected to GPIO_11, active-low with 1ms RC debounce
     * 
     * Long press (3+ seconds) triggers binding mode entry.
     */
    MAIN_DBG("Initializing binding button (BTN1)...\n");
    if (bind_button_init() != 0) {
        MAIN_DBG("WARNING: Binding button init failed (continuing anyway)\n");
    } else {
        bind_button_set_callback(on_bind_button_event);
        MAIN_DBG("  Hold BTN1 for 3 seconds to enter binding mode\n");
    }
    
    /* Set initial rate and update PFD interval */
    elrs_rx_set_rate(config->initial_rate);
    const elrs_rf_params_t *params = ELRS_RX.rf_params;
    if (params != NULL) {
        pfd_set_interval(params->interval_us);
    }
    
    /* Initialize state */
    elrs.mode = ELRS_MODE_RX;
    elrs.conn_state = ELRS_DISCONNECTED;
    elrs.packet_pending = false;
    elrs.packet_valid = false;
    elrs.tock_fired = false;
    elrs.expecting_packet = false;
    elrs.initialized = true;
    
    uint32_t now = get_millis();
    elrs.last_packet_ms = now;
    elrs.disconnected_ms = now;
    elrs.rf_mode_last_cycled = now;
    
    /* Initialize rate cycling state
     * Citation: ExpressLRS rx_main.cpp lines 1579-1591
     *   scanIndex = config.GetRateInitialIdx();
     *   RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;
     * 
     * Start with the initial rate configured, and use a slower multiplier
     * initially (half of slow) to give the configured rate more time to connect.
     */
    elrs.scan_index = config->initial_rate;
    elrs.rf_mode_cycle_mult = RFMODE_CYCLE_MULTIPLIER_SLOW / 2;  /* Start slow on initial rate */
    elrs.lock_rf_mode = false;
    
    /* Calculate initial cycle interval based on FHSS parameters
     * Citation: ExpressLRS rx_main.cpp line 354
     *   cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);
     * Wait for 110% of time it takes to cycle through all freqs in FHSS table
     */
    if (params != NULL) {
        uint32_t interval_us = params->interval_us;
        uint8_t hop_interval = params->fhss_hop_interval;
        uint32_t num_fhss = FHSSgetChannelCount();
        elrs.cycle_interval_ms = ((uint32_t)11U * num_fhss * hop_interval * interval_us) / (10U * 1000U);
        MAIN_DBG("Cycle interval: %" PRIu32 " ms (fhss=%lu, hop=%u, interval=%lu)\n", 
                 elrs.cycle_interval_ms, num_fhss, hop_interval, interval_us);
    } else {
        elrs.cycle_interval_ms = 3000;  /* Default fallback */
    }
    
    /* Start receiver - configure radio and enter RX mode
     * Citation: ExpressLRS rx_main.cpp - setup()
     *   Radio.RXnb() called after initialization to start receiving
     */
    MAIN_DBG("Starting receiver...\n");
    elrs_rx_start();
    
    MAIN_DBG("ELRS Main setup complete\n");
    MAIN_DBG("  UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
             config->uid[0], config->uid[1], config->uid[2],
             config->uid[3], config->uid[4], config->uid[5]);
    MAIN_DBG("  Rate: %d, Interval: %" PRIu32 " µs\n",
             config->initial_rate, params ? params->interval_us : 0);
    
    return true;
}

void elrs_main_shutdown(void)
{
    MAIN_DBG("Shutting down ELRS Main\n");
    
    /* Stop hardware timer */
    hw_timer_stop();
    hw_timer_deinit();
    
    /* Stop radio */
    elrs_rx_stop();
    
    /* Deinit CRSF */
    crsf_deinit();
    
    elrs.initialized = false;
    elrs.mode = ELRS_MODE_IDLE;
}

/*******************************************************************************
 * Main Loop
 * 
 * Citation: ExpressLRS rx_main.cpp - loop()
 *   Main processing loop handles:
 *   - Connection state updates
 *   - CRSF output
 *   - Rate cycling when disconnected
 ******************************************************************************/

bool elrs_main_loop(void)
{
    uint32_t now;
    bool packet_processed = false;
    
    if (!elrs.initialized) {
        return false;
    }
    
    now = get_millis();
    
    /***************************************************************************
     * CRITICAL: Call elrs_rx_loop() to poll radio IRQ and process packets
     * 
     * Citation: ExpressLRS rx_main.cpp - loop()
     *   The main loop must poll the radio for IRQs and process received packets.
     *   elrs_rx_loop() reads IRQ status, validates CRC, and calls our callback.
     * 
     * FIX: This was missing, causing no packets to be processed!
     * The packet_pending flag is set by on_packet_received() callback which is
     * invoked by elrs_rx_loop() when a valid packet is received.
     **************************************************************************/
    elrs_rx_loop();
    
    /***************************************************************************
     * ELRS 4.0: Handle Deferred Rate Change from SYNC Packet
     * 
     * Citation: ExpressLRS rx_main.cpp lines 2082-2095 - loop()
     *   if (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRateIndex || SwitchModePending)
     *   {
     *       SetRFLinkRate(ExpressLRS_nextAirRateIndex, SwitchModePending);
     *       SwitchModePending = false;
     *   }
     * 
     * Rate changes from SYNC packets are deferred to main loop because:
     *   1. Radio reconfiguration in packet handler causes PFD timing issues
     *   2. hwTimer interval must be updated atomically with rate change
     *   3. Avoids race conditions with FHSS hopping in TOCK callback
     **************************************************************************/
    if (ELRS_RX.switch_mode_pending || ELRS_RX.next_rate != ELRS_RX.current_rate) {
        elrs_rate_index_t new_rate = ELRS_RX.next_rate;
        
        /* Validate and apply the rate change */
        if (new_rate < RATE_MAX) {
            MAIN_DBG("Applying deferred rate change: %d -> %d\n", 
                     ELRS_RX.current_rate, new_rate);
            
            /* Citation: ExpressLRS rx_main.cpp - SetRFLinkRate()
             *   1. Update current rate index
             *   2. Get RF params for new rate
             *   3. Reconfigure radio
             *   4. Update hwTimer interval
             *   5. Update PFD interval
             */
            ELRS_RX.current_rate = new_rate;
            ELRS_RX.rf_params = elrs_rx_get_rf_params(new_rate);
            
            if (ELRS_RX.rf_params != NULL) {
                /* Reconfigure radio for new rate */
                elrs_rx_config_radio(ELRS_RX.rf_params, ELRS_RX.current_freq_hz);
                
                /* Update hwTimer interval for new rate
                 * Citation: ExpressLRS rx_main.cpp - SetRFLinkRate()
                 *   hwTimer::updateInterval(ModParams->interval);
                 */
                hw_timer_set_interval(ELRS_RX.rf_params->interval_us);
                
                /* Update PFD interval */
                pfd_set_interval(ELRS_RX.rf_params->interval_us);
                
                /* Update cycle interval for potential future rate cycling */
                uint32_t interval_us = ELRS_RX.rf_params->interval_us;
                uint8_t hop_interval = ELRS_RX.rf_params->fhss_hop_interval;
                uint32_t num_fhss = FHSSgetChannelCount();
                elrs.cycle_interval_ms = ((uint32_t)11U * num_fhss * hop_interval * interval_us) / (10U * 1000U);
                
                MAIN_DBG("  New interval: %lu us, cycle: %lu ms\n",
                         ELRS_RX.rf_params->interval_us, elrs.cycle_interval_ms);
            }
        }
        
        /* Clear the pending flag */
        ELRS_RX.switch_mode_pending = false;
    }
    
    /***************************************************************************
     * Process pending packet (set by radio ISR via elrs_rx_loop callback)
     * 
     * Citation: ExpressLRS rx_main.cpp - ProcessRFPacket()
     **************************************************************************/
    if (elrs.packet_pending) {
        elrs.packet_pending = false;
        
        if (elrs.packet_valid) {
            /* Packet was received and validated */
            elrs.packets_received++;
            elrs.last_packet_ms = now;
            
            /* Feed PFD with phase offset */
            pfd_sample(elrs.packet_offset_us);
            
            /* Apply PFD correction to hwTimer */
            int32_t correction = pfd_calc_result();
            hw_timer_phase_shift(correction);
            
            /* Update LQ */
            lq_calc_packet_received();
            
            /* Update RSSI/SNR filters */
            int8_t rssi, snr;
            elrs_rx_get_packet_status(&rssi, &snr);
            lpf_update(&rssi_filter, rssi);
            lpf_update(&snr_filter, snr);
            
            /*******************************************************************
             * CRITICAL FIX: Reset cycle multiplier on valid packet
             * 
             * Citation: ExpressLRS rx_main.cpp line 1179 - ProcessRFPacket()
             *   // Extend sync duration since we've received a packet at this rate
             *   // but do not extend it indefinitely
             *   RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;
             * 
             * When we receive a valid packet (even before full connection), we
             * should slow down the rate cycling to give this rate more time to
             * fully synchronize. Without this, the RX might cycle away from a
             * rate that's almost working.
             ******************************************************************/
            elrs.rf_mode_cycle_mult = RFMODE_CYCLE_MULTIPLIER_SLOW;
            
            /* Notify channels callback if registered */
            if (elrs.channels_cb != NULL) {
                elrs.channels_cb(elrs_rx_get_channels());
            }
            
            packet_processed = true;
        }
    }
    
    /***************************************************************************
     * Update connection state machine
     **************************************************************************/
    update_connection_state(packet_processed);
    
    /***************************************************************************
     * CRSF output (send at ~250Hz)
     * 
     * Citation: ExpressLRS rx_main.cpp
     *   CRSF output sent after valid packet or at regular interval
     **************************************************************************/
    if ((now - elrs.last_crsf_ms) >= CRSF_SEND_INTERVAL_MS) {
        send_crsf_output();
        elrs.last_crsf_ms = now;
    }
    
    /***************************************************************************
     * Link statistics (send at ~10Hz)
     **************************************************************************/
    if ((now - elrs.last_stats_ms) >= LINK_STATS_INTERVAL_MS) {
        send_link_stats();
        elrs.last_stats_ms = now;
    }
    
    /***************************************************************************
     * Rate cycling when disconnected
     * 
     * Citation: ExpressLRS rx_main.cpp line 1616
     *   if (LockRFmode == false && (now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier))
     * 
     * The cycle interval is dynamically calculated based on FHSS parameters.
     * The multiplier starts high (slow cycling) and drops to 1 (fast cycling)
     * after attempting the initial rate.
     **************************************************************************/
    if (elrs.conn_state == ELRS_DISCONNECTED) {
        uint32_t cycle_timeout = elrs.cycle_interval_ms * elrs.rf_mode_cycle_mult;
        if (!elrs.lock_rf_mode && (now - elrs.rf_mode_last_cycled) > cycle_timeout) {
            cycle_rf_mode(now);
        }
        
        /* WiFi auto-enter check */
        if (elrs.config.wifi_on_no_conn && elrs.config.wifi_timeout_ms > 0) {
            if ((now - elrs.disconnected_ms) >= elrs.config.wifi_timeout_ms) {
                MAIN_DBG("WiFi auto-enter timeout, entering WiFi mode\n");
                elrs_main_enter_wifi_mode();
            }
        }
    }
    
    /***************************************************************************
     * Poll binding button (BTN1)
     * 
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
     *   BTN1 (GPIO_11) for binding mode entry via long press (3+ seconds)
     *
     * The callback (on_bind_button_event) will be invoked on button release
     * if a long press was detected.
     **************************************************************************/
    bind_button_poll();
    
    /***************************************************************************
     * Update status LEDs
     * 
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4
     *   LED0 (GPIO_10) and LED1 (ULP_GPIO_2) indicate receiver state.
     *   Must call periodically for blink patterns to work.
     **************************************************************************/
    status_led_update();
    
    return packet_processed;
}

/*******************************************************************************
 * Connection State Management - ELRS 4.0 Exact Implementation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 802-879
 *   LostConnection(), TentativeConnection(), GotConnection()
 * 
 * Citation: ExpressLRS rx_main.cpp lines 2097-2124
 *   Main loop connection state checks
 ******************************************************************************/

/**
 * @brief Calculate minimum LQ threshold for connection validation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 226-240 - minLqForChaos()
 *   Determines the minimum number of CRC-passing packets we could receive
 *   on a single channel out of 100 packets that fill the LQcalc span.
 *   The LQ must be GREATER THAN this value to confirm we're truly connected
 *   and not just receiving on a single frequency by chance.
 */
static uint8_t min_lq_for_chaos(void)
{
    /* Citation: ExpressLRS rx_main.cpp lines 237-239
     * FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
     * With interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
     */
    const uint32_t numfhss = FHSSgetChannelCount();
    const uint8_t interval = (ELRS_RX.rf_params != NULL) ? 
                             ELRS_RX.rf_params->fhss_hop_interval : 4;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

/**
 * @brief Update PFD phase tracking filters
 * 
 * Citation: ExpressLRS rx_main.cpp lines 607-646 - updatePhaseLock()
 *   Called from TOCK callback to update phase offset filters.
 *   LPF_Offset and LPF_OffsetDx track phase stability.
 */
static void update_phase_lock(void)
{
    if (elrs.conn_state == ELRS_DISCONNECTED || !pfd_is_locked()) {
        return;
    }
    
    /* Get raw offset from PFD
     * Citation: ExpressLRS rx_main.cpp line 611
     *   int32_t RawOffset = PFDloop.calcResult();
     */
    int32_t raw_offset = pfd_get_raw_offset();
    
    /* Update filtered offset
     * Citation: ExpressLRS rx_main.cpp line 612
     *   int32_t Offset = LPF_Offset.update(RawOffset);
     */
    int32_t offset = lpf_update(&lpf_offset, raw_offset);
    
    /* Update derivative (rate of change)
     * Citation: ExpressLRS rx_main.cpp line 613
     *   int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
     */
    int32_t offset_dx = lpf_update(&lpf_offset_dx, raw_offset - pfd_prev_raw_offset);
    pfd_prev_raw_offset = raw_offset;
    
    /* Frequency offset adjustment when locked
     * Citation: ExpressLRS rx_main.cpp lines 616-630
     *   if (RXtimerState == tim_locked) { ... adjust freq offset ... }
     */
    if (ELRS_RX.timer_state == RX_TIMER_CONNECTED) {
        /* Limit rate of freq offset adjustment - only every 8th nonce */
        if ((ELRS_RX.nonce % 8) == 0) {
            if (offset > 0) {
                hw_timer_inc_freq_offset(1);   /* Timer slow - increase interval */
            } else if (offset < 0) {
                hw_timer_inc_freq_offset(-1);  /* Timer fast - decrease interval */
            }
        }
    }
    
    /* Phase shift based on connection state
     * Citation: ExpressLRS rx_main.cpp lines 632-639
     */
    if (elrs.conn_state != ELRS_CONNECTED) {
        hw_timer_phase_shift(raw_offset >> 1);  /* Faster correction when not connected */
    } else {
        hw_timer_phase_shift(offset >> 2);      /* Slower, smoother when connected */
    }
    
    (void)offset_dx;  /* Used in GotConnection() check */
}

/**
 * @brief Handle lost connection - ELRS 4.0 exact implementation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 802-831 - LostConnection()
 */
static void handle_connection_lost(bool resume_rx)
{
    MAIN_DBG("Lost conn fc=%ld fo=%ld\n", 
             (long)pfd_get_freq_offset(), (long)hw_timer_get_freq_offset());
    
    /* Citation: ExpressLRS rx_main.cpp line 806
     * setConnectionState(disconnected);
     */
    elrs.conn_state = ELRS_DISCONNECTED;
    ELRS_RX.timer_state = RX_TIMER_DISCONNECTED;
    elrs.disconnected_ms = get_millis();
    
    /* Citation: ExpressLRS rx_main.cpp lines 808-816 */
    hw_timer_reset_freq_offset();
    pfd_prev_raw_offset = 0;
    elrs.got_connection_ms = 0;
    
    /* Reset LQ */
    lq_calc_reset();
    
    /* Reset PFD filters
     * Citation: ExpressLRS rx_main.cpp lines 814-815
     *   LPF_Offset.init(0);
     *   LPF_OffsetDx.init(0);
     */
    lpf_reset(&lpf_offset);
    lpf_reset(&lpf_offset_dx);
    lpf_seed(&lpf_offset, 0);
    lpf_seed(&lpf_offset_dx, 0);
    
    /* Reset RSSI/SNR filters */
    lpf_reset(&rssi_filter);
    lpf_reset(&snr_filter);
    
    /* Reset state flags */
    ELRS_RX.got_sync = false;
    ELRS_RX.model_match = false;
    ELRS_RX.nonce_sync_locked = false;
    
    /* Citation: ExpressLRS rx_main.cpp lines 820-831
     * Stop timer, set rate, optionally resume RX
     */
    if (hw_timer_is_running()) {
        hw_timer_pause();
    }
    
    /* CRITICAL FIX: Use next_rate (pending rate), not scan_index
     * 
     * Citation: ExpressLRS rx_main.cpp line 825 - LostConnection()
     *   SetRFLinkRate(ExpressLRS_nextAirRateIndex, false);
     * 
     * When losing connection, ELRS stays on the rate that was pending
     * (from SYNC packet or current operating rate), not jumping to
     * the scan index. This preserves the current working rate.
     * 
     * The scan_index is only used during cycleRfMode() when actively
     * cycling through rates looking for a TX.
     */
    elrs_rx_set_rate(ELRS_RX.next_rate);
    
    /* Return to initial FHSS frequency for scanning */
    uint32_t init_freq = FHSSgetInitialFreq();
    elrs_rx_set_frequency(init_freq);
    FHSSsetCurrIndex(0);
    
    if (resume_rx) {
        elrs_rx_enter_rx_mode(0);
    }
    
    /* Update LED */
    status_led_set_mode(LED_MODE_DISCONNECTED);
}

/**
 * @brief Enter tentative connection state - ELRS 4.0 exact implementation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 834-856 - TentativeConnection()
 */
static void tentative_connection(uint32_t now)
{
    /* Citation: ExpressLRS rx_main.cpp line 836 - Reset PFD first */
    pfd_reset();
    
    /* Citation: ExpressLRS rx_main.cpp line 837 */
    elrs.conn_state = ELRS_TENTATIVE;
    ELRS_RX.model_match = false;  /* Will be set by SYNC processing */
    ELRS_RX.timer_state = RX_TIMER_DISCONNECTED;  /* Still disconnected until GotConnection */
    
    MAIN_DBG("Tentative conn\n");
    
    /* Citation: ExpressLRS rx_main.cpp lines 841-842 */
    pfd_prev_raw_offset = 0;
    lpf_reset(&lpf_offset);
    lpf_seed(&lpf_offset, 0);
    
    /* Citation: ExpressLRS rx_main.cpp line 844 - Reset cycle timer */
    elrs.rf_mode_last_cycled = now;
    elrs.connected_ms = now;
    
    /* Citation: ExpressLRS rx_main.cpp line 847
     * Use this rate as the initial rate next time if we connected on it
     * config.SetRateInitialIdx(ExpressLRS_nextAirRateIndex);
     */
    elrs.config.initial_rate = elrs_rx_get_rate();
    
    /* Start hwTimer - caller handles this after TentativeConnection returns
     * Citation: ExpressLRS rx_main.cpp lines 854-856
     *   The caller MUST call hwTimer::resume()
     * 
     * API Note: New API separates interval setting from start.
     */
    if (ELRS_RX.rf_params != NULL) {
        hw_timer_set_interval(ELRS_RX.rf_params->interval_us);
        pfd_set_interval(ELRS_RX.rf_params->interval_us);
        hw_timer_start();
    }
    
    /* Use fast PFD for quick lock */
    pfd_fast_mode();
    
    /* Update LED to fast blink for tentative state */
    status_led_set_mode(LED_MODE_TENTATIVE);
}

/**
 * @brief Confirm connection - ELRS 4.0 exact implementation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 858-879 - GotConnection()
 */
static void got_connection(uint32_t now)
{
    /* Citation: ExpressLRS rx_main.cpp lines 860-863 */
    if (elrs.conn_state == ELRS_CONNECTED) {
        return;  /* Already connected */
    }
    
    /* Citation: ExpressLRS rx_main.cpp line 865
     * LockRFmode = firmwareOptions.lock_on_first_connection;
     */
    elrs.lock_rf_mode = elrs.config.lock_on_first_connection;
    
    /* Citation: ExpressLRS rx_main.cpp lines 867-869 */
    elrs.conn_state = ELRS_CONNECTED;
    ELRS_RX.timer_state = RX_TIMER_TENTATIVE;  /* tim_tentative until stable */
    elrs.got_connection_ms = now;
    
    /* Switch to normal PFD tracking (slower alpha for stability) */
    pfd_normal_mode();
    
    MAIN_DBG("Got conn\n");
    
    /* Update LED to solid for connected state */
    status_led_set_mode(LED_MODE_CONNECTED);
}

/**
 * @brief Main connection state update - ELRS 4.0 exact implementation
 * 
 * Citation: ExpressLRS rx_main.cpp lines 2077-2124 - loop() connection checks
 */
static void update_connection_state(bool packet_received)
{
    uint32_t now = get_millis();
    elrs_connection_state_t old_state = elrs.conn_state;
    
    /* Citation: ExpressLRS rx_main.cpp lines 2082-2095
     * Handle forced rate change when connected
     */
    /* (Simplified - forced rate change not yet implemented) */
    
    /* Citation: ExpressLRS rx_main.cpp lines 2097-2103
     * Check for bad sync in tentative state (RxLockTimeoutMs abort)
     */
    if (elrs.conn_state == ELRS_TENTATIVE) {
        uint32_t rx_lock_timeout_ms = 2500;  /* Default fallback */
        if (ELRS_RX.rf_params != NULL) {
            /* ELRS 4.0: Use rate-specific RxLockTimeoutMs from RF params table
             * Citation: ExpressLRS common.cpp - expresslrs_rf_pref_params table
             */
            rx_lock_timeout_ms = ELRS_RX.rf_params->rx_lock_timeout_ms;
        }
        
        if ((now - elrs.last_sync_ms) > rx_lock_timeout_ms && elrs.last_sync_ms != 0) {
            MAIN_DBG("Bad sync, aborting\n");
            handle_connection_lost(true);
            elrs.rf_mode_last_cycled = now;
            elrs.last_sync_ms = now;
            goto state_done;
        }
    }
    
    /* Handle SYNC packet reception -> TentativeConnection
     * Citation: ExpressLRS rx_main.cpp - ProcessRfPacket_SYNC calls TentativeConnection
     */
    if (elrs.conn_state == ELRS_DISCONNECTED && packet_received && ELRS_RX.got_sync) {
        tentative_connection(now);
        elrs.last_sync_ms = now;
        goto state_done;
    }
    
    /* Citation: ExpressLRS rx_main.cpp lines 2107-2111
     * Check for connection lost when connected
     */
    if (elrs.conn_state == ELRS_CONNECTED) {
        uint32_t disconnect_timeout_ms = CONN_LOST_TIMEOUT_MS;  /* Default fallback */
        if (ELRS_RX.rf_params != NULL) {
            /* ELRS 4.0: Use rate-specific DisconnectTimeoutMs from RF params table
             * Citation: ExpressLRS common.cpp - expresslrs_rf_pref_params table
             */
            disconnect_timeout_ms = ELRS_RX.rf_params->disconnect_timeout_ms;
        }
        
        if ((int32_t)(now - elrs.last_packet_ms) > (int32_t)disconnect_timeout_ms) {
            handle_connection_lost(true);
            goto state_done;
        }
    }
    
    /* Citation: ExpressLRS rx_main.cpp line 2113
     * Check for transition from tentative to connected (GotConnection criteria)
     * 
     * ELRS 4.0 EXACT CODE:
     *   if ((connectionState == tentative) && 
     *       (abs(LPF_OffsetDx.value()) <= 10) && 
     *       (LPF_Offset.value() < 100) && 
     *       (LQCalc.getLQRaw() > minLqForChaos()))
     * 
     * Note: LPF_Offset uses raw value (not abs) - offset is typically positive
     * when we receive early, so < 100 check is sufficient.
     */
    if (elrs.conn_state == ELRS_TENTATIVE) {
        int32_t offset_dx = lpf_get_value(&lpf_offset_dx);
        int32_t offset = lpf_get_value(&lpf_offset);
        uint8_t lq_raw = lq_calc_get_lq_raw();
        uint8_t min_lq = min_lq_for_chaos();
        
        /* ELRS 4.0 exact checks - note offset is NOT abs() */
        bool dx_stable = (offset_dx >= -10 && offset_dx <= 10);  /* abs(OffsetDx) <= 10 */
        bool offset_small = (offset < 100);  /* Offset < 100 (NOT abs!) */
        bool lq_good = (lq_raw > min_lq);
        
        if (dx_stable && offset_small && lq_good) {
            got_connection(now);
        }
    }
    
    /* Citation: ExpressLRS rx_main.cpp lines 2120-2124
     * Timer state transition: tim_tentative -> tim_locked
     */
    if (ELRS_RX.timer_state == RX_TIMER_TENTATIVE) {
        if ((now - elrs.got_connection_ms) > CONSIDER_CONN_GOOD_MS) {
            int32_t offset_dx = lpf_get_value(&lpf_offset_dx);
            if (offset_dx >= -5 && offset_dx <= 5) {
                ELRS_RX.timer_state = RX_TIMER_CONNECTED;
                MAIN_DBG("Timer locked\n");
            }
        }
    }

state_done:
    /* Notify callback on state change */
    if (old_state != elrs.conn_state && elrs.connect_cb != NULL) {
        elrs.connect_cb(elrs.conn_state);
    }
}

/*******************************************************************************
 * hwTimer Callbacks (ISR Context!)
 * 
 * Citation: ExpressLRS rx_main.cpp
 *   HWtimerCallbackTick - called mid-interval
 *   HWtimerCallbackTock - called when packet expected
 * 
 * WARNING: These are called from interrupt context!
 ******************************************************************************/

void elrs_main_hw_timer_tick(void)
{
    /***************************************************************************
     * TICK - Mid-interval processing
     * 
     * Citation: ExpressLRS rx_main.cpp - HWtimerCallbackTick()
     *   - Calculate LQ for this interval
     *   - Prepare for telemetry
     **************************************************************************/
    
    /* LQ: Count this slot as missed if no packet was received */
    if (!elrs.packet_valid) {
        lq_calc_packet_missed();
    }
    
    /* Increment expected counter */
    lq_calc_inc_expected();
    
    /* Clear packet flags for next interval */
    elrs.packet_valid = false;
    elrs.expecting_packet = false;
    
    /* Handle telemetry transmission if needed
     * Citation: ExpressLRS rx_main.cpp - TLM handling in Tick
     */
    elrs.tlm_ratio_counter++;
    /* TODO: Add telemetry transmission */
}

void elrs_main_hw_timer_tock(void)
{
    /***************************************************************************
     * TOCK - Packet expected time
     * 
     * Citation: ExpressLRS rx_main.cpp - HWtimerCallbackTock()
     *   - Increment OtaNonce FIRST (before FHSS)
     *   - Handle FHSS hopping (only when OtaNonce % FHSShopInterval == 0)
     *   - Record timestamp for PFD
     *   - Start RX if not already receiving
     * 
     * CRITICAL FIX: Nonce must be incremented in TOCK callback, NOT on packet
     * receipt. The nonce is used for:
     *   1. CRC calculation (must match TX nonce)
     *   2. FHSS hop timing (TX and RX must hop at same nonce values)
     * 
     * Citation: ExpressLRS rx_main.cpp line 785
     *   void ICACHE_RAM_ATTR HWtimerCallbackTock() {
     *       ...
     *       OtaNonce++;
     *       HandleFHSS();
     *       ...
     *   }
     **************************************************************************/
    
    elrs.tock_fired = true;
    elrs.expecting_packet = true;
    
    /* Record TOCK timestamp for PFD offset calculation
     * 
     * Citation: ExpressLRS rx_main.cpp - HWtimerCallbackTock()
     *   The TOCK time is recorded as the reference for PFD phase calculation.
     *   When a packet arrives, its timing is compared against this TOCK time.
     */
    uint32_t tock_time = hw_timer_get_micros();
    elrs.last_packet_us = tock_time;
    pfd_set_tock_time(tock_time);
    
    /* Increment nonce FIRST - before FHSS handling
     * Citation: ExpressLRS rx_main.cpp - OtaNonce++ happens at TOCK
     * 
     * ELRS 4.0 Nonce Sync: We also increment nonce_rx which tracks our local
     * nonce counter. This is compared against TX nonce in SYNC packets to
     * verify synchronization before allowing connection.
     * 
     * Citation: ExpressLRS 4.0 Release Notes
     *   "More robust syncing - The OTA now requires a counter synchronization 
     *    lock to function."
     */
    /* CRITICAL FIX: Only increment nonce when NOT disconnected
     *
     * Citation: ExpressLRS rx_main.cpp HWtimerCallbackTock()
     *   In ELRS, the timer only runs when connected/tentative.
     *   When disconnected, the RX sits at OtaNonce=0 waiting for SYNC.
     *   After SYNC, OtaNonce is set to sync->nonce, then increments.
     *
     * BUG FIX: Previously OtaNonce was incrementing every TOCK even when
     * disconnected, causing CRC mismatches for RC_DATA packets (which use
     * OtaNonce as CRC seed). SYNC packets worked because they always use nonce=0.
     */
    if (elrs.conn_state != ELRS_DISCONNECTED) {
        OtaNonce++;
        ELRS_RX.nonce = OtaNonce;
        ELRS_RX.nonce_rx++;  /* ELRS 4.0: Track RX's local nonce for sync verification */
    }
    
    /* Handle FHSS - only hop when connected and at hop interval
     * 
     * Citation: ExpressLRS rx_main.cpp - HandleFHSS()
     *   uint8_t modresultFHSS = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
     *   if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || 
     *       InBindingMode || 
     *       (modresultFHSS != 0) || 
     *       (connectionState == disconnected))
     *   {
     *       return;  // DON'T HOP
     *   }
     *   Radio.SetFrequencyReg(FHSSgetNextFreq());
     */
    if (ELRS_RX.rf_params != NULL && 
        ELRS_RX.rf_params->fhss_hop_interval != 0 &&
        elrs.conn_state != ELRS_DISCONNECTED &&
        !elrs_rx_is_binding_mode())
    {
        uint8_t hop_interval = ELRS_RX.rf_params->fhss_hop_interval;
        if ((OtaNonce % hop_interval) == 0)
        {
            /* Time to hop to next frequency */
            uint32_t next_freq = FHSSgetNextFreq();
            elrs_rx_set_frequency(next_freq);
            elrs.fhss_index = FHSSgetCurrIndex();
        }
    }
    
    /* Update phase lock - MUST be called in TOCK
     * 
     * Citation: ExpressLRS rx_main.cpp line 789 - HWtimerCallbackTock()
     *   OtaNonce++;
     *   HandleFHSS();
     *   updateDiversity();
     *   bool tlmSent = HandleSendDataDl();
     *   updatePhaseLock();  // <-- Called here after FHSS
     *
     * This updates the LPF filters for phase tracking and applies
     * frequency/phase corrections based on PFD measurements.
     */
    update_phase_lock();
    
    /* Ensure radio is in RX mode */
    elrs_rx_enter_rx_mode(0);
}

/*******************************************************************************
 * Rate Cycling
 * 
 * Citation: ExpressLRS rx_main.cpp lines 1610-1638 - cycleRfMode()
 *   
 * When disconnected, cycles through all supported RF rates looking for a TX.
 * Uses scanIndex to iterate through rates, skipping unsupported ones.
 * After the first cycle, switches to fast cycling (multiplier = 1).
 ******************************************************************************/

/**
 * @brief Check if a rate is supported on this hardware
 * 
 * Citation: ExpressLRS rx_main.cpp - isSupportedRFRate()
 * For 900MHz single radio (LR1121), we support all 900MHz LoRa rates.
 * 
 * The LR1121 supports all 900MHz rates. For 2.4GHz or dual-band rates,
 * additional hardware configuration would be required.
 */
static bool is_supported_rf_rate(elrs_rate_index_t rate)
{
    /* For 900MHz domain with LR1121, support all 900 LoRa rates
     * Citation: ExpressLRS common.h - expresslrs_RFrates_e
     */
    switch (rate) {
        /* All 900MHz rates supported on LR1121 */
        case RATE_LORA_900_500HZ:       /* Fastest 900MHz */
        case RATE_LORA_900_333HZ_8CH:   /* 8-channel variant */
        case RATE_LORA_900_250HZ:
        case RATE_LORA_900_200HZ_8CH:   /* 8-channel variant */
        case RATE_LORA_900_200HZ:
        case RATE_LORA_900_150HZ:
        case RATE_LORA_900_100HZ_8CH:   /* 8-channel variant */
        case RATE_LORA_900_100HZ:
        case RATE_LORA_900_50HZ:        /* Default/Binding rate */
        case RATE_LORA_900_25HZ:        /* Long range */
        case RATE_LORA_900_50HZ_DVDA:   /* DVDA mode */
            return true;
            
        /* 2.4GHz rates - NOT supported on single 900MHz LR1121 hardware */
        case RATE_LORA_2G4_500HZ:
        case RATE_LORA_2G4_333HZ_8CH:
        case RATE_LORA_2G4_250HZ:
        case RATE_LORA_2G4_200HZ_8CH:
        case RATE_LORA_2G4_200HZ:
        case RATE_LORA_2G4_150HZ:
        case RATE_LORA_2G4_100HZ_8CH:
        case RATE_LORA_2G4_100HZ:
        case RATE_LORA_2G4_50HZ:
        case RATE_LORA_2G4_25HZ:
            return false;
            
        /* Dual-band rates - require dual radio hardware */
        case RATE_LORA_DUAL_150HZ:
        case RATE_LORA_DUAL_100HZ_8CH:
            return false;
            
        default:
            return false;
    }
}

static void cycle_rf_mode(uint32_t now)
{
    /* Update timing
     * Citation: ExpressLRS rx_main.cpp line 1618
     *   RFmodeLastCycled = now;
     */
    elrs.rf_mode_last_cycled = now;
    
    /* Set the rate before incrementing scanIndex (use current scanIndex)
     * Citation: ExpressLRS rx_main.cpp line 1621
     *   SetRFLinkRate(scanIndex % RATE_MAX, false);
     */
    elrs_rate_index_t next_rate = elrs.scan_index % RATE_MAX;
    
    MAIN_DBG("Rate cycling: scanIndex=%u -> rate=%d (interval=%lu us)\n", 
             elrs.scan_index, next_rate,
             elrs_rx_get_rf_params(next_rate)->interval_us);
    
    /* Set the new rate */
    elrs_rx_set_rate(next_rate);
    
    /* Update PFD interval for new rate */
    const elrs_rf_params_t *params = ELRS_RX.rf_params;
    if (params != NULL) {
        pfd_set_interval(params->interval_us);
        
        /* Recalculate cycle interval for new rate
         * Citation: ExpressLRS rx_main.cpp line 354
         *   cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);
         */
        uint32_t interval_us = params->interval_us;
        uint8_t hop_interval = params->fhss_hop_interval;
        uint32_t num_fhss = FHSSgetChannelCount();
        elrs.cycle_interval_ms = ((uint32_t)11U * num_fhss * hop_interval * interval_us) / (10U * 1000U);
        
        /* Ensure minimum cycle interval */
        if (elrs.cycle_interval_ms < 100) {
            elrs.cycle_interval_ms = 100;
        }
    }
    
    /* Reset LQ for fresh measurement at this rate
     * Citation: ExpressLRS rx_main.cpp lines 1622-1623
     *   LQCalc.reset100();
     */
    lq_calc_reset();
    
    /* Increment scanIndex for next cycle
     * Citation: ExpressLRS rx_main.cpp line 1625
     *   scanIndex++;
     */
    elrs.scan_index++;
    
    /* Skip unsupported rates
     * Citation: ExpressLRS rx_main.cpp lines 1630-1634
     *   while (!isSupportedRFRate(scanIndex % RATE_MAX))
     *   {
     *       DBGLN("Skip %u", get_elrs_airRateConfig(scanIndex % RATE_MAX)->interval);
     *       scanIndex++;
     *   }
     */
    while (!is_supported_rf_rate(elrs.scan_index % RATE_MAX)) {
        MAIN_DBG("  Skip unsupported rate %d\n", elrs.scan_index % RATE_MAX);
        elrs.scan_index++;
    }
    
    /* Reset to initial FHSS freq */
    uint32_t init_freq = FHSSgetInitialFreq();
    elrs_rx_set_frequency(init_freq);
    FHSSsetCurrIndex(0);
    
    /* CRITICAL: Put radio in RX mode after rate/freq change!
     * Citation: ExpressLRS rx_main.cpp line 1626
     *   Radio.RXnb();
     */
    elrs_rx_enter_rx_mode(0);  /* 0 = continuous RX */
    
    /* Switch to fast cycling after first attempt
     * Citation: ExpressLRS rx_main.cpp line 1637
     *   RFmodeCycleMultiplier = 1;
     */
    elrs.rf_mode_cycle_mult = 1;
}

/*******************************************************************************
 * CRSF Output
 ******************************************************************************/

static void send_crsf_output(void)
{
    const elrs_channel_data_t *ch = elrs_rx_get_channels();
    
    if (ch != NULL) {
        crsf_send_rc_channels(ch->ch);
    }
}

static void send_link_stats(void)
{
    crsf_link_stats_t stats;
    
    stats.uplink_RSSI_1 = (uint8_t)(120 + lpf_get_value(&rssi_filter));  /* Convert to CRSF format */
    stats.uplink_RSSI_2 = stats.uplink_RSSI_1;
    stats.uplink_Link_quality = lq_calc_get_lq();
    stats.uplink_SNR = (int8_t)lpf_get_value(&snr_filter);
    stats.active_antenna = 0;
    stats.rf_Mode = (uint8_t)elrs_rx_get_rate();
    stats.uplink_TX_Power = 0;  /* TODO: Get TX power from telemetry */
    stats.downlink_RSSI_1 = 0;
    stats.downlink_Link_quality = 0;
    stats.downlink_SNR = 0;
    
    crsf_send_link_stats(&stats);
}

/*******************************************************************************
 * Mode Control
 ******************************************************************************/

elrs_mode_t elrs_main_get_mode(void)
{
    return elrs.mode;
}

bool elrs_main_enter_wifi_mode(void)
{
    MAIN_DBG("Entering WiFi mode\n");
    
    /* Stop radio */
    elrs_rx_stop();
    hw_timer_stop();
    
    elrs.mode = ELRS_MODE_WIFI;
    
    /* Set LED to WiFi mode pattern (LED1 fast blink)
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4 - LED1 on ULP_GPIO_2
     */
    status_led_set_mode(LED_MODE_WIFI);
    
    /* Call WiFi callback to start HTTP server
     * Citation: ExpressLRS devWIFI.cpp - WifiService() 
     *   When wifi_start is triggered, starts AP and HTTP server
     */
    if (elrs.wifi_cb != NULL) {
        MAIN_DBG("Invoking WiFi callback to start HTTP server\n");
        elrs.wifi_cb();
    }
    
    return true;
}

bool elrs_main_exit_wifi_mode(void)
{
    MAIN_DBG("Exiting WiFi mode\n");
    
    elrs.mode = ELRS_MODE_RX;
    
    /* Restart radio */
    elrs_rx_start();
    
    /* Restore LED to disconnected state (will update when connected) */
    status_led_set_mode(LED_MODE_DISCONNECTED);
    
    return true;
}

/**
 * @brief Binding Mode Implementation
 * 
 * Citation: ExpressLRS 4.0 rx_main.cpp - EnterBindingMode()
 * 
 * OTA Binding Protocol:
 * 1. Switch to well-known binding UID: {0, 1, 2, 3, 4, 5}
 * 2. Re-initialize FHSS with binding UID seed
 * 3. Listen for SYNC packets on binding frequency
 * 4. Extract real UID from received SYNC packet
 * 5. Save UID to NVM3 and restart with new UID
 * 
 * The binding UID {0,1,2,3,4,5} is used by all ELRS devices when
 * in binding mode, allowing any TX to bind to any RX.
 */

/* Well-known ELRS binding UID
 * Citation: ExpressLRS common.h - BindingUID
 */
static const uint8_t BINDING_UID[6] = {0, 1, 2, 3, 4, 5};

/* Saved UID before entering binding mode (for restoration on exit) */
static uint8_t saved_uid[6];
static bool binding_in_progress = false;

bool elrs_main_enter_binding_mode(void)
{
    MAIN_DBG("=== Entering OTA Binding Mode ===\n");
    
    /* Save current UID for restoration if binding is cancelled */
    const elrs_main_config_t *current_config = &elrs.config;
    memcpy(saved_uid, current_config->uid, 6);
    
    MAIN_DBG("Saved current UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
             saved_uid[0], saved_uid[1], saved_uid[2],
             saved_uid[3], saved_uid[4], saved_uid[5]);
    
    /* Stop current receiver operation */
    hw_timer_stop();
    
    /* Set mode to binding */
    elrs.mode = ELRS_MODE_BINDING;
    binding_in_progress = true;
    
    /* Switch to binding UID
     * Citation: ExpressLRS rx_main.cpp - EnterBindingMode()
     *   Sets UID to BindingUID and re-initializes FHSS
     */
    MAIN_DBG("Switching to binding UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
             BINDING_UID[0], BINDING_UID[1], BINDING_UID[2],
             BINDING_UID[3], BINDING_UID[4], BINDING_UID[5]);
    
    /* Update global UID used by OTA CRC
     * Citation: ExpressLRS common.cpp - UID global array
     */
    extern uint8_t UID[6];  /* Global UID from elrs_globals.c */
    memcpy(UID, BINDING_UID, 6);
    
    /* Re-initialize FHSS with binding UID seed
     * Citation: ExpressLRS common.cpp uidMacSeedGet()
     *   macSeed = (UID[2] << 24) + (UID[3] << 16) + (UID[4] << 8) + (UID[5] ^ OTA_VERSION_ID)
     */
    uint32_t binding_seed = ((uint32_t)BINDING_UID[2] << 24) | 
                            ((uint32_t)BINDING_UID[3] << 16) | 
                            ((uint32_t)BINDING_UID[4] << 8) | 
                            ((uint32_t)BINDING_UID[5] ^ OTA_VERSION_ID);
    
    MAIN_DBG("Binding FHSS seed: 0x%08" PRIX32 "\n", binding_seed);
    FHSSrandomiseFHSSsequence(binding_seed);
    
    /* Re-initialize OTA CRC with binding UID */
    OtaUpdateCrcInitFromUid();
    
    /* Configure RX with binding UID for SYNC packet validation */
    elrs_rx_set_binding_uid(BINDING_UID);
    
    /* Get initial binding frequency */
    uint32_t bind_freq = FHSSgetInitialFreq();
    MAIN_DBG("Binding frequency: %" PRIu32 " Hz\n", bind_freq);
    
    /* Configure radio for 50Hz rate (commonly used for binding)
     * Citation: ExpressLRS - Uses slowest rate for best range during binding
     */
    const elrs_rf_params_t *bind_params = elrs_rx_get_rf_params(RATE_LORA_900_50HZ);
    elrs_rx_config_radio(bind_params, bind_freq);
    
    /* Enter continuous RX mode to listen for binding packets */
    elrs_rx_enter_rx_mode(0);  /* 0 = continuous RX */
    
    MAIN_DBG("Binding mode active - listening for TX binding packets...\n");
    MAIN_DBG("Put TX in binding mode and wait for connection\n");
    
    /* Set LED to binding mode pattern (alternating blink)
     * Citation: ug590-brd2708a-user-guide.pdf Section 3.4 - LED0 & LED1 available
     */
    status_led_set_mode(LED_MODE_BINDING);
    
    return true;
}

void elrs_main_exit_binding_mode(void)
{
    MAIN_DBG("=== Exiting Binding Mode ===\n");
    
    binding_in_progress = false;
    elrs.mode = ELRS_MODE_RX;
    
    /* Restore original UID and restart receiver */
    extern uint8_t UID[6];
    memcpy(UID, saved_uid, 6);
    
    MAIN_DBG("Restored UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
             saved_uid[0], saved_uid[1], saved_uid[2],
             saved_uid[3], saved_uid[4], saved_uid[5]);
    
    /* Re-initialize with saved UID */
    uint32_t fhss_seed = ((uint32_t)saved_uid[2] << 24) | 
                         ((uint32_t)saved_uid[3] << 16) | 
                         ((uint32_t)saved_uid[4] << 8) | 
                         ((uint32_t)saved_uid[5] ^ OTA_VERSION_ID);
    FHSSrandomiseFHSSsequence(fhss_seed);
    OtaUpdateCrcInitFromUid();
    
    /* Clear binding UID in RX */
    elrs_rx_set_binding_uid(NULL);
    
    /* Restart normal receiver operation */
    elrs_rx_start();
    
    /* Restore LED to disconnected state (will change to connected if link established) */
    status_led_set_mode(LED_MODE_DISCONNECTED);
    
    MAIN_DBG("Binding mode exited, normal operation resumed\n");
}

/**
 * @brief Complete binding with received UID
 * 
 * Called by elrs_rx when a valid SYNC packet is received during binding mode.
 * The SYNC packet contains UID bytes 4 and 5 from the TX.
 * 
 * @param uid4 UID byte 4 from TX
 * @param uid5 UID byte 5 from TX
 */
void elrs_main_binding_complete(uint8_t uid4, uint8_t uid5)
{
    uint8_t new_uid[6];
    
    MAIN_DBG("=== Binding Complete! ===\n");
    MAIN_DBG("Received TX UID bytes: [4]=0x%02X, [5]=0x%02X\n", uid4, uid5);
    
    /* In ELRS binding, only UID[4] and UID[5] are transmitted in SYNC packets.
     * The full UID must be reconstructed. For simplicity, we use a default
     * pattern for bytes 0-3 and the received bytes for 4-5.
     * 
     * NOTE: In actual ELRS, the full UID is derived from the binding phrase
     * which should be configured separately. This binding mode captures
     * the TX's UID signature for verification purposes.
     * 
     * For full compatibility, users should configure the same binding phrase
     * on both TX and RX via the web interface.
     */
    
    /* Use the binding UID bytes 0-3, with received 4-5 */
    new_uid[0] = BINDING_UID[0];
    new_uid[1] = BINDING_UID[1];
    new_uid[2] = BINDING_UID[2];
    new_uid[3] = BINDING_UID[3];
    new_uid[4] = uid4;
    new_uid[5] = uid5;
    
    MAIN_DBG("New UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
             new_uid[0], new_uid[1], new_uid[2],
             new_uid[3], new_uid[4], new_uid[5]);
    
    /* Stop binding mode */
    binding_in_progress = false;
    elrs.mode = ELRS_MODE_RX;
    
    /* Save new UID to config and NVM3 */
    extern int elrs_config_set_uid(const uint8_t uid[6]);
    extern int elrs_config_save(void);
    
    int result = elrs_config_set_uid(new_uid);
    if (result != 0) {
        MAIN_DBG("ERROR: Failed to set UID: %d\n", result);
        return;
    }
    
    result = elrs_config_save();
    if (result != 0) {
        MAIN_DBG("ERROR: Failed to save config: %d\n", result);
        return;
    }
    
    MAIN_DBG("UID saved to NVM3!\n");
    
    /* Update global UID */
    extern uint8_t UID[6];
    memcpy(UID, new_uid, 6);
    
    /* Re-initialize FHSS with new UID */
    uint32_t fhss_seed = ((uint32_t)new_uid[2] << 24) | 
                         ((uint32_t)new_uid[3] << 16) | 
                         ((uint32_t)new_uid[4] << 8) | 
                         ((uint32_t)new_uid[5] ^ OTA_VERSION_ID);
    
    MAIN_DBG("New FHSS seed: 0x%08" PRIX32 "\n", fhss_seed);
    FHSSrandomiseFHSSsequence(fhss_seed);
    OtaUpdateCrcInitFromUid();
    
    /* Clear binding mode in RX */
    elrs_rx_set_binding_uid(NULL);
    
    /* Restart receiver with new UID */
    elrs_rx_start();
    
    MAIN_DBG("Binding successful! RX now bound to TX.\n");
}

/**
 * @brief Check if currently in binding mode
 */
bool elrs_main_is_binding_mode(void)
{
    return binding_in_progress;
}

/*******************************************************************************
 * Status & Statistics
 ******************************************************************************/

bool elrs_main_is_connected(void)
{
    return (elrs.conn_state == ELRS_CONNECTED);
}

elrs_connection_state_t elrs_main_get_connection_state(void)
{
    return elrs.conn_state;
}

uint8_t elrs_main_get_lq(void)
{
    return lq_calc_get_lq();
}

int8_t elrs_main_get_rssi(void)
{
    return (int8_t)lpf_get_value(&rssi_filter);
}

int8_t elrs_main_get_snr(void)
{
    return (int8_t)lpf_get_value(&snr_filter);
}

const elrs_channel_data_t* elrs_main_get_channels(void)
{
    return elrs_rx_get_channels();
}

void elrs_main_get_link_stats(crsf_link_stats_t *stats)
{
    if (stats == NULL) return;
    
    stats->uplink_RSSI_1 = (uint8_t)(120 + lpf_get_value(&rssi_filter));
    stats->uplink_RSSI_2 = stats->uplink_RSSI_1;
    stats->uplink_Link_quality = lq_calc_get_lq();
    stats->uplink_SNR = (int8_t)lpf_get_value(&snr_filter);
    stats->active_antenna = 0;
    stats->rf_Mode = (uint8_t)elrs_rx_get_rate();
    stats->uplink_TX_Power = 0;
    stats->downlink_RSSI_1 = 0;
    stats->downlink_Link_quality = 0;
    stats->downlink_SNR = 0;
}

elrs_rate_index_t elrs_main_get_rate(void)
{
    return elrs_rx_get_rate();
}

bool elrs_main_set_rate(elrs_rate_index_t rate)
{
    return elrs_rx_set_rate(rate);
}

void elrs_main_set_connect_callback(elrs_connect_callback_t callback)
{
    elrs.connect_cb = callback;
}

void elrs_main_set_channels_callback(elrs_channels_callback_t callback)
{
    elrs.channels_cb = callback;
}

void elrs_main_set_wifi_callback(elrs_wifi_mode_callback_t callback)
{
    elrs.wifi_cb = callback;
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static uint32_t get_millis(void)
{
    return osKernelGetTickCount();
}
