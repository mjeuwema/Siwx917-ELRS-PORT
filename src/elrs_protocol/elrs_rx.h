/**
 * @file elrs_rx.h
 * @brief ELRS Receiver Main Loop for SiW917 + LR1121
 * 
 * This module implements the core ELRS receiver functionality:
 *   - Radio initialization with LoRa modulation
 *   - SYNC packet detection and binding verification
 *   - FHSS frequency hopping synchronization
 *   - RC channel data decoding
 *   - Telemetry transmission (downlink)
 *   - Connection state management
 * 
 * Architecture:
 *   ELRS RX Main → HAL Bridge (lr1121_hal.c) → Native Driver (lr1121_driver.c) → SiW917 HW
 * 
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf
 */

#ifndef ELRS_RX_H
#define ELRS_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "elrs_platform.h"
#include "lr1121_regs.h"
#include "fhss.h"
#include "ota.h"
#include "cmsis_os2.h"  /* For osThreadId_t, osThreadFlagsSet/Wait */

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Connection States
 * 
 * Citation: ExpressLRS rx_main.cpp line ~160
 ******************************************************************************/

typedef enum {
    ELRS_DISCONNECTED = 0,  /* No connection, scanning for SYNC */
    ELRS_TENTATIVE,         /* SYNC received, awaiting confirmation */
    ELRS_CONNECTED          /* Stable connection established */
} elrs_connection_state_t;

/*******************************************************************************
 * RX Timer States
 * 
 * Citation: ExpressLRS rx_main.cpp line ~162
 ******************************************************************************/

typedef enum {
    RX_TIMER_DISCONNECTED = 0,
    RX_TIMER_TENTATIVE,
    RX_TIMER_CONNECTED
} elrs_rx_timer_state_t;

/*******************************************************************************
 * Air Rate Configuration
 * 
 * Citation: ExpressLRS common.h - expresslrs_rf_pref_params_s
 * Citation: ExpressLRS common.cpp lines 33-51 (LR1121 rate table)
 ******************************************************************************/

typedef enum {
    /* 900 MHz rates - ordered by decreasing packet rate
     * 
     * Citation: ExpressLRS 4.0 common.h lines 83-94
     * These are LOCAL indices into rf_params_table[], NOT the OTA enum values.
     * The OTA enum values are stored in rf_params_table[].enum_rate field.
     */
    RATE_LORA_900_500HZ = 0,     /* enum_rate=9  - Fastest 900MHz */
    RATE_LORA_900_333HZ_8CH,     /* enum_rate=8  - 8-channel variant */
    RATE_LORA_900_250HZ,         /* enum_rate=7  */
    RATE_LORA_900_200HZ_8CH,     /* enum_rate=6  - 8-channel variant */
    RATE_LORA_900_200HZ,         /* enum_rate=5  */
    RATE_LORA_900_150HZ,         /* enum_rate=4  - NEW: was missing */
    RATE_LORA_900_100HZ_8CH,     /* enum_rate=3  - 8-channel variant */
    RATE_LORA_900_100HZ,         /* enum_rate=2  */
    RATE_LORA_900_50HZ,          /* enum_rate=1  */
    RATE_LORA_900_25HZ,          /* enum_rate=0  - Long range */
    RATE_LORA_900_50HZ_DVDA,     /* enum_rate=10 - DVDA mode */
    
    /* 2.4 GHz rates - ordered by decreasing packet rate
     * 
     * Citation: ExpressLRS 4.0 common.h lines 96-105
     */
    RATE_LORA_2G4_500HZ,         /* enum_rate=29 - Fastest 2.4GHz */
    RATE_LORA_2G4_333HZ_8CH,     /* enum_rate=28 - 8-channel variant */
    RATE_LORA_2G4_250HZ,         /* enum_rate=27 */
    RATE_LORA_2G4_200HZ_8CH,     /* enum_rate=26 - NEW: was missing */
    RATE_LORA_2G4_200HZ,         /* enum_rate=25 - NEW: was missing */
    RATE_LORA_2G4_150HZ,         /* enum_rate=24 */
    RATE_LORA_2G4_100HZ_8CH,     /* enum_rate=23 - 8-channel variant */
    RATE_LORA_2G4_100HZ,         /* enum_rate=22 - NEW: was missing */
    RATE_LORA_2G4_50HZ,          /* enum_rate=21 */
    RATE_LORA_2G4_25HZ,          /* enum_rate=20 - NEW: was missing, long range */
    
    /* Dual band rates
     * 
     * Citation: ExpressLRS 4.0 common.h lines 114-115
     */
    RATE_LORA_DUAL_150HZ,        /* enum_rate=101 */
    RATE_LORA_DUAL_100HZ_8CH,    /* enum_rate=100 */
    
    RATE_MAX  /* Total: 23 rates */
} elrs_rate_index_t;

/**
 * RF performance parameters for each rate
 * Citation: ExpressLRS common.h line 150
 * Citation: ExpressLRS common.h expresslrs_rf_pref_params_s
 */
typedef struct {
    uint8_t                     index;          /* Rate index */
    uint8_t                     enum_rate;      /* Rate enum value (for TX<->RX matching) */
    uint8_t                     bw;             /* LoRa bandwidth (lr11xx_radio_lora_bw_t) */
    uint8_t                     sf;             /* Spreading factor (lr11xx_radio_lora_sf_t) */
    uint8_t                     cr;             /* Coding rate (lr11xx_radio_lora_cr_t) */
    uint8_t                     preamble_len;   /* Preamble length in symbols */
    uint8_t                     tlm_ratio;      /* Telemetry ratio */
    uint8_t                     fhss_hop_interval; /* Hops per SYNC packet */
    uint32_t                    interval_us;    /* Packet interval in microseconds */
    uint32_t                    toa_us;         /* Time On Air in microseconds (for PFD slack) */
    uint8_t                     payload_len;    /* OTA packet size */
    int8_t                      rx_sensitivity; /* RX sensitivity in dBm (for rate selection) */
    
    /* ELRS 4.0 RF Performance Parameters
     * Citation: ExpressLRS common.h lines 152-158 - expresslrs_rf_pref_params_s
     */
    uint16_t                    disconnect_timeout_ms;  /* Time without packet before disconnect */
    uint16_t                    rx_lock_timeout_ms;     /* Max time tentative->connected */
} elrs_rf_params_t;

/**
 * PACKET_TO_TOCK_SLACK - Minimum time between packet RX and TOCK callback
 * Citation: ExpressLRS rx_main.cpp line 60
 *   #define PACKET_TO_TOCK_SLACK 200
 * 
 * This ensures there's time to process the packet before TOCK fires.
 */
#define PACKET_TO_TOCK_SLACK    200

/*******************************************************************************
 * Channel Data Structure
 ******************************************************************************/

#define ELRS_NUM_CHANNELS   16

typedef struct {
    uint16_t ch[ELRS_NUM_CHANNELS];     /* Channel values (CRSF format: 172-1811) */
    bool     armed;                      /* Arm status */
    uint32_t last_update_ms;             /* Timestamp of last update */
} elrs_channel_data_t;

/*******************************************************************************
 * Link Statistics (RX-specific)
 * 
 * Note: This is different from elrs_link_stats_t in elrs_platform.h which is
 * the CRSF telemetry format. This struct is for internal RX statistics.
 ******************************************************************************/

typedef struct {
    int8_t   rssi_ant1;         /* RSSI from antenna 1 in dBm */
    int8_t   rssi_ant2;         /* RSSI from antenna 2 in dBm */
    int8_t   snr;               /* Signal-to-noise ratio */
    uint8_t  lq;                /* Link quality (0-100%) */
    uint8_t  antenna;           /* Active antenna (0 or 1) */
    uint32_t packets_received;  /* Total packets received */
    uint32_t packets_lost;      /* Estimated lost packets */
} elrs_rx_link_stats_t;

/*******************************************************************************
 * RX State Structure
 ******************************************************************************/

typedef struct {
    /* Connection state */
    elrs_connection_state_t     conn_state;
    elrs_rx_timer_state_t       timer_state;
    
    /* Timing */
    uint32_t    last_packet_ms;         /* Time of last valid packet */
    uint32_t    last_sync_ms;           /* Time of last SYNC packet */
    uint32_t    connected_ms;           /* Time when connection established */
    uint32_t    cycle_interval_ms;      /* Rate cycling interval */
    
    /* Rate configuration */
    elrs_rate_index_t           current_rate;
    const elrs_rf_params_t     *rf_params;
    
    /* FHSS state */
    uint8_t     fhss_index;             /* Current FHSS sequence index */
    uint32_t    current_freq_hz;        /* Current RF frequency */
    
    /* Packet handling */
    OTA_Packet_s    rx_packet;          /* Received packet buffer */
    uint8_t         nonce;              /* Current packet nonce */
    bool            got_sync;           /* SYNC packet received flag */
    
    /* ELRS 4.0 Nonce Synchronization Lock
     * 
     * Citation: ExpressLRS 4.0 Release Notes
     *   "More robust syncing - The OTA now requires a counter synchronization 
     *    lock to function."
     * 
     * The RX maintains its own nonce counter (nonce_rx) which is incremented
     * in TOCK. On SYNC packet receipt, we compare our nonce_rx with the TX's
     * nonce. Connection is only established when they are synchronized.
     * 
     * This prevents false connections and ensures reliable CRC validation
     * since CRC includes the nonce value.
     */
    uint8_t         nonce_rx;           /* RX's local nonce counter (incremented in TOCK) */
    bool            nonce_sync_locked;  /* True when nonce_rx matches TX nonce */
    int8_t          nonce_sync_diff;    /* Last measured nonce difference for debugging */
    
    /* Channel data */
    elrs_channel_data_t channels;
    
    /* Link statistics */
    elrs_rx_link_stats_t   link_stats;
    
    /* Binding info */
    uint8_t     uid[6];                 /* Unique ID from binding phrase */
    bool        model_match;            /* Model ID matched */
    uint8_t     model_id;               /* Expected model ID */
    
    /* Radio IRQ flag and timing
     * 
     * Citation: ExpressLRS rx_main.cpp - ProcessRFPacket()
     *   Timestamp must be captured at IRQ time, not after SPI read.
     *   This is critical for PFD phase offset calculation accuracy.
     */
    volatile bool       irq_pending;
    volatile uint32_t   irq_timestamp_us;   /* Timestamp captured at DIO1 IRQ */
    
    /* ELRS 4.0: Deferred Rate Change
     * 
     * Citation: ExpressLRS rx_main.cpp lines 1049-1054 - ProcessRfPacket_SYNC()
     *   ExpressLRS_nextAirRateIndex = enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);
     * 
     * Citation: ExpressLRS rx_main.cpp lines 2082-2095 - loop()
     *   if (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRateIndex || SwitchModePending)
     *   {
     *       SetRFLinkRate(ExpressLRS_nextAirRateIndex, SwitchModePending);
     *       SwitchModePending = false;
     *   }
     * 
     * Rate changes from SYNC packets are NOT applied immediately in the packet handler.
     * Instead, they are deferred to the main loop for safe application because:
     *   1. Reconfiguring radio in ISR/packet handler can cause timing issues
     *   2. Rate change may need hwTimer interval update (must be atomic)
     *   3. Avoids race conditions with FHSS hopping
     */
    elrs_rate_index_t   next_rate;          /* Pending rate index from SYNC */
    bool                switch_mode_pending; /* Flag: rate change queued */
    
} elrs_rx_state_t;

/*******************************************************************************
 * Global State (extern)
 ******************************************************************************/

extern elrs_rx_state_t ELRS_RX;

/*******************************************************************************
 * Thread Flag Definitions for Interrupt-Driven Operation
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsWait()
 *   Thread flags provide efficient task notification from ISR context.
 *   ISR sets flag, task waits on flag - zero CPU usage while waiting.
 * 
 * Citation: ExpressLRS 4.0 rx_main.cpp
 *   Official ELRS uses DIO1 hardware interrupts for packet notification.
 ******************************************************************************/

/** Thread flag set by DIO1 ISR when radio IRQ fires (RX_DONE, TIMEOUT, etc.) */
#define ELRS_RX_FLAG_RADIO_IRQ   (1UL << 0)

/** Thread flag to request task exit (for clean shutdown) */
#define ELRS_RX_FLAG_EXIT        (1UL << 1)

/** All ELRS RX thread flags mask */
#define ELRS_RX_FLAGS_ALL        (ELRS_RX_FLAG_RADIO_IRQ | ELRS_RX_FLAG_EXIT)

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

/**
 * @brief Callback for packet received events
 * 
 * Called by elrs_rx_loop() when a valid packet is received.
 * The callback receives the timestamp of when the packet arrived,
 * which can be used for PFD phase offset calculation.
 * 
 * @param timestamp_us  Microsecond timestamp of packet arrival
 * @param packet_type   Type of packet (PACKET_TYPE_SYNC, PACKET_TYPE_RCDATA, etc.)
 */
typedef void (*elrs_rx_packet_callback_t)(uint32_t timestamp_us, uint8_t packet_type);

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/

/**
 * @brief Register a callback for packet received events
 * 
 * The callback is called from elrs_rx_loop() (task context, not ISR)
 * immediately after a valid packet is received and processed.
 * 
 * @param callback  Function to call, or NULL to disable
 */
void elrs_rx_set_packet_callback(elrs_rx_packet_callback_t callback);

/**
 * @brief Set the ELRS RX task thread handle for interrupt signaling
 * 
 * Must be called before elrs_rx_init() to enable interrupt-driven operation.
 * If not called, elrs_rx_loop() will fall back to polling mode.
 * 
 * @param thread_id  FreeRTOS/CMSIS-RTOS2 thread handle from osThreadNew()
 */
void elrs_rx_set_task_handle(osThreadId_t thread_id);

/**
 * @brief Initialize ELRS receiver
 * 
 * Initializes:
 *   - LR1121 radio via HAL
 *   - FHSS sequence from UID
 *   - OTA CRC from UID
 *   - Initial rate configuration
 * 
 * @param uid       6-byte Unique ID (from binding phrase hash)
 * @param domain    Regulatory domain (DOMAIN_FCC915, DOMAIN_ISM2G4, etc.)
 * @return true on success
 */
bool elrs_rx_init(const uint8_t *uid, fhss_domain_e domain);

/**
 * @brief Set model match ID
 * 
 * When set, RX will only respond to packets with matching model ID.
 * Set to 0 to disable model match.
 * 
 * @param model_id  Model ID to match (0-255)
 */
void elrs_rx_set_model_match(uint8_t model_id);

/**
 * @brief Start receiver
 * 
 * Puts radio in RX mode and begins listening for SYNC packets.
 * Call after elrs_rx_init().
 */
void elrs_rx_start(void);

/**
 * @brief Stop receiver
 * 
 * Puts radio in standby mode and stops reception.
 */
void elrs_rx_stop(void);

/**
 * @brief Main RX processing loop
 * 
 * Must be called regularly from main loop.
 * Handles:
 *   - Packet reception and decoding
 *   - FHSS frequency hopping
 *   - Connection state management
 *   - Telemetry transmission
 * 
 * @return true if a valid packet was received this call
 */
bool elrs_rx_loop(void);

/**
 * @brief Radio ISR callback
 * 
 * Called from GPIO interrupt when DIO1 fires.
 * Sets irq_pending flag for elrs_rx_loop() to process.
 */
void elrs_rx_isr(void);

/**
 * @brief Get current channel data
 * 
 * @return Pointer to channel data structure
 */
const elrs_channel_data_t* elrs_rx_get_channels(void);

/**
 * @brief Get current link statistics
 * 
 * @return Pointer to link stats structure
 */
const elrs_rx_link_stats_t* elrs_rx_get_link_stats(void);

/**
 * @brief Get connection state
 * 
 * @return Current connection state
 */
elrs_connection_state_t elrs_rx_get_state(void);

/**
 * @brief Check if receiver is connected
 * 
 * @return true if in CONNECTED state with recent packets
 */
bool elrs_rx_is_connected(void);

/**
 * @brief Force rate change
 * 
 * @param rate_index New rate index
 * @return true if rate change was applied
 */
bool elrs_rx_set_rate(elrs_rate_index_t rate_index);

/**
 * @brief Get current rate index
 * 
 * @return Current rate index
 */
elrs_rate_index_t elrs_rx_get_rate(void);

/*******************************************************************************
 * Low-Level Radio Configuration Functions
 ******************************************************************************/

/**
 * @brief Configure radio for specified rate
 * 
 * Sets LoRa modulation parameters, packet params, and frequency.
 * 
 * @param params    RF parameters for the rate
 * @param freq_hz   Initial frequency in Hz
 * @return true on success
 */
bool elrs_rx_config_radio(const elrs_rf_params_t *params, uint32_t freq_hz);

/**
 * @brief Set radio frequency
 * 
 * @param freq_hz   Frequency in Hz
 */
void elrs_rx_set_frequency(uint32_t freq_hz);

/**
 * @brief Enter RX mode
 * 
 * @param timeout_ms    RX timeout (0 = continuous)
 */
void elrs_rx_enter_rx_mode(uint32_t timeout_ms);

/**
 * @brief Read received packet from radio
 * 
 * @param buffer    Buffer to store packet
 * @param max_len   Maximum buffer length
 * @return Number of bytes received, or 0 if no packet
 */
uint8_t elrs_rx_read_packet(uint8_t *buffer, uint8_t max_len);

/**
 * @brief Get RSSI and SNR from last packet
 * 
 * @param rssi      Output RSSI in dBm
 * @param snr       Output SNR in dB
 */
void elrs_rx_get_packet_status(int8_t *rssi, int8_t *snr);

/*******************************************************************************
 * Binding Mode Functions
 ******************************************************************************/

/**
 * @brief Set binding UID for OTA binding mode
 * 
 * When set, the RX will accept SYNC packets with this UID and capture
 * the TX's UID bytes for binding completion.
 * 
 * @param binding_uid   6-byte binding UID, or NULL to disable binding mode
 */
void elrs_rx_set_binding_uid(const uint8_t *binding_uid);

/**
 * @brief Check if binding mode is active
 * 
 * @return true if in binding mode
 */
bool elrs_rx_is_binding_mode(void);

/**
 * @brief Get RF parameters for a specific rate index
 * 
 * @param rate_index   Rate enum (elrs_rate_e)
 * @return Pointer to RF params, or NULL if invalid index
 */
const elrs_rf_params_t* elrs_rx_get_rf_params(uint8_t rate_index);

/*******************************************************************************
 * Diagnostic Functions
 ******************************************************************************/

/**
 * @brief Print comprehensive RF subsystem diagnostic information
 * 
 * Outputs detailed information about the RF subsystem state:
 *   - Chip mode (should be RX)
 *   - IRQ status and error flags
 *   - Hardware errors (GetErrors)
 *   - Current frequency and modulation settings
 *   - UID and CRC configuration
 *   - RF switch configuration reminder
 * 
 * Call this after elrs_rx_start() to verify the RF chain is correctly
 * configured. Useful for debugging when packets are not being received.
 */
void elrs_rx_print_rf_diagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* ELRS_RX_H */
