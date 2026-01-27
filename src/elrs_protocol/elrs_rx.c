/**
 * @file elrs_rx.c
 * @brief ELRS Receiver Implementation for SiW917 + LR1121
 * 
 * This file implements the core ELRS receiver functionality.
 * 
 * Citation: ExpressLRS 4.0 src/src/rx_main.cpp
 * Citation: ExpressLRS 4.0 src/src/common.cpp (rate tables)
 * Citation: 61252685.LR1121_V2_1_data_sheet.pdf
 */

#include "elrs_rx.h"
#include "lr1121_hal.h"
#include "lr1121_elrs_init.h"  /* For ELRS_CMD_SET_DIO_AS_RF_SWITCH and other LR1121 commands */
#include "fhss.h"
#include "ota.h"
#include "crc.h"
#include "random.h"
#include "hw_timer.h"  /* For hw_timer_get_micros() - used by ISR for timestamp capture */
#include "rsi_debug.h"
#include "cmsis_os2.h"

#include <string.h>
#include <inttypes.h>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/* Debug output enable */
#define ELRS_RX_DEBUG   1

#if ELRS_RX_DEBUG
    #define RX_DBG(fmt, ...)    DEBUGOUT("[ELRS_RX] " fmt, ##__VA_ARGS__)
#else
    #define RX_DBG(fmt, ...)    ((void)0)
#endif

/* Connection timeouts */
#define CONNECTION_LOST_TIMEOUT_MS      1500    /* Time without packets before disconnect */
#define TENTATIVE_TIMEOUT_MS            500     /* Time to stay in TENTATIVE state */
#define CONSIDER_CONN_GOOD_MS           1000    /* Time before connection considered good */

/* Rate cycling */
#define RATE_CYCLE_INTERVAL_MS          3000    /* Time between rate changes when disconnected */

/* Link quality calculation window */
#define LQ_CALC_WINDOW                  100     /* Number of packets for LQ calculation */

/*******************************************************************************
 * RF Parameters Table
 * 
 * Citation: ExpressLRS common.cpp lines 33-51 (LR1121 rate table)
 * 
 * Format: { index, bw, sf, cr, preamble_len, tlm_ratio, fhss_hop_interval, interval_us, payload_len }
 * 
 * Bandwidths (LR1121):
 *   LR11XX_RADIO_LORA_BW_500 = 0x06
 *   LR11XX_RADIO_LORA_BW_800 = 0x0F (2.4 GHz only)
 * 
 * Spreading Factors:
 *   LR11XX_RADIO_LORA_SF5-SF12 = 0x05-0x0C
 * 
 * Coding Rates:
 *   LR11XX_RADIO_LORA_CR_4_5 through CR_LI_4_8
 ******************************************************************************/

/**
 * RF Parameters Table with TOA values for PFD slack calculation
 * 
 * Citation: ExpressLRS common.cpp lines 33-51 (LR1121 rate table)
 * Citation: ExpressLRS common.h expresslrs_rf_pref_params_s (TOA values)
 * 
 * Format: { index, enum_rate, bw, sf, cr, preamble, tlm_ratio, fhss_interval, interval_us, toa_us, payload, sensitivity }
 * 
 * TOA (Time On Air) values are calculated based on LoRa air time formula:
 *   TOA = (preamble + 4.25 + 8 + ceil((8*payload + 28 - 4*SF + 16) / (4*SF)) * (CR+4)) * Tsym
 *   where Tsym = 2^SF / BW
 * 
 * These TOA values are approximations based on ExpressLRS measurements.
 */
static const elrs_rf_params_t rf_params_table[RATE_MAX] = {
    /*******************************************************************************
     * 900 MHz Rate Table
     * 
     * CRITICAL: The enum_rate values MUST match the expresslrs_RFrates_e enum
     * from ELRS 4.0 common.h. These values are transmitted in the SYNC packet
     * and used by enumRatetoIndex() to find the correct rate configuration.
     * 
     * Citation: ExpressLRS 4.0 src/include/common.h lines 83-94
     *   RATE_LORA_900_25HZ = 0,
     *   RATE_LORA_900_50HZ = 1,
     *   RATE_LORA_900_100HZ = 2,
     *   RATE_LORA_900_100HZ_8CH = 3,
     *   RATE_LORA_900_150HZ = 4,
     *   RATE_LORA_900_200HZ = 5,
     *   RATE_LORA_900_200HZ_8CH = 6,
     *   RATE_LORA_900_250HZ = 7,
     *   RATE_LORA_900_333HZ_8CH = 8,
     *   RATE_LORA_900_500HZ = 9,
     *   RATE_LORA_900_50HZ_DVDA = 10,
     * 
     * Format: { index, enum_rate, bw, sf, cr, preamble, tlm_ratio, fhss_interval, interval_us, toa_us, payload, sensitivity }
     * 
     * TOA (Time On Air) values are calculated based on LoRa air time formula:
     *   TOA = (preamble + 4.25 + 8 + ceil((8*payload + 28 - 4*SF + 16) / (4*SF)) * (CR+4)) * Tsym
     *   where Tsym = 2^SF / BW
     ******************************************************************************/
    
    /* Index 0: RATE_LORA_900_500HZ: SF5, BW500, interval=2ms - enum_rate=9 
     * Citation: ELRS 4.0 common.cpp - Fastest 900MHz rate, short range racing
     * TOA ~ 1.3ms at BW500/SF5, interval 2ms -> 500 packets/sec
     * RF Perf params: DisconnectTimeoutMs=2500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_500HZ, 9, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_4_8, 
      8, TLM_RATIO_1_128, 4, 2000, 1300, OTA4_PACKET_SIZE, -105, 2500, 2500 },
    
    /* Index 1: RATE_LORA_900_333HZ_8CH: SF5, BW500, interval=3ms, 8-byte - enum_rate=8
     * Citation: ELRS 4.0 common.cpp - Full resolution 8-channel mode
     * TOA ~ 1.8ms at BW500/SF5 with 13-byte payload
     * RF Perf params: DisconnectTimeoutMs=2500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_333HZ_8CH, 8, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_4_8,
      8, TLM_RATIO_1_128, 4, 3003, 1800, OTA8_PACKET_SIZE, -105, 2500, 2500 },
    
    /* Index 2: RATE_LORA_900_250HZ: SF5, BW500, interval=4ms - enum_rate=7 
     * Citation: ELRS 4.0 common.cpp - Standard racing rate
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_250HZ, 7, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_4_8, 
      8, TLM_RATIO_1_64, 4, 4000, 1300, OTA4_PACKET_SIZE, -105, 3500, 2500 },
    
    /* Index 3: RATE_LORA_900_200HZ_8CH: SF5, BW500, interval=5ms, 8-byte - enum_rate=6 
     * Citation: ELRS 4.0 common.cpp - Full resolution 8-channel mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_200HZ_8CH, 6, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_4_7,
      8, TLM_RATIO_1_64, 4, 5000, 1800, OTA8_PACKET_SIZE, -105, 3500, 2500 },
    
    /* Index 4: RATE_LORA_900_200HZ: SF6, BW500, interval=5ms - enum_rate=5 
     * Citation: ELRS 4.0 common.cpp - Balanced performance/range
     * RF Perf params: DisconnectTimeoutMs=3000, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_200HZ, 5, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_7,
      8, TLM_RATIO_1_64, 4, 5000, 2200, OTA4_PACKET_SIZE, -108, 3000, 2500 },
    
    /* Index 5: RATE_LORA_900_150HZ: SF6, BW500, interval=6.666ms - enum_rate=4 
     * Citation: ELRS 4.0 common.cpp - NEW: Good balance between latency and range
     * TOA ~ 2.2ms at BW500/SF6, interval 6666us -> 150 packets/sec
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_150HZ, 4, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_7,
      8, TLM_RATIO_1_32, 4, 6666, 2200, OTA4_PACKET_SIZE, -108, 3500, 2500 },
    
    /* Index 6: RATE_LORA_900_100HZ_8CH: SF6, BW500, interval=10ms, 8-byte - enum_rate=3 
     * Citation: ELRS 4.0 common.cpp - Full resolution 8-channel mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_100HZ_8CH, 3, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_8,
      8, TLM_RATIO_1_32, 4, 10000, 2800, OTA8_PACKET_SIZE, -108, 3500, 2500 },
    
    /* Index 7: RATE_LORA_900_100HZ: SF7, BW500, interval=10ms - enum_rate=2 
     * Citation: ELRS 4.0 common.cpp - Standard freestyle/LR mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_100HZ, 2, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_CR_4_7,
      8, TLM_RATIO_1_32, 4, 10000, 4200, OTA4_PACKET_SIZE, -112, 3500, 2500 },
    
    /* Index 8: RATE_LORA_900_50HZ: SF8, BW500, interval=20ms - enum_rate=1 
     * Citation: ELRS 4.0 common.cpp - Long range mode
     * RF Perf params: DisconnectTimeoutMs=4000, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_50HZ, 1, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF8, LR11XX_RADIO_LORA_CR_4_7,
      10, TLM_RATIO_1_16, 4, 20000, 8200, OTA4_PACKET_SIZE, -115, 4000, 2500 },
    
    /* Index 9: RATE_LORA_900_25HZ: SF9, BW500, interval=40ms - enum_rate=0 
     * Citation: ELRS 4.0 common.cpp - Maximum range, wings/sailplanes
     * RF Perf params: DisconnectTimeoutMs=6000, RxLockTimeoutMs=4000 */
    { RATE_LORA_900_25HZ, 0, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF9, LR11XX_RADIO_LORA_CR_4_7,
      10, TLM_RATIO_1_8, 2, 40000, 16400, OTA4_PACKET_SIZE, -118, 6000, 4000 },
    
    /* Index 10: RATE_LORA_900_50HZ_DVDA: SF6, BW500, interval=5ms - enum_rate=10 
     * Citation: ELRS 4.0 common.cpp - DVDA (Diversity) mode, interleaved with 2.4GHz
     * RF Perf params: DisconnectTimeoutMs=3000, RxLockTimeoutMs=2500 */
    { RATE_LORA_900_50HZ_DVDA, 10, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_7,
      8, TLM_RATIO_1_64, 2, 5000, 2200, OTA4_PACKET_SIZE, -108, 3000, 2500 },
    
    /*******************************************************************************
     * 2.4 GHz Rate Table
     * 
     * Citation: ExpressLRS 4.0 src/include/common.h lines 96-105
     *   RATE_LORA_2G4_25HZ = 20,
     *   RATE_LORA_2G4_50HZ = 21,
     *   RATE_LORA_2G4_100HZ = 22,
     *   RATE_LORA_2G4_100HZ_8CH = 23,
     *   RATE_LORA_2G4_150HZ = 24,
     *   RATE_LORA_2G4_200HZ = 25,
     *   RATE_LORA_2G4_200HZ_8CH = 26,
     *   RATE_LORA_2G4_250HZ = 27,
     *   RATE_LORA_2G4_333HZ_8CH = 28,
     *   RATE_LORA_2G4_500HZ = 29,
     * 
     * Note: 2.4GHz uses LR11XX Long Interleaving coding rates (CR_LI_4_x)
     ******************************************************************************/
    
    /* Index 11: RATE_LORA_2G4_500HZ: SF5, BW800, interval=2ms - enum_rate=29 
     * Citation: ELRS 4.0 common.cpp - Fastest 2.4GHz, micro racing
     * RF Perf params: DisconnectTimeoutMs=2500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_500HZ, 29, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_LI_4_6,
      12, TLM_RATIO_1_128, 4, 2000, 831, OTA4_PACKET_SIZE, -105, 2500, 2500 },
    
    /* Index 12: RATE_LORA_2G4_333HZ_8CH: SF5, BW800, interval=3ms, 8-byte - enum_rate=28 
     * Citation: ELRS 4.0 common.cpp - Full resolution 8-channel mode
     * RF Perf params: DisconnectTimeoutMs=2500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_333HZ_8CH, 28, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_128, 4, 3003, 1207, OTA8_PACKET_SIZE, -105, 2500, 2500 },
    
    /* Index 13: RATE_LORA_2G4_250HZ: SF6, BW800, interval=4ms - enum_rate=27 
     * Citation: ELRS 4.0 common.cpp - Standard racing
     * RF Perf params: DisconnectTimeoutMs=3000, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_250HZ, 27, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_LI_4_8,
      14, TLM_RATIO_1_64, 4, 4000, 1876, OTA4_PACKET_SIZE, -108, 3000, 2500 },
    
    /* Index 14: RATE_LORA_2G4_200HZ_8CH: SF6, BW800, interval=5ms, 8-byte - enum_rate=26 
     * Citation: ELRS 4.0 common.cpp - NEW: Full resolution 8-channel mode
     * TOA ~ 2.2ms at BW800/SF6 with 13-byte payload
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_200HZ_8CH, 26, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_64, 4, 5000, 2200, OTA8_PACKET_SIZE, -108, 3500, 2500 },
    
    /* Index 15: RATE_LORA_2G4_200HZ: SF6, BW800, interval=5ms - enum_rate=25 
     * Citation: ELRS 4.0 common.cpp - NEW: Balanced performance/range
     * TOA ~ 1.9ms at BW800/SF6
     * Note: LR1121 doesn't support CR_LI_4_7, using CR_LI_4_6 as closest match
     * RF Perf params: DisconnectTimeoutMs=3000, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_200HZ, 25, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_LI_4_6,
      12, TLM_RATIO_1_64, 4, 5000, 1900, OTA4_PACKET_SIZE, -108, 3000, 2500 },
    
    /* Index 16: RATE_LORA_2G4_150HZ: SF7, BW800, interval=6.666ms - enum_rate=24 
     * Citation: ELRS 4.0 common.cpp - Good freestyle rate
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_150HZ, 24, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_32, 4, 6666, 3080, OTA4_PACKET_SIZE, -112, 3500, 2500 },
    
    /* Index 17: RATE_LORA_2G4_100HZ_8CH: SF7, BW800, interval=10ms, 8-byte - enum_rate=23 
     * Citation: ELRS 4.0 common.cpp - Full resolution 8-channel mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_100HZ_8CH, 23, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_32, 4, 10000, 3680, OTA8_PACKET_SIZE, -112, 3500, 2500 },
    
    /* Index 18: RATE_LORA_2G4_100HZ: SF7, BW800, interval=10ms - enum_rate=22 
     * Citation: ELRS 4.0 common.cpp - NEW: Standard freestyle/mid-range
     * TOA ~ 3.1ms at BW800/SF7 
     * Note: LR1121 doesn't support CR_LI_4_7, using CR_LI_4_6 as closest match
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_100HZ, 22, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_CR_LI_4_6,
      12, TLM_RATIO_1_32, 4, 10000, 3100, OTA4_PACKET_SIZE, -112, 3500, 2500 },
    
    /* Index 19: RATE_LORA_2G4_50HZ: SF8, BW800, interval=20ms - enum_rate=21 
     * Citation: ELRS 4.0 common.cpp - Long range 2.4GHz
     * RF Perf params: DisconnectTimeoutMs=4000, RxLockTimeoutMs=2500 */
    { RATE_LORA_2G4_50HZ, 21, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF8, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_16, 2, 20000, 6160, OTA4_PACKET_SIZE, -115, 4000, 2500 },
    
    /* Index 20: RATE_LORA_2G4_25HZ: SF9, BW800, interval=40ms - enum_rate=20 
     * Citation: ELRS 4.0 common.cpp - NEW: Maximum 2.4GHz range, wings/sailplanes
     * TOA ~ 12.3ms at BW800/SF9
     * RF Perf params: DisconnectTimeoutMs=6000, RxLockTimeoutMs=4000 */
    { RATE_LORA_2G4_25HZ, 20, LR11XX_RADIO_LORA_BW_800, LR11XX_RADIO_LORA_SF9, LR11XX_RADIO_LORA_CR_LI_4_8,
      12, TLM_RATIO_1_8, 2, 40000, 12300, OTA4_PACKET_SIZE, -118, 6000, 4000 },
    
    /*******************************************************************************
     * Dual Band Rate Table
     * 
     * Citation: ExpressLRS 4.0 src/include/common.h lines 114-115
     *   RATE_LORA_DUAL_100HZ_8CH = 100,
     *   RATE_LORA_DUAL_150HZ = 101,
     * 
     * These rates are used for simultaneous 900MHz + 2.4GHz diversity
     ******************************************************************************/
    
    /* Index 21: RATE_LORA_DUAL_150HZ: SF6, BW500 - enum_rate=101 
     * Citation: ELRS 4.0 common.cpp - Dual-band diversity mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_DUAL_150HZ, 101, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_8,
      12, TLM_RATIO_1_32, 4, 6666, 2800, OTA4_PACKET_SIZE, -108, 3500, 2500 },
    
    /* Index 22: RATE_LORA_DUAL_100HZ_8CH: SF6, BW500, 8-byte - enum_rate=100 
     * Citation: ELRS 4.0 common.cpp - Dual-band diversity 8-channel mode
     * RF Perf params: DisconnectTimeoutMs=3500, RxLockTimeoutMs=2500 */
    { RATE_LORA_DUAL_100HZ_8CH, 100, LR11XX_RADIO_LORA_BW_500, LR11XX_RADIO_LORA_SF6, LR11XX_RADIO_LORA_CR_4_8,
      18, TLM_RATIO_1_32, 4, 10000, 3200, OTA8_PACKET_SIZE, -108, 3500, 2500 },
};

/*******************************************************************************
 * Global State
 ******************************************************************************/

elrs_rx_state_t ELRS_RX;

/* LQ calculation buffer */
static uint8_t lq_buffer[LQ_CALC_WINDOW];
static uint8_t lq_index = 0;
static uint8_t lq_count = 0;

/*******************************************************************************
 * Interrupt-Driven Operation State
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsSet() / osThreadFlagsWait()
 *   Thread flags allow ISR-to-task notification with zero polling overhead.
 *   ISR sets flag via osThreadFlagsSet(), task blocks on osThreadFlagsWait().
 * 
 * When task_handle is set:
 *   - DIO1 ISR signals task via osThreadFlagsSet(ELRS_RX_FLAG_RADIO_IRQ)
 *   - elrs_rx_loop() waits on flags instead of polling
 *   - CPU usage drops from ~30% to <1% between packets
 * 
 * When task_handle is NULL (fallback):
 *   - elrs_rx_loop() polls IRQ status every call (original behavior)
 ******************************************************************************/
static osThreadId_t elrs_task_handle = NULL;

/* Packet callback for elrs_main integration */
static elrs_rx_packet_callback_t packet_callback = NULL;

/*******************************************************************************
 * Binding Mode State
 * 
 * Citation: ExpressLRS 4.0 rx_main.cpp - EnterBindingMode()
 * During binding mode, the RX listens with a well-known binding UID
 * and captures the TX's real UID from SYNC packets.
 ******************************************************************************/
static bool binding_mode_active = false;
static uint8_t binding_uid[6] = {0};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void process_sync_packet(OTA_Sync_s *sync);
static void process_rc_packet(OTA_Packet_s *pkt);
static void update_connection_state(bool packet_received);
static void update_link_quality(bool packet_received);
static uint32_t get_millis(void);

/*******************************************************************************
 * Rate Enum to Index Translation
 * 
 * Citation: ExpressLRS common.cpp lines 157-171 - enumRatetoIndex()
 *   The OTA SYNC packet contains rfRateEnum which is an enum value,
 *   NOT an array index. This function converts the enum to an index.
 * 
 * For the LR1121 rate table, the enum values match the indices in our
 * rf_params_table. However, to be safe and compatible with future changes,
 * we search for a matching enum_rate value.
 ******************************************************************************/

/**
 * @brief Convert OTA rate enum to rate table index
 * 
 * @param rf_rate_enum  Rate enum received in SYNC packet
 * @return elrs_rate_index_t  Index into rf_params_table, or default rate if not found
 */
static elrs_rate_index_t enumRatetoIndex(uint8_t rf_rate_enum)
{
    for (int i = 0; i < RATE_MAX; i++) {
        if (rf_params_table[i].enum_rate == rf_rate_enum) {
            return (elrs_rate_index_t)i;
        }
    }
    /* Not found - return default rate */
    RX_DBG("Unknown rate enum %d, using default\n", rf_rate_enum);
    return RATE_LORA_900_100HZ;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

bool elrs_rx_init(const uint8_t *uid, fhss_domain_e domain __attribute__((unused)))
{
    lr1121_hal_status_t hal_status;
    
    RX_DBG("Initializing ELRS RX...\n");
    
    /* Clear state */
    memset(&ELRS_RX, 0, sizeof(ELRS_RX));
    
    /* Store UID - copy to both local state and global UID array
     * The global UID[] is used by OtaUpdateCrcInitFromUid() for CRC init.
     * Citation: ELRS 4.0 uses global UID[] throughout OTA.cpp
     */
    if (uid != NULL) {
        memcpy(ELRS_RX.uid, uid, 6);
        memcpy(UID, uid, 6);  /* Update global UID for OTA CRC */
        RX_DBG("UID SET: input=%02X:%02X:%02X:%02X:%02X:%02X\n",
               uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
        RX_DBG("UID SET: ELRS_RX.uid=%02X:%02X:%02X:%02X:%02X:%02X\n",
               ELRS_RX.uid[0], ELRS_RX.uid[1], ELRS_RX.uid[2],
               ELRS_RX.uid[3], ELRS_RX.uid[4], ELRS_RX.uid[5]);
        RX_DBG("UID SET: global UID=%02X:%02X:%02X:%02X:%02X:%02X\n",
               UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
    } else {
        RX_DBG("WARNING: uid parameter is NULL!\n");
    }
    
    /* Initialize HAL */
    hal_status = lr1121_hal_init();
    if (hal_status != LR1121_HAL_OK) {
        RX_DBG("HAL init failed: %d\n", hal_status);
        return false;
    }
    
    /***************************************************************************
     * CRITICAL: Flash ELRS custom firmware if not already installed
     * 
     * The LR1121 requires ExpressLRS custom firmware (type 0xF3) for ELRS
     * operation. This firmware adds commands like:
     *   - 0x0700 GetPacket - Combined packet retrieval
     *   - 0x0701 SetFreqSetRx - Combined frequency set and RX entry
     *   - etc.
     * 
     * Without this firmware, packet data will be corrupted/garbled.
     * 
     * Citation: ExpressLRS LR1121.cpp - CheckVersion() auto-upgrades firmware
     **************************************************************************/
    {
        extern int lr1121_flash_elrs_firmware(void);
        int fw_result = lr1121_flash_elrs_firmware();
        if (fw_result != 0) {
            RX_DBG("WARNING: ELRS firmware flash failed: %d\n", fw_result);
            /* Continue anyway - might already have correct firmware */
        }
    }
    
    /* Reset radio */
    hal_status = lr1121_hal_reset(false);
    if (hal_status != LR1121_HAL_OK) {
        RX_DBG("Radio reset failed: %d\n", hal_status);
        return false;
    }
    
    /***************************************************************************
     * CRITICAL FIX #1: ClearErrors - Clear any latched error flags after reset
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp Begin() line 123:
     *   hal.WriteCommand(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_All);
     * 
     * Citation: LR1121 Datasheet Section 11.2.6 "ClearErrors" (opcode 0x010E)
     *   "Clears the error registers. This command should be called after a 
     *    reset to ensure no stale error flags persist."
     * 
     * WHY THIS IS CRITICAL:
     *   After reset, the LR1121 may have latched error flags from previous
     *   sessions or from the reset process itself. These can interfere with
     *   proper operation if not cleared before configuration.
     **************************************************************************/
    {
        hal_status = lr1121_hal_write_command(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: ClearErrors failed: %d\n", hal_status);
            /* Non-fatal - continue */
        } else {
            RX_DBG("ClearErrors OK - Post-reset error flags cleared\n");
        }
        
        /* Wait for command to complete */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-ClearErrors BUSY timeout\n");
        }
    }
    
    /***************************************************************************
     * CRITICAL: LR1121 Post-Reset Initialization Sequence
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp Begin() / Config() sequence
     * Citation: Semtech lr11xx_system.c initialization sequence
     * Citation: Waveshare Core1121_XF_Demo lr1121_config.cpp line 108
     * 
     * After reset, the LR1121 requires these commands BEFORE it can receive:
     *   0. SetStandby(RC) - MANDATORY! SetTcxoMode only works in STDBY_RC
     *   1. SetTcxoMode - CRITICAL! Configure TCXO timing for crystal startup
     *   2. SetStandby(XOSC) - Switch system clock from RC to TCXO
     *   3. SetRegMode(DCDC) - Enable DC-DC regulator for proper power
     *   4. CalibrateImage - Calibrate RF frontend for 900MHz or 2.4GHz
     * 
     * Without SetTcxoMode, the radio reports HF_XOSC_START_ERR (0x0020) and
     * will NOT receive any packets!
     **************************************************************************/
    
    /* Step 0a: SetStandby(STDBY_RC) - MANDATORY before SetTcxoMode!
     * 
     * Citation: LR1121 Datasheet Section 6.3.2 "SetTcxoMode":
     *   "Command only operates in Standby RC mode, otherwise it returns 
     *    CMD_FAIL on the next GetStatus() command."
     * 
     * CRITICAL: Even after reset, we MUST explicitly enter STDBY_RC mode
     * before calling SetTcxoMode. This ensures the command succeeds.
     */
    {
        uint8_t standby_rc = 0x00;  /* STDBY_RC mode */
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC,
                                                         &standby_rc, 1, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetStandby(RC) failed: %d\n", hal_status);
        } else {
            RX_DBG("SetStandby(RC) OK - Preparing for SetTcxoMode\n");
        }
        
        /* Wait for chip to enter STDBY_RC */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-SetStandby(RC) BUSY timeout\n");
        }
    }
    
    /* Step 0b: SetTcxoMode - MUST be called in STDBY_RC mode!
     * 
     * Citation: Waveshare Core1121_XF_Demo lr1121_config.cpp line 108:
     *   lr11xx_system_set_tcxo_mode(context, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300);
     * 
     * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
     *   Opcode: 0x0117
     *   Parameters: [Voltage Trim (1 byte)][Delay (3 bytes, in 30.52µs steps)]
     * 
     *   Voltage Trim Values:
     *     0x00 = 1.6V, 0x01 = 1.7V, 0x02 = 1.8V, 0x03 = 2.2V,
     *     0x04 = 2.4V, 0x05 = 2.7V, 0x06 = 3.0V, 0x07 = 3.3V
     * 
     *   Delay: 300 ticks × 30.52µs ≈ 9.16ms (Waveshare default)
     * 
     * WHY THIS IS NEEDED:
     *   Even though the Core1121-HF module has an externally-powered TCXO,
     *   the LR1121 chip still needs SetTcxoMode to configure the XOSC startup
     *   timing and detection circuitry. Without this command, attempting to
     *   enter RX mode causes HF_XOSC_START_ERR because the chip doesn't know
     *   how long to wait for the oscillator to stabilize.
     */
    {
        /*
         * TCXO Configuration for Waveshare Core1121-HF - TCXO powered by LR1121 VTCXO
         * 
         * Citation: Waveshare Core1121_XF_Demo - uses voltage=0 for external TCXO
         * Citation: Semtech LR11xx SDK - LR11XX_RADIO_TCXO_CTRL_NONE for external supply
         * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
         * 
         * CRITICAL (2026-01-21): The Waveshare Core1121-XF has an EXTERNALLY-POWERED
         * 32MHz TCXO that is ALWAYS ON from the 3.3V rail. The LR1121's internal
         * VTCXO regulator is NOT used to power the TCXO.
         * 
         * Voltage Trim Values:
         *   0x00 = NONE (external TCXO - NO internal regulator)  <-- USE THIS!
         *   0x01 = 1.7V, 0x02 = 1.8V, 0x03 = 2.2V,
         *   0x04 = 2.4V, 0x05 = 2.7V, 0x06 = 3.0V, 0x07 = 3.3V
         * 
         * Hardware Configuration:
         *   - XTA pin: Connected to TCXO output (32MHz clock input)
         *   - XTB pin: Not connected (NC) - single-ended TCXO input
         *   - TCXO powered directly from 3.3V rail (always on)
         * 
         * Why SetTcxoMode is STILL required:
         *   - Switches LR1121 from default XO (crystal) mode to TCXO input mode
         *   - Without this, chip expects crystal oscillator, not single-ended clock
         *   - Skipping SetTcxoMode causes oscillator failures ("stuck in sleep")
         *
         * TCXO delay SIGNIFICANTLY INCREASED for hardware SPI timing!
         * 
         * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
         *   Delay = ticks × 30.52µs (24-bit value)
         *   3277 ticks × 30.52µs = ~100ms
         * 
         * CRITICAL FIX (2026-01-22): Hardware SPI Wake Issue
         * ===================================================
         * With hardware SPI vs soft SPI, the timing is different:
         *   - HW SPI is faster, giving less settling time between operations
         *   - The external TCXO needs more time to stabilize after wake
         *   - PLL needs time to lock to the TCXO reference
         * 
         * Changed from 656 ticks (~20ms) to 3277 ticks (~100ms) for reliable wake.
         * This matches conservative timing used by RadioMaster targets.
         */
        /* TCXO Configuration for Waveshare Core1121-XF
         * 
         * Citation: Waveshare Core1121_XF_Demo lr1121_config.cpp line 108
         * Citation: ELRS wakeup testing (2026-01-24) showed 3.0V is required
         * 
         * The Waveshare module's TCXO is powered by the LR1121's internal VTCXO
         * regulator, NOT directly from VCC. We MUST set voltage=0x06 (3.0V) to
         * enable the internal regulator and power the TCXO.
         * 
         * Parameters:
         *   voltage = 0x06 (3.0V - powers TCXO via VTCXO regulator)
         *   delay = 300 ticks (~9ms) - matches Waveshare demo
         */
        uint8_t tcxo_params[4];
        tcxo_params[0] = 0x06;  /* 3.0V - TCXO powered by LR1121's VTCXO regulator */
        tcxo_params[1] = 0x00;  /* Delay MSB */
        tcxo_params[2] = 0x01;  /* Delay MID: 0x00012C = 300 ticks (~9ms) */
        tcxo_params[3] = 0x2C;  /* Delay LSB - matches Waveshare demo */
        
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_TCXO_MODE_OC,
                                                         tcxo_params, 4, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetTcxoMode failed: %d\n", hal_status);
            /* Continue anyway - may work on some modules */
        } else {
            RX_DBG("SetTcxoMode(0x06=3.0V, ~9ms) OK - TCXO powered by VTCXO\n");
        }
        
        /* Wait for TCXO to stabilize
         * Citation: Waveshare demo waits for BUSY after SetTcxoMode
         */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-SetTcxoMode BUSY timeout\n");
        }
    }
    
    /* Step 0c: SetStandby(XOSC) - CRITICAL: Switch system clock to TCXO
     * 
     * Citation: LR1121 Datasheet Section 11.2.2 "SetStandby" (opcode 0x011C)
     * Citation: User analysis - After SetTcxoMode, chip is STILL on RC oscillator!
     * 
     * CRITICAL: SetTcxoMode powers the TCXO but does NOT switch the system clock!
     * We MUST call SetStandby(0x01) to switch from RC to XOSC (TCXO).
     * 
     * Without this step:
     *   - The TCXO is powered but not being used as system clock
     *   - PLL lock errors and BUSY timeouts occur during RX/TX operations  
     *   - Chip mode remains STDBY_RC (mode 1) instead of STDBY_XOSC (mode 2)
     * 
     * Standby Mode Parameter:
     *   0x00 = STDBY_RC (internal RC oscillator)
     *   0x01 = STDBY_XOSC (external crystal/TCXO)
     */
    {
        uint8_t standby_xosc = 0x01;  /* STDBY_XOSC mode */
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC,
                                                         &standby_xosc, 1, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetStandby(XOSC) failed: %d\n", hal_status);
            /* This is critical - TCXO won't be used without this */
        } else {
            RX_DBG("SetStandby(XOSC) OK - Clock switched to TCXO\n");
        }
        
        /* Wait for XOSC/TCXO startup - BUSY will be HIGH while switching */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-SetStandby(XOSC) BUSY timeout\n");
        }
    }
    
    /* Step 1: SetRegMode(DCDC) - Enable DC-DC regulator
     * 
     * Citation: ExpressLRS LR1121.cpp line 100
     *   smtc_shield_lr11xx_common_get_reg_mode() returns DCDC
     *   lr11xx_system_set_reg_mode(context, regulator);
     * 
     * Opcode: 0x0110, Parameter: 0x01 (DCDC mode)
     * 
     * Citation: LR1121 Datasheet Section 5.1.2 SetRegMode
     *   0x00 = LDO mode (less efficient)
     *   0x01 = DC-DC mode (more efficient, required for high performance)
     */
    {
        uint8_t reg_mode = 0x01;  /* DC-DC mode */
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_REGMODE_OC,
                                                         &reg_mode, 1, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetRegMode(DCDC) failed: %d\n", hal_status);
            /* Non-fatal - continue with LDO mode */
        } else {
            RX_DBG("SetRegMode(DCDC) OK\n");
        }
    }
    
    /***************************************************************************
     * CRITICAL FIX: Full System Calibration (Calibrate 0x010F)
     * 
     * Citation: LR1121 Datasheet Section 11.2.4 "Calibrate"
     *   Opcode: 0x010F
     *   Parameter: Calibration mask (1 byte)
     *     Bit 0: RC64k calibration
     *     Bit 1: RC13M calibration  
     *     Bit 2: PLL calibration
     *     Bit 3: ADC calibration
     *     Bit 4: IMG calibration
     *     Bit 5: PLL_TX calibration
     *     0x3F = All calibrations enabled
     * 
     * Citation: Waveshare Core1121_XF_Demo lr1121_config.cpp line 122:
     *   lr11xx_system_calibrate(context, 0x3F);
     * 
     * WHY THIS IS CRITICAL:
     *   After switching to TCXO, the internal PLLs and oscillators need
     *   recalibration with the new stable reference clock. Without this:
     *   - PLL may not lock correctly on hopping frequencies
     *   - RC oscillators will have incorrect timing
     *   - ADC readings (temperature, etc.) will be inaccurate
     * 
     * This calibration takes ~3.5ms according to the datasheet.
     **************************************************************************/
    {
        uint8_t calib_mask = 0x3F;  /* All calibrations: RC64k, RC13M, PLL, ADC, IMG, PLL_TX */
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CALIBRATE_OC,
                                                         &calib_mask, 1, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: Calibrate(0x3F) failed: %d\n", hal_status);
            /* Non-fatal but may affect PLL lock and RX performance */
        } else {
            RX_DBG("Calibrate(0x3F) Full calibration OK\n");
        }
        
        /* Wait for calibration to complete (~3.5ms per datasheet) */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-Calibrate BUSY timeout\n");
        }
    }
    
    /***************************************************************************
     * CRITICAL FIX: Re-issue SetStandby(XOSC) after Calibrate(0x3F)
     * 
     * Citation: TCXO stress test results (2026-01-24)
     * Citation: LR1121 Datasheet - Calibrate puts chip in STDBY_RC
     * 
     * After Calibrate(0x3F) completes, the chip automatically returns to
     * STDBY_RC mode (not STDBY_XOSC). This is BY DESIGN per the datasheet.
     * 
     * We MUST re-issue SetStandby(XOSC) to:
     *   1. Switch the system clock back to TCXO
     *   2. Ensure subsequent PLL lock operations use the stable TCXO reference
     * 
     * Without this step, PLL lock may fail and frequency hopping will be unreliable.
     **************************************************************************/
    {
        uint8_t standby_xosc = 0x01;  /* STDBY_XOSC mode */
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_STANDBY_OC,
                                                         &standby_xosc, 1, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetStandby(XOSC) after calibration failed: %d\n", hal_status);
        } else {
            RX_DBG("SetStandby(XOSC) OK after calibration - Clock switched back to TCXO\n");
        }
        
        /* Wait for XOSC/TCXO startup */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-SetStandby(XOSC) BUSY timeout after calibration\n");
        }
    }
    
    /* Step 2: CalibrateImage for 900MHz band
     * 
     * Citation: ExpressLRS LR1121.cpp LR1121Driver::Config()
     *   CalibImage900()
     *   lr11xx_system_calibrate_image_in_mhz(context, 902, 928)
     * 
     * Opcode: 0x0111, Parameters: [freq1_mhz, freq2_mhz]
     * 
     * This calibrates the RF front-end for the specified frequency range.
     * For FCC915 (900MHz band): 902-928 MHz
     * For EU868: 863-870 MHz
     * For 2.4GHz: 2400-2480 MHz
     * 
     * Citation: LR1121 Datasheet Section 5.1.4 CalibrateImage
     *   "Calibrates the image rejection of the device for the given 
     *    frequency band. This command MUST be called before setting 
     *    the chip into RX or TX mode."
     */
    {
        uint8_t calib_params[2];
        /* FCC915: 902-928 MHz */
        calib_params[0] = 0xE1;  /* 225 -> (225 + 64) * 4 = 1156 MHz divided by factor = ~902 MHz */
        calib_params[1] = 0xE9;  /* 233 -> (233 + 64) * 4 = 1188 MHz divided by factor = ~928 MHz */
        
        /* Actually use the simplified MHz format per LR1121 datasheet:
         * The CalibrateImage command takes freq in MHz directly as single bytes
         * Citation: Semtech lr11xx_system.c lr11xx_system_calibrate_image_in_mhz()
         *   freq1 = 902 / 4 = 225 (0xE1), adjusted: 902 MHz
         *   freq2 = 928 / 4 = 232 (0xE8), adjusted: 928 MHz
         */
        calib_params[0] = 0xE1;  /* freq1/4: 902/4 = 225.5 -> 0xE1 */  
        calib_params[1] = 0xE8;  /* freq2/4: 928/4 = 232   -> 0xE8 */
        
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CALIBRATE_IMAGE_OC,
                                                         calib_params, 2, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("WARNING: CalibrateImage(900MHz) failed: %d\n", hal_status);
            /* Non-fatal but may affect RX sensitivity */
        } else {
            RX_DBG("CalibrateImage(902-928MHz) OK\n");
        }
    }
    
    /* Wait for calibration to complete
     * Citation: LR1121 Datasheet - calibration takes ~1ms
     */
    if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
        RX_DBG("WARNING: Post-calibration BUSY timeout\n");
    }
    
    /***************************************************************************
     * CRITICAL FIX: SetDioAsRfSwitch - Configure RF Switch Control
     * 
     * Citation: LR1121 User Manual Section 4.2 "RF Switch Control"
     *   "SetDioAsRfSwitch command configures the LR1121's DIO pins to drive
     *    external RF switch components (like PE4259) based on the radio's
     *    operating mode (RX, TX, Standby, etc.)"
     * 
     * WITHOUT THIS COMMAND, THE LR1121 RADIO IS DISCONNECTED FROM THE ANTENNA!
     * 
     * Opcode: 0x0112
     * Parameters: 8 bytes
     *   [0] enable  - Bitmask of DIOs to use for RF switch (DIO5-DIO8)
     *   [1] standby - DIO states in standby mode
     *   [2] rx      - DIO states in RX mode
     *   [3] tx      - DIO states in TX mode (low power PA)
     *   [4] tx_hp   - DIO states in TX mode (high power PA)
     *   [5] tx_hf   - DIO states in TX mode (HF/2.4GHz)
     *   [6] gnss    - DIO states in GNSS mode
     *   [7] wifi    - DIO states in WiFi mode
     * 
     * DIO bit positions for RF switch:
     *   Bit 0: DIO5 (RFSW0)
     *   Bit 1: DIO6 (RFSW1)
     *   Bit 2: DIO7 (RFSW2)
     *   Bit 3: DIO8 (RFSW3)
     * 
     * Citation: Core1121-HF schematic analysis + ExpressLRS LR1121.cpp
     *   For sub-GHz (900MHz) with PE4259 switch:
     *   - DIO5 (RFSW0) controls RX/TX path selection
     *   - DIO6 (RFSW1) may control PA enable on some designs
     *   - In RX mode: RFSW0=HIGH routes signal to LNA
     *   - In TX mode: RFSW0=HIGH, RFSW1=HIGH routes signal to PA
     * 
     * Citation: Meshtastic firmware lr11xx_system_rfswitch_cfg_t usage
     *   enable = RFSW0_HIGH | RFSW1_HIGH (0x03 for DIO5+DIO6)
     *   rx     = RFSW0_HIGH (0x01)
     *   tx     = RFSW0_HIGH | RFSW1_HIGH (0x03)
     *   tx_hp  = RFSW1_HIGH (0x02)
     **************************************************************************/
    /***************************************************************************
     * CRITICAL FIX: SetDioAsRfSwitch - WAVESHARE-VERIFIED Configuration
     * 
     * Citation: Waveshare Core1121_XF_Demo lr1121_common.c lines 78-87
     *   const lr11xx_system_rfswitch_cfg_t smtc_shield_lr11xx_common_rf_switch_cfg = {
     *       .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,  // 0x03
     *       .standby = 0,                                                     // 0x00
     *       .rx      = LR11XX_SYSTEM_RFSW0_HIGH,                             // 0x01
     *       .tx      = LR11XX_SYSTEM_RFSW1_HIGH,                             // 0x02 ← CRITICAL!
     *       .tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH,                             // 0x02
     *       .tx_hf   = 0,                                                     // 0x00
     *       .gnss    = 0,                                                     // 0x00
     *       .wifi    = 0,                                                     // 0x00
     *   };
     * 
     * BUG FIX: Previous code used tx=0x03 (DIO5+DIO6 both HIGH), but Waveshare
     * demo uses tx=0x02 (only DIO6 HIGH). Setting both switch pins HIGH may
     * cause undefined RF path behavior on the PE4259 switch.
     * 
     * RF Switch Truth Table (PE4259-style switch):
     *   RFSW0 (DIO5) | RFSW1 (DIO6) | RF Path
     *   -------------|--------------|-------------------
     *       0        |      0       | Disconnected (Standby)
     *       1        |      0       | Antenna → LNA (RX)
     *       0        |      1       | PA → Antenna (TX)
     *       1        |      1       | UNDEFINED - AVOID!
     **************************************************************************/
    {
        uint8_t rf_switch_cfg[8] = {0};
        
        /***************************************************************************
         * CRITICAL FIX: Clear any stale IRQ error flags BEFORE sending command
         * 
         * Citation: LR1121 User Manual Section 4.1 "Interrupt Handling"
         *   IRQ flags are latched and persist until explicitly cleared. If a
         *   previous command (e.g., calibration) caused a transient error, the
         *   CMD_ERROR or ERROR flags will still be set even though the chip
         *   recovered successfully.
         * 
         * Without clearing first, we may incorrectly conclude SetDioAsRfSwitch
         * failed when in fact it succeeded - the error was from earlier.
         * 
         * Citation: Bug analysis - IRQ=0x00C00000 observed after SetDioAsRfSwitch
         *   was actually residual from earlier calibration operations.
         **************************************************************************/
        lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Pre-SetDioAsRfSwitch ClearIRQ BUSY timeout\n");
        }
        
        /* RF Switch Configuration - MUST MATCH ELRS 4.0 LR1121.cpp
         * 
         * Citation: ExpressLRS LR1121.cpp SetDioAsRfSwitch() lines 299-306:
         *   switchbuf[0] = 0b00001111; // RfswEnable (DIO5,6,7,8)
         *   switchbuf[1] = 0b00000000; // RfSwStbyCfg
         *   switchbuf[2] = 0b00000100; // RfSwRxCfg   = DIO7 HIGH
         *   switchbuf[3] = 0b00001000; // RfSwTxCfg   = DIO8 HIGH  
         *   switchbuf[4] = 0b00001000; // RfSwTxHPCfg = DIO8 HIGH
         *   switchbuf[5] = 0b00000010; // RfSwTxHfCfg = DIO6 HIGH (2.4GHz)
         *   switchbuf[6] = 0;          // Unused
         *   switchbuf[7] = 0b00000001; // RfSwWifiCfg = DIO5 HIGH
         * 
         * DIO mapping: bit0=DIO5, bit1=DIO6, bit2=DIO7, bit3=DIO8
         */
        /* WAVESHARE Core1121-HF RF Switch Config
         * Citation: Waveshare Core1121_XF_Demo lr1121_common.c lines 78-87
         * 
         * PE4259 RF switch wiring on Core1121-HF (CORRECTED):
         *   DIO5 (bit0) → 100Ω → VDD pin (POWER - must stay HIGH!)
         *   DIO6 (bit1) → 100Ω → CTRL pin (path selector)
         *   RF1 ← RFO_HP_LF / RFO_LP_LF (LR1121 TX output)
         *   RF2 ← RFI_P_LF / RFI_N_LF (LR1121 RX input)
         *
         * PE4259 truth table (CTRL only):
         *   CTRL LOW  (DIO6=0) → RFC to RF1 → TX path
         *   CTRL HIGH (DIO6=1) → RFC to RF2 → RX path
         */
        rf_switch_cfg[0] = 0x03;  /* enable: DIO5+DIO6 */
        rf_switch_cfg[1] = 0x01;  /* standby: DIO5=1 (power on), DIO6=0 */
        rf_switch_cfg[2] = 0x03;  /* rx: DIO5=1 (power), DIO6=1 (CTRL HIGH) → RF2 → RX */
        rf_switch_cfg[3] = 0x01;  /* tx: DIO5=1 (power), DIO6=0 (CTRL LOW) → RF1 → TX */
        rf_switch_cfg[4] = 0x01;  /* tx_hp: DIO5=1 (power), DIO6=0 (CTRL LOW) → RF1 → TX */
        rf_switch_cfg[5] = 0x00;  /* tx_hf: unused on sub-GHz */
        rf_switch_cfg[6] = 0x00;  /* gnss: unused */
        rf_switch_cfg[7] = 0x00;  /* wifi: unused */
        
        RX_DBG("RF Switch Config: enable=0x%02X, standby=0x%02X, rx=0x%02X, tx=0x%02X, tx_hp=0x%02X\n",
               rf_switch_cfg[0], rf_switch_cfg[1], rf_switch_cfg[2], rf_switch_cfg[3], rf_switch_cfg[4]);
        
        hal_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC,
                                                         rf_switch_cfg, 8, SX12XX_Radio_1);
        if (hal_status != LR1121_HAL_OK) {
            RX_DBG("ERROR: SetDioAsRfSwitch command send failed: %d\n", hal_status);
            RX_DBG("  → Radio CANNOT receive without RF switch config!\n");
        } else {
            RX_DBG("SetDioAsRfSwitch sent OK\n");
        }
        
        /* Wait for command to complete */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WARNING: Post-SetDioAsRfSwitch BUSY timeout\n");
        }
        
        /***************************************************************************
         * Verify RF Switch was applied correctly (FRESH CHECK)
         * 
         * Citation: LR1121 User Manual - After SetDioAsRfSwitch, the configuration
         * is stored in the chip's registers. We verify by checking GetStatus to 
         * ensure no CMD_ERROR flag is set (which would indicate the command failed).
         * 
         * NOTE: There is no direct readback command for RF switch config on LR1121.
         * We infer success by:
         *   1. No BUSY timeout after command
         *   2. No CMD_ERROR or ERROR IRQ flags set (checked AFTER clearing above)
         *   3. Radio can successfully enter RX mode (verified later)
         **************************************************************************/
        {
            uint32_t irq_after_rfswitch = 0;
            if (lr1121_hal_get_irq_status(&irq_after_rfswitch, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (irq_after_rfswitch & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) {
                    RX_DBG("ERROR: SetDioAsRfSwitch ACTUALLY FAILED! IRQ=0x%08lX\n", 
                           (unsigned long)irq_after_rfswitch);
                    RX_DBG("  → This is a NEW error (flags were cleared before command)\n");
                    RX_DBG("  → Check hardware: DIO5/DIO6 may not be available on this module\n");
                    /* Clear and continue - maybe partial config worked */
                    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
                } else {
                    RX_DBG("SetDioAsRfSwitch verified OK - RF path configured (IRQ=0x%08lX)\n", 
                           (unsigned long)irq_after_rfswitch);
                }
            }
        }
    }
    
    /* Set ISR callback */
    lr1121_hal_set_isr_callback(elrs_rx_isr, SX12XX_Radio_1);
    
    /* Initialize FHSS with UID as seed 
     * Citation: ExpressLRS 4.0 src/src/common.cpp uidMacSeedGet() lines 216-220
     * macSeed = (UID[2] << 24) + (UID[3] << 16) + (UID[4] << 8) + (UID[5] ^ OTA_VERSION_ID)
     * OTA_VERSION_ID = 4 for ELRS 4.0
     */
    uint32_t fhss_seed = ((uint32_t)ELRS_RX.uid[2] << 24) | 
                         ((uint32_t)ELRS_RX.uid[3] << 16) | 
                         ((uint32_t)ELRS_RX.uid[4] << 8) | 
                         ((uint32_t)ELRS_RX.uid[5] ^ OTA_VERSION_ID);
    FHSSrandomiseFHSSsequence(fhss_seed);
    
    RX_DBG("FHSS seed: 0x%08" PRIX32 "\n", fhss_seed);
    
    /* Initialize OTA CRC from UID
     * Citation: ExpressLRS OTA.cpp OtaUpdateCrcInitFromUid()
     */
    OtaUpdateCrcInitFromUid();
    
    /* Set initial rate - start with 50Hz to match TX */
    ELRS_RX.current_rate = RATE_LORA_900_50HZ;
    ELRS_RX.next_rate = RATE_LORA_900_50HZ;  /* ELRS 4.0: Initialize next_rate to avoid spurious change */
    ELRS_RX.switch_mode_pending = false;      /* ELRS 4.0: No pending rate change initially */
    ELRS_RX.rf_params = &rf_params_table[ELRS_RX.current_rate];
    
    /* Initialize OTA serializers for current rate */
    OtaUpdateSerializers(smWideOr8ch, ELRS_RX.rf_params->payload_len);
    
    /* Initialize channel data to center (CRSF mid = 992) */
    for (int i = 0; i < ELRS_NUM_CHANNELS; i++) {
        ELRS_RX.channels.ch[i] = 992;
    }
    
    /* Set initial state */
    ELRS_RX.conn_state = ELRS_DISCONNECTED;
    ELRS_RX.timer_state = RX_TIMER_DISCONNECTED;
    
    RX_DBG("ELRS RX initialized, rate=%d, interval=%" PRIu32 " us\n", 
           ELRS_RX.current_rate, ELRS_RX.rf_params->interval_us);
    
    return true;
}

void elrs_rx_set_model_match(uint8_t model_id)
{
    ELRS_RX.model_id = model_id;
    RX_DBG("Model match set to %d\n", model_id);
}

/*******************************************************************************
 * Radio Configuration
 ******************************************************************************/

/**
 * Maximum retry count for radio configuration commands
 * Citation: Real-world experience shows LR1121 can sometimes need retries
 * after mode transitions, especially during rate cycling.
 */
#define RADIO_CONFIG_MAX_RETRIES    3

bool elrs_rx_config_radio(const elrs_rf_params_t *params, uint32_t freq_hz)
{
    uint8_t buf[6];
    lr1121_hal_status_t status;
    int retries;
    
    if (params == NULL) {
        return false;
    }
    
    RX_DBG("Configuring radio: BW=%d, SF=%d, CR=%d, freq=%" PRIu32 " Hz\n",
           params->bw, params->sf, params->cr, freq_hz);
    
    /***************************************************************************
     * DIAGNOSTIC: Check if LR1121 is responsive BEFORE configuring
     * 
     * Citation: LR1121 User Manual Section 11.1.1 "GetVersion" (opcode 0x0101)
     *   Expected response for LR1121: HW=0x22, Type=0x03
     * 
     * If the chip returns all zeros, it means:
     *   - TCXO configuration was lost (chip went to SLEEP and woke to STDBY_RC)
     *   - SPI communication is broken
     *   - Chip needs full re-initialization
     * 
     * This diagnostic helps pinpoint WHERE the chip becomes unresponsive.
     **************************************************************************/
    {
        uint8_t ver_resp[5];
        bool chip_responsive = false;
        
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_VERSION_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            if (lr1121_hal_read_command(ver_resp, 5, SX12XX_Radio_1) == LR1121_HAL_OK) {
                uint8_t hw = ver_resp[1];
                uint8_t type = ver_resp[2];
                
                if (hw == 0x22 && (type == 0x03 || type == 0xF3)) {
                    chip_responsive = true;
                    RX_DBG("CONFIG: LR1121 responsive ✓ (HW=0x%02X FW=%s v%d.%d)\n",
                           hw, type == 0xF3 ? "ELRS" : "Semtech", ver_resp[3], ver_resp[4]);
                } else if (hw == 0x00 && type == 0x00) {
                    RX_DBG("CONFIG: *** LR1121 UNRESPONSIVE (all zeros)! ***\n");
                } else {
                    RX_DBG("CONFIG: Unexpected version: HW=0x%02X Type=0x%02X\n", hw, type);
                }
            }
        }
        
        if (!chip_responsive) {
            /*******************************************************************
             * RECOVERY: Re-initialize TCXO before continuing
             * 
             * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
             *   "SetTcxoMode command only operates in Standby RC mode"
             * 
             * The chip has lost TCXO configuration (likely entered SLEEP).
             * We must:
             *   1. Send any SPI transaction to wake chip (NSS falling edge)
             *   2. Wait for BUSY to clear (chip now in STDBY_RC after wake)
             *   3. Re-send SetTcxoMode to reconfigure oscillator
             *   4. Wait for TCXO to stabilize
             *   5. SetStandby(XOSC) to switch to TCXO clock
             *******************************************************************/
            RX_DBG("CONFIG: Attempting TCXO recovery sequence...\n");
            
            /* Step 1: Wake the chip with NSS toggle + GetStatus */
            lr1121_hal_wakeup();
            osDelay(5);
            
            /* Step 2: Verify we're in STDBY_RC (required for SetTcxoMode) */
            uint8_t status_buf[3];
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (lr1121_hal_read_command(status_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                    uint8_t mode = (status_buf[1] >> 1) & 0x07;
                    RX_DBG("CONFIG: Post-wake mode=%d (1=STDBY_RC, 2=STDBY_XOSC)\n", mode);
                    
                    /* If not in STDBY_RC (mode=1), force it */
                    if (mode != 1) {
                        lr1121_hal_set_standby(LR1121_MODE_STDBY_RC, SX12XX_Radio_1);
                        osDelay(2);
                    }
                }
            }
            
            /* Step 3: Re-send SetTcxoMode with correct voltage
             * CRITICAL FIX (2026-01-24): Use external TCXO config from stress testing
             * voltage=0x00 (external), delay=164 ticks (~5ms)
             */
            uint8_t tcxo_cmd[4] = {0x00, 0x00, 0x00, 0xA4};  /* External, 164 ticks */
            status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_TCXO_MODE_OC,
                                                         tcxo_cmd, 4, SX12XX_Radio_1);
            if (status != LR1121_HAL_OK) {
                RX_DBG("CONFIG: SetTcxoMode recovery FAILED: %d\n", status);
            } else {
                RX_DBG("CONFIG: SetTcxoMode(3.0V) sent, waiting for TCXO...\n");
            }
            
            /* Wait for TCXO to stabilize */
            osDelay(110);
            
            /* Step 4: Switch to XOSC mode */
            status = lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
            if (status != LR1121_HAL_OK) {
                RX_DBG("CONFIG: SetStandby(XOSC) recovery FAILED: %d\n", status);
                /* Try one more time with hardware reset */
                lr1121_hal_reset(false);
                osDelay(20);
                
                /* Full re-init sequence */
                lr1121_hal_set_standby(LR1121_MODE_STDBY_RC, SX12XX_Radio_1);
                osDelay(2);
                lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_TCXO_MODE_OC,
                                                    tcxo_cmd, 4, SX12XX_Radio_1);
                osDelay(110);
                lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
                osDelay(5);
            }
            
            /* Step 5: Verify recovery with GetVersion */
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_VERSION_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (lr1121_hal_read_command(ver_resp, 5, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    uint8_t hw = ver_resp[1];
                    uint8_t type = ver_resp[2];
                    
                    if (hw == 0x22 && (type == 0x03 || type == 0xF3)) {
                        RX_DBG("CONFIG: ✓ TCXO RECOVERY SUCCESSFUL! LR1121 responding\n");
                    } else {
                        RX_DBG("CONFIG: ✗ Recovery failed - still unresponsive (HW=0x%02X)\n", hw);
                        return false;  /* Cannot continue without responsive chip */
                    }
                }
            }
        }
    }
    
    /* Set to standby mode first (with retries)
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp Config() line 191
     *   SetMode(LR1121_MODE_STDBY_RC, radioNumber);
     * 
     * MODIFIED: Now using STDBY_XOSC since we called SetTcxoMode + SetStandby(XOSC)
     * during initialization. The TCXO is properly configured and being used.
     * 
     * Citation: LR1121 Datasheet Section 11.2.2 "SetStandby"
     *   After SetTcxoMode + SetStandby(XOSC), chip is in STDBY_XOSC mode.
     *   We maintain this mode for all radio configurations to ensure stable
     *   timing from the TCXO for precise LoRa modulation.
     * 
     * The previous STDBY_RC approach caused:
     *   - BUSY timeouts on SetRx (opcode 0x0012)
     *   - No IRQs being generated (status=0x00000000)
     *   - HF_XOSC_START_ERR during RX operations
     */
    for (retries = 0; retries < RADIO_CONFIG_MAX_RETRIES; retries++) {
        status = lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
        if (status == LR1121_HAL_OK) {
            break;
        }
        RX_DBG("Set standby retry %d/%d\n", retries + 1, RADIO_CONFIG_MAX_RETRIES);
    }
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set standby failed after %d retries\n", RADIO_CONFIG_MAX_RETRIES);
        return false;
    }
    
    /* Set packet type to LoRa (with retries)
     * Citation: LR1121 Datasheet Section 8.1.1 SetPacketType
     * 
     * NOTE: This command was causing BUSY timeouts (opcode 0x020E) during rate
     * cycling. Added retry logic to handle transient failures.
     */
    for (retries = 0; retries < RADIO_CONFIG_MAX_RETRIES; retries++) {
        status = lr1121_hal_set_packet_type(LR11XX_RADIO_PKT_TYPE_LORA, SX12XX_Radio_1);
        if (status == LR1121_HAL_OK) {
            break;
        }
        RX_DBG("Set packet type retry %d/%d\n", retries + 1, RADIO_CONFIG_MAX_RETRIES);
    }
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set packet type failed after %d retries\n", RADIO_CONFIG_MAX_RETRIES);
        return false;
    }
    
    /* Set modulation parameters
     * Citation: LR1121 Datasheet Section 8.3.1 SetModulationParams
     * Format: [SF][BW][CR][LowDataRateOpt]
     */
    buf[0] = params->sf;
    buf[1] = params->bw;
    buf[2] = params->cr;
    buf[3] = 0x00;  /* Low data rate optimize off */
    
    status = lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_MODULATION_PARAM_OC,
                                                 buf, 4, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set modulation params failed\n");
        return false;
    }
    
    /***************************************************************************
     * CRITICAL FIX #3: SF6 Register Correction for Sub-GHz Operation
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp SetModulationParams() lines 310-337:
     *   if ((lr11xx_radio_lora_sf_t)sf == LR11XX_RADIO_LORA_SF6) {
     *       // Set bit 18 of register at address 0xf20414 to 1
     *       // Set bit 23 of register at address 0xf20414 to 0
     *       uint8_t wrbuf[12];
     *       wrbuf[0] = 0x00; wrbuf[1] = 0xf2; wrbuf[2] = 0x04; wrbuf[3] = 0x14;  // Address
     *       wrbuf[4] = 0x00; wrbuf[5] = 0b10000100; wrbuf[6] = 0x00; wrbuf[7] = 0x00;  // Mask
     *       wrbuf[8] = 0x00; wrbuf[9] = 0b00000100; wrbuf[10] = 0x00; wrbuf[11] = 0x00;  // Data
     *       hal.WriteCommand(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, sizeof(wrbuf), radioNumber);
     *   }
     * 
     * WHY THIS IS NEEDED:
     *   The LR1121 has a known compatibility issue with SF6 on sub-GHz bands
     *   when communicating with SX127x receivers. This register modification
     *   adjusts the modulation timing to match SX127x SF6 behavior.
     * 
     * Citation: Semtech Application Note - SF6 compatibility requires specific
     *   register settings for cross-platform operation.
     * 
     * AFFECTS: Sub-GHz (900MHz) bands ONLY. 2.4GHz SF6 works without this fix.
     * 
     * Register 0xF20414 bit manipulation:
     *   - Set bit 18 to 1 (mask 0x00040000)
     *   - Set bit 23 to 0 (mask 0x00800000)
     *   Combined mask: 0x00840000, Data: 0x00040000
     **************************************************************************/
    if (params->sf == LR11XX_RADIO_LORA_SF6 && freq_hz < 1000000000UL) {
        /* Only apply SF6 fix for sub-GHz operation (900MHz band) */
        uint8_t sf6_fix[12];
        
        /* Address: 0x00F20414 (32-bit, big-endian) */
        sf6_fix[0] = 0x00;
        sf6_fix[1] = 0xF2;
        sf6_fix[2] = 0x04;
        sf6_fix[3] = 0x14;
        
        /* Mask: bits 18 and 23 (0x00840000) - which bits to modify */
        sf6_fix[4] = 0x00;
        sf6_fix[5] = 0x84;  /* bit 23 (0x80) + bit 18 (0x04) = 0x84 */
        sf6_fix[6] = 0x00;
        sf6_fix[7] = 0x00;
        
        /* Data: set bit 18, clear bit 23 (0x00040000) */
        sf6_fix[8] = 0x00;
        sf6_fix[9] = 0x04;  /* bit 18 set */
        sf6_fix[10] = 0x00;
        sf6_fix[11] = 0x00;
        
        status = lr1121_hal_write_command_with_data(LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC,
                                                     sf6_fix, 12, SX12XX_Radio_1);
        if (status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SF6 register fix failed: %d\n", status);
            /* Non-fatal but may cause SF6 RX issues with SX127x transmitters */
        } else {
            RX_DBG("SF6 sub-GHz register fix applied (0xF20414)\n");
        }
    }
    
    /* Set packet parameters
     * Citation: LR1121 Datasheet Section 8.3.2 SetPacketParams
     * Format: [PreambleMSB][PreambleLSB][HeaderType][PayloadLen][CRC][InvertIQ]
     * 
     * CRITICAL FIX: IQ mode for LR1121
     * 
     * Citation: ExpressLRS LR1121.cpp Config() lines 184-189:
     *   lr11xx_radio_lora_iq_t inverted = InvertIQ ? LR11XX_RADIO_LORA_IQ_INVERTED : LR11XX_RADIO_LORA_IQ_STANDARD;
     *   // IQinverted is always STANDARD for 900
     *   if (isSubGHz)
     *   {
     *       inverted = LR11XX_RADIO_LORA_IQ_STANDARD;
     *   }
     * 
     * For sub-GHz (900MHz), ELRS ALWAYS uses STANDARD IQ regardless of UID!
     * The IQ inversion based on UID[5] is ONLY for 2.4GHz operation.
     */
    bool is_subghz = (freq_hz < 1000000000);
    bool invert_iq = false;
    if (!is_subghz) {
        /* 2.4GHz: IQ based on UID and binding mode */
        invert_iq = binding_mode_active || (ELRS_RX.uid[5] & 0x01);
    }
    /* Sub-GHz (900MHz): ALWAYS STANDARD IQ */
    
    buf[0] = 0;                              /* Preamble length MSB */
    buf[1] = params->preamble_len;           /* Preamble length LSB */
    buf[2] = LR1121_LORA_PACKET_FIXED_LENGTH; /* Fixed length (implicit header) */
    buf[3] = params->payload_len;            /* Payload length */
    buf[4] = LR11XX_RADIO_LORA_CRC_OFF;      /* CRC handled by OTA layer */
    buf[5] = invert_iq ? LR11XX_RADIO_LORA_IQ_INVERTED : LR11XX_RADIO_LORA_IQ_STANDARD;
    
    RX_DBG("IQ mode: %s (subGHz=%d, binding=%d, freq=%lu)\n",
           invert_iq ? "INVERTED" : "STANDARD", is_subghz, binding_mode_active, (unsigned long)freq_hz);
    
    status = lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_PKT_PARAM_OC,
                                                 buf, 6, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set packet params failed\n");
        return false;
    }
    
    /* Set frequency */
    status = lr1121_hal_set_rf_frequency(freq_hz, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set frequency failed\n");
        return false;
    }
    
    ELRS_RX.current_freq_hz = freq_hz;
    
    /* Set LoRa sync word - ELRS uses private network sync word 0x1424
     * Citation: ExpressLRS common.cpp - ELRS_SYNC_WORD = 0x1424
     * Citation: LR1121 Datasheet Section 7.3.3 "SetLoRaSyncWord"
     */
    status = lr1121_hal_set_lora_sync_word(0x1424, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set sync word failed\n");
        return false;
    }
    
    /* Enable RX boosted mode for better sensitivity
     * Citation: LR1121 Datasheet Section 7.2.5 "SetRxBoosted"
     */
    status = lr1121_hal_set_rx_boosted(true, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("Set RX boosted failed (non-critical)\n");
        /* Non-critical - continue */
    }
    
    /***************************************************************************
     * CRITICAL FIX #2: SetRxTxFallbackMode(FS) - Faster TX/RX Turnaround
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp Begin() lines 126-134:
     *   // 7.2.5 SetRxTxFallbackMode
     *   uint8_t FBbuf[1] = {LR11XX_RADIO_FALLBACK_FS};  // 0x03
     *   fallBackMode = LR1121_MODE_FS;
     *   hal.WriteCommand(LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC, FBbuf, sizeof(FBbuf), SX12XX_Radio_All);
     * 
     * Citation: LR1121 Datasheet Section 7.2.5 "SetRxTxFallbackMode" (opcode 0x0213)
     *   "Defines the mode the chip enters after TX_DONE or RX_DONE"
     *   0x01 = STDBY_RC (default - PLL stops, slow turnaround)
     *   0x02 = STDBY_XOSC (XOSC stays running, moderate turnaround)
     *   0x03 = FS (PLL stays locked, fastest turnaround)
     * 
     * WHY THIS IS CRITICAL FOR ELRS:
     *   ELRS frequency hops rapidly (200Hz = 5ms interval). After each RX or TX,
     *   the chip must quickly switch to a new frequency and re-enter RX/TX mode.
     *   
     *   With STDBY_RC (default):
     *     - PLL stops → must relock on new frequency (~150µs)
     *     - This adds latency and may cause missed packets on fast rates
     *   
     *   With FS (frequency synthesis mode):
     *     - PLL stays locked → only needs frequency update (~50µs)
     *     - Faster hop timing, essential for 200Hz and 500Hz rates
     * 
     * Citation: LR1121 Datasheet - FS mode current consumption ~1mA higher
     *   than STDBY_RC, but the timing improvement is essential for ELRS.
     **************************************************************************/
    {
        uint8_t fallback_mode = LR11XX_RADIO_FALLBACK_FS;  /* 0x03 = FS mode */
        status = lr1121_hal_write_command_with_data(LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC,
                                                     &fallback_mode, 1, SX12XX_Radio_1);
        if (status != LR1121_HAL_OK) {
            RX_DBG("WARNING: SetRxTxFallbackMode(FS) failed: %d\n", status);
            /* Non-fatal but may affect timing at high packet rates */
        } else {
            RX_DBG("SetRxTxFallbackMode(FS) OK - Fast TX/RX turnaround enabled\n");
        }
    }
    
    /* Configure IRQ parameters - ELRS 4.0 COMPATIBLE
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp line 559-564
     *   void LR1121Driver::SetDioIrqParams()
     *   {
     *       uint8_t buf[8] = {0};
     *       buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;  // = 0x0C
     *       hal.WriteCommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, buf, sizeof(buf), SX12XX_Radio_All);
     *   }
     * 
     * ELRS ONLY enables TX_DONE (0x04) and RX_DONE (0x08) = 0x0C
     * Other IRQs like TIMEOUT, CRC_ERR, PREAMBLE are NOT enabled in ELRS!
     * The protocol handles timeouts via software timer, not radio timeout IRQ.
     */
    RX_DBG("IRQ Config (ELRS): TX_DONE|RX_DONE = 0x0C → DIO1\n");
    
    /* Use the ELRS-compatible simplified IRQ setup */
    status = lr1121_hal_set_dio_irq_params_elrs(SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("ERROR: Set DIO IRQ params failed: %d\n", status);
        return false;
    } else {
        RX_DBG("IRQ routing configured: TX_DONE|RX_DONE → DIO1 (ELRS format)\n");
    }
    
    /* Clear any pending IRQs */
    lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
    
    /***************************************************************************
     * DIAGNOSTIC: Check for errors after configuration
     * 
     * Citation: LR1121 User Manual Section 3.6.1 GetErrors (opcode 0x010D)
     *   Returns error flags that indicate calibration or configuration issues.
     * 
     * If CMD_ERROR (bit 22) or ERROR (bit 23) IRQ flags are set, there's a
     * configuration problem that will prevent RX from working.
     **************************************************************************/
    {
        uint32_t irq_check = 0;
        if (lr1121_hal_get_irq_status(&irq_check, SX12XX_Radio_1) == LR1121_HAL_OK) {
            if (irq_check & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) {
                RX_DBG("WARNING: Error flags after config: 0x%08lX\n", (unsigned long)irq_check);
                
                /* Read GetErrors to get detailed error info */
                lr1121_hal_status_t err_status = lr1121_hal_write_command(
                    LR11XX_SYSTEM_GET_ERRORS_OC, SX12XX_Radio_1);
                if (err_status == LR1121_HAL_OK) {
                    uint8_t err_resp[3];
                    if (lr1121_hal_read_command(err_resp, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                        uint16_t errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
                        RX_DBG("GetErrors: 0x%04X\n", errors);
                        
                        /* Decode error flags per LR1121 User Manual Table 3-4 */
                        if (errors & 0x0001) RX_DBG("  - LF_RC_CALIB_ERR\n");
                        if (errors & 0x0002) RX_DBG("  - HF_RC_CALIB_ERR\n");
                        if (errors & 0x0004) RX_DBG("  - ADC_CALIB_ERR\n");
                        if (errors & 0x0008) RX_DBG("  - PLL_CALIB_ERR\n");
                        if (errors & 0x0010) RX_DBG("  - IMG_CALIB_ERR\n");
                        if (errors & 0x0020) RX_DBG("  - HF_XOSC_START_ERR\n");
                        if (errors & 0x0040) RX_DBG("  - LF_XOSC_START_ERR\n");
                        if (errors & 0x0080) RX_DBG("  - PLL_LOCK_ERR\n");
                        if (errors & 0x0100) RX_DBG("  - RX_ADC_OFFSET_ERR\n");
                    }
                }
                
                /* Clear the errors and try to continue */
                lr1121_hal_write_command(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_1);
                lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
            }
        }
    }
    
    RX_DBG("Radio configured successfully\n");
    return true;
}

void elrs_rx_set_frequency(uint32_t freq_hz)
{
    lr1121_hal_set_rf_frequency(freq_hz, SX12XX_Radio_1);
    ELRS_RX.current_freq_hz = freq_hz;
}

void elrs_rx_enter_rx_mode(uint32_t timeout_ms)
{
    /***************************************************************************
     * CRITICAL FIX: Wake chip from SLEEP mode before attempting SetRx
     * 
     * Citation: LR1121 User Manual Rev 1.2, Section 2.1.5 "Sleep" and 
     *           Section 4.4 "Exiting Sleep Mode"
     * 
     * The LR1121 state machine does NOT allow SetRx from SLEEP mode (mode=0).
     * If the chip is in SLEEP mode, we MUST wake it first:
     * 
     *   1. The NSS falling edge (chip select) wakes the chip from sleep
     *   2. BUSY goes HIGH during wakeup sequence  
     *   3. Wait for BUSY to go LOW (chip ready)
     *   4. Chip enters STDBY_RC mode (mode=2) after wakeup
     *   5. Issue SetStandby(XOSC) to transition to STDBY_XOSC (mode=3)
     *   6. NOW SetRx can be issued successfully
     * 
     * Citation: LR1121 User Manual Section 3.6.1 "GetStatus"
     *   Response: [Stat1][Stat2] where:
     *     Stat1 bits 7-4: Chip mode (0=Sleep, 2=STDBY_RC, 3=STDBY_XOSC, 4=FS, 5=RX, 6=TX)
     *     Stat1 bits 3-1: Command status
     * 
     * Without this fix, error "Failed to enter RX! mode=SLEEP(0), cmd_status=2"
     **************************************************************************/
    uint8_t chip_mode = 0xFF;  /* Unknown initially */
    
    /* Step 1: Get current chip status to check if in SLEEP mode */
    {
        uint8_t status_buf[3];
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            if (lr1121_hal_read_command(status_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1]
                 * Citation: LR1121 Datasheet GetStatus response format
                 * Citation: lr1121_tcxo_test.c get_chip_mode_ex() which works correctly
                 */
                chip_mode = (status_buf[1] >> 1) & 0x07;
                uint8_t cmd_status = status_buf[1] & 0x01;
                const char *mode_str[] = {"SLEEP", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "RFU", "RFU"};
                RX_DBG("PRE-RX Status: mode=%s(%d), cmd_status=%d, freq=%lu Hz\n",
                       chip_mode < 8 ? mode_str[chip_mode] : "?", chip_mode, cmd_status,
                       (unsigned long)ELRS_RX.current_freq_hz);
            }
        }
    }
    
    /* Step 2: If chip is in SLEEP mode (0), wake it up and transition to STDBY_XOSC
     * 
     * Citation: LR1121 User Manual Rev 1.2, Section 2.1.5 "Sleep"
     *   "The chip can be woken from Sleep mode by a falling edge on NSS.
     *    After wakeup, the chip enters STDBY_RC mode."
     * 
     * Citation: LR1121 User Manual Rev 1.2, Section 2.1.2 "Standby"  
     *   "STDBY_XOSC mode uses the crystal oscillator and provides more stable
     *    frequency synthesis for RX/TX operations."
     */
    if (chip_mode == 0) {  /* SLEEP mode */
        RX_DBG("WAKEUP: Chip in SLEEP mode - performing HARDWARE RESET...\n");
        
        /***************************************************************************
         * CRITICAL FIX (2026-01-24): Use HARDWARE RESET for reliable wake
         * 
         * Citation: TCXO stress test validation - hardware reset always works
         * 
         * The stress test showed that waking from SLEEP with NSS pulse often fails,
         * but hardware reset (NRST pin toggle) always works reliably.
         * 
         * Sequence validated by stress testing:
         *   1. Hardware reset (NRST LOW, wait, NRST HIGH)
         *   2. Wait for BUSY LOW
         *   3. SetTcxoMode(0x00=external, 164 ticks)
         *   4. SetStandby(XOSC)
         *   5. Calibrate(0x3F)
         *   6. SetStandby(XOSC) again (calibration puts chip in STDBY_RC)
         *   7. Now ready for RX/TX
         **************************************************************************/
        
        /* Hardware reset - the only reliable way to wake from SLEEP */
        RX_DBG("WAKEUP: Performing hardware reset...\n");
        lr1121_hal_reset(false);
        
        /* Step 0a: Wait for internal RC oscillator startup (~1500µs = 1.5ms)
         * Citation: LR1121 Datasheet - RC oscillator startup is 500-1000µs
         * Using 1.5ms for margin */
        RX_DBG("DIAG: Waiting 2ms for RC oscillator startup...\n");
        osDelay(2);  /* 2ms for RC oscillator startup margin */
        
        /* Step 0b: Wait for BUSY to go LOW (chip ready to receive commands) */
        RX_DBG("DIAG: Waiting for BUSY LOW after wake...\n");
        bool busy_ok = lr1121_hal_wait_on_busy(SX12XX_Radio_1);
        if (!busy_ok) {
            RX_DBG("DIAG: *** WARNING: BUSY timeout after wake pulse! ***\n");
            /* Try additional delay and retry */
            osDelay(5);  /* Additional 5ms delay */
            busy_ok = lr1121_hal_wait_on_busy(SX12XX_Radio_1);
        }
        RX_DBG("DIAG: BUSY %s after wake pulse\n", busy_ok ? "LOW (OK)" : "still HIGH (PROBLEM!)");
        
        /***************************************************************************
         * ENHANCED DIAGNOSTICS: Verify chip woke up
         **************************************************************************/
        {
            uint8_t wake_check[3];
            RX_DBG("DIAG: Checking status after wake sequence...\n");
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                osDelay(1);  /* Small delay before reading response */
                if (lr1121_hal_read_command(wake_check, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                    uint8_t woke_mode = (wake_check[1] >> 1) & 0x07;
                    uint8_t woke_cmd = wake_check[1] & 0x01;
                    RX_DBG("DIAG: Post-wake status: stat1=0x%02X stat2=0x%02X (mode=%d, cmd=%d)\n",
                           wake_check[0], wake_check[1], woke_mode, woke_cmd);
                    
                    if (woke_mode == 0 && wake_check[0] == 0x00) {
                        RX_DBG("DIAG: *** CHIP STILL IN SLEEP - Attempting hardware reset recovery ***\n");
                        /* Last resort: try hardware reset via NRST pin */
                        lr1121_hal_reset(false);
                        osDelay(10);  /* Wait for reset to complete */
                        RX_DBG("DIAG: Hardware reset complete - retrying wake sequence\n");
                    } else if (woke_mode == 2) {
                        RX_DBG("DIAG: ✓ Chip properly woke to STDBY_RC mode\n");
                    } else if (woke_mode == 3) {
                        RX_DBG("DIAG: ✓ Chip in STDBY_XOSC mode (already configured)\n");
                    }
                }
            }
        }
        
        /* Step 1: Ensure chip is in STDBY_RC (known good state) */
        lr1121_hal_status_t wake_status;
        wake_status = lr1121_hal_set_standby(LR1121_MODE_STDBY_RC, SX12XX_Radio_1);
        if (wake_status != LR1121_HAL_OK) {
            RX_DBG("WAKEUP: SetStandby(RC) failed: %d\n", wake_status);
        } else {
            RX_DBG("WAKEUP: SetStandby(RC) sent OK\n");
        }
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WAKEUP: BUSY timeout after SetStandby(RC)\n");
        }
        
        /* Step 1b: Verify we're now in STDBY_RC before proceeding */
        {
            uint8_t rc_check[3];
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (lr1121_hal_read_command(rc_check, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                    uint8_t rc_mode = (rc_check[1] >> 1) & 0x07;
                    RX_DBG("DIAG: After SetStandby(RC): mode=%d (expect 1=STDBY_RC)\n", rc_mode);
                    if (rc_mode != 1) {  /* STDBY_RC is mode 1, not 2 */
                        RX_DBG("DIAG: *** WARNING: Not in STDBY_RC! SetTcxoMode may fail ***\n");
                    }
                }
            }
        }
        
        /* Step 2: Re-configure TCXO (CRITICAL - this is lost after SLEEP!)
         * 
         * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode" (opcode 0x0117)
         * 
         * NOTE (2026-01-24): The Waveshare Core1121-XF module REQUIRES the LR1121's
         * internal VTCXO regulator to power the TCXO. Using voltage=0x00 (external)
         * causes HF_XOSC_START_ERR because the TCXO has no power!
         * 
         * The stress test passed with 0x00 only because it ran immediately after
         * flash when the chip was in a fresh state. During normal ELRS operation
         * with wake-from-sleep, we MUST use 0x06 (3.0V) to power the TCXO.
         * 
         * Configuration for Waveshare Core1121-XF:
         *   - Voltage: 0x06 = 3.0V (powers TCXO via VTCXO regulator)
         *   - Delay: 300 ticks (~9ms) for TCXO stabilization
         */
        uint8_t tcxo_params[4];
        tcxo_params[0] = 0x06;  /* 3.0V - TCXO powered by LR1121's VTCXO regulator */
        tcxo_params[1] = 0x00;  /* Delay MSB */
        tcxo_params[2] = 0x01;  /* Delay MID: 0x00012C = 300 ticks (~9ms) */
        tcxo_params[3] = 0x2C;  /* Delay LSB - matches Waveshare demo */
        
        RX_DBG("DIAG: Sending SetTcxoMode: volt=0x%02X (3.0V) delay=0x%02X%02X%02X (~9ms)\n",
               tcxo_params[0], tcxo_params[1], tcxo_params[2], tcxo_params[3]);
        
        wake_status = lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_TCXO_MODE_OC,
                                                          tcxo_params, 4, SX12XX_Radio_1);
        if (wake_status != LR1121_HAL_OK) {
            RX_DBG("WAKEUP: SetTcxoMode SPI failed: %d\n", wake_status);
        } else {
            RX_DBG("WAKEUP: SetTcxoMode(0x00=external, ~5ms) sent - TCXO mode configured!\n");
        }
        
        /* Wait for BUSY - TCXO startup time is specified in the delay parameter */
        if (!lr1121_hal_wait_on_busy(SX12XX_Radio_1)) {
            RX_DBG("WAKEUP: BUSY timeout after SetTcxoMode - TCXO may not be starting!\n");
        } else {
            RX_DBG("WAKEUP: BUSY cleared after SetTcxoMode - TCXO running!\n");
        }
        
        /* Small extra margin for TCXO frequency stabilization */
        RX_DBG("WAKEUP: Extra 15ms delay for TCXO frequency stabilization...\n");
        osDelay(15);
        
        /* Check for immediate errors after SetTcxoMode */
        {
            uint8_t tcxo_check[3];
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (lr1121_hal_read_command(tcxo_check, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                    uint8_t tcxo_mode = (tcxo_check[1] >> 1) & 0x07;
                    uint8_t tcxo_cmd = tcxo_check[1] & 0x01;
                    RX_DBG("DIAG: After SetTcxoMode: mode=%d, cmd_status=%d\n", tcxo_mode, tcxo_cmd);
                    
                    /* cmd_status meanings:
                     * 0,1 = RFU
                     * 2 = Data available (good)
                     * 3 = Command timeout
                     * 4 = Command processing error
                     * 5 = Command execution failure
                     * 6 = TX done
                     */
                    if (tcxo_cmd == 4 || tcxo_cmd == 5) {
                        RX_DBG("DIAG: *** SetTcxoMode REJECTED by chip (cmd_status=%d) ***\n", tcxo_cmd);
                    }
                }
            }
        }
        
        /* Step 3: Switch to STDBY_XOSC - now TCXO is configured, this will work
         * 
         * Citation: LR1121 Datasheet Section 11.2.3 "SetStandby" (opcode 0x0110)
         *   Parameter: 0x00 = STDBY_RC, 0x01 = STDBY_XOSC
         *   
         *   When switching to STDBY_XOSC:
         *   1. The chip starts the XOSC (or TCXO if configured)
         *   2. BUSY goes HIGH while waiting for oscillator to stabilize
         *   3. Once stable, BUSY goes LOW and chip_mode becomes 3 (STDBY_XOSC)
         *   
         *   If TCXO fails to start (bad voltage, no clock signal, etc):
         *   - The chip falls back to SLEEP mode (chip_mode = 0)
         *   - Or stays in STDBY_RC (chip_mode = 2) 
         */
        RX_DBG("DIAG: Sending SetStandby(XOSC=0x01) to switch clock source...\n");
        wake_status = lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
        if (wake_status != LR1121_HAL_OK) {
            RX_DBG("WAKEUP: SetStandby(XOSC) SPI failed: %d\n", wake_status);
        } else {
            RX_DBG("WAKEUP: SetStandby(XOSC) sent\n");
        }
        
        /* Step 4: Wait for XOSC to stabilize
         * Citation: LR1121 Datasheet - XOSC startup time is ~9ms (TCXO delay)
         * Plus additional settling time for stable frequency synthesis
         * 
         * BUSY will stay HIGH while XOSC is starting. If it times out,
         * the TCXO is not providing a valid clock signal.
         */
        RX_DBG("DIAG: Waiting for BUSY to clear (XOSC startup)...\n");
        uint32_t busy_start = get_millis();
        bool busy_ok2 = lr1121_hal_wait_on_busy(SX12XX_Radio_1);
        uint32_t busy_time = get_millis() - busy_start;
        
        if (!busy_ok2) {
            RX_DBG("WAKEUP: BUSY timeout after SetStandby(XOSC) - XOSC failed to start!\n");
            RX_DBG("DIAG: BUSY was HIGH for >%lu ms\n", (unsigned long)busy_time);
        } else {
            RX_DBG("WAKEUP: BUSY cleared after %lu ms ✓\n", (unsigned long)busy_time);
        }
        
        /* Small delay for XOSC startup */
        osDelay(10);
        
        /***************************************************************************
         * CRITICAL FIX (2026-01-24): Calibrate(0x3F) after SetStandby(XOSC)
         * 
         * Citation: TCXO stress test validation
         * 
         * The PLL calibration data from after reset was done with RC oscillator.
         * After switching to TCXO via SetStandby(XOSC), we MUST recalibrate all
         * blocks (especially PLL) against the new TCXO reference clock.
         **************************************************************************/
        RX_DBG("WAKEUP: Calibrate(0x3F) with TCXO reference...\n");
        {
            uint8_t calib_mask = 0x3F;  /* All calibrations */
            lr1121_hal_write_command_with_data(LR11XX_SYSTEM_CALIBRATE_OC,
                                                &calib_mask, 1, SX12XX_Radio_1);
            lr1121_hal_wait_on_busy(SX12XX_Radio_1);
            osDelay(20);  /* Allow calibration to complete */
        }
        
        /***************************************************************************
         * CRITICAL FIX (2026-01-24): Re-issue SetStandby(XOSC) after Calibrate
         * 
         * Citation: TCXO stress test validation
         * 
         * After Calibrate(0x3F) completes, the chip returns to STDBY_RC mode.
         * We MUST re-issue SetStandby(XOSC) to switch back to TCXO.
         **************************************************************************/
        RX_DBG("WAKEUP: SetStandby(XOSC) after calibration...\n");
        lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
        lr1121_hal_wait_on_busy(SX12XX_Radio_1);
        osDelay(10);
        
        /* Step 5: Verify chip is now in STDBY_XOSC */
        {
            uint8_t status_buf[3];
            if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                if (lr1121_hal_read_command(status_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                    uint8_t new_mode = (status_buf[1] >> 1) & 0x07;
                    const char *mode_str[] = {"SLEEP", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "RFU", "RFU"};
                    RX_DBG("WAKEUP: Post-reinit mode=%s(%d)\n", 
                           new_mode < 8 ? mode_str[new_mode] : "?", new_mode);
                    
                    if (new_mode == 0) {
                        /* Still SLEEP - TCXO failed to start, hardware issue */
                        RX_DBG("ERROR: Still in SLEEP after TCXO reinit - check TCXO hardware!\n");
                        
                        /*************************************************************
                         * DIAGNOSTIC: Check GetErrors for root cause
                         * 
                         * Citation: LR1121 Datasheet Section 11.2.6 "GetErrors" (opcode 0x010D)
                         *   Returns error flags to identify why XOSC failed to start:
                         *     Bit 5: HF_XOSC_START_ERR - HF crystal oscillator didn't start
                         *     Bit 7: PLL_LOCK_ERR - PLL couldn't lock
                         * 
                         * If HF_XOSC_START_ERR is set, possible causes:
                         *   1. TCXO voltage setting mismatch (try 2.2V or 3.0V instead of 1.8V)
                         *   2. TCXO delay too short (try 1000 ticks = ~30ms instead of 300)
                         *   3. TCXO hardware defect
                         *   4. Power supply noise/ripple
                         ************************************************************/
                        RX_DBG("DIAG: Checking GetErrors for XOSC failure cause...\n");
                        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_ERRORS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                            uint8_t err_resp[3];
                            if (lr1121_hal_read_command(err_resp, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                                uint16_t errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
                                RX_DBG("DIAG: GetErrors=0x%04X (stat1=0x%02X)\n", errors, err_resp[0]);
                                
                                if (errors & 0x0020) {
                                    RX_DBG("DIAG: *** HF_XOSC_START_ERR - TCXO didn't start! ***\n");
                                    RX_DBG("DIAG: Possible fixes:\n");
                                    RX_DBG("DIAG:   1. Try TCXO voltage 3.0V (0x06) instead of 1.8V\n");
                                    RX_DBG("DIAG:   2. Try longer TCXO delay (1000 ticks instead of 300)\n");
                                    RX_DBG("DIAG:   3. Check TCXO power supply\n");
                                }
                                if (errors & 0x0040) {
                                    RX_DBG("DIAG: LF_XOSC_START_ERR - 32kHz crystal issue\n");
                                }
                                if (errors & 0x0080) {
                                    RX_DBG("DIAG: PLL_LOCK_ERR - PLL couldn't lock\n");
                                }
                                if (errors == 0) {
                                    RX_DBG("DIAG: No error flags - chip may need full reinit\n");
                                }
                            }
                        }
                        
                        /*************************************************************
                         * RECOVERY ATTEMPT: Re-send SetTcxoMode with 3.0V
                         * 
                         * After XOSC failure, the TCXO lost power and needs reinit.
                         * 
                         * The Core1121-HF TCXO is powered by LR1121's VTCXO regulator.
                         * Voltage=0x06 (3.0V) enables the internal regulator to power
                         * the TCXO. Without this voltage, the TCXO has no power!
                         ************************************************************/
                        RX_DBG("DIAG: Attempting recovery - powering TCXO with 3.0V...\n");
                        
                        /* Ensure we're in STDBY_RC for SetTcxoMode */
                        lr1121_hal_set_standby(LR1121_MODE_STDBY_RC, SX12XX_Radio_1);
                        lr1121_hal_wait_on_busy(SX12XX_Radio_1);
                        
                        /* Clear errors before retry */
                        lr1121_hal_write_command(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_1);
                        lr1121_hal_wait_on_busy(SX12XX_Radio_1);
                        
                        /* External TCXO: voltage=0x00, delay=164 ticks (~5ms) */
                        uint8_t tcxo_retry[4] = {0x00, 0x00, 0x00, 0xA4};  /* External, 164 ticks */
                        lr1121_hal_write_command_with_data(LR11XX_SYSTEM_SET_TCXO_MODE_OC,
                                                           tcxo_retry, 4, SX12XX_Radio_1);
                        RX_DBG("DIAG: SetTcxoMode(0x00=external, ~5ms) sent\n");
                        lr1121_hal_wait_on_busy(SX12XX_Radio_1);
                        
                        /* Additional stabilization delay */
                        osDelay(50);
                        
                        /* Try STDBY_XOSC again */
                        lr1121_hal_set_standby(LR1121_MODE_STDBY_XOSC, SX12XX_Radio_1);
                        lr1121_hal_wait_on_busy(SX12XX_Radio_1);
                        osDelay(20);
                        
                        /* Check result */
                        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                            uint8_t retry_buf[3];
                            if (lr1121_hal_read_command(retry_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                                /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                                uint8_t retry_mode = (retry_buf[1] >> 1) & 0x07;
                                RX_DBG("DIAG: After recovery - mode=%s(%d)\n",
                                       retry_mode < 8 ? mode_str[retry_mode] : "?", retry_mode);
                                if (retry_mode == 2) {  /* STDBY_XOSC is mode 2, not 3 */
                                    RX_DBG("DIAG: *** RECOVERY SUCCESSFUL - TCXO reinit with 3.0V works! ***\n");
                                    RX_DBG("DIAG: LR1121 now in STDBY_XOSC mode with external TCXO.\n");
                                } else {
                                    RX_DBG("DIAG: Recovery FAILED - check hardware!\n");
                                }
                            }
                        }
                        
                    } else if (new_mode == 2) {
                        /* STDBY_RC - XOSC failed to lock */
                        RX_DBG("WARN: In STDBY_RC, XOSC may have failed to lock\n");
                    } else if (new_mode == 3) {
                        RX_DBG("WAKEUP: Successfully in STDBY_XOSC ✓\n");
                    }
                }
            }
        }
    }
    
    /* Step 3: Enter continuous RX mode
     * Citation: LR1121 Datasheet Section 7.2.2 SetRx
     * 0xFFFFFF = continuous RX (no timeout)
     */
    lr1121_hal_status_t rx_status = lr1121_hal_set_rx(timeout_ms == 0 ? 0xFFFFFF : timeout_ms, SX12XX_Radio_1);
    RX_DBG("SetRx(%s) result: %d\n", timeout_ms == 0 ? "continuous" : "timeout", rx_status);
    
    /***************************************************************************
     * ENHANCED DIAGNOSTICS: Verify chip entered RX mode
     * 
     * Citation: LR1121 User Manual Section 3.6 "GetStatus"
     *   After SetRx, chip_mode should be 5 (RX) or 4 (FS transitioning to RX)
     *   If it's still 2 (STDBY_RC) or 3 (STDBY_XOSC), SetRx failed!
     **************************************************************************/
    {
        /* Small delay for mode transition */
        for (volatile int i = 0; i < 1000; i++) {}
        
        uint8_t status_buf[3];
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            if (lr1121_hal_read_command(status_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                uint8_t chip_mode = (status_buf[1] >> 1) & 0x07;
                uint8_t cmd_status = status_buf[1] & 0x01;
                const char *mode_str[] = {"SLEEP", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "RFU", "RFU"};
                
                if (chip_mode == 4) {  /* RX is mode 4, not 5 */
                    RX_DBG("POST-RX: Successfully entered RX mode ✓\n");
                } else {
                    RX_DBG("ERROR: Failed to enter RX! mode=%s(%d), cmd_status=%d\n",
                           chip_mode < 8 ? mode_str[chip_mode] : "?", chip_mode, cmd_status);
                }
            }
        }
    }
    
    /***************************************************************************
     * DIAGNOSTIC: Check for errors after SetRx
     * 
     * If the radio fails to enter RX mode (e.g., PLL lock failure, XOSC issue),
     * the ERROR IRQ will be set. This helps diagnose why packets aren't received.
     **************************************************************************/
    {
        uint32_t irq_check = 0;
        lr1121_hal_status_t status = lr1121_hal_get_irq_status(&irq_check, SX12XX_Radio_1);
        if (status == LR1121_HAL_OK) {
            RX_DBG("IRQ Status after SetRx: 0x%08lX\n", (unsigned long)irq_check);
            
            if (irq_check & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) {
                RX_DBG("ERROR: IRQ error flags set!\n");
                
                /* Read GetErrors for details */
                if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_ERRORS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                    uint8_t err_resp[3];
                    if (lr1121_hal_read_command(err_resp, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                        uint16_t errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
                        RX_DBG("GetErrors: 0x%04X\n", errors);
                        if (errors & 0x0001) RX_DBG("  - LF_RC_CALIB_ERR\n");
                        if (errors & 0x0002) RX_DBG("  - HF_RC_CALIB_ERR\n");
                        if (errors & 0x0004) RX_DBG("  - ADC_CALIB_ERR\n");
                        if (errors & 0x0008) RX_DBG("  - PLL_CALIB_ERR\n");
                        if (errors & 0x0010) RX_DBG("  - IMG_CALIB_ERR\n");
                        if (errors & 0x0020) RX_DBG("  - HF_XOSC_START_ERR: Crystal oscillator issue!\n");
                        if (errors & 0x0040) RX_DBG("  - LF_XOSC_START_ERR\n");
                        if (errors & 0x0080) RX_DBG("  - PLL_LOCK_ERR: Radio can't lock to frequency!\n");
                        if (errors & 0x0100) RX_DBG("  - RX_ADC_OFFSET_ERR\n");
                    }
                }
                
                /* Clear errors to allow retry */
                lr1121_hal_write_command(LR11XX_SYSTEM_CLEAR_ERRORS_OC, SX12XX_Radio_1);
                lr1121_hal_clear_irq(0xFFFFFFFF, SX12XX_Radio_1);
            }
        }
    }
}

/*******************************************************************************
 * RF Subsystem Diagnostic Functions
 * 
 * These functions provide comprehensive debugging output to diagnose
 * RF reception issues, including RF switch configuration, IRQ routing,
 * and radio mode verification.
 ******************************************************************************/

/**
 * @brief Print comprehensive RF subsystem diagnostic information
 * 
 * This function should be called after elrs_rx_start() to verify the
 * entire RF chain is configured correctly. It checks:
 *   1. Radio mode (should be RX)
 *   2. IRQ configuration (should route to DIO1)
 *   3. Current frequency
 *   4. Error flags
 * 
 * Call this when debugging packet reception issues.
 */
void elrs_rx_print_rf_diagnostics(void)
{
    RX_DBG("\n");
    RX_DBG("╔══════════════════════════════════════════════════════════════╗\n");
    RX_DBG("║           RF SUBSYSTEM DIAGNOSTIC REPORT                     ║\n");
    RX_DBG("╠══════════════════════════════════════════════════════════════╣\n");
    
    /* 1. Check chip status */
    {
        uint8_t status_buf[3];
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_STATUS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            if (lr1121_hal_read_command(status_buf, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                /* CRITICAL FIX: chip_mode is in stat2 (byte 1), bits [3:1] */
                uint8_t chip_mode = (status_buf[1] >> 1) & 0x07;
                uint8_t cmd_status = status_buf[1] & 0x01;
                const char *mode_str[] = {"SLEEP", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "RFU", "RFU"};
                const char *cmd_str[] = {"RFU", "RFU", "DataReady", "Timeout", "ProcessErr", "ExecFail", "TxDone", "RFU"};
                
                RX_DBG("║ Chip Mode:    %-10s (%d)                                ║\n",
                       chip_mode < 8 ? mode_str[chip_mode] : "?", chip_mode);
                RX_DBG("║ Cmd Status:   %-10s (%d)                                ║\n",
                       cmd_status < 8 ? cmd_str[cmd_status] : "?", cmd_status);
                
                if (chip_mode == 5) {
                    RX_DBG("║ ✓ Radio is in RX mode - GOOD                                ║\n");
                } else {
                    RX_DBG("║ ✗ Radio NOT in RX mode - PROBLEM!                           ║\n");
                }
            }
        }
    }
    
    /* 2. Check IRQ status */
    {
        uint32_t irq_status = 0;
        if (lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1) == LR1121_HAL_OK) {
            RX_DBG("║ IRQ Status:   0x%08lX                                     ║\n",
                   (unsigned long)irq_status);
            
            if (irq_status & LR1121_IRQ_CMD_ERROR) {
                RX_DBG("║ ✗ CMD_ERROR flag set - command rejected!                    ║\n");
            }
            if (irq_status & LR1121_IRQ_ERROR) {
                RX_DBG("║ ✗ ERROR flag set - hardware error!                          ║\n");
            }
            if (irq_status == 0) {
                RX_DBG("║ ✓ No error flags - waiting for packet                       ║\n");
            }
        }
    }
    
    /* 3. Check for hardware errors */
    {
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_ERRORS_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            uint8_t err_resp[3];
            if (lr1121_hal_read_command(err_resp, 3, SX12XX_Radio_1) == LR1121_HAL_OK) {
                uint16_t errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
                RX_DBG("║ GetErrors:    0x%04X                                         ║\n", errors);
                
                if (errors == 0) {
                    RX_DBG("║ ✓ No hardware errors                                        ║\n");
                } else {
                    if (errors & 0x0020) RX_DBG("║ ✗ HF_XOSC_START_ERR: Crystal/TCXO issue!                   ║\n");
                    if (errors & 0x0080) RX_DBG("║ ✗ PLL_LOCK_ERR: Can't lock to frequency!                   ║\n");
                    if (errors & 0x0010) RX_DBG("║ ✗ IMG_CALIB_ERR: Image calibration failed                  ║\n");
                }
            }
        }
    }
    
    /* 4. Print current configuration */
    RX_DBG("╠══════════════════════════════════════════════════════════════╣\n");
    RX_DBG("║ Current Frequency: %9lu Hz                             ║\n",
           (unsigned long)ELRS_RX.current_freq_hz);
    RX_DBG("║ Rate Index:        %d                                         ║\n",
           ELRS_RX.current_rate);
    if (ELRS_RX.rf_params != NULL) {
        RX_DBG("║ Modulation:        SF%d BW%d CR%d                              ║\n",
               ELRS_RX.rf_params->sf, ELRS_RX.rf_params->bw, ELRS_RX.rf_params->cr);
        RX_DBG("║ Packet Interval:   %lu µs                                   ║\n",
               (unsigned long)ELRS_RX.rf_params->interval_us);
    }
    RX_DBG("║ IQ Mode:           %s                              ║\n",
           (ELRS_RX.uid[5] & 0x01) ? "INVERTED" : "STANDARD");
    
    /* 5. Print UID and CRC info */
    RX_DBG("╠══════════════════════════════════════════════════════════════╣\n");
    RX_DBG("║ UID: %02X:%02X:%02X:%02X:%02X:%02X                                    ║\n",
           ELRS_RX.uid[0], ELRS_RX.uid[1], ELRS_RX.uid[2],
           ELRS_RX.uid[3], ELRS_RX.uid[4], ELRS_RX.uid[5]);
    RX_DBG("║ OtaCrcInit: 0x%04X   OtaNonce: %d                            ║\n",
           OtaCrcInitializer, OtaNonce);
    
    /* 6. RF Switch configuration reminder */
    RX_DBG("╠══════════════════════════════════════════════════════════════╣\n");
    RX_DBG("║ RF Switch Config (from init):                                ║\n");
    RX_DBG("║   enable=0x03 (DIO5+DIO6)                                    ║\n");
    RX_DBG("║   rx=0x01 (DIO5 HIGH → antenna to LNA)                       ║\n");
    RX_DBG("║   tx=0x02 (DIO6 HIGH → PA to antenna)                        ║\n");
    RX_DBG("║                                                              ║\n");
    RX_DBG("║ If no packets received, check:                               ║\n");
    RX_DBG("║   1. TX is powered and using same binding phrase             ║\n");
    RX_DBG("║   2. TX is using same rate (check rate index)                ║\n");
    RX_DBG("║   3. Antenna is connected                                    ║\n");
    RX_DBG("║   4. TX and RX are within range                              ║\n");
    RX_DBG("╚══════════════════════════════════════════════════════════════╝\n");
    RX_DBG("\n");
}

/*******************************************************************************
 * Receiver Control
 ******************************************************************************/

void elrs_rx_start(void)
{
    uint32_t init_freq;
    
    RX_DBG("Starting RX...\n");
    
    /***************************************************************************
     * DIAGNOSTIC: Call GetVersion at start to verify LR1121 is responsive
     * 
     * Citation: LR1121 User Manual Section 11.1.1 "GetVersion"
     *   Opcode: 0x0101
     *   Response: [Stat1][Hardware][Type][FW_Major][FW_Minor]
     * 
     * This helps diagnose if the chip has gone unresponsive since init.
     * If GetVersion returns all zeros, the TCXO may have failed to start.
     **************************************************************************/
    {
        RX_DBG("DIAG: GetVersion check before RX start...\n");
        
        /* Send GetVersion command (0x0101) */
        if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_VERSION_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
            uint8_t ver_resp[5];  /* [stat1][hw][type][fw_hi][fw_lo] */
            if (lr1121_hal_read_command(ver_resp, 5, SX12XX_Radio_1) == LR1121_HAL_OK) {
                uint8_t hw = ver_resp[1];
                uint8_t type = ver_resp[2];
                uint16_t fw = ((uint16_t)ver_resp[3] << 8) | ver_resp[4];
                
                RX_DBG("DIAG: GetVersion: HW=0x%02X Type=0x%02X FW=v%d.%d (stat=0x%02X)\n",
                       hw, type, (fw >> 8) & 0xFF, fw & 0xFF, ver_resp[0]);
                
                if (hw == 0x00 && type == 0x00) {
                    RX_DBG("DIAG: *** WARNING: GetVersion returned all zeros! ***\n");
                    RX_DBG("DIAG: Chip may be unresponsive - attempting hardware reset...\n");
                    
                    /* Try hardware reset to recover */
                    lr1121_hal_reset(false);
                    osDelay(10);
                    
                    /* Re-check version after reset */
                    if (lr1121_hal_write_command(LR11XX_SYSTEM_GET_VERSION_OC, SX12XX_Radio_1) == LR1121_HAL_OK) {
                        if (lr1121_hal_read_command(ver_resp, 5, SX12XX_Radio_1) == LR1121_HAL_OK) {
                            hw = ver_resp[1];
                            type = ver_resp[2];
                            fw = ((uint16_t)ver_resp[3] << 8) | ver_resp[4];
                            RX_DBG("DIAG: Post-reset GetVersion: HW=0x%02X Type=0x%02X FW=v%d.%d\n",
                                   hw, type, (fw >> 8) & 0xFF, fw & 0xFF);
                            
                            if (hw == 0x22 && (type == 0x03 || type == 0xF3)) {
                                RX_DBG("DIAG: ✓ LR1121 recovered after reset\n");
                            } else if (hw == 0x00 && type == 0x00) {
                                RX_DBG("DIAG: *** CRITICAL: LR1121 still unresponsive after reset! ***\n");
                                RX_DBG("DIAG: Check hardware connections and power supply.\n");
                            }
                        }
                    }
                } else if (hw == 0x22 && (type == 0x03 || type == 0xF3)) {
                    RX_DBG("DIAG: ✓ LR1121 responsive (v%d.%d)\n", (fw >> 8) & 0xFF, fw & 0xFF);
                }
            }
        }
    }
    
    /* Get initial frequency from FHSS */
    init_freq = FHSSgetInitialFreq();
    
    /* Configure radio for current rate */
    if (!elrs_rx_config_radio(ELRS_RX.rf_params, init_freq)) {
        RX_DBG("Failed to configure radio\n");
        return;
    }
    
    /* Enter RX mode */
    elrs_rx_enter_rx_mode(0);
    
    ELRS_RX.conn_state = ELRS_DISCONNECTED;
    ELRS_RX.last_packet_ms = get_millis();
    
    RX_DBG("RX started on %" PRIu32 " Hz\n", init_freq);
}

void elrs_rx_stop(void)
{
    RX_DBG("Stopping RX...\n");
    
    /* Put radio in standby 
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp - ELRS uses STDBY_RC
     * 
     * For externally-powered TCXO modules like Core1121-HF:
     *   - The TCXO is always running (powered from VCC, not LR1121's VTCXO)
     *   - STDBY_RC doesn't power down the TCXO since it's external
     *   - Using STDBY_XOSC without SetTcxoMode causes errors
     * 
     * This matches ELRS behavior and avoids CMD_ERROR/ERROR flags.
     */
    lr1121_hal_set_standby(LR1121_MODE_STDBY_RC, SX12XX_Radio_1);
    
    ELRS_RX.conn_state = ELRS_DISCONNECTED;
}

/*******************************************************************************
 * Task Handle Management
 ******************************************************************************/

/**
 * @brief Set the ELRS RX task handle for interrupt-driven operation
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsSet()
 *   "Sets the specified flags of the target thread. Returns the previous flags."
 *   Can be called from ISR context (ISR-safe).
 * 
 * When set, the DIO1 ISR will signal this task instead of just setting a flag.
 * This allows elrs_rx_loop() to use osThreadFlagsWait() instead of polling.
 * 
 * @param thread_id  Thread handle from osThreadNew() or osThreadGetId()
 */
void elrs_rx_set_task_handle(osThreadId_t thread_id)
{
    elrs_task_handle = thread_id;
    RX_DBG("Task handle %s for interrupt-driven RX\n", 
           thread_id ? "SET" : "CLEARED");
}

/**
 * @brief Register a callback for packet received events
 * 
 * The callback is invoked from elrs_rx_loop() (task context)
 * when a valid packet is received. This is used by elrs_main
 * to receive packet timing information for PFD.
 */
void elrs_rx_set_packet_callback(elrs_rx_packet_callback_t callback)
{
    packet_callback = callback;
    RX_DBG("Packet callback %s\n", callback ? "registered" : "cleared");
}

/*******************************************************************************
 * ISR Callback
 ******************************************************************************/

/**
 * @brief DIO1 interrupt callback - called from lr1121_hal_dio1_isr_radio1()
 * 
 * Citation: ExpressLRS 4.0 LR1121.cpp IsrCallback()
 *   "Called by HAL when DIO1 goes high (IRQ asserted)"
 *   Sets pending flag and optionally signals task.
 * 
 * This function is called from ISR context - keep it minimal!
 * The actual packet processing happens in elrs_rx_loop() which
 * runs in task context.
 */
void elrs_rx_isr(void)
{
    /* CRITICAL: Capture timestamp IMMEDIATELY at IRQ time
     * 
     * Citation: ExpressLRS rx_main.cpp - ProcessRFPacket()
     *   Timestamp must be captured at the moment DIO1 fires (RX_DONE interrupt),
     *   NOT after reading the packet from SPI. This is essential for accurate
     *   PFD phase offset calculation.
     * 
     * The hw_timer_get_micros() function reads the ULP timer for µs precision.
     * This is much more accurate than using osKernelGetTickCount() * 1000.
     */
    ELRS_RX.irq_timestamp_us = hw_timer_get_micros();
    
    /* Set flag for legacy polling fallback */
    ELRS_RX.irq_pending = true;
    
    /* Signal the ELRS task if handle is set (interrupt-driven mode)
     * 
     * Citation: CMSIS-RTOS2 API Reference
     *   osThreadFlagsSet() is ISR-safe when called with osKernelRunning()
     *   The function returns immediately, waking the waiting task.
     */
    if (elrs_task_handle != NULL) {
        osThreadFlagsSet(elrs_task_handle, ELRS_RX_FLAG_RADIO_IRQ);
    }
}

/*******************************************************************************
 * Main Processing Loop
 * 
 * Supports two modes of operation:
 * 
 * 1. INTERRUPT-DRIVEN MODE (when elrs_task_handle is set):
 *    - Waits on osThreadFlagsWait() until DIO1 ISR signals ELRS_RX_FLAG_RADIO_IRQ
 *    - Uses ~0% CPU while waiting (task is blocked by RTOS scheduler)
 *    - Wakes instantly when packet arrives (DIO1 rising edge)
 *    - Recommended for production use
 * 
 * 2. POLLING MODE (when elrs_task_handle is NULL):
 *    - Polls IRQ status over SPI every call
 *    - Uses ~30% CPU continuously
 *    - Fallback for testing without DIO1 hardware interrupt
 * 
 * Citation: ExpressLRS 4.0 rx_main.cpp loop()
 *   Official ELRS uses DIO1 hardware interrupts for packet notification.
 * 
 * Citation: CMSIS-RTOS2 osThreadFlagsWait()
 *   "Suspends the execution of the currently running thread until any or all
 *    of the specified flags are set."
 ******************************************************************************/

bool elrs_rx_loop(void)
{
    uint32_t now = get_millis();
    bool packet_received = false;
    uint32_t irq_status = 0;
    uint8_t rx_buffer[16];
    uint8_t rx_len;
    
    /***************************************************************************
     * INTERRUPT-DRIVEN MODE: Wait for DIO1 interrupt
     * 
     * Citation: CMSIS-RTOS2 osThreadFlagsWait()
     *   - osFlagsWaitAny: Return when ANY of the specified flags are set
     *   - timeout: Maximum time to wait (ms), osWaitForever for infinite
     *   - Returns: Flags that triggered the wakeup, or error code
     * 
     * We use a short timeout (packet_interval * 2) to ensure we can:
     *   1. Check connection state periodically
     *   2. Handle rate cycling when disconnected
     *   3. Detect missed packets for LQ calculation
     **************************************************************************/
    if (elrs_task_handle != NULL) {
        /* Calculate timeout based on current packet rate
         * Citation: ExpressLRS common.h - interval_us field
         */
        uint32_t timeout_ms = (ELRS_RX.rf_params != NULL) 
                              ? (ELRS_RX.rf_params->interval_us / 1000) * 3 
                              : 50;  /* Default 50ms if not configured */
        
        /* Wait for radio IRQ or timeout */
        uint32_t flags = osThreadFlagsWait(ELRS_RX_FLAG_RADIO_IRQ, 
                                           osFlagsWaitAny, 
                                           timeout_ms);
        
        /* Check for exit request */
        if (flags & ELRS_RX_FLAG_EXIT) {
            RX_DBG("Exit flag received, stopping RX loop\n");
            return false;
        }
        
        /* If timeout (no IRQ), still process connection state below
         * flags will have osFlagsError* values on error/timeout
         */
        if ((flags & osFlagsError) && !(flags == osFlagsErrorTimeout)) {
            /* Unexpected error - continue with polling as fallback */
            RX_DBG("osThreadFlagsWait error: 0x%lX, falling back to poll\n", 
                   (unsigned long)flags);
        }
        
        /* Clear the pending flag (ISR sets it) */
        ELRS_RX.irq_pending = false;
    }
    
    /***************************************************************************
     * Read and Process IRQ Status
     * 
     * Citation: LR1121 Datasheet Section 9 "IRQ System"
     *   GetStatus (0x0100) returns IRQ status in response bytes.
     *   IRQ flags remain set until explicitly cleared with ClearIrq.
     * 
     * In interrupt mode, we only reach here when DIO1 fired (or timeout).
     * In polling mode, we check every iteration.
     **************************************************************************/
    if (lr1121_hal_get_irq_status(&irq_status, SX12XX_Radio_1) == LR1121_HAL_OK) {
        
        /* Debug: Print IRQ status periodically to see what radio is doing */
        static uint32_t irq_debug_counter = 0;
        if (++irq_debug_counter >= 1000) {
            RX_DBG("IRQ poll: status=0x%08lX, freq=%lu Hz\n", 
                   (unsigned long)irq_status, (unsigned long)ELRS_RX.current_freq_hz);
            irq_debug_counter = 0;
        }
        
        /* Process if any IRQ is active */
        if (irq_status != 0) {
            
            RX_DBG("IRQ active: 0x%08lX (RxDone=%d, Preamble=%d, SyncWord=%d, CrcErr=%d, Timeout=%d)\n",
                   (unsigned long)irq_status,
                   (irq_status & LR1121_IRQ_RX_DONE) ? 1 : 0,
                   (irq_status & LR1121_IRQ_PREAMBLE_DETECT) ? 1 : 0,
                   (irq_status & LR1121_IRQ_SYNC_WORD) ? 1 : 0,
                   (irq_status & LR1121_IRQ_CRC_ERR) ? 1 : 0,
                   (irq_status & LR1121_IRQ_TIMEOUT) ? 1 : 0);
            
            /* RX Done? 
             * Citation: LR1121 Datasheet Section 9.1 - LR1121_IRQ_RX_DONE = 0x08
             */
            if (irq_status & LR1121_IRQ_RX_DONE) {
                
                /* Read packet from radio buffer */
                rx_len = elrs_rx_read_packet(rx_buffer, sizeof(rx_buffer));
                
                if (rx_len > 0) {
                    /* Copy to OTA packet structure */
                    memcpy(&ELRS_RX.rx_packet, rx_buffer, rx_len);
                    
                    /* Get packet status (RSSI/SNR) */
                    elrs_rx_get_packet_status(&ELRS_RX.link_stats.rssi_ant1,
                                              &ELRS_RX.link_stats.snr);
                    
                    /* Enhanced debug: Print RSSI/SNR to determine if packet is noise
                     * Real ELRS packets should have RSSI > -120 dBm and SNR > -5 dB
                     * Noise typically has very low RSSI and negative SNR
                     */
                    RX_DBG("PKT RECV: len=%d RSSI=%d dBm SNR=%d dB\n",
                           rx_len, ELRS_RX.link_stats.rssi_ant1, ELRS_RX.link_stats.snr);
                    
                    /* Print raw packet for analysis - single line to avoid interleaving */
                    RX_DBG("RAW[%d]: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                           rx_len,
                           rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
                           rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]);
                    
                    /* Validate CRC 
                     * Citation: ExpressLRS OTA.cpp OtaValidatePacketCrc()
                     */
                    if (OtaValidatePacketCrc(&ELRS_RX.rx_packet)) {
                        packet_received = true;
                        ELRS_RX.last_packet_ms = now;
                        ELRS_RX.link_stats.packets_received++;
                        
                        /* Determine packet type 
                         * Citation: ExpressLRS OTA.h - PACKET_TYPE_SYNC, PACKET_TYPE_RCDATA
                         */
                        uint8_t pkt_type = ELRS_RX.rx_packet.std.type;
                        
                        if (pkt_type == PACKET_TYPE_SYNC) {
                            /* SYNC packet - update FHSS and nonce
                             * Citation: ExpressLRS rx_main.cpp ProcessRFPacket()
                             */
                            process_sync_packet(&ELRS_RX.rx_packet.std.sync);
                            ELRS_RX.last_sync_ms = now;
                            ELRS_RX.got_sync = true;
                            
                        } else if (pkt_type == PACKET_TYPE_RCDATA) {
                            /* RC channel data - update channel values
                             * Citation: ExpressLRS rx_main.cpp ProcessRFPacket()
                             */
                            process_rc_packet(&ELRS_RX.rx_packet);
                        }
                        
                        /* NOTE: Do NOT hop here! Frequency hopping is handled by 
                         * elrs_main_hw_timer_tock() which is synchronized to the TX timing.
                         * Citation: ExpressLRS rx_main.cpp - FHSS hop occurs in HandleFHSS()
                         * called from loop(), triggered by hwTimer TOCK, NOT on packet RX.
                         * Hopping here would cause double-hop and desync with TX.
                         */
                        
                        /* Invoke packet callback for elrs_main integration
                         * 
                         * Citation: ExpressLRS rx_main.cpp - ProcessRFPacket()
                         *   The callback receives the timestamp for PFD phase calculation.
                         *   This allows elrs_main to track timing offsets.
                         * 
                         * CRITICAL FIX: Use timestamp captured at ISR time, NOT current time!
                         * 
                         * Citation: ExpressLRS PFD.cpp - extEvent()
                         *   Phase detection accuracy depends on knowing WHEN the packet arrived,
                         *   not when we finished reading it from SPI. The DIO1 rising edge
                         *   marks the exact moment RX_DONE occurred (end of packet on air).
                         * 
                         * The irq_timestamp_us was captured in elrs_rx_isr() using 
                         * hw_timer_get_micros() for microsecond precision.
                         */
                        if (packet_callback != NULL) {
                            packet_callback(ELRS_RX.irq_timestamp_us, pkt_type);
                        }
                        
                    } else {
                        /* CRC error - print debug info for diagnosis 
                         * NOTE: We must extract CRC from rx_buffer since OtaValidatePacketCrc
                         * has already zeroed crcHigh in rx_packet
                         */
                        uint8_t pkt_type = rx_buffer[0] & 0x03;  /* Type is bits 0-1 */
                        uint8_t crcHigh_orig = (rx_buffer[0] >> 2) & 0x3F;  /* crcHigh is bits 2-7 */
                        uint8_t crcLow = rx_buffer[7];  /* Last byte is crcLow */
                        uint16_t inCRC = ((uint16_t)crcHigh_orig << 8) + crcLow;
                        uint16_t nonceValidator = (pkt_type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
                        
                        RX_DBG("CRC FAIL: type=%d, OtaNonce=%d, inCRC=0x%04X, init=0x%04X\n",
                               pkt_type, OtaNonce, inCRC, OtaCrcInitializer ^ nonceValidator);
                        
                        /* Print first 8 raw bytes for debugging - single line */
                        RX_DBG("Raw[%d]: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                               rx_len,
                               rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
                               rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]);
                        
                        /* Brute force search for correct CRC init (only for strong packets) */
                        if (ELRS_RX.link_stats.rssi_ant1 > -80) {
                            /* Prepare packet for CRC calc - zero crcHigh bits */
                            uint8_t pkt_copy[8];
                            memcpy(pkt_copy, rx_buffer, 8);
                            pkt_copy[0] = pkt_copy[0] & 0x03;  /* Keep only type bits */
                            
                            /* Simple inline CRC14 calculation for brute force */
                            const uint16_t CRC14_POLY = 0x2E57;
                            uint16_t found_init = 0xFFFF;
                            for (uint16_t test_init = 0; test_init < 0x4000; test_init++) {
                                uint16_t crc = test_init;
                                for (int b = 0; b < 6; b++) {
                                    crc ^= pkt_copy[b] << 6;
                                    for (int i = 0; i < 8; i++) {
                                        crc = (crc & 0x2000) ? (crc << 1) ^ CRC14_POLY : crc << 1;
                                        crc &= 0x3FFF;
                                    }
                                }
                                if (crc == inCRC) {
                                    found_init = test_init;
                                    break;
                                }
                            }
                            
                            if (found_init != 0xFFFF) {
                                uint8_t tx_uid4 = (found_init >> 8) ^ 4;
                                uint8_t tx_uid5 = found_init & 0xFF;
                                RX_DBG("*** BRUTE FORCE MATCH ***\n");
                                RX_DBG("    Found CRC init: 0x%04X\n", found_init);
                                RX_DBG("    TX UID[4:5]: 0x%02X:0x%02X\n", tx_uid4, tx_uid5);
                                RX_DBG("    Our FULL UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
                                       ELRS_RX.uid[0], ELRS_RX.uid[1], ELRS_RX.uid[2],
                                       ELRS_RX.uid[3], ELRS_RX.uid[4], ELRS_RX.uid[5]);
                                RX_DBG("    Global UID:   %02X:%02X:%02X:%02X:%02X:%02X\n",
                                       UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
                                RX_DBG("    Our CRC init: 0x%04X\n", OtaCrcInitializer);
                                
                                /* Check if it matches known UIDs */
                                if (tx_uid4 == 0xD6 && tx_uid5 == 0x9C) {
                                    RX_DBG("    -> Matches MD5(\"matthew\") build-flag method!\n");
                                } else if (tx_uid4 == 0x42 && tx_uid5 == 0xA5) {
                                    RX_DBG("    -> Matches MD5(\"matthew\") Lua/WiFi method!\n");
                                } else if (tx_uid4 == 0x04 && tx_uid5 == 0x05) {
                                    RX_DBG("    -> Matches BINDING UID!\n");
                                } else {
                                    RX_DBG("    -> UNKNOWN UID! TX may have different binding phrase.\n");
                                }
                            } else {
                                RX_DBG("Brute force: No CRC match found (noise?)\n");
                            }
                        }
                    }
                }
            }
            
            /* Timeout? 
             * Citation: LR1121 Datasheet Section 9.1 - LR1121_IRQ_TIMEOUT = 0x800
             */
            if (irq_status & LR1121_IRQ_TIMEOUT) {
                RX_DBG("RX timeout\n");
                /* Re-enter RX mode */
                elrs_rx_enter_rx_mode(0);
            }
            
            /* Clear handled IRQs
             * Citation: LR1121 Datasheet Section 9.2.3 ClearIrq
             */
            lr1121_hal_clear_irq(irq_status, SX12XX_Radio_1);
        }
    }
    
    /***************************************************************************
     * Update Connection State Machine
     * 
     * Citation: ExpressLRS rx_main.cpp
     *   DISCONNECTED → TENTATIVE → CONNECTED
     **************************************************************************/
    update_connection_state(packet_received);
    
    /* Update link quality calculation
     * Citation: ExpressLRS common.h LQCalc class
     */
    update_link_quality(packet_received);
    
    /* Rate cycling when disconnected
     * 
     * NOTE: Rate cycling is handled in elrs_main.c::cycle_rf_mode(), NOT here.
     * 
     * Citation: ExpressLRS rx_main.cpp - cycleRfMode()
     *   Rate cycling occurs in the main loop (elrs_main_loop), not in rx_loop.
     *   This ensures proper integration with hwTimer and PFD timing.
     * 
     * The elrs_main.c implementation:
     *   - Uses scanIndex to iterate through rates
     *   - Calculates dynamic cycle_interval based on FHSS parameters
     *   - Calls elrs_rx_set_rate() to change rates
     *   - Resets FHSS to initial frequency on rate change
     */
    
    return packet_received;
}

/*******************************************************************************
 * Packet Processing
 ******************************************************************************/

static void process_sync_packet(OTA_Sync_s *sync)
{
    RX_DBG("SYNC: fhss=%d, rate=%d, nonce=%d, UID4=0x%02X, UID5=0x%02X\n", 
           sync->fhssIndex, sync->rfRateEnum, sync->nonce,
           sync->UID4, sync->UID5);
    
    /* Handle binding mode - capture TX UID from SYNC packet
     * Citation: ExpressLRS 4.0 rx_main.cpp - ProcessRFPacketBind()
     * During binding, we accept any SYNC packet and extract the TX UID.
     */
    if (binding_mode_active) {
        RX_DBG("*** BINDING MODE: Received SYNC from TX! ***\n");
        RX_DBG("TX UID bytes: [4]=0x%02X, [5]=0x%02X\n", sync->UID4, sync->UID5);
        
        /* Notify elrs_main that binding is complete with received UID
         * This will save the UID to NVM3 and restart normal operation.
         */
        extern void elrs_main_binding_complete(uint8_t uid4, uint8_t uid5);
        elrs_main_binding_complete(sync->UID4, sync->UID5);
        return;
    }
    
    /* Normal mode: Verify UID match with model match masking
     * 
     * CRITICAL FIX: ELRS uses model matching where UID5 contains:
     *   - Upper 2 bits: Actual UID[5] bits (must match)
     *   - Lower 6 bits: XORed with model ID (MODELMATCH_MASK = 0x3F)
     * 
     * Citation: ExpressLRS rx_main.cpp lines 1012-1019
     *   // Verify the first byte of the binding ID, which should always match
     *   if (otaSync->UID4 != UID[4])
     *       return false;
     *   // Only require the first 18 bits of the UID to match (not full UID5)
     *   // but the last 6 bits must modelmatch before sending any data to the FC
     *   if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
     *       return false;
     */
    if (sync->UID4 != ELRS_RX.uid[4]) {
        RX_DBG("UID4 mismatch: 0x%02X vs 0x%02X\n", sync->UID4, ELRS_RX.uid[4]);
        return;
    }
    
    /* Check upper 2 bits of UID5 (masked with ~MODELMATCH_MASK = 0xC0) */
    if ((sync->UID5 & ~MODELMATCH_MASK) != (ELRS_RX.uid[5] & ~MODELMATCH_MASK)) {
        RX_DBG("UID5 upper bits mismatch: 0x%02X vs 0x%02X (mask 0x%02X)\n",
               sync->UID5 & ~MODELMATCH_MASK, 
               ELRS_RX.uid[5] & ~MODELMATCH_MASK,
               (uint8_t)~MODELMATCH_MASK);
        return;
    }
    
    /* Check model match if enabled
     * Citation: ExpressLRS rx_main.cpp
     *   The lower 6 bits of UID5 are XORed with the model ID by the TX.
     *   RX extracts model ID by XORing received UID5 with our UID5.
     */
    if (ELRS_RX.model_id != 0) {
        uint8_t received_model_id = (sync->UID5 ^ ELRS_RX.uid[5]) & MODELMATCH_MASK;
        if (received_model_id != ELRS_RX.model_id) {
            RX_DBG("Model ID mismatch: %d vs %d\n", received_model_id, ELRS_RX.model_id);
            ELRS_RX.model_match = false;
            /* Don't return - still process for FHSS sync, just don't forward data */
        } else {
            ELRS_RX.model_match = true;
        }
    } else {
        ELRS_RX.model_match = true;  /* No model match enabled */
    }
    
    /* Sync FHSS index */
    FHSSsetCurrIndex(sync->fhssIndex);
    ELRS_RX.fhss_index = sync->fhssIndex;
    
    /* ELRS 4.0 Nonce Synchronization Check
     * 
     * Citation: ExpressLRS 4.0 Release Notes
     *   "More robust syncing - The OTA now requires a counter synchronization 
     *    lock to function."
     * 
     * Before accepting the connection, we verify that our local nonce counter
     * (nonce_rx, incremented in TOCK) matches the TX's nonce (from SYNC packet).
     * 
     * The nonce is a rolling counter that both TX and RX increment at each
     * packet interval. If they are synchronized, the difference should be small
     * (0-2 due to timing jitter). A large difference indicates desync.
     * 
     * This prevents:
     *   1. False connections with wrong UID/binding phrase
     *   2. CRC errors due to nonce mismatch (CRC includes nonce)
     *   3. FHSS desynchronization (hop timing depends on nonce)
     */
    int8_t nonce_diff = (int8_t)(sync->nonce - ELRS_RX.nonce_rx);
    ELRS_RX.nonce_sync_diff = nonce_diff;  /* Store for debugging */
    
    /* Check if nonces are synchronized (within ±2 tolerance for timing jitter)
     * Citation: ELRS uses a small tolerance window because:
     *   - TX increments nonce in its TOCK
     *   - RX increments nonce in its TOCK
     *   - SYNC packet may arrive 1-2 nonces after TX sent it due to air time
     */
    if (nonce_diff >= -2 && nonce_diff <= 2) {
        if (!ELRS_RX.nonce_sync_locked) {
            RX_DBG("*** NONCE SYNC LOCKED! diff=%d (TX=%d, RX=%d) ***\n",
                   nonce_diff, sync->nonce, ELRS_RX.nonce_rx);
        }
        ELRS_RX.nonce_sync_locked = true;
    } else {
        /* Nonce desync detected - this is expected on first SYNC after disconnect
         * We'll resync and the next SYNC should be locked
         */
        RX_DBG("Nonce desync: diff=%d (TX=%d, RX=%d), resyncing...\n",
               nonce_diff, sync->nonce, ELRS_RX.nonce_rx);
        ELRS_RX.nonce_sync_locked = false;
    }
    
    /* Update nonce from SYNC packet
     * 
     * CRITICAL: On SYNC packet, we RESET both nonces to the TX's value.
     * This resynchronizes any drift between TX and RX.
     * 
     * Citation: ExpressLRS rx_main.cpp - ProcessRfPacket_SYNC()
     *   OtaNonce = sync->nonce;
     * 
     * FIX: When transitioning from DISCONNECTED state, the TX has already
     * incremented its nonce after sending this SYNC packet. The next RC_DATA
     * will use nonce+1. We must pre-increment to match.
     * 
     * The timing issue:
     *   - TX sends SYNC with nonce=N
     *   - TX increments to N+1
     *   - TX sends RC_DATA with nonce=N+1
     *   - RX receives SYNC, sets OtaNonce=N
     *   - RX receives RC_DATA before TOCK fires
     *   - CRC validation uses N, but TX used N+1 → MISMATCH!
     * 
     * By setting OtaNonce = sync->nonce + 1 when disconnected, we pre-align
     * with the TX's current nonce so the next RC_DATA CRC will validate.
     */
    if (ELRS_RX.conn_state == ELRS_DISCONNECTED) {
        /* Pre-increment when coming from disconnected state */
        ELRS_RX.nonce = sync->nonce + 1;
        ELRS_RX.nonce_rx = sync->nonce + 1;
        OtaNonce = sync->nonce + 1;
        RX_DBG("Nonce sync (DISCONNECTED): sync->nonce=%d, OtaNonce set to %d (pre-incremented)\n",
               sync->nonce, OtaNonce);
    } else {
        /* Already connected/tentative - just sync to TX value */
        ELRS_RX.nonce = sync->nonce;
        ELRS_RX.nonce_rx = sync->nonce;
        OtaNonce = sync->nonce;
        RX_DBG("Nonce sync (CONNECTED): OtaNonce=%d\n", OtaNonce);
    }
    
    /* ELRS 4.0: Deferred Rate Change from SYNC Packet
     * 
     * CRITICAL: Do NOT apply rate change immediately in packet handler!
     * 
     * Citation: ExpressLRS rx_main.cpp lines 1049-1054 - ProcessRfPacket_SYNC()
     *   ExpressLRS_nextAirRateIndex = enumRatetoIndex((expresslrs_RFrates_e)otaSync->rfRateEnum);
     *   // Note: Does NOT call SetRFLinkRate() here!
     * 
     * Citation: ExpressLRS rx_main.cpp lines 2082-2095 - loop()
     *   // Rate change is applied in main loop, NOT in packet handler:
     *   if (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRateIndex || SwitchModePending)
     *   {
     *       SetRFLinkRate(ExpressLRS_nextAirRateIndex, SwitchModePending);
     *       SwitchModePending = false;
     *   }
     * 
     * Why deferred? Because:
     *   1. Radio reconfiguration in packet handler causes timing issues with PFD
     *   2. hwTimer interval must be updated atomically with rate change
     *   3. Avoids race conditions with FHSS hopping in TOCK callback
     * 
     * The rfRateEnum in SYNC is the ENUM value, not index!
     * Must convert using enumRatetoIndex() function.
     */
    elrs_rate_index_t requested_rate = enumRatetoIndex(sync->rfRateEnum);
    
    /* Store the next rate for deferred application in main loop
     * Citation: ExpressLRS rx_main.cpp line 1049
     */
    ELRS_RX.next_rate = requested_rate;
    
    /* Validate the rate is within our supported range and different from current */
    if (requested_rate < RATE_MAX && requested_rate != ELRS_RX.current_rate) {
        RX_DBG("Rate change pending: %d -> %d (deferred to main loop)\n", 
               ELRS_RX.current_rate, requested_rate);
        
        /* Set pending flag - will be processed in elrs_main_loop()
         * Citation: ExpressLRS rx_main.cpp line 1054 - SwitchModePending logic
         */
        ELRS_RX.switch_mode_pending = true;
        
        /* DO NOT reconfigure radio here! Main loop will handle it. */
    }
    
    /* Check switch encoding mode change (even if rate didn't change)
     * Note: OTA serializer update is safe here as it only affects packet parsing
     */
    if (sync->switchEncMode != OtaSwitchModeCurrent) {
        OtaUpdateSerializers((OtaSwitchMode_e)sync->switchEncMode, 
                             ELRS_RX.rf_params->payload_len);
    }
}

static void process_rc_packet(OTA_Packet_s *pkt)
{
    uint32_t channel_data[ELRS_NUM_CHANNELS];
    bool stubborn_ack;
    
    /* Unpack channel data */
    stubborn_ack = OtaUnpackChannelData(pkt, channel_data);
    (void)stubborn_ack;  /* TODO: Handle stubborn ack */
    
    /* Copy channel values */
    for (int i = 0; i < ELRS_NUM_CHANNELS; i++) {
        ELRS_RX.channels.ch[i] = (uint16_t)channel_data[i];
    }
    
    /* Update armed status */
    if (ELRS_RX.rf_params->payload_len == OTA8_PACKET_SIZE) {
        ELRS_RX.channels.armed = pkt->full.rc.isArmed;
    } else {
        ELRS_RX.channels.armed = pkt->std.rc.isArmed;
    }
    
    ELRS_RX.channels.last_update_ms = get_millis();
    
    /* NOTE: Do NOT increment nonce here!
     * 
     * CRITICAL FIX: The nonce is incremented in hw_timer_tock(), NOT on packet
     * receipt. This is essential for ELRS protocol synchronization.
     * 
     * Citation: ExpressLRS rx_main.cpp - OtaNonce++ is in HWtimerCallbackTock()
     *   The TX increments nonce at fixed timer intervals, not per-packet.
     *   The RX must do the same to stay synchronized for CRC and FHSS.
     */
}

/*******************************************************************************
 * Connection State Management
 ******************************************************************************/

static void update_connection_state(bool packet_received)
{
    uint32_t now = get_millis();
    uint32_t since_last_packet = now - ELRS_RX.last_packet_ms;
    
    switch (ELRS_RX.conn_state) {
        case ELRS_DISCONNECTED:
            if (packet_received && ELRS_RX.got_sync) {
                ELRS_RX.conn_state = ELRS_TENTATIVE;
                ELRS_RX.connected_ms = now;
                RX_DBG("State: DISCONNECTED -> TENTATIVE\n");
            }
            break;
            
        case ELRS_TENTATIVE:
            if (since_last_packet > CONNECTION_LOST_TIMEOUT_MS) {
                ELRS_RX.conn_state = ELRS_DISCONNECTED;
                ELRS_RX.got_sync = false;
                RX_DBG("State: TENTATIVE -> DISCONNECTED (timeout)\n");
            } else if ((now - ELRS_RX.connected_ms) > CONSIDER_CONN_GOOD_MS) {
                ELRS_RX.conn_state = ELRS_CONNECTED;
                RX_DBG("State: TENTATIVE -> CONNECTED\n");
            }
            break;
            
        case ELRS_CONNECTED:
            if (since_last_packet > CONNECTION_LOST_TIMEOUT_MS) {
                ELRS_RX.conn_state = ELRS_DISCONNECTED;
                ELRS_RX.got_sync = false;
                RX_DBG("State: CONNECTED -> DISCONNECTED (timeout)\n");
            }
            break;
    }
}

/*******************************************************************************
 * Link Quality
 ******************************************************************************/

static void update_link_quality(bool packet_received)
{
    /* Simple LQ calculation based on received packets in window */
    lq_buffer[lq_index] = packet_received ? 1 : 0;
    lq_index = (lq_index + 1) % LQ_CALC_WINDOW;
    
    if (lq_count < LQ_CALC_WINDOW) {
        lq_count++;
    }
    
    /* Calculate percentage */
    uint8_t received = 0;
    for (int i = 0; i < lq_count; i++) {
        received += lq_buffer[i];
    }
    
    ELRS_RX.link_stats.lq = (received * 100) / lq_count;
}

/*******************************************************************************
 * RF Parameters Access
 ******************************************************************************/

const elrs_rf_params_t* elrs_rx_get_rf_params(uint8_t rate_index)
{
    if (rate_index >= RATE_MAX) {
        return NULL;
    }
    return &rf_params_table[rate_index];
}

/*******************************************************************************
 * Packet Reading - Using ELRS Custom GetPacket Command
 ******************************************************************************/

/* ELRS custom firmware command for combined packet retrieval */
#define LR11XX_RADIO_GET_PACKET 0x0700

uint8_t elrs_rx_read_packet(uint8_t *buffer, uint8_t max_len)
{
    uint8_t rx_buf[24];  /* 6 header + max 13 payload (OTA8) + margin */
    lr1121_hal_status_t status;
    
    /***************************************************************************
     * FIX: Use dynamic payload length from rf_params, not hardcoded 8
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp
     *   PayloadLength is set per air-rate:
     *   - OTA4 modes: 8 bytes
     *   - OTA8/full-res modes: 13 bytes
     * 
     * Using hardcoded 8 truncates OTA8 packets before their CRC!
     **************************************************************************/
    uint8_t payload_len = 8;  /* Default for OTA4 */
    if (ELRS_RX.rf_params != NULL) {
        payload_len = ELRS_RX.rf_params->payload_len;
    }
    
    /* Clear buffer to detect actual received bytes */
    memset(rx_buf, 0xAA, sizeof(rx_buf));
    
    /***************************************************************************
     * CRITICAL: Use ELRS Custom GetPacket Command (0x0700)
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp RXnbISR() lines 694-697:
     *   hal.WriteCommand(LR11XX_RADIO_GET_PACKET, radioNumber);
     *   hal.ReadCommand(rx_buf, PayloadLength + 6, radioNumber);
     *   codec->decode(RXdataBuffer, rx_buf + 6, PayloadLength);
     * 
     * The ELRS custom firmware (type 0xF3) adds this command which:
     *   - Returns 6 header/status bytes followed by the actual packet data
     *   - The packet data starts at offset 6, NOT offset 0
     **************************************************************************/
    
    /* Send GetPacket command */
    status = lr1121_hal_write_command(LR11XX_RADIO_GET_PACKET, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("GetPacket cmd failed: %d\n", status);
        return 0;
    }
    
    /* Read response: 6 header bytes + PayloadLength bytes
     * Citation: ELRS reads PayloadLength + 6 bytes, then skips first 6
     */
    uint8_t read_len = payload_len + 6;
    if (read_len > sizeof(rx_buf)) {
        read_len = sizeof(rx_buf);
    }
    
    status = lr1121_hal_read_command(rx_buf, read_len, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        RX_DBG("GetPacket read failed: %d\n", status);
        return 0;
    }
    
    /* DIAGNOSTIC: Print ALL bytes received as simple hex dump */
    RX_DBG("GetPacket[%d]:", read_len);
    for (int i = 0; i < read_len; i++) {
        DEBUGOUT(" %02X", rx_buf[i]);
    }
    DEBUGOUT("\n");
    RX_DBG(" len=%d RSSI=%d dBm SNR=%d dB\n", 
           payload_len, -(int8_t)(rx_buf[5]/2), (int8_t)rx_buf[4]/RADIO_SNR_SCALE);
    
    /***************************************************************************
     * FIX: Copy from offset 6, not 4
     * 
     * Citation: ELRS 4.0 LR1121.cpp RXnbISR():
     *   codec->decode(RXdataBuffer, rx_buf + 6, PayloadLength);
     * 
     * The GetPacket response format is:
     *   [0-5] = 6 status/header bytes
     *   [6..] = actual packet payload
     **************************************************************************/
    if (payload_len > max_len) {
        RX_DBG("Invalid payload len: %d (max=%d)\n", payload_len, max_len);
        return 0;
    }
    memcpy(buffer, &rx_buf[6], payload_len);
    
    return payload_len;
}

void elrs_rx_get_packet_status(int8_t *rssi, int8_t *snr)
{
    uint8_t buf[4];
    lr1121_hal_status_t status;
    
    /* GetPacketStatus for LoRa
     * Citation: LR1121 Datasheet Section 7.2.4
     * Returns: [status][rssi_pkt][snr_pkt][signal_rssi]
     */
    status = lr1121_hal_write_command(LR11XX_RADIO_GET_PKT_STATUS_OC, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        *rssi = 0;
        *snr = 0;
        return;
    }
    
    status = lr1121_hal_read_command(buf, 4, SX12XX_Radio_1);
    if (status != LR1121_HAL_OK) {
        *rssi = 0;
        *snr = 0;
        return;
    }
    
    /* RSSI is -val/2 in dBm
     * SNR is val/4 in dB
     */
    *rssi = -(int8_t)(buf[1] / 2);
    *snr = (int8_t)(buf[2]) / RADIO_SNR_SCALE;
}

/*******************************************************************************
 * Accessor Functions
 ******************************************************************************/

const elrs_channel_data_t* elrs_rx_get_channels(void)
{
    return &ELRS_RX.channels;
}

const elrs_rx_link_stats_t* elrs_rx_get_link_stats(void)
{
    return &ELRS_RX.link_stats;
}

elrs_connection_state_t elrs_rx_get_state(void)
{
    return ELRS_RX.conn_state;
}

bool elrs_rx_is_connected(void)
{
    return (ELRS_RX.conn_state == ELRS_CONNECTED);
}

bool elrs_rx_set_rate(elrs_rate_index_t rate_index)
{
    if (rate_index >= RATE_MAX) {
        return false;
    }
    
    ELRS_RX.current_rate = rate_index;
    ELRS_RX.next_rate = rate_index;  /* ELRS 4.0: Keep next_rate in sync */
    ELRS_RX.switch_mode_pending = false;  /* Clear any pending flag */
    ELRS_RX.rf_params = &rf_params_table[rate_index];
    
    /* Reconfigure radio */
    return elrs_rx_config_radio(ELRS_RX.rf_params, ELRS_RX.current_freq_hz);
}

elrs_rate_index_t elrs_rx_get_rate(void)
{
    return ELRS_RX.current_rate;
}

/*******************************************************************************
 * Timing
 ******************************************************************************/

/**
 * @brief Get millisecond timestamp
 * 
 * Uses CMSIS tick count. Must be called from thread context.
 */
static uint32_t get_millis(void)
{
    return osKernelGetTickCount();
}

/*******************************************************************************
 * Binding Mode Functions
 * 
 * Citation: ExpressLRS 4.0 rx_main.cpp - EnterBindingMode()
 ******************************************************************************/

void elrs_rx_set_binding_uid(const uint8_t *uid)
{
    if (uid != NULL) {
        memcpy(binding_uid, uid, 6);
        binding_mode_active = true;
        RX_DBG("Binding mode enabled with UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
               binding_uid[0], binding_uid[1], binding_uid[2],
               binding_uid[3], binding_uid[4], binding_uid[5]);
    } else {
        binding_mode_active = false;
        memset(binding_uid, 0, 6);
        RX_DBG("Binding mode disabled\n");
    }
}

bool elrs_rx_is_binding_mode(void)
{
    return binding_mode_active;
}
