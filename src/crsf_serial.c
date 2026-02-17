/**
 * @file crsf_serial.c
 * @brief CRSF Serial Output Implementation for SiW917
 *
 * Implements CRSF protocol output to flight controllers.
 * 
 * NOTE: This is a stub implementation that stores data but doesn't actually
 * send over USART. To enable real USART output:
 *   1. Add sl_si91x_usart component to your .slcp project
 *   2. Uncomment USART code below and remove stubs
 *
 * Citation: TBS CRSF Protocol Specification
 * Citation: ExpressLRS src/lib/CrsfProtocol/crsf_protocol.h
 */

#include "crsf_serial.h"
#include "rsi_debug.h"
#include <string.h>

/* Uncomment when USART is configured in project */
/* #include "sl_si91x_usart.h" */
/* #define CRSF_USE_USART 1 */

/*******************************************************************************
 * Module State
 ******************************************************************************/

static bool g_initialized = false;
static uint32_t g_tx_count = 0;

#ifdef CRSF_USE_USART
static sl_usart_handle_t g_usart_handle = NULL;
#endif

/* CRC lookup table (polynomial 0xD5) */
static uint8_t crc8_table[256];
static bool crc_table_initialized = false;

/*******************************************************************************
 * Debug Output
 ******************************************************************************/

#ifndef DEBUGOUT
#define DEBUGOUT printf
#endif

#define CRSF_DBG(fmt, ...) DEBUGOUT("[CRSF] " fmt, ##__VA_ARGS__)

/*******************************************************************************
 * CRC Calculation
 ******************************************************************************/

static void init_crc8_table(void)
{
    if (crc_table_initialized) return;
    
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        }
        crc8_table[i] = crc;
    }
    crc_table_initialized = true;
}

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

/*******************************************************************************
 * USART Callback (required by driver)
 ******************************************************************************/

#ifdef CRSF_USE_USART
static void usart_callback(uint32_t event)
{
    /* Handle TX complete, errors, etc. */
    (void)event;
}
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int crsf_serial_init(uint32_t baud_rate)
{
    if (g_initialized) {
        CRSF_DBG("Already initialized\n");
        return 0;
    }
    
    /* Initialize CRC table */
    init_crc8_table();
    
    /* Use default baud rate if not specified */
    if (baud_rate == 0) {
        baud_rate = CRSF_SERIAL_BAUDRATE_DEFAULT;
    }

#ifdef CRSF_USE_USART
    sl_status_t status;
    
    /* USART configuration for CRSF */
    sl_si91x_usart_control_config_t config = {
        .baudrate = baud_rate,
        .mode = SL_USART_MODE_ASYNCHRONOUS,
        .parity = SL_USART_NO_PARITY,
        .stopbits = SL_USART_STOP_BITS_1,
        .hwflowcontrol = SL_USART_FLOW_CONTROL_NONE,
        .databits = SL_USART_DATA_BITS_8,
        .misc_control = SL_USART_MISC_CONTROL_NONE,
        .usart_module = USART_0,
        .config_enable = ENABLE,
        .synch_mode = DISABLE,
    };
    
    /* Initialize USART0 */
    status = sl_si91x_usart_init(USART_0, &g_usart_handle);
    if (status != SL_STATUS_OK) {
        CRSF_DBG("USART init failed: 0x%lx\n", (unsigned long)status);
        return -1;
    }
    
    /* Register callback */
    status = sl_si91x_usart_register_event_callback(usart_callback);
    if (status != SL_STATUS_OK && status != SL_STATUS_BUSY) {
        CRSF_DBG("USART callback failed: 0x%lx\n", (unsigned long)status);
    }
    
    /* Set control configuration */
    status = sl_si91x_usart_set_configuration(g_usart_handle, &config);
    if (status != SL_STATUS_OK) {
        CRSF_DBG("USART config failed: 0x%lx\n", (unsigned long)status);
        return -2;
    }
    
    CRSF_DBG("Initialized at %lu baud (USART)\n", (unsigned long)baud_rate);
#else
    /* Stub mode - no actual USART output */
    CRSF_DBG("Initialized (STUB MODE - no USART output)\n");
    (void)baud_rate;
#endif
    
    g_initialized = true;
    g_tx_count = 0;
    return 0;
}

void crsf_serial_deinit(void)
{
    if (!g_initialized) return;
    
#ifdef CRSF_USE_USART
    sl_si91x_usart_deinit(g_usart_handle);
    g_usart_handle = NULL;
#endif
    
    g_initialized = false;
    CRSF_DBG("Deinitialized\n");
}

bool crsf_serial_is_ready(void)
{
    return g_initialized;
}

int crsf_serial_send_channels(const uint32_t *channels)
{
    if (!g_initialized || channels == NULL) {
        return -1;
    }
    
    /*
     * CRSF RC Channels Packed Frame Format:
     *   [0]    Sync byte (0xC8)
     *   [1]    Frame length (24 = payload + type + CRC)
     *   [2]    Frame type (0x16)
     *   [3-24] 16 channels packed as 11-bit values (22 bytes)
     *   [25]   CRC8
     */
    uint8_t frame[26];
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 24;  /* Length: type + 22 bytes channels + CRC */
    frame[2] = CRSF_FRAMETYPE_RC;
    
    /* Pack 16 channels (11-bit each) into 22 bytes
     * Citation: CRSF spec - channels packed little-endian bit-wise
     */
    uint8_t *payload = &frame[3];
    memset(payload, 0, 22);
    
    uint32_t bits = 0;
    uint8_t bitsAvail = 0;
    uint8_t byteIdx = 0;
    
    for (int i = 0; i < CRSF_SERIAL_NUM_CHANNELS; i++) {
        /* Clamp channel value */
        uint32_t ch = channels[i];
        if (ch < CRSF_SERIAL_CHANNEL_MIN) ch = CRSF_SERIAL_CHANNEL_MIN;
        if (ch > CRSF_SERIAL_CHANNEL_MAX) ch = CRSF_SERIAL_CHANNEL_MAX;
        
        /* Add 11 bits to accumulator */
        bits |= (ch & 0x7FF) << bitsAvail;
        bitsAvail += 11;
        
        /* Output complete bytes */
        while (bitsAvail >= 8 && byteIdx < 22) {
            payload[byteIdx++] = bits & 0xFF;
            bits >>= 8;
            bitsAvail -= 8;
        }
    }
    
    /* CRC over type + payload (bytes 2-24) */
    frame[25] = crsf_crc8(&frame[2], 23);
    
#ifdef CRSF_USE_USART
    /* Transmit via USART */
    sl_status_t status = sl_si91x_usart_send_data(g_usart_handle, frame, 26);
    if (status != SL_STATUS_OK) {
        return -2;
    }
#else
    /* Stub mode - data prepared but not sent */
    (void)frame;
#endif
    
    g_tx_count++;
    return 0;
}

int crsf_serial_send_link_stats(const crsf_link_stats_t *stats)
{
    if (!g_initialized || stats == NULL) {
        return -1;
    }
    
    /*
     * CRSF Link Statistics Frame Format:
     *   [0]    Sync byte (0xC8)
     *   [1]    Frame length (12 = 10 payload + type + CRC)
     *   [2]    Frame type (0x14)
     *   [3-12] 10 bytes of link stats
     *   [13]   CRC8
     */
    uint8_t frame[14];
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 12;
    frame[2] = CRSF_FRAMETYPE_LINK;
    
    /* Payload - matches crsfLinkStatistics_t exactly */
    frame[3]  = stats->uplink_rssi_1;
    frame[4]  = stats->uplink_rssi_2;
    frame[5]  = stats->uplink_lq;
    frame[6]  = stats->uplink_snr;
    frame[7]  = stats->active_antenna;
    frame[8]  = stats->rf_mode;
    frame[9]  = stats->uplink_tx_power;
    frame[10] = stats->downlink_rssi;
    frame[11] = stats->downlink_lq;
    frame[12] = stats->downlink_snr;
    
    /* CRC over type + payload */
    frame[13] = crsf_crc8(&frame[2], 11);
    
#ifdef CRSF_USE_USART
    /* Transmit via USART */
    sl_status_t status = sl_si91x_usart_send_data(g_usart_handle, frame, 14);
    if (status != SL_STATUS_OK) {
        return -2;
    }
#else
    /* Stub mode - data prepared but not sent */
    (void)frame;
#endif
    
    g_tx_count++;
    return 0;
}

uint32_t crsf_serial_get_tx_count(void)
{
    return g_tx_count;
}
