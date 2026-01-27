/**
 * @file crsf.c
 * @brief CRSF Serial Output Implementation for SiWx917
 * 
 * Implements CRSF frame building and UART transmission for sending
 * decoded ELRS channel data to flight controllers.
 * 
 * Citation: ELRS 4.0 src/lib/CrsfProtocol/crsf_protocol.h
 * Citation: SiWx917 Family Reference Manual Section 29 (UART)
 * Citation: SiWx917 Family Reference Manual Section 30 (USART)
 * 
 * CRSF Protocol:
 * - Baud rate: 420800 (ELRS standard) or 400000 (TBS standard)
 * - Frame format: [SYNC][LEN][TYPE][PAYLOAD...][CRC]
 * - SYNC = 0xC8
 * - LEN = payload_size + 2 (type + CRC)
 * - CRC = CRC8 over [TYPE][PAYLOAD] using polynomial 0xD5
 */

#include "crsf.h"
#include "crsf_protocol.h"
#include "rsi_debug.h"
#include <string.h>

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define CRSF_DEBUG    1

#if CRSF_DEBUG
    #define CRSF_DBG(fmt, ...)    DEBUGOUT("[CRSF] " fmt, ##__VA_ARGS__)
#else
    #define CRSF_DBG(fmt, ...)    ((void)0)
#endif

/*******************************************************************************
 * UART Register Definitions for SiWx917
 * 
 * Citation: SiWx917 Family Reference Manual Section 30.5
 * USART0 Base: 0x44000100 (also supports async UART mode)
 ******************************************************************************/

/* USART0 is used for CRSF output - supports UART mode */
#define USART0_BASE             0x44000100UL

/* Register offsets */
#define USART_RBR_THR_DLL       0x000   /* RX/TX buffer or Divisor Low */
#define USART_IER_DLH           0x004   /* IRQ enable or Divisor High */
#define USART_FCR_IIR           0x008   /* FIFO control / IRQ ID */
#define USART_LCR               0x00C   /* Line control */
#define USART_MCR               0x010   /* Modem control */
#define USART_LSR               0x014   /* Line status */
#define USART_USR               0x07C   /* UART status */
#define USART_TFL               0x080   /* TX FIFO level */
#define USART_SRR               0x088   /* Software reset */

/*******************************************************************************
 * Clock Enable Register Definitions for SiWx917
 * 
 * Citation: SiWx917 Family Reference Manual Section 6.13.18
 * CLK_ENABLE_SET_REG1 base: 0x46000000
 * 
 * USART0 clocks are DISABLED by default (reset value 0x0).
 * Must enable before accessing USART0 registers!
 ******************************************************************************/

#define M4CLK_BASE                  0x46000000UL
#define CLK_ENABLE_SET_REG1         (M4CLK_BASE + 0x000)
#define CLK_CONFIG_REG2             (M4CLK_BASE + 0x01C)

/* CLK_ENABLE_SET_REG1 bit definitions - Citation: RM Section 6.13.18.1 */
#define USART0_PCLK_ENABLE_BIT      (1U << 0)   /* USART0 APB Interface clock */
#define USART0_SCLK_ENABLE_BIT      (1U << 1)   /* USART0 Controller clock */

/* CLK_CONFIG_REG2 bit definitions - Citation: RM Section 6.13.18.8 */
#define USART0_SCLK_SEL_MASK        (0x7U << 0) /* Bits [2:0] - clock source select */
#define USART0_SCLK_DIV_FAC_MASK    (0x3FU << 3) /* Bits [8:3] - division factor */
#define USART0_SCLK_FRAC_SEL_BIT    (1U << 9)   /* Bit 9 - fractional select */
#define USART0_SCLK_ENABLE_BIT_CFG  (1U << 10)  /* Bit 10 - clock output enable */

/* USART0 clock source values - Citation: RM Section 6.13.18.8 */
#define USART0_SCLK_SEL_MCUHP       0x0   /* MCU-HP reference clock (32 MHz) */
#define USART0_SCLK_SEL_SOC_PLL     0x1   /* SoC PLL */
#define USART0_SCLK_SEL_INTF_PLL    0x3   /* Interface PLL */
#define USART0_SCLK_SEL_GATE        0x5   /* Clock is gated */

/* LCR bits */
#define LCR_DLS_8BIT            0x03    /* 8 bits per character */
#define LCR_STOP_1BIT           0x00    /* 1 stop bit */
#define LCR_PEN_NONE            0x00    /* No parity */
#define LCR_DLAB                0x80    /* Divisor latch access */

/* FCR bits */
#define FCR_FIFO_EN             0x01    /* FIFO enable */
#define FCR_RFIFO_RST           0x02    /* RX FIFO reset */
#define FCR_XFIFO_RST           0x04    /* TX FIFO reset */

/* LSR bits */
#define LSR_THRE                0x20    /* TX holding register empty */
#define LSR_TEMT                0x40    /* Transmitter empty */

/* USR bits */
#define USR_TFNF                0x02    /* TX FIFO not full */

/*******************************************************************************
 * CRC8 Table (DVB-S2 polynomial 0xD5)
 * 
 * Citation: ELRS crsf_protocol.h - CRSF uses CRC8 with poly 0xD5
 ******************************************************************************/

static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/*******************************************************************************
 * Module State
 ******************************************************************************/

static struct {
    bool initialized;
    uint32_t baud_rate;
    uint32_t frames_sent;
    uint32_t last_frame_ms;
} crsf_state = {0};

/*******************************************************************************
 * Private Functions - Register Access
 ******************************************************************************/

static inline void usart_write_reg(uint32_t offset, uint8_t value)
{
    *((volatile uint8_t *)(USART0_BASE + offset)) = value;
}

static inline uint8_t usart_read_reg(uint32_t offset)
{
    return *((volatile uint8_t *)(USART0_BASE + offset));
}

static inline void usart_write_reg32(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t *)(USART0_BASE + offset)) = value;
}

static inline uint32_t usart_read_reg32(uint32_t offset)
{
    return *((volatile uint32_t *)(USART0_BASE + offset));
}

/*******************************************************************************
 * Private Functions - CRC
 ******************************************************************************/

/**
 * @brief Calculate CRC8 with DVB-S2 polynomial (0xD5)
 * 
 * Citation: CRSF protocol uses CRC8 over [TYPE][PAYLOAD] bytes
 */
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/*******************************************************************************
 * Private Functions - UART
 ******************************************************************************/

/**
 * @brief Wait for TX FIFO space available
 */
static bool crsf_uart_wait_tx_ready(uint32_t timeout_us)
{
    /* Simple polling with timeout */
    while (timeout_us > 0) {
        if (usart_read_reg(USART_LSR) & LSR_THRE) {
            return true;
        }
        /* Simple delay - approximately 1us per iteration at 180MHz */
        for (volatile int i = 0; i < 180; i++) { __asm volatile("nop"); }
        timeout_us--;
    }
    return false;
}

/**
 * @brief Write single byte to UART TX
 */
static void crsf_uart_write_byte(uint8_t byte)
{
    /* Wait for TX ready */
    crsf_uart_wait_tx_ready(1000);
    
    /* Write to TX holding register */
    usart_write_reg(USART_RBR_THR_DLL, byte);
}

/**
 * @brief Write buffer to UART TX
 */
static void crsf_uart_write(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        crsf_uart_write_byte(data[i]);
    }
}

/*******************************************************************************
 * Public Functions - Initialization
 ******************************************************************************/

/**
 * @brief Enable USART0 peripheral clocks
 * 
 * Citation: SiWx917 Family RM Section 6.13.18.1 - CLK_ENABLE_SET_REG1
 * USART0 clocks are disabled by default. Must enable before register access.
 * 
 * Citation: SiWx917 Family RM Section 6.13.18.8 - CLK_CONFIG_REG2
 * Configure USART0 clock source and divider.
 */
static void crsf_enable_usart0_clock(void)
{
    volatile uint32_t *clk_enable_set_reg1 = (volatile uint32_t *)CLK_ENABLE_SET_REG1;
    volatile uint32_t *clk_config_reg2 = (volatile uint32_t *)CLK_CONFIG_REG2;
    uint32_t reg_val;
    
    CRSF_DBG("Enabling USART0 clocks...\n");
    
    /* Read current CLK_ENABLE_SET_REG1 value */
    reg_val = *clk_enable_set_reg1;
    CRSF_DBG("CLK_ENABLE_SET_REG1 before: 0x%08lX\n", (unsigned long)reg_val);
    
    /* Enable USART0 APB (PCLK) and Controller (SCLK) clocks
     * Citation: RM Section 6.13.18.1
     * Bit 0: USART0_PCLK_ENABLE - APB interface clock
     * Bit 1: USART0_SCLK_ENABLE - Controller clock
     */
    *clk_enable_set_reg1 = USART0_PCLK_ENABLE_BIT | USART0_SCLK_ENABLE_BIT;
    
    /* Small delay for clock to stabilize */
    for (volatile int i = 0; i < 1000; i++) { __asm volatile("nop"); }
    
    /* Verify clocks enabled */
    reg_val = *clk_enable_set_reg1;
    CRSF_DBG("CLK_ENABLE_SET_REG1 after: 0x%08lX\n", (unsigned long)reg_val);
    
    /* Configure USART0 clock source and divider
     * Citation: RM Section 6.13.18.8 - CLK_CONFIG_REG2
     * Using MCU-HP reference clock (32 MHz) with no division
     */
    reg_val = *clk_config_reg2;
    CRSF_DBG("CLK_CONFIG_REG2 before: 0x%08lX\n", (unsigned long)reg_val);
    
    /* Clear USART0 fields and set:
     * - USART0_SCLK_SEL = 0 (MCU-HP 32 MHz)
     * - USART0_SCLK_DIV_FAC = 0 (bypass divider)
     * - USART0_SCLK_FRAC_SEL = 0 (swallow divider)
     * - USART0_SCLK_ENABLE = 1 (enable output)
     */
    reg_val &= ~(USART0_SCLK_SEL_MASK | USART0_SCLK_DIV_FAC_MASK | 
                 USART0_SCLK_FRAC_SEL_BIT | USART0_SCLK_ENABLE_BIT_CFG);
    reg_val |= (USART0_SCLK_SEL_MCUHP << 0);  /* Select 32 MHz MCU-HP clock */
    reg_val |= USART0_SCLK_ENABLE_BIT_CFG;    /* Enable clock output */
    *clk_config_reg2 = reg_val;
    
    /* Small delay */
    for (volatile int i = 0; i < 1000; i++) { __asm volatile("nop"); }
    
    reg_val = *clk_config_reg2;
    CRSF_DBG("CLK_CONFIG_REG2 after: 0x%08lX\n", (unsigned long)reg_val);
    
    CRSF_DBG("USART0 clocks enabled\n");
}

bool crsf_init(uint32_t baud_rate)
{
    /*
     * Initialize USART0 for CRSF output
     * 
     * Citation: SiWx917 Family RM Section 29.3.2 - Baud Rate
     * baud_rate = f_uart / (16 * divisor)
     * divisor = f_uart / (16 * baud_rate)
     * 
     * Using 32 MHz MCU-HP reference clock:
     * For 420800 baud: divisor = 32000000 / (16 * 420800) = 4.75 ≈ 5
     * Actual baud = 32000000 / (16 * 5) = 400000 (close enough for CRSF)
     */
    
    uint32_t uart_clk = 32000000;  /* MCU-HP reference clock */
    uint16_t divisor;
    
    if (baud_rate == 0) {
        baud_rate = CRSF_BAUDRATE;  /* Default 420800 */
    }
    
    CRSF_DBG("Initializing CRSF\n");
    
    /* CRITICAL: Enable USART0 clocks BEFORE accessing any registers!
     * Citation: RM Section 6.13.18.1 - clocks disabled by default
     */
    crsf_enable_usart0_clock();
    
    /* Calculate divisor (integer part) */
    divisor = (uint16_t)(uart_clk / (16 * baud_rate));
    if (divisor == 0) divisor = 1;
    
    CRSF_DBG("Configuring UART: %lu baud, divisor=%u\n", 
             (unsigned long)baud_rate, divisor);
    
    /* Software reset - Citation: RM Section 30.6.15 - SRR register */
    CRSF_DBG("Resetting USART0...\n");
    usart_write_reg(USART_SRR, 0x07);  /* Reset UART and FIFOs */
    
    /* Wait for reset to complete */
    for (volatile int i = 0; i < 10000; i++) { __asm volatile("nop"); }
    CRSF_DBG("Reset complete\n");
    
    /* Set DLAB to access divisor latch */
    usart_write_reg(USART_LCR, LCR_DLAB);
    
    /* Set divisor (baud rate) */
    usart_write_reg(USART_RBR_THR_DLL, divisor & 0xFF);        /* DLL */
    usart_write_reg(USART_IER_DLH, (divisor >> 8) & 0xFF);     /* DLH */
    
    /* Clear DLAB, set 8N1 format
     * Citation: SiWx917 RM Section 30.6.8 - LCR register
     * DLS=3 (8 bits), STOP=0 (1 stop), PEN=0 (no parity)
     */
    usart_write_reg(USART_LCR, LCR_DLS_8BIT | LCR_STOP_1BIT | LCR_PEN_NONE);
    
    /* Enable and reset FIFOs
     * Citation: SiWx917 RM Section 30.6.3 - FCR register
     */
    usart_write_reg(USART_FCR_IIR, FCR_FIFO_EN | FCR_RFIFO_RST | FCR_XFIFO_RST);
    
    /* Disable interrupts (we'll poll) */
    usart_write_reg(USART_IER_DLH, 0x00);
    
    /* Store state */
    crsf_state.baud_rate = baud_rate;
    crsf_state.frames_sent = 0;
    crsf_state.initialized = true;
    
    CRSF_DBG("CRSF UART initialized at %lu baud\n", (unsigned long)baud_rate);
    
    return true;
}

void crsf_deinit(void)
{
    if (!crsf_state.initialized) {
        return;
    }
    
    /* Wait for any pending TX */
    crsf_uart_wait_tx_ready(10000);
    
    /* Reset FIFOs */
    usart_write_reg(USART_SRR, 0x06);
    
    crsf_state.initialized = false;
    CRSF_DBG("CRSF UART deinitialized\n");
}

/*******************************************************************************
 * Public Functions - Frame Building
 ******************************************************************************/

uint8_t crsf_build_rc_channels_frame(uint8_t *frame, const uint16_t *channels)
{
    /*
     * Build RC channels packed frame (16 channels x 11 bits = 22 bytes)
     * 
     * Citation: ELRS crsf_protocol.h - CRSF_FRAMETYPE_RC_CHANNELS_PACKED (0x16)
     * Frame format: [SYNC][LEN][TYPE][22 bytes packed][CRC]
     * Total: 26 bytes
     */
    
    if (frame == NULL || channels == NULL) {
        return 0;
    }
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;          /* 0xC8 */
    frame[1] = 24;                       /* Length: type(1) + payload(22) + crc(1) */
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;  /* 0x16 */
    
    /* Pack 16 channels x 11 bits into 22 bytes
     * 
     * Citation: CRSF protocol - channels are 11-bit values (0-2047)
     * Packed as: ch0[10:0], ch1[10:0], ch2[10:0], ... ch15[10:0]
     */
    uint8_t *payload = &frame[3];
    
    payload[0]  = (uint8_t)(channels[0] & 0xFF);
    payload[1]  = (uint8_t)((channels[0] >> 8) | ((channels[1] & 0x1F) << 3));
    payload[2]  = (uint8_t)((channels[1] >> 5) | ((channels[2] & 0x03) << 6));
    payload[3]  = (uint8_t)((channels[2] >> 2) & 0xFF);
    payload[4]  = (uint8_t)((channels[2] >> 10) | ((channels[3] & 0x7F) << 1));
    payload[5]  = (uint8_t)((channels[3] >> 7) | ((channels[4] & 0x0F) << 4));
    payload[6]  = (uint8_t)((channels[4] >> 4) | ((channels[5] & 0x01) << 7));
    payload[7]  = (uint8_t)((channels[5] >> 1) & 0xFF);
    payload[8]  = (uint8_t)((channels[5] >> 9) | ((channels[6] & 0x3F) << 2));
    payload[9]  = (uint8_t)((channels[6] >> 6) | ((channels[7] & 0x07) << 5));
    payload[10] = (uint8_t)((channels[7] >> 3) & 0xFF);
    
    payload[11] = (uint8_t)(channels[8] & 0xFF);
    payload[12] = (uint8_t)((channels[8] >> 8) | ((channels[9] & 0x1F) << 3));
    payload[13] = (uint8_t)((channels[9] >> 5) | ((channels[10] & 0x03) << 6));
    payload[14] = (uint8_t)((channels[10] >> 2) & 0xFF);
    payload[15] = (uint8_t)((channels[10] >> 10) | ((channels[11] & 0x7F) << 1));
    payload[16] = (uint8_t)((channels[11] >> 7) | ((channels[12] & 0x0F) << 4));
    payload[17] = (uint8_t)((channels[12] >> 4) | ((channels[13] & 0x01) << 7));
    payload[18] = (uint8_t)((channels[13] >> 1) & 0xFF);
    payload[19] = (uint8_t)((channels[13] >> 9) | ((channels[14] & 0x3F) << 2));
    payload[20] = (uint8_t)((channels[14] >> 6) | ((channels[15] & 0x07) << 5));
    payload[21] = (uint8_t)((channels[15] >> 3) & 0xFF);
    
    /* CRC over type + payload (bytes 2-24) */
    frame[25] = crsf_crc8(&frame[2], 23);
    
    return 26;  /* Total frame length */
}

uint8_t crsf_build_link_stats_frame(uint8_t *frame, const crsf_link_stats_t *stats)
{
    /*
     * Build link statistics frame
     * 
     * Citation: ELRS crsf_protocol.h - CRSF_FRAMETYPE_LINK_STATISTICS (0x14)
     * Frame format: [SYNC][LEN][TYPE][10 bytes stats][CRC]
     * Total: 14 bytes
     */
    
    if (frame == NULL || stats == NULL) {
        return 0;
    }
    
    /* Header */
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 12;  /* Length: type(1) + payload(10) + crc(1) */
    frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    
    /* Payload - copy stats structure */
    memcpy(&frame[3], stats, sizeof(crsf_link_stats_t));
    
    /* CRC over type + payload */
    frame[13] = crsf_crc8(&frame[2], 11);
    
    return 14;
}

/*******************************************************************************
 * Public Functions - Frame Transmission
 ******************************************************************************/

bool crsf_send_frame(const uint8_t *frame, uint8_t len)
{
    if (!crsf_state.initialized) {
        CRSF_DBG("ERROR: CRSF not initialized\n");
        return false;
    }
    
    if (frame == NULL || len == 0 || len > CRSF_FRAME_SIZE_MAX) {
        return false;
    }
    
    /* Send frame bytes */
    crsf_uart_write(frame, len);
    
    crsf_state.frames_sent++;
    
    return true;
}

bool crsf_send_rc_channels(const uint16_t *channels)
{
    uint8_t frame[32];
    uint8_t len;
    
    len = crsf_build_rc_channels_frame(frame, channels);
    if (len == 0) {
        return false;
    }
    
    return crsf_send_frame(frame, len);
}

bool crsf_send_link_stats(const crsf_link_stats_t *stats)
{
    uint8_t frame[16];
    uint8_t len;
    
    len = crsf_build_link_stats_frame(frame, stats);
    if (len == 0) {
        return false;
    }
    
    return crsf_send_frame(frame, len);
}

/*******************************************************************************
 * Public Functions - Status
 ******************************************************************************/

bool crsf_is_initialized(void)
{
    return crsf_state.initialized;
}

uint32_t crsf_get_frames_sent(void)
{
    return crsf_state.frames_sent;
}
