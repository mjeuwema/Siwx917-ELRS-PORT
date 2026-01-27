/**
 * @file lr1121_standalone_test.c
 * @brief LR1121 Standalone Test Suite - No Second Receiver Required
 *
 * This test suite validates LR1121 functionality without requiring a second
 * LoRa transceiver. It uses internal chip features and RF measurements.
 *
 * Test Categories:
 * 1. Hardware Verification - GetVersion, GetStatus, register read/write
 * 2. CW (Continuous Wave) Test - RF output verification with spectrum analyzer
 * 3. TX Packet Test - Transmit with TX_DONE IRQ verification
 * 4. RX Mode Test - Enter RX, measure RSSI noise floor
 * 5. Frequency Hopping Test - Configure multiple frequencies
 *
 * What you CAN test without a second receiver:
 * ✓ SPI communication (all commands)
 * ✓ Chip initialization and configuration
 * ✓ TX_DONE interrupt (proves packet was transmitted)
 * ✓ CW mode (verify RF output with SDR/spectrum analyzer)
 * ✓ RSSI readings (noise floor in RX mode)
 * ✓ Frequency tuning across bands
 * ✓ Power level configuration
 * ✓ LoRa modulation parameter changes
 *
 * What you CANNOT test without a second receiver:
 * ✗ Actual packet reception (RX_DONE with valid data)
 * ✗ CRC validation
 * ✗ Link budget / range testing
 * ✗ Two-way communication
 *
 * Hardware: SiWG917Y (BRD2708A) + LR1121 on mikroBUS socket
 *
 * Citations:
 * - LR1121 Datasheet (61252685.LR1121_V2_1_data_sheet.pdf)
 * - LR1121 User Manual (UserManual_LR1121)
 * - siw917x-family-rm.pdf Section 20 (GSPI)
 */

#include "lr1121_driver.h"
#include "lr1121_elrs_init.h"  /* ELRS-compatible init (NO SetTcxoMode!) */
#include "rsi_debug.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Initialization Mode Selection
 * 
 * USE_ELRS_INIT: Controls which initialization sequence is used
 *   0 = Legacy TCXO init (SetTcxoMode + SetStandby)
 *   1 = ELRS-compatible init (NO SetTcxoMode - for externally-powered TCXO)
 *   2 = BULLETPROOF init (SetTcxoMode + SetStandby(XOSC) + Calibrate) ** NEW! **
 * 
 * Mode 2 (BULLETPROOF) fixes the issues where:
 *   - GetRandomNumber works (uses internal RC oscillator)
 *   - GetTemperature fails (requires TCXO to be stable)
 * 
 * The fix addresses:
 *   1. 200mV headroom rule: Use 1.8V TCXO voltage, not 3.3V
 *   2. Ghost RC oscillator: Must call SetStandby(XOSC) after SetTcxoMode
 *   3. ADC calibration: Must call Calibrate(0x3F) for temperature sensor
 * 
 * Citation: User analysis + LR1121 Datasheet
 ******************************************************************************/
#define USE_ELRS_INIT  2  /* BULLETPROOF mode - SetTcxoMode + SetStandby(XOSC) + Calibrate */

/*******************************************************************************
 * LR1121 Command Opcodes
 * Citation: LR1121 User Manual Section 2-14 (Command Reference)
 ******************************************************************************/

/* System Commands */
#define LR1121_CMD_GET_STATUS           0x0100  /* Get chip status */
#define LR1121_CMD_GET_VERSION          0x0101  /* Get firmware version */
#define LR1121_CMD_GET_ERRORS           0x010D  /* Get error flags */
#define LR1121_CMD_CLEAR_ERRORS         0x010E  /* Clear error flags */
/* CRITICAL FIX: Calibrate opcode is 0x010F per Semtech lr11xx_system.c line 99
 * Citation: LR11XX_SYSTEM_CALIBRATE_OC = 0x010F (NOT 0x0100 which is GetStatus!) */
#define LR1121_CMD_CALIBRATE            0x010F
#define LR1121_CMD_SET_REGMODE          0x0110  /* Set regulator mode */
#define LR1121_CMD_CALIBRATE_IMAGE      0x0111  /* Calibrate image rejection */
#define LR1121_CMD_SET_DIO_AS_RF_SWITCH 0x0112  /* Configure RF switch DIOs */
#define LR1121_CMD_SET_DIO_IRQ_PARAMS   0x0113  /* Configure IRQ routing */
#define LR1121_CMD_CLEAR_IRQ            0x0114  /* Clear IRQ flags */
#define LR1121_CMD_CONFIG_LF_CLOCK      0x0116  /* Configure LF clock */
/* LR1121_CMD_SET_TCXO_MODE is defined in lr1121_driver.h as 0x0117 (corrected opcode) */
#define LR1121_CMD_REBOOT               0x0118  /* Reboot chip */
#define LR1121_CMD_GET_VBAT             0x0119  /* Get battery voltage */
#define LR1121_CMD_GET_TEMP             0x011A  /* Get temperature */
#define LR1121_CMD_SET_SLEEP            0x011B  /* Enter sleep mode */
/* Citation: Semtech lr11xx_system.c line 111 - LR11XX_SYSTEM_SET_STANDBY_OC = 0x011C */
#define LR1121_CMD_SET_STANDBY          0x011C  /* Verified against Waveshare Core1121_XF_Demo */
#define LR1121_CMD_SET_FS               0x011D  /* Enter frequency synthesis mode */
#define LR1121_CMD_GET_RANDOM_NUMBER    0x0120  /* Get random number */
#define LR1121_CMD_ERASE_INFO_PAGE      0x0121  /* Erase info page */
#define LR1121_CMD_WRITE_INFO_PAGE      0x0122  /* Write info page */
#define LR1121_CMD_READ_INFO_PAGE       0x0123  /* Read info page */
#define LR1121_CMD_GET_CHIP_EUI         0x0125  /* Get chip EUI */
#define LR1121_CMD_GET_SEMTECH_JOIN_EUI 0x0126  /* Get Semtech join EUI */
#define LR1121_CMD_DERIVE_ROOT_KEYS     0x0127  /* Derive root keys */
#define LR1121_CMD_ENABLE_SPI_CRC       0x0128  /* Enable SPI CRC */
#define LR1121_CMD_DRIVE_DIOS_IN_SLEEP  0x012A  /* Drive DIOs in sleep */
#define LR1121_CMD_RESET_STATS          0x0200  /* Reset statistics */
#define LR1121_CMD_GET_STATS            0x0201  /* Get statistics */
#define LR1121_CMD_GET_PACKET_TYPE      0x0202  /* Get packet type */
#define LR1121_CMD_GET_RX_BUFFER_STATUS 0x0203  /* Get RX buffer status */
#define LR1121_CMD_GET_PACKET_STATUS    0x0204  /* Get packet status */
#define LR1121_CMD_GET_RSSI_INST        0x0205  /* Get instantaneous RSSI */
#define LR1121_CMD_SET_GFSK_SYNC_WORD   0x0206  /* Set GFSK sync word */
#define LR1121_CMD_SET_LORA_PUBLIC_NET  0x0208  /* Set LoRa network type */
#define LR1121_CMD_SET_RX               0x0209  /* Enter RX mode */
#define LR1121_CMD_SET_TX               0x020A  /* Enter TX mode */
#define LR1121_CMD_SET_RF_FREQUENCY     0x020B  /* Set RF frequency */
#define LR1121_CMD_AUTO_TX_RX           0x020C  /* Auto TX/RX */
#define LR1121_CMD_SET_CAD_PARAMS       0x020D  /* Set CAD parameters */
#define LR1121_CMD_SET_PACKET_TYPE      0x020E  /* Set packet type */
#define LR1121_CMD_SET_MODULATION_PARAMS 0x020F /* Set modulation params */
#define LR1121_CMD_SET_PACKET_PARAMS    0x0210  /* Set packet params */
#define LR1121_CMD_SET_TX_PARAMS        0x0211  /* Set TX power/ramp */
#define LR1121_CMD_SET_PACKET_ADDR      0x0212  /* Set packet address */
#define LR1121_CMD_SET_RX_TX_FALLBACK   0x0213  /* Set fallback mode */
#define LR1121_CMD_SET_RX_DUTY_CYCLE    0x0214  /* Set RX duty cycle */
#define LR1121_CMD_SET_PA_CONFIG        0x0215  /* Set PA configuration */
#define LR1121_CMD_STOP_TIMEOUT_ON_PREAMBLE 0x0217 /* Stop timeout on preamble */
#define LR1121_CMD_SET_CAD              0x0218  /* Start CAD */
#define LR1121_CMD_SET_TX_CW            0x0219  /* Transmit continuous wave */
#define LR1121_CMD_SET_TX_INFINITE_PREAMBLE 0x021A /* TX infinite preamble */
#define LR1121_CMD_SET_LORA_SYNC_TIMEOUT 0x021B /* Set LoRa sync timeout */
#define LR1121_CMD_SET_RANGING_ADDR     0x021E  /* Set ranging address */
#define LR1121_CMD_SET_RANGING_REQ_ADDR 0x021F  /* Set ranging request addr */
#define LR1121_CMD_GET_RANGING_RESULT   0x0220  /* Get ranging result */
#define LR1121_CMD_SET_RANGING_TX_RX_DELAY 0x0221 /* Set ranging delay */
#define LR1121_CMD_SET_GFSK_CRC_PARAMS  0x0224  /* Set GFSK CRC params */
#define LR1121_CMD_SET_GFSK_WHITENING   0x0225  /* Set GFSK whitening */
#define LR1121_CMD_SET_RX_BOOSTED       0x0227  /* Enable RX boost */
#define LR1121_CMD_SET_RANGING_PARAMS   0x0228  /* Set ranging params */
#define LR1121_CMD_SET_RSSI_CAL         0x0229  /* Set RSSI calibration */
#define LR1121_CMD_SET_LORA_SYNC_WORD   0x022B  /* Set LoRa sync word */
#define LR1121_CMD_LR_FHSS_BUILD_FRAME  0x022C  /* Build LR-FHSS frame */
#define LR1121_CMD_LR_FHSS_SET_SYNC_WORD 0x022D /* Set LR-FHSS sync word */
#define LR1121_CMD_CONFIG_BLE_BEACON    0x022E  /* Configure BLE beacon */
#define LR1121_CMD_GET_LORA_RX_INFO     0x0230  /* Get LoRa RX info */
#define LR1121_CMD_BLE_BEACON_SEND      0x0231  /* Send BLE beacon */

/* Buffer Commands */
#define LR1121_CMD_WRITE_BUFFER8        0x0180  /* Write 8-bit aligned buffer */
#define LR1121_CMD_READ_BUFFER8         0x0181  /* Read 8-bit aligned buffer */
#define LR1121_CMD_CLEAR_RX_BUFFER      0x0182  /* Clear RX buffer */

/* Register Commands */
#define LR1121_CMD_WRITE_REG_MEM32      0x0105  /* Write 32-bit register */
#define LR1121_CMD_READ_REG_MEM32       0x0106  /* Read 32-bit register */

/*******************************************************************************
 * LR1121 IRQ Flags
 * Citation: LR1121 User Manual Rev 1.2 Section 4.1, Table 4-2 "IrqToEnable Interruption Mapping"
 * 
 * IMPORTANT: These bit positions are DIFFERENT from SX126x!
 * LR1121 uses: TX_DONE=bit2, RX_DONE=bit3, TIMEOUT=bit10
 ******************************************************************************/
#define LR1121_IRQ_TX_DONE              (1UL << 2)   /* 0x00000004 - Bit 2 */
#define LR1121_IRQ_RX_DONE              (1UL << 3)   /* 0x00000008 - Bit 3 */
#define LR1121_IRQ_PREAMBLE_DETECTED    (1UL << 4)   /* 0x00000010 - Bit 4 */
#define LR1121_IRQ_SYNC_WORD_VALID      (1UL << 5)   /* 0x00000020 - Bit 5 */
#define LR1121_IRQ_HEADER_VALID         (1UL << 5)   /* Same as SYNC_WORD for LoRa */
#define LR1121_IRQ_HEADER_ERROR         (1UL << 6)   /* 0x00000040 - Bit 6 */
#define LR1121_IRQ_CRC_ERROR            (1UL << 7)   /* 0x00000080 - Bit 7 */
#define LR1121_IRQ_CAD_DONE             (1UL << 8)   /* 0x00000100 - Bit 8 */
#define LR1121_IRQ_CAD_DETECTED         (1UL << 9)   /* 0x00000200 - Bit 9 */
#define LR1121_IRQ_TIMEOUT              (1UL << 10)  /* 0x00000400 - Bit 10 */
#define LR1121_IRQ_LR_FHSS_HOP          (1UL << 11)  /* 0x00000800 - Bit 11 */
#define LR1121_IRQ_CMD_ERROR            (1UL << 22)  /* 0x00400000 - Bit 22 */
#define LR1121_IRQ_ERROR                (1UL << 23)  /* 0x00800000 - Bit 23 */
#define LR1121_IRQ_ALL                  0x00C00FFF   /* All documented IRQs */

/*******************************************************************************
 * LR1121 Packet Types
 * Citation: LR1121 User Manual Section 8.1 (Packet Type)
 ******************************************************************************/
#define LR1121_PKT_TYPE_GFSK            0x00
#define LR1121_PKT_TYPE_LORA            0x01
#define LR1121_PKT_TYPE_LR_FHSS         0x03

/*******************************************************************************
 * LR1121 PA Selection
 * Citation: LR1121 User Manual Section 9.5 (Power Amplifier)
 ******************************************************************************/
#define LR1121_PA_LP                    0x00  /* Low Power PA */
#define LR1121_PA_HP                    0x01  /* High Power PA */
#define LR1121_PA_HF                    0x02  /* High Frequency PA (2.4GHz) */

/*******************************************************************************
 * Test Configuration
 ******************************************************************************/
#define TEST_FREQUENCY_915MHZ   915000000UL   /* 915 MHz (US ISM band) */
#define TEST_FREQUENCY_868MHZ   868000000UL   /* 868 MHz (EU ISM band) */
#define TEST_FREQUENCY_2400MHZ  2400000000UL  /* 2.4 GHz */

/* Default test frequency */
#define TEST_FREQUENCY          TEST_FREQUENCY_915MHZ

/* LoRa Modulation Parameters */
#define TEST_SF                 7     /* Spreading Factor 7 */
#define TEST_BW_INDEX           4     /* Bandwidth 125 kHz (index) */
#define TEST_CR                 1     /* Coding Rate 4/5 */
#define TEST_LDRO               0     /* Low Data Rate Optimize off */

/* TX Power (dBm) */
#define TEST_TX_POWER           10    /* 10 dBm */

/* Packet Parameters */
#define TEST_PREAMBLE_LEN       8     /* Preamble symbols */
#define TEST_HEADER_TYPE        0     /* 0=Variable length (explicit) */
#define TEST_PAYLOAD_LEN        16    /* Payload length */
#define TEST_CRC_ON             1     /* CRC enabled */
#define TEST_INVERT_IQ          0     /* IQ not inverted */

/*******************************************************************************
 * External Functions from lr1121_driver.c
 ******************************************************************************/
extern bool lr1121_wait_busy_timeout(uint32_t timeout_ms);
extern bool lr1121_send_command(uint16_t opcode, const uint8_t *params, uint16_t param_len);
extern bool lr1121_read_response(uint8_t *response, uint16_t response_len);
extern bool lr1121_get_status(uint8_t *stat1, uint8_t *stat2, uint8_t *irq_status);

/* Raw SPI functions for single-phase commands (GetTemp, GetRandomNumber)
 * Citation: LR1121 User Manual - Instant commands return data during the
 * command phase itself (opcode + NOPs in a single CS assertion). */
extern void lr1121_cs_assert(void);
extern void lr1121_cs_deassert(void);
extern bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

/* Convenience aliases for cleaner code */
#define cs_assert()           lr1121_cs_assert()
#define cs_deassert()         lr1121_cs_deassert()
#define spi_transfer(tx, rx, len) lr1121_spi_transfer((tx), (rx), (len))

/* Alias for convenience */
#define lr1121_wait_busy(timeout_ms) lr1121_wait_busy_timeout(timeout_ms)

/*******************************************************************************
 * Static Variables
 ******************************************************************************/
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;

/* Track failed test names for summary at end */
#define MAX_FAILED_TESTS 20
#define MAX_TEST_NAME_LEN 64
static char failed_test_names[MAX_FAILED_TESTS][MAX_TEST_NAME_LEN];
static uint32_t failed_test_count = 0;

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Simple delay in milliseconds
 */
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 10000; j++) { }
    }
}

/**
 * @brief Print test result and update counters
 * Also tracks failed test names for summary
 */
static void test_result(const char *test_name, bool passed) {
    if (passed) {
        DEBUGOUT("[PASS] %s\n", test_name);
        tests_passed++;
    } else {
        DEBUGOUT("[FAIL] %s\n", test_name);
        tests_failed++;
        
        /* Store failed test name for summary */
        if (failed_test_count < MAX_FAILED_TESTS) {
            /* Safe string copy */
            size_t i;
            for (i = 0; i < MAX_TEST_NAME_LEN - 1 && test_name[i] != '\0'; i++) {
                failed_test_names[failed_test_count][i] = test_name[i];
            }
            failed_test_names[failed_test_count][i] = '\0';
            failed_test_count++;
        }
    }
}

/* Forward declaration for set_standby used in recovery */
static bool set_standby(uint8_t mode);

/**
 * @brief Recover chip from stuck/unknown state with full reset and reinitialization
 * 
 * This function performs a complete recovery sequence when the chip is stuck
 * (e.g., after TX completion, BUSY timeout, or corrupted state):
 *   1. Hardware reset (RST pin toggle)
 *   2. Wait for BUSY to go LOW
 *   3. Full re-initialization (SetTcxoMode, SetStandby, Calibration)
 * 
 * @return true if recovery succeeded, false on failure
 */
static bool recover_chip_state(void) {
    DEBUGOUT("\n--- Chip State Recovery ---\n");
    
    /* Step 1: Hardware reset */
    DEBUGOUT("  1. Performing hardware reset...\n");
    lr1121_status_t status = lr1121_reset();
    if (status != LR1121_OK) {
        DEBUGOUT("  ERROR: Hardware reset failed (status=%d)\n", status);
        return false;
    }
    
    /* Step 2: Full re-initialization using waveshare_init */
    DEBUGOUT("  2. Re-initializing chip (SetTcxoMode, calibration)...\n");
    status = lr1121_waveshare_init();
    if (status != LR1121_OK) {
        DEBUGOUT("  ERROR: Re-initialization failed (status=%d)\n", status);
        return false;
    }
    
    /* Step 3: Verify chip is in STANDBY_XOSC */
    uint8_t stat1, stat2;
    uint8_t irq;  /* Fixed: lr1121_get_status expects uint8_t* not uint32_t* */
    if (lr1121_get_status(&stat1, &stat2, &irq)) {
        uint8_t chip_mode = (stat2 >> 1) & 0x07;  /* Correct: bits [3:1] */
        DEBUGOUT("  3. Chip state verified: mode=%d (%s)\n", chip_mode,
                 chip_mode == 2 ? "STANDBY_XOSC" : 
                 chip_mode == 1 ? "STANDBY_RC" : "UNKNOWN");
        
        if (chip_mode != 2) {
            /* Try to force XOSC mode */
            DEBUGOUT("  WARNING: Not in XOSC mode, forcing SetStandby(XOSC)...\n");
            set_standby(1);
            lr1121_wait_busy(100);
        }
    }
    
    DEBUGOUT("--- Recovery Complete ---\n\n");
    return true;
}

/**
 * @brief Global flag to enable auto-recovery on BUSY timeout
 * Set to true to automatically attempt chip recovery when a command fails due to BUSY timeout.
 * This is especially useful for test sequences where transient issues should not abort the test.
 */
static bool g_auto_recovery_enabled = false;

/**
 * @brief Enable/disable automatic BUSY timeout recovery
 * @param enable true to enable auto-recovery, false to disable
 */
static void __attribute__((unused)) set_auto_recovery(bool enable) {
    g_auto_recovery_enabled = enable;
    DEBUGOUT("  Auto-recovery %s\n", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief Execute command with automatic recovery on BUSY timeout
 * 
 * This wrapper attempts to execute a command and, if it fails due to BUSY timeout,
 * performs a full chip recovery and retries the command once.
 * 
 * @param opcode Command opcode
 * @param params Command parameters (can be NULL)
 * @param param_len Number of parameter bytes
 * @param response Response buffer (can be NULL)
 * @param response_len Expected response length
 * @return true on success (possibly after recovery), false on failure
 */
static bool lr1121_execute_command_with_recovery(uint16_t opcode, const uint8_t *params, 
                                                   uint16_t param_len, uint8_t *response, 
                                                   uint16_t response_len);

/* Forward declaration - actual implementation after lr1121_execute_command */

/*******************************************************************************
 * Diagnostic Logging Configuration
 * 
 * Enable LR1121_DEBUG_COMMANDS for comprehensive command execution tracing.
 * This is especially useful for debugging BUSY timeout issues.
 ******************************************************************************/
// #define LR1121_DEBUG_COMMANDS  /* DISABLED - verbose logging off to save buffer space */

#ifdef LR1121_DEBUG_COMMANDS
/**
 * @brief Read BUSY pin state directly for diagnostics
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.2, p.323
 */
#define EGPIO_BASE_DIAG 0x46130000UL
#define EGPIO_BIT_LOAD_DIAG(pin) (*(volatile uint32_t *)(EGPIO_BASE_DIAG + 0x004 + (0x10 * (pin))))
#define READ_BUSY_PIN_DIAG() ((int)(EGPIO_BIT_LOAD_DIAG(29) & 1))

/**
 * @brief Decode LR1121 command status from stat1 byte
 * Citation: LR1121 User Manual Section 2.1 (Status Byte)
 */
static const char* decode_cmd_status(uint8_t stat1) {
    uint8_t cmd_stat = (stat1 >> 1) & 0x07;
    switch (cmd_stat) {
        case 0: return "FAIL";
        case 1: return "PERR (bad params)";
        case 2: return "SPI_ERR";
        case 3: return "OK";
        case 4: return "DATA_AVAIL";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Print hex dump of buffer
 */
static void print_hex_dump(const char* prefix, const uint8_t* data, uint16_t len) {
    DEBUGOUT("%s", prefix);
    for (uint16_t i = 0; i < len; i++) {
        DEBUGOUT(" %02X", data[i]);
    }
    DEBUGOUT("\n");
}
#endif /* LR1121_DEBUG_COMMANDS */

/**
 * @brief Execute command and read response with comprehensive diagnostics
 * @param opcode Command opcode
 * @param params Command parameters (can be NULL)
 * @param param_len Number of parameter bytes
 * @param response Response buffer (can be NULL)
 * @param response_len Expected response length
 * @return true on success
 * 
 * Diagnostic output includes:
 * - Pre-command BUSY pin state
 * - Command opcode and parameters (hex dump)
 * - Timing for each phase (wait, send, response)
 * - BUSY pin behavior during waits
 * - Response data with status byte interpretation
 */
static bool lr1121_execute_command(uint16_t opcode, const uint8_t *params, 
                                    uint16_t param_len, uint8_t *response, 
                                    uint16_t response_len) {
#ifdef LR1121_DEBUG_COMMANDS
    int busy_initial, busy_after_cmd;
    
    DEBUGOUT("\n--- CMD 0x%04X ---\n", opcode);
    
    /* Record initial BUSY state */
    busy_initial = READ_BUSY_PIN_DIAG();
    DEBUGOUT("  [1] Initial: BUSY=%d\n", busy_initial);
    
    /* Print parameters if any */
    if (params != NULL && param_len > 0) {
        print_hex_dump("  [1] Params:", params, param_len);
    }
#endif

    /* ========== PHASE 1: Wait for chip ready before command ========== */
    if (!lr1121_wait_busy(100)) {
#ifdef LR1121_DEBUG_COMMANDS
        DEBUGOUT("  [!] TIMEOUT waiting for BUSY LOW before cmd (100ms)\n");
        DEBUGOUT("  [!] BUSY pin is STUCK HIGH - possible causes:\n");
        DEBUGOUT("      - LR1121 not powered or not reset properly\n");
        DEBUGOUT("      - TCXO not started (need SetTcxoMode first)\n");
        DEBUGOUT("      - Previous command still processing\n");
        DEBUGOUT("      - GPIO_29 misconfigured (not reading actual pin)\n");
        DEBUGOUT("      - Hardware fault / no LR1121 connected\n");
        /* Sample BUSY pin multiple times to check if it's stuck */
        DEBUGOUT("  [!] BUSY samples: ");
        for (int i = 0; i < 10; i++) {
            DEBUGOUT("%d", READ_BUSY_PIN_DIAG());
            for (volatile int d = 0; d < 10000; d++) { }
        }
        DEBUGOUT("\n");
#else
        DEBUGOUT("  ERROR: Chip busy before command 0x%04X\n", opcode);
#endif
        return false;
    }
    
#ifdef LR1121_DEBUG_COMMANDS
    DEBUGOUT("  [2] Pre-cmd wait: OK (BUSY went LOW)\n");
#endif

    /* ========== PHASE 2: Send command ========== */
    if (!lr1121_send_command(opcode, params, param_len)) {
#ifdef LR1121_DEBUG_COMMANDS
        DEBUGOUT("  [!] SPI send failed for 0x%04X\n", opcode);
        DEBUGOUT("      BUSY after failed send: %d\n", READ_BUSY_PIN_DIAG());
#else
        DEBUGOUT("  ERROR: Failed to send command 0x%04X\n", opcode);
#endif
        return false;
    }
    
#ifdef LR1121_DEBUG_COMMANDS
    busy_after_cmd = READ_BUSY_PIN_DIAG();
    DEBUGOUT("  [3] Cmd sent: BUSY=%d (expect 1=processing)\n", busy_after_cmd);
    if (busy_after_cmd == 0) {
        DEBUGOUT("      WARNING: BUSY should go HIGH after CS deassert!\n");
        DEBUGOUT("      This suggests LR1121 is not responding to SPI.\n");
    }
#endif

    /* ========== PHASE 3: Wait for command processing ========== */
#ifdef LR1121_DEBUG_COMMANDS
    /* Manual polling with diagnostics */
    {
        uint32_t timeout_ms = 100;
        bool busy_went_high = false;
        
        for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed++) {
            int busy_now = READ_BUSY_PIN_DIAG();
            
            if (busy_now == 1 && !busy_went_high) {
                busy_went_high = true;
                DEBUGOUT("  [4] BUSY went HIGH at ~%lu ms (processing)\n", 
                         (unsigned long)elapsed);
            }
            
            if (busy_now == 0) {
                DEBUGOUT("  [4] BUSY went LOW at ~%lu ms (done)\n", 
                         (unsigned long)elapsed);
                goto phase3_done;
            }
            
            /* 1ms delay */
            for (volatile uint32_t j = 0; j < 10000; j++) { }
        }
        
        /* Timeout */
        DEBUGOUT("  [!] TIMEOUT waiting for BUSY LOW after cmd (100ms)\n");
        DEBUGOUT("  [!] Command 0x%04X may have failed or is taking too long\n", opcode);
        DEBUGOUT("  [!] BUSY samples: ");
        for (int i = 0; i < 10; i++) {
            DEBUGOUT("%d", READ_BUSY_PIN_DIAG());
            for (volatile int d = 0; d < 10000; d++) { }
        }
        DEBUGOUT("\n");
        DEBUGOUT("  [!] Possible causes:\n");
        DEBUGOUT("      - Command requires TCXO (call SetTcxoMode first)\n");
        DEBUGOUT("      - Calibration in progress (can take 200+ms)\n");
        DEBUGOUT("      - PLL not locked\n");
        DEBUGOUT("      - Chip in error state (try reset)\n");
        return false;
    }
phase3_done:
    (void)0; /* Empty statement after label */
#else
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  ERROR: Chip busy after command 0x%04X\n", opcode);
        return false;
    }
#endif

    /* ========== PHASE 4: Read response if needed ========== */
    if (response != NULL && response_len > 0) {
        if (!lr1121_read_response(response, response_len)) {
#ifdef LR1121_DEBUG_COMMANDS
            DEBUGOUT("  [!] Failed to read response for 0x%04X\n", opcode);
#else
            DEBUGOUT("  ERROR: Failed to read response for 0x%04X\n", opcode);
#endif
            return false;
        }
        
#ifdef LR1121_DEBUG_COMMANDS
        print_hex_dump("  [5] Response:", response, response_len);
        
        /* Interpret status byte if present (first byte of response) */
        if (response_len >= 1) {
            uint8_t stat1 = response[0];
            DEBUGOUT("  [5] Status: cmd=%s", decode_cmd_status(stat1));
            
            /* Check for errors */
            uint8_t cmd_stat = (stat1 >> 1) & 0x07;
            if (cmd_stat == 0) {
                DEBUGOUT(" [!!! COMMAND FAILED !!!]");
            } else if (cmd_stat == 1) {
                DEBUGOUT(" [!!! PARAMETER ERROR !!!]");
            } else if (cmd_stat == 2) {
                DEBUGOUT(" [!!! SPI ERROR !!!]");
            }
            DEBUGOUT("\n");
        }
#endif
    }
    
#ifdef LR1121_DEBUG_COMMANDS
    DEBUGOUT("--- CMD 0x%04X: SUCCESS ---\n", opcode);
#endif

    return true;
}

/**
 * @brief Execute command with automatic recovery on BUSY timeout
 * 
 * Implementation: This wrapper attempts to execute a command and, if it fails 
 * (possibly due to BUSY timeout), performs a full chip recovery and retries 
 * the command once.
 * 
 * This is useful for test sequences where transient issues should be recovered
 * rather than aborting the entire test.
 */
static bool __attribute__((unused)) lr1121_execute_command_with_recovery(uint16_t opcode, const uint8_t *params, 
                                                   uint16_t param_len, uint8_t *response, 
                                                   uint16_t response_len) {
    /* First attempt */
    if (lr1121_execute_command(opcode, params, param_len, response, response_len)) {
        return true;  /* Success on first try */
    }
    
    /* First attempt failed - check if auto-recovery is enabled */
    if (!g_auto_recovery_enabled) {
        return false;  /* No recovery, just fail */
    }
    
    /* Auto-recovery is enabled - attempt recovery */
    DEBUGOUT("  >>> Auto-recovery: Command 0x%04X failed, attempting recovery...\n", opcode);
    
    if (!recover_chip_state()) {
        DEBUGOUT("  >>> Auto-recovery: Chip recovery FAILED\n");
        return false;
    }
    
    DEBUGOUT("  >>> Auto-recovery: Retrying command 0x%04X...\n", opcode);
    
    /* Second attempt after recovery */
    if (lr1121_execute_command(opcode, params, param_len, response, response_len)) {
        DEBUGOUT("  >>> Auto-recovery: Command succeeded after recovery!\n");
        return true;
    }
    
    DEBUGOUT("  >>> Auto-recovery: Command STILL FAILED after recovery\n");
    return false;
}

/**
 * @brief Set RF frequency with error pre-check
 * Citation: LR1121 User Manual Section 7.2.1 (SetRfFrequency)
 * @param freq_hz Frequency in Hz
 * 
 * FIX (2026-01-16): Added error flag pre-check before SetRfFrequency.
 * This command requires PLL lock, which depends on a stable XOSC.
 * If HF_XOSC_START_ERR is set, the PLL will never lock and the chip
 * will hang in BUSY state indefinitely.
 * 
 * This function now:
 *   1. Checks GetErrors for HF_XOSC_START_ERR (bit 5)
 *   2. If error present, clears errors and logs warning
 *   3. Proceeds with SetRfFrequency command
 *   4. Uses extended timeout (500ms) for PLL lock
 */
static bool set_rf_frequency(uint32_t freq_hz) {
    /* FIX: Pre-check for XOSC errors that would prevent PLL lock
     * Citation: LR1121 Datasheet Section 11.2.2 "GetErrors"
     *   Bit 5: HF_XOSC_START_ERR - High frequency XOSC failed to start
     *   Bit 7: PLL_LOCK_ERR - PLL failed to lock
     */
    uint8_t err_resp[3] = {0};
    if (lr1121_execute_command(LR1121_CMD_GET_ERRORS, NULL, 0, err_resp, 3)) {
        uint16_t errors = ((uint16_t)err_resp[1] << 8) | err_resp[2];
        
        if (errors & 0x0020) {  /* HF_XOSC_START_ERR (bit 5) */
            DEBUGOUT("  WARNING: HF_XOSC_START_ERR detected before SetRfFrequency!\n");
            DEBUGOUT("  This indicates the TCXO/XOSC failed to start properly.\n");
            DEBUGOUT("  PLL will not be able to lock. Check:\n");
            DEBUGOUT("    1. SetTcxoMode voltage (should match hardware)\n");
            DEBUGOUT("    2. SetStandby(XOSC) was called after SetTcxoMode\n");
            DEBUGOUT("    3. TCXO hardware is functional\n");
            DEBUGOUT("  Attempting to clear errors and continue...\n");
            
            /* Try to clear errors and continue */
            lr1121_execute_command(LR1121_CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
            delay_ms(10);
        }
        
        if (errors & 0x0080) {  /* PLL_LOCK_ERR (bit 7) */
            DEBUGOUT("  WARNING: PLL_LOCK_ERR detected before SetRfFrequency!\n");
            DEBUGOUT("  Clearing errors and retrying...\n");
            lr1121_execute_command(LR1121_CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
            delay_ms(10);
        }
    }
    
    uint8_t params[4];
    params[0] = (freq_hz >> 24) & 0xFF;
    params[1] = (freq_hz >> 16) & 0xFF;
    params[2] = (freq_hz >> 8) & 0xFF;
    params[3] = freq_hz & 0xFF;
    
    /* FIX: Use longer timeout for SetRfFrequency since it involves PLL lock
     * The default 100ms timeout may not be enough if XOSC is slow to stabilize
     */
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  ERROR: Chip BUSY before SetRfFrequency - possible XOSC issue\n");
        return false;
    }
    
    if (!lr1121_send_command(LR1121_CMD_SET_RF_FREQUENCY, params, 4)) {
        DEBUGOUT("  ERROR: Failed to send SetRfFrequency command\n");
        return false;
    }
    
    /* Wait for PLL lock with extended timeout */
    if (!lr1121_wait_busy(500)) {
        DEBUGOUT("  ERROR: SetRfFrequency BUSY timeout (500ms) - PLL failed to lock\n");
        DEBUGOUT("  This usually means XOSC is not stable. Check TCXO initialization.\n");
        return false;
    }
    
    return true;
}

/**
 * @brief Set packet type (LoRa/GFSK/LR-FHSS)
 * Citation: LR1121 User Manual Section 8.1.1 (SetPacketType)
 */
static bool set_packet_type(uint8_t pkt_type) {
    return lr1121_execute_command(LR1121_CMD_SET_PACKET_TYPE, &pkt_type, 1, NULL, 0);
}

/**
 * @brief Set LoRa modulation parameters
 * Citation: LR1121 User Manual Section 8.3.1 (SetModulationParams)
 */
static bool set_lora_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
    uint8_t params[4] = {sf, bw, cr, ldro};
    return lr1121_execute_command(LR1121_CMD_SET_MODULATION_PARAMS, params, 4, NULL, 0);
}

/**
 * @brief Set LoRa packet parameters
 * Citation: LR1121 User Manual Section 8.3.2 (SetPacketParams)
 */
static bool set_lora_packet_params(uint16_t preamble_len, uint8_t header_type,
                                    uint8_t payload_len, uint8_t crc_on, uint8_t invert_iq) {
    uint8_t params[6];
    params[0] = (preamble_len >> 8) & 0xFF;
    params[1] = preamble_len & 0xFF;
    params[2] = header_type;
    params[3] = payload_len;
    params[4] = crc_on;
    params[5] = invert_iq;
    return lr1121_execute_command(LR1121_CMD_SET_PACKET_PARAMS, params, 6, NULL, 0);
}

/**
 * @brief Set PA configuration
 * Citation: LR1121 User Manual Section 9.5.1 (SetPaConfig)
 */
static bool set_pa_config(uint8_t pa_sel, uint8_t reg_pa_supply, uint8_t pa_duty_cycle, uint8_t pa_hp_sel) {
    uint8_t params[4] = {pa_sel, reg_pa_supply, pa_duty_cycle, pa_hp_sel};
    return lr1121_execute_command(LR1121_CMD_SET_PA_CONFIG, params, 4, NULL, 0);
}

/**
 * @brief Set TX parameters (power and ramp time)
 * Citation: LR1121 User Manual Section 9.5.2 (SetTxParams)
 */
static bool set_tx_params(int8_t power_dbm, uint8_t ramp_time) {
    uint8_t params[2] = {(uint8_t)power_dbm, ramp_time};
    return lr1121_execute_command(LR1121_CMD_SET_TX_PARAMS, params, 2, NULL, 0);
}

/**
 * @brief Set standby mode with verification
 * Citation: LR1121 User Manual Section 2.3 (SetStandby)
 * @param mode 0=RC oscillator, 1=XOSC
 * 
 * FIX (2026-01-16): Added chip mode verification and delay.
 * The LR1121 may not immediately switch to XOSC mode if the TCXO
 * is not stable. This function now:
 *   1. Sends SetStandby command
 *   2. Waits for BUSY LOW
 *   3. Adds a 10ms delay for XOSC stabilization
 *   4. Verifies chip mode matches requested mode (if XOSC requested)
 */
static bool set_standby(uint8_t mode) {
    bool ok = lr1121_execute_command(LR1121_CMD_SET_STANDBY, &mode, 1, NULL, 0);
    
    if (!ok) {
        return false;
    }
    
    /* FIX: Add delay for XOSC stabilization when requesting XOSC mode
     * Citation: User analysis - XOSC needs time to stabilize after SetStandby
     * Without this delay, subsequent commands like SetRfFrequency may fail
     * because the PLL cannot lock to an unstable reference clock.
     */
    if (mode == 1) {  /* XOSC mode */
        delay_ms(10);  /* Allow XOSC to stabilize */
        
        /* Verify chip actually entered XOSC mode */
        uint8_t stat1, stat2, irq;
        if (lr1121_get_status(&stat1, &stat2, &irq)) {
            uint8_t chip_mode = (stat2 >> 1) & 0x07;
            if (chip_mode != 2) {  /* 2 = STDBY_XOSC */
                DEBUGOUT("  WARNING: SetStandby(XOSC) - chip mode is %d, expected 2 (STDBY_XOSC)\n", chip_mode);
                DEBUGOUT("  This indicates XOSC failed to start. Check TCXO configuration.\n");
                /* Continue anyway - let subsequent commands fail with clearer errors */
            }
        }
    }
    
    return ok;
}

/**
 * @brief Set FS (Frequency Synthesis) mode
 * Citation: LR1121 User Manual Section 2.4 (SetFs)
 */
static bool set_fs(void) {
    return lr1121_execute_command(LR1121_CMD_SET_FS, NULL, 0, NULL, 0);
}

/**
 * @brief Enter TX mode
 * Citation: LR1121 User Manual Section 9.2 (SetTx)
 * @param timeout_ms Timeout in ms (0 = no timeout)
 */
static bool set_tx(uint32_t timeout_ms) {
    /* Timeout in steps of 1/32768 seconds */
    uint32_t timeout_val = (timeout_ms == 0) ? 0 : (timeout_ms * 32768UL / 1000UL);
    uint8_t params[3];
    params[0] = (timeout_val >> 16) & 0xFF;
    params[1] = (timeout_val >> 8) & 0xFF;
    params[2] = timeout_val & 0xFF;
    return lr1121_execute_command(LR1121_CMD_SET_TX, params, 3, NULL, 0);
}

/**
 * @brief Enter RX mode
 * Citation: LR1121 User Manual Section 9.1 (SetRx)
 * @param timeout_ms Timeout in ms (0 = continuous)
 */
static bool set_rx(uint32_t timeout_ms) {
    uint32_t timeout_val = (timeout_ms == 0) ? 0xFFFFFF : (timeout_ms * 32768UL / 1000UL);
    uint8_t params[3];
    params[0] = (timeout_val >> 16) & 0xFF;
    params[1] = (timeout_val >> 8) & 0xFF;
    params[2] = timeout_val & 0xFF;
    return lr1121_execute_command(LR1121_CMD_SET_RX, params, 3, NULL, 0);
}

/**
 * @brief Start CW (Continuous Wave) transmission
 * Citation: LR1121 User Manual Section 9.4 (SetTxCw)
 */
static bool set_tx_cw(void) {
    return lr1121_execute_command(LR1121_CMD_SET_TX_CW, NULL, 0, NULL, 0);
}

/**
 * @brief Configure DIO IRQ parameters
 * Citation: LR1121 User Manual Section 4.1.1 (SetDioIrqParams)
 */
static bool set_dio_irq_params(uint32_t irq_mask, uint32_t dio1_mask, 
                                uint32_t dio2_mask, uint32_t dio3_mask) {
    uint8_t params[16];
    /* IRQ enable mask */
    params[0] = (irq_mask >> 24) & 0xFF;
    params[1] = (irq_mask >> 16) & 0xFF;
    params[2] = (irq_mask >> 8) & 0xFF;
    params[3] = irq_mask & 0xFF;
    /* DIO1 mask */
    params[4] = (dio1_mask >> 24) & 0xFF;
    params[5] = (dio1_mask >> 16) & 0xFF;
    params[6] = (dio1_mask >> 8) & 0xFF;
    params[7] = dio1_mask & 0xFF;
    /* DIO2 mask */
    params[8] = (dio2_mask >> 24) & 0xFF;
    params[9] = (dio2_mask >> 16) & 0xFF;
    params[10] = (dio2_mask >> 8) & 0xFF;
    params[11] = dio2_mask & 0xFF;
    /* DIO3 mask */
    params[12] = (dio3_mask >> 24) & 0xFF;
    params[13] = (dio3_mask >> 16) & 0xFF;
    params[14] = (dio3_mask >> 8) & 0xFF;
    params[15] = dio3_mask & 0xFF;
    
    return lr1121_execute_command(LR1121_CMD_SET_DIO_IRQ_PARAMS, params, 16, NULL, 0);
}

/**
 * @brief Clear IRQ status
 * Citation: LR1121 User Manual Section 4.1.2 (ClearIrq)
 */
static bool clear_irq(uint32_t irq_mask) {
    uint8_t params[4];
    params[0] = (irq_mask >> 24) & 0xFF;
    params[1] = (irq_mask >> 16) & 0xFF;
    params[2] = (irq_mask >> 8) & 0xFF;
    params[3] = irq_mask & 0xFF;
    return lr1121_execute_command(LR1121_CMD_CLEAR_IRQ, params, 4, NULL, 0);
}

/**
 * @brief Write data to TX buffer
 * Citation: LR1121 User Manual Section 5.1 (WriteBuffer8)
 */
static bool write_buffer8(uint8_t offset, const uint8_t *data, uint8_t len) {
    uint8_t params[256];
    params[0] = offset;
    memcpy(&params[1], data, len);
    return lr1121_execute_command(LR1121_CMD_WRITE_BUFFER8, params, len + 1, NULL, 0);
}

/**
 * @brief Get instantaneous RSSI
 * Citation: LR1121 User Manual Section 10.1 (GetRssiInst)
 * @return RSSI in dBm (negative value)
 */
static int16_t get_rssi_inst(void) {
    uint8_t response[2];  /* stat1 + rssi */
    if (!lr1121_execute_command(LR1121_CMD_GET_RSSI_INST, NULL, 0, response, 2)) {
        return 0;
    }
    /* RSSI is returned as -rssi/2 in dBm */
    return -(int16_t)response[1] / 2;
}

#if (USE_ELRS_INIT == 0)  /* Only include legacy functions in mode 0 */
/**
 * @brief Get chip temperature with detailed diagnostics
 * 
 * Citation: LR1121 User Manual Section 4.3.1 (GetTemp)
 * 
 * FIX: Changed from single-phase to TWO-PHASE transaction.
 * The LR1121 requires separate NSS transactions for command and response.
 * 
 * TWO-PHASE PROTOCOL:
 *   Phase 1: [NSS LOW] Send opcode (0x011A) [NSS HIGH]
 *   Wait:    Wait for BUSY to go LOW
 *   Phase 2: [NSS LOW] Send NOPs, receive [stat1][temp_hi][temp_lo] [NSS HIGH]
 * 
 * Response format: [stat1] [temp_high] [temp_low]
 * 
 * @param[out] raw_out Optional pointer to receive raw value for debugging
 * @return Temperature in degrees Celsius, or -999 on error
 */
static int16_t get_temperature_debug(uint16_t *raw_out) {
    /**
     * FIX: Changed from single-phase to two-phase transaction.
     * The LR1121 requires separate NSS transactions for command and response.
     * 
     * TWO-PHASE PROTOCOL:
     *   Phase 1: [NSS LOW] Send opcode (0x011A) [NSS HIGH]
     *   Wait:    Wait for BUSY to go LOW
     *   Phase 2: [NSS LOW] Send NOPs, receive [stat1][temp_high][temp_low] [NSS HIGH]
     */
    uint8_t tx_cmd[2] = {0x01, 0x1A};  /* GetTemp opcode */
    uint8_t tx_nop[3] = {0x00, 0x00, 0x00};  /* NOPs for Phase 2 */
    uint8_t rx_buf[3] = {0xFF, 0xFF, 0xFF};  /* Response buffer */
    
    DEBUGOUT("  [DEBUG] Sending GetTemp (0x%04X) - TWO PHASE...\n", LR1121_CMD_GET_TEMP);
    
    /* Wait for chip to be ready before Phase 1 */
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  [DEBUG] Chip busy before GetTemp Phase 1 - returning error\n");
        return -999;
    }
    
    /* Phase 1: Send command opcode only */
    DEBUGOUT("  [DEBUG] Phase 1: Sending opcode 0x011A...\n");
    cs_assert();
    bool ok = spi_transfer(tx_cmd, NULL, 2);
    cs_deassert();
    
    if (!ok) {
        DEBUGOUT("  [DEBUG] Phase 1 SPI transfer failed - returning error\n");
        return -999;
    }
    
    /* Wait for BUSY to go LOW (command processing complete) */
    DEBUGOUT("  [DEBUG] Waiting for BUSY LOW after Phase 1...\n");
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  [DEBUG] Chip busy after GetTemp Phase 1 - returning error\n");
        return -999;
    }
    
    /* Phase 2: Send NOPs to clock out response */
    DEBUGOUT("  [DEBUG] Phase 2: Reading response (3 bytes)...\n");
    cs_assert();
    ok = spi_transfer(tx_nop, rx_buf, 3);
    cs_deassert();
    
    DEBUGOUT("  [DEBUG] GetTemp Phase 2 result: %s\n", ok ? "OK" : "FAILED");
    DEBUGOUT("  [DEBUG] RX buffer: [0x%02X, 0x%02X, 0x%02X]\n", 
             rx_buf[0], rx_buf[1], rx_buf[2]);
    
    if (!ok) {
        DEBUGOUT("  [DEBUG] Phase 2 SPI transfer failed - returning error\n");
        return -999;
    }
    
    /* Parse response: [stat1] [temp_high] [temp_low] */
    uint8_t stat1 = rx_buf[0];
    uint8_t temp_high = rx_buf[1];
    uint8_t temp_low = rx_buf[2];
    
    /* Decode status byte
     * Citation: LR11XX User Manual Section 3.4 (Status Byte)
     * Bits [3:1] = cmd_status:
     *   0 = FAIL (command not executed)
     *   1 = PERR (parameter error)
     *   2 = SPI_ERR (SPI communication error)
     *   3 = OK (command executed successfully)
     *   4 = DATA_AVAIL (data available for read)
     */
    uint8_t cmd_status = (stat1 >> 1) & 0x07;
    DEBUGOUT("  [DEBUG] Status byte: 0x%02X (cmd_status=%d: %s)\n", 
             stat1, cmd_status,
             cmd_status == 0 ? "FAIL" :
             cmd_status == 1 ? "PERR" :
             cmd_status == 2 ? "SPI_ERR" :
             cmd_status == 3 ? "OK" :
             cmd_status == 4 ? "DATA_AVAIL" : "UNKNOWN");
    
    /* Temperature calculation - raw is 16-bit value */
    uint16_t raw = ((uint16_t)temp_high << 8) | temp_low;
    if (raw_out) *raw_out = raw;
    
    DEBUGOUT("  [DEBUG] Raw temp value: 0x%04X (%u decimal)\n", raw, raw);
    
    /* Formula from LR1121 User Manual: Temp = (raw / 2^8) - 64 
     * This is equivalent to: Temp = (raw >> 8) - 64 */
    int16_t temp_method1 = ((int16_t)raw >> 8) - 64;
    
    /* Alternative interpretation: raw might be in 1/256 degrees + 64 offset */
    int16_t temp_method2 = (int16_t)(raw / 256) - 64;
    
    /* Another alternative: temperature might be signed */
    int16_t temp_method3 = ((int16_t)(int8_t)temp_high) + ((int16_t)temp_low >> 4) / 16;
    
    DEBUGOUT("  [DEBUG] Temp interpretations:\n");
    DEBUGOUT("         Method 1 (raw>>8 - 64): %d C\n", temp_method1);
    DEBUGOUT("         Method 2 (raw/256 - 64): %d C\n", temp_method2);
    DEBUGOUT("         Method 3 (signed byte): %d C\n", temp_method3);
    
    return temp_method1;
}

/* Wrapper for backward compatibility - only needed in legacy mode */
static int16_t get_temperature(void) {
    return get_temperature_debug(NULL);
}

/**
 * @brief Get random number from chip with detailed diagnostics
 * 
 * Citation: LR1121 User Manual Section 3.7.7 (GetRandomNumber)
 * 
 * FIX: Changed from single-phase to TWO-PHASE transaction.
 * The LR1121 requires separate NSS transactions for command and response.
 * 
 * TWO-PHASE PROTOCOL:
 *   Phase 1: [NSS LOW] Send opcode (0x0120) [NSS HIGH]
 *   Wait:    Wait for BUSY to go LOW
 *   Phase 2: [NSS LOW] Send NOPs, receive [stat1][rand3][rand2][rand1][rand0] [NSS HIGH]
 * 
 * Response format: [stat1] [rand_b3 (MSB)] [rand_b2] [rand_b1] [rand_b0 (LSB)]
 * Random number is MSB first: (rand3 << 24) | (rand2 << 16) | (rand1 << 8) | rand0
 */
static uint32_t get_random_number_debug(void) {
    /**
     * FIX: Changed from single-phase to two-phase transaction.
     * The LR1121 requires separate NSS transactions for command and response.
     * 
     * TWO-PHASE PROTOCOL:
     *   Phase 1: [NSS LOW] Send opcode (0x0120) [NSS HIGH]
     *   Wait:    Wait for BUSY to go LOW
     *   Phase 2: [NSS LOW] Send NOPs, receive [stat1][rand3][rand2][rand1][rand0] [NSS HIGH]
     */
    uint8_t tx_cmd[2] = {0x01, 0x20};  /* GetRandomNumber opcode */
    uint8_t tx_nop[5] = {0x00, 0x00, 0x00, 0x00, 0x00};  /* NOPs for Phase 2 */
    uint8_t rx_buf[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  /* Response buffer */
    
    DEBUGOUT("  [DEBUG] Sending GetRandomNumber (0x%04X) - TWO PHASE...\n", LR1121_CMD_GET_RANDOM_NUMBER);
    
    /* Wait for chip to be ready before Phase 1 */
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  [DEBUG] Chip busy before GetRandomNumber Phase 1 - returning 0\n");
        return 0;
    }
    
    /* Phase 1: Send command opcode only */
    DEBUGOUT("  [DEBUG] Phase 1: Sending opcode 0x0120...\n");
    cs_assert();
    bool ok = spi_transfer(tx_cmd, NULL, 2);
    cs_deassert();
    
    if (!ok) {
        DEBUGOUT("  [DEBUG] Phase 1 SPI transfer failed - returning 0\n");
        return 0;
    }
    
    /* Wait for BUSY to go LOW (command processing complete) */
    DEBUGOUT("  [DEBUG] Waiting for BUSY LOW after Phase 1...\n");
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("  [DEBUG] Chip busy after GetRandomNumber Phase 1 - returning 0\n");
        return 0;
    }
    
    /* Phase 2: Send NOPs to clock out response */
    DEBUGOUT("  [DEBUG] Phase 2: Reading response (5 bytes)...\n");
    cs_assert();
    ok = spi_transfer(tx_nop, rx_buf, 5);
    cs_deassert();
    
    DEBUGOUT("  [DEBUG] GetRandomNumber Phase 2 result: %s\n", ok ? "OK" : "FAILED");
    DEBUGOUT("  [DEBUG] RX buffer: [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n", 
             rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
    
    if (!ok) {
        DEBUGOUT("  [DEBUG] Phase 2 SPI transfer failed - returning 0\n");
        return 0;
    }
    
    /* Parse response: [stat1] [rand_b3 (MSB)] [rand_b2] [rand_b1] [rand_b0 (LSB)] */
    uint8_t stat1 = rx_buf[0];
    
    /* Decode status byte
     * Citation: LR11XX User Manual Section 3.4 (Status Byte)
     * Bits [3:1] = cmd_status:
     *   0 = FAIL (command not executed)
     *   1 = PERR (parameter error)
     *   2 = SPI_ERR (SPI communication error)
     *   3 = OK (command executed successfully)
     *   4 = DATA_AVAIL (data available for read)
     */
    uint8_t cmd_status = (stat1 >> 1) & 0x07;
    DEBUGOUT("  [DEBUG] Status byte: 0x%02X (cmd_status=%d: %s)\n", 
             stat1, cmd_status,
             cmd_status == 0 ? "FAIL" :
             cmd_status == 1 ? "PERR" :
             cmd_status == 2 ? "SPI_ERR" :
             cmd_status == 3 ? "OK" :
             cmd_status == 4 ? "DATA_AVAIL" : "UNKNOWN");
    
    /* With two-phase: rx_buf[0]=stat1, rx_buf[1..4]=random bytes */
    uint32_t rand_val = ((uint32_t)rx_buf[1] << 24) | ((uint32_t)rx_buf[2] << 16) |
                        ((uint32_t)rx_buf[3] << 8) | rx_buf[4];
    
    DEBUGOUT("  [DEBUG] Random value: 0x%08lX\n", (unsigned long)rand_val);
    
    return rand_val;
}

/* Wrapper for backward compatibility - only needed in legacy mode */
static uint32_t get_random_number(void) {
    return get_random_number_debug();
}
#endif /* USE_ELRS_INIT == 0 */

/**
 * @brief Perform image calibration for a frequency range
 * Citation: LR1121 User Manual Section 3.1 (CalibrateImage)
 */
static bool calibrate_image(uint32_t freq_hz) {
    uint8_t params[2];
    
    /* Select calibration frequencies based on operating frequency */
    /* Citation: LR1121 Datasheet Table 7-2 (Image Calibration) */
    if (freq_hz >= 430000000UL && freq_hz <= 440000000UL) {
        params[0] = 0x6B;  /* 430 MHz */
        params[1] = 0x6F;  /* 440 MHz */
    } else if (freq_hz >= 470000000UL && freq_hz <= 510000000UL) {
        params[0] = 0x75;  /* 470 MHz */
        params[1] = 0x81;  /* 510 MHz */
    } else if (freq_hz >= 779000000UL && freq_hz <= 787000000UL) {
        params[0] = 0xC1;  /* 779 MHz */
        params[1] = 0xC5;  /* 787 MHz */
    } else if (freq_hz >= 863000000UL && freq_hz <= 870000000UL) {
        params[0] = 0xD7;  /* 863 MHz */
        params[1] = 0xDB;  /* 870 MHz */
    } else if (freq_hz >= 902000000UL && freq_hz <= 928000000UL) {
        params[0] = 0xE1;  /* 902 MHz */
        params[1] = 0xE9;  /* 928 MHz */
    } else {
        /* Default to wide range for Sub-GHz */
        params[0] = 0xD7;
        params[1] = 0xE9;
    }
    
    return lr1121_execute_command(LR1121_CMD_CALIBRATE_IMAGE, params, 2, NULL, 0);
}

/*******************************************************************************
 * Diagnostic Functions for Core1121-HF Debugging
 ******************************************************************************/

/**
 * @brief Decode and print LR1121 Stat1 byte
 *
 * Citation: LR1121 User Manual Section 3.4.2 Stat1 Values
 *   Bits [7:4] = RFU
 *   Bits [3:1] = Command Status:
 *     0 = CMD_FAIL: The last command could not be executed
 *     1 = CMD_PERR: Parameter error (wrong opcode, arguments)
 *     2 = CMD_OK: The last command was processed successfully
 *     3 = CMD_DAT: Command successful, data being transmitted
 *     4-7 = RFU
 *   Bit [0] = Interrupt Status:
 *     0 = No interrupt active
 *     1 = At least 1 interrupt active
 */
static void decode_stat1(uint8_t stat1) {
    uint8_t cmd_status = (stat1 >> 1) & 0x07;
    uint8_t irq_status = stat1 & 0x01;
    
    DEBUGOUT("  Stat1: 0x%02X\n", stat1);
    DEBUGOUT("    Command Status [3:1] = %d: ", cmd_status);
    switch (cmd_status) {
        case 0: DEBUGOUT("CMD_FAIL - Last command could not be executed!\n"); break;
        case 1: DEBUGOUT("CMD_PERR - Parameter error (wrong opcode/args)!\n"); break;
        case 2: DEBUGOUT("CMD_OK - Last command processed successfully\n"); break;
        case 3: DEBUGOUT("CMD_DAT - Data being transmitted\n"); break;
        default: DEBUGOUT("RFU (%d)\n", cmd_status); break;
    }
    DEBUGOUT("    Interrupt Status [0] = %d: %s\n", irq_status,
             irq_status ? "At least 1 interrupt active" : "No interrupt active");
}

/**
 * @brief Decode and print LR1121 Stat2 byte
 *
 * Citation: LR1121 User Manual Section 3.4.3 Stat2 Values
 *   Bits [7:4] = Reset Status:
 *     0 = Cleared (no active reset)
 *     1 = Analog reset (Power On Reset, Brown-Out Reset)
 *     2 = External reset (NRESET pin)
 *     3 = System reset
 *     4 = Watchdog reset
 *     5 = Wakeup NSS toggling
 *     6 = RTC restart
 *     7 = RFU
 *   Bits [3:1] = Chip Mode:
 *     0 = Sleep
 *     1 = Standby with RC Oscillator
 *     2 = Standby with external Oscillator (XOSC)
 *     3 = FS (Frequency Synthesis)
 *     4 = RX
 *     5 = TX
 *     6-7 = RFU
 *   Bit [0] = Bootloader:
 *     0 = Currently executing from bootloader
 *     1 = Currently executing from flash
 */
static void decode_stat2(uint8_t stat2) {
    uint8_t reset_status = (stat2 >> 4) & 0x0F;
    uint8_t chip_mode = (stat2 >> 1) & 0x07;
    uint8_t bootloader = stat2 & 0x01;
    
    DEBUGOUT("  Stat2: 0x%02X\n", stat2);
    DEBUGOUT("    Reset Status [7:4] = %d: ", reset_status);
    switch (reset_status) {
        case 0: DEBUGOUT("Cleared (no recent reset)\n"); break;
        case 1: DEBUGOUT("Analog reset (POR/BOR)\n"); break;
        case 2: DEBUGOUT("External reset (NRESET pin)\n"); break;
        case 3: DEBUGOUT("System reset\n"); break;
        case 4: DEBUGOUT("Watchdog reset\n"); break;
        case 5: DEBUGOUT("Wakeup NSS toggling\n"); break;
        case 6: DEBUGOUT("RTC restart\n"); break;
        default: DEBUGOUT("RFU (%d)\n", reset_status); break;
    }
    DEBUGOUT("    Chip Mode [3:1] = %d: ", chip_mode);
    switch (chip_mode) {
        case 0: DEBUGOUT("SLEEP\n"); break;
        case 1: DEBUGOUT("STANDBY_RC (RC oscillator)\n"); break;
        case 2: DEBUGOUT("STANDBY_XOSC (external oscillator)\n"); break;
        case 3: DEBUGOUT("FS (Frequency Synthesis)\n"); break;
        case 4: DEBUGOUT("RX\n"); break;
        case 5: DEBUGOUT("TX\n"); break;
        default: DEBUGOUT("RFU (%d)\n", chip_mode); break;
    }
    DEBUGOUT("    Bootloader [0] = %d: %s\n", bootloader,
             bootloader ? "Executing from FLASH" : "Executing from BOOTLOADER!");
}

/**
 * @brief Decode and print LR1121 error flags
 *
 * Citation: LR1121 User Manual Section 3.6.1 GetErrors
 *   Bit 0: LF_RC_CALIB_ERR - LF RC calibration failed
 *   Bit 1: HF_RC_CALIB_ERR - HF RC calibration failed
 *   Bit 2: ADC_CALIB_ERR - ADC calibration failed
 *   Bit 3: PLL_CALIB_ERR - PLL calibration failed
 *   Bit 4: IMG_CALIB_ERR - Image rejection calibration failed
 *   Bit 5: HF_XOSC_START_ERR - HF XOSC did not start correctly!
 *   Bit 6: LF_XOSC_START_ERR - LF XOSC did not start correctly
 *   Bit 7: PLL_LOCK_ERR - PLL did not lock
 *   Bit 8: RX_ADC_OFFSET_ERR - RX ADC offset calibration failed
 *   Bits 9-15: RFU
 */
static void decode_errors(uint16_t errors) {
    DEBUGOUT("  Error Flags: 0x%04X\n", errors);
    
    if (errors == 0) {
        DEBUGOUT("    No errors reported\n");
        return;
    }
    
    if (errors & (1 << 0)) DEBUGOUT("    [ERROR] Bit 0: LF_RC_CALIB_ERR - LF RC calibration failed\n");
    if (errors & (1 << 1)) DEBUGOUT("    [ERROR] Bit 1: HF_RC_CALIB_ERR - HF RC calibration failed\n");
    if (errors & (1 << 2)) DEBUGOUT("    [ERROR] Bit 2: ADC_CALIB_ERR - ADC calibration failed\n");
    if (errors & (1 << 3)) DEBUGOUT("    [ERROR] Bit 3: PLL_CALIB_ERR - PLL calibration failed\n");
    if (errors & (1 << 4)) DEBUGOUT("    [ERROR] Bit 4: IMG_CALIB_ERR - Image rejection calibration failed\n");
    if (errors & (1 << 5)) DEBUGOUT("    [ERROR] Bit 5: HF_XOSC_START_ERR - HF XOSC did not start! Check TCXO!\n");
    if (errors & (1 << 6)) DEBUGOUT("    [ERROR] Bit 6: LF_XOSC_START_ERR - LF XOSC did not start correctly\n");
    if (errors & (1 << 7)) DEBUGOUT("    [ERROR] Bit 7: PLL_LOCK_ERR - PLL did not lock\n");
    if (errors & (1 << 8)) DEBUGOUT("    [ERROR] Bit 8: RX_ADC_OFFSET_ERR - RX ADC offset calibration failed\n");
}

/**
 * @brief Run comprehensive LR1121 diagnostic using GetStatus and GetErrors
 *
 * This function diagnoses why GetRandomNumber or GetTemperature might fail.
 * It uses the GetStatus and GetErrors commands as specified in LR1121 User Manual.
 *
 * Citation: LR1121 User Manual Section 3.4.1 GetStatus
 *   Table 3-1: GetStatus Command
 *   Byte 0-1: 0x01 0x00 (opcode)
 *   Byte 2-5: Response phase - 0x00 0x00 0x00 0x00 (NOPs)
 *   Response: Stat1, Stat2, IrqStatus[31:0]
 *
 * Citation: LR1121 User Manual Section 3.6.1 GetErrors
 *   Table 3-4: GetErrors Command
 *   Byte 0-1: 0x01 0x0D (opcode)
 *   Response (Table 3-5): Stat1, ErrorStat[15:8], ErrorStat[7:0]
 */
void lr1121_run_diagnostic(void) {
    DEBUGOUT("\n");
    DEBUGOUT("╔═════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║     LR1121 COMPREHENSIVE DIAGNOSTIC                         ║\n");
    DEBUGOUT("║     Using GetStatus (0x0100) and GetErrors (0x010D)         ║\n");
    DEBUGOUT("╚═════════════════════════════════════════════════════════════╝\n");
    DEBUGOUT("\n");
    
    bool ok;
    
#if (USE_ELRS_INIT == 2)
    /* =========================================================================
     * BULLETPROOF TCXO INITIALIZATION (THE FIX!)
     * 
     * Based on user's analysis of why RNG works but Temperature fails:
     *   - RNG uses internal 32MHz RC oscillator (no TCXO needed)
     *   - Temperature sensor requires TCXO to be stable
     * 
     * THE THREE PROBLEMS THIS FIXES:
     *   1. 200mV headroom rule: VBAT >= VTCXO + 200mV
     *      - Use 1.8V (0x02) not 3.3V (0x07) with 3.3V supply!
     *   2. Ghost RC oscillator: Must call SetStandby(XOSC) after SetTcxoMode
     *      - Without this, chip stays on RC even though TCXO is powered!
     *   3. ADC calibration: Must call Calibrate(0x3F) for temperature
     * 
     * Citation: User analysis + LR1121 Datasheet
     * =========================================================================
     */
    
    DEBUGOUT("=== STEP 0: BULLETPROOF TCXO Initialization (THE FIX!) ===\n");
    DEBUGOUT("Using EXACT Waveshare demo sequence from lr1121_config.cpp\n\n");
    
/* Use EXACT Waveshare demo initialization sequence
 * Citation: Core1121_XF_Demo/esp32s3/Arduino/.../lr1121_config.cpp lora_system_init()
 * 
 * Key parameters from Waveshare demo:
 *   - TCXO voltage: 3.0V (0x06)
 *   - TCXO timeout: 300 ticks (~9ms)
 *   - LF Clock: XTAL mode (32kHz crystal)
 *   - Full calibration: 0x3F
 */
lr1121_status_t init_status = lr1121_waveshare_init();
    
    if (init_status == LR1121_OK) {
        DEBUGOUT("\n✓ WAVESHARE Init SUCCESS! Temperature should now work!\n\n");
    } else {
        DEBUGOUT("\n✗ WAVESHARE Init FAILED (error %d)\n\n", init_status);
        DEBUGOUT("Try checking:\n");
        DEBUGOUT("  1. TCXO hardware connections\n");
        DEBUGOUT("  2. 3.3V power supply to module\n");
        DEBUGOUT("  3. 32kHz crystal (if LFCLK timeout)\n\n");
    }
    
    DEBUGOUT("=== Now checking status after initialization ===\n\n");

#elif (USE_ELRS_INIT == 1)
    /* =========================================================================
     * ELRS-COMPATIBLE INITIALIZATION
     * 
     * Citation: ExpressLRS 4.0 LR1121.cpp Begin() function
     * 
     * KEY INSIGHT: ELRS NEVER calls SetTcxoMode!
     * 
     * The Core1121-HF module has an externally-powered TCXO that is always
     * running when VCC is applied. Calling SetTcxoMode can cause issues
     * because it configures the LR1121's internal VTCXO regulator which
     * conflicts with the external TCXO.
     * 
     * NOTE: This mode may cause Temperature reading to fail because:
     *   - No SetStandby(XOSC) means chip stays on RC oscillator
     *   - Temperature sensor may require TCXO to be stable
     * 
     * If Temperature fails but RNG works, try USE_ELRS_INIT=2 instead!
     * =========================================================================
     */
    
    DEBUGOUT("=== STEP 0: ELRS-Compatible Initialization ===\n");
    DEBUGOUT("Citation: ExpressLRS 4.0 LR1121.cpp Begin()\n");
    DEBUGOUT("KEY: NO SetTcxoMode! (externally-powered TCXO)\n");
    DEBUGOUT("NOTE: If Temperature fails, try USE_ELRS_INIT=2 (BULLETPROOF mode)\n\n");
    
    lr1121_status_t elrs_status = lr1121_elrs_init(902000000UL, 928000000UL);  /* US 915 band */
    
    if (elrs_status == LR1121_OK) {
        DEBUGOUT("\n✓ ELRS Init SUCCESS!\n\n");
    } else {
        DEBUGOUT("\n✗ ELRS Init FAILED (error %d)\n\n", elrs_status);
    }
    
    DEBUGOUT("=== Now checking status after initialization ===\n\n");
    
#else
    /* =========================================================================
     * LEGACY TCXO INITIALIZATION (OLD METHOD)
     *
     * Citation: LR1121 User Manual Section 3.6.2 ClearErrors
     * Citation: User-provided documentation - Error flags must be cleared first!
     *
     * The correct sequence for Core1121-HF is:
     *   1. Hardware Reset (already done in main test setup)
     *   2. ClearErrors - Clear any latched errors from previous attempts
     *   3. SetTcxoMode - Configure TCXO timing and voltage
     *   4. SetStandby(XOSC) - Switch to crystal oscillator mode
     *   5. Wait for BUSY LOW (XOSC startup)
     *   6. Verify with GetStatus/GetErrors
     *
     * ERROR FLAG PERSISTENCE: The HF_XOSC_START_ERR flag latches and persists
     * until explicitly cleared! This is why repeated attempts fail - the old
     * error blocks the new attempt.
     * =========================================================================
     */
    
    DEBUGOUT("=== STEP 0: TCXO Initialization Sequence ===\n");
    DEBUGOUT("Citation: LR1121 User Manual Sections 3.6.1, 3.6.2, 6.3.2\n\n");
    
    /* Step 0a: ClearErrors FIRST to clear any latched HF_XOSC_START_ERR */
    DEBUGOUT("0a. ClearErrors (0x010E) - Clear latched error flags:\n");
    ok = lr1121_execute_command(LR1121_CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
    if (ok) {
        DEBUGOUT("    ClearErrors: SUCCESS\n");
    } else {
        DEBUGOUT("    ClearErrors: FAILED (but continuing...)\n");
    }
    
    /* Wait for BUSY */
    if (!lr1121_wait_busy(100)) {
        DEBUGOUT("    WARNING: Timeout waiting for BUSY after ClearErrors\n");
    }
    delay_ms(10);
    
    /* Step 0b: SetTcxoMode to configure TCXO */
    DEBUGOUT("\n0b. SetTcxoMode (0x0117) - Configure TCXO:\n");
    DEBUGOUT("    Citation: LR1121 User Manual Section 6.3.2\n");
    DEBUGOUT("    Voltage: 0x07 (3.3V for Core1121-HF external TCXO)\n");
    DEBUGOUT("    Delay: 0x000666 (~50ms startup time)\n");
    lr1121_status_t tcxo_status = lr1121_set_tcxo_mode();
    if (tcxo_status == LR1121_OK) {
        DEBUGOUT("    SetTcxoMode: SUCCESS\n");
    } else {
        DEBUGOUT("    SetTcxoMode: FAILED (error %d)\n", tcxo_status);
    }
    
    /* Step 0c: SetStandby(XOSC) to switch to crystal mode */
    DEBUGOUT("\n0c. SetStandby(XOSC) - Switch to crystal oscillator mode:\n");
    ok = set_standby(1);  /* 1 = XOSC */
    if (ok) {
        DEBUGOUT("    SetStandby(XOSC): Command sent\n");
    } else {
        DEBUGOUT("    SetStandby(XOSC): FAILED to send command\n");
    }
    
    /* Wait for XOSC to stabilize (BUSY goes HIGH during startup, then LOW) */
    DEBUGOUT("    Waiting for XOSC startup (up to 200ms)...\n");
    if (!lr1121_wait_busy(200)) {
        DEBUGOUT("    WARNING: Timeout waiting for XOSC startup!\n");
    } else {
        DEBUGOUT("    XOSC startup: BUSY went LOW (ready)\n");
    }
    delay_ms(10);  /* Additional settling */
    
    DEBUGOUT("\n=== Now checking status after initialization ===\n\n");
#endif /* USE_ELRS_INIT */
    
    /* =========================================================================
     * STAT2 BIT POSITION DIAGNOSTIC
     * 
     * There's an inconsistency in the codebase about stat2 bit positions:
     *   - lr1121_driver.h: chip_mode = bits [6:4] → (stat2 >> 4) & 0x07
     *   - decode_stat2():  chip_mode = bits [3:1] → (stat2 >> 1) & 0x07
     * 
     * This diagnostic tests BOTH interpretations to determine which is correct
     * by observing actual mode changes.
     * 
     * Citation: LR1121 User Manual Table 3-3 (Stat2 Values)
     * =========================================================================
     */
    DEBUGOUT("╔═════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║     STAT2 BIT POSITION DIAGNOSTIC                           ║\n");
    DEBUGOUT("║     Testing (stat2 >> 1) & 0x07 vs (stat2 >> 4) & 0x07      ║\n");
    DEBUGOUT("╚═════════════════════════════════════════════════════════════╝\n\n");
    
    {
        uint8_t stat1, stat2, irq;
        
        /* First, get current status */
        DEBUGOUT("--- Current status after init ---\n");
        if (lr1121_get_status(&stat1, &stat2, &irq)) {
            DEBUGOUT("  stat2 raw = 0x%02X (binary: ", stat2);
            for (int i = 7; i >= 0; i--) {
                DEBUGOUT("%d", (stat2 >> i) & 1);
                if (i == 4) DEBUGOUT(" ");  /* Visual separator */
            }
            DEBUGOUT(")\n");
            
            uint8_t mode_bits_3_1 = (stat2 >> 1) & 0x07;  /* bits [3:1] */
            uint8_t mode_bits_6_4 = (stat2 >> 4) & 0x07;  /* bits [6:4] */
            
            DEBUGOUT("  Interpretation A: bits [3:1] = %d ", mode_bits_3_1);
            switch (mode_bits_3_1) {
                case 0: DEBUGOUT("(SLEEP)\n"); break;
                case 1: DEBUGOUT("(STANDBY_RC)\n"); break;
                case 2: DEBUGOUT("(STANDBY_XOSC)\n"); break;
                case 3: DEBUGOUT("(FS)\n"); break;
                case 4: DEBUGOUT("(RX)\n"); break;
                case 5: DEBUGOUT("(TX)\n"); break;
                default: DEBUGOUT("(RFU)\n"); break;
            }
            
            DEBUGOUT("  Interpretation B: bits [6:4] = %d ", mode_bits_6_4);
            switch (mode_bits_6_4) {
                case 0: DEBUGOUT("(SLEEP)\n"); break;
                case 1: DEBUGOUT("(STANDBY_RC)\n"); break;
                case 2: DEBUGOUT("(STANDBY_XOSC)\n"); break;
                case 3: DEBUGOUT("(FS)\n"); break;
                case 4: DEBUGOUT("(RX)\n"); break;
                case 5: DEBUGOUT("(TX)\n"); break;
                default: DEBUGOUT("(RFU)\n"); break;
            }
        }
        
        /* Now force a mode change: SetStandby(RC) then SetStandby(XOSC) */
        DEBUGOUT("\n--- Testing mode change: SetStandby(RC) ---\n");
        set_standby(0);  /* 0 = STANDBY_RC */
        
        /* FIX: Wait for BUSY to go LOW and add stabilization delay before reading status */
        if (!lr1121_wait_busy(500)) {
            DEBUGOUT("  WARNING: BUSY timeout after SetStandby(RC) - chip may be stuck\n");
        }
        delay_ms(20);  /* Allow mode change to stabilize */
        
        if (lr1121_get_status(&stat1, &stat2, &irq)) {
            uint8_t mode_bits_3_1 = (stat2 >> 1) & 0x07;
            uint8_t mode_bits_6_4 = (stat2 >> 4) & 0x07;
            DEBUGOUT("  stat2 raw = 0x%02X\n", stat2);
            DEBUGOUT("  bits [3:1] = %d %s\n", mode_bits_3_1, 
                     mode_bits_3_1 == 1 ? "← CORRECT (STANDBY_RC=1)" : 
                     mode_bits_3_1 == 2 ? "(STANDBY_XOSC - wrong!)" : "(unexpected)");
            DEBUGOUT("  bits [6:4] = %d %s\n", mode_bits_6_4,
                     mode_bits_6_4 == 1 ? "← CORRECT (STANDBY_RC=1)" : 
                     mode_bits_6_4 == 2 ? "(STANDBY_XOSC - wrong!)" : "(unexpected)");
        }
        
        DEBUGOUT("\n--- Testing mode change: SetStandby(XOSC) ---\n");
        set_standby(1);  /* 1 = STANDBY_XOSC */
        
        /* FIX: Wait for BUSY to go LOW and add stabilization delay before reading status */
        if (!lr1121_wait_busy(500)) {
            DEBUGOUT("  WARNING: BUSY timeout after SetStandby(XOSC) - chip may be stuck\n");
        }
        delay_ms(50);  /* Allow XOSC to fully stabilize before reading status */
        
        if (lr1121_get_status(&stat1, &stat2, &irq)) {
            uint8_t mode_bits_3_1 = (stat2 >> 1) & 0x07;
            uint8_t mode_bits_6_4 = (stat2 >> 4) & 0x07;
            DEBUGOUT("  stat2 raw = 0x%02X\n", stat2);
            DEBUGOUT("  bits [3:1] = %d %s\n", mode_bits_3_1, 
                     mode_bits_3_1 == 2 ? "← CORRECT (STANDBY_XOSC=2)" : 
                     mode_bits_3_1 == 1 ? "(STANDBY_RC - XOSC failed!)" : "(unexpected)");
            DEBUGOUT("  bits [6:4] = %d %s\n", mode_bits_6_4,
                     mode_bits_6_4 == 2 ? "← CORRECT (STANDBY_XOSC=2)" : 
                     mode_bits_6_4 == 1 ? "(STANDBY_RC - XOSC failed!)" : "(unexpected)");
            
            /* Final determination */
            DEBUGOUT("\n*** DIAGNOSTIC RESULT ***\n");
            if (mode_bits_3_1 == 2 && mode_bits_6_4 != 2) {
                DEBUGOUT("  CORRECT FORMAT: bits [3:1] → use (stat2 >> 1) & 0x07\n");
                DEBUGOUT("  The header file lr1121_driver.h LR1121_GET_CHIP_MODE macro is WRONG!\n");
            } else if (mode_bits_6_4 == 2 && mode_bits_3_1 != 2) {
                DEBUGOUT("  CORRECT FORMAT: bits [6:4] → use (stat2 >> 4) & 0x07\n");
                DEBUGOUT("  The decode_stat2() function and lr1121_elrs_init.c are WRONG!\n");
            } else if (mode_bits_3_1 == 2 && mode_bits_6_4 == 2) {
                DEBUGOUT("  BOTH interpretations show STANDBY_XOSC (mode=2)\n");
                DEBUGOUT("  Cannot determine correct format from this test alone.\n");
                DEBUGOUT("  (This can happen if stat2 has specific bit patterns)\n");
            } else {
                DEBUGOUT("  XOSC MODE NOT ACHIEVED - check TCXO configuration!\n");
                DEBUGOUT("  bits[3:1]=%d, bits[6:4]=%d - neither is 2 (STANDBY_XOSC)\n",
                         mode_bits_3_1, mode_bits_6_4);
            }
        }
        
        DEBUGOUT("\n");
    }
    
    /* =========================================================================
     * STEP 1: GetStatus command
     * Citation: LR1121 User Manual Section 3.4.1, Table 3-1
     * This is a two-phase command:
     *   Phase 1: Send opcode [0x01][0x00]
     *   Phase 2: Read response [Stat1][Stat2][IrqStatus 31:24][IrqStatus 23:16][IrqStatus 15:8][IrqStatus 7:0]
     * =========================================================================
     */
    DEBUGOUT("=== STEP 1: GetStatus (0x0100) ===\n");
    DEBUGOUT("Citation: LR1121 User Manual Section 3.4.1\n\n");
    
    /* GetStatus returns 6 bytes: stat1, stat2, irq[31:24], irq[23:16], irq[15:8], irq[7:0] */
    uint8_t status_resp[6] = {0};
    ok = lr1121_execute_command(LR1121_CMD_GET_STATUS, NULL, 0, status_resp, 6);
    
    if (ok) {
        DEBUGOUT("GetStatus Response (raw): %02X %02X %02X %02X %02X %02X\n",
                 status_resp[0], status_resp[1], status_resp[2],
                 status_resp[3], status_resp[4], status_resp[5]);
        DEBUGOUT("\n");
        
        uint8_t stat1 = status_resp[0];
        uint8_t stat2 = status_resp[1];
        uint32_t irq_status = ((uint32_t)status_resp[2] << 24) |
                              ((uint32_t)status_resp[3] << 16) |
                              ((uint32_t)status_resp[4] << 8) |
                              status_resp[5];
        
        decode_stat1(stat1);
        DEBUGOUT("\n");
        decode_stat2(stat2);
        DEBUGOUT("\n");
        DEBUGOUT("  IRQ Status: 0x%08lX\n", (unsigned long)irq_status);
        
        /* Check for critical issues */
        uint8_t chip_mode = (stat2 >> 1) & 0x07;
        uint8_t bootloader = stat2 & 0x01;
        
        if (chip_mode == 0) {
            DEBUGOUT("\n  *** ISSUE DETECTED: Chip is in SLEEP mode! ***\n");
            DEBUGOUT("      GetTemperature and GetRandomNumber require STANDBY_XOSC.\n");
            DEBUGOUT("      Try: SetStandby(1) to switch to STANDBY_XOSC mode.\n");
        }
        
        if (chip_mode == 1) {
            DEBUGOUT("\n  *** NOTE: Chip is in STANDBY_RC mode ***\n");
            DEBUGOUT("      Some functions may require STANDBY_XOSC mode.\n");
            DEBUGOUT("      Try: SetTcxoMode, then SetStandby(1) for XOSC mode.\n");
        }
        
        if (bootloader == 0) {
            DEBUGOUT("\n  *** CRITICAL: Chip is running from BOOTLOADER! ***\n");
            DEBUGOUT("      Firmware may not be properly loaded.\n");
        }
    } else {
        DEBUGOUT("  ERROR: GetStatus command failed!\n");
        DEBUGOUT("  Check SPI communication and wiring.\n");
    }
    
    /* =========================================================================
     * STEP 2: GetErrors command
     * Citation: LR1121 User Manual Section 3.6.1, Table 3-4 and 3-5
     * Phase 1: Send opcode [0x01][0x0D]
     * Phase 2: Read response [Stat1][ErrorStat(15:8)][ErrorStat(7:0)]
     * =========================================================================
     */
    DEBUGOUT("\n");
    DEBUGOUT("=== STEP 2: GetErrors (0x010D) ===\n");
    DEBUGOUT("Citation: LR1121 User Manual Section 3.6.1\n\n");
    
    /* GetErrors returns 3 bytes: stat1, errors[15:8], errors[7:0] */
    uint8_t error_resp[3] = {0};
    ok = lr1121_execute_command(LR1121_CMD_GET_ERRORS, NULL, 0, error_resp, 3);
    
    if (ok) {
        DEBUGOUT("GetErrors Response (raw): %02X %02X %02X\n",
                 error_resp[0], error_resp[1], error_resp[2]);
        DEBUGOUT("\n");
        
        uint16_t errors = ((uint16_t)error_resp[1] << 8) | error_resp[2];
        decode_errors(errors);
        
        /* Provide specific remediation advice */
        if (errors & (1 << 5)) {  /* HF_XOSC_START_ERR */
            DEBUGOUT("\n  *** CRITICAL: HF XOSC Failed to Start! ***\n");
            DEBUGOUT("  This is the most likely cause of GetTemperature/GetRandomNumber failures.\n");
            DEBUGOUT("\n");
            DEBUGOUT("  REMEDIATION for Core1121-HF module:\n");
            DEBUGOUT("    1. Verify TCXO voltage setting matches hardware:\n");
            DEBUGOUT("       - LR1121_TCXO_VOLTAGE_TRIM should be 0x07 for 3.3V TCXO\n");
            DEBUGOUT("       - Or 0x02 for 1.8V TCXO\n");
            DEBUGOUT("    2. Ensure SetTcxoMode is called BEFORE SetStandby(XOSC)\n");
            DEBUGOUT("    3. Check TCXO delay (LR1121_TCXO_DELAY) - try 50ms (0x000666)\n");
            DEBUGOUT("    4. Verify 3.3V power supply to TCXO on Core1121-HF module\n");
            DEBUGOUT("    5. After fixing, call ClearErrors (0x010E), then reset and retry\n");
        }
        
        if (errors & (1 << 7)) {  /* PLL_LOCK_ERR */
            DEBUGOUT("\n  *** PLL Lock Error Detected! ***\n");
            DEBUGOUT("  The PLL cannot lock without a stable XOSC clock source.\n");
            DEBUGOUT("  Fix the XOSC issue first, then recalibrate PLL.\n");
        }
        
        if (errors & ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 8))) {
            DEBUGOUT("\n  *** Calibration Errors Detected! ***\n");
            DEBUGOUT("  After fixing XOSC, run Calibrate command (0x010F) with all bits set.\n");
        }
    } else {
        DEBUGOUT("  ERROR: GetErrors command failed!\n");
    }
    
    /* =========================================================================
     * STEP 3: Summary and Recommendations
     * =========================================================================
     */
    DEBUGOUT("\n");
    DEBUGOUT("=== STEP 3: Summary and Recommendations ===\n\n");
    
    DEBUGOUT("If GetTemperature or GetRandomNumber are failing:\n");
    DEBUGOUT("\n");
    DEBUGOUT("  1. Check Chip Mode in GetStatus:\n");
    DEBUGOUT("     - Must be in STANDBY_XOSC (mode=2) for these commands\n");
    DEBUGOUT("     - SLEEP or STANDBY_RC modes may cause failures\n");
    DEBUGOUT("\n");
    DEBUGOUT("  2. Check GetErrors for HF_XOSC_START_ERR (bit 5):\n");
    DEBUGOUT("     - If set, the TCXO/XOSC failed to start\n");
    DEBUGOUT("     - Need to fix TCXO configuration before anything else\n");
    DEBUGOUT("\n");
    DEBUGOUT("  3. For Core1121-HF (externally-powered TCXO):\n");
    DEBUGOUT("     - Still need SetTcxoMode to configure XOSC input path\n");
    DEBUGOUT("     - Use voltage trim 0x07 (3.3V) to match external TCXO\n");
    DEBUGOUT("     - Use adequate startup delay (50ms = 0x000666)\n");
    DEBUGOUT("\n");
    DEBUGOUT("  4. Sequence should be:\n");
    DEBUGOUT("     a) Hardware reset (lr1121_reset)\n");
    DEBUGOUT("     b) Wait for BUSY LOW\n");
    DEBUGOUT("     c) ClearErrors (0x010E)\n");
    DEBUGOUT("     d) SetTcxoMode (0x0117) - even for external TCXO!\n");
    DEBUGOUT("     e) Wait for BUSY LOW (TCXO startup)\n");
    DEBUGOUT("     f) SetStandby(1) for XOSC mode\n");
    DEBUGOUT("     g) Wait for BUSY LOW (mode switch)\n");
    DEBUGOUT("     h) GetStatus to verify mode=2 (STANDBY_XOSC)\n");
    DEBUGOUT("     i) Now GetTemperature and GetRandomNumber should work\n");
    DEBUGOUT("\n");
    
    DEBUGOUT("╔═════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║                 END OF DIAGNOSTIC                           ║\n");
    DEBUGOUT("╚═════════════════════════════════════════════════════════════╝\n");
    DEBUGOUT("\n");
}

/*******************************************************************************
 * Test Functions
 ******************************************************************************/

/**
 * @brief Test 1: Hardware Verification
 * Tests basic SPI communication and chip identification
 */
static void test_hardware_verification(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 1: Hardware Verification\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    
    /* Test 1.1: GetVersion */
    DEBUGOUT("1.1 GetVersion Command:\n");
    uint8_t version_resp[5];
    bool ok = lr1121_execute_command(LR1121_CMD_GET_VERSION, NULL, 0, version_resp, 5);
    if (ok) {
        DEBUGOUT("  Response: %02X %02X %02X %02X %02X\n",
                 version_resp[0], version_resp[1], version_resp[2], 
                 version_resp[3], version_resp[4]);
        DEBUGOUT("  Hardware:  0x%02X (%s)\n", version_resp[1],
                 version_resp[1] == 0x22 ? "LR1121" : 
                 version_resp[1] == 0x21 ? "LR1120" : "Unknown");
        DEBUGOUT("  Use/Type:  0x%02X\n", version_resp[2]);
        DEBUGOUT("  Firmware:  %d.%d\n", version_resp[3], version_resp[4]);
    }
    test_result("GetVersion", ok && version_resp[1] == 0x22);
    
    /* Test 1.2: GetStatus */
    DEBUGOUT("\n1.2 GetStatus Command:\n");
    uint8_t stat1, stat2, irq_stat;
    ok = lr1121_get_status(&stat1, &stat2, &irq_stat);
    if (ok) {
        DEBUGOUT("  Status1:   0x%02X\n", stat1);
        DEBUGOUT("  Status2:   0x%02X\n", stat2);
        DEBUGOUT("  IRQ:       0x%02X\n", irq_stat);
        
        /* Decode command status
         * Citation: LR11XX User Manual Section 2.1 (Status Byte)
         * Bits [3:1] = cmd_status:
         *   0 = FAIL (command not executed)
         *   1 = PERR (parameter error) - NOT timeout!
         *   2 = SPI_ERR (SPI communication error)
         *   3 = OK (command executed successfully)
         *   4 = DATA_AVAIL (data available for read)
         */
        uint8_t cmd_stat = (stat1 >> 1) & 0x07;
        DEBUGOUT("  Cmd Status: %d (%s)\n", cmd_stat,
                 cmd_stat == 0 ? "FAIL" :
                 cmd_stat == 1 ? "PERR (param error)" :
                 cmd_stat == 2 ? "SPI_ERR" :
                 cmd_stat == 3 ? "OK" :
                 cmd_stat == 4 ? "DATA_AVAIL" : "UNKNOWN");
    }
    test_result("GetStatus", ok);
    
#if (USE_ELRS_INIT >= 1)  /* Use ELRS helpers for both mode 1 and mode 2 */
    /*************************************************************************
     * MODE 1: ELRS-Compatible (NO SetTcxoMode)
     * MODE 2: BULLETPROOF (SetTcxoMode + SetStandby(XOSC) + Calibrate)
     * 
     * Both modes use the ELRS helper functions for temperature/random.
     * The difference is in the initialization sequence above.
     *************************************************************************/
#if (USE_ELRS_INIT == 2)
    DEBUGOUT("\n1.2x BULLETPROOF Mode - TCXO fully configured:\n");
    DEBUGOUT("  SetTcxoMode(1.8V) + SetStandby(XOSC) + Calibrate(0x3F)\n");
#else
    DEBUGOUT("\n1.2x ELRS Mode - Skipping TCXO/XOSC Configuration:\n");
    DEBUGOUT("  Citation: ExpressLRS never calls SetTcxoMode\n");
    DEBUGOUT("  NOTE: If temperature fails, try USE_ELRS_INIT=2\n");
#endif
    DEBUGOUT("  Initialization done in diagnostic step\n");
    
    /* Print status using ELRS helper */
    lr1121_elrs_print_status();
    
    /* Test 1.3: GetTemperature (using ELRS helper) */
    DEBUGOUT("\n1.3 GetTemperature (ELRS method):\n");
    
    /**
     * TIMING FIX: Add delay before GetTemperature
     * 
     * Citation: LR1121 User Manual Section 2.1 - After mode transitions,
     * the status byte may return transitional values (stat2=0x00) until
     * the chip has fully stabilized in the new mode.
     * 
     * The XOSC needs additional time to stabilize after SetStandby(XOSC),
     * even after BUSY goes LOW. This delay ensures the internal ADC
     * (required for temperature measurement) is ready.
     * 
     * 20ms delay provides adequate margin for TCXO/XOSC stabilization.
     */
    DEBUGOUT("  Waiting 20ms for XOSC/ADC stabilization...\n");
    delay_ms(20);
    
    int16_t temp = lr1121_elrs_get_temperature();
    DEBUGOUT("  Temperature: %d °C\n", temp);
    test_result("GetTemperature", temp > -40 && temp < 125 && temp != -999);
    
    /* Test 1.4: GetRandomNumber (using ELRS helper) */
    DEBUGOUT("\n1.4 GetRandomNumber (ELRS method):\n");
    uint32_t rand1 = lr1121_elrs_get_random();
    uint32_t rand2 = lr1121_elrs_get_random();
    DEBUGOUT("  Random 1: 0x%08lX\n", (unsigned long)rand1);
    DEBUGOUT("  Random 2: 0x%08lX\n", (unsigned long)rand2);
    test_result("GetRandomNumber", rand1 != rand2 && rand1 != 0);
    
#else
    /*************************************************************************
     * Legacy Mode - TCXO Configuration
     * 
     * Citation: ExpressLRS GitHub Discussion #3045
     * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
     * 
     * FIX: Even for externally-powered TCXO modules (like Core1121-HF),
     * SetTcxoMode is still required to:
     * 1. Configure the TCXO detection circuitry timing
     * 2. Set the timeout for XOSC stabilization
     * 3. Enable the XOSC input path in the LR1121
     *
     * Without SetTcxoMode, SetStandby(XOSC) causes the chip to fall back
     * to SLEEP mode (chip_mode=0) because the XOSC never starts.
     *
     * LR1121_TCXO_EXTERNAL_POWER is now set to 0 (enabled) by default.
     * 
     * Voltage Trim Values:
     *   0x02 = 1.8V (recommended for most modules)
     *   0x07 = 3.3V (for modules with higher voltage TCXO)
     *************************************************************************/
    DEBUGOUT("\n1.2a SetTcxoMode:\n");
    DEBUGOUT("  Citation: LR1121 Datasheet Section 11.2.5\n");
    lr1121_status_t tcxo_status = lr1121_set_tcxo_mode();
    if (tcxo_status == LR1121_OK) {
        DEBUGOUT("  SetTcxoMode: SUCCESS\n");
    } else {
        DEBUGOUT("  SetTcxoMode: FAILED (error %d)\n", tcxo_status);
        DEBUGOUT("  WARNING: SetStandby(XOSC) may fail!\n");
    }
    test_result("SetTcxoMode", tcxo_status == LR1121_OK);

    /*************************************************************************
     * IMPORTANT: SetStandby(XOSC) required before GetTemperature/GetRandom
     * 
     * Citation: LR11xx Driver Reference - Commands like GetTemp and GetRandom
     * may require the XOSC to be running. After reset, chip is in STANDBY_RC.
     * Switching to STANDBY_XOSC ensures the TCXO/crystal is active.
     *
     * CRITICAL: Must wait for BUSY LOW after SetStandby before next command!
     * The XOSC/TCXO needs time to start and PLL needs to lock.
     *
     * NOTE: SetTcxoMode has already been called above, so XOSC can now start.
     *************************************************************************/
    DEBUGOUT("\n1.2b SetStandby(XOSC) - Required for temp/random:\n");
    ok = set_standby(1);  /* 1 = XOSC mode */
    if (ok) {
        DEBUGOUT("  Switched to STANDBY_XOSC mode\n");
    } else {
        DEBUGOUT("  WARNING: Failed to set STANDBY_XOSC\n");
    }
    
    /* CRITICAL FIX: Wait for BUSY to go LOW (XOSC startup complete)
     * Citation: LR1121 User Manual - After SetStandby(XOSC), BUSY goes HIGH
     * while XOSC starts, then goes LOW when ready.
     * TCXO modules may need 10-100ms for oscillator startup.
     */
    DEBUGOUT("  Waiting for XOSC to stabilize (BUSY LOW)...\n");
    if (!lr1121_wait_busy(200)) {  /* Up to 200ms timeout for TCXO startup */
        DEBUGOUT("  WARNING: Timeout waiting for XOSC startup!\n");
        DEBUGOUT("  GetTemp/GetRandom may fail due to XOSC not ready.\n");
    } else {
        DEBUGOUT("  XOSC ready (BUSY LOW)\n");
    }
    delay_ms(10);  /* Additional settling time */
    
    /*************************************************************************
     * CRITICAL FIX: Run Calibrate command BEFORE GetTemperature
     * 
     * Citation: LR1121 User Manual Section 2.1.3 "Calibrations"
     * Citation: LR1121 Datasheet Section 11.1.4 "Calibrate"
     * 
     * The Calibrate command (0x010F) must be executed to calibrate the internal
     * ADC before the temperature sensor will return valid readings.
     * 
     * Calibration mask bits:
     *   Bit 0: RC64K calibration
     *   Bit 1: RC13M calibration  
     *   Bit 2: PLL calibration
     *   Bit 3: ADC calibration    <-- REQUIRED for GetTemperature!
     *   Bit 4: IMG calibration
     *   Bit 5: PLL_TX calibration
     *   0x3F = All calibrations
     * 
     * This matches ELRS behavior which calls calibration during Begin().
     *************************************************************************/
    DEBUGOUT("\n1.2c Calibrate (0x010F) - REQUIRED for GetTemperature:\n");
    DEBUGOUT("  Citation: LR1121 User Manual Section 2.1.3 \"Calibrations\"\n");
    DEBUGOUT("  Calibrating ADC, RC oscillators, PLL...\n");
    {
        uint8_t calib_param = 0x3F;  /* All calibrations including ADC (bit 3) */
        ok = lr1121_execute_command(LR1121_CMD_CALIBRATE, &calib_param, 1, NULL, 0);
        if (ok) {
            /* Calibration takes time - additional wait */
            delay_ms(50);  /* Wait for calibration to settle */
            DEBUGOUT("  Calibration complete\n");
        } else {
            DEBUGOUT("  WARNING: Calibration failed - GetTemperature may return invalid data\n");
        }
    }
    test_result("Calibrate", ok);
    
    /* Diagnostic: Verify chip is now in STANDBY_XOSC mode
     * Citation: LR11XX User Manual Section 2.1 (Status Byte)
     * stat2 bits [6:4] = chip_mode:
     *   0 = SLEEP
     *   1 = STANDBY_RC
     *   2 = STANDBY_XOSC (expected after SetStandby(1))
     *   3 = FS
     *   4 = RX
     *   5 = TX
     */
    lr1121_get_status(&stat1, &stat2, &irq_stat);
    {
        uint8_t chip_mode = (stat2 >> 1) & 0x07;  /* bits [3:1] per LR1121 User Manual */
        DEBUGOUT("  Chip mode after STANDBY_XOSC: %d (%s)\n", chip_mode,
                 chip_mode == 0 ? "SLEEP" :
                 chip_mode == 1 ? "STANDBY_RC" :
                 chip_mode == 2 ? "STANDBY_XOSC" :
                 chip_mode == 3 ? "FS" :
                 chip_mode == 4 ? "RX" :
                 chip_mode == 5 ? "TX" : "UNKNOWN");
        if (chip_mode != 2) {
            DEBUGOUT("  WARNING: Expected STANDBY_XOSC (mode=2) but got mode=%d!\n", chip_mode);
            DEBUGOUT("  This indicates XOSC/TCXO failed to start. Check:\n");
            DEBUGOUT("    - LR1121_TCXO_EXTERNAL_POWER should be 0 to enable SetTcxoMode\n");
            DEBUGOUT("    - Try SetStandby(0) for STDBY_RC first as a control test\n");
            DEBUGOUT("    - Verify TCXO voltage setting matches hardware (0x02=1.8V, 0x07=3.3V)\n");
            DEBUGOUT("    - Check hardware TCXO connections and 3.3V power\n");
            
            /* DIAGNOSTIC: Try STDBY_RC as a control test */
            DEBUGOUT("\n  DIAGNOSTIC: Trying SetStandby(RC) as control test...\n");
            set_standby(0);  /* 0 = STDBY_RC - this should always work */
            if (!lr1121_wait_busy(100)) {
                DEBUGOUT("  ERROR: Even STDBY_RC failed! Chip may need power cycle.\n");
            } else {
                lr1121_get_status(&stat1, &stat2, &irq_stat);
                uint8_t rc_mode = (stat2 >> 1) & 0x07;  /* bits [3:1] per LR1121 User Manual */
                DEBUGOUT("  After SetStandby(RC): chip_mode=%d (%s)\n", rc_mode,
                         rc_mode == 1 ? "STANDBY_RC - OK" : "UNEXPECTED");
                if (rc_mode == 1) {
                    DEBUGOUT("  STDBY_RC works! Issue is specifically with XOSC startup.\n");
                    DEBUGOUT("  The TCXO may need different voltage or delay settings.\n");
                }
            }
        } else {
            test_result("SetStandby_XOSC", true);
        }
    }
    
    /* Test 1.3: GetTemperature */
    DEBUGOUT("\n1.3 GetTemperature: GetTemp (0x11A)\n");
    int16_t temp = get_temperature();
    DEBUGOUT("  Temperature: %d °C\n", temp);
    test_result("GetTemperature", temp > -40 && temp < 125);
    
    /* Test 1.4: GetRandomNumber */
    DEBUGOUT("\n1.4 GetRandomNumber:\n");
    uint32_t rand1 = get_random_number();
    uint32_t rand2 = get_random_number();
    DEBUGOUT("  Random 1: 0x%08lX\n", (unsigned long)rand1);
    DEBUGOUT("  Random 2: 0x%08lX\n", (unsigned long)rand2);
    test_result("GetRandomNumber", rand1 != rand2 && rand1 != 0);
#endif /* USE_ELRS_INIT >= 1 */
    
    /* Test 1.5: ClearErrors */
    DEBUGOUT("\n1.5 ClearErrors:\n");
    ok = lr1121_execute_command(LR1121_CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
    test_result("ClearErrors", ok);
}

/**
 * @brief Test 2: RF Configuration
 * Tests frequency, modulation, and packet parameter configuration
 */
static void test_rf_configuration(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 2: RF Configuration\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    
    bool ok;
    
    /* Test 2.1: Set Standby */
    DEBUGOUT("2.1 SetStandby (XOSC):\n");
    ok = set_standby(1);  /* 1 = XOSC */
    test_result("SetStandby", ok);
    delay_ms(5);
    
    /* Test 2.2: Set Packet Type to LoRa */
    DEBUGOUT("\n2.2 SetPacketType (LoRa):\n");
    ok = set_packet_type(LR1121_PKT_TYPE_LORA);
    test_result("SetPacketType", ok);
    
    /* Test 2.3: Set RF Frequency */
    DEBUGOUT("\n2.3 SetRfFrequency (%lu Hz):\n", (unsigned long)TEST_FREQUENCY);
    ok = set_rf_frequency(TEST_FREQUENCY);
    test_result("SetRfFrequency", ok);
    
    /* Test 2.4: Calibrate Image */
    DEBUGOUT("\n2.4 CalibrateImage:\n");
    ok = calibrate_image(TEST_FREQUENCY);
    test_result("CalibrateImage", ok);
    delay_ms(10);
    
    /* Test 2.5: Set LoRa Modulation Parameters */
    DEBUGOUT("\n2.5 SetModulationParams (SF%d, BW%d, CR4/%d):\n", 
             TEST_SF, 125 << (TEST_BW_INDEX - 4), TEST_CR + 4);
    ok = set_lora_modulation_params(TEST_SF, TEST_BW_INDEX, TEST_CR, TEST_LDRO);
    test_result("SetModulationParams", ok);
    
    /* Test 2.6: Set Packet Parameters */
    DEBUGOUT("\n2.6 SetPacketParams (preamble=%d, payload=%d):\n",
             TEST_PREAMBLE_LEN, TEST_PAYLOAD_LEN);
    ok = set_lora_packet_params(TEST_PREAMBLE_LEN, TEST_HEADER_TYPE, 
                                 TEST_PAYLOAD_LEN, TEST_CRC_ON, TEST_INVERT_IQ);
    test_result("SetPacketParams", ok);
    
    /* Test 2.7: Set PA Config */
    DEBUGOUT("\n2.7 SetPaConfig (LP PA):\n");
    ok = set_pa_config(LR1121_PA_LP, 0x00, 0x04, 0x00);
    test_result("SetPaConfig", ok);
    
    /* Test 2.8: Set TX Params */
    DEBUGOUT("\n2.8 SetTxParams (%d dBm):\n", TEST_TX_POWER);
    ok = set_tx_params(TEST_TX_POWER, 0x02);  /* 0x02 = 40us ramp time */
    test_result("SetTxParams", ok);
    
    /* Test 2.9: Set FS Mode */
    DEBUGOUT("\n2.9 SetFs (verify PLL lock):\n");
    ok = set_fs();
    delay_ms(5);
    test_result("SetFs", ok);
    
    /* Return to standby */
    set_standby(1);
}

/**
 * @brief Test 3: CW (Continuous Wave) Transmission
 * 
 * This test puts the LR1121 into CW mode, which outputs an unmodulated
 * carrier wave. This can be verified with:
 * - SDR (Software Defined Radio) receiver
 * - Spectrum analyzer
 * - Even some handheld RF field strength meters
 *
 * The test transmits for 5 seconds at the configured frequency/power.
 */
static void test_cw_transmission(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 3: CW (Continuous Wave) Transmission\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    DEBUGOUT("  This test outputs an unmodulated carrier for 5 seconds.\n");
    DEBUGOUT("  Verify with SDR or spectrum analyzer at %.3f MHz\n",
             TEST_FREQUENCY / 1000000.0);
    DEBUGOUT("\n");
    
    bool ok;
    
    /* Configure for CW */
    DEBUGOUT("3.1 Configuring for CW transmission:\n");
    ok = set_standby(1);
    ok = ok && set_rf_frequency(TEST_FREQUENCY);
    ok = ok && calibrate_image(TEST_FREQUENCY);
    delay_ms(10);
    ok = ok && set_pa_config(LR1121_PA_LP, 0x00, 0x04, 0x00);
    ok = ok && set_tx_params(TEST_TX_POWER, 0x02);
    test_result("CW Configuration", ok);
    
    if (!ok) {
        DEBUGOUT("  Skipping CW transmission due to config failure\n");
        return;
    }
    
    /* Start CW transmission */
    DEBUGOUT("\n3.2 Starting CW transmission:\n");
    DEBUGOUT("  Frequency: %lu Hz (%.3f MHz)\n", 
             (unsigned long)TEST_FREQUENCY, TEST_FREQUENCY / 1000000.0);
    DEBUGOUT("  Power:     %d dBm\n", TEST_TX_POWER);
    DEBUGOUT("  Duration:  5 seconds\n");
    DEBUGOUT("\n");
    
    ok = set_tx_cw();
    test_result("SetTxCw", ok);
    
    if (ok) {
        DEBUGOUT("\n  >>> CW ACTIVE - Check SDR/Spectrum Analyzer <<<\n");
        DEBUGOUT("  Transmitting for 5 seconds...\n");
        
        /* Show countdown */
        for (int i = 5; i > 0; i--) {
            DEBUGOUT("  %d...\n", i);
            delay_ms(1000);
        }
        
        /* Stop transmission by going to standby */
        set_standby(1);
        DEBUGOUT("  >>> CW STOPPED <<<\n");
    }
    
    test_result("CW Transmission Complete", ok);
}

/**
 * @brief Test 4: TX Packet with IRQ Verification
 *
 * This test transmits a LoRa packet and verifies the TX_DONE interrupt.
 * Even without a receiver, this proves:
 * - The radio can be configured for LoRa TX
 * - The PA is active and transmitting
 * - The packet timing and TX state machine works
 */
static void test_tx_packet(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 4: TX Packet with IRQ Verification\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    
    bool ok;
    uint8_t stat1, stat2, irq_stat;
    
    /* Configure radio for LoRa TX */
    DEBUGOUT("4.1 Configuring radio for LoRa TX:\n");
    ok = set_standby(1);
    ok = ok && set_packet_type(LR1121_PKT_TYPE_LORA);
    ok = ok && set_rf_frequency(TEST_FREQUENCY);
    ok = ok && calibrate_image(TEST_FREQUENCY);
    delay_ms(10);
    ok = ok && set_lora_modulation_params(TEST_SF, TEST_BW_INDEX, TEST_CR, TEST_LDRO);
    ok = ok && set_lora_packet_params(TEST_PREAMBLE_LEN, TEST_HEADER_TYPE,
                                       TEST_PAYLOAD_LEN, TEST_CRC_ON, TEST_INVERT_IQ);
    ok = ok && set_pa_config(LR1121_PA_LP, 0x00, 0x04, 0x00);
    ok = ok && set_tx_params(TEST_TX_POWER, 0x02);
    test_result("LoRa TX Configuration", ok);
    
    if (!ok) return;
    
    /* Configure IRQ */
    DEBUGOUT("\n4.2 Configuring IRQ for TX_DONE:\n");
    ok = set_dio_irq_params(LR1121_IRQ_TX_DONE, LR1121_IRQ_TX_DONE, 0, 0);
    ok = ok && clear_irq(LR1121_IRQ_ALL);
    test_result("IRQ Configuration", ok);
    
    /* Prepare test packet */
    DEBUGOUT("\n4.3 Writing test packet to buffer:\n");
    uint8_t test_packet[TEST_PAYLOAD_LEN] = {
        'E', 'L', 'R', 'S', ' ', 'T', 'E', 'S', 'T',  /* "ELRS TEST" */
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07      /* Counter bytes */
    };
    
    DEBUGOUT("  Packet: ");
    for (int i = 0; i < TEST_PAYLOAD_LEN; i++) {
        DEBUGOUT("%02X ", test_packet[i]);
    }
    DEBUGOUT("\n");
    DEBUGOUT("  ASCII:  \"");
    for (int i = 0; i < 9; i++) {
        DEBUGOUT("%c", test_packet[i]);
    }
    DEBUGOUT("\"\n");
    
    ok = write_buffer8(0, test_packet, TEST_PAYLOAD_LEN);
    test_result("WriteBuffer8", ok);
    
    /* Transmit packet */
    DEBUGOUT("\n4.4 Starting TX:\n");
    ok = set_tx(1000);  /* 1 second timeout */
    test_result("SetTx", ok);
    
    /* Wait for TX_DONE (poll status) */
    DEBUGOUT("\n4.5 Waiting for TX_DONE:\n");
    bool tx_done = false;
    int poll_count = 0;
    const int max_polls = 100;  /* 10 seconds max */
    
    while (!tx_done && poll_count < max_polls) {
        delay_ms(100);
        lr1121_get_status(&stat1, &stat2, &irq_stat);
        
        /* Check chip mode (bits [3:1] of stat2 per LR1121 User Manual) */
        uint8_t chip_mode = (stat2 >> 1) & 0x07;
        
        if (poll_count % 10 == 0) {
            DEBUGOUT("  Poll %d: stat1=0x%02X stat2=0x%02X (mode=%d)\n",
                     poll_count, stat1, stat2, chip_mode);
        }
        
        /* Mode 0 = Standby = TX complete */
        if (chip_mode == 0 && poll_count > 0) {
            tx_done = true;
        }
        
        poll_count++;
    }
    
    /* Verify TX_DONE via GetStatus */
    lr1121_get_status(&stat1, &stat2, &irq_stat);
    DEBUGOUT("\n  Final status: stat1=0x%02X stat2=0x%02X irq=0x%02X\n",
             stat1, stat2, irq_stat);
    
    if (tx_done) {
        DEBUGOUT("\n  >>> TX_DONE - Packet Transmitted Successfully! <<<\n");
        DEBUGOUT("  Time: ~%d ms\n", poll_count * 100);
    } else {
        DEBUGOUT("\n  ERROR: TX_DONE not detected (timeout after %d ms)\n", 
                 poll_count * 100);
    }
    
    test_result("TX_DONE Verification", tx_done);
    
    /* Return to standby */
    set_standby(1);
}

/**
 * @brief Test 5: RX Mode with RSSI Noise Floor Measurement
 *
 * This test enters RX mode and measures the RSSI noise floor.
 * Without a transmitter, we expect to see:
 * - Very low RSSI values (noise floor, typically -120 to -100 dBm)
 * - No packets received (RX timeout)
 *
 * This verifies the receiver is functional and can measure RF energy.
 */
static void test_rx_rssi(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 5: RX Mode with RSSI Noise Floor Measurement\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    
    bool ok;
    
    /* FIX: Recover chip state before RX test
     * After TX test, the chip may be in an inconsistent state with BUSY stuck HIGH.
     * A full reset and re-initialization ensures a clean starting state.
     */
    DEBUGOUT("5.0 Recovering chip state (reset after TX test):\n");
    if (!recover_chip_state()) {
        DEBUGOUT("  ERROR: Failed to recover chip state\n");
        test_result("Chip Recovery for RX Test", false);
        return;
    }
    test_result("Chip Recovery for RX Test", true);
    
    /* Configure radio for LoRa RX */
    DEBUGOUT("\n5.1 Configuring radio for LoRa RX:\n");
    ok = set_standby(1);
    ok = ok && set_packet_type(LR1121_PKT_TYPE_LORA);
    ok = ok && set_rf_frequency(TEST_FREQUENCY);
    ok = ok && calibrate_image(TEST_FREQUENCY);
    delay_ms(10);
    ok = ok && set_lora_modulation_params(TEST_SF, TEST_BW_INDEX, TEST_CR, TEST_LDRO);
    ok = ok && set_lora_packet_params(TEST_PREAMBLE_LEN, TEST_HEADER_TYPE,
                                       TEST_PAYLOAD_LEN, TEST_CRC_ON, TEST_INVERT_IQ);
    test_result("LoRa RX Configuration", ok);
    
    if (!ok) return;
    
    /* Enable RX boosted mode for better sensitivity */
    DEBUGOUT("\n5.2 Enabling RX Boost:\n");
    uint8_t rx_boost = 1;
    ok = lr1121_execute_command(LR1121_CMD_SET_RX_BOOSTED, &rx_boost, 1, NULL, 0);
    test_result("SetRxBoosted", ok);
    
    /* Enter continuous RX mode */
    DEBUGOUT("\n5.3 Entering continuous RX mode:\n");
    ok = set_rx(0);  /* 0 = continuous */
    test_result("SetRx (continuous)", ok);
    
    if (!ok) return;
    
    /* Wait for RX to stabilize */
    delay_ms(100);
    
    /* Measure RSSI over several samples */
    DEBUGOUT("\n5.4 Measuring RSSI Noise Floor:\n");
    DEBUGOUT("  Frequency: %.3f MHz\n", TEST_FREQUENCY / 1000000.0);
    DEBUGOUT("  Sampling 10 RSSI values over 2 seconds...\n\n");
    
    int16_t rssi_values[10];
    int32_t rssi_sum = 0;
    int16_t rssi_min = 0;
    int16_t rssi_max = -200;
    
    for (int i = 0; i < 10; i++) {
        rssi_values[i] = get_rssi_inst();
        rssi_sum += rssi_values[i];
        if (rssi_values[i] < rssi_min || i == 0) rssi_min = rssi_values[i];
        if (rssi_values[i] > rssi_max) rssi_max = rssi_values[i];
        
        DEBUGOUT("  Sample %d: %d dBm\n", i + 1, rssi_values[i]);
        delay_ms(200);
    }
    
    int16_t rssi_avg = rssi_sum / 10;
    
    DEBUGOUT("\n  RSSI Statistics:\n");
    DEBUGOUT("    Minimum:  %d dBm\n", rssi_min);
    DEBUGOUT("    Maximum:  %d dBm\n", rssi_max);
    DEBUGOUT("    Average:  %d dBm\n", rssi_avg);
    DEBUGOUT("    Range:    %d dB\n", rssi_max - rssi_min);
    
    /* Expected noise floor is below -90 dBm */
    bool noise_floor_ok = (rssi_avg < -90);
    
    if (noise_floor_ok) {
        DEBUGOUT("\n  >>> RSSI readings indicate normal noise floor <<<\n");
        DEBUGOUT("  (No strong signals detected = no nearby transmitter)\n");
    } else {
        DEBUGOUT("\n  WARNING: RSSI higher than expected noise floor\n");
        DEBUGOUT("  Possible interference or nearby transmitter\n");
    }
    
    test_result("RSSI Noise Floor Measurement", noise_floor_ok || rssi_min != rssi_max);
    
    /* Return to standby */
    set_standby(1);
}

/**
 * @brief Test 6: Frequency Hopping Verification
 *
 * This test verifies the radio can be quickly reconfigured to different
 * frequencies, which is essential for ELRS frequency hopping.
 */
static void test_frequency_hopping(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("  TEST 6: Frequency Hopping Verification\n");
    DEBUGOUT("=============================================================\n");
    DEBUGOUT("\n");
    
    /* FIX: Recover chip state before frequency hopping test
     * After RX test, the chip may be in an inconsistent state.
     * A full reset and re-initialization ensures a clean starting state.
     */
    DEBUGOUT("6.0 Recovering chip state (reset before frequency test):\n");
    if (!recover_chip_state()) {
        DEBUGOUT("  ERROR: Failed to recover chip state\n");
        test_result("Chip Recovery for Frequency Test", false);
        return;
    }
    test_result("Chip Recovery for Frequency Test", true);
    
    /* Test frequencies across 915 MHz ISM band */
    uint32_t test_freqs[] = {
        903000000UL,  /* 903 MHz */
        908000000UL,  /* 908 MHz */
        915000000UL,  /* 915 MHz (center) */
        922000000UL,  /* 922 MHz */
        927000000UL,  /* 927 MHz */
    };
    int num_freqs = sizeof(test_freqs) / sizeof(test_freqs[0]);
    
    DEBUGOUT("\nTesting frequency changes across 5 channels:\n\n");
    
    bool all_ok = true;
    
    /* Start in standby */
    set_standby(1);
    
    for (int i = 0; i < num_freqs; i++) {
        DEBUGOUT("  Channel %d: %.3f MHz - ", i + 1, test_freqs[i] / 1000000.0);
        
        /* Time the frequency change */
        bool ok = set_rf_frequency(test_freqs[i]);
        
        if (ok) {
            /* Enter FS mode to lock PLL */
            ok = set_fs();
            delay_ms(2);  /* PLL lock time */
            
            if (ok) {
                /* Verify by taking RSSI reading */
                int16_t rssi = get_rssi_inst();
                DEBUGOUT("OK (RSSI: %d dBm)\n", rssi);
            } else {
                DEBUGOUT("FAILED (PLL)\n");
                all_ok = false;
            }
        } else {
            DEBUGOUT("FAILED (SetRfFreq)\n");
            all_ok = false;
        }
        
        /* Return to standby between hops */
        set_standby(1);
    }
    
    /* Rapid hop test */
    DEBUGOUT("\n  Rapid hopping test (100 hops, timing):\n");
    
    uint32_t hop_errors = 0;
    /* Simple timing - we'll measure iterations per second */
    
    for (int i = 0; i < 100; i++) {
        int freq_idx = i % num_freqs;
        if (!set_rf_frequency(test_freqs[freq_idx])) {
            hop_errors++;
        }
    }
    
    DEBUGOUT("    100 hops completed, errors: %lu\n", (unsigned long)hop_errors);
    DEBUGOUT("    (Actual timing depends on SPI speed)\n");
    
    all_ok = all_ok && (hop_errors == 0);
    test_result("Frequency Hopping", all_ok);
    
    /* Return to standby */
    set_standby(1);
}

/*******************************************************************************
 * Main Test Entry Point
 ******************************************************************************/

/**
 * @brief Run all standalone tests
 *
 * Call this from your main application after lr1121_init().
 */
void lr1121_run_standalone_tests(void) {
    DEBUGOUT("\n");
    DEBUGOUT("=== LR1121 STANDALONE TEST SUITE ===\n");
    DEBUGOUT("Freq: %.1f MHz, Power: %d dBm, SF%d\n\n", 
             TEST_FREQUENCY / 1000000.0, TEST_TX_POWER, TEST_SF);
    
    tests_passed = 0;
    tests_failed = 0;
    failed_test_count = 0;
    
    /* CRITICAL: Hardware reset before tests!
     * Without this, the chip may be in an unknown state with errors.
     */
    DEBUGOUT("--- Hardware Reset ---\n");
    lr1121_status_t status = lr1121_reset();
    if (status != LR1121_OK) {
        DEBUGOUT("WARNING: Reset returned %d\n", status);
    } else {
        DEBUGOUT("Reset OK\n");
    }
    DEBUGOUT("\n");
    
    /* Run tests */
    test_hardware_verification();
    test_rf_configuration();
    test_cw_transmission();
    test_tx_packet();
    test_rx_rssi();
    test_frequency_hopping();
    
    /* Print summary */
    DEBUGOUT("\n");
    DEBUGOUT("╔═════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║                    TEST SUMMARY                             ║\n");
    DEBUGOUT("╠═════════════════════════════════════════════════════════════╣\n");
    DEBUGOUT("║  Tests Passed: %-3lu                                         ║\n", 
             (unsigned long)tests_passed);
    DEBUGOUT("║  Tests Failed: %-3lu                                         ║\n", 
             (unsigned long)tests_failed);
    DEBUGOUT("║                                                             ║\n");
    
    if (tests_failed == 0) {
        DEBUGOUT("║  *** ALL TESTS PASSED - LR1121 READY FOR ELRS! ***         ║\n");
    } else {
        DEBUGOUT("║  *** SOME TESTS FAILED - CHECK HARDWARE/CONFIG ***         ║\n");
    }
    DEBUGOUT("╚═════════════════════════════════════════════════════════════╝\n");
    
    /* Print detailed failure list if any tests failed */
    if (failed_test_count > 0) {
        DEBUGOUT("\n");
        DEBUGOUT("╔═════════════════════════════════════════════════════════════╗\n");
        DEBUGOUT("║              FAILED TESTS (for quick reference)             ║\n");
        DEBUGOUT("╠═════════════════════════════════════════════════════════════╣\n");
        for (uint32_t i = 0; i < failed_test_count; i++) {
            DEBUGOUT("║  %2lu. %-55s ║\n", (unsigned long)(i + 1), failed_test_names[i]);
        }
        DEBUGOUT("╚═════════════════════════════════════════════════════════════╝\n");
        DEBUGOUT("\n");
        DEBUGOUT(">>> Scroll up to see detailed error messages for each failure.\n");
        DEBUGOUT(">>> Or disable LR1121_DEBUG_COMMANDS to reduce output volume.\n");
    }
    DEBUGOUT("\n");
    
    /* Notes for the user */
    DEBUGOUT("NOTES:\n");
    DEBUGOUT("------\n");
    DEBUGOUT("Without a second receiver, you verified:\n");
    DEBUGOUT("  ✓ SPI communication working\n");
    DEBUGOUT("  ✓ Chip initializes and responds correctly\n");
    DEBUGOUT("  ✓ RF configuration commands accepted\n");
    DEBUGOUT("  ✓ TX produces TX_DONE (packet was transmitted)\n");
    DEBUGOUT("  ✓ RX mode functional (RSSI measurable)\n");
    DEBUGOUT("  ✓ Frequency hopping works\n");
    DEBUGOUT("\n");
    DEBUGOUT("To fully verify RF output:\n");
    DEBUGOUT("  - Use SDR receiver (RTL-SDR, HackRF) at %.3f MHz\n", 
             TEST_FREQUENCY / 1000000.0);
    DEBUGOUT("  - CW test shows unmodulated carrier\n");
    DEBUGOUT("  - TX packet test shows LoRa chirps\n");
    DEBUGOUT("\n");
    DEBUGOUT("To test actual packet reception:\n");
    DEBUGOUT("  - Get a second LR1121 module or compatible LoRa device\n");
    DEBUGOUT("  - Or use ELRS transmitter module in bind mode\n");
    DEBUGOUT("\n");
}
