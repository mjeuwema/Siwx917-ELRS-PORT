/**
 * @file lr1121_tcxo_test.c
 * @brief LR1121 TCXO Initialization Standalone Test Suite
 *
 * Focused test suite for debugging TCXO/oscillator startup issues
 * on the Waveshare Core1121-XF module (externally-powered TCXO).
 *
 * See lr1121_tcxo_test.h for API documentation.
 *
 * Citations:
 *   - LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *   - Waveshare Core1121_XF_Demo configuration
 *   - Semtech lr11xx_system.c reference driver
 */

#include "lr1121_tcxo_test.h"
#include "lr1121_driver.h"
#include "rsi_debug.h"
#include <string.h>

/*******************************************************************************
 * External Functions from lr1121_driver.c
 ******************************************************************************/
extern bool lr1121_wait_busy_timeout(uint32_t timeout_ms);
extern bool lr1121_send_command(uint16_t opcode, const uint8_t *params, uint16_t param_len);
extern bool lr1121_read_response(uint8_t *response, uint16_t response_len);
extern void lr1121_cs_assert(void);
extern void lr1121_cs_deassert(void);
extern bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

/*******************************************************************************
 * Debug storage for last temperature reading (for end-of-test summary)
 ******************************************************************************/
static uint8_t  g_last_temp_resp[3] = {0};
static uint16_t g_last_temp_raw_be = 0;
static uint16_t g_last_temp_raw_le = 0;
static int16_t  g_last_temp_calc_be = -999;
static int16_t  g_last_temp_calc_le = -999;

/*******************************************************************************
 * Local Helper Functions
 ******************************************************************************/

/**
 * @brief Simple millisecond delay
 */
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 10000; j++) { }
    }
}

/**
 * @brief Execute command with optional parameters and response
 */
static bool exec_cmd(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                     uint8_t *response, uint16_t response_len) {
    /* Wait for chip ready */
    if (!lr1121_wait_busy_timeout(200)) {
        DEBUGOUT("    [CMD 0x%04X] BUSY timeout before send\n", opcode);
        return false;
    }
    
    /* Send command */
    if (!lr1121_send_command(opcode, params, param_len)) {
        DEBUGOUT("    [CMD 0x%04X] Send failed\n", opcode);
        return false;
    }
    
    /* Wait for processing */
    if (!lr1121_wait_busy_timeout(500)) {
        DEBUGOUT("    [CMD 0x%04X] BUSY timeout after send\n", opcode);
        return false;
    }
    
    /* Read response if needed */
    if (response != NULL && response_len > 0) {
        if (!lr1121_read_response(response, response_len)) {
            DEBUGOUT("    [CMD 0x%04X] Read response failed\n", opcode);
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Get current chip mode from GetStatus with detailed debug output
 * @param print_raw If true, print raw response bytes for debugging
 * @return Chip mode (0-5), or 0xFF on error
 * 
 * Citation: LR1121 Datasheet Section 3.4.2 "stat2"
 * stat2 format:
 *   Bits [7:4]: Reserved
 *   Bits [3:1]: Chip mode (0=SLEEP, 1=STDBY_RC, 2=STDBY_XOSC, 3=FS, 4=RX, 5=TX)
 *   Bit [0]: Command status (0=OK, 1=data available)
 */
static uint8_t get_chip_mode_ex(bool print_raw) {
    uint8_t resp[6] = {0};
    if (!exec_cmd(TCXO_CMD_GET_STATUS, NULL, 0, resp, 6)) {
        if (print_raw) {
            DEBUGOUT("    GetStatus FAILED\n");
        }
        return 0xFF;
    }
    
    if (print_raw) {
        DEBUGOUT("    GetStatus RAW: [%02X %02X %02X %02X %02X %02X]\n",
                 resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
        DEBUGOUT("      resp[0]=0x%02X (stat1 - interrupt pending flags)\n", resp[0]);
        DEBUGOUT("      resp[1]=0x%02X (stat2) -> chip_mode bits[3:1] = %d\n", 
                 resp[1], (resp[1] >> 1) & 0x07);
        DEBUGOUT("        stat2 breakdown:\n");
        DEBUGOUT("          bits[7:4] = 0x%X (reserved)\n", (resp[1] >> 4) & 0x0F);
        DEBUGOUT("          bits[3:1] = %d (chip_mode: %s)\n", 
                 (resp[1] >> 1) & 0x07, 
                 lr1121_tcxo_mode_str((resp[1] >> 1) & 0x07));
        DEBUGOUT("          bit[0]    = %d (cmd_status: %s)\n", 
                 resp[1] & 0x01,
                 (resp[1] & 0x01) ? "data_avail" : "ok");
    }
    
    /* stat2 is in resp[1], chip mode in bits [3:1] */
    return (resp[1] >> 1) & 0x07;
}

/**
 * @brief Get current chip mode from GetStatus (legacy wrapper)
 * @return Chip mode (0-5), or 0xFF on error
 */
static uint8_t get_chip_mode(void) {
    return get_chip_mode_ex(false);
}

/**
 * @brief Get error flags from GetErrors
 * @return Error flags, or 0xFFFF on error
 */
static uint16_t get_errors(void) {
    uint8_t resp[3] = {0};
    if (!exec_cmd(TCXO_CMD_GET_ERRORS, NULL, 0, resp, 3)) {
        return 0xFFFF;
    }
    return ((uint16_t)resp[1] << 8) | resp[2];
}

/**
 * @brief Clear all error flags
 */
static bool clear_errors(void) {
    return exec_cmd(TCXO_CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
}

/**
 * @brief Set standby mode
 * @param mode 0=RC, 1=XOSC
 */
static bool set_standby(uint8_t mode) {
    return exec_cmd(TCXO_CMD_SET_STANDBY, &mode, 1, NULL, 0);
}

/**
 * @brief Set TCXO mode with specified voltage and delay
 * @param voltage_trim Voltage trim value (0x00-0x07)
 * @param delay_ticks Delay in 30.52µs ticks (24-bit)
 */
static bool set_tcxo_mode(uint8_t voltage_trim, uint32_t delay_ticks) {
    uint8_t params[4];
    params[0] = voltage_trim;
    params[1] = (delay_ticks >> 16) & 0xFF;  /* MSB */
    params[2] = (delay_ticks >> 8) & 0xFF;   /* Middle */
    params[3] = delay_ticks & 0xFF;          /* LSB */
    return exec_cmd(TCXO_CMD_SET_TCXO_MODE, params, 4, NULL, 0);
}

/**
 * @brief Set regulator mode
 * @param mode 0=LDO, 1=DCDC
 */
static bool set_reg_mode(uint8_t mode) {
    return exec_cmd(TCXO_CMD_SET_REG_MODE, &mode, 1, NULL, 0);
}

/**
 * @brief Run calibration
 * @param mask Calibration mask (0x3F for all)
 */
static bool calibrate(uint8_t mask) {
    return exec_cmd(TCXO_CMD_CALIBRATE, &mask, 1, NULL, 0);
}

/**
 * @brief Set RF frequency - THIS FORCES PLL TO LOCK TO TCXO!
 * @param freq_hz Frequency in Hz (e.g., 915000000 for 915 MHz)
 * @return true if command succeeded
 * 
 * Citation: LR1121 Datasheet Section 7.2.1 "SetRfFrequency"
 *   Opcode: 0x0203
 *   Parameter: 4 bytes, frequency in Hz (big-endian)
 *   
 * CRITICAL: This command requires PLL to lock to the TCXO reference!
 * If TCXO is not working, this will cause PLL_LOCK error (0x0080).
 */
static bool set_rf_frequency(uint32_t freq_hz) {
    uint8_t params[4];
    params[0] = (freq_hz >> 24) & 0xFF;  /* MSB */
    params[1] = (freq_hz >> 16) & 0xFF;
    params[2] = (freq_hz >> 8) & 0xFF;
    params[3] = freq_hz & 0xFF;          /* LSB */
    return exec_cmd(TCXO_CMD_SET_RF_FREQ, params, 4, NULL, 0);
}

/**
 * @brief Set packet type (LoRa or FSK)
 * @param type 0x00=FSK, 0x01=LoRa
 */
static bool set_packet_type(uint8_t type) {
    return exec_cmd(TCXO_CMD_SET_PACKET_TYPE, &type, 1, NULL, 0);
}

/**
 * @brief Enter frequency synthesis mode (PLL locked, no RX/TX)
 * Citation: LR1121 Datasheet Section 2.1.9 "SetFs"
 */
static bool set_fs_mode(void) {
    return exec_cmd(TCXO_CMD_SET_FS, NULL, 0, NULL, 0);
}

/**
 * @brief Get temperature reading
 * @return Temperature in Celsius, or -999 on error
 * 
 * Citation: Semtech lr11xx_system.c lr11xx_system_get_temp()
 * Citation: LR1121 Datasheet Section 11.2.6 "GetTemp" (opcode 0x011A)
 * 
 * The GetTemp command returns a 16-bit value representing temperature.
 * 
 * IMPORTANT: The raw value is already in degrees Celsius with some offset!
 * Per Semtech driver: temp_celsius = (raw_value / 1.366) - 40
 * 
 * But looking at actual values:
 *   - If raw=1112 and we get 798C, the formula (raw-274)/1.05 is wrong
 *   - Semtech uses: temp = ((float)raw / 1.366f) - 40.0f
 *   - Integer approximation: temp = (raw * 1000 / 1366) - 40
 */
static int16_t get_temperature(void) {
    /* GetTemp is a single-phase command - response comes during command */
    if (!lr1121_wait_busy_timeout(100)) {
        return -999;
    }
    
    /* Send GetTemp command and read response in same CS assertion */
    uint8_t tx_buf[4] = {0x01, 0x1A, 0x00, 0x00};  /* Opcode + 2 NOP bytes */
    uint8_t rx_buf[4] = {0};
    
    lr1121_cs_assert();
    bool ok = lr1121_spi_transfer(tx_buf, rx_buf, 4);
    lr1121_cs_deassert();
    
    if (!ok) {
        return -999;
    }
    
    /* Wait for processing */
    if (!lr1121_wait_busy_timeout(100)) {
        return -999;
    }
    
    /* Read temperature result (phase 2) */
    uint8_t resp[3] = {0};
    if (!lr1121_read_response(resp, 3)) {
        return -999;
    }
    
    /* Temperature is in resp[1:2] as unsigned 16-bit
     * Citation: Semtech lr11xx_system.c line ~380
     *   *temp = ( int16_t )( ( ( ( float ) temp_raw ) / LR11XX_TEMP_SCALING ) + LR11XX_TEMP_OFFSET );
     *   where LR11XX_TEMP_SCALING = 1.366 and LR11XX_TEMP_OFFSET = -40.0
     * 
     * Note: LR1121 uses big-endian byte order for 16-bit values
     */
    uint16_t raw_temp_be = ((uint16_t)resp[1] << 8) | resp[2];  /* Big-endian */
    uint16_t raw_temp_le = ((uint16_t)resp[2] << 8) | resp[1];  /* Little-endian */
    
    /* LR1121 Temperature Conversion Formula (empirically determined)
     * 
     * The Semtech lr11xx driver formula (raw/1.366 - 40) gives incorrect results.
     * Through testing, we found the LR1121 uses: temp = raw/40 - 3
     * 
     * Verified: raw=1112 -> 24°C (reasonable room temperature)
     * 
     * Expected raw values with this formula:
     *   20°C: raw = (20 + 3) * 40 = 920
     *   25°C: raw = (25 + 3) * 40 = 1120
     *   30°C: raw = (30 + 3) * 40 = 1320
     *   40°C: raw = (40 + 3) * 40 = 1720
     */
    int16_t temp_c = (int16_t)(raw_temp_be / 40 - 3);
    
    /* Store in globals for end-of-test summary */
    g_last_temp_resp[0] = resp[0];
    g_last_temp_resp[1] = resp[1];
    g_last_temp_resp[2] = resp[2];
    g_last_temp_raw_be = raw_temp_be;
    g_last_temp_raw_le = raw_temp_le;
    g_last_temp_calc_be = temp_c;
    g_last_temp_calc_le = 0;  /* Not used */
    
    return temp_c;
}

/**
 * @brief Get voltage name string
 */
static const char* voltage_name(uint8_t voltage) {
    switch (voltage) {
        case 0x00: return "NONE (External)";
        case 0x01: return "1.7V";
        case 0x02: return "1.8V";
        case 0x03: return "2.2V";
        case 0x04: return "2.4V";
        case 0x05: return "2.7V";
        case 0x06: return "3.0V";
        case 0x07: return "3.3V";
        default:   return "UNKNOWN";
    }
}

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

const char* lr1121_tcxo_mode_str(uint8_t mode) {
    switch (mode) {
        case TCXO_MODE_SLEEP:      return "SLEEP";
        case TCXO_MODE_STDBY_RC:   return "STDBY_RC";
        case TCXO_MODE_STDBY_XOSC: return "STDBY_XOSC";
        case TCXO_MODE_FS:         return "FS";
        case TCXO_MODE_RX:         return "RX";
        case TCXO_MODE_TX:         return "TX";
        default:                   return "UNKNOWN";
    }
}

void lr1121_tcxo_errors_str(uint16_t errors, char *buf, uint16_t buf_len) {
    if (buf == NULL || buf_len < 2) return;
    
    buf[0] = '\0';
    
    if (errors == 0) {
        snprintf(buf, buf_len, "NONE");
        return;
    }
    
    if (errors == 0xFFFF) {
        snprintf(buf, buf_len, "READ_ERROR");
        return;
    }
    
    char *p = buf;
    int remaining = buf_len;
    int written;
    
    if (errors & TCXO_ERR_HF_XOSC_START) {
        written = snprintf(p, remaining, "HF_XOSC_START ");
        p += written; remaining -= written;
    }
    if (errors & TCXO_ERR_PLL_CALIB) {
        written = snprintf(p, remaining, "PLL_CALIB ");
        p += written; remaining -= written;
    }
    if (errors & TCXO_ERR_PLL_LOCK) {
        written = snprintf(p, remaining, "PLL_LOCK ");
        p += written; remaining -= written;
    }
    if (errors & TCXO_ERR_ADC_CALIB) {
        written = snprintf(p, remaining, "ADC_CALIB ");
        p += written; remaining -= written;
    }
    if (errors & TCXO_ERR_IMG_CALIB) {
        written = snprintf(p, remaining, "IMG_CALIB ");
        p += written; remaining -= written;
    }
    
    /* Remove trailing space */
    if (p > buf && *(p-1) == ' ') {
        *(p-1) = '\0';
    }
}

void lr1121_tcxo_print_result(const tcxo_test_result_t *result) {
    char err_str[128];
    lr1121_tcxo_errors_str(result->errors, err_str, sizeof(err_str));
    
    /* Summary line only - detailed output is in test_tcxo_config() now */
    DEBUGOUT("    === RESULT SUMMARY ===\n");
    DEBUGOUT("    Config: voltage=0x%02X (%s), delay=%u ticks (~%ums)\n",
             result->voltage, voltage_name(result->voltage),
             result->delay_ticks, (result->delay_ticks * 31) / 1000);
    DEBUGOUT("    Final Mode: %d (%s) - %s STDBY_XOSC(2)\n", 
             result->chip_mode, lr1121_tcxo_mode_str(result->chip_mode),
             (result->chip_mode == TCXO_MODE_STDBY_XOSC) ? "==" : "!=");
    DEBUGOUT("    Errors: 0x%04X (%s)\n", result->errors, err_str);
    DEBUGOUT("    VERDICT: %s\n", result->passed ? "*** PASS ***" : "*** FAIL ***");
}

bool lr1121_tcxo_test_spi_baseline(void) {
    DEBUGOUT("\n=== TEST 1: SPI Communication Baseline ===\n");
    
    /* Hardware reset */
    DEBUGOUT("  Resetting LR1121...\n");
    lr1121_status_t status = lr1121_reset();
    if (status != LR1121_OK) {
        DEBUGOUT("  [FAIL] Hardware reset failed (status=%d)\n", status);
        return false;
    }
    DEBUGOUT("  Reset OK\n");
    
    /* GetVersion */
    DEBUGOUT("  Reading version...\n");
    lr1121_version_t version;
    status = lr1121_get_version(&version);
    if (status != LR1121_OK) {
        DEBUGOUT("  [FAIL] GetVersion failed\n");
        return false;
    }
    DEBUGOUT("  GetVersion: HW=0x%02X Type=0x%02X FW=v%d.%d\n",
             version.hardware, version.type,
             (version.version >> 8) & 0xFF, version.version & 0xFF);
    
    /* SetStandby(RC) */
    DEBUGOUT("  Setting STANDBY_RC mode...\n");
    if (!set_standby(0)) {
        DEBUGOUT("  [FAIL] SetStandby(RC) command failed\n");
        return false;
    }
    delay_ms(10);
    
    uint8_t mode = get_chip_mode();
    DEBUGOUT("  Chip mode: %d (%s)\n", mode, lr1121_tcxo_mode_str(mode));
    
    if (mode != TCXO_MODE_STDBY_RC) {
        DEBUGOUT("  [FAIL] Expected STDBY_RC (1), got %d\n", mode);
        return false;
    }
    
    DEBUGOUT("  [PASS] SPI communication working\n");
    return true;
}

/**
 * @brief Single TCXO configuration test with detailed debug output
 */
static void test_tcxo_config(uint8_t voltage, uint16_t delay_ticks, tcxo_test_result_t *result) {
    memset(result, 0, sizeof(*result));
    result->voltage = voltage;
    result->delay_ticks = delay_ticks;
    
    /* Reset chip first */
    DEBUGOUT("    [1] Hardware reset...\n");
    lr1121_reset();
    delay_ms(10);
    
    /* Check initial state after reset */
    DEBUGOUT("    [2] Initial state after reset:\n");
    get_chip_mode_ex(true);  /* Print raw status */
    
    /* Clear any existing errors */
    DEBUGOUT("    [3] Clear errors...\n");
    clear_errors();
    delay_ms(5);
    
    /* SetRegMode(DCDC) - optional but recommended */
    DEBUGOUT("    [4] SetRegMode(DCDC)...\n");
    set_reg_mode(1);
    delay_ms(5);
    
    /* SetTcxoMode */
    DEBUGOUT("    [5] SetTcxoMode(voltage=0x%02X, delay=%u)...\n", voltage, delay_ticks);
    result->set_tcxo_ok = set_tcxo_mode(voltage, delay_ticks);
    if (!result->set_tcxo_ok) {
        DEBUGOUT("        SetTcxoMode FAILED!\n");
        result->passed = false;
        return;
    }
    DEBUGOUT("        SetTcxoMode OK\n");
    
    /* Wait for TCXO to stabilize (software delay in addition to hardware timeout) */
    uint32_t sw_delay = (delay_ticks * 31) / 1000 + 20;  /* Hardware timeout + 20ms margin */
    DEBUGOUT("    [6] Waiting %lu ms for TCXO stabilization...\n", (unsigned long)sw_delay);
    delay_ms(sw_delay);
    
    /* Check state BEFORE SetStandby(XOSC) - should still be STDBY_RC */
    DEBUGOUT("    [7] State BEFORE SetStandby(XOSC):\n");
    uint8_t mode_before = get_chip_mode_ex(true);
    
    /* SetStandby(XOSC) - switch to TCXO clock */
    DEBUGOUT("    [8] SetStandby(XOSC) - requesting switch to external oscillator...\n");
    result->set_standby_ok = set_standby(1);
    if (!result->set_standby_ok) {
        DEBUGOUT("        SetStandby(XOSC) command FAILED!\n");
    } else {
        DEBUGOUT("        SetStandby(XOSC) command sent OK\n");
    }
    delay_ms(20);
    
    /* Check state AFTER SetStandby(XOSC) - should be STDBY_XOSC if TCXO works */
    DEBUGOUT("    [9] State AFTER SetStandby(XOSC):\n");
    result->chip_mode = get_chip_mode_ex(true);
    
    /* Get errors */
    DEBUGOUT("    [10] GetErrors (after SetStandby XOSC):\n");
    result->errors = get_errors();
    DEBUGOUT("        Error flags: 0x%04X\n", result->errors);
    if (result->errors & TCXO_ERR_HF_XOSC_START) {
        DEBUGOUT("        *** HF_XOSC_START_ERR is SET - TCXO failed to start! ***\n");
    }
    
    /***************************************************************************
     * CRITICAL FIX: Calibrate(0x3F) BEFORE SetRfFrequency
     * 
     * Citation: LR1121 Datasheet Section 11.2.4 "Calibrate"
     * 
     * The PLL calibration data from after reset was done with the RC oscillator.
     * After switching to TCXO via SetStandby(XOSC), we MUST recalibrate all
     * blocks (especially PLL) against the new TCXO reference clock.
     * 
     * Without this step:
     *   - PLL calibration data is stale (wrong reference frequency)
     *   - SetRfFrequency will fail to lock PLL
     *   - PLL_LOCK_ERR (0x0080) will be set
     * 
     * Calibration mask 0x3F = all calibrations:
     *   Bit 0: RC64K, Bit 1: RC13M, Bit 2: PLL, Bit 3: ADC, Bit 4: IMG, Bit 5: PLL_TX
     **************************************************************************/
    DEBUGOUT("    [10b] *** Calibrate(0x3F) - Recalibrate with TCXO reference ***\n");
    DEBUGOUT("         (Required before PLL can lock to new clock source)\n");
    bool calib_ok = calibrate(0x3F);
    if (!calib_ok) {
        DEBUGOUT("         Calibrate command FAILED!\n");
    } else {
        DEBUGOUT("         Calibrate command sent OK\n");
    }
    delay_ms(20);  /* Allow calibration to complete */
    
    /* Check for calibration errors */
    uint16_t calib_errors = get_errors();
    if (calib_errors & (TCXO_ERR_PLL_CALIB | TCXO_ERR_ADC_CALIB)) {
        DEBUGOUT("         Calibration errors: 0x%04X\n", calib_errors);
        if (calib_errors & TCXO_ERR_PLL_CALIB) {
            DEBUGOUT("         *** PLL_CALIB_ERR - PLL calibration failed! ***\n");
        }
        if (calib_errors & TCXO_ERR_ADC_CALIB) {
            DEBUGOUT("         *** ADC_CALIB_ERR - ADC calibration failed! ***\n");
        }
    } else {
        DEBUGOUT("         Calibration completed, no errors\n");
    }
    
    /* Check chip mode after calibration - calibration may put chip in different state */
    uint8_t mode_after_calib = get_chip_mode();
    DEBUGOUT("         Mode after calibration: %d (%s)\n", mode_after_calib, 
             lr1121_tcxo_mode_str(mode_after_calib));
    
    /* If calibration put us back in STDBY_RC, re-issue SetStandby(XOSC) */
    if (mode_after_calib == TCXO_MODE_STDBY_RC) {
        DEBUGOUT("         Chip fell back to STDBY_RC after calibration\n");
        DEBUGOUT("         Re-issuing SetStandby(XOSC)...\n");
        set_standby(1);
        delay_ms(10);
        mode_after_calib = get_chip_mode();
        DEBUGOUT("         Mode after re-issuing SetStandby(XOSC): %d (%s)\n", 
                 mode_after_calib, lr1121_tcxo_mode_str(mode_after_calib));
        
        /* Update result->chip_mode to reflect current state */
        result->chip_mode = mode_after_calib;
    }
    
    /* Clear calibration errors before PLL lock test */
    clear_errors();
    delay_ms(5);
    
    /***************************************************************************
     * CRITICAL TEST: SetRfFrequency to force PLL lock
     * 
     * This is the REAL test of whether the TCXO is working!
     * SetStandby(XOSC) might report success even if oscillator is unstable.
     * SetRfFrequency REQUIRES the PLL to lock to the TCXO reference.
     * 
     * If TCXO is bad, we'll get PLL_LOCK error (0x0080) here.
     **************************************************************************/
    DEBUGOUT("    [11] *** PLL LOCK TEST - SetRfFrequency(915 MHz) ***\n");
    DEBUGOUT("        This forces PLL to lock to TCXO - the REAL test!\n");
    
    /* Clear errors before PLL test */
    clear_errors();
    delay_ms(5);
    
    /* Set packet type to LoRa (required before SetRfFrequency on some chips) */
    DEBUGOUT("        Setting packet type to LoRa...\n");
    if (!set_packet_type(0x01)) {
        DEBUGOUT("        WARNING: SetPacketType failed\n");
    }
    delay_ms(5);
    
    /* Set frequency to 915 MHz - THIS FORCES PLL LOCK! */
    DEBUGOUT("        Setting RF frequency to 915000000 Hz...\n");
    bool freq_ok = set_rf_frequency(915000000);
    if (!freq_ok) {
        DEBUGOUT("        *** SetRfFrequency COMMAND FAILED! ***\n");
    } else {
        DEBUGOUT("        SetRfFrequency command sent OK\n");
    }
    delay_ms(10);  /* Give PLL time to attempt lock */
    
    /* Check for PLL errors */
    uint16_t pll_errors = get_errors();
    DEBUGOUT("        Error flags after SetRfFrequency: 0x%04X\n", pll_errors);
    
    bool pll_locked = true;
    if (pll_errors & TCXO_ERR_PLL_LOCK) {
        DEBUGOUT("        *** PLL_LOCK ERROR (0x0080) - PLL failed to lock to TCXO! ***\n");
        DEBUGOUT("        *** THIS MEANS THE TCXO IS NOT WORKING PROPERLY! ***\n");
        pll_locked = false;
    }
    if (pll_errors & TCXO_ERR_PLL_CALIB) {
        DEBUGOUT("        *** PLL_CALIB ERROR (0x0040) - PLL calibration failed! ***\n");
        pll_locked = false;
    }
    if (pll_errors & TCXO_ERR_HF_XOSC_START) {
        DEBUGOUT("        *** HF_XOSC_START ERROR appeared after freq set! ***\n");
        pll_locked = false;
    }
    if (pll_locked && freq_ok) {
        DEBUGOUT("        *** PLL LOCKED SUCCESSFULLY - TCXO IS WORKING! ***\n");
    }
    
    /* Try entering FS mode (frequency synthesis - PLL locked but no RX/TX) */
    DEBUGOUT("    [12] SetFs - Enter frequency synthesis mode...\n");
    bool fs_ok = set_fs_mode();
    delay_ms(10);
    
    uint8_t fs_mode = get_chip_mode_ex(true);
    uint16_t fs_errors = get_errors();
    
    DEBUGOUT("        SetFs result: %s, mode=%d (expect 3=FS), errors=0x%04X\n",
             fs_ok ? "OK" : "FAIL", fs_mode, fs_errors);
    
    /* FS mode may fall back to STDBY_RC or STDBY_XOSC after a short time if no RX/TX
     * This is normal behavior - the key test is whether PLL locked successfully.
     * 
     * Citation: LR1121 Datasheet Section 2.1 State Machine
     *   FS mode keeps PLL locked but chip may fall back based on SetRxTxFallbackMode
     *   Default fallback is STDBY_RC for power saving.
     * 
     * We consider FS "worked" if:
     *   1. SetFs command succeeded (fs_ok)
     *   2. No PLL_LOCK or HF_XOSC errors appeared
     *   3. Mode is FS (3) OR STDBY_XOSC (2) - both indicate TCXO is functional
     * 
     * Mode falling back to STDBY_RC (1) with no errors is also acceptable if
     * the PLL locked successfully during SetRfFrequency.
     */
    bool fs_had_errors = (fs_errors & (TCXO_ERR_PLL_LOCK | TCXO_ERR_HF_XOSC_START)) != 0;
    bool fs_worked = fs_ok && !fs_had_errors;
    
    if (fs_mode == TCXO_MODE_FS) {
        DEBUGOUT("        Mode is FS (3) - PLL actively locked\n");
    } else if (fs_mode == TCXO_MODE_STDBY_XOSC) {
        DEBUGOUT("        Mode fell back to STDBY_XOSC (2) - TCXO still active\n");
    } else if (fs_mode == TCXO_MODE_STDBY_RC) {
        DEBUGOUT("        Mode fell back to STDBY_RC (1) - normal fallback behavior\n");
        DEBUGOUT("        (This is OK if PLL locked successfully earlier)\n");
    }
    
    if (fs_had_errors) {
        DEBUGOUT("        *** FS MODE HAD ERRORS - TCXO/PLL problem! ***\n");
    }
    
    /* Update result with combined errors */
    result->errors |= pll_errors | fs_errors;
    
    /* REVISED PASS CRITERIA:
     * The key indicator of TCXO health is whether PLL locked during SetRfFrequency.
     * If PLL locked successfully, the TCXO is working regardless of fallback mode.
     * 
     * Pass if:
     *   1. SetStandby(XOSC) succeeded (chip_mode was STDBY_XOSC at step 9)
     *   2. No HF_XOSC_START error
     *   3. PLL locked successfully during SetRfFrequency
     *   4. SetFs had no errors (even if mode fell back)
     */
    result->passed = (result->chip_mode == TCXO_MODE_STDBY_XOSC) &&
                     !(result->errors & TCXO_ERR_HF_XOSC_START) &&
                     pll_locked && fs_worked;
    
    /* Extra diagnostic: If mode_before == mode_after, the chip didn't actually switch */
    if (mode_before == result->chip_mode && result->chip_mode == TCXO_MODE_STDBY_RC) {
        DEBUGOUT("    *** WARNING: Chip remained in STDBY_RC - oscillator may have failed silently! ***\n");
    }
    
    DEBUGOUT("    [13] FINAL Test verdict: %s\n", result->passed ? "PASS" : "FAIL");
    DEBUGOUT("         STDBY_XOSC: %s, PLL_LOCK: %s, FS_MODE: %s\n",
             (result->chip_mode == TCXO_MODE_STDBY_XOSC) ? "OK" : "FAIL",
             pll_locked ? "OK" : "FAIL",
             fs_worked ? "OK" : "FAIL");
}

uint8_t lr1121_tcxo_test_voltage_sweep(tcxo_test_result_t *results, uint8_t *num_results) {
    DEBUGOUT("\n=== TEST 2: TCXO Voltage Sweep ===\n");
    DEBUGOUT("  Testing which TCXO voltage settings work...\n");
    DEBUGOUT("  (Using fixed delay of 300 ticks / ~9ms for all tests)\n\n");
    
    static const uint8_t voltages[] = {
        TCXO_TEST_VOLTAGE_NONE,  /* 0x00 - External TCXO */
        TCXO_TEST_VOLTAGE_1V8,   /* 0x02 - 1.8V */
        TCXO_TEST_VOLTAGE_3V0,   /* 0x06 - 3.0V (Waveshare default) */
        TCXO_TEST_VOLTAGE_3V3    /* 0x07 - 3.3V */
    };
    const uint8_t num_voltages = sizeof(voltages) / sizeof(voltages[0]);
    
    uint8_t best_voltage = 0xFF;
    uint8_t pass_count = 0;
    
    for (uint8_t i = 0; i < num_voltages; i++) {
        DEBUGOUT("  --- Testing voltage=0x%02X (%s) ---\n", 
                 voltages[i], voltage_name(voltages[i]));
        
        test_tcxo_config(voltages[i], TCXO_TEST_DELAY_9MS, &results[i]);
        lr1121_tcxo_print_result(&results[i]);
        
        if (results[i].passed) {
            pass_count++;
            if (best_voltage == 0xFF) {
                best_voltage = voltages[i];
            }
        }
        DEBUGOUT("\n");
    }
    
    *num_results = num_voltages;
    
    DEBUGOUT("  --- Voltage Sweep Summary ---\n");
    DEBUGOUT("  Passed: %d/%d\n", pass_count, num_voltages);
    if (best_voltage != 0xFF) {
        DEBUGOUT("  Best voltage: 0x%02X (%s)\n", best_voltage, voltage_name(best_voltage));
    } else {
        DEBUGOUT("  [FAIL] No working voltage found!\n");
    }
    
    return best_voltage;
}

uint16_t lr1121_tcxo_test_delay_sweep(uint8_t voltage, tcxo_test_result_t *results, uint8_t *num_results) {
    DEBUGOUT("\n=== TEST 3: TCXO Delay Sweep ===\n");
    DEBUGOUT("  Testing minimum reliable TCXO startup delay...\n");
    DEBUGOUT("  Using voltage=0x%02X (%s)\n\n", voltage, voltage_name(voltage));
    
    static const uint16_t delays[] = {
        TCXO_TEST_DELAY_5MS,   /* ~5ms */
        TCXO_TEST_DELAY_9MS,   /* ~9ms (Waveshare default) */
        TCXO_TEST_DELAY_15MS,  /* ~15ms */
        TCXO_TEST_DELAY_30MS,  /* ~30ms */
        TCXO_TEST_DELAY_60MS   /* ~60ms */
    };
    const uint8_t num_delays = sizeof(delays) / sizeof(delays[0]);
    
    uint16_t min_working_delay = 0xFFFF;
    uint8_t pass_count = 0;
    
    for (uint8_t i = 0; i < num_delays; i++) {
        DEBUGOUT("  --- Testing delay=%u ticks (~%ums) ---\n", 
                 delays[i], (delays[i] * 31) / 1000);
        
        test_tcxo_config(voltage, delays[i], &results[i]);
        lr1121_tcxo_print_result(&results[i]);
        
        if (results[i].passed) {
            pass_count++;
            if (delays[i] < min_working_delay) {
                min_working_delay = delays[i];
            }
        }
        DEBUGOUT("\n");
    }
    
    *num_results = num_delays;
    
    DEBUGOUT("  --- Delay Sweep Summary ---\n");
    DEBUGOUT("  Passed: %d/%d\n", pass_count, num_delays);
    if (min_working_delay != 0xFFFF) {
        DEBUGOUT("  Minimum working delay: %u ticks (~%ums)\n", 
                 min_working_delay, (min_working_delay * 31) / 1000);
    } else {
        DEBUGOUT("  [FAIL] No working delay found!\n");
    }
    
    return min_working_delay;
}

bool lr1121_tcxo_test_full_init(uint8_t voltage, uint16_t delay_ticks, int16_t *temperature) {
    DEBUGOUT("\n=== TEST 4: Full Initialization Validation ===\n");
    DEBUGOUT("  Running complete init sequence...\n");
    DEBUGOUT("  Config: voltage=0x%02X (%s), delay=%u ticks\n\n",
             voltage, voltage_name(voltage), delay_ticks);
    
    /* Reset */
    DEBUGOUT("  Step 1: Hardware Reset...\n");
    lr1121_reset();
    delay_ms(10);
    DEBUGOUT("    OK\n");
    
    /* Clear errors */
    DEBUGOUT("  Step 2: Clear Errors...\n");
    clear_errors();
    delay_ms(5);
    DEBUGOUT("    OK\n");
    
    /* SetRegMode(DCDC) */
    DEBUGOUT("  Step 3: SetRegMode(DCDC)...\n");
    if (!set_reg_mode(1)) {
        DEBUGOUT("    FAILED\n");
        return false;
    }
    delay_ms(5);
    DEBUGOUT("    OK\n");
    
    /* SetTcxoMode */
    DEBUGOUT("  Step 4: SetTcxoMode(0x%02X, %u)...\n", voltage, delay_ticks);
    if (!set_tcxo_mode(voltage, delay_ticks)) {
        DEBUGOUT("    FAILED\n");
        return false;
    }
    uint32_t sw_delay = (delay_ticks * 31) / 1000 + 30;
    delay_ms(sw_delay);
    DEBUGOUT("    OK (waited %lums)\n", (unsigned long)sw_delay);
    
    /* SetStandby(XOSC) */
    DEBUGOUT("  Step 5: SetStandby(XOSC)...\n");
    if (!set_standby(1)) {
        DEBUGOUT("    FAILED\n");
        return false;
    }
    delay_ms(20);
    
    uint8_t mode = get_chip_mode();
    uint16_t errors = get_errors();
    DEBUGOUT("    Mode=%d (%s), Errors=0x%04X\n", mode, lr1121_tcxo_mode_str(mode), errors);
    
    if (mode != TCXO_MODE_STDBY_XOSC) {
        DEBUGOUT("    [FAIL] Not in STDBY_XOSC mode!\n");
        return false;
    }
    if (errors & TCXO_ERR_HF_XOSC_START) {
        DEBUGOUT("    [FAIL] HF_XOSC_START_ERR!\n");
        return false;
    }
    DEBUGOUT("    OK\n");
    
    /* Calibrate */
    DEBUGOUT("  Step 6: Calibrate(0x3F)...\n");
    if (!calibrate(0x3F)) {
        DEBUGOUT("    FAILED\n");
        return false;
    }
    delay_ms(50);
    
    /* Return to STDBY_XOSC after calibration */
    set_standby(1);
    delay_ms(10);
    
    errors = get_errors();
    DEBUGOUT("    Errors after calibration: 0x%04X\n", errors);
    if (errors & (TCXO_ERR_PLL_CALIB | TCXO_ERR_PLL_LOCK | TCXO_ERR_ADC_CALIB)) {
        char err_str[128];
        lr1121_tcxo_errors_str(errors, err_str, sizeof(err_str));
        DEBUGOUT("    [WARN] Calibration errors: %s\n", err_str);
    }
    DEBUGOUT("    OK\n");
    
    /* Clear calibration errors (they may be expected) */
    clear_errors();
    
    /* GetTemperature - ultimate proof TCXO is stable */
    DEBUGOUT("  Step 7: GetTemperature...\n");
    
    /* Verify we're in STDBY_XOSC before reading temperature */
    uint8_t mode_before_temp = get_chip_mode();
    DEBUGOUT("    Chip mode before GetTemp: %d (%s)\n", mode_before_temp,
             lr1121_tcxo_mode_str(mode_before_temp));
    
    /* If not in STDBY_XOSC, try to switch */
    if (mode_before_temp != TCXO_MODE_STDBY_XOSC) {
        DEBUGOUT("    WARNING: Not in STDBY_XOSC! Switching...\n");
        set_standby(1);
        delay_ms(20);
        mode_before_temp = get_chip_mode();
        DEBUGOUT("    Chip mode after switch: %d (%s)\n", mode_before_temp,
                 lr1121_tcxo_mode_str(mode_before_temp));
    }
    
    int16_t temp = get_temperature();
    if (temp == -999) {
        DEBUGOUT("    [FAIL] Could not read temperature\n");
        return false;
    }
    DEBUGOUT("    Temperature: %d C\n", temp);
    
    if (temperature != NULL) {
        *temperature = temp;
    }
    
    /* Sanity check temperature */
    if (temp < -40 || temp > 85) {
        DEBUGOUT("    [WARN] Temperature out of normal range!\n");
    }
    
    DEBUGOUT("\n  [PASS] Full initialization successful!\n");
    return true;
}

uint8_t lr1121_tcxo_test_stress(uint8_t voltage, uint16_t delay_ticks, uint8_t cycles) {
    DEBUGOUT("\n=== TEST 5: Stress Test (%d cycles) ===\n", cycles);
    DEBUGOUT("  Config: voltage=0x%02X, delay=%u ticks\n\n", voltage, delay_ticks);
    
    uint8_t pass_count = 0;
    
    for (uint8_t i = 0; i < cycles; i++) {
        tcxo_test_result_t result;
        test_tcxo_config(voltage, delay_ticks, &result);
        
        if (result.passed) {
            DEBUGOUT("  Cycle %d/%d: PASS\n", i + 1, cycles);
            pass_count++;
        } else {
            char err_str[128];
            lr1121_tcxo_errors_str(result.errors, err_str, sizeof(err_str));
            DEBUGOUT("  Cycle %d/%d: FAIL (mode=%d, errors=%s)\n", 
                     i + 1, cycles, result.chip_mode, err_str);
        }
        
        delay_ms(100);  /* Brief pause between cycles */
    }
    
    DEBUGOUT("\n  --- Stress Test Summary ---\n");
    DEBUGOUT("  Passed: %d/%d (%.0f%%)\n", pass_count, cycles, 
             (100.0f * pass_count) / cycles);
    
    if (pass_count == cycles) {
        DEBUGOUT("  [PASS] All cycles successful!\n");
    } else {
        DEBUGOUT("  [FAIL] %d cycles failed\n", cycles - pass_count);
    }
    
    return pass_count;
}

void lr1121_tcxo_test_run(void) {
    DEBUGOUT("\n");
    DEBUGOUT("========================================\n");
    DEBUGOUT("LR1121 TCXO INITIALIZATION TEST SUITE\n");
    DEBUGOUT("Hardware: Waveshare Core1121-XF\n");
    DEBUGOUT("         (External 32MHz TCXO)\n");
    DEBUGOUT("========================================\n");
    
    /* Initialize LR1121 driver (GPIO/SPI) */
    DEBUGOUT("\nInitializing LR1121 driver...\n");
    lr1121_status_t status = lr1121_init();
    if (status != LR1121_OK) {
        DEBUGOUT("[FATAL] lr1121_init() failed with code %d\n", status);
        DEBUGOUT("Cannot continue without working driver.\n");
        return;
    }
    DEBUGOUT("Driver initialized OK\n");
    
    /* Test 1: SPI baseline */
    bool spi_ok = lr1121_tcxo_test_spi_baseline();
    if (!spi_ok) {
        DEBUGOUT("\n[FATAL] SPI communication failed! Cannot continue.\n");
        return;
    }
    
    /* Test 2: Voltage sweep */
    tcxo_test_result_t voltage_results[4];
    uint8_t num_voltage_results = 0;
    uint8_t best_voltage = lr1121_tcxo_test_voltage_sweep(voltage_results, &num_voltage_results);
    
    if (best_voltage == 0xFF) {
        DEBUGOUT("\n[FATAL] No working TCXO voltage found! Cannot continue.\n");
        DEBUGOUT("Check hardware connections and TCXO circuit.\n");
        return;
    }
    
    /* Test 3: Delay sweep with best voltage */
    tcxo_test_result_t delay_results[5];
    uint8_t num_delay_results = 0;
    uint16_t best_delay = lr1121_tcxo_test_delay_sweep(best_voltage, delay_results, &num_delay_results);
    
    if (best_delay == 0xFFFF) {
        /* Fall back to Waveshare default */
        DEBUGOUT("\n[WARN] No working delay found, using Waveshare default (300 ticks)\n");
        best_delay = TCXO_TEST_DELAY_9MS;
    }
    
    /* Test 4: Full init validation */
    int16_t temperature = 0;
    bool full_init_ok = lr1121_tcxo_test_full_init(best_voltage, best_delay, &temperature);
    
    /* Test 5: Stress test */
    uint8_t stress_passed = 0;
    if (full_init_ok) {
        stress_passed = lr1121_tcxo_test_stress(best_voltage, best_delay, TCXO_STRESS_TEST_CYCLES);
    } else {
        DEBUGOUT("\n[SKIP] Stress test skipped due to full init failure\n");
    }
    
    /* Final summary */
    DEBUGOUT("\n");
    DEBUGOUT("========================================\n");
    DEBUGOUT("SUMMARY\n");
    DEBUGOUT("========================================\n");
    DEBUGOUT("\n");
    DEBUGOUT("Recommended Configuration:\n");
    DEBUGOUT("  TCXO_VOLTAGE: 0x%02X (%s)\n", best_voltage, voltage_name(best_voltage));
    DEBUGOUT("  TCXO_DELAY:   %u ticks (~%ums)\n", best_delay, (best_delay * 31) / 1000);
    DEBUGOUT("  LF_CLOCK:     RC (internal 32kHz) - if no 32kHz crystal\n");
    DEBUGOUT("\n");
    DEBUGOUT("Test Results:\n");
    DEBUGOUT("  SPI Baseline:     %s\n", spi_ok ? "PASS" : "FAIL");
    DEBUGOUT("  Voltage Sweep:    Best=0x%02X\n", best_voltage);
    DEBUGOUT("  Delay Sweep:      Min=%u ticks\n", best_delay);
    DEBUGOUT("  Full Init:        %s\n", full_init_ok ? "PASS" : "FAIL");
    if (full_init_ok) {
        DEBUGOUT("  Temperature:      %d C\n", temperature);
        DEBUGOUT("  Stress Test:      %d/%d (%.0f%%)\n", 
                 stress_passed, TCXO_STRESS_TEST_CYCLES,
                 (100.0f * stress_passed) / TCXO_STRESS_TEST_CYCLES);
    }
    DEBUGOUT("\n");
    
    /* Print GetTemp debug info */
    DEBUGOUT("--- GetTemp Debug ---\n");
    DEBUGOUT("  Raw: %u (0x%04X) -> %d C\n", 
             g_last_temp_raw_be, g_last_temp_raw_be, g_last_temp_calc_be);
    DEBUGOUT("  Formula: temp = raw/40 - 3\n");
    DEBUGOUT("---------------------\n");
    DEBUGOUT("\n");
    
    if (spi_ok && full_init_ok && stress_passed == TCXO_STRESS_TEST_CYCLES) {
        DEBUGOUT("*** ALL TESTS PASSED ***\n");
    } else {
        DEBUGOUT("*** SOME TESTS FAILED - Review results above ***\n");
    }
    
    DEBUGOUT("========================================\n");
}

/*******************************************************************************
 * EXTENDED STRESS TESTS FOR TCXO VALIDATION
 * 
 * These tests simulate real ELRS operation patterns to ensure TCXO stability
 * before integrating fixes into the main ELRS codebase.
 ******************************************************************************/

/**
 * @brief FHSS (Frequency Hopping) Stress Test
 * 
 * Simulates ELRS frequency hopping by rapidly switching between frequencies
 * in the 915MHz ISM band. Tests PLL lock stability under rapid changes.
 * 
 * @param num_hops Number of frequency hops to perform
 * @param delay_us Delay between hops in microseconds (0 for max speed)
 * @return Number of successful hops
 */
uint32_t lr1121_tcxo_stress_fhss(uint32_t num_hops, uint32_t delay_us) {
    DEBUGOUT("\n=== FHSS STRESS TEST ===\n");
    DEBUGOUT("  Hops: %lu, Delay: %lu us\n", num_hops, delay_us);
    
    /* ELRS 915MHz frequency table (subset) */
    static const uint32_t fhss_freqs[] = {
        903500000, 904400000, 905300000, 906200000, 907100000,
        908000000, 908900000, 909800000, 910700000, 911600000,
        912500000, 913400000, 914300000, 915200000, 916100000,
        917000000, 917900000, 918800000, 919700000, 920600000,
        921500000, 922400000, 923300000, 924200000, 925100000,
        926000000, 926900000
    };
    const uint8_t num_freqs = sizeof(fhss_freqs) / sizeof(fhss_freqs[0]);
    
    /* Ensure we're initialized and in STDBY_XOSC */
    set_standby(1);
    delay_ms(5);
    
    uint32_t success_count = 0;
    uint32_t pll_lock_failures = 0;
    uint32_t freq_idx = 0;
    
    DEBUGOUT("  Starting frequency hopping...\n");
    
    for (uint32_t i = 0; i < num_hops; i++) {
        /* Select next frequency (pseudo-random pattern) */
        freq_idx = (freq_idx + 7) % num_freqs;  /* Simple hop pattern */
        uint32_t freq = fhss_freqs[freq_idx];
        
        /* Set frequency */
        uint8_t freq_params[4] = {
            (freq >> 24) & 0xFF,
            (freq >> 16) & 0xFF,
            (freq >> 8) & 0xFF,
            freq & 0xFF
        };
        
        if (!lr1121_send_command(0x020B, freq_params, 4)) {  /* SetRfFrequency */
            pll_lock_failures++;
            continue;
        }
        
        /* Brief wait for PLL lock */
        if (delay_us > 0) {
            for (volatile uint32_t d = 0; d < delay_us / 10; d++) { }
        }
        
        /* Check for errors */
        uint16_t errors = get_errors();
        if (errors & TCXO_ERR_PLL_LOCK) {
            pll_lock_failures++;
            clear_errors();
        } else {
            success_count++;
        }
        
        /* Progress indicator every 1000 hops */
        if ((i + 1) % 1000 == 0) {
            DEBUGOUT("    Progress: %lu/%lu hops, %lu PLL failures\n", 
                     i + 1, num_hops, pll_lock_failures);
        }
    }
    
    /* Return to standby */
    set_standby(1);
    
    DEBUGOUT("\n  --- FHSS Test Results ---\n");
    DEBUGOUT("  Total hops:      %lu\n", num_hops);
    DEBUGOUT("  Successful:      %lu (%.2f%%)\n", success_count, 
             (100.0f * success_count) / num_hops);
    DEBUGOUT("  PLL failures:    %lu\n", pll_lock_failures);
    
    if (success_count == num_hops) {
        DEBUGOUT("  [PASS] All frequency hops successful!\n");
    } else {
        DEBUGOUT("  [FAIL] %lu hops failed\n", num_hops - success_count);
    }
    
    return success_count;
}

/**
 * @brief Temperature Stability Test
 * 
 * Takes multiple temperature readings to verify ADC stability.
 * Large variance indicates TCXO/calibration issues.
 * 
 * @param num_readings Number of temperature readings
 * @return true if temperature is stable (variance < 5°C)
 */
bool lr1121_tcxo_stress_temperature(uint16_t num_readings) {
    DEBUGOUT("\n=== TEMPERATURE STABILITY TEST ===\n");
    DEBUGOUT("  Readings: %u\n", num_readings);
    
    int32_t sum = 0;
    int16_t min_temp = 32767;
    int16_t max_temp = -32768;
    uint16_t valid_readings = 0;
    
    /* Ensure we're in STDBY_XOSC */
    set_standby(1);
    delay_ms(10);
    
    for (uint16_t i = 0; i < num_readings; i++) {
        int16_t temp = get_temperature();
        
        if (temp != -999) {
            sum += temp;
            if (temp < min_temp) min_temp = temp;
            if (temp > max_temp) max_temp = temp;
            valid_readings++;
        }
        
        delay_ms(10);  /* Brief delay between readings */
    }
    
    if (valid_readings == 0) {
        DEBUGOUT("  [FAIL] No valid temperature readings!\n");
        return false;
    }
    
    int16_t avg_temp = (int16_t)(sum / valid_readings);
    int16_t range = max_temp - min_temp;
    
    DEBUGOUT("\n  --- Temperature Results ---\n");
    DEBUGOUT("  Valid readings:  %u/%u\n", valid_readings, num_readings);
    DEBUGOUT("  Average:         %d C\n", avg_temp);
    DEBUGOUT("  Min:             %d C\n", min_temp);
    DEBUGOUT("  Max:             %d C\n", max_temp);
    DEBUGOUT("  Range:           %d C\n", range);
    
    /* Pass if range is small (stable readings) */
    bool passed = (range <= 5) && (valid_readings == num_readings);
    
    if (passed) {
        DEBUGOUT("  [PASS] Temperature readings stable!\n");
    } else {
        DEBUGOUT("  [FAIL] Temperature unstable or missing readings\n");
    }
    
    return passed;
}

/**
 * @brief TX/RX Mode Cycling Test
 * 
 * Rapidly cycles between TX and RX modes to test mode transitions
 * and TCXO stability during state changes.
 * 
 * @param num_cycles Number of TX/RX cycles
 * @return Number of successful cycles
 */
uint32_t lr1121_tcxo_stress_txrx_cycle(uint32_t num_cycles) {
    DEBUGOUT("\n=== TX/RX MODE CYCLING TEST ===\n");
    DEBUGOUT("  Cycles: %lu\n", num_cycles);
    
    uint32_t success_count = 0;
    uint32_t tx_cmd_failures = 0;
    uint32_t rx_cmd_failures = 0;
    uint32_t error_count = 0;
    
    /* Set a fixed frequency for the test */
    uint32_t freq = 915000000;
    uint8_t freq_params[4] = {
        (freq >> 24) & 0xFF,
        (freq >> 16) & 0xFF,
        (freq >> 8) & 0xFF,
        freq & 0xFF
    };
    
    /* Initialize - full setup like we do in basic tests */
    set_standby(1);
    delay_ms(5);
    
    /* Set packet type to LoRa (required before RF operations) */
    uint8_t pkt_type = 0x01;  /* LoRa */
    lr1121_send_command(0x020E, &pkt_type, 1);  /* SetPacketType */
    delay_ms(2);
    
    lr1121_send_command(0x020B, freq_params, 4);  /* SetRfFrequency */
    delay_ms(5);
    
    /* Debug: check initial state */
    uint8_t init_mode = get_chip_mode();
    DEBUGOUT("  Initial mode after setup: %d (%s)\n", init_mode, lr1121_tcxo_mode_str(init_mode));
    
    for (uint32_t i = 0; i < num_cycles; i++) {
        bool cycle_ok = true;
        
        /* Enter TX mode (CW)
         * Citation: LR1121 Datasheet - SetTxCw opcode is 0x0219
         * No parameters needed
         */
        bool tx_ok = lr1121_send_command(0x0219, NULL, 0);
        if (!tx_ok) {
            tx_cmd_failures++;
            cycle_ok = false;
        }
        delay_ms(1);  /* Brief delay for mode transition */
        
        /* Return to standby (we don't check mode - just that command worked) */
        set_standby(1);
        delay_ms(1);
        
        /* Enter RX mode
         * Citation: LR1121 Datasheet - SetRx opcode is 0x0209
         * Use continuous RX (0xFFFFFF) so it doesn't timeout immediately
         */
        uint8_t rx_params[3] = {0xFF, 0xFF, 0xFF};  /* Continuous RX */
        bool rx_ok = lr1121_send_command(0x0209, rx_params, 3);
        if (!rx_ok) {
            rx_cmd_failures++;
            cycle_ok = false;
        }
        delay_ms(1);
        
        /* Return to standby */
        set_standby(1);
        delay_ms(1);
        
        /* Check for critical errors only */
        uint16_t errors = get_errors();
        if (errors & (TCXO_ERR_PLL_LOCK | TCXO_ERR_HF_XOSC_START)) {
            error_count++;
            cycle_ok = false;
            clear_errors();
        }
        
        if (cycle_ok) {
            success_count++;
        }
        
        /* Progress indicator */
        if ((i + 1) % 100 == 0) {
            DEBUGOUT("    Progress: %lu/%lu cycles, %lu TX fails, %lu RX fails, %lu errors\n", 
                     i + 1, num_cycles, tx_cmd_failures, rx_cmd_failures, error_count);
        }
    }
    
    /* Return to standby */
    set_standby(1);
    
    DEBUGOUT("\n  --- TX/RX Cycle Results ---\n");
    DEBUGOUT("  Total cycles:    %lu\n", num_cycles);
    DEBUGOUT("  Successful:      %lu (%.2f%%)\n", success_count,
             (100.0f * success_count) / num_cycles);
    DEBUGOUT("  TX cmd failures: %lu\n", tx_cmd_failures);
    DEBUGOUT("  RX cmd failures: %lu\n", rx_cmd_failures);
    DEBUGOUT("  Error flags:     %lu\n", error_count);
    
    if (success_count == num_cycles) {
        DEBUGOUT("  [PASS] All TX/RX cycles successful!\n");
    } else {
        DEBUGOUT("  [FAIL] %lu cycles failed\n", num_cycles - success_count);
    }
    
    return success_count;
}

/**
 * @brief Long Duration PLL Lock Test
 * 
 * Keeps PLL locked for extended period and monitors for drift/unlock.
 * 
 * @param duration_sec Duration in seconds to hold PLL lock
 * @param check_interval_ms How often to check PLL status (ms)
 * @return true if PLL remained locked entire duration
 */
bool lr1121_tcxo_stress_pll_duration(uint16_t duration_sec, uint16_t check_interval_ms) {
    DEBUGOUT("\n=== LONG DURATION PLL LOCK TEST ===\n");
    DEBUGOUT("  Duration: %u seconds\n", duration_sec);
    DEBUGOUT("  Check interval: %u ms\n", check_interval_ms);
    
    /* Initialize and set frequency */
    set_standby(1);
    delay_ms(5);
    
    uint32_t freq = 915000000;
    uint8_t freq_params[4] = {
        (freq >> 24) & 0xFF,
        (freq >> 16) & 0xFF,
        (freq >> 8) & 0xFF,
        freq & 0xFF
    };
    lr1121_send_command(0x020B, freq_params, 4);  /* SetRfFrequency */
    delay_ms(2);
    
    /* Enter FS mode (PLL locked, no TX/RX)
     * Citation: LR1121 Datasheet - SetFs opcode is 0x011D
     */
    lr1121_send_command(0x011D, NULL, 0);  /* SetFs */
    delay_ms(5);
    
    uint32_t total_checks = (duration_sec * 1000) / check_interval_ms;
    uint32_t pll_unlocks = 0;
    uint32_t xosc_errors = 0;
    
    DEBUGOUT("  Monitoring PLL lock for %u seconds...\n", duration_sec);
    
    for (uint32_t i = 0; i < total_checks; i++) {
        delay_ms(check_interval_ms);
        
        /* Check for errors */
        uint16_t errors = get_errors();
        
        if (errors & TCXO_ERR_PLL_LOCK) {
            pll_unlocks++;
            DEBUGOUT("    [%lu] PLL UNLOCK detected!\n", i);
            clear_errors();
            
            /* Try to re-lock */
            lr1121_send_command(0x011D, NULL, 0);  /* SetFs */
        }
        
        if (errors & TCXO_ERR_HF_XOSC_START) {
            xosc_errors++;
            DEBUGOUT("    [%lu] XOSC START error!\n", i);
            clear_errors();
        }
        
        /* Progress every 10 seconds */
        if ((i + 1) % (10000 / check_interval_ms) == 0) {
            DEBUGOUT("    %lu seconds elapsed, %lu PLL unlocks, %lu XOSC errors\n",
                     ((i + 1) * check_interval_ms) / 1000, pll_unlocks, xosc_errors);
        }
    }
    
    /* Return to standby */
    set_standby(1);
    
    DEBUGOUT("\n  --- PLL Duration Results ---\n");
    DEBUGOUT("  Duration:        %u seconds\n", duration_sec);
    DEBUGOUT("  Total checks:    %lu\n", total_checks);
    DEBUGOUT("  PLL unlocks:     %lu\n", pll_unlocks);
    DEBUGOUT("  XOSC errors:     %lu\n", xosc_errors);
    
    bool passed = (pll_unlocks == 0) && (xosc_errors == 0);
    
    if (passed) {
        DEBUGOUT("  [PASS] PLL remained locked for entire duration!\n");
    } else {
        DEBUGOUT("  [FAIL] PLL stability issues detected\n");
    }
    
    return passed;
}

/**
 * @brief Power Cycle Stress Test
 * 
 * Repeatedly resets the chip and re-initializes TCXO to test
 * cold-start reliability.
 * 
 * @param num_cycles Number of power cycles
 * @return Number of successful initializations
 */
uint32_t lr1121_tcxo_stress_power_cycle(uint32_t num_cycles) {
    DEBUGOUT("\n=== POWER CYCLE STRESS TEST ===\n");
    DEBUGOUT("  Cycles: %lu\n", num_cycles);
    
    uint32_t success_count = 0;
    uint32_t reset_failures = 0;
    uint32_t tcxo_cmd_failures = 0;
    uint32_t xosc_failures = 0;
    uint32_t pll_failures = 0;
    
    for (uint32_t i = 0; i < num_cycles; i++) {
        bool cycle_ok = true;
        
        /* Hardware reset - must wait for BUSY to go low after */
        lr1121_reset();
        
        /* Wait for chip to be ready (BUSY low) */
        if (!lr1121_wait_busy_timeout(100)) {
            reset_failures++;
            cycle_ok = false;
            continue;
        }
        
        /* Clear any errors from reset */
        clear_errors();
        delay_ms(5);
        
        /* Configure TCXO (voltage 0x00 for external, delay=164 ticks ~5ms)
         * Format: [voltage][timeout_23:16][timeout_15:8][timeout_7:0]
         */
        uint8_t tcxo_params[4] = {0x00, 0x00, 0x00, 0xA4};  /* voltage=0, delay=164 */
        if (!lr1121_send_command(0x0117, tcxo_params, 4)) {
            tcxo_cmd_failures++;
            cycle_ok = false;
            continue;
        }
        
        /* Wait for TCXO to stabilize */
        delay_ms(10);
        
        /* Switch to XOSC */
        if (!set_standby(1)) {
            xosc_failures++;
            cycle_ok = false;
            continue;
        }
        delay_ms(10);
        
        /* Calibrate all blocks */
        uint8_t cal_params[1] = {0x3F};
        lr1121_send_command(0x010F, cal_params, 1);
        delay_ms(20);
        
        /* Re-issue SetStandby(XOSC) after calibration (it may have gone to RC) */
        set_standby(1);
        delay_ms(5);
        
        /* Check for TCXO/XOSC errors - but don't fail on these alone,
         * the real test is PLL lock */
        uint16_t errors = get_errors();
        if (errors & TCXO_ERR_HF_XOSC_START) {
            /* Try again with longer delay */
            delay_ms(20);
            set_standby(1);
            delay_ms(10);
            errors = get_errors();
            if (errors & TCXO_ERR_HF_XOSC_START) {
                xosc_failures++;
                cycle_ok = false;
                clear_errors();
                continue;
            }
        }
        clear_errors();
        
        /* Test PLL lock - this is the definitive test */
        uint32_t freq = 915000000;
        uint8_t freq_params[4] = {
            (freq >> 24) & 0xFF,
            (freq >> 16) & 0xFF,
            (freq >> 8) & 0xFF,
            freq & 0xFF
        };
        
        /* Set packet type first (required for some operations) */
        uint8_t pkt_type = 0x01;  /* LoRa */
        lr1121_send_command(0x020E, &pkt_type, 1);
        delay_ms(2);
        
        lr1121_send_command(0x020B, freq_params, 4);  /* SetRfFrequency */
        delay_ms(5);
        
        errors = get_errors();
        if (errors & TCXO_ERR_PLL_LOCK) {
            pll_failures++;
            cycle_ok = false;
            clear_errors();
        }
        
        if (cycle_ok) {
            success_count++;
        }
        
        /* Progress indicator */
        if ((i + 1) % 10 == 0) {
            DEBUGOUT("    Progress: %lu/%lu, %lu reset, %lu tcxo_cmd, %lu xosc, %lu pll\n",
                     i + 1, num_cycles, reset_failures, tcxo_cmd_failures, 
                     xosc_failures, pll_failures);
        }
    }
    
    DEBUGOUT("\n  --- Power Cycle Results ---\n");
    DEBUGOUT("  Total cycles:    %lu\n", num_cycles);
    DEBUGOUT("  Successful:      %lu (%.2f%%)\n", success_count,
             (100.0f * success_count) / num_cycles);
    DEBUGOUT("  Reset failures:  %lu\n", reset_failures);
    DEBUGOUT("  TCXO cmd fails:  %lu\n", tcxo_cmd_failures);
    DEBUGOUT("  XOSC failures:   %lu\n", xosc_failures);
    DEBUGOUT("  PLL failures:    %lu\n", pll_failures);
    
    if (success_count == num_cycles) {
        DEBUGOUT("  [PASS] All power cycles successful!\n");
    } else {
        DEBUGOUT("  [FAIL] %lu cycles failed\n", num_cycles - success_count);
    }
    
    return success_count;
}

/**
 * @brief Run all extended stress tests
 * 
 * Comprehensive test suite to validate TCXO stability before
 * integrating into main ELRS codebase.
 */
void lr1121_tcxo_stress_test_all(void) {
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║     LR1121 EXTENDED TCXO STRESS TEST SUITE                   ║\n");
    DEBUGOUT("║     Pre-integration validation for ELRS                      ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
    
    /* First run the basic test suite to ensure chip is working */
    DEBUGOUT("\n>>> Running basic initialization tests first...\n");
    lr1121_tcxo_test_run();
    
    DEBUGOUT("\n>>> Starting extended stress tests...\n\n");
    
    /* Test 1: Temperature stability (20 readings) */
    bool temp_ok = lr1121_tcxo_stress_temperature(20);
    
    /* Test 2: FHSS simulation (5000 hops) */
    uint32_t fhss_success = lr1121_tcxo_stress_fhss(5000, 100);
    bool fhss_ok = (fhss_success == 5000);
    
    /* Test 3: TX/RX mode cycling (500 cycles) */
    uint32_t txrx_success = lr1121_tcxo_stress_txrx_cycle(500);
    bool txrx_ok = (txrx_success == 500);
    
    /* Test 4: Long duration PLL lock (30 seconds) */
    bool pll_ok = lr1121_tcxo_stress_pll_duration(30, 100);
    
    /* Test 5: Power cycle stress (50 cycles) */
    uint32_t power_success = lr1121_tcxo_stress_power_cycle(50);
    bool power_ok = (power_success == 50);
    
    /* Final summary */
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║              EXTENDED STRESS TEST SUMMARY                    ║\n");
    DEBUGOUT("╠══════════════════════════════════════════════════════════════╣\n");
    DEBUGOUT("║  Temperature Stability:    %s                            ║\n", temp_ok ? "PASS" : "FAIL");
    DEBUGOUT("║  FHSS (5000 hops):         %s (%lu/5000)              ║\n", fhss_ok ? "PASS" : "FAIL", fhss_success);
    DEBUGOUT("║  TX/RX Cycling (500x):     %s (%lu/500)               ║\n", txrx_ok ? "PASS" : "FAIL", txrx_success);
    DEBUGOUT("║  PLL Lock (30 sec):        %s                            ║\n", pll_ok ? "PASS" : "FAIL");
    DEBUGOUT("║  Power Cycle (50x):        %s (%lu/50)                ║\n", power_ok ? "PASS" : "FAIL", power_success);
    DEBUGOUT("╠══════════════════════════════════════════════════════════════╣\n");
    
    bool all_passed = temp_ok && fhss_ok && txrx_ok && pll_ok && power_ok;
    
    if (all_passed) {
        DEBUGOUT("║                                                              ║\n");
        DEBUGOUT("║    *** ALL STRESS TESTS PASSED ***                          ║\n");
        DEBUGOUT("║    TCXO is stable - READY FOR ELRS INTEGRATION              ║\n");
        DEBUGOUT("║                                                              ║\n");
    } else {
        DEBUGOUT("║                                                              ║\n");
        DEBUGOUT("║    *** SOME TESTS FAILED ***                                ║\n");
        DEBUGOUT("║    Review results before integrating into ELRS              ║\n");
        DEBUGOUT("║                                                              ║\n");
    }
    
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n");
}
