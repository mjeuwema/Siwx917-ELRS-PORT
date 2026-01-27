/**
 * @file external_tcxo_test.c
 * @brief Test for Externally-Powered TCXO on Core1121-HF Module
 * 
 * The Core1121-HF module has its TCXO powered directly from VCC (3.3V),
 * NOT from the LR1121's internal VTCXO regulator (DIO3).
 * 
 * Citation: ExpressLRS GitHub Discussion #3045
 *   "most manufactured devices directly connect the TCXO to a separate power source"
 * 
 * This means:
 * 1. The 32MHz TCXO is ALWAYS running when the module is powered
 * 2. SetTcxoMode() configures the LR1121's internal DIO3 regulator (NOT needed!)
 * 3. We may need to skip SetTcxoMode and just configure for external clock
 * 
 * Test Strategy:
 * - Test 1: Skip SetTcxoMode entirely, try SetStandby(XOSC) directly
 * - Test 2: Call SetTcxoMode with timeout=0 (immediate, no power control)
 * - Test 3: Check if module actually has a TCXO vs plain crystal (XTA/XTB)
 */

#include "lr1121_driver.h"
#include "rsi_debug.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* Command opcodes */
#define CMD_GET_STATUS       0x0100
#define CMD_GET_ERRORS       0x010D
#define CMD_CLEAR_ERRORS     0x010E
#define CMD_SET_STANDBY      0x011C
/* FIX: Changed from 0x0097 to 0x0117 - correct opcode per LR1121 datasheet */
#define CMD_SET_TCXO_MODE    0x0117
#define CMD_CALIBRATE        0x010F  /* Full calibration command */
#define CMD_SET_REG_MODE     0x0110  /* DC-DC vs LDO mode */

/* External functions from lr1121_driver.c */
extern bool lr1121_wait_busy_timeout(uint32_t timeout_ms);
extern bool lr1121_send_command(uint16_t opcode, const uint8_t *params, uint16_t param_len);
extern bool lr1121_read_response(uint8_t *response, uint16_t response_len);
extern void lr1121_cs_assert(void);
extern void lr1121_cs_deassert(void);
extern bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);
extern lr1121_status_t lr1121_reset(void);

/* Delay helper */
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 10000; j++) { }
    }
}

/* Simple command execution with debug output */
static bool exec_cmd_verbose(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                             uint8_t *response, uint16_t response_len) {
    DEBUGOUT("  Sending cmd 0x%04X", opcode);
    if (param_len > 0) {
        DEBUGOUT(" params:");
        for (int i = 0; i < param_len; i++) DEBUGOUT(" %02X", params[i]);
    }
    DEBUGOUT("\n");
    
    if (!lr1121_wait_busy_timeout(100)) {
        DEBUGOUT("  ERROR: BUSY timeout before command\n");
        return false;
    }
    if (!lr1121_send_command(opcode, params, param_len)) {
        DEBUGOUT("  ERROR: Send command failed\n");
        return false;
    }
    if (!lr1121_wait_busy_timeout(500)) {  /* Longer timeout for calibration */
        DEBUGOUT("  ERROR: BUSY timeout after command\n");
        return false;
    }
    if (response && response_len > 0) {
        if (!lr1121_read_response(response, response_len)) {
            DEBUGOUT("  ERROR: Read response failed\n");
            return false;
        }
        DEBUGOUT("  Response:");
        for (int i = 0; i < response_len; i++) DEBUGOUT(" %02X", response[i]);
        DEBUGOUT("\n");
    }
    DEBUGOUT("  OK\n");
    return true;
}

/* Simplified command execution */
static bool exec_cmd(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                     uint8_t *response, uint16_t response_len) {
    if (!lr1121_wait_busy_timeout(100)) return false;
    if (!lr1121_send_command(opcode, params, param_len)) return false;
    if (!lr1121_wait_busy_timeout(500)) return false;
    if (response && response_len > 0) {
        if (!lr1121_read_response(response, response_len)) return false;
    }
    return true;
}

/* Get current chip mode from GetStatus */
static int get_chip_mode(void) {
    uint8_t resp[6] = {0};
    if (!exec_cmd(CMD_GET_STATUS, NULL, 0, resp, 6)) return -1;
    return (resp[1] >> 1) & 0x07;  /* stat2 bits [3:1] */
}

/* Get error flags */
static uint16_t get_errors(void) {
    uint8_t resp[3] = {0};
    if (!exec_cmd(CMD_GET_ERRORS, NULL, 0, resp, 3)) return 0xFFFF;
    return ((uint16_t)resp[1] << 8) | resp[2];
}

/* Clear errors */
static bool clear_errors(void) {
    return exec_cmd(CMD_CLEAR_ERRORS, NULL, 0, NULL, 0);
}

/* Set standby mode (0=RC, 1=XOSC) */
static bool set_standby(uint8_t mode) {
    return exec_cmd(CMD_SET_STANDBY, &mode, 1, NULL, 0);
}

/* Set regulator mode (0=LDO, 1=DC-DC) - Currently unused but kept for future tests */
#if 0  /* Disabled to avoid unused function warning */
static bool set_reg_mode(uint8_t mode) {
    return exec_cmd(CMD_SET_REG_MODE, &mode, 1, NULL, 0);
}
#endif

/* Get chip mode string */
static const char* mode_str(int mode) {
    switch(mode) {
        case 0: return "SLEEP";
        case 1: return "STANDBY_RC";
        case 2: return "STANDBY_XOSC";
        case 3: return "FS";
        case 4: return "RX";
        case 5: return "TX";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Test 1: Skip SetTcxoMode entirely
 * 
 * For externally-powered TCXO, the clock is always present.
 * Just try SetStandby(XOSC) directly after reset.
 */
void test1_skip_tcxo_mode(void) {
    int mode;
    uint16_t errors;
    
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║  TEST 1: Skip SetTcxoMode (External TCXO Test)               ║\n");
    DEBUGOUT("║  Theory: TCXO is powered from VCC, already running           ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    DEBUGOUT("Step 1: Hardware reset\n");
    lr1121_reset();
    delay_ms(100);
    
    DEBUGOUT("Step 2: Clear any existing errors\n");
    clear_errors();
    delay_ms(10);
    
    DEBUGOUT("Step 3: Check initial state\n");
    mode = get_chip_mode();
    errors = get_errors();
    DEBUGOUT("  Initial: Mode=%d (%s), Errors=0x%04X\n", mode, mode_str(mode), errors);
    
    DEBUGOUT("Step 4: Try SetStandby(XOSC) WITHOUT SetTcxoMode\n");
    DEBUGOUT("  Citation: If TCXO is externally powered, chip should see clock\n");
    set_standby(1);  /* XOSC mode */
    delay_ms(100);   /* Wait for oscillator */
    
    DEBUGOUT("Step 5: Check result\n");
    mode = get_chip_mode();
    errors = get_errors();
    DEBUGOUT("  Result: Mode=%d (%s), Errors=0x%04X\n", mode, mode_str(mode), errors);
    
    if (mode == 2) {
        DEBUGOUT("\n✓✓✓ SUCCESS! Chip entered STANDBY_XOSC without SetTcxoMode! ✓✓✓\n");
        DEBUGOUT("    The TCXO is externally powered - no configuration needed.\n");
    } else if (errors & 0x0020) {
        DEBUGOUT("\n✗ FAILED: HF_XOSC_START_ERR still present\n");
        DEBUGOUT("    Chip fell back to mode=%d after XOSC failure\n", mode);
    } else {
        DEBUGOUT("\n? Unexpected result - check manually\n");
    }
}

/**
 * @brief Test 2: SetTcxoMode with timeout=0
 * 
 * Some documentation suggests timeout=0 means "TCXO externally powered"
 */
void test2_tcxo_timeout_zero(void) {
    int mode;
    uint16_t errors;
    
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║  TEST 2: SetTcxoMode with timeout=0 (External TCXO Mode)     ║\n");
    DEBUGOUT("║  Theory: timeout=0 means TCXO is already running             ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    DEBUGOUT("Step 1: Hardware reset\n");
    lr1121_reset();
    delay_ms(100);
    
    DEBUGOUT("Step 2: Clear any existing errors\n");
    clear_errors();
    delay_ms(10);
    
    DEBUGOUT("Step 3: SetTcxoMode with timeout=0\n");
    DEBUGOUT("  Citation: LR1121 User Manual Section 6.3.2\n");
    DEBUGOUT("  If timeout=0, chip expects TCXO to be immediately available\n");
    
    uint8_t params[4] = {
        0x07,  /* voltage trim (ignored for external) */
        0x00,  /* delay MSB */
        0x00,  /* delay MID */
        0x00   /* delay LSB = 0 (immediate) */
    };
    exec_cmd_verbose(CMD_SET_TCXO_MODE, params, 4, NULL, 0);
    delay_ms(10);
    
    DEBUGOUT("Step 4: SetStandby(XOSC)\n");
    set_standby(1);
    delay_ms(100);
    
    DEBUGOUT("Step 5: Check result\n");
    mode = get_chip_mode();
    errors = get_errors();
    DEBUGOUT("  Result: Mode=%d (%s), Errors=0x%04X\n", mode, mode_str(mode), errors);
    
    if (mode == 2) {
        DEBUGOUT("\n✓✓✓ SUCCESS! Chip entered STANDBY_XOSC! ✓✓✓\n");
    } else {
        DEBUGOUT("\n✗ Test 2 did not resolve the issue\n");
    }
}

/**
 * @brief Test 3: Try calibration in STANDBY_RC mode
 * 
 * The Calibrate command should work in STANDBY_RC mode.
 * This tests if the chip is functional without XOSC.
 */
void test3_calibrate_in_rc_mode(void) {
    int mode;
    uint16_t errors;
    
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║  TEST 3: Calibration in STANDBY_RC Mode                      ║\n");
    DEBUGOUT("║  Theory: Verify chip is functional, XOSC is the only issue   ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    DEBUGOUT("Step 1: Hardware reset\n");
    lr1121_reset();
    delay_ms(100);
    
    DEBUGOUT("Step 2: Clear errors and set STANDBY_RC\n");
    clear_errors();
    set_standby(0);  /* RC mode */
    delay_ms(10);
    
    mode = get_chip_mode();
    DEBUGOUT("  Mode after SetStandby(RC): %d (%s)\n", mode, mode_str(mode));
    
    DEBUGOUT("Step 3: Run calibration (all blocks)\n");
    DEBUGOUT("  Citation: LR1121 User Manual - Calibrate command\n");
    DEBUGOUT("  This should work in STANDBY_RC mode\n");
    
    /* Calibrate command: CalibParam is a bitmask of blocks to calibrate
     * Bit 0: RC64K
     * Bit 1: RC13M  
     * Bit 2: PLL
     * Bit 3: ADC
     * Bit 4: IMG
     * Bit 5: PLL_TX
     * 0x3F = calibrate all blocks
     */
    uint8_t calib_param = 0x3F;  /* All blocks */
    bool calib_ok = exec_cmd_verbose(CMD_CALIBRATE, &calib_param, 1, NULL, 0);
    
    delay_ms(100);  /* Wait for calibration */
    
    DEBUGOUT("Step 4: Check calibration result\n");
    mode = get_chip_mode();
    errors = get_errors();
    DEBUGOUT("  After calibration: Mode=%d (%s), Errors=0x%04X\n", mode, mode_str(mode), errors);
    
    if (calib_ok && (errors & 0x0020) == 0) {
        DEBUGOUT("\n✓ Calibration succeeded in STANDBY_RC mode\n");
        DEBUGOUT("  Chip is functional - XOSC is the specific issue\n");
    } else {
        DEBUGOUT("\n✗ Calibration issue - check error flags\n");
    }
}

/**
 * @brief Test 4: Check if module might have crystal instead of TCXO
 * 
 * Some cheaper modules use a plain 32MHz crystal (not TCXO).
 * These require different initialization (no SetTcxoMode at all).
 */
void test4_crystal_mode_check(void) {
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║  TEST 4: Crystal vs TCXO Detection                           ║\n");
    DEBUGOUT("║  Theory: Module might use crystal, not TCXO                  ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    DEBUGOUT("Checking module type...\n");
    DEBUGOUT("\n");
    DEBUGOUT("IMPORTANT: If the Core1121-HF uses a plain CRYSTAL (not TCXO):\n");
    DEBUGOUT("  1. SetTcxoMode should NOT be called at all\n");
    DEBUGOUT("  2. The crystal needs time to start oscillating\n");
    DEBUGOUT("  3. SetStandby(XOSC) should work after crystal stabilizes\n");
    DEBUGOUT("\n");
    DEBUGOUT("If the module has a TCXO:\n");
    DEBUGOUT("  1. TCXO starts immediately when power is applied\n");
    DEBUGOUT("  2. SetTcxoMode is only needed if TCXO is powered from DIO3\n");
    DEBUGOUT("  3. For external-power TCXO, SetStandby(XOSC) should work\n");
    DEBUGOUT("\n");
    
    DEBUGOUT("Running crystal startup test (longer delays)...\n");
    
    lr1121_reset();
    delay_ms(500);  /* Long delay for crystal startup */
    
    clear_errors();
    
    DEBUGOUT("Trying SetStandby(XOSC) after 500ms crystal warm-up...\n");
    set_standby(1);
    delay_ms(200);
    
    int mode = get_chip_mode();
    uint16_t errors = get_errors();
    DEBUGOUT("  Result: Mode=%d (%s), Errors=0x%04X\n", mode, mode_str(mode), errors);
    
    if (mode == 2) {
        DEBUGOUT("\n✓ Crystal/TCXO started after extended delay!\n");
    } else {
        DEBUGOUT("\n✗ Still failing - this appears to be a hardware issue\n");
        DEBUGOUT("  The oscillator (TCXO or crystal) is not producing a clock signal.\n");
    }
}

/**
 * @brief Run all external TCXO tests
 */
void external_tcxo_test_all(void) {
    DEBUGOUT("\n");
    DEBUGOUT("████████████████████████████████████████████████████████████████\n");
    DEBUGOUT("█  EXTERNAL TCXO TEST SUITE FOR CORE1121-HF                    █\n");
    DEBUGOUT("█  Testing different initialization strategies                  █\n");
    DEBUGOUT("████████████████████████████████████████████████████████████████\n");
    
    test1_skip_tcxo_mode();
    test2_tcxo_timeout_zero();
    test3_calibrate_in_rc_mode();
    test4_crystal_mode_check();
    
    DEBUGOUT("\n");
    DEBUGOUT("████████████████████████████████████████████████████████████████\n");
    DEBUGOUT("█  TEST SUITE COMPLETE                                          █\n");
    DEBUGOUT("████████████████████████████████████████████████████████████████\n");
    DEBUGOUT("\n");
    DEBUGOUT("CONCLUSION:\n");
    DEBUGOUT("If ALL tests show HF_XOSC_START_ERR (0x0020), the problem is HARDWARE:\n");
    DEBUGOUT("  - The TCXO/crystal is not oscillating\n");
    DEBUGOUT("  - Check 32MHz clock signal on XTA pin with oscilloscope\n");
    DEBUGOUT("  - Module may be defective\n");
    DEBUGOUT("\n");
    DEBUGOUT("If Test 1 or 2 succeeded, update your driver to use that method.\n");
}
