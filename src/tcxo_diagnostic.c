/**
 * @file tcxo_diagnostic.c
 * @brief Deep TCXO Diagnostic for Waveshare Core1121-XF Module
 * 
 * UPDATED 2026-01-21: External TCXO Configuration
 * ================================================
 * The Waveshare Core1121-XF has an EXTERNALLY-POWERED 32MHz TCXO that is
 * always on from the 3.3V rail. The LR1121's internal VTCXO regulator
 * is NOT used to power the TCXO.
 *
 * Correct Configuration:
 *   - Voltage parameter = 0 (TCXO_CTRL_NONE) - no internal regulator
 *   - Delay = ~5-10ms for stabilization (TCXO already running)
 *   - SetTcxoMode is STILL required to switch from XO to TCXO input mode
 * 
 * This diagnostic helps verify if the TCXO init is working properly.
 * 
 * Citations:
 * - LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 * - LR1121 User Manual Section 3.6.1 "GetErrors"
 * - Waveshare Core1121_XF_Demo configuration (voltage=0 for external TCXO)
 * - Semtech LR11xx SDK - LR11XX_RADIO_TCXO_CTRL_NONE for external supply
 */

#include "lr1121_driver.h"
#include "rsi_debug.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* Command opcodes
 * Citation: LR1121 Datasheet Section 11 "Command Reference"
 */
#define CMD_GET_STATUS       0x0100
#define CMD_GET_ERRORS       0x010D
#define CMD_CLEAR_ERRORS     0x010E
#define CMD_SET_REG_MODE     0x0110  /* SetRegMode - LDO/DC-DC selection */
#define CMD_SET_TCXO_MODE    0x0117  /* SetTcxoMode - TCXO voltage/delay */
#define CMD_GET_TEMP         0x011A
#define CMD_SET_STANDBY      0x011C

/* External functions from lr1121_driver.c */
extern bool lr1121_wait_busy_timeout(uint32_t timeout_ms);
extern bool lr1121_send_command(uint16_t opcode, const uint8_t *params, uint16_t param_len);
extern bool lr1121_read_response(uint8_t *response, uint16_t response_len);
extern void lr1121_cs_assert(void);
extern void lr1121_cs_deassert(void);
extern bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

/* Delay helper */
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 10000; j++) { }
    }
}

/* Simple command execution */
static bool exec_cmd(uint16_t opcode, const uint8_t *params, uint16_t param_len,
                     uint8_t *response, uint16_t response_len) {
    if (!lr1121_wait_busy_timeout(100)) return false;
    if (!lr1121_send_command(opcode, params, param_len)) return false;
    if (!lr1121_wait_busy_timeout(100)) return false;
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

/* Send SetTcxoMode with specific voltage and delay */
static bool set_tcxo_mode(uint8_t voltage_trim, uint32_t delay_steps) {
    uint8_t params[4];
    params[0] = voltage_trim;
    params[1] = (delay_steps >> 16) & 0xFF;
    params[2] = (delay_steps >> 8) & 0xFF;
    params[3] = delay_steps & 0xFF;
    return exec_cmd(CMD_SET_TCXO_MODE, params, 4, NULL, 0);
}

/**
 * @brief Deep TCXO diagnostic
 * Run this to diagnose why HF_XOSC_START_ERR persists
 */
void tcxo_deep_diagnostic(void) {
    int mode;
    uint16_t errors;
    bool success;
    
    DEBUGOUT("\n");
    DEBUGOUT("╔══════════════════════════════════════════════════════════════╗\n");
    DEBUGOUT("║      TCXO DIAGNOSTIC FOR WAVESHARE CORE1121-XF               ║\n");
    DEBUGOUT("║      External TCXO Config: voltage=0, delay=~20ms            ║\n");
    DEBUGOUT("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    /* ========================================================================
     * TEST 1: Verify SPI communication works (STANDBY_RC mode)
     * If chip can enter STANDBY_RC, SPI is working fine.
     * ======================================================================== */
    DEBUGOUT("═══ TEST 1: SPI Communication Check (STANDBY_RC) ═══\n");
    
    clear_errors();
    delay_ms(10);
    
    success = set_standby(0);  /* RC mode */
    delay_ms(50);
    
    mode = get_chip_mode();
    DEBUGOUT("  SetStandby(RC): %s\n", success ? "sent" : "FAILED");
    DEBUGOUT("  Chip Mode: %d (expect 1=STANDBY_RC)\n", mode);
    
    if (mode == 1) {
        DEBUGOUT("  ✓ PASS: SPI communication working, chip enters STANDBY_RC\n");
    } else {
        DEBUGOUT("  ✗ FAIL: Cannot even enter STANDBY_RC! Check SPI wiring.\n");
        return;
    }
    DEBUGOUT("\n");
    
    /* ========================================================================
     * TEST 2: Check if errors clear properly
     * ======================================================================== */
    DEBUGOUT("═══ TEST 2: Error Flag Persistence Test ═══\n");
    
    errors = get_errors();
    DEBUGOUT("  Errors BEFORE ClearErrors: 0x%04X\n", errors);
    
    clear_errors();
    delay_ms(50);
    
    errors = get_errors();
    DEBUGOUT("  Errors AFTER ClearErrors:  0x%04X\n", errors);
    
    if (errors == 0) {
        DEBUGOUT("  ✓ PASS: Errors cleared successfully\n");
    } else {
        DEBUGOUT("  ✗ NOTE: Some errors persist (may be set by failed XOSC)\n");
    }
    DEBUGOUT("\n");
    
    /* ========================================================================
     * TEST 3: Try EXTERNAL TCXO configuration FIRST (Waveshare Core1121-XF)
     * Citation: Waveshare Core1121_XF_Demo uses voltage=0 for external TCXO
     * Citation: Semtech LR11xx SDK - LR11XX_RADIO_TCXO_CTRL_NONE
     * ======================================================================== */
    DEBUGOUT("═══ TEST 3: External TCXO Configuration (voltage=0) ═══\n");
    DEBUGOUT("  The Waveshare Core1121-XF has an EXTERNALLY-POWERED TCXO\n");
    DEBUGOUT("  that is always on from the 3.3V rail.\n");
    DEBUGOUT("  Configuration: voltage=0 (no internal regulator), delay=~10ms\n\n");
    
    /* Test external TCXO configuration: voltage=0, delay=~10ms */
    DEBUGOUT("  --- Testing EXTERNAL TCXO: voltage=0 (TCXO_CTRL_NONE) ---\n");
    
    clear_errors();
    delay_ms(10);
    
    /* Set DC-DC mode first */
    DEBUGOUT("    Setting DC-DC mode...\n");
    uint8_t dcdc_mode = 0x01;
    exec_cmd(CMD_SET_REG_MODE, &dcdc_mode, 1, NULL, 0);
    delay_ms(10);
    
    set_standby(0);  /* RC mode */
    delay_ms(50);
    
    /* External TCXO: voltage=0, delay=~10ms (328 ticks = 0x148) */
    uint32_t external_delay = 0x000148;  /* ~10ms = 328 ticks * 30.52µs */
    DEBUGOUT("    Sending SetTcxoMode(volt=0x00, delay=~10ms) for EXTERNAL TCXO...\n");
    success = set_tcxo_mode(0x00, external_delay);
    
    if (!success) {
        DEBUGOUT("    SetTcxoMode: Command FAILED to send\n");
    } else {
        /* Short delay - external TCXO is already running */
        DEBUGOUT("    Waiting 20ms for TCXO sync...\n");
        delay_ms(20);
        
        /* Try switching to XOSC */
        DEBUGOUT("    Sending SetStandby(XOSC)...\n");
        set_standby(1);
        delay_ms(100);
        
        /* Check result */
        mode = get_chip_mode();
        errors = get_errors();
        
        DEBUGOUT("    Result: Mode=%d, Errors=0x%04X\n", mode, errors);
        
        if (mode == 2) {
            DEBUGOUT("    ✓✓✓ SUCCESS! EXTERNAL TCXO WORKING! ✓✓✓\n");
            DEBUGOUT("\n");
            DEBUGOUT("  *** Configuration CORRECT in lr1121_elrs_init.h: ***\n");
            DEBUGOUT("  ***   #define TCXO_ACTIVE_VOLTAGE  TCXO_VOLTAGE_NONE  // 0x00 ***\n");
            DEBUGOUT("  ***   #define TCXO_ACTIVE_DELAY    TCXO_DELAY_328_TICKS  // ~10ms ***\n");
            return;
        } else if (errors & 0x20) {
            DEBUGOUT("    ✗ HF_XOSC_START_ERR with external TCXO config\n");
        } else {
            DEBUGOUT("    ✗ Mode not STANDBY_XOSC\n");
        }
    }
    DEBUGOUT("\n");
    
    /* ========================================================================
     * TEST 4: Try internal TCXO voltages (fallback if external fails)
     * ======================================================================== */
    DEBUGOUT("═══ TEST 4: Internal TCXO Voltage Sweep (fallback) ═══\n");
    DEBUGOUT("  If external TCXO failed, testing internal regulator voltages:\n");
    DEBUGOUT("    0x07 (3.3V), 0x06 (3.0V), 0x05 (2.7V), 0x04 (2.4V)\n\n");
    
    /* Test in priority order: 3.3V, 3.0V, 2.7V, 2.4V */
    uint8_t voltages[] = {0x07, 0x06, 0x05, 0x04};
    const char* voltage_names[] = {"3.3V", "3.0V", "2.7V", "2.4V"};
    uint32_t delay_150ms = 0x001333;  /* ~150ms */
    
    for (int i = 0; i < 4; i++) {
        DEBUGOUT("  --- Testing voltage: %s (0x%02X) ---\n", voltage_names[i], voltages[i]);
        
        /* Reset to known state */
        clear_errors();
        delay_ms(10);
        
        uint8_t dcdc = 0x01;
        exec_cmd(CMD_SET_REG_MODE, &dcdc, 1, NULL, 0);
        delay_ms(10);
        
        set_standby(0);  /* RC mode */
        delay_ms(50);
        
        /* Try SetTcxoMode with this voltage */
        DEBUGOUT("    Sending SetTcxoMode(volt=0x%02X, delay=~150ms)...\n", voltages[i]);
        success = set_tcxo_mode(voltages[i], delay_150ms);
        
        if (!success) {
            DEBUGOUT("    SetTcxoMode: Command FAILED to send\n");
            continue;
        }
        
        delay_ms(200);
        
        /* Try switching to XOSC */
        DEBUGOUT("    Sending SetStandby(XOSC)...\n");
        set_standby(1);
        delay_ms(200);
        
        /* Check result */
        mode = get_chip_mode();
        errors = get_errors();
        
        DEBUGOUT("    Result: Mode=%d, Errors=0x%04X\n", mode, errors);
        
        if (mode == 2) {
            DEBUGOUT("    ✓✓✓ SUCCESS! XOSC started with %s ✓✓✓\n", voltage_names[i]);
            DEBUGOUT("\n");
            DEBUGOUT("  *** FIX: Update lr1121_elrs_init.h: ***\n");
            DEBUGOUT("  ***   #define TCXO_ACTIVE_VOLTAGE  0x%02X  // %s ***\n", voltages[i], voltage_names[i]);
            DEBUGOUT("  ***   #define TCXO_ACTIVE_DELAY    TCXO_DELAY_1500_TICKS  // ~46ms ***\n");
            return;
        } else if (errors & 0x20) {
            DEBUGOUT("    ✗ FAIL: HF_XOSC_START_ERR still present\n");
        } else {
            DEBUGOUT("    ✗ FAIL: Mode not STANDBY_XOSC\n");
        }
        DEBUGOUT("\n");
    }
    
    /* ========================================================================
     * TEST 5: Extended delay test with external TCXO config
     * ======================================================================== */
    DEBUGOUT("═══ TEST 5: External TCXO with Extended Delays ═══\n");
    DEBUGOUT("  Testing external TCXO (voltage=0) with varying delays\n\n");
    
    /* Delays in 30.52µs ticks */
    uint32_t ext_delays[] = {
        0x0000A4,  /* 164 ticks = ~5ms */
        0x000148,  /* 328 ticks = ~10ms */
        0x0001F4,  /* 500 ticks = ~15ms */
        0x0003E8   /* 1000 ticks = ~30ms */
    };
    const char* ext_delay_names[] = {"~5ms (164)", "~10ms (328)", "~15ms (500)", "~30ms (1000)"};
    
    for (int i = 0; i < 4; i++) {
        DEBUGOUT("  --- Testing external TCXO with delay: %s ---\n", ext_delay_names[i]);
        
        clear_errors();
        delay_ms(10);
        
        /* Set DC-DC mode first */
        uint8_t dcdc_m = 0x01;
        exec_cmd(CMD_SET_REG_MODE, &dcdc_m, 1, NULL, 0);
        delay_ms(10);
        
        set_standby(0);
        delay_ms(50);
        
        /* External TCXO: voltage=0 with varying delays */
        success = set_tcxo_mode(0x00, ext_delays[i]);
        
        /* Wait: configured delay + margin */
        uint32_t wait_ms = ((ext_delays[i] * 31) / 1000) + 20;
        DEBUGOUT("    Waiting %lums for TCXO sync...\n", (unsigned long)wait_ms);
        delay_ms(wait_ms);
        
        set_standby(1);
        delay_ms(100);
        
        mode = get_chip_mode();
        errors = get_errors();
        
        DEBUGOUT("    Result: Mode=%d, Errors=0x%04X\n", mode, errors);
        
        if (mode == 2) {
            DEBUGOUT("    ✓✓✓ SUCCESS! External TCXO with %s delay ✓✓✓\n", ext_delay_names[i]);
            DEBUGOUT("\n");
            DEBUGOUT("  *** FIX: Update lr1121_elrs_init.h: ***\n");
            DEBUGOUT("  ***   #define TCXO_ACTIVE_VOLTAGE  TCXO_VOLTAGE_NONE  // 0x00 ***\n");
            DEBUGOUT("  ***   #define TCXO_ACTIVE_DELAY    0x%06lX  // %s ***\n", 
                     (unsigned long)ext_delays[i], ext_delay_names[i]);
            return;
        } else if (errors & 0x20) {
            DEBUGOUT("    ✗ HF_XOSC_START_ERR - external TCXO may have issue\n");
        }
        DEBUGOUT("\n");
    }
    
    /* ========================================================================
     * CONCLUSION: Hardware failure
     * ======================================================================== */
    DEBUGOUT("═══════════════════════════════════════════════════════════════\n");
    DEBUGOUT("                    DIAGNOSTIC CONCLUSION\n");
    DEBUGOUT("═══════════════════════════════════════════════════════════════\n\n");
    
    DEBUGOUT("  All software configurations tested - XOSC still fails to start.\n\n");
    
    DEBUGOUT("  LIKELY CAUSE: Hardware issue with Core1121-HF module:\n");
    DEBUGOUT("    1. TCXO crystal itself is faulty/damaged\n");
    DEBUGOUT("    2. TCXO signal not reaching LR1121 XOSC input\n");
    DEBUGOUT("       (cold solder joint, broken trace, or ESD damage)\n");
    DEBUGOUT("    3. TCXO not getting proper power (despite 3.3V measured)\n\n");
    
    DEBUGOUT("  VERIFICATION STEPS:\n");
    DEBUGOUT("    1. Try a DIFFERENT Core1121-HF module if available\n");
    DEBUGOUT("    2. Use oscilloscope to verify 32MHz signal at:\n");
    DEBUGOUT("       - TCXO output pin\n");
    DEBUGOUT("       - LR1121 XOSC input pin (pin 10 on LR1121)\n");
    DEBUGOUT("    3. Check for shorts or opens on TCXO circuit\n");
    DEBUGOUT("    4. Contact module supplier for RMA/replacement\n\n");
    
    DEBUGOUT("  Note: The LR1121 CAN still operate in STANDBY_RC mode,\n");
    DEBUGOUT("  but RF accuracy and temperature functions will be limited.\n");
    DEBUGOUT("═══════════════════════════════════════════════════════════════\n");
}

/**
 * @brief Quick TCXO test - test external TCXO configuration
 *
 * For Waveshare Core1121-XF with externally-powered TCXO:
 *   - Voltage = 0 (no internal regulator)
 *   - Delay = ~10ms (TCXO already running, just stabilization)
 */
void tcxo_quick_test(void) {
    DEBUGOUT("\n=== Quick TCXO Test (External TCXO Config) ===\n");
    
    /* External TCXO config - Waveshare Core1121-XF */
    uint8_t volt = 0x00;  /* 0 = External TCXO, no internal regulator */
    uint32_t delay = 0x000148;  /* ~10ms = 328 ticks */
    
    DEBUGOUT("Config: Voltage=0x%02X (EXTERNAL TCXO - no internal reg)\n", volt);
    DEBUGOUT("        Delay=0x%06lX (~10ms stabilization)\n", (unsigned long)delay);
    DEBUGOUT("\n");
    
    /* Clear errors */
    clear_errors();
    delay_ms(10);
    
    /* Check errors after clear */
    uint16_t err_before = get_errors();
    DEBUGOUT("Errors after clear: 0x%04X\n", err_before);
    
    /* Set DC-DC mode */
    DEBUGOUT("Setting DC-DC mode...\n");
    uint8_t dcdc = 0x01;
    exec_cmd(CMD_SET_REG_MODE, &dcdc, 1, NULL, 0);
    delay_ms(10);
    
    /* Send SetTcxoMode for external TCXO */
    DEBUGOUT("Sending SetTcxoMode(0, ~10ms) for EXTERNAL TCXO...\n");
    bool ok = set_tcxo_mode(volt, delay);
    DEBUGOUT("  Result: %s\n", ok ? "OK" : "FAILED");
    
    /* Short wait - external TCXO already running */
    DEBUGOUT("Waiting 20ms for TCXO sync...\n");
    delay_ms(20);
    
    /* Try XOSC mode */
    DEBUGOUT("Sending SetStandby(XOSC)...\n");
    set_standby(1);
    delay_ms(100);
    
    /* Check status */
    int mode = get_chip_mode();
    uint16_t err_after = get_errors();
    
    DEBUGOUT("\nResults:\n");
    DEBUGOUT("  Chip Mode: %d ", mode);
    switch (mode) {
        case 0: DEBUGOUT("(SLEEP)\n"); break;
        case 1: DEBUGOUT("(STANDBY_RC) ← XOSC FAILED!\n"); break;
        case 2: DEBUGOUT("(STANDBY_XOSC) ← SUCCESS!\n"); break;
        default: DEBUGOUT("(Unknown)\n"); break;
    }
    DEBUGOUT("  Errors: 0x%04X", err_after);
    if (err_after & 0x20) DEBUGOUT(" [HF_XOSC_START_ERR]");
    if (err_after & 0x80) DEBUGOUT(" [PLL_LOCK_ERR]");
    DEBUGOUT("\n");
    
    if (mode == 2 && !(err_after & 0x20)) {
        DEBUGOUT("\n✓ EXTERNAL TCXO is working!\n");
        DEBUGOUT("  Correct config: voltage=0, delay=~10ms\n");
    } else {
        DEBUGOUT("\n✗ TCXO still failing - run tcxo_deep_diagnostic() for full analysis\n");
    }
}
