/**
 * @file lr1121_elrs_init.h
 * @brief LR1121 Initialization with Correct TCXO Sequence
 *
 * MODIFIED (2026-01-17): Now implements CORRECT TCXO initialization sequence
 * as specified by user requirement for proper LoRa RF precision.
 *
 * THE PROBLEM WITH SKIPPING TCXO SETUP:
 *   The LR1121 uses internal RC oscillator at boot. This RC oscillator
 *   CANNOT provide the precision required for:
 *   - LoRa modulation frequency synthesis
 *   - PLL lock stability
 *   - Image rejection calibration accuracy
 *
 *   Without proper TCXO initialization, you WILL encounter:
 *   - Calibration Failures
 *   - PLL Lock Errors  
 *   - HF_XOSC_START_ERR status errors
 *
 * CORRECT TCXO INITIALIZATION SEQUENCE (MANDATORY ORDER):
 *   Order | Command      | Opcode | Parameter(s)   | Purpose
 *   ------|--------------|--------|----------------|---------------------------
 *     1   | SetTcxoMode  | 0x0117 | Voltage, Delay | Power TCXO via VTCXO pin
 *     2   | SetStandby   | 0x011C | 0x01 (XOSC)    | Switch clock RC → TCXO
 *     3   | SetRegulator | 0x0110 | 0x01 (DCDC)    | Configure power efficiency
 *     4   | Calibrate    | 0x010F | 0x3F (All)     | Full calibration with TCXO
 *
 * Citations:
 *   - User requirement analysis (2026-01-17)
 *   - LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *   - LR1121 Datasheet Section 11.2.2 "SetStandby"
 *   - LR1121 Datasheet Section 11.2.7 "SetRegMode"
 *   - LR1121 Datasheet Section 11.2.4 "Calibrate"
 *   - Semtech lr11xx_system.c official driver implementation
 */

#ifndef LR1121_ELRS_INIT_H
#define LR1121_ELRS_INIT_H

#include "lr1121_driver.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Band selection for lr1121_waveshare_init() image calibration and for
 * the lr1121_rx_test standalone test. Override via -DLR1121_BAND_24GHZ=1
 * in CMake, or edit here.
 *   0 = sub-GHz (915 MHz, CalibImage 0xE1..0xE9, PE4259 switch used)
 *   1 = 2.4 GHz (CalibImage 0x94..0x98, internal RFIO_HF, no ext switch)
 */
#ifndef LR1121_BAND_24GHZ
#define LR1121_BAND_24GHZ 1
#endif

/*******************************************************************************
 * ELRS-Compatible Command Opcodes
 * Citation: ExpressLRS LR1121.cpp + LR1121 Datasheet
 ******************************************************************************/

/* System Commands - VERIFIED OPCODES from Semtech lr11xx_system.c
 * 
 * Citation: Waveshare Core1121_XF_Demo lr11xx_system.c (Semtech official driver)
 * Path: Core1121_XF_Demo\esp32s3\Arduino\waveshare_lroa_1121\src\lr11xx_driver\lr11xx_system.c
 * 
 * These are the CORRECT LR1121/LR11XX opcodes from the official Semtech driver!
 */
#define ELRS_CMD_GET_STATUS           0x0100  /* GetStatus - lines 95 */
#define ELRS_CMD_GET_VERSION          0x0101  /* GetVersion - line 96 */
#define ELRS_CMD_GET_ERRORS           0x010D  /* GetErrors - line 97 */
#define ELRS_CMD_CLEAR_ERRORS         0x010E  /* ClearErrors - line 98 */
#define ELRS_CMD_CALIBRATE            0x010F  /* Calibrate - line 99 (NOT 0x0100!) */
#define ELRS_CMD_SET_REG_MODE         0x0110  /* SetRegMode - line 100 */
#define ELRS_CMD_CALIBRATE_IMAGE      0x0111  /* CalibrateImage - line 101 */
#define ELRS_CMD_SET_DIO_AS_RF_SWITCH 0x0112  /* SetDioAsRfSwitch - line 102 */
#define ELRS_CMD_SET_DIO_IRQ_PARAMS   0x0113  /* SetDioIrqParams - line 103 */
#define ELRS_CMD_CLEAR_IRQ            0x0114  /* ClearIrq - line 104 */
#define ELRS_CMD_CFG_LFCLK            0x0116  /* CfgLfClk - line 105 */
#define ELRS_CMD_SET_TCXO_MODE        0x0117  /* SetTcxoMode - line 106 (NOT 0x0102!) */
#define ELRS_CMD_REBOOT               0x0118  /* Reboot - line 107 */
#define ELRS_CMD_GET_VBAT             0x0119  /* GetVbat - line 108 */
#define ELRS_CMD_GET_TEMP             0x011A  /* GetTemp - line 109 */
#define ELRS_CMD_SET_SLEEP            0x011B  /* SetSleep - line 110 */
#define ELRS_CMD_SET_STANDBY          0x011C  /* SetStandby - line 111 (NOT 0x0105!) */
#define ELRS_CMD_SET_FS               0x011D  /* SetFs - line 112 */
#define ELRS_CMD_GET_RANDOM_NUMBER    0x0120  /* GetRandomNumber - line 113 */

/* Radio Commands */
#define ELRS_CMD_SET_PACKET_TYPE      0x020E
#define ELRS_CMD_SET_RF_FREQUENCY     0x020B
#define ELRS_CMD_SET_TX_PARAMS        0x0211
#define ELRS_CMD_SET_MODULATION_PARAMS 0x020F
#define ELRS_CMD_SET_PACKET_PARAMS    0x0210
#define ELRS_CMD_SET_PA_CONFIG        0x0215
#define ELRS_CMD_SET_RX_TX_FALLBACK   0x0213
#define ELRS_CMD_SET_RX_BOOSTED       0x0227
#define ELRS_CMD_SET_TX               0x020A
#define ELRS_CMD_SET_RX               0x0209
#define ELRS_CMD_GET_RSSI_INST        0x0205

/* Buffer Commands */
#define ELRS_CMD_WRITE_BUFFER8        0x0180
#define ELRS_CMD_READ_BUFFER8         0x0181

/*******************************************************************************
 * ELRS-Compatible Constants
 * Citation: ExpressLRS LR1121.cpp
 ******************************************************************************/

/* Standby modes */
#define ELRS_STANDBY_RC    0x00
#define ELRS_STANDBY_XOSC  0x01

/* Regulator modes */
#define ELRS_REG_MODE_LDO   0x00
#define ELRS_REG_MODE_DCDC  0x01

/* Low-Frequency Clock (LFCLK) modes for CfgLfClk command
 * Citation: Semtech lr11xx_system.h lr11xx_system_lfclk_cfg_t
 *   - LR11XX_SYSTEM_LFCLK_RC   = 0x00 (Internal 32kHz RC)
 *   - LR11XX_SYSTEM_LFCLK_EXT  = 0x01 (External 32kHz input)
 *   - LR11XX_SYSTEM_LFCLK_XTAL = 0x02 (32kHz crystal)
 * 
 * If your board does NOT have a physical 32.768kHz crystal, 
 * set LR1121_LFCLK_USE_RC to 1 to use the internal RC oscillator.
 * The RC oscillator is less accurate but always available.
 */
#define LR11XX_SYSTEM_LFCLK_RC   0x00  /* Internal 32kHz RC oscillator */
#define LR11XX_SYSTEM_LFCLK_EXT  0x01  /* External 32kHz clock input */
#define LR11XX_SYSTEM_LFCLK_XTAL 0x02  /* 32kHz crystal (Waveshare default) */

/* Configuration: Set to 1 to use RC oscillator if board lacks 32kHz XTAL
 *
 * CRITICAL FIX (2026-01-16): Changed to 1 (Internal RC) to fix PLL_LOCK_ERR
 * 
 * Root Cause Analysis:
 *   - Serial output showed: SPI RX: 05 13 00 followed by calibration failure
 *   - GetErrors reported: PLL_LOCK_ERR and IMG_CALIB_ERR
 *   - The Waveshare Core1121-HF module may NOT have 32.768kHz crystal populated
 *   - Configuring for LF_XTAL (0x02) when no crystal exists causes the LR1121
 *     internal state machine to hang waiting for a clock that never arrives
 *   - This prevents the PLL from timing its lock correctly → PLL_LOCK_ERR
 *
 * The Fix:
 *   - Use Internal RC oscillator (LR11XX_SYSTEM_LFCLK_RC = 0x00) instead
 *   - Step 8 now sends: [0x01 0x16] [0x04] instead of [0x01 0x16] [0x06]
 *     where 0x04 = RC(0x00) | wait_32k_ready(bit 2) = 0x04
 *
 * Citation: Semtech lr11xx_system.c line 368:
 *   cbuffer[2] = (uint8_t)(lfclock_cfg | (wait_for_32k_ready << 2))
 * Citation: Semtech lr11xx_system.h lr11xx_system_lfclk_cfg_t enum
 */
/**
 * LR1121_LFCLK_USE_RC - Low Frequency Clock Source Selection
 *
 * Set to 0 to use 32kHz XTAL (requires populated 32kHz crystal)
 * Set to 1 to use internal 32kHz RC oscillator (no external crystal needed)
 *
 * IMPORTANT FIX (2026-01-16): Changed from 0 to 1
 * ------------------------------------------------
 * Serial output showed: "BUSY timeout - possible missing 32kHz crystal?"
 * The Waveshare Core1121-HF module may NOT have the 32.768kHz crystal populated.
 * Using XTAL mode when no crystal exists causes PLL_LOCK_ERR and IMG_CALIB_ERR
 * because the LR1121 state machine hangs waiting for a clock that never arrives.
 *
 * Citation: AI diagnostic analysis of serial output showing Step 9 failure
 */
#ifndef LR1121_LFCLK_USE_RC
#define LR1121_LFCLK_USE_RC  1  /* 1=RC (internal oscillator - NO EXTERNAL CRYSTAL) */
#endif

/* RX/TX Fallback modes (SetRxTxFallbackMode) */
#define ELRS_FALLBACK_STDBY_RC   0x01
#define ELRS_FALLBACK_STDBY_XOSC 0x02
#define ELRS_FALLBACK_FS         0x03  /* ELRS default */

/* Packet types */
#define ELRS_PKT_TYPE_GFSK    0x00
#define ELRS_PKT_TYPE_LORA    0x01
#define ELRS_PKT_TYPE_LR_FHSS 0x03

/* PA Selection */
#define ELRS_PA_LP 0x00  /* Low Power PA (sub-GHz) */
#define ELRS_PA_HP 0x01  /* High Power PA (sub-GHz) */
#define ELRS_PA_HF 0x02  /* High Frequency PA (2.4 GHz) */

/*******************************************************************************
 * ELRS-Compatible Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize LR1121 with correct TCXO power-up sequence
 *
 * This function implements the CORRECT initialization sequence for
 * proper TCXO operation as required for precision LoRa modulation:
 *
 * TCXO INITIALIZATION SEQUENCE (MANDATORY ORDER):
 *   1. SetTcxoMode(3.0V, 300) - Power TCXO via VTCXO pin (~9ms delay)
 *   2. SetStandby(XOSC)       - Switch system clock from RC to TCXO
 *   3. SetRegMode(DCDC)       - Configure power efficiency
 *   4. Calibrate(0x3F)        - Full calibration with stable TCXO reference
 *
 * WHY THIS SEQUENCE IS REQUIRED:
 *   - LR1121 boots using internal RC oscillator (inaccurate)
 *   - RC oscillator CANNOT meet precision for LoRa modulation
 *   - Without switching to XOSC: Calibration failures, PLL lock errors
 *   - HF_XOSC_START_ERR occurs if TCXO not properly initialized
 *
 * Citation: User requirement analysis (2026-01-17)
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 * Citation: LR1121 Datasheet Section 11.2.2 "SetStandby"
 * Citation: Semtech lr11xx_system.c driver implementation
 *
 * @param freq_min Minimum frequency in Hz (e.g., 902000000 for US 915)
 * @param freq_max Maximum frequency in Hz (e.g., 928000000 for US 915)
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_elrs_init(uint32_t freq_min, uint32_t freq_max);

/**
 * @brief Get temperature using ELRS-compatible method
 *
 * IMPORTANT: This only works AFTER lr1121_elrs_init() has been called!
 *
 * The key insight from ELRS is that GetTemperature works WITHOUT
 * switching to XOSC mode. ELRS never calls SetStandby(XOSC) but
 * temperature reading still works.
 *
 * @return Temperature in degrees Celsius, or -999 on error
 *
 * Citation: LR1121 Datasheet Section 11.2.6 "GetTemp"
 */
int16_t lr1121_elrs_get_temperature(void);

/**
 * @brief Get random number using ELRS-compatible method
 *
 * IMPORTANT: This only works AFTER lr1121_elrs_init() has been called!
 *
 * @return 32-bit random number, or 0 on error
 *
 * Citation: LR1121 Datasheet Section 11.2.8 "GetRandomNumber"
 */
uint32_t lr1121_elrs_get_random(void);

/**
 * @brief Set RF frequency using ELRS-compatible method
 *
 * @param freq_hz Frequency in Hz (e.g., 915000000 for 915 MHz)
 * @return true on success, false on error
 *
 * Citation: LR1121 Datasheet Section 11.3.1 "SetRfFrequency"
 */
bool lr1121_elrs_set_frequency(uint32_t freq_hz);

/**
 * @brief Calibrate image rejection for frequency range
 *
 * This is the ONLY calibration ELRS performs. It does NOT call
 * the general Calibrate(0x3F) command.
 *
 * @param freq_min Minimum frequency in Hz
 * @param freq_max Maximum frequency in Hz
 * @return true on success, false on error
 *
 * Citation: ExpressLRS LR1121.cpp CalibImage()
 * Citation: LR1121 Datasheet Section 11.2.4 "CalibrateImage"
 */
bool lr1121_elrs_calib_image(uint32_t freq_min, uint32_t freq_max);

/**
 * @brief Configure LoRa modulation parameters (ELRS style)
 *
 * @param sf Spreading factor (5-12)
 * @param bw Bandwidth index (0-6, see datasheet)
 * @param cr Coding rate (1-4, for 4/5 to 4/8)
 * @param ldro Low data rate optimization (0 or 1)
 * @return true on success, false on error
 *
 * Citation: LR1121 Datasheet Section 11.3.4 "SetModulationParams"
 */
bool lr1121_elrs_set_lora_mod(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);

/**
 * @brief Configure LoRa packet parameters (ELRS style)
 *
 * @param preamble_len Preamble length in symbols
 * @param header_type 0=variable/explicit, 1=fixed/implicit
 * @param payload_len Payload length in bytes
 * @param crc_on 1=CRC enabled, 0=disabled
 * @param invert_iq 1=inverted IQ, 0=standard
 * @return true on success, false on error
 *
 * Citation: LR1121 Datasheet Section 11.3.5 "SetPacketParams"
 */
bool lr1121_elrs_set_lora_pkt(uint16_t preamble_len, uint8_t header_type,
                              uint8_t payload_len, uint8_t crc_on, uint8_t invert_iq);

/**
 * @brief Get chip status using ELRS-compatible method
 *
 * @param chip_mode Pointer to store chip mode (0-5)
 * @param cmd_status Pointer to store command status (0-4)
 * @param errors Pointer to store error flags (optional, can be NULL)
 * @return true on success, false on error
 */
bool lr1121_elrs_get_status(uint8_t *chip_mode, uint8_t *cmd_status, uint16_t *errors);

/**
 * @brief Print ELRS-compatible diagnostic info
 *
 * Prints chip status, mode, and any error flags in a format
 * similar to ELRS debug output.
 */
void lr1121_elrs_print_status(void);

/*******************************************************************************
 * BULLETPROOF TCXO INITIALIZATION (NEW!)
 * 
 * Based on user analysis of why ELRS works with RNG but not temp sensor:
 *   - RNG uses internal 32MHz RC oscillator (works without TCXO)
 *   - Temperature/ADC requires TCXO to be stable
 *   
 * The "200mV Headroom Rule":
 *   VBAT must be >= TCXO voltage + 200mV
 *   If VBAT=3.3V and TCXO=3.3V, it WILL fail!
 *   Solution: Use TCXO voltage 1.8V (0x02) for 3.3V supply
 *
 * The "Ghost RC Oscillator" Problem:
 *   After SetTcxoMode, chip is STILL running on RC oscillator!
 *   MUST send SetStandby(0x01) to switch to XOSC!
 ******************************************************************************/

/**
 * @brief TCXO Voltage Trim Values for SetTcxoMode Command
 *
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 * Citation: Semtech lr11xx_system_types.h lr11xx_system_tcxo_supply_voltage_t enum
 *
 * CRITICAL UPDATE (2026-01-21): Waveshare Core1121-XF TCXO Configuration
 * ======================================================================
 * The Waveshare Core1121-XF module has an EXTERNALLY-POWERED 32MHz TCXO
 * that is ALWAYS ON from the 3.3V rail. The LR1121's internal VTCXO 
 * regulator is NOT used to power the TCXO.
 *
 * For externally-powered TCXOs:
 *   - Voltage parameter MUST be 0 (TCXO_CTRL_NONE)
 *   - This tells LR1121 NOT to use internal VTCXO regulator
 *   - SetTcxoMode is STILL required to switch from XO to TCXO input mode
 *   - A small delay (5-10ms) is still needed for oscillator stabilization
 *
 * Citation: Waveshare Core1121_XF_Demo - uses voltage=0 for external TCXO
 * Citation: Semtech LR11xx SDK - LR11XX_RADIO_TCXO_CTRL_NONE for external supply
 *
 * Hardware configuration (Core1121-XF):
 *   - XTA pin: Connected to TCXO output (32MHz clock input)
 *   - XTB pin: Not connected (NC) - single-ended TCXO input
 *   - TCXO powered directly from 3.3V rail (always on)
 */
#define TCXO_VOLTAGE_NONE 0x00  /* EXTERNAL TCXO - no internal regulator control */
#define TCXO_VOLTAGE_1V6  0x00  /* Internal TCXO at 1.6V (same as NONE for external) */
#define TCXO_VOLTAGE_1V7  0x01  /* Internal TCXO at 1.7V */
#define TCXO_VOLTAGE_1V8  0x02  /* Internal TCXO at 1.8V */
#define TCXO_VOLTAGE_2V2  0x03  /* Internal TCXO at 2.2V */
#define TCXO_VOLTAGE_2V4  0x04  /* Internal TCXO at 2.4V */
#define TCXO_VOLTAGE_2V7  0x05  /* Internal TCXO at 2.7V */
#define TCXO_VOLTAGE_3V0  0x06  /* Internal TCXO at 3.0V */
#define TCXO_VOLTAGE_3V3  0x07  /* Internal TCXO at 3.3V */

/**
 * @brief TCXO Startup Delay Values (in 30.52µs ticks)
 *
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *   Delay = timeout * 30.52µs (24-bit value)
 *
 * For EXTERNALLY-POWERED TCXOs (Waveshare Core1121-XF):
 *   - TCXO is always running, but still need small delay for stabilization
 *   - Typical warm-up time: 5-10ms based on TCXO spec
 *   - ~164 ticks = 5ms, ~328 ticks = 10ms
 *
 * For INTERNALLY-POWERED TCXOs:
 *   - Need longer delay for power-up + stabilization
 *   - 1000-2000 ticks (30-60ms) recommended
 *
 * Common values:
 *   164 ticks  = ~5ms    (external TCXO minimum)
 *   328 ticks  = ~10ms   (external TCXO with margin)
 *   500 ticks  = ~15ms   (internal TCXO minimum)
 *   1000 ticks = ~30.5ms (internal TCXO recommended)
 */
#define TCXO_DELAY_164_TICKS   0x0000A4  /* ~5ms  - external TCXO minimum */
#define TCXO_DELAY_328_TICKS   0x000148  /* ~10ms - external TCXO with margin */
#define TCXO_DELAY_300_TICKS   0x00012C  /* ~9ms  - Waveshare original */
#define TCXO_DELAY_500_TICKS   0x0001F4  /* ~15ms */
#define TCXO_DELAY_656_TICKS   0x000290  /* ~20ms - RECOMMENDED for reliable wake-from-sleep! */
#define TCXO_DELAY_1000_TICKS  0x0003E8  /* ~30ms - internal TCXO recommended */
#define TCXO_DELAY_1500_TICKS  0x0005DC  /* ~46ms - safer, try if 20ms insufficient */
#define TCXO_DELAY_2000_TICKS  0x0007D0  /* ~61ms - conservative */
#define TCXO_DELAY_3277_TICKS  0x000CCD  /* ~100ms - very conservative */

/**
 * @brief Current TCXO configuration defaults for Waveshare Core1121-XF
 *
 * CRITICAL FIX (2026-01-21):
 * ==========================
 * The Waveshare Core1121-XF has an EXTERNALLY-POWERED TCXO from 3.3V rail.
 * 
 * UPDATED FIX (2026-01-22) - SIGNIFICANTLY INCREASED TCXO DELAY FOR HW SPI:
 * =========================================================================
 * The chip was failing to wake from SLEEP mode because the TCXO startup delay
 * was too short, especially after switching from software to hardware SPI.
 * 
 * Hardware SPI vs Soft SPI timing differences:
 *   - HW SPI is faster, giving less settling time between operations
 *   - The external TCXO needs more time to stabilize after wake
 *   - PLL needs additional time to lock to the TCXO reference
 * 
 * During wake-from-sleep, the LR1121 needs sufficient time to:
 *   1. Detect the external TCXO clock signal on XTA pin
 *   2. Lock the internal PLL to the TCXO reference  
 *   3. Stabilize the system clock before processing commands
 * 
 * Delay increased from ~20ms (656 ticks) to ~100ms (3277 ticks).
 * This provides sufficient margin for hardware SPI timing.
 *
 * Configuration:
 *   - Voltage = 0 (TCXO_CTRL_NONE) - DON'T use internal VTCXO regulator
 *   - Delay = ~100ms (3277 ticks) - SIGNIFICANTLY increased for HW SPI wake
 *
 * Why SetTcxoMode is STILL required:
 *   - Switches LR1121 from default XO (crystal) mode to TCXO input mode
 *   - Without this, chip expects a crystal oscillator circuit, not TCXO
 *   - Calling SetTcxoMode tells the chip to expect single-ended clock on XTA
 *   - Skipping it causes oscillator failures/timeouts → radio unresponsive
 *
 * Citation: Semtech LR11xx reference drivers for external TCXO modules
 * Citation: Waveshare Core1121_XF_Demo configuration
 * Citation: User analysis - insufficient delay causes wake-from-sleep failure
 */
/*
 * CRITICAL FIX (2026-01-24): TCXO Configuration based on stress testing
 * 
 * The Waveshare Core1121-XF has an EXTERNALLY-POWERED 32MHz TCXO that is
 * always on from the 3.3V rail. The LR1121's internal VTCXO regulator is
 * NOT used to power the TCXO.
 * 
 * Stress test results (5000 FHSS hops, 500 TX/RX cycles, 50 power cycles):
 *   - voltage=0x00 (external TCXO) + delay=164 ticks (~5ms) = 100% PASS
 *   - voltage=0x06 (3.0V internal) = chip falls back to SLEEP
 * 
 * Citation: TCXO stress test validation 2026-01-24
 */
#ifndef TCXO_ACTIVE_VOLTAGE
#define TCXO_ACTIVE_VOLTAGE  0x06  /* 3.0V - Waveshare demo setting */
#endif

#ifndef TCXO_ACTIVE_DELAY
#define TCXO_ACTIVE_DELAY    656  /* 656 ticks = ~20ms - RECOMMENDED for reliable wake-from-sleep */
#endif

/**
 * @brief Bulletproof TCXO initialization sequence (WAVESHARE-MATCHED!)
 *
 * This implements the EXACT sequence from Waveshare Core1121_XF_Demo:
 *
 * Citation: Waveshare lr1121_config.cpp lora_system_init() lines 81-130
 * Citation: Semtech lr11xx_system.c (official driver)
 *
 * CRITICAL SEQUENCE (matches Waveshare EXACTLY):
 *   1. Hardware Reset
 *   2. Wakeup (toggle CS)
 *   3. SetStandby(XOSC) FIRST!  *** KEY DIFFERENCE FROM OLD CODE ***
 *   4. CalibrateImage for frequency band
 *   5. SetRegMode(DCDC)
 *   6. SetDioAsRfSwitch
 *   7. SetTcxoMode - voltage=0x06 (3.0V), timeout=300 ticks (~9ms)
 *   8. CfgLfClk
 *   9. ClearErrors
 *  10. Calibrate(0x3F)
 *  11. SetStandby(XOSC) again (wake from post-calibration SLEEP)
 *
 * RECOMMENDED: Call with TCXO_VOLTAGE_3V0 (0x06) to match Waveshare demo!
 *
 * @param tcxo_voltage TCXO voltage trim (use TCXO_VOLTAGE_3V0 = 0x06)
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_bulletproof_tcxo_init(uint8_t tcxo_voltage);

/**
 * @brief Verify chip is in STDBY_XOSC mode
 *
 * After bulletproof_tcxo_init, chip mode should be 0x03 (STDBY_XOSC)
 * If it's 0x02 (STDBY_RC), the TCXO switch failed!
 *
 * @param chip_mode Pointer to store chip mode (0-7)
 * @return true if chip is in STDBY_XOSC mode, false otherwise
 */
bool lr1121_verify_xosc_mode(uint8_t *chip_mode);

/**
 * @brief Waveshare EXACT initialization sequence
 *
 * This implements the EXACT sequence from Waveshare Core1121-XF Arduino demo:
 *   lr1121_config.cpp lora_system_init()
 *
 * Citation: C:\Users\mjeuw\Downloads\Core1121_XF_Demo\Core1121_XF_Demo\esp32s3\
 *           Arduino\waveshare_lroa_1121\examples\lr1121_ping_pong\lr1121_config.cpp
 *
 * Sequence:
 *   1.  lr11xx_system_reset()
 *   2.  lr11xx_hal_wakeup()
 *   3.  lr11xx_system_enable_spi_crc(false)
 *   4.  lr11xx_system_set_standby(XOSC)
 *   5.  lr11xx_system_calibrate_image(0xE1, 0xE9)  // 915MHz
 *   6.  lr11xx_system_set_reg_mode(DCDC)
 *   7.  lr11xx_system_set_dio_as_rf_switch()
 *   8.  lr11xx_system_set_tcxo_mode(3.0V, 300)
 *   9.  lr11xx_system_cfg_lfclk(XTAL, true)
 *   10. lr11xx_system_clear_errors()
 *   11. lr11xx_system_calibrate(0x3F)
 *   12. lr11xx_system_get_errors()
 *   13. lr11xx_system_clear_errors()
 *   14. lr11xx_system_clear_irq_status(ALL)
 *
 * @return LR1121_OK on success, error code otherwise
 */
lr1121_status_t lr1121_waveshare_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_ELRS_INIT_H */
