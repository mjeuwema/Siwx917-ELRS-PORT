/**
 * @file lr1121_standalone_test.h
 * @brief LR1121 Standalone Test Suite Header
 *
 * Test suite for LR1121 that doesn't require a second receiver.
 * Tests SPI communication, RF configuration, TX/RX functionality,
 * and frequency hopping using internal chip features.
 */

#ifndef LR1121_STANDALONE_TEST_H
#define LR1121_STANDALONE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run comprehensive LR1121 diagnostic using GetStatus and GetErrors
 *
 * This function diagnoses why GetRandomNumber or GetTemperature might fail.
 * It uses the GetStatus (0x0100) and GetErrors (0x010D) commands as specified
 * in the LR1121 User Manual Sections 3.4 and 3.6.
 *
 * Key diagnostics:
 * - GetStatus: Returns chip mode, reset status, command status
 * - GetErrors: Returns calibration and XOSC error flags
 *
 * Common issues diagnosed:
 * - Chip in SLEEP mode (needs STANDBY_XOSC for temp/random)
 * - HF_XOSC_START_ERR (TCXO not configured properly)
 * - PLL_LOCK_ERR (oscillator not stable)
 *
 * Citation: LR1121 User Manual Sections 3.4.1, 3.4.2, 3.4.3, 3.6.1
 */
void lr1121_run_diagnostic(void);

/**
 * @brief Run all standalone tests
 *
 * This function runs the complete test suite:
 * 1. Hardware Verification (GetVersion, GetStatus, Temperature)
 * 2. RF Configuration (Frequency, Modulation, PA)
 * 3. CW Transmission (5 seconds for SDR verification)
 * 4. TX Packet with TX_DONE IRQ verification
 * 5. RX Mode with RSSI Noise Floor measurement
 * 6. Frequency Hopping verification
 *
 * Call this after lr1121_init() has been successfully completed.
 *
 * Without a second receiver, these tests verify:
 * - SPI communication is working
 * - Chip responds to all commands correctly
 * - TX produces TX_DONE interrupt (packet transmitted)
 * - RX mode can measure RSSI (receiver functional)
 * - Frequency changes work (essential for ELRS)
 *
 * To fully verify RF output, use an SDR receiver (RTL-SDR, HackRF)
 * or spectrum analyzer tuned to 915 MHz (or your test frequency).
 */
void lr1121_run_standalone_tests(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_STANDALONE_TEST_H */
