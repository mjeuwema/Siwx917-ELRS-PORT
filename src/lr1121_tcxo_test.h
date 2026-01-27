/**
 * @file lr1121_tcxo_test.h
 * @brief LR1121 TCXO Initialization Standalone Test Suite
 *
 * Focused test suite for debugging TCXO/oscillator startup issues
 * on the Waveshare Core1121-XF module (externally-powered TCXO).
 *
 * Hardware: SiWG917Y (BRD2708A) + Waveshare Core1121-XF on mikroBUS
 *
 * Test Categories:
 *   1. SPI Communication Baseline - Verify basic communication
 *   2. TCXO Voltage Sweep - Find working voltage settings
 *   3. TCXO Delay Sweep - Find minimum reliable delay
 *   4. Full Init Validation - Complete sequence with temp reading
 *   5. Stress Test - Verify reliability over multiple cycles
 *
 * Citations:
 *   - LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *   - Waveshare Core1121_XF_Demo configuration
 *   - Semtech lr11xx_system.c reference driver
 */

#ifndef LR1121_TCXO_TEST_H
#define LR1121_TCXO_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * TCXO Voltage Values to Test
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 ******************************************************************************/
#define TCXO_TEST_VOLTAGE_NONE   0x00  /* External TCXO - no internal regulator */
#define TCXO_TEST_VOLTAGE_1V8    0x02  /* 1.8V - common for internal TCXOs */
#define TCXO_TEST_VOLTAGE_3V0    0x06  /* 3.0V - Waveshare demo uses this */
#define TCXO_TEST_VOLTAGE_3V3    0x07  /* 3.3V - max voltage */

/*******************************************************************************
 * TCXO Delay Values to Test (in 30.52µs ticks)
 * Citation: LR1121 Datasheet Section 11.2.5
 *   delay_ms = ticks * 30.52 / 1000
 ******************************************************************************/
#define TCXO_TEST_DELAY_5MS      164   /* ~5ms - minimum for external TCXO */
#define TCXO_TEST_DELAY_9MS      300   /* ~9ms - Waveshare demo default */
#define TCXO_TEST_DELAY_15MS     500   /* ~15ms */
#define TCXO_TEST_DELAY_30MS     1000  /* ~30ms - conservative */
#define TCXO_TEST_DELAY_60MS     2000  /* ~60ms - very conservative */

/*******************************************************************************
 * LR1121 Command Opcodes (subset needed for TCXO testing)
 * Citation: LR1121 Datasheet Section 11 "Command Reference"
 ******************************************************************************/
#define TCXO_CMD_GET_STATUS       0x0100
#define TCXO_CMD_GET_VERSION      0x0101
#define TCXO_CMD_GET_ERRORS       0x010D
#define TCXO_CMD_CLEAR_ERRORS     0x010E
#define TCXO_CMD_CALIBRATE        0x010F
#define TCXO_CMD_SET_REG_MODE     0x0110
#define TCXO_CMD_SET_TCXO_MODE    0x0117
#define TCXO_CMD_GET_TEMP         0x011A
#define TCXO_CMD_SET_STANDBY      0x011C
#define TCXO_CMD_SET_RF_FREQ      0x0203  /* SetRfFrequency - forces PLL lock */
#define TCXO_CMD_SET_PACKET_TYPE  0x0201  /* SetPacketType (LoRa/FSK) */
#define TCXO_CMD_SET_RX           0x0209  /* SetRx - enter receive mode */
#define TCXO_CMD_SET_FS           0x020D  /* SetFs - frequency synthesis mode */

/*******************************************************************************
 * LR1121 Error Flags
 * Citation: LR1121 Datasheet Section 3.6.1 "GetErrors"
 ******************************************************************************/
#define TCXO_ERR_HF_XOSC_START    0x0020  /* HF XOSC failed to start */
#define TCXO_ERR_PLL_CALIB        0x0040  /* PLL calibration error */
#define TCXO_ERR_PLL_LOCK         0x0080  /* PLL failed to lock */
#define TCXO_ERR_ADC_CALIB        0x0100  /* ADC calibration error */
#define TCXO_ERR_IMG_CALIB        0x0200  /* Image rejection calibration error */

/*******************************************************************************
 * Chip Mode Values
 * Citation: LR1121 Datasheet Section 3.4.2 "stat2"
 ******************************************************************************/
#define TCXO_MODE_SLEEP           0
#define TCXO_MODE_STDBY_RC        1
#define TCXO_MODE_STDBY_XOSC      2
#define TCXO_MODE_FS              3
#define TCXO_MODE_RX              4
#define TCXO_MODE_TX              5

/*******************************************************************************
 * Test Result Structure
 ******************************************************************************/
typedef struct {
    uint8_t  voltage;           /* TCXO voltage that was tested */
    uint16_t delay_ticks;       /* TCXO delay that was tested */
    bool     set_tcxo_ok;       /* SetTcxoMode succeeded */
    bool     set_standby_ok;    /* SetStandby(XOSC) succeeded */
    uint8_t  chip_mode;         /* Resulting chip mode */
    uint16_t errors;            /* Error flags from GetErrors */
    bool     passed;            /* Overall pass/fail */
} tcxo_test_result_t;

/*******************************************************************************
 * Test Configuration
 ******************************************************************************/
#define TCXO_STRESS_TEST_CYCLES  10  /* Number of init cycles for stress test */

/*******************************************************************************
 * Public Function Declarations
 ******************************************************************************/

/**
 * @brief Run all TCXO tests
 *
 * Executes the complete test suite:
 *   1. SPI baseline check
 *   2. Voltage sweep
 *   3. Delay sweep (using best voltage)
 *   4. Full init validation
 *   5. Stress test
 *
 * Results are printed to serial output.
 */
void lr1121_tcxo_test_run(void);

/**
 * @brief Test 1: Verify SPI communication baseline
 *
 * Performs:
 *   - Hardware reset
 *   - GetVersion to verify chip responds
 *   - SetStandby(RC) to verify commands work
 *
 * @return true if SPI communication working, false otherwise
 */
bool lr1121_tcxo_test_spi_baseline(void);

/**
 * @brief Test 2: Sweep TCXO voltage values
 *
 * Tests voltage values: 0x00, 0x02, 0x06, 0x07
 * For each voltage:
 *   - Reset chip
 *   - SetTcxoMode with test voltage
 *   - SetStandby(XOSC)
 *   - Check chip mode and errors
 *
 * @param results Array to store results (min 4 elements)
 * @param num_results Pointer to store number of results
 * @return Best working voltage, or 0xFF if none work
 */
uint8_t lr1121_tcxo_test_voltage_sweep(tcxo_test_result_t *results, uint8_t *num_results);

/**
 * @brief Test 3: Sweep TCXO delay values
 *
 * Tests delay values: 164, 300, 500, 1000, 2000 ticks
 * Uses specified voltage for all tests.
 *
 * @param voltage TCXO voltage to use (from voltage sweep)
 * @param results Array to store results (min 5 elements)
 * @param num_results Pointer to store number of results
 * @return Minimum working delay, or 0xFFFF if none work
 */
uint16_t lr1121_tcxo_test_delay_sweep(uint8_t voltage, tcxo_test_result_t *results, uint8_t *num_results);

/**
 * @brief Test 4: Full initialization sequence validation
 *
 * Runs complete init sequence with best voltage/delay:
 *   - SetTcxoMode
 *   - SetStandby(XOSC)
 *   - Calibrate(0x3F)
 *   - GetTemperature (validates TCXO stable)
 *
 * @param voltage TCXO voltage to use
 * @param delay_ticks TCXO delay to use
 * @param temperature Pointer to store temperature reading (or NULL)
 * @return true if full init successful and temperature readable
 */
bool lr1121_tcxo_test_full_init(uint8_t voltage, uint16_t delay_ticks, int16_t *temperature);

/**
 * @brief Test 5: Stress test - multiple init cycles
 *
 * Runs the full init sequence multiple times to check reliability.
 *
 * @param voltage TCXO voltage to use
 * @param delay_ticks TCXO delay to use
 * @param cycles Number of test cycles
 * @return Number of successful cycles
 */
uint8_t lr1121_tcxo_test_stress(uint8_t voltage, uint16_t delay_ticks, uint8_t cycles);

/**
 * @brief Print test result details
 *
 * @param result Test result to print
 */
void lr1121_tcxo_print_result(const tcxo_test_result_t *result);

/**
 * @brief Decode chip mode to string
 *
 * @param mode Chip mode value (0-5)
 * @return String description of mode
 */
const char* lr1121_tcxo_mode_str(uint8_t mode);

/**
 * @brief Decode error flags to string
 *
 * @param errors Error flags from GetErrors
 * @param buf Buffer to store string (min 128 bytes)
 * @param buf_len Buffer length
 */
void lr1121_tcxo_errors_str(uint16_t errors, char *buf, uint16_t buf_len);

/*******************************************************************************
 * Extended Stress Tests for ELRS Integration Validation
 ******************************************************************************/

/**
 * @brief Run all extended stress tests
 * 
 * Comprehensive test suite including:
 *   - Temperature stability
 *   - FHSS frequency hopping (5000 hops)
 *   - TX/RX mode cycling (500 cycles)
 *   - Long duration PLL lock (30 seconds)
 *   - Power cycle stress (50 cycles)
 */
void lr1121_tcxo_stress_test_all(void);

/**
 * @brief FHSS (Frequency Hopping) Stress Test
 * 
 * Simulates ELRS frequency hopping pattern across 915MHz band.
 * 
 * @param num_hops Number of frequency hops to perform
 * @param delay_us Delay between hops in microseconds
 * @return Number of successful hops
 */
uint32_t lr1121_tcxo_stress_fhss(uint32_t num_hops, uint32_t delay_us);

/**
 * @brief Temperature Stability Test
 * 
 * Takes multiple temperature readings to verify ADC/TCXO stability.
 * 
 * @param num_readings Number of temperature readings
 * @return true if temperature readings are stable (range < 5C)
 */
bool lr1121_tcxo_stress_temperature(uint16_t num_readings);

/**
 * @brief TX/RX Mode Cycling Test
 * 
 * Rapidly cycles between TX and RX modes.
 * 
 * @param num_cycles Number of TX/RX cycles
 * @return Number of successful cycles
 */
uint32_t lr1121_tcxo_stress_txrx_cycle(uint32_t num_cycles);

/**
 * @brief Long Duration PLL Lock Test
 * 
 * Holds PLL locked and monitors for unlock events.
 * 
 * @param duration_sec Duration in seconds
 * @param check_interval_ms How often to check status (ms)
 * @return true if PLL remained locked
 */
bool lr1121_tcxo_stress_pll_duration(uint16_t duration_sec, uint16_t check_interval_ms);

/**
 * @brief Power Cycle Stress Test
 * 
 * Repeatedly resets chip and re-initializes TCXO.
 * 
 * @param num_cycles Number of power cycles
 * @return Number of successful initializations
 */
uint32_t lr1121_tcxo_stress_power_cycle(uint32_t num_cycles);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_TCXO_TEST_H */
