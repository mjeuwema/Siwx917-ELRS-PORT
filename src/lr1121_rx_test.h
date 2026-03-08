/**
 * @file lr1121_rx_test.h
 * @brief Standalone LR1121 RX test - bypasses ELRS C++ stack entirely
 *
 * This test isolates whether the LR1121 radio can actually receive packets
 * by using the proven C driver directly. It tests 3 layers independently:
 *   1. Does the LR1121 enter RX mode? (SPI GetStatus check)
 *   2. Does the LR1121 set IRQ flags when a packet arrives? (SPI IRQ poll)
 *   3. Does GPIO_46 (DIO9) go HIGH? (Direct pin read)
 *
 * Usage: Call lr1121_rx_test_run() from gspi_example.c after waveshare init.
 *        Must have ELRS TX transmitting on 915.5 MHz.
 */

#ifndef LR1121_RX_TEST_H
#define LR1121_RX_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the standalone RX test
 *
 * This function:
 *   1. Configures LoRa RX on 915.5 MHz (SF7, BW500, CR4/5)
 *   2. Sets DIO IRQ params to route RX_DONE to DIO9
 *   3. Enters continuous RX mode
 *   4. Polls for 30 seconds, printing:
 *      - GPIO_46 pin state (raw DIO9 read)
 *      - SPI IRQ status register
 *      - Chip mode (should be 4 = RX)
 *      - Any received packet payload
 */
void lr1121_rx_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* LR1121_RX_TEST_H */
