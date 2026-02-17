/**
 * @file SPIEx.cpp
 * @brief ELRS SPIEx implementation for SiW917
 * 
 * Uses the lr1121_driver.c SPI functions for actual hardware access.
 */

#include "SPIEx.h"
#include "logging.h"
#include <string.h>

// Include our C SPI driver
extern "C" {
#include "lr1121_driver.h"
}

// Global instances
SPIClass SPI;
SPIExClass SPIEx;

//-----------------------------------------------------------------------------
// SPIClass Implementation
//-----------------------------------------------------------------------------

uint8_t SPIClass::transfer(uint8_t data) {
    uint8_t rx = 0;
    lr1121_spi_transfer(&data, &rx, 1);
    return rx;
}

void SPIClass::transfer(void *buf, size_t count) {
    // In-place transfer
    uint8_t *data = (uint8_t *)buf;
    lr1121_spi_transfer(data, data, count);
}

//-----------------------------------------------------------------------------
// SPIExClass Implementation
//-----------------------------------------------------------------------------

void SPIExClass::write(uint8_t cs_mask, uint8_t *data, uint32_t size) {
    _transfer(cs_mask, data, size, false);
}

void SPIExClass::read(uint8_t cs_mask, uint8_t *data, uint32_t size) {
    _transfer(cs_mask, data, size, true);
}

void SPIExClass::_transfer(uint8_t cs_mask, uint8_t *data, uint32_t size, bool reading) {
    // cs_mask indicates which radio(s) to select:
    // SX12XX_Radio_1 = 0x01, SX12XX_Radio_2 = 0x02, SX12XX_Radio_All = 0x03
    // For SiW917 we only support Radio_1
    
    if (size == 0) return;
    
    // Assert CS
    lr1121_cs_assert();
    
    if (reading) {
        // Read operation: send data as dummy, receive response
        uint8_t rx_buf[64];
        if (size <= sizeof(rx_buf)) {
            lr1121_spi_transfer(data, rx_buf, size);
            memcpy(data, rx_buf, size);
        } else {
            // Larger transfers: do in chunks
            for (uint32_t i = 0; i < size; i++) {
                uint8_t rx;
                lr1121_spi_transfer(&data[i], &rx, 1);
                data[i] = rx;
            }
        }
    } else {
        // Write operation: send data, ignore response
        uint8_t dummy[64];
        if (size <= sizeof(dummy)) {
            lr1121_spi_transfer(data, dummy, size);
        } else {
            for (uint32_t i = 0; i < size; i++) {
                uint8_t rx;
                lr1121_spi_transfer(&data[i], &rx, 1);
            }
        }
    }
    
    // Deassert CS
    lr1121_cs_deassert();
}
