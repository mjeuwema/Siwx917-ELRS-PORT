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

namespace {
constexpr size_t kSpiScratchSize = 512;
uint8_t gTransferWriteDummy[kSpiScratchSize];
uint8_t gTransferReadBuf[kSpiScratchSize];
uint8_t gSpiexWriteDummy[kSpiScratchSize];
} // namespace

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

void SPIClass::transferBytes(const uint8_t *tx, uint8_t *rx, size_t count) {
  if (count == 0)
    return;

  if (rx != nullptr) {
    // Full duplex: separate tx and rx buffers
    lr1121_spi_transfer(tx, rx, count);
  } else {
    // Write-only: use a shared dummy buffer so tx data is NOT overwritten
    // without putting a large scratch buffer on the task stack.
    if (count <= sizeof(gTransferWriteDummy)) {
      lr1121_spi_transfer(tx, gTransferWriteDummy, count);
    } else {
      for (size_t i = 0; i < count; i++) {
        uint8_t d = 0;
        lr1121_spi_transfer(&tx[i], &d, 1);
      }
    }
  }
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

void SPIExClass::_transfer(uint8_t cs_mask, uint8_t *data, uint32_t size,
                           bool reading) {
  // cs_mask indicates which radio(s) to select:
  // SX12XX_Radio_1 = 0x01, SX12XX_Radio_2 = 0x02, SX12XX_Radio_All = 0x03
  // For SiW917 we only support Radio_1
  (void)cs_mask;

  if (size == 0)
    return;

  // Assert CS
  lr1121_cs_assert();

  if (reading) {
    // Read operation: send data as dummy, receive response
    if (size <= sizeof(gTransferReadBuf)) {
      lr1121_spi_transfer(data, gTransferReadBuf, size);
      memcpy(data, gTransferReadBuf, size);
    } else {
      for (uint32_t i = 0; i < size; i++) {
        uint8_t rx = 0;
        lr1121_spi_transfer(&data[i], &rx, 1);
        data[i] = rx;
      }
    }
  } else {
    // Write operation: send data, ignore response
    if (size <= sizeof(gSpiexWriteDummy)) {
      lr1121_spi_transfer(data, gSpiexWriteDummy, size);
    } else {
      for (uint32_t i = 0; i < size; i++) {
        uint8_t rx = 0;
        lr1121_spi_transfer(&data[i], &rx, 1);
      }
    }
  }

  // Deassert CS
  lr1121_cs_deassert();
}
