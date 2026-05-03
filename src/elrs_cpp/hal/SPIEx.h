/**
 * @file SPIEx.h
 * @brief ELRS SPIEx interface for SiW917
 * 
 * Provides the same SPI interface that ELRS expects.
 * Uses our existing lr1121_driver.c for actual SPI operations.
 */

#pragma once

#include "targets.h"
#include "../lib/SX12xxDriverCommon/SX12xxDriverCommon.h"

/**
 * @brief Standard Arduino SPI class stub
 */
class SPIClass {
public:
    void begin() {}
    void end() {}
    void beginTransaction() {}
    void endTransaction() {}
    void setBitOrder(uint8_t order) { _bitOrder = order; }
    void setDataMode(uint8_t mode) { _dataMode = mode; }
    void setFrequency(uint32_t freq) { _frequency = freq; }
    void setHwCs(bool use) { _hwCs = use; }
    
    uint8_t transfer(uint8_t data);
    void transfer(void *buf, size_t count);
    void transferBytes(const uint8_t *tx, uint8_t *rx, size_t count);

private:
    uint8_t _bitOrder = MSBFIRST;
    uint8_t _dataMode = SPI_MODE0;
    uint32_t _frequency = 16000000;
    bool _hwCs = false;
};

extern SPIClass SPI;

/**
 * @brief ELRS SPIEx class for dual-radio support
 * 
 * SiW917 implementation only supports single radio (Radio_1)
 */
class SPIExClass : public SPIClass {
public:
    /**
     * @brief Write data to selected radio(s)
     * @param cs_mask Radio selection (SX12XX_Radio_1, _2, or _All)
     * @param data Data buffer to write
     * @param size Number of bytes
     */
    void write(uint8_t cs_mask, uint8_t *data, uint32_t size);
    
    /**
     * @brief Read data from selected radio
     * @param cs_mask Radio selection
     * @param data Full-duplex buffer: contents are sent on MOSI and replaced
     *             with MISO bytes, matching upstream ELRS SPIEx semantics.
     * @param size Number of bytes
     */
    void read(uint8_t cs_mask, uint8_t *data, uint32_t size);
    
private:
    void _transfer(uint8_t cs_mask, uint8_t *data, uint32_t size, bool reading);
};

extern SPIExClass SPIEx;
