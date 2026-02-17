/**
 * @file Arduino.h
 * @brief Arduino API compatibility layer for SiW917
 *
 * Provides Arduino-like functions for the ELRS codebase to use on SiW917.
 * This allows upstream ELRS C++ code to compile with minimal changes.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//=============================================================================
// Arduino Pin Definitions
//=============================================================================

#define HIGH 1
#define LOW  0

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

#define RISING  0x01
#define FALLING 0x02
#define CHANGE  0x03

// Undefined pin marker
#define UNDEF_PIN (-1)

// SPI modes
#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define MSBFIRST 1
#define LSBFIRST 0

//=============================================================================
// Arduino Math Utilities
//=============================================================================

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//=============================================================================
// C Function Declarations
//=============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// Timing functions
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void yield(void);

// GPIO functions
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
void attachInterrupt(int pin, void (*callback)(void), int mode);
void detachInterrupt(int pin);

#define digitalPinToInterrupt(pin) (pin)

#ifdef __cplusplus
} // extern "C"
#endif

//=============================================================================
// C++ Only Section
//=============================================================================

#ifdef __cplusplus

#include <cstdio>
#include <algorithm>

// Use std::min and std::max for C++
using std::min;
using std::max;

/**
 * @brief Minimal Stream class for logging
 */
class Stream {
public:
    virtual ~Stream() = default;
    virtual size_t write(uint8_t c) { return 0; }
    virtual size_t write(const uint8_t *buffer, size_t size) { return 0; }
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    
    size_t print(const char *s) {
        size_t n = 0;
        while (*s) {
            write(*s++);
            n++;
        }
        return n;
    }
    
    size_t println(const char *s = "") {
        size_t n = print(s);
        n += write('\r');
        n += write('\n');
        return n;
    }
    
    size_t println(int val) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", val);
        return println(buf);
    }
};

/**
 * @brief Serial port using SiW917 debug UART
 */
class HardwareSerial : public Stream {
public:
    void begin(unsigned long baud) {}
    void end() {}
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int available() override { return 0; }
    int read() override { return -1; }
};

extern HardwareSerial Serial;
extern Stream *BackpackOrLogStrm;

/**
 * @brief Minimal String class for ELRS compatibility
 */
class String {
public:
    String() : _buffer(nullptr), _length(0) {}
    String(const char *str) {
        if (str) {
            _length = strlen(str);
            _buffer = new char[_length + 1];
            strcpy(_buffer, str);
        } else {
            _buffer = nullptr;
            _length = 0;
        }
    }
    String(const String &other) {
        _length = other._length;
        if (other._buffer) {
            _buffer = new char[_length + 1];
            strcpy(_buffer, other._buffer);
        } else {
            _buffer = nullptr;
        }
    }
    ~String() { delete[] _buffer; }
    
    String& operator=(const String &other) {
        if (this != &other) {
            delete[] _buffer;
            _length = other._length;
            if (other._buffer) {
                _buffer = new char[_length + 1];
                strcpy(_buffer, other._buffer);
            } else {
                _buffer = nullptr;
            }
        }
        return *this;
    }
    
    const char* c_str() const { return _buffer ? _buffer : ""; }
    size_t length() const { return _length; }
    bool isEmpty() const { return _length == 0; }
    
private:
    char *_buffer;
    size_t _length;
};

#endif // __cplusplus
