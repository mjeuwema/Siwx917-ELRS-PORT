/**
 * @file logging.h
 * @brief ELRS logging macros for SiW917
 * 
 * Provides the same logging interface as ELRS for compatibility.
 */

#pragma once

#include "Arduino.h"
#include <cstdio>
#include <cstdarg>

// Debug printf implementation - use printf directly to avoid Serial class issues
inline void debugPrintf(const char* fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    // Use printf instead of Serial.print to avoid vtable/initialization issues
    printf("%s", buf);
}

// Global stream for logging
extern Stream *BackpackOrLogStrm;
#define LOGGING_UART (*BackpackOrLogStrm)

// Error logging - always enabled
#define ERRLN(msg, ...) do { \
    printf("ERROR: "); \
    printf(msg, ##__VA_ARGS__); \
    printf("\n"); \
} while(0)

// Debug logging macros
#if defined(DEBUG_LOG)
    #define DBGCR   printf("\n")
    #define DBGW(c) putchar(c)
    #define DBG(msg, ...)   debugPrintf(msg, ##__VA_ARGS__)
    #define DBGLN(msg, ...) do { \
        debugPrintf(msg, ##__VA_ARGS__); \
        printf("\n"); \
    } while(0)

    // Verbose logging
    #if defined(DEBUG_LOG_VERBOSE)
        #define DBGVCR DBGCR
        #define DBGVW(c) DBGW(c)
        #define DBGV(...) DBG(__VA_ARGS__)
        #define DBGVLN(...) DBGLN(__VA_ARGS__)
    #else
        #define DBGVCR
        #define DBGVW(c)
        #define DBGV(...)
        #define DBGVLN(...)
    #endif
#else
    // Logging disabled
    #define DBGCR
    #define DBGW(c)
    #define DBG(...)
    #define DBGLN(...)
    #define DBGVCR
    #define DBGV(...)
    #define DBGVLN(...)
#endif

// Unused variable macro
#define UNUSED(x) (void)(x)
