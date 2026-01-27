/**
 * @file WebContent.h
 * @brief ExpressLRS Web UI content wrapper for SiWx917
 * 
 * This wraps the pre-built ELRS web assets (gzip-compressed) for use
 * with the SiWx917 HTTP server.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Neutralize Arduino/ESP-specific macros for ARM */
#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef PGM_VOID_P
#define PGM_VOID_P const void *
#endif

/* Include the generated ExpressLRS web assets header */
#include "web-lr1121-rx.h"

/**
 * @brief Find a web asset by path
 * @param path The URL path (e.g., "/index.html", "/assets/app-xxx.js")
 * @return Pointer to WebAsset or NULL if not found
 */
static inline const WebAsset* find_web_asset(const char* path) {
    for (size_t i = 0; i < WEB_ASSETS_COUNT; i++) {
        if (strcmp(path, WEB_ASSETS[i].path) == 0) {
            return &WEB_ASSETS[i];
        }
    }
    return NULL;
}

#ifdef __cplusplus
}
#endif
