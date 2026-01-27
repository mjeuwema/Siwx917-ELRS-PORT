/***************************************************************************/ /**
 * @file
 * @brief WiFi AP + GSPI/LR1121 Merged Application
 *******************************************************************************
 * # License
 * <b>Copyright 2022-2026 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * Merged application combining:
 * - WiFi Access Point example (for ELRS OTA)
 * - GSPI/LR1121 driver (for radio communication)
 *
 ******************************************************************************/

#include "cmsis_os2.h"
#include "gspi_example.h"

/*******************************************************************************
 * Application Entry Point
 * 
 * Called by sl_main after FreeRTOS scheduler is ready to start.
 * Creates the application task(s) before the scheduler runs.
 ******************************************************************************/
void app_init(const void *unused)
{
  (void)unused;
  
  /* 
   * The gspi_example_init() handles test mode selection.
   * When TEST_MODE == TEST_MODE_WIFI_HTTP (7), it creates a FreeRTOS task
   * that will run the WiFi AP + HTTP server test.
   * 
   * The task runs after osKernelStart() is called by sl_main.
   */
  gspi_example_init();
}
