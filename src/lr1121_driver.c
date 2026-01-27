/**
 * @file lr1121_driver.c
 * @brief Minimal LR1121 driver implementation for SIW917
 *
 * This is a minimal standalone implementation to test SPI communication
 * with the LR1121 radio transceiver via the SIW917 GSPI peripheral.
 *
 * Key Implementation Notes:
 * - Uses Silicon Labs SDK GSPI driver (sl_si91x_gspi)
 * - Manual CS control via GPIO for LR1121's two-phase SPI protocol
 * - Direct register access for GPIO/pad configuration
 *
 * LR1121 SPI Protocol (Citation: UserManual_LR1121_v1_2.pdf):
 * The LR1121 uses a two-phase SPI protocol:
 *
 * WRITE COMMAND (sending opcode + parameters):
 *   1. Wait for BUSY LOW
 *   2. Assert NSS (LOW)
 *   3. Send 16-bit opcode (MSB first) + any parameters
 *   4. Deassert NSS (HIGH)
 *   5. LR1121 pulls BUSY HIGH while processing
 *
 * READ RESPONSE (getting data back):
 *   1. Wait for BUSY LOW (processing complete)
 *   2. Assert NSS (LOW)
 *   3. Send NOP bytes (0x00) to clock out response data
 *   4. Deassert NSS (HIGH)
 *
 * Documentation Citations:
 * - UserManual_LR1121_v1_2.pdf "SPI Communication" section
 * - siw917x-family-rm.pdf Section 20 "Generic SPI Primary (GSPI)"
 * - siw917x-family-rm.pdf Section 11 "GPIO"
 */

#include "lr1121_driver.h"
#include "rsi_debug.h"
#include "rsi_egpio.h"
#include "rsi_rom_egpio.h"
#include "sl_si91x_gspi.h"
#include "core_cm4.h"  /* For __DSB() memory barrier */

/* SL_STATUS_EMPTY may not be defined in older SDK versions
 * It's used by GSPI when DMA completes but FIFO appears empty
 */
#ifndef SL_STATUS_EMPTY
#define SL_STATUS_EMPTY ((sl_status_t)0x0022)
#endif

/* USE_SOFT_SPI: Bit-bang SPI implementation
 * 
 * Uncomment to use software bit-bang SPI (slower but proven to work).
 * Comment out to use hardware GSPI with DMA (faster, required for ELRS).
 */
// #define USE_SOFT_SPI

#include <string.h>

/* Soft SPI function removed - Logic inlined in spi_transfer */

/*******************************************************************************
 * Direct Register Definitions for GPIO and Pad Configuration
 *
 * These registers must be configured BEFORE using GSPI to ensure the GPIO
 * pins are properly muxed for the GSPI peripheral function.
 *
 * Citation: siw917x-family-rm.pdf Section 11.4.1 MEM_GPIO_ACCESS_CTRL_SET
 * Citation: siw917x-family-rm.pdf Section 24.5.15 MCR_GENERIC_CTRL_1_REG
 ******************************************************************************/

/* GPIO PAD Control - Take MCU control of GPIO_25-30 from NWP */
#define GPIO_PAD_CTRL_BASE 0x41300000UL
#define MEM_GPIO_ACCESS_CTRL_SET                                               \
  (*(volatile uint32_t *)(GPIO_PAD_CTRL_BASE + 0x000))
#define NWP_MCUHP_GPIO_CTRL2_BIT (1UL << 5)

/* MCU Configuration Register - HOST_PADS_GPIO_MODE */
#define MCR_BASE 0x46008000UL
#define MCR_GENERIC_CTRL_1_REG (*(volatile uint32_t *)(MCR_BASE + 0x044))

/*******************************************************************************
 * EGPIO Clock Enable Registers - CRITICAL!
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.3, p.98
 *   CLK_ENABLE_SET_REG2 at offset 0x008:
 *   - Bit 21 (EGPIO_PCLK_ENABLE): EGPIO APB Clock Enable
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.5, p.102
 *   CLK_ENABLE_SET_REG3 at offset 0x010:
 *   - Bit 16 (EGPIO_CLK_ENABLE): EGPIO Controller Clock Enable (reset=0!)
 *
 * IMPORTANT: Both clocks are DISABLED at reset! Must enable before GPIO works.
 ******************************************************************************/
#define M4CLK_BASE 0x46000000UL
#define CLK_ENABLE_SET_REG2  (*(volatile uint32_t *)(M4CLK_BASE + 0x008))
#define CLK_ENABLE_SET_REG3  (*(volatile uint32_t *)(M4CLK_BASE + 0x010))
#define EGPIO_PCLK_ENABLE_BIT  (1UL << 21)  /* CLK_ENABLE_SET_REG2 bit 21 */
#define EGPIO_CLK_ENABLE_BIT   (1UL << 16)  /* CLK_ENABLE_SET_REG3 bit 16 */

/* HOST_PADS_GPIO_MODE bit positions for GPIO_26-30
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 24.5.15 MCR_GENERIC_CTRL_1_REG, p.612
 *   Bits 18:14 - HOST_PADS_GPIO_MODE
 *   "Control bits for GPIO_26 to GPIO_30 to use either as host interface pins or GPIO pins."
 *   "One bit per pin. 0 = Host Interface, 1 = GPIO."
 *
 * CRITICAL: At reset, these bits are 0 (Host Interface / SDIO mode), NOT GPIO mode!
 * We MUST set these bits to 1 to use the pins as GPIO!
 */
#define HOST_PADS_GPIO26_BIT (1UL << 14)  /* GPIO_26 (MISO) */
#define HOST_PADS_GPIO27_BIT (1UL << 15)  /* GPIO_27 (MOSI) */
#define HOST_PADS_GPIO28_BIT (1UL << 16)  /* GPIO_28 (CS) */
#define HOST_PADS_GPIO29_BIT (1UL << 17)  /* GPIO_29 (BUSY) */
#define HOST_PADS_GPIO30_BIT (1UL << 18)  /* GPIO_30 (RST) */
#define HOST_PADS_GPIO_MODE_ALL \
  (HOST_PADS_GPIO26_BIT | HOST_PADS_GPIO27_BIT | HOST_PADS_GPIO28_BIT | \
   HOST_PADS_GPIO29_BIT | HOST_PADS_GPIO30_BIT)

/* HP GPIO Direct Register Access (Pins 25-30)
 *
 * CRITICAL FIX: GPIO_25-30 are on EGPIO PORT 1, NOT PORT 0!
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.11, p.320
 *   "Base address for EGPIO instance: 0x4613_0000"
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Table 11.4 "EGPIO Port Register Mapping", p.292
 *   EGPIO PORT 1 (SL_GPIO_PORT_B) bit mapping:
 *     Bit 15 = GPIO_31
 *     Bit 14 = GPIO_30  (RST)
 *     Bit 13 = GPIO_29  (BUSY)
 *     Bit 12 = GPIO_28  (CS)
 *     Bit 11 = GPIO_27  (MOSI)
 *     Bit 10 = GPIO_26  (MISO)
 *     Bit 9  = GPIO_25  (SCK)
 *     Bits 0-8 = Not present
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.1, p.321
 *   GPIO_CONFIG_REG_x: Offset = 0x000 + (0x10 * pin_number)
 *   - DIRECTION bit (bit 0): 0 = Output, 1 = Input
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.4-11.12.9, p.324-326
 *   PORT registers use offset 0x1000 + (0x40 * port_number)
 *   - PORT 1 offset: 0x1000 + 0x40 = 0x1040
 */
#define EGPIO_BASE 0x46130000UL

/* Per-Pin GPIO_CONFIG_REG for direction control
 * Offset = 0x000 + (0x10 * pin_number)
 * Bit 0 = DIRECTION (0=output, 1=input)
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.11, p.320
 */
#define EGPIO_GPIO_CONFIG_REG(pin) (*(volatile uint32_t *)(EGPIO_BASE + (0x10 * (pin))))

/* Per-Pin BIT_LOAD_REG for individual pin read/write
 * Offset = 0x004 + (0x10 * pin_number)
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.2 BIT_LOAD_REG_x, p.323
 *   "Reading BIT_LOAD reads the logic level present at the pin."
 *   "Sets pin value on write."
 */
#define EGPIO_BIT_LOAD_REG(pin) (*(volatile uint32_t *)(EGPIO_BASE + 0x004 + (0x10 * (pin))))

/* PORT 1 registers for GPIO_25-30
 * Port 1 base offset = 0x1000 + (0x40 * 1) = 0x1040
 */
#define EGPIO_PORT1_BASE (EGPIO_BASE + 0x1040)
#define EGPIO_PORT1_LOAD_REG   (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x00))  /* 0x1040 */
#define EGPIO_PORT1_SET_REG    (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x04))  /* 0x1044 */
#define EGPIO_PORT1_CLR_REG    (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x08))  /* 0x1048 */
#define EGPIO_PORT1_READ_REG   (*(volatile uint32_t *)(EGPIO_PORT1_BASE + 0x14))  /* 0x1054 */

/* Bit position in PORT 1 for each GPIO pin
 * GPIO_25 = bit 9, GPIO_26 = bit 10, ..., GPIO_30 = bit 14
 */
#define HP_GPIO_PORT1_BIT(pin) (1UL << ((pin) - 25 + 9))

/* Direction control via per-pin GPIO_CONFIG_REG */
#define HP_GPIO_SET_OUTPUT(pin) (EGPIO_GPIO_CONFIG_REG(pin) &= ~(1UL << 0))  /* DIRECTION=0 */
#define HP_GPIO_SET_INPUT(pin)  (EGPIO_GPIO_CONFIG_REG(pin) |=  (1UL << 0))  /* DIRECTION=1 */

/* Output control via PORT 1 SET/CLR registers (ORIGINAL - keeping for reference)
 * #define HP_GPIO_SET_HIGH(pin) (EGPIO_PORT1_SET_REG = HP_GPIO_PORT1_BIT(pin))
 * #define HP_GPIO_SET_LOW(pin)  (EGPIO_PORT1_CLR_REG = HP_GPIO_PORT1_BIT(pin))
 */

/* Output control via BIT_LOAD_REG (per-pin register - RECOMMENDED by datasheet)
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.2 BIT_LOAD_REG_x, p.323
 *   "Writing BIT_LOAD will set or clear the pin output."
 *   "Reading BIT_LOAD reads the logic level present at the pin."
 * 
 * This method directly writes to the individual pin's register instead of
 * using the PORT-wide SET/CLR registers, which may have additional requirements.
 */
#define HP_GPIO_SET_HIGH(pin) (EGPIO_BIT_LOAD_REG(pin) = 1)
#define HP_GPIO_SET_LOW(pin)  (EGPIO_BIT_LOAD_REG(pin) = 0)

/* Input read via BIT_LOAD_REG (individual pin read - recommended method)
 * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.2, p.323
 * "Reading BIT_LOAD reads the logic level present at the pin."
 */
#define HP_GPIO_READ(pin) (EGPIO_BIT_LOAD_REG(pin) & 1)

/* Alternative: Input read via PORT 1 READ register (for bulk reads) */
#define HP_GPIO_READ_PORT1(pin) ((EGPIO_PORT1_READ_REG >> ((pin) - 25 + 9)) & 1)

/* PAD Configuration - Enable receiver for MISO (GPIO_26) */
#define PAD_CONFIG_BASE 0x46004000UL
/* Note: PAD_CONFIG_REG is already defined in rsi_egpio.h, using it directly */
#define PADCONFIG_REN_BIT (1UL << 4) /* Receiver Enable */
#define PADCONFIG_SMT_BIT (1UL << 3) /* Schmitt Trigger */

/* GSPI Peripheral Registers for Full-Duplex Mode
 * Citation: siw917x-family-rm.pdf Section 20.4 "GSPI Primary Register Map"
 * Base address: 0x4503_0000
 */
#define GSPI_BASE 0x45030000UL
#define GSPI_CLK_CONFIG_REG (*(volatile uint32_t *)(GSPI_BASE + 0x000))
#define GSPI_BUS_MODE_REG (*(volatile uint32_t *)(GSPI_BASE + 0x004))
#define GSPI_CONFIG1_REG (*(volatile uint32_t *)(GSPI_BASE + 0x010))
#define GSPI_CONFIG2_REG (*(volatile uint32_t *)(GSPI_BASE + 0x014))
#define GSPI_WRITE_DATA2_REG (*(volatile uint32_t *)(GSPI_BASE + 0x018))
#define GSPI_FIFO_THRLD_REG (*(volatile uint32_t *)(GSPI_BASE + 0x01C))
#define GSPI_STATUS_REG (*(volatile uint32_t *)(GSPI_BASE + 0x020))
#define GSPI_WRITE_FIFO (*(volatile uint32_t *)(GSPI_BASE + 0x080))
#define GSPI_READ_FIFO (*(volatile uint32_t *)(GSPI_BASE + 0x080))

/* GSPI Register bit definitions */
#define GSPI_CONFIG1_FULL_DUPLEX_EN (1UL << 15)
#define GSPI_BUS_MODE_GPIO_MODE_EN (0x3FUL << 5)
#define GSPI_CLK_CONFIG_CLK_EN (1UL << 1)
#define GSPI_CONFIG2_RD_DATA_SWAP_ALL ((1UL << 4) | (1UL << 5) | (1UL << 6))
#define GSPI_STATUS_BUSY (1UL << 0)

/*******************************************************************************
 * Static Variables
 ******************************************************************************/

static sl_gspi_handle_t gspi_handle = NULL;
static volatile bool gspi_transfer_complete = false;
static bool driver_initialized = false;

/*******************************************************************************
 * GSPI Callback
 ******************************************************************************/

static void gspi_callback_event(uint32_t event) {
  switch (event) {
  case SL_GSPI_TRANSFER_COMPLETE:
    gspi_transfer_complete = true;
    break;
  case SL_GSPI_DATA_LOST:
    DEBUGOUT("LR1121: GSPI data lost!\n");
    gspi_transfer_complete = true; /* Set to avoid infinite wait */
    break;
  case SL_GSPI_MODE_FAULT:
    DEBUGOUT("LR1121: GSPI mode fault!\n");
    gspi_transfer_complete = true; /* Set to avoid infinite wait */
    break;
  }
}

/*******************************************************************************
 * Static Helper Functions
 ******************************************************************************/

/**
 * @brief Configure GPIO pads for GSPI peripheral use
 */
static void configure_gpio_pads(void) {
#ifdef USE_SOFT_SPI
  DEBUGOUT("=== LR1121 Driver Initial GPIO pads ===\n");
  DEBUGOUT("LR1121: Configuring GPIO pads (Soft SPI)...\n");

  /*******************************************************************************
   * CRITICAL: Enable EGPIO peripheral clocks FIRST!
   *
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.3, p.98
   *   CLK_ENABLE_SET_REG2: Bit 21 = EGPIO_PCLK_ENABLE (APB clock)
   *
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 6.13.18.5, p.102
   *   CLK_ENABLE_SET_REG3: Bit 16 = EGPIO_CLK_ENABLE (controller clock)
   *   Reset value = 0x0 (DISABLED!) - Must enable for GPIO to work!
   ******************************************************************************/
  DEBUGOUT("Enabling EGPIO clocks (CRITICAL - disabled at reset!)...\n");
  DEBUGOUT("  CLK_ENABLE_SET_REG2 BEFORE: 0x%08lX\n", (unsigned long)CLK_ENABLE_SET_REG2);
  DEBUGOUT("  CLK_ENABLE_SET_REG3 BEFORE: 0x%08lX\n", (unsigned long)CLK_ENABLE_SET_REG3);
  
  CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;  /* Enable EGPIO APB clock */
  CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;   /* Enable EGPIO controller clock */
  
  /* Wait for clocks to stabilize */
  for (volatile int i = 0; i < 1000; i++) { }
  
  DEBUGOUT("  CLK_ENABLE_SET_REG2 AFTER:  0x%08lX (expect bit21=1)\n", (unsigned long)CLK_ENABLE_SET_REG2);
  DEBUGOUT("  CLK_ENABLE_SET_REG3 AFTER:  0x%08lX (expect bit16=1)\n", (unsigned long)CLK_ENABLE_SET_REG3);

  /* Step 1: Take MCU control of GPIO_25-30 from NWP
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.4.1, p.309
   * MEM_GPIO_ACCESS_CTRL_SET at 0x4130_0000: Write bit 5 = 1 to enable MCU control
   */
  MEM_GPIO_ACCESS_CTRL_SET = NWP_MCUHP_GPIO_CTRL2_BIT;
  for (volatile int i = 0; i < 100; i++) { }
  DEBUGOUT("Step 1: MEM_GPIO_ACCESS_CTRL_SET = MCU control of GPIO_25-30\n");

  /* Step 2: CRITICAL - Enable GPIO mode for GPIO_26-30 via MCR_GENERIC_CTRL_1_REG
   *
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 24.5.15, p.612
   *   MCR_GENERIC_CTRL_1_REG at 0x46008044
   *   Bits 18:14 - HOST_PADS_GPIO_MODE
   *   "Control bits for GPIO_26 to GPIO_30 to use either as host interface pins or GPIO pins."
   *   "One bit per pin. 0 = Host Interface, 1 = GPIO."
   *
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.2.5.1, p.294
   *   "Note: To use GPIO_26 through GPIO_30 as GPIO or route to peripherals, 
   *    the corresponding bits in MCR_GENERIC_CTRL_1_REG.HOST_PADS_GPIO_MODE 
   *    field must be configured for GPIO mode."
   *
   * At reset, bits 14-18 are ALL ZERO = Host Interface (SDIO) mode
   * We MUST set them to 1 for GPIO mode - THIS IS THE ROOT CAUSE OF GPIO NOT WORKING!
   */
  DEBUGOUT("Step 2: MCR_GENERIC_CTRL_1_REG - Enable GPIO mode for GPIO_26-30\n");
  DEBUGOUT("  MCR_GENERIC_CTRL_1 BEFORE: 0x%08lX\n", 
           (unsigned long)MCR_GENERIC_CTRL_1_REG);
  
  /* Set bits 14-18 to enable GPIO mode for GPIO_26-30 */
  MCR_GENERIC_CTRL_1_REG |= HOST_PADS_GPIO_MODE_ALL;
  for (volatile int i = 0; i < 100; i++) { }
  
  DEBUGOUT("  MCR_GENERIC_CTRL_1 AFTER:  0x%08lX (expect bits 14-18 = 1)\n", 
           (unsigned long)MCR_GENERIC_CTRL_1_REG);
  DEBUGOUT("  Verify: HOST_PADS_GPIO_MODE = 0x%lX (should be 0x1F for all 5 pins)\n",
           (unsigned long)((MCR_GENERIC_CTRL_1_REG >> 14) & 0x1F));

  /* Enable receiver on GPIO_26 (MISO) - CRITICAL for reads
   * Citation: siw917x-family-rm.pdf Section 11.6.1
   * PAD_CONFIG_REG bits:
   *   Bit 4 (REN): Receiver Enable - MUST be 1 for input
   *   Bit 3 (SMT): Schmitt Trigger - improves noise immunity
   */
  DEBUGOUT("  PAD_CONFIG_REG(26) BEFORE = 0x%08lX\n", 
           (unsigned long)PAD_CONFIG_REG(26));
  PAD_CONFIG_REG(26) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  for (volatile int i = 0; i < 100; i++) {
  }
  DEBUGOUT("  PAD_CONFIG_REG(26) AFTER  = 0x%08lX\n", 
           (unsigned long)PAD_CONFIG_REG(26));

  /* Configure GPIO_25 (SCK) and GPIO_27 (MOSI) for output */
  /* Clear REN (Receiver Enable) to ensure output mode */
  PAD_CONFIG_REG(25) &= ~PADCONFIG_REN_BIT;
  PAD_CONFIG_REG(27) &= ~PADCONFIG_REN_BIT;
  
  /* Enable receiver on GPIO_29 (BUSY) - input pin
   * Per Table 11.3, GPIO_26-29 have REN=1 at reset, but enable explicitly
   */
  PAD_CONFIG_REG(29) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  for (volatile int i = 0; i < 100; i++) {
  }

  DEBUGOUT("LR1121: PAD_CONFIG summary:\n");
  DEBUGOUT("  GPIO_25 (CLK)  = 0x%08lX\n", (unsigned long)PAD_CONFIG_REG(25));
  DEBUGOUT("  GPIO_26 (MISO) = 0x%08lX (REN bit=%d)\n", 
           (unsigned long)PAD_CONFIG_REG(26),
           (PAD_CONFIG_REG(26) & PADCONFIG_REN_BIT) ? 1 : 0);
  DEBUGOUT("  GPIO_27 (MOSI) = 0x%08lX\n", (unsigned long)PAD_CONFIG_REG(27));
  DEBUGOUT("  GPIO_28 (CS)   = 0x%08lX\n", (unsigned long)PAD_CONFIG_REG(28));
  DEBUGOUT("  GPIO_29 (BUSY) = 0x%08lX\n", (unsigned long)PAD_CONFIG_REG(29));
  DEBUGOUT("  GPIO_30 (RST)  = 0x%08lX\n", (unsigned long)PAD_CONFIG_REG(30));
#else
  /* Hardware GSPI mode - SDK handles pin mux, but we MUST configure:
   * 1. MCR_GENERIC_CTRL_1_REG to enable GPIO mode for GPIO_26-30
   * 2. PAD_CONFIG_REG for MISO receiver enable
   * Without these, the pins stay in Host Interface (SDIO) mode!
   */
  DEBUGOUT("LR1121: Configuring GPIO pads for Hardware GSPI...\n");
  
  /* Enable EGPIO clocks */
  CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;
  CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;
  for (volatile int i = 0; i < 1000; i++) { }
  
  /* Take MCU control of GPIO_25-30 from NWP */
  MEM_GPIO_ACCESS_CTRL_SET = NWP_MCUHP_GPIO_CTRL2_BIT;
  for (volatile int i = 0; i < 100; i++) { }
  
  /* CRITICAL: Enable GPIO mode for GPIO_26-30 
   * At reset, bits 14-18 are 0 = Host Interface (SDIO) mode
   * Must set to 1 for GPIO/peripheral mode
   */
  DEBUGOUT("  MCR_GENERIC_CTRL_1 BEFORE: 0x%08lX\n", (unsigned long)MCR_GENERIC_CTRL_1_REG);
  MCR_GENERIC_CTRL_1_REG |= HOST_PADS_GPIO_MODE_ALL;
  for (volatile int i = 0; i < 100; i++) { }
  DEBUGOUT("  MCR_GENERIC_CTRL_1 AFTER:  0x%08lX\n", (unsigned long)MCR_GENERIC_CTRL_1_REG);
  
  /* Enable receiver on MISO (GPIO_26) - CRITICAL for reads! */
  PAD_CONFIG_REG(26) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  
  /* Enable receiver on BUSY (GPIO_29) */
  PAD_CONFIG_REG(29) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  
  DEBUGOUT("  PAD_CONFIG_REG(26/MISO) = 0x%08lX (REN=%d)\n", 
           (unsigned long)PAD_CONFIG_REG(26),
           (PAD_CONFIG_REG(26) & PADCONFIG_REN_BIT) ? 1 : 0);
#endif
  DEBUGOUT("LR1121: GPIO pads configured\n");
}

/* configure_gspi_peripheral() REMOVED - not used in working example
 * The SDK handles all GSPI peripheral configuration internally.
 * Direct register manipulation was causing issues.
 */

/**
 * @brief Simple delay in milliseconds
 */
static void delay_ms(uint32_t ms) {
  /* Simple busy-wait delay - approximately 1ms per loop at ~40MHz */
  for (uint32_t i = 0; i < ms; i++) {
    for (volatile uint32_t j = 0; j < 10000; j++) {
    }
  }
}

/**
 * @brief Simple delay in microseconds
 */
static void delay_us(uint32_t us) {
  /* Simple busy-wait delay */
  for (uint32_t i = 0; i < us; i++) {
    for (volatile uint32_t j = 0; j < 10; j++) {
    }
  }
}

/**
 * @brief Read BUSY pin state using direct PORT 1 register access
 *
 * Citation: siw917x-family-rm.pdf Rev 1.2, Table 11.4, p.292
 * GPIO_29 (BUSY) is bit 13 of PORT 1
 */
static int read_busy_pin(void) {
  return HP_GPIO_READ(LR1121_PIN_BUSY);
}

/**
 * @brief Assert CS (drive LOW) using direct PORT 1 register access
 *
 * Citation: UserManual_LR1121_v1_2.pdf - NSS setup time
 * Citation: siw917x-family-rm.pdf Rev 1.2, Table 11.4, p.292
 * GPIO_28 (CS) is bit 12 of PORT 1
 */
static void cs_assert(void) {
  HP_GPIO_SET_LOW(LR1121_PIN_NSS);
  delay_us(1); /* NSS setup time */
}

/**
 * @brief Deassert CS (drive HIGH) using direct PORT 1 register access
 *
 * Citation: UserManual_LR1121_v1_2.pdf - NSS hold time
 */
static void cs_deassert(void) {
  delay_us(1); /* NSS hold time */
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);
  delay_us(1); /* Inter-transaction gap */
}

/**
 * @brief Wait for GSPI to become idle
 */
__attribute__((unused)) static void wait_gspi_idle(void) {
  uint32_t timeout = 100000;
  while ((GSPI_STATUS_REG & GSPI_STATUS_BUSY) && timeout > 0) {
    timeout--;
  }
}

/**
 * @brief Transfer data buffer via GSPI using direct register access
 * (full-duplex)
 *
 * This function bypasses the SDK's sl_si91x_gspi_transfer_data() to avoid
 * conflicts with manual CS control. The SDK function internally tries to
 * control CS via ARM_SPI_CONTROL_SS, which fails when GPIO_28 is configured
 * as a GPIO output for manual CS control.
 *
 * Citation: siw917x-family-rm.pdf Section 20.3.1 "Programming Sequence"
 * - Write Operation: Write data to GSPI_WRITE_FIFO, set GSPI_MANUAL_WR
 * - Read Operation: Read from GSPI_READ_FIFO when not empty
 * - Full Duplex: SPI_FULL_DUPLEX_EN enables simultaneous read while writing
 *
 * @param tx_data Data to send (can be NULL for receive-only)
 * @param rx_data Buffer to receive data (can be NULL for send-only)
 * @param length Number of bytes to transfer
 * @return true on success, false on error
 */
static bool spi_transfer(const uint8_t *tx_data, uint8_t *rx_data,
                         uint16_t length) {
#ifdef USE_SOFT_SPI
  /* Soft SPI (Bit-Bang) Transfer using Direct HP GPIO Register Access */
  DEBUGOUT("SPI TX:");
  for (uint16_t i = 0; i < length; i++) {
    uint8_t tx_byte = (tx_data != NULL) ? tx_data[i] : 0x00;
    uint8_t rx_byte = 0;
    DEBUGOUT(" %02X", tx_byte);

    for (int bit = 7; bit >= 0; bit--) {
      /* 1. Setup MOSI Data */
      if ((tx_byte >> bit) & 0x01) {
        HP_GPIO_SET_HIGH(LR1121_PIN_MOSI);
      } else {
        HP_GPIO_SET_LOW(LR1121_PIN_MOSI);
      }

      /* Delay */
      for (volatile int d = 0; d < 50; d++)
        ;

      /* 2. Clock High (Rising Edge - Sample) */
      HP_GPIO_SET_HIGH(LR1121_PIN_SCK);

      /* Delay & Sample MISO */
      for (volatile int d = 0; d < 25; d++)
        ;
      if (HP_GPIO_READ(LR1121_PIN_MISO)) {
        rx_byte |= (1 << bit);
      }

      /* Delay */
      for (volatile int d = 0; d < 25; d++)
        ;

      /* 3. Clock Low (Falling Edge) */
      HP_GPIO_SET_LOW(LR1121_PIN_SCK);
    }

    if (rx_data != NULL) {
      rx_data[i] = rx_byte;
    }
  }
  DEBUGOUT("\n");

  /* Print received data */
  if (rx_data != NULL) {
    DEBUGOUT("SPI RX:");
    for (uint16_t i = 0; i < length; i++) {
      DEBUGOUT(" %02X", rx_data[i]);
    }
    DEBUGOUT("\n");
  }
  return true;
#else
  /* Hardware GSPI Transfer using SDK API functions
   * 
   * This uses the SDK's sl_si91x_gspi_transfer_data() which should handle
   * the full-duplex SPI transfer properly. We manage CS manually.
   */
  sl_status_t status;
  
  /* CRITICAL: DMA requires static buffers with proper alignment!
   * Stack buffers don't work with DMA because:
   * 1. DMA operates asynchronously after function returns
   * 2. Stack memory can be reused before DMA completes
   * 3. Cache coherency issues on ARM Cortex-M4
   *
   * Using static buffers ensures memory persistence during DMA transfer.
   * __attribute__((aligned(4))) ensures 32-bit alignment for DMA.
   * volatile prevents compiler from optimizing away reads after DMA.
   */
  static volatile uint8_t __attribute__((aligned(4))) temp_tx[256];
  static volatile uint8_t __attribute__((aligned(4))) temp_rx[256];

  if (length > sizeof(temp_tx)) {
    DEBUGOUT("LR1121: SPI transfer too long (%d > %d)\n", length, (int)sizeof(temp_tx));
    return false;
  }

  /* Clear RX buffer with marker pattern to detect if DMA updates it */
  memset((void*)temp_rx, 0xAA, length);

  /* Prepare TX buffer */
  if (tx_data != NULL) {
    memcpy((void*)temp_tx, tx_data, length);
  } else {
    memset((void*)temp_tx, 0x00, length);
  }
  
  /* Memory barrier to ensure CPU writes to TX buffer are complete before DMA starts
   * __DSB() - Data Synchronization Barrier: ensures all memory accesses complete
   * __ISB() - Instruction Synchronization Barrier: flushes pipeline
   */
  __DSB();
  __ISB();

  /* Set slave number before each transfer (matching working example) */
  sl_si91x_gspi_set_slave_number(GSPI_SLAVE_0);

  /* Reset transfer complete flag */
  gspi_transfer_complete = false;

  /* Use SDK transfer function for full-duplex SPI
   * This should send temp_tx while receiving into temp_rx simultaneously
   */
  status = sl_si91x_gspi_transfer_data(gspi_handle, (uint8_t*)temp_tx, (uint8_t*)temp_rx, length);
  
  /* ALWAYS wait for transfer complete, regardless of status.
   * The SDK may return various status codes, but we need to ensure
   * the DMA has actually finished before reading the data.
   */
  if (status != SL_STATUS_OK && status != SL_STATUS_EMPTY) {
    DEBUGOUT("LR1121: SDK sl_si91x_gspi_transfer_data failed: 0x%04lX\n", (unsigned long)status);
    return false;
  }
  
  /* Wait for transfer to complete (callback sets flag)
   * CRITICAL: Always wait, even if status is SL_STATUS_EMPTY
   */
  uint32_t timeout = 100000;
  while (!gspi_transfer_complete && timeout > 0) {
    timeout--;
  }
  
  if (timeout == 0) {
    DEBUGOUT("LR1121: SPI transfer timeout (status was 0x%04lX)\n", (unsigned long)status);
    return false;
  }
  
  /* Memory barrier to ensure DMA writes are visible to CPU
   * This is critical for ARM Cortex-M4 with DMA
   */
  __DSB();
  __ISB();
  
  /* Copy received data to output buffer if provided */
  if (rx_data != NULL) {
    memcpy(rx_data, (void*)temp_rx, length);
  }

  return true;
#endif /* USE_SOFT_SPI */
}

/**
 * @brief Send command to LR1121 (Phase 1 of SPI protocol)
 *
 * Citation: UserManual_LR1121_v1_2.pdf - Write Command
 *
 * During the write command process, the controller first sends a 16-bit
 * opcode and then sends the required parameters. When there is a falling
 * edge on the NSS pin, the LR1121 will automatically pull up the BUSY
 * signal to indicate that the command is being processed.
 *
 * @param opcode 16-bit command opcode
 * @param params Parameter bytes (can be NULL if no parameters)
 * @param param_len Number of parameter bytes
 * @return true on success, false on error
 */
static bool lr1121_send_command_internal(uint16_t opcode, const uint8_t *params,
                                        uint16_t param_len) {
  uint8_t tx_buf[16]; /* Command buffer: 2 bytes opcode + up to 14 params */
  uint8_t rx_buf[16]; /* Receive buffer (we ignore this for command phase) */
  uint16_t total_len = 2 + param_len;

  if (total_len > sizeof(tx_buf)) {
    DEBUGOUT("LR1121: Command too long!\n");
    return false;
  }

  /* Build command buffer: opcode MSB first, then parameters */
  tx_buf[0] = (opcode >> 8) & 0xFF; /* Opcode MSB */
  tx_buf[1] = opcode & 0xFF;        /* Opcode LSB */

  if (params != NULL && param_len > 0) {
    memcpy(&tx_buf[2], params, param_len);
  }

  /* DEBUG: Show full command buffer before sending */
  DEBUGOUT("[CMD] total_len=%u bytes: ", total_len);
  for (uint16_t i = 0; i < total_len; i++) {
    DEBUGOUT("%02X ", tx_buf[i]);
  }
  DEBUGOUT("\n");

  /* Assert CS, send command, deassert CS */
  cs_assert();
  bool result = spi_transfer(tx_buf, rx_buf, total_len);
  cs_deassert();

  return result;
}

/**
 * @brief Read response from LR1121 (Phase 2 of SPI protocol)
 *
 * Citation: UserManual_LR1121_v1_2.pdf - Read Command
 *
 * Once the data is ready (BUSY LOW), the controller removes (reads) the
 * returned data from SPI by continuously sending NOP (0x00 bytes).
 *
 * @param rx_data Buffer to store received data
 * @param length Number of bytes to read
 * @return true on success, false on error
 */
static bool lr1121_read_response_internal(uint8_t *rx_data, uint16_t length) {
  uint8_t tx_buf[16]; /* NOP bytes to clock out data */

  if (length > sizeof(tx_buf)) {
    DEBUGOUT("LR1121: Response too long!\n");
    return false;
  }

  /* Fill TX buffer with NOP (0x00) bytes */
  memset(tx_buf, 0x00, length);

  /* Assert CS, clock out data with NOPs, deassert CS */
  cs_assert();
  bool result = spi_transfer(tx_buf, rx_data, length);
  cs_deassert();

  return result;
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

bool lr1121_wait_busy(void) {
  uint32_t timeout_us = LR1121_BUSY_TIMEOUT_US;
  uint32_t loop_count = 0;

  while (timeout_us > 0) {
    if (read_busy_pin() == 0) {
      return true; /* BUSY is LOW = ready */
    }

    /* Small delay between checks */
    delay_us(10);
    timeout_us -= 10;
    loop_count++;

    /* Periodic progress indicator */
    if (loop_count % 10000 == 0) {
      DEBUGOUT(".");
    }
  }

  DEBUGOUT("\nLR1121: BUSY timeout!\n");
  return false;
}

lr1121_status_t lr1121_init(void) {
  sl_status_t status;

  if (driver_initialized) {
    return LR1121_OK;
  }

  DEBUGOUT("\n=== LR1121 Driver Initialization ===\n");

  /* Step 1: Configure GPIO pads for GSPI */
  DEBUGOUT("\n=== LR1121 Driver Initial GPIO pads ===\n");
  configure_gpio_pads();

#ifdef USE_SOFT_SPI
  /* Step 2-4: Configure all GPIO pins using Direct Register Access for Soft SPI
   *
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 11.12.1, p.321
   * GPIO_CONFIG_REG_x DIRECTION bit: 0 = Output, 1 = Input
   * GPIO_CONFIG_REG_x MODE bits[5:2]: 0 = GPIO mode (not peripheral)
   */
  DEBUGOUT("LR1121: Initializing Soft SPI (Bit-Bang) with direct register access...\n");

  /* Set all pins to GPIO mode (MODE=0) by clearing bits 5:2 */
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_SCK)  &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_MISO) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_MOSI) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_NSS)  &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_BUSY) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_RST)  &= ~(0xF << 2);

  /* Configure outputs: SCK (25), MOSI (27), CS (28), RST (30) */
  HP_GPIO_SET_OUTPUT(LR1121_PIN_SCK);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_MOSI);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_NSS);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_RST);
  
  /* Configure inputs: MISO (26), BUSY (29) */
  HP_GPIO_SET_INPUT(LR1121_PIN_MISO);
  HP_GPIO_SET_INPUT(LR1121_PIN_BUSY);

  /* Set Idle State (SPI Mode 0: SCK Low, MOSI Low, CS High, RST High) */
  HP_GPIO_SET_LOW(LR1121_PIN_SCK);
  HP_GPIO_SET_LOW(LR1121_PIN_MOSI);
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);  /* CS idle HIGH */
  HP_GPIO_SET_HIGH(LR1121_PIN_RST);  /* RST idle HIGH (not in reset) */
#else
  /* Step 2: Configure BUSY pin (GPIO_29) as input */
  DEBUGOUT("LR1121: Configuring BUSY pin (GPIO-%d) as input\n",
           LR1121_PIN_BUSY);
  RSI_EGPIO_SetPinMux(EGPIO, 0, LR1121_PIN_BUSY, EGPIO_PIN_MUX_MODE0);
  RSI_EGPIO_SetDir(EGPIO, 0, LR1121_PIN_BUSY, EGPIO_CONFIG_DIR_INPUT);

  /* Enable receiver on BUSY pin */
  PAD_CONFIG_REG(LR1121_PIN_BUSY) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);

  /* Step 3: Configure RST pin (GPIO_30) as output, drive HIGH (not reset) */
  DEBUGOUT("LR1121: Configuring RST pin (GPIO-%d) as output\n", LR1121_PIN_RST);
  RSI_EGPIO_SetPinMux(EGPIO, 0, LR1121_PIN_RST, EGPIO_PIN_MUX_MODE0);
  RSI_EGPIO_SetDir(EGPIO, 0, LR1121_PIN_RST, EGPIO_CONFIG_DIR_OUTPUT);
  RSI_EGPIO_SetPin(EGPIO, 0, LR1121_PIN_RST, 1); /* RST HIGH (not reset) */
#endif

#ifdef USE_SOFT_SPI

  DEBUGOUT("LR1121: GPIO_CONFIG_REG direction configured\n");
  
  /* ========== GPIO DIAGNOSTIC TEST ========== */
  DEBUGOUT("\n=== GPIO Pin Diagnostic (WITH CLOCK ENABLE FIX) ===\n");
  DEBUGOUT("Direction: 0=output, 1=input\n");
  
  /* Check direction configuration */
  uint32_t dir25 = EGPIO_GPIO_CONFIG_REG(25) & 1;
  uint32_t dir26 = EGPIO_GPIO_CONFIG_REG(26) & 1;
  uint32_t dir27 = EGPIO_GPIO_CONFIG_REG(27) & 1;
  uint32_t dir28 = EGPIO_GPIO_CONFIG_REG(28) & 1;
  uint32_t dir29 = EGPIO_GPIO_CONFIG_REG(29) & 1;
  uint32_t dir30 = EGPIO_GPIO_CONFIG_REG(30) & 1;
  
  DEBUGOUT("GPIO_CONFIG_REG direction check:\n");
  DEBUGOUT("  GPIO_25 (SCK):  dir=%lu %s\n", (unsigned long)dir25, dir25==0 ? "OK" : "FAIL");
  DEBUGOUT("  GPIO_26 (MISO): dir=%lu %s\n", (unsigned long)dir26, dir26==1 ? "OK" : "FAIL");
  DEBUGOUT("  GPIO_27 (MOSI): dir=%lu %s\n", (unsigned long)dir27, dir27==0 ? "OK" : "FAIL");
  DEBUGOUT("  GPIO_28 (CS):   dir=%lu %s\n", (unsigned long)dir28, dir28==0 ? "OK" : "FAIL");
  DEBUGOUT("  GPIO_29 (BUSY): dir=%lu %s\n", (unsigned long)dir29, dir29==1 ? "OK" : "FAIL");
  DEBUGOUT("  GPIO_30 (RST):  dir=%lu %s\n", (unsigned long)dir30, dir30==0 ? "OK" : "FAIL");

  /* Test BIT_LOAD_REG reads (individual pin read - recommended) */
  DEBUGOUT("\nBIT_LOAD_REG reads (recommended for individual pins):\n");
  DEBUGOUT("  BIT_LOAD_REG(26/MISO) = %lu\n", (unsigned long)(EGPIO_BIT_LOAD_REG(26) & 1));
  DEBUGOUT("  BIT_LOAD_REG(29/BUSY) = %lu\n", (unsigned long)(EGPIO_BIT_LOAD_REG(29) & 1));
  
  /* Test PORT1_READ_REG (bulk read) */
  DEBUGOUT("\nPORT1_READ_REG = 0x%08lX\n", (unsigned long)EGPIO_PORT1_READ_REG);
  DEBUGOUT("  bit9  (GPIO_25/SCK)  = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(25));
  DEBUGOUT("  bit10 (GPIO_26/MISO) = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(26));
  DEBUGOUT("  bit11 (GPIO_27/MOSI) = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(27));
  DEBUGOUT("  bit12 (GPIO_28/CS)   = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(28));
  DEBUGOUT("  bit13 (GPIO_29/BUSY) = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(29));
  DEBUGOUT("  bit14 (GPIO_30/RST)  = %lu\n", (unsigned long)HP_GPIO_READ_PORT1(30));

  /* Test HP_GPIO_READ macro (using BIT_LOAD_REG) */
  DEBUGOUT("\nHP_GPIO_READ macro (uses BIT_LOAD_REG):\n");
  DEBUGOUT("  HP_GPIO_READ(26/MISO) = %lu\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_MISO));
  DEBUGOUT("  HP_GPIO_READ(29/BUSY) = %lu\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_BUSY));
  
  /* Test: Read MISO 10 times in a row */
  DEBUGOUT("\nMISO 10-sample test: ");
  for (int i = 0; i < 10; i++) {
    DEBUGOUT("%lu", (unsigned long)HP_GPIO_READ(LR1121_PIN_MISO));
    for (volatile int d = 0; d < 1000; d++);
  }
  DEBUGOUT("\n");
  
  /* Test: Toggle CLK and read MISO (should stay stable if no device) */
  DEBUGOUT("MISO while toggling CLK: ");
  for (int i = 0; i < 8; i++) {
    HP_GPIO_SET_HIGH(LR1121_PIN_SCK);
    for (volatile int d = 0; d < 100; d++);
    DEBUGOUT("%lu", (unsigned long)HP_GPIO_READ(LR1121_PIN_MISO));
    HP_GPIO_SET_LOW(LR1121_PIN_SCK);
    for (volatile int d = 0; d < 100; d++);
  }
  DEBUGOUT("\n");
  
  /* NOTE: Hardware verification test MOVED to lr1121_hw_verification_test()
   * The test should be run AFTER reset, not during init, because
   * the LR1121 won't respond correctly until it's been reset.
   */
  DEBUGOUT("=== End GPIO Diagnostic ===\n\n");
  /* ========================================== */

  driver_initialized = true;
  /* Skip SDK GSPI Init */
  goto skip_gspi_init;
#endif

  /* =========================================================================
   * SIMPLIFIED GSPI INIT - Matching working sl_si91x_gspi example
   * 
   * The working example uses only these SDK calls:
   *   1. sl_si91x_gspi_init()
   *   2. sl_si91x_gspi_set_configuration()
   *   3. sl_si91x_gspi_register_event_callback()
   *
   * REMOVED (not in working example):
   *   - sl_si91x_gspi_configure_clock() 
   *   - sl_si91x_gspi_set_slave_number() during init (moved to before transfer)
   *   - sl_si91x_gspi_set_master_state()
   *   - configure_gspi_peripheral() direct register writes
   * =========================================================================
   */

  /* Step 1: Initialize GSPI peripheral via SDK */
  DEBUGOUT("LR1121: Initializing GSPI via SDK...\n");
  status = sl_si91x_gspi_init(SL_GSPI_MASTER, &gspi_handle);

  if (status == SL_STATUS_BUSY) {
    DEBUGOUT("LR1121: GSPI busy - deinitializing first...\n");
    extern sl_gspi_driver_t Driver_GSPI_MASTER;
    sl_si91x_gspi_deinit((sl_gspi_handle_t)&Driver_GSPI_MASTER);
    delay_ms(10);
    status = sl_si91x_gspi_init(SL_GSPI_MASTER, &gspi_handle);
  }

  if (status != SL_STATUS_OK) {
    DEBUGOUT("LR1121: GSPI init failed: 0x%04lX\n", status);
    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("LR1121: GSPI init OK\n");

  /* Step 2: Configure GSPI parameters
   * NOTE: swap_read = 1 matches Silicon Labs gspi_example.c (GSPI_SWAP_READ_DATA = 1)
   */
  sl_gspi_control_config_t gspi_config = {
      .bit_width = 8,
      .clock_mode = SL_GSPI_MODE_0,
      .slave_select_mode = SL_GSPI_MASTER_HW_OUTPUT,
      .bitrate = 2000000,           /* 2 MHz - SLOW for debugging SPI issues */
      .swap_read = 0,               /* FIXED: Disable read swap - was corrupting packet data */
      .swap_write = 0,
  };

  status = sl_si91x_gspi_set_configuration(gspi_handle, &gspi_config);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("LR1121: GSPI config failed: 0x%04lX\n", status);
    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("LR1121: GSPI configured: 2 MHz (SLOW DEBUG), Mode 0, 8-bit\n");

  /* Step 3: Register callback */
  status = sl_si91x_gspi_register_event_callback(gspi_handle, gspi_callback_event);
  if (status != SL_STATUS_OK && status != SL_STATUS_BUSY) {
    DEBUGOUT("LR1121: GSPI callback registration failed: 0x%04lX\n", status);
    return LR1121_ERROR_SPI_INIT;
  }
  DEBUGOUT("LR1121: GSPI callback registered\n");

  driver_initialized = true;

#ifdef USE_SOFT_SPI
skip_gspi_init:
#endif

  /* Step 9: Configure CS pin for manual control
   * Required for LR1121's two-phase SPI protocol where CS must stay
   * asserted across multiple bytes within each phase.
   */
  RSI_EGPIO_SetPinMux(EGPIO, 0, LR1121_PIN_NSS, EGPIO_PIN_MUX_MODE0);
  RSI_EGPIO_SetDir(EGPIO, 0, LR1121_PIN_NSS, EGPIO_CONFIG_DIR_OUTPUT);
  RSI_EGPIO_SetPin(EGPIO, 0, LR1121_PIN_NSS, 1);

  DEBUGOUT("LR1121: Driver initialization complete\n");
  driver_initialized = true;

  return LR1121_OK;
}

void lr1121_deinit(void) {
  if (!driver_initialized) {
    return;
  }

  DEBUGOUT("LR1121: Deinitializing...\n");

  if (gspi_handle != NULL) {
    sl_si91x_gspi_deinit(gspi_handle);
    gspi_handle = NULL;
  }

  driver_initialized = false;
  DEBUGOUT("LR1121: Deinitialized\n");
}

lr1121_status_t lr1121_reset(void) {
  DEBUGOUT("\nPerforming hardware reset...\n");

  /* Drive RST LOW */
  RSI_EGPIO_SetPin(EGPIO, 0, LR1121_PIN_RST, 0);
  delay_ms(LR1121_RESET_PULSE_MS);

  /* Drive RST HIGH */
  RSI_EGPIO_SetPin(EGPIO, 0, LR1121_PIN_RST, 1);
  DEBUGOUT("LR1121: Hardware reset performed\n");

  /* Wait for recovery (BUSY is HIGH for ~230ms after reset) */
  DEBUGOUT("LR1121: Waiting for reset recovery (%d ms)...\n",
           LR1121_RESET_RECOVERY_MS);
  delay_ms(LR1121_RESET_RECOVERY_MS);

  /* Wait for BUSY to go LOW */
  DEBUGOUT("LR1121: Waiting for BUSY LOW...");
  if (!lr1121_wait_busy()) {
    DEBUGOUT(" TIMEOUT!\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT(" OK\n");

  return LR1121_OK;
}

lr1121_status_t lr1121_get_version(lr1121_version_t *version) {
  /*
   * LR1121 SPI Read Response Format:
   * Citation: LR11xx User Manual Section "SPI Interface"
   *
   * During read phase, the FIRST NOP byte clocked returns a dummy/status byte
   * (stat1), then subsequent bytes contain the actual response data.
   *
   * For GetVersion (0x0101), the response is:
   *   Byte 0: stat1 (dummy status byte - discard)
   *   Byte 1: Status from command
   *   Byte 2: Hardware version
   *   Byte 3: Firmware type (use code)
   *   Byte 4: Firmware version MSB
   *   Byte 5: Firmware version LSB
   *
   * Total: 6 bytes (1 dummy + 5 data)
   */
  uint8_t response[6]; /* stat1(dummy) + Status + HW + Type + VerMSB + VerLSB */

  if (version == NULL) {
    return LR1121_ERROR_SPI_INIT;
  }

  DEBUGOUT("\n=== Getting version ===\n");

  /*
   * LR1121 GetVersion Protocol:
   * Citation: UserManual_LR1121_v1_2.pdf - Read Command
   *
   * Phase 1: Send Command
   * - Wait for BUSY LOW
   * - Assert NSS, send opcode 0x0101, deassert NSS
   * - LR1121 pulls BUSY HIGH while processing
   *
   * Phase 2: Read Response
   * - Wait for BUSY LOW (processing complete)
   * - Assert NSS, send NOP bytes, receive response, deassert NSS
   * - First byte is dummy stat1, actual data starts at byte 1
   */

  /* Phase 1: Wait for BUSY LOW before sending command */
  DEBUGOUT("Phase 1: Waiting for BUSY LOW...");
  if (!lr1121_wait_busy()) {
    DEBUGOUT(" TIMEOUT!\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT(" OK\n");

  /* Phase 1: Send GetVersion command (opcode 0x0101, no parameters) */
  DEBUGOUT("Phase 1: Sending command 0x%04X\n", LR1121_CMD_GET_VERSION);
  if (!lr1121_send_command_internal(LR1121_CMD_GET_VERSION, NULL, 0)) {
    DEBUGOUT("Phase 1: Command send failed!\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Phase 2: Wait for BUSY LOW (command processing complete) */
  DEBUGOUT("Phase 2: Waiting for BUSY LOW (processing)...");
  if (!lr1121_wait_busy()) {
    DEBUGOUT(" TIMEOUT!\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT(" OK\n");

  /* Phase 2: Read response (6 bytes: stat1(dummy) + Status + HW + Type + Version[2])
   * Citation: LR11xx protocol requires reading 1 extra dummy byte at start
   */
  DEBUGOUT("Phase 2: Reading response (6 bytes: 1 dummy + 5 data)...\n");
  if (!lr1121_read_response_internal(response, sizeof(response))) {
    DEBUGOUT("Phase 2: Response read failed!\n");
    return LR1121_ERROR_SPI_INIT;
  }

  /* Parse response according to Semtech LR11xx User Manual Section 2.3.1
   * 
   * GetVersion (0x0101) Response Format:
   *   Byte 0: stat1 (status byte returned during read phase)
   *   Byte 1: HW version (0x22 = LR1121)
   *   Byte 2: Use/Type (0x01 = LR1121 modem, 0x03 = Production)
   *   Byte 3: FW Major version
   *   Byte 4: FW Minor version
   *   Byte 5: (extra/padding)
   *
   * Note: There is NO separate "stat2" - stat1 already contains the status!
   */
  uint8_t stat1 = response[0];       /* Status byte (stat1) - check for errors */
  uint8_t hw_version = response[1];  /* Hardware version (0x22 = LR1121) */
  uint8_t fw_use = response[2];      /* Firmware use/type */
  uint8_t fw_major = response[3];    /* Firmware major version */
  uint8_t fw_minor = response[4];    /* Firmware minor version */

  DEBUGOUT("\nResponse (raw): %02X %02X %02X %02X %02X %02X\n",
           response[0], response[1], response[2], response[3], response[4], response[5]);
  DEBUGOUT("  stat1=0x%02X (status during read)\n", stat1);
  DEBUGOUT("  HW=0x%02X   Use=0x%02X   FW=%d.%d\n", 
           hw_version, fw_use, fw_major, fw_minor);

  /* Fill version structure */
  version->hardware = hw_version;
  version->type = fw_use;
  version->version = ((uint16_t)fw_major << 8) | fw_minor;

  return LR1121_OK;
}

void lr1121_test_communication(void) {
  lr1121_status_t status;
  lr1121_version_t version;

  DEBUGOUT("\n");
  DEBUGOUT("========================================\n");
  DEBUGOUT("  GSPI Example: LR1121 Test Mode\n");
  DEBUGOUT("  (WITH SPI RE-INIT + TCXO FIX)\n");
  DEBUGOUT("========================================\n");

  /* Initialize driver */
  status = lr1121_init();
  if (status != LR1121_OK) {
    DEBUGOUT("FAILED: Initialization error %d\n", status);
    return;
  }

  /*************************************************************************
   * FIX 1: Extended Hardware Reset (100ms pulse)
   *
   * Citation: LR1121 Datasheet Section 4.2.1 "Reset Timing"
   * - Minimum reset pulse is 100µs, but longer reset (100ms) recommended
   *   for recovering from stuck states
   * - Reset pulse increased from 1ms to 100ms in lr1121_driver.h
   *************************************************************************/
  DEBUGOUT("\n--- FIX 1: Extended Hardware Reset ---\n");
  DEBUGOUT("Reset pulse: %d ms (increased from 1ms for stuck recovery)\n", 
           LR1121_RESET_PULSE_MS);
  
  status = lr1121_reset();
  if (status != LR1121_OK) {
    DEBUGOUT("FAILED: Reset error %d\n", status);
    /* Continue anyway to try other fixes */
  }

  /*************************************************************************
   * NEW: Post-Reset Hardware Verification
   *
   * Run hardware verification AFTER reset when LR1121 should be responding.
   * This helps diagnose whether the issue is GPIO configuration or
   * physical connectivity.
   *************************************************************************/
  DEBUGOUT("\n--- Hardware Verification (POST-RESET) ---\n");
  lr1121_hw_verification_test();

  /*************************************************************************
   * FIX 2: SPI Peripheral Re-initialization
   *
   * Citation: siw917x-family-rm.pdf Section 20 "Generic SPI Primary (GSPI)"
   * - Reset GSPI FIFOs and clear any hung state before communication
   * - This clears stuck transfers from prior failed attempts
   *************************************************************************/
  DEBUGOUT("\n--- FIX 2: GSPI Re-initialization ---\n");
  lr1121_reinit_spi();

  /*************************************************************************
   * FIX 3: SetTcxoMode Wake-Up Command
   *
   * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
   * - TCXO modules (Core1121-HF) require explicit TCXO configuration
   * - Without this, oscillator may not start and PLL won't lock
   * - BUSY stays HIGH if PLL not locked
   *************************************************************************/
  DEBUGOUT("\n--- FIX 3: SetTcxoMode Wake-Up ---\n");
  status = lr1121_set_tcxo_mode();
  if (status != LR1121_OK) {
    DEBUGOUT("WARNING: SetTcxoMode failed (error %d)\n", status);
    DEBUGOUT("Continuing with GetVersion attempt...\n");
  }

  /* Get version */
  DEBUGOUT("\n--- Attempting GetVersion ---\n");
  status = lr1121_get_version(&version);
  if (status != LR1121_OK) {
    DEBUGOUT("FAILED: GetVersion error %d\n", status);
    DEBUGOUT("\n");
    DEBUGOUT("========================================\n");
    DEBUGOUT("  TROUBLESHOOTING CHECKLIST\n");
    DEBUGOUT("========================================\n");
    DEBUGOUT("1. Power Supply:\n");
    DEBUGOUT("   - Is VDDIO (1.8-3.6V) connected?\n");
    DEBUGOUT("   - Is VDDRF connected?\n");
    DEBUGOUT("   - Check with multimeter for stable voltage\n");
    DEBUGOUT("\n");
    DEBUGOUT("2. Pin Connections (mikroBUS socket):\n");
    DEBUGOUT("   - GPIO_25 (SCK)  -> LR1121 SCK\n");
    DEBUGOUT("   - GPIO_26 (MISO) -> LR1121 MISO\n");
    DEBUGOUT("   - GPIO_27 (MOSI) -> LR1121 MOSI\n");
    DEBUGOUT("   - GPIO_28 (CS)   -> LR1121 NSS\n");
    DEBUGOUT("   - GPIO_29 (BUSY) -> LR1121 BUSY\n");
    DEBUGOUT("   - GPIO_30 (RST)  -> LR1121 NRESET\n");
    DEBUGOUT("\n");
    DEBUGOUT("3. BUSY Pin Behavior:\n");
    DEBUGOUT("   - Run BUSY monitor test (TEST_MODE_BUSY_MONITOR)\n");
    DEBUGOUT("   - BUSY should go HIGH for ~230ms after reset\n");
    DEBUGOUT("   - If BUSY never goes HIGH: power or reset issue\n");
    DEBUGOUT("   - If BUSY stays HIGH: oscillator not starting\n");
    DEBUGOUT("\n");
    DEBUGOUT("4. Crystal/TCXO:\n");
    DEBUGOUT("   - For TCXO modules: SetTcxoMode is required\n");
    DEBUGOUT("   - Check crystal connections and load capacitors\n");
    DEBUGOUT("========================================\n");
    return;
  }

  /* Print results 
   * Citation: LR1121 User Manual Section 2.3.1 "GetVersion"
   *   HW values: 0x01=LR1110, 0x02=LR1120, 0x22=LR1121
   *   Use values: 0x01=LR1121 transceiver, 0x03=Production firmware
   */
  DEBUGOUT("\n");
  DEBUGOUT("=====================================\n");
  DEBUGOUT("      LR1121 Version Information\n");
  DEBUGOUT("=====================================\n");
  DEBUGOUT("Hardware Version:     0x%02X", version.hardware);
  if (version.hardware == 0x22) {
    DEBUGOUT(" (LR1121) ✓\n");
  } else if (version.hardware == 0x02) {
    DEBUGOUT(" (LR1120)\n");
  } else if (version.hardware == 0x01) {
    DEBUGOUT(" (LR1110)\n");
  } else {
    DEBUGOUT(" (Unknown)\n");
  }

  DEBUGOUT("Firmware Use/Type:    0x%02X", version.type);
  if (version.type == 0x01) {
    DEBUGOUT(" (LR1121 Transceiver)\n");
  } else if (version.type == 0x03) {
    DEBUGOUT(" (Production Firmware) ✓\n");
  } else {
    DEBUGOUT(" (Unknown type)\n");
  }

  DEBUGOUT("Firmware Version:     %d.%d (0x%04X)\n", 
           (version.version >> 8) & 0xFF, 
           version.version & 0xFF, 
           version.version);

  /* Success check based on correct hardware ID */
  if (version.hardware == 0x22) {
    DEBUGOUT("\n*** SUCCESS: LR1121 communication verified! ***\n");
  } else if (version.hardware == 0x01 || version.hardware == 0x02) {
    DEBUGOUT("\nNOTE: Detected LR11%d0, not LR1121\n", version.hardware == 0x01 ? 1 : 2);
  } else {
    DEBUGOUT("\nWARNING: Unexpected hardware version 0x%02X\n", version.hardware);
    DEBUGOUT("This may indicate SPI communication issues.\n");
  }
  DEBUGOUT("\n");
}

/*******************************************************************************
 * NEW: SetTcxoMode Implementation
 *
 * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
 *
 * Sends the SetTcxoMode command to initialize the TCXO oscillator.
 * This is REQUIRED for TCXO-based modules (Core1121-HF) to start the
 * oscillator and lock the PLL. Without this, the chip may never respond.
 *
 * Command format:
 *   Opcode: 0x0117 (2 bytes) - FIX: Was incorrectly documented as 0x0097
 *   Param1: Voltage Trim (1 byte) - configured via LR1121_TCXO_VOLTAGE_TRIM
 *   Param2: Delay (3 bytes, MSB first) - 50ms startup delay
 *
 * CRITICAL: The voltage trim must match the actual TCXO voltage:
 * - For internally-powered TCXO: Use the LR1121's VTCXO regulator setting
 * - For externally-powered TCXO: Use the voltage actually applied to TCXO
 *
 * Core1121-HF has 3.3V externally-powered TCXO, so use 0x07 (3.3V).
 ******************************************************************************/
lr1121_status_t lr1121_set_tcxo_mode(void) {
#if LR1121_TCXO_EXTERNAL_POWER
  /***************************************************************************
   * EXTERNALLY POWERED TCXO - SKIP SetTcxoMode!
   *
   * Citation: ExpressLRS GitHub Discussion #3045
   * "Most manufactured devices directly connect the TCXO to a separate
   * power source."
   *
   * The Core1121-HF module has its 32MHz TCXO powered directly from the
   * module's 3.3V supply rail, NOT from the LR1121's internal VTCXO 
   * regulator pin. This means:
   *
   * 1. The TCXO is already running as soon as VDD is applied
   * 2. SetTcxoMode is NOT needed and may cause PERR (parameter error)
   * 3. SetStandby(XOSC) can be called directly after reset
   *
   * Return success immediately without sending the command.
   ***************************************************************************/
  DEBUGOUT("\n=== SetTcxoMode SKIPPED ===\n");
  DEBUGOUT("Citation: Core1121-HF has externally-powered TCXO\n");
  DEBUGOUT("  The 32MHz TCXO is powered from module's 3.3V rail\n");
  DEBUGOUT("  NOT from LR1121's internal VTCXO regulator\n");
  DEBUGOUT("  SetTcxoMode is NOT needed - TCXO already running!\n");
  DEBUGOUT("  (Set LR1121_TCXO_EXTERNAL_POWER=0 to enable SetTcxoMode)\n");
  
  return LR1121_OK;
#else
  /***************************************************************************
   * INTERNALLY POWERED TCXO - Send SetTcxoMode command
   *
   * Citation: LR1121 Datasheet Section 11.2.5 "SetTcxoMode"
   *
   * For modules where TCXO is powered from LR1121's internal VTCXO 
   * regulator, this command must be sent to enable the regulator and
   * configure the voltage.
   ***************************************************************************/
  uint8_t params[4];
  
  /* Voltage trim to actual voltage mapping (LR1121 User Manual Section 6.3.2) */
  static const char* voltage_names[] = {
    "1.6V", "1.7V", "1.8V", "2.2V", "2.4V", "2.7V", "3.0V", "3.3V"
  };
  const char* voltage_str = (LR1121_TCXO_VOLTAGE_TRIM <= 7) ? 
                            voltage_names[LR1121_TCXO_VOLTAGE_TRIM] : "INVALID";
  
  DEBUGOUT("\n=== Sending SetTcxoMode command ===\n");
  DEBUGOUT("Citation: LR1121 User Manual Section 6.3.2\n");
  DEBUGOUT("  Voltage Trim: 0x%02X (%s)\n", LR1121_TCXO_VOLTAGE_TRIM, voltage_str);
  DEBUGOUT("  Delay: 0x%06X (~50ms startup)\n", LR1121_TCXO_DELAY);
  
  /* Wait for BUSY LOW before sending command */
  DEBUGOUT("Waiting for BUSY LOW...");
  if (!lr1121_wait_busy()) {
    DEBUGOUT(" TIMEOUT!\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT(" OK\n");
  
  /* Build parameter buffer:
   * [0] = Voltage trim (1 byte)
   * [1-3] = Delay in 30.52µs steps (3 bytes, MSB first)
   * 
   * Citation: LR1121 User Manual Section 6.3.2 "SetTcxoMode"
   * Total command: [Opcode 2 bytes][Voltage 1 byte][Delay 3 bytes] = 6 bytes
   */
  uint32_t delay_value = LR1121_TCXO_DELAY;  /* Force evaluation */
  params[0] = LR1121_TCXO_VOLTAGE_TRIM;
  params[1] = (uint8_t)((delay_value >> 16) & 0xFF);  /* Delay MSB */
  params[2] = (uint8_t)((delay_value >> 8) & 0xFF);   /* Delay middle */
  params[3] = (uint8_t)(delay_value & 0xFF);          /* Delay LSB */
  
  /* DEBUG: Verify params array contents */
  DEBUGOUT("  params[0] (voltage) = 0x%02X\n", params[0]);
  DEBUGOUT("  params[1] (delay MSB) = 0x%02X\n", params[1]);
  DEBUGOUT("  params[2] (delay MID) = 0x%02X\n", params[2]);
  DEBUGOUT("  params[3] (delay LSB) = 0x%02X\n", params[3]);
  
  /* Send SetTcxoMode command */
  DEBUGOUT("Sending opcode 0x%04X...\n", LR1121_CMD_SET_TCXO_MODE);
  if (!lr1121_send_command_internal(LR1121_CMD_SET_TCXO_MODE, params, sizeof(params))) {
    DEBUGOUT("FAILED: SetTcxoMode command send failed!\n");
    return LR1121_ERROR_SPI_INIT;
  }
  
  /* Wait for command to complete (PLL to lock) */
  DEBUGOUT("Waiting for PLL lock (BUSY LOW)...");
  if (!lr1121_wait_busy()) {
    DEBUGOUT(" TIMEOUT!\n");
    return LR1121_ERROR_BUSY_TIMEOUT;
  }
  DEBUGOUT(" OK - TCXO configured, PLL locked\n");
  
  return LR1121_OK;
#endif
}

/*******************************************************************************
 * NEW: Hardware Verification Test
 *
 * This test should be run AFTER lr1121_reset() to verify GPIO connectivity
 * and LR1121 response. The LR1121 won't drive MISO correctly until it's been
 * properly reset.
 *
 * Citation: BRD2708A User Guide - mikroBUS Socket Pinout
 *   - GPIO_25 = SCK, GPIO_26 = MISO, GPIO_27 = MOSI
 *   - GPIO_28 = CS, GPIO_29 = BUSY (AN), GPIO_30 = RST
 ******************************************************************************/
void lr1121_hw_verification_test(void) {
#ifdef USE_SOFT_SPI
  DEBUGOUT("\n=== POST-RESET HARDWARE VERIFICATION TEST ===\n");
  DEBUGOUT("This test runs AFTER reset to verify LR1121 response.\n\n");
  
  /* Test BUSY pin - should be LOW after successful reset */
  DEBUGOUT("1. BUSY Pin Test (should be LOW after reset):\n");
  DEBUGOUT("   BUSY reads (10 samples, 100ms apart): ");
  int busy_high_count = 0;
  for (int i = 0; i < 10; i++) {
    int busy_val = HP_GPIO_READ(LR1121_PIN_BUSY);
    DEBUGOUT("%d", busy_val);
    if (busy_val) busy_high_count++;
    delay_ms(100);
  }
  DEBUGOUT("\n");
  DEBUGOUT("   BUSY HIGH count: %d/10 ", busy_high_count);
  if (busy_high_count == 0) {
    DEBUGOUT("(GOOD - LR1121 is ready)\n");
  } else if (busy_high_count == 10) {
    DEBUGOUT("(BAD - LR1121 stuck BUSY, may need power cycle)\n");
  } else {
    DEBUGOUT("(MIXED - LR1121 may be processing)\n");
  }
  
  /* Test MISO with CS asserted after reset */
  DEBUGOUT("\n2. MISO Test with CS Asserted:\n");
  HP_GPIO_SET_LOW(LR1121_PIN_NSS);  /* Assert CS */
  delay_ms(1);
  
  DEBUGOUT("   MISO reads with CS=LOW (10 samples, 50ms apart): ");
  int miso_high_count = 0;
  for (int i = 0; i < 10; i++) {
    int miso_val = HP_GPIO_READ(LR1121_PIN_MISO);
    DEBUGOUT("%d", miso_val);
    if (miso_val) miso_high_count++;
    delay_ms(50);
  }
  DEBUGOUT("\n");
  
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);  /* Deassert CS */
  
  DEBUGOUT("   MISO HIGH count: %d/10 ", miso_high_count);
  if (miso_high_count > 0 && miso_high_count < 10) {
    DEBUGOUT("(LR1121 is responding - GOOD)\n");
  } else if (miso_high_count == 0) {
    DEBUGOUT("(All LOW - LR1121 may not be connected or powered)\n");
  } else {
    DEBUGOUT("(All HIGH - MISO may have pull-up)\n");
  }
  
  /* Quick SPI test - send GetStatus command and check for non-zero response */
  DEBUGOUT("\n3. Quick SPI Response Test:\n");
  DEBUGOUT("   Sending GetStatus (0x0100) and reading response...\n");
  
  /* Wait for BUSY LOW */
  if (!lr1121_wait_busy()) {
    DEBUGOUT("   BUSY timeout - cannot test SPI\n");
    goto test_end;
  }
  
  /* Send GetStatus command (0x0100) */
  uint8_t cmd[2] = {0x01, 0x00};
  uint8_t rx[2];
  HP_GPIO_SET_LOW(LR1121_PIN_NSS);
  spi_transfer(cmd, rx, 2);
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);
  
  delay_ms(1);  /* Wait for processing */
  
  /* Wait for BUSY LOW */
  if (!lr1121_wait_busy()) {
    DEBUGOUT("   BUSY timeout after command\n");
    goto test_end;
  }
  
  /* Read response */
  uint8_t nop[3] = {0x00, 0x00, 0x00};
  uint8_t resp[3];
  HP_GPIO_SET_LOW(LR1121_PIN_NSS);
  spi_transfer(nop, resp, 3);
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);
  
  DEBUGOUT("   Response: %02X %02X %02X\n", resp[0], resp[1], resp[2]);
  
  if (resp[0] == 0x00 && resp[1] == 0x00 && resp[2] == 0x00) {
    DEBUGOUT("   Result: All zeros - LR1121 NOT RESPONDING\n");
    DEBUGOUT("   Check: Physical connections, power supply, module presence\n");
  } else {
    DEBUGOUT("   Result: Non-zero response - LR1121 IS RESPONDING!\n");
    DEBUGOUT("   Status byte: 0x%02X\n", resp[1]);
  }

test_end:
  DEBUGOUT("\n=== End Post-Reset Hardware Verification ===\n\n");
#endif
}

/*******************************************************************************
 * NEW: GSPI Re-initialization
 *
 * Citation: siw917x-family-rm.pdf Section 20.5.6 GSPI_FIFO_THRLD
 *   - WFIFO_RESET (bit 8): Write FIFO Reset
 *   - RFIFO_RESET (bit 9): Read FIFO Reset
 *
 * This function resets the GSPI FIFOs and peripheral to clear any hung state
 * from prior failed SPI operations.
 ******************************************************************************/
void lr1121_reinit_spi(void) {
  DEBUGOUT("\n=== Re-initializing GSPI peripheral ===\n");
  DEBUGOUT("Citation: siw917x-family-rm.pdf Section 20\n");
  
  /* Reset both TX and RX FIFOs
   * Citation: siw917x-family-rm.pdf Section 20.5.6 GSPI_FIFO_THRLD
   */
  DEBUGOUT("Resetting GSPI FIFOs...\n");
  uint32_t fifo_thrld = GSPI_FIFO_THRLD_REG;
  GSPI_FIFO_THRLD_REG = fifo_thrld | (1UL << 9) | (1UL << 8);
  for (volatile int i = 0; i < 1000; i++) { }
  GSPI_FIFO_THRLD_REG = fifo_thrld;
  
  /* Clear any pending manual operations
   * Citation: siw917x-family-rm.pdf Section 20.5.3 GSPI_CONFIG1
   */
  DEBUGOUT("Clearing GSPI manual mode bits...\n");
  GSPI_CONFIG1_REG &= ~((1UL << 1) | (1UL << 2));  /* Clear GSPI_MANUAL_WR/RD */
  for (volatile int i = 0; i < 100; i++) { }
  
  /* Wait for GSPI not busy */
  uint32_t timeout = 10000;
  while ((GSPI_STATUS_REG & GSPI_STATUS_BUSY) && timeout > 0) {
    timeout--;
  }
  
  if (timeout == 0) {
    DEBUGOUT("WARNING: GSPI still busy after reset!\n");
  } else {
    DEBUGOUT("GSPI re-initialized successfully\n");
  }
  
  /* Ensure proper idle state for Soft SPI */
#ifdef USE_SOFT_SPI
  DEBUGOUT("Setting SPI idle state (Mode 0)...\n");
  HP_GPIO_SET_LOW(LR1121_PIN_SCK);   /* SCK idle LOW for Mode 0 */
  HP_GPIO_SET_LOW(LR1121_PIN_MOSI);  /* MOSI idle LOW */
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);  /* CS idle HIGH (deasserted) */
#endif
  
  DEBUGOUT("GSPI ready for communication\n\n");
}

/*******************************************************************************
 * GPIO Toggle Test for Multimeter Verification
 *
 * This test slowly toggles each SPI output pin so you can verify with a
 * multimeter that the SiWx917 is actually outputting voltage.
 *
 * Citation: ug590-brd2708a-user-guide.pdf Table 3.3 "mikroBUS Socket Pinout"
 *   - GPIO_25: SCK  (SPI Clock)      - mikroBUS pin 4 (SCK)
 *   - GPIO_26: MISO (SPI Data In)    - mikroBUS pin 5 (MISO) - INPUT
 *   - GPIO_27: MOSI (SPI Data Out)   - mikroBUS pin 6 (MOSI)
 *   - GPIO_28: CS   (Chip Select)    - mikroBUS pin 3 (CS)
 *   - GPIO_29: BUSY (Status Input)   - mikroBUS pin 15 (INT) - INPUT
 *   - GPIO_30: RST  (Reset Output)   - mikroBUS pin 16 (RST)
 ******************************************************************************/
void lr1121_gpio_toggle_test(uint32_t cycles) {
#ifdef USE_SOFT_SPI
  DEBUGOUT("\n");
  DEBUGOUT("=============================================================\n");
  DEBUGOUT("  GPIO TOGGLE TEST - Verify with Multimeter\n");
  DEBUGOUT("=============================================================\n");
  DEBUGOUT("\n");
  DEBUGOUT("This test toggles each SPI OUTPUT pin slowly so you can\n");
  DEBUGOUT("measure voltage with a multimeter.\n");
  DEBUGOUT("\n");
  DEBUGOUT("Expected readings when pin is HIGH: ~3.3V\n");
  DEBUGOUT("Expected readings when pin is LOW:  ~0V\n");
  DEBUGOUT("\n");
  DEBUGOUT("mikroBUS Socket Pin Mapping:\n");
  DEBUGOUT("  GPIO_25 (SCK)  = mikroBUS pin 4  (SCK)\n");
  DEBUGOUT("  GPIO_27 (MOSI) = mikroBUS pin 6  (MOSI)\n");
  DEBUGOUT("  GPIO_28 (CS)   = mikroBUS pin 3  (CS)\n");
  DEBUGOUT("  GPIO_30 (RST)  = mikroBUS pin 16 (RST)\n");
  DEBUGOUT("\n");
  DEBUGOUT("INPUT pins (read only, not toggled):\n");
  DEBUGOUT("  GPIO_26 (MISO) = mikroBUS pin 5  (MISO)\n");
  DEBUGOUT("  GPIO_29 (BUSY) = mikroBUS pin 15 (INT)\n");
  DEBUGOUT("\n");
  
  if (cycles == 0) {
    DEBUGOUT("Running in INFINITE loop - reset MCU to exit\n");
  } else {
    DEBUGOUT("Running %lu cycles per pin\n", (unsigned long)cycles);
  }
  DEBUGOUT("\n");
  
  /* Ensure GPIO configuration is correct */
  DEBUGOUT("Configuring GPIO pins...\n");
  
  /* Enable EGPIO clocks */
  CLK_ENABLE_SET_REG2 = EGPIO_PCLK_ENABLE_BIT;
  CLK_ENABLE_SET_REG3 = EGPIO_CLK_ENABLE_BIT;
  for (volatile int i = 0; i < 1000; i++) { }
  
  /* Take MCU control of GPIO_25-30 */
  MEM_GPIO_ACCESS_CTRL_SET = NWP_MCUHP_GPIO_CTRL2_BIT;
  for (volatile int i = 0; i < 100; i++) { }
  
  /* Set GPIO mode for all pins */
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_SCK)  &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_MISO) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_MOSI) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_NSS)  &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_BUSY) &= ~(0xF << 2);
  EGPIO_GPIO_CONFIG_REG(LR1121_PIN_RST)  &= ~(0xF << 2);
  
  /* Configure outputs */
  HP_GPIO_SET_OUTPUT(LR1121_PIN_SCK);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_MOSI);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_NSS);
  HP_GPIO_SET_OUTPUT(LR1121_PIN_RST);
  
  /* Configure inputs */
  HP_GPIO_SET_INPUT(LR1121_PIN_MISO);
  HP_GPIO_SET_INPUT(LR1121_PIN_BUSY);
  
  /* Enable receiver on input pins */
  PAD_CONFIG_REG(LR1121_PIN_MISO) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  PAD_CONFIG_REG(LR1121_PIN_BUSY) |= (PADCONFIG_REN_BIT | PADCONFIG_SMT_BIT);
  
  DEBUGOUT("GPIO pins configured\n\n");
  
  /* CRITICAL: Enable GPIO mode for GPIO_26-30 via HOST_PADS_GPIO_MODE
   * Citation: siw917x-family-rm.pdf Rev 1.2, Section 24.5.15, p.612
   * At reset, these bits are 0 (Host Interface / SDIO mode), NOT GPIO mode!
   */
  DEBUGOUT("Setting HOST_PADS_GPIO_MODE for GPIO_26-30...\n");
  DEBUGOUT("  MCR_GENERIC_CTRL_1 BEFORE: 0x%08lX\n", (unsigned long)MCR_GENERIC_CTRL_1_REG);
  MCR_GENERIC_CTRL_1_REG |= HOST_PADS_GPIO_MODE_ALL;
  for (volatile int i = 0; i < 100; i++) { }
  DEBUGOUT("  MCR_GENERIC_CTRL_1 AFTER:  0x%08lX\n", (unsigned long)MCR_GENERIC_CTRL_1_REG);
  DEBUGOUT("  HOST_PADS_GPIO_MODE bits[18:14] = 0x%lX (should be 0x1F)\n",
           (unsigned long)((MCR_GENERIC_CTRL_1_REG >> 14) & 0x1F));
  
  /* Set high drive strength for output pins (E = 3 = 12mA) */
  DEBUGOUT("\nSetting PAD_CONFIG drive strength (12mA) for outputs...\n");
  PAD_CONFIG_REG(LR1121_PIN_SCK)  = (PAD_CONFIG_REG(LR1121_PIN_SCK)  & ~0x3) | 0x3;  /* 12mA */
  PAD_CONFIG_REG(LR1121_PIN_MOSI) = (PAD_CONFIG_REG(LR1121_PIN_MOSI) & ~0x3) | 0x3;  /* 12mA */
  PAD_CONFIG_REG(LR1121_PIN_NSS)  = (PAD_CONFIG_REG(LR1121_PIN_NSS)  & ~0x3) | 0x3;  /* 12mA */
  PAD_CONFIG_REG(LR1121_PIN_RST)  = (PAD_CONFIG_REG(LR1121_PIN_RST)  & ~0x3) | 0x3;  /* 12mA */
  
  /* Dump all GPIO configuration registers for debugging */
  DEBUGOUT("\n=== Complete GPIO Register Dump ===\n");
  
  DEBUGOUT("Register base addresses (verify these are correct!):\n");
  DEBUGOUT("  EGPIO_BASE           = 0x%08lX (expect 0x46130000)\n", (unsigned long)EGPIO_BASE);
  DEBUGOUT("  EGPIO_PORT1_BASE     = 0x%08lX (expect 0x46131040)\n", (unsigned long)EGPIO_PORT1_BASE);
  DEBUGOUT("  BIT_LOAD_REG(25) addr= 0x%08lX\n", (unsigned long)(EGPIO_BASE + 0x004 + (0x10 * 25)));
  DEBUGOUT("  BIT_LOAD_REG(27) addr= 0x%08lX\n", (unsigned long)(EGPIO_BASE + 0x004 + (0x10 * 27)));
  DEBUGOUT("  BIT_LOAD_REG(28) addr= 0x%08lX\n", (unsigned long)(EGPIO_BASE + 0x004 + (0x10 * 28)));
  DEBUGOUT("  BIT_LOAD_REG(30) addr= 0x%08lX\n", (unsigned long)(EGPIO_BASE + 0x004 + (0x10 * 30)));
  
  DEBUGOUT("\nClock registers:\n");
  DEBUGOUT("  CLK_ENABLE_SET_REG2 = 0x%08lX (bit21 EGPIO_PCLK)\n", (unsigned long)CLK_ENABLE_SET_REG2);
  DEBUGOUT("  CLK_ENABLE_SET_REG3 = 0x%08lX (bit16 EGPIO_CLK)\n", (unsigned long)CLK_ENABLE_SET_REG3);
  DEBUGOUT("\nControl registers:\n");
  DEBUGOUT("  MEM_GPIO_ACCESS_CTRL_SET = 0x%08lX (bit5 MCU ctrl)\n", (unsigned long)MEM_GPIO_ACCESS_CTRL_SET);
  DEBUGOUT("  MCR_GENERIC_CTRL_1_REG   = 0x%08lX (bits14-18 GPIO mode)\n", (unsigned long)MCR_GENERIC_CTRL_1_REG);
  DEBUGOUT("\nGPIO_CONFIG_REG (DIRECTION bit0: 0=out, 1=in; MODE bits5:2):\n");
  DEBUGOUT("  GPIO_25 (SCK):  0x%08lX (dir=%lu, mode=%lu)\n", 
           (unsigned long)EGPIO_GPIO_CONFIG_REG(25),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(25) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(25) >> 2) & 0xF));
  DEBUGOUT("  GPIO_26 (MISO): 0x%08lX (dir=%lu, mode=%lu)\n",
           (unsigned long)EGPIO_GPIO_CONFIG_REG(26),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(26) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(26) >> 2) & 0xF));
  DEBUGOUT("  GPIO_27 (MOSI): 0x%08lX (dir=%lu, mode=%lu)\n",
           (unsigned long)EGPIO_GPIO_CONFIG_REG(27),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(27) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(27) >> 2) & 0xF));
  DEBUGOUT("  GPIO_28 (CS):   0x%08lX (dir=%lu, mode=%lu)\n",
           (unsigned long)EGPIO_GPIO_CONFIG_REG(28),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(28) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(28) >> 2) & 0xF));
  DEBUGOUT("  GPIO_29 (BUSY): 0x%08lX (dir=%lu, mode=%lu)\n",
           (unsigned long)EGPIO_GPIO_CONFIG_REG(29),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(29) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(29) >> 2) & 0xF));
  DEBUGOUT("  GPIO_30 (RST):  0x%08lX (dir=%lu, mode=%lu)\n",
           (unsigned long)EGPIO_GPIO_CONFIG_REG(30),
           (unsigned long)(EGPIO_GPIO_CONFIG_REG(30) & 1),
           (unsigned long)((EGPIO_GPIO_CONFIG_REG(30) >> 2) & 0xF));
  DEBUGOUT("\nPAD_CONFIG_REG (E bits1:0, POS bit2, SMT bit3, REN bit4, SR bit5, P bits7:6):\n");
  DEBUGOUT("  GPIO_25 (SCK):  0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(25),
           (unsigned long)(PAD_CONFIG_REG(25) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(25) >> 4) & 1));
  DEBUGOUT("  GPIO_26 (MISO): 0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(26),
           (unsigned long)(PAD_CONFIG_REG(26) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(26) >> 4) & 1));
  DEBUGOUT("  GPIO_27 (MOSI): 0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(27),
           (unsigned long)(PAD_CONFIG_REG(27) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(27) >> 4) & 1));
  DEBUGOUT("  GPIO_28 (CS):   0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(28),
           (unsigned long)(PAD_CONFIG_REG(28) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(28) >> 4) & 1));
  DEBUGOUT("  GPIO_29 (BUSY): 0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(29),
           (unsigned long)(PAD_CONFIG_REG(29) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(29) >> 4) & 1));
  DEBUGOUT("  GPIO_30 (RST):  0x%08lX (E=%lu, REN=%lu)\n",
           (unsigned long)PAD_CONFIG_REG(30),
           (unsigned long)(PAD_CONFIG_REG(30) & 0x3),
           (unsigned long)((PAD_CONFIG_REG(30) >> 4) & 1));
  DEBUGOUT("=== End Register Dump ===\n\n");
  
  /* Start toggle test */
  uint32_t cycle_count = 0;
  
  while (cycles == 0 || cycle_count < cycles) {
    cycle_count++;
    
    DEBUGOUT("=== Cycle %lu ===\n", (unsigned long)cycle_count);
    
    /* Read and display input pins */
    DEBUGOUT("INPUT pins: MISO(26)=%lu  BUSY(29)=%lu\n",
             (unsigned long)HP_GPIO_READ(LR1121_PIN_MISO),
             (unsigned long)HP_GPIO_READ(LR1121_PIN_BUSY));
    
    /* Test GPIO_25 (SCK) */
    DEBUGOUT("\nGPIO_25 (SCK) -> HIGH (expect ~3.3V on mikroBUS pin 4)\n");
    HP_GPIO_SET_HIGH(LR1121_PIN_SCK);
    DEBUGOUT("  BIT_LOAD_REG(25) readback = %lu (expect 1)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_SCK));
    delay_ms(500);
    
    DEBUGOUT("GPIO_25 (SCK) -> LOW (expect ~0V on mikroBUS pin 4)\n");
    HP_GPIO_SET_LOW(LR1121_PIN_SCK);
    DEBUGOUT("  BIT_LOAD_REG(25) readback = %lu (expect 0)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_SCK));
    delay_ms(500);
    
    /* Test GPIO_27 (MOSI) */
    DEBUGOUT("\nGPIO_27 (MOSI) -> HIGH (expect ~3.3V on mikroBUS pin 6)\n");
    HP_GPIO_SET_HIGH(LR1121_PIN_MOSI);
    DEBUGOUT("  BIT_LOAD_REG(27) readback = %lu (expect 1)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_MOSI));
    delay_ms(500);
    
    DEBUGOUT("GPIO_27 (MOSI) -> LOW (expect ~0V on mikroBUS pin 6)\n");
    HP_GPIO_SET_LOW(LR1121_PIN_MOSI);
    DEBUGOUT("  BIT_LOAD_REG(27) readback = %lu (expect 0)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_MOSI));
    delay_ms(500);
    
    /* Test GPIO_28 (CS) */
    DEBUGOUT("\nGPIO_28 (CS) -> HIGH (expect ~3.3V on mikroBUS pin 3)\n");
    HP_GPIO_SET_HIGH(LR1121_PIN_NSS);
    DEBUGOUT("  BIT_LOAD_REG(28) readback = %lu (expect 1)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_NSS));
    delay_ms(500);
    
    DEBUGOUT("GPIO_28 (CS) -> LOW (expect ~0V on mikroBUS pin 3)\n");
    HP_GPIO_SET_LOW(LR1121_PIN_NSS);
    DEBUGOUT("  BIT_LOAD_REG(28) readback = %lu (expect 0)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_NSS));
    delay_ms(500);
    
    /* Test GPIO_30 (RST) */
    DEBUGOUT("\nGPIO_30 (RST) -> HIGH (expect ~3.3V on mikroBUS pin 16)\n");
    HP_GPIO_SET_HIGH(LR1121_PIN_RST);
    DEBUGOUT("  BIT_LOAD_REG(30) readback = %lu (expect 1)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_RST));
    delay_ms(500);
    
    DEBUGOUT("GPIO_30 (RST) -> LOW (expect ~0V on mikroBUS pin 16)\n");
    HP_GPIO_SET_LOW(LR1121_PIN_RST);
    DEBUGOUT("  BIT_LOAD_REG(30) readback = %lu (expect 0)\n", (unsigned long)HP_GPIO_READ(LR1121_PIN_RST));
    delay_ms(500);
    
    /* Re-read input pins */
    DEBUGOUT("\nINPUT pins after toggle: MISO(26)=%lu  BUSY(29)=%lu\n",
             (unsigned long)HP_GPIO_READ(LR1121_PIN_MISO),
             (unsigned long)HP_GPIO_READ(LR1121_PIN_BUSY));
    
    DEBUGOUT("\n--- End Cycle %lu ---\n\n", (unsigned long)cycle_count);
    
    /* Small delay between cycles */
    delay_ms(1000);
  }
  
  /* Restore idle state */
  DEBUGOUT("Test complete. Restoring idle state...\n");
  HP_GPIO_SET_LOW(LR1121_PIN_SCK);
  HP_GPIO_SET_LOW(LR1121_PIN_MOSI);
  HP_GPIO_SET_HIGH(LR1121_PIN_NSS);
  HP_GPIO_SET_HIGH(LR1121_PIN_RST);
  
  DEBUGOUT("Idle state: SCK=LOW, MOSI=LOW, CS=HIGH, RST=HIGH\n");
  DEBUGOUT("\n=== GPIO Toggle Test Complete ===\n\n");
  
#else
  DEBUGOUT("GPIO Toggle Test requires USE_SOFT_SPI to be defined\n");
  (void)cycles;
#endif
}

/*******************************************************************************
 * Additional Public Functions for Standalone Test Suite
 ******************************************************************************/

/**
 * @brief Wait for BUSY pin to go LOW with configurable timeout
 */
bool lr1121_wait_busy_timeout(uint32_t timeout_ms) {
  uint32_t timeout_count = timeout_ms * 10; /* 10 iterations per ms approximately */
  
  while (timeout_count > 0) {
    if (read_busy_pin() == 0) {
      return true;
    }
    delay_us(100);
    timeout_count--;
  }
  
  return false;
}

/**
 * @brief Wrapper to make lr1121_send_command public
 * Note: The static version is used internally, this wraps it for external use
 */
bool lr1121_send_command_pub(uint16_t opcode, const uint8_t *params, uint16_t param_len) {
  uint8_t tx_buf[258]; /* Max opcode (2) + params (256) */
  uint8_t rx_buf[258];
  uint16_t total_len = 2 + param_len;

  if (total_len > sizeof(tx_buf)) {
    return false;
  }

  tx_buf[0] = (opcode >> 8) & 0xFF;
  tx_buf[1] = opcode & 0xFF;

  if (params != NULL && param_len > 0) {
    memcpy(&tx_buf[2], params, param_len);
  }

  cs_assert();
  bool result = spi_transfer(tx_buf, rx_buf, total_len);
  cs_deassert();

  return result;
}

/* Alias for external linkage */
bool lr1121_send_command(uint16_t opcode, const uint8_t *params, uint16_t param_len) {
  return lr1121_send_command_pub(opcode, params, param_len);
}

/**
 * @brief Wrapper to make lr1121_read_response public
 */
bool lr1121_read_response(uint8_t *response, uint16_t response_len) {
  uint8_t tx_buf[258];

  if (response_len > sizeof(tx_buf)) {
    return false;
  }

  memset(tx_buf, 0x00, response_len);
  memset(response, 0xBB, response_len);  /* Pre-fill to detect if SPI writes anything */

  cs_assert();
  bool result = spi_transfer(tx_buf, response, response_len);
  cs_deassert();
  
  /* DEBUG: Show raw SPI read result */
  DEBUGOUT("[SPI_RD] len=%d:", response_len);
  for (int i = 0; i < (response_len > 20 ? 20 : response_len); i++) {
    DEBUGOUT(" %02X", response[i]);
  }
  DEBUGOUT("\n");

  return result;
}

/**
 * @brief Get LR1121 status bytes
 * 
 * Citation: LR1121 User Manual Section 2.1 (GetStatus)
 * GetStatus returns status in the response bytes during command phase.
 */
bool lr1121_get_status(uint8_t *stat1, uint8_t *stat2, uint8_t *irq_status) {
  uint8_t tx_buf[4] = {0x01, 0x00, 0x00, 0x00};  /* GetStatus opcode + 2 NOP */
  uint8_t rx_buf[4];

  if (!lr1121_wait_busy_timeout(100)) {
    return false;
  }

  cs_assert();
  bool result = spi_transfer(tx_buf, rx_buf, 4);
  cs_deassert();

  if (result) {
    /* Response format: [dummy][stat1][stat2][irq] */
    if (stat1) *stat1 = rx_buf[1];
    if (stat2) *stat2 = rx_buf[2];
    if (irq_status) *irq_status = rx_buf[3];
  }

  return result;
}

/* Note: lr1121_wait_busy(void) is defined earlier in this file for internal use.
 * Use lr1121_wait_busy_timeout() for external calls that need configurable timeout. */

/*******************************************************************************
 * Public Raw SPI Functions for Single-Phase Commands
 * 
 * These wrapper functions expose the internal static SPI functions for
 * commands like GetTemperature (0x011A) and GetRandomNumber (0x0120) that
 * require single-phase SPI transactions where the response is returned
 * DURING the command transaction, not in a separate Phase 2 NOP read.
 * 
 * Citation: LR1121 User Manual - Instant commands return data during the
 * command phase itself (opcode + NOPs in a single CS assertion).
 ******************************************************************************/

/**
 * @brief Public wrapper: Assert CS (drive LOW) for SPI transaction
 * 
 * Citation: LR1121 Datasheet Section 3 "SPI Interface"
 * NSS must be driven LOW to select the LR1121 for SPI communication.
 */
void lr1121_cs_assert(void) {
  cs_assert();
}

/**
 * @brief Public wrapper: Deassert CS (drive HIGH) after SPI transaction
 * 
 * Citation: LR1121 Datasheet Section 3 "SPI Interface"
 * NSS rising edge triggers command processing in the LR1121.
 */
void lr1121_cs_deassert(void) {
  cs_deassert();
}

/**
 * @brief Public wrapper: Raw SPI transfer (full-duplex)
 *
 * Performs a full-duplex SPI transfer without CS control.
 * Caller must handle CS assertion/deassertion using lr1121_cs_assert()
 * and lr1121_cs_deassert().
 *
 * Citation: siw917x-family-rm.pdf Section 20 "Generic SPI Primary (GSPI)"
 * Full-duplex mode transfers data in both directions simultaneously.
 *
 * @param tx_data Data to transmit (can be NULL for receive-only)
 * @param rx_data Buffer for received data (can be NULL for transmit-only)
 * @param length Number of bytes to transfer
 * @return true on success, false on error
 */
bool lr1121_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length) {
  return spi_transfer(tx_data, rx_data, length);
}

/*******************************************************************************
 * FIRMWARE UPDATE SUPPORT
 * 
 * Citation: ExpressLRS LR1121.cpp - Firmware update implementation
 ******************************************************************************/

/* Additional bootloader opcodes needed for firmware update */
#define LR11XX_SYSTEM_REBOOT_OC           0x0118  /* Reboot command */

/* Firmware update state structure */
typedef struct {
  uint32_t expected_size;    /**< Expected firmware size from X-FileSize header */
  uint32_t total_size;       /**< Total bytes written to flash */
  uint32_t left_over;        /**< Bytes remaining in buffer (< 256) */
  uint8_t buffer[256];       /**< Write buffer (256 bytes per flash write) */
  bool in_progress;          /**< Update is in progress */
} lr1121_update_state_t;

/* Static update state (single update at a time) */
static lr1121_update_state_t lr1121_update_state = { 0 };

/**
 * @brief Write buffered data to LR1121 flash (internal helper)
 */
static bool lr1121_write_flash_chunk(const uint8_t *data, uint32_t data_size) {
  uint8_t packet[262];  /* 2B opcode + 4B address + 256B data */
  uint32_t write_size;
  uint32_t flash_address = lr1121_update_state.total_size;

  /* Build command header */
  packet[0] = (uint8_t)(LR1121_OPCODE_BL_WRITE_FLASH >> 8);   /* 0x80 */
  packet[1] = (uint8_t)(LR1121_OPCODE_BL_WRITE_FLASH & 0xFF); /* 0x03 */
  packet[2] = (uint8_t)(flash_address >> 24);  /* Address MSB */
  packet[3] = (uint8_t)(flash_address >> 16);
  packet[4] = (uint8_t)(flash_address >> 8);
  packet[5] = (uint8_t)(flash_address);        /* Address LSB */

  /* Calculate write size */
  write_size = lr1121_update_state.left_over;
  if (data != NULL) {
    memcpy(lr1121_update_state.buffer + lr1121_update_state.left_over, data, data_size);
    write_size += data_size;
  }

  /* Copy buffer to packet */
  memcpy(&packet[6], lr1121_update_state.buffer, write_size);

  DEBUGOUT("LR1121 OTA: Write 0x%08lX (%lu bytes)\n", 
           (unsigned long)flash_address, (unsigned long)write_size);

  /* Wait for BUSY LOW before sending */
  if (!lr1121_wait_busy_timeout(1000)) {
    DEBUGOUT("LR1121 OTA: ERROR - BUSY timeout before flash write\n");
    return false;
  }

  /* Send write command */
  cs_assert();
  bool spi_ok = spi_transfer(packet, NULL, 6 + write_size);
  cs_deassert();

  if (!spi_ok) {
    DEBUGOUT("LR1121 OTA: ERROR - SPI transfer failed\n");
    return false;
  }

  /* Wait for flash write to complete */
  if (!lr1121_wait_busy_timeout(5000)) {
    DEBUGOUT("LR1121 OTA: ERROR - BUSY timeout after flash write\n");
    return false;
  }

  /* Update state */
  lr1121_update_state.total_size += write_size;
  lr1121_update_state.left_over = 0;

  return true;
}

/**
 * @brief Get LR1121 firmware version using specified command opcode
 */
bool lr1121_get_firmware_version(lr1121_firmware_version_t *version, uint16_t opcode) {
  uint8_t response[5];  /* stat1 + HW + Type + VerMSB + VerLSB */

  if (version == NULL) {
    return false;
  }

  /* Phase 1: Send GetVersion command */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("LR1121 OTA: BUSY timeout before GetVersion\n");
    return false;
  }

  if (!lr1121_send_command(opcode, NULL, 0)) {
    DEBUGOUT("LR1121 OTA: Failed to send GetVersion command\n");
    return false;
  }

  /* Phase 2: Wait for command processing and read response */
  if (!lr1121_wait_busy_timeout(100)) {
    DEBUGOUT("LR1121 OTA: BUSY timeout after GetVersion\n");
    return false;
  }

  if (!lr1121_read_response(response, 5)) {
    DEBUGOUT("LR1121 OTA: Failed to read GetVersion response\n");
    return false;
  }

  /* Parse response: [stat1][HW][Type][VerMSB][VerLSB] */
  version->hardware = response[1];
  version->type = response[2];
  version->version = (uint16_t)((response[3] << 8) | response[4]);

  DEBUGOUT("LR1121 OTA: Version HW=0x%02X Type=0x%02X FW=0x%04X\n",
           version->hardware, version->type, version->version);

  return true;
}

/**
 * @brief Begin LR1121 firmware update
 */
int lr1121_begin_update(uint32_t expected_size) {
  lr1121_firmware_version_t version;
  uint8_t mode;

  DEBUGOUT("\n=== LR1121 OTA: Beginning firmware update ===\n");
  DEBUGOUT("LR1121 OTA: Expected size: %lu bytes\n", (unsigned long)expected_size);

  /* Initialize update state */
  memset(&lr1121_update_state, 0, sizeof(lr1121_update_state));
  lr1121_update_state.expected_size = expected_size;
  lr1121_update_state.in_progress = true;

  /* Step 1: Reboot LR1121 to bootloader mode */
  DEBUGOUT("LR1121 OTA: Rebooting to bootloader mode...\n");
  mode = 3;  /* Bootloader mode */
  if (!lr1121_send_command(LR11XX_SYSTEM_REBOOT_OC, &mode, 1)) {
    DEBUGOUT("LR1121 OTA: Failed to send reboot command\n");
    lr1121_update_state.in_progress = false;
    return -1;
  }

  /* Wait for reboot to complete */
  if (!lr1121_wait_busy_timeout(1000)) {
    DEBUGOUT("LR1121 OTA: Timeout waiting for bootloader\n");
    lr1121_update_state.in_progress = false;
    return -1;
  }

  /* Step 2: Verify we're in bootloader mode */
  DEBUGOUT("LR1121 OTA: Verifying bootloader mode...\n");
  if (!lr1121_get_firmware_version(&version, LR1121_OPCODE_BL_GET_VERSION)) {
    DEBUGOUT("LR1121 OTA: Failed to get bootloader version\n");
    lr1121_update_state.in_progress = false;
    return -1;
  }

  if (version.type != 0xDF) {
    DEBUGOUT("LR1121 OTA: Not in bootloader! type=0x%02X (expected 0xDF)\n", version.type);
    lr1121_update_state.in_progress = false;
    return -1;
  }
  DEBUGOUT("LR1121 OTA: Bootloader confirmed (type=0xDF)\n");

  /* Step 3: Erase flash */
  DEBUGOUT("LR1121 OTA: Erasing flash (this takes ~3 seconds)...\n");
  
  uint8_t erase_tx[2] = {0x80, 0x01};  /* BL_ERASE_FLASH_OC */
  cs_assert();
  bool result = spi_transfer(erase_tx, NULL, 2);
  cs_deassert();
  
  if (!result) {
    DEBUGOUT("LR1121 OTA: Failed to send erase command\n");
    lr1121_update_state.in_progress = false;
    return -1;
  }

  /* Wait for erase to complete with polling */
  uint32_t elapsed_ms = 0;
  while (elapsed_ms < 10000) {
    int busy = read_busy_pin();
    if (busy == 0) {
      DEBUGOUT("LR1121 OTA: Erase complete after %lu ms\n", (unsigned long)elapsed_ms);
      break;
    }
    delay_ms(100);
    elapsed_ms += 100;
  }

  if (elapsed_ms >= 10000) {
    DEBUGOUT("LR1121 OTA: TIMEOUT waiting for flash erase!\n");
    lr1121_update_state.in_progress = false;
    return -1;
  }

  /* Add settling time */
  delay_ms(100);

  DEBUGOUT("LR1121 OTA: Ready for firmware upload\n");
  return 0;
}

/**
 * @brief Write firmware data chunk to LR1121
 */
int lr1121_write_update_bytes(const uint8_t *data, uint32_t size) {
  if (!lr1121_update_state.in_progress) {
    DEBUGOUT("LR1121 OTA: No update in progress!\n");
    return -1;
  }

  /* Process data in 256-byte chunks */
  while (size >= 256 - lr1121_update_state.left_over) {
    uint32_t chunk_size = 256 - lr1121_update_state.left_over;
    if (chunk_size > size) {
      chunk_size = size;
    }
    
    if (!lr1121_write_flash_chunk(data, chunk_size)) {
      DEBUGOUT("LR1121 OTA: Flash write failed!\n");
      lr1121_update_state.in_progress = false;
      return -1;
    }
    
    size -= chunk_size;
    data += chunk_size;
  }

  /* Store remaining data in buffer */
  if (size > 0) {
    memcpy(lr1121_update_state.buffer + lr1121_update_state.left_over, data, size);
    lr1121_update_state.left_over += size;
  }

  return 0;
}

/**
 * @brief Complete LR1121 firmware update
 */
int lr1121_end_update(void) {
  lr1121_firmware_version_t version;
  uint8_t param;

  if (!lr1121_update_state.in_progress) {
    DEBUGOUT("LR1121 OTA: No update in progress!\n");
    return -1;
  }

  DEBUGOUT("\n=== LR1121 OTA: Completing firmware update ===\n");

  /* Step 1: Flush remaining buffered data */
  if (lr1121_update_state.left_over > 0) {
    DEBUGOUT("LR1121 OTA: Flushing %lu remaining bytes\n", 
             (unsigned long)lr1121_update_state.left_over);
    if (!lr1121_write_flash_chunk(NULL, 0)) {
      DEBUGOUT("LR1121 OTA: Final flush failed!\n");
      lr1121_update_state.in_progress = false;
      return -1;
    }
  }

  /* Verify size match */
  if (lr1121_update_state.total_size != lr1121_update_state.expected_size) {
    DEBUGOUT("LR1121 OTA: Size mismatch! Expected %lu, got %lu\n",
             (unsigned long)lr1121_update_state.expected_size,
             (unsigned long)lr1121_update_state.total_size);
    lr1121_update_state.in_progress = false;
    return -1;
  }

  /* Step 2: Reboot from bootloader to application */
  DEBUGOUT("LR1121 OTA: Rebooting to application...\n");
  param = 0;
  if (!lr1121_send_command(LR1121_OPCODE_BL_REBOOT, &param, 1)) {
    DEBUGOUT("LR1121 OTA: Failed to send reboot command\n");
    lr1121_update_state.in_progress = false;
    return -2;
  }

  /* Wait for reboot */
  delay_ms(300);

  if (!lr1121_wait_busy_timeout(2000)) {
    DEBUGOUT("LR1121 OTA: Timeout waiting for application boot\n");
    lr1121_update_state.in_progress = false;
    return -2;
  }

  /* Step 3: Verify no longer in bootloader mode */
  DEBUGOUT("LR1121 OTA: Verifying application mode...\n");
  if (!lr1121_get_firmware_version(&version, LR1121_OPCODE_GET_VERSION)) {
    DEBUGOUT("LR1121 OTA: Failed to get application version\n");
    lr1121_update_state.in_progress = false;
    return -2;
  }

  if (version.type == 0xDF) {
    DEBUGOUT("LR1121 OTA: Still in bootloader mode! Update FAILED.\n");
    lr1121_update_state.in_progress = false;
    return -3;
  }

  DEBUGOUT("LR1121 OTA: Firmware update complete!\n");
  lr1121_update_state.in_progress = false;
  return 0;
}

/*******************************************************************************
 * DIO1 INTERRUPT SUPPORT
 * 
 * Citation: siw917x-family-rm.pdf Section 11.10 "UULP GPIO Interrupts"
 * DIO1 is connected to UULP_VBAT_GPIO_2 on BRD2708A
 ******************************************************************************/

/* UULP GPIO Base Address */
#define UULP_GPIO_INTR_BASE   0x12080000UL

/* UULP_VBAT_GPIO_2 Configuration Register (for DIO1 input) */
#define UULP_VBAT_GPIO2_CONFIG_REG  (*(volatile uint32_t *)(0x24048620UL))

/* UULP GPIO Interrupt Configuration and Status Registers */
#define UULP_GPIO_CONFIG_REG  (*(volatile uint32_t *)(UULP_GPIO_INTR_BASE + 0x010))
#define UULP_GPIO_STATUS_REG  (*(volatile uint32_t *)(UULP_GPIO_INTR_BASE + 0x014))

/* Interrupt enable bits for UULP_VBAT_GPIO_2 */
#define UULP_GPIO2_RE_EN_BIT  (1UL << 2)   /* Rising Edge Enable */
#define UULP_GPIO2_FE_EN_BIT  (1UL << 10)  /* Falling Edge Enable */
#define UULP_GPIO2_LL_EN_BIT  (1UL << 18)  /* Level Low Enable */
#define UULP_GPIO2_HL_EN_BIT  (1UL << 27)  /* Level High Enable */

/* UULP GPIO Config Register bits */
#define UULP_GPIO_MODE_MASK       0x07UL
#define UULP_GPIO_MODE_GPIO       0x00UL
#define UULP_GPIO_DIRECTION_BIT   (1UL << 7)  /* 0=Output, 1=Input */
#define UULP_GPIO_REN_BIT         (1UL << 4)  /* Receiver Enable */

/* Static callback storage */
static lr1121_dio1_callback_t dio1_callback = NULL;

/**
 * @brief Initialize DIO1 interrupt support
 */
lr1121_status_t lr1121_dio1_init(void)
{
  DEBUGOUT("LR1121: Initializing DIO1 interrupt (UULP_VBAT_GPIO_2)...\n");
  
  /* Step 1: Configure UULP_VBAT_GPIO_2 as GPIO input */
  uint32_t gpio_config = UULP_VBAT_GPIO2_CONFIG_REG;
  
  gpio_config &= ~UULP_GPIO_MODE_MASK;
  gpio_config |= UULP_GPIO_MODE_GPIO;
  gpio_config |= UULP_GPIO_DIRECTION_BIT;  /* Input */
  gpio_config |= UULP_GPIO_REN_BIT;        /* Receiver enable */
  
  UULP_VBAT_GPIO2_CONFIG_REG = gpio_config;
  
  /* Wait for configuration to settle */
  for (volatile int i = 0; i < 100; i++) { }
  
  DEBUGOUT("  UULP_VBAT_GPIO2_CONFIG_REG = 0x%08lX\n", 
           (unsigned long)UULP_VBAT_GPIO2_CONFIG_REG);
  
  /* Step 2: Configure rising-edge interrupt for DIO1 */
  uint32_t intr_config = UULP_GPIO_CONFIG_REG;
  
  /* Clear existing interrupt enables for GPIO_2 */
  intr_config &= ~(UULP_GPIO2_RE_EN_BIT | UULP_GPIO2_FE_EN_BIT | 
                   UULP_GPIO2_LL_EN_BIT | UULP_GPIO2_HL_EN_BIT);
  
  /* Enable rising-edge interrupt */
  intr_config |= UULP_GPIO2_RE_EN_BIT;
  
  UULP_GPIO_CONFIG_REG = intr_config;
  
  /* Wait for configuration to settle */
  for (volatile int i = 0; i < 100; i++) { }
  
  DEBUGOUT("  UULP_GPIO_CONFIG_REG = 0x%08lX\n", 
           (unsigned long)UULP_GPIO_CONFIG_REG);
  
  /* Step 3: Clear any pending interrupt status */
  UULP_GPIO_STATUS_REG = 0xFFFFFFFFUL;
  
  DEBUGOUT("LR1121: DIO1 interrupt initialized (currently %s)\n",
           lr1121_dio1_read() ? "HIGH" : "LOW");
  
  return LR1121_OK;
}

/**
 * @brief Enable DIO1 interrupt in NVIC
 */
void lr1121_dio1_enable(void) {
  #define UULP_GPIO_IRQn 5
  
  /* Enable UULP GPIO interrupt in NVIC */
  NVIC_EnableIRQ((IRQn_Type)UULP_GPIO_IRQn);
  NVIC_SetPriority((IRQn_Type)UULP_GPIO_IRQn, 5);
  
  /* Unmask UULP GPIO interrupt (bit 2 for UULP_VBAT_GPIO_2) */
  volatile uint32_t *uulp_intr_mask_clr = (volatile uint32_t *)0x12080004;
  *uulp_intr_mask_clr = (1 << 2);
  
  DEBUGOUT("LR1121: DIO1 interrupt enabled (IRQn=%d)\n", UULP_GPIO_IRQn);
}

/**
 * @brief Disable DIO1 interrupt in NVIC
 */
void lr1121_dio1_disable(void)
{
  #define UULP_GPIO_IRQn 5
  
  /* Disable UULP GPIO interrupt in NVIC */
  NVIC_DisableIRQ((IRQn_Type)UULP_GPIO_IRQn);
  
  /* Mask UULP GPIO interrupt */
  volatile uint32_t *uulp_intr_mask_set = (volatile uint32_t *)0x12080000;
  *uulp_intr_mask_set = (1 << 2);
  
  DEBUGOUT("LR1121: DIO1 interrupt disabled\n");
}

/**
 * @brief Read current state of DIO1 pin
 */
int lr1121_dio1_read(void)
{
  return (UULP_GPIO_STATUS_REG >> 2) & 0x01;
}

/**
 * @brief Register a callback function for DIO1 interrupts
 */
void lr1121_dio1_set_callback(lr1121_dio1_callback_t callback)
{
  dio1_callback = callback;
  DEBUGOUT("LR1121: DIO1 callback %s\n", callback ? "registered" : "unregistered");
}

/**
 * @brief DIO1 ISR handler (called from IRQ handler)
 */
void lr1121_dio1_isr_handler(void)
{
  /* Read interrupt status */
  uint32_t status = UULP_GPIO_STATUS_REG;
  
  /* Check if GPIO_2 interrupt is active */
  if (status & (1UL << 2)) {
    /* Clear the interrupt by writing 1 to the status bit */
    UULP_GPIO_STATUS_REG = (1UL << 2);
    
    /* Call registered callback if set */
    if (dio1_callback != NULL) {
      dio1_callback();
    }
  }
}

/**
 * @brief UULP GPIO IRQ Handler - Entry point from NVIC
 */
void NPSS_TO_MCU_GPIO_INTR_IRQHandler(void)
{
  lr1121_dio1_isr_handler();
}

/**
 * @brief Flash ELRS firmware to LR1121 (stub)
 * 
 * This is a stub function that assumes the ELRS firmware is already 
 * flashed on the LR1121. The actual firmware flashing code was in 
 * lr1121_driver_backup.c but is not needed if firmware is pre-flashed.
 * 
 * The ELRS firmware provides custom opcodes for optimized packet handling:
 *   - 0x0700 GetPacket - Combined packet retrieval
 *   - 0x0701 SetFreqSetRx - Combined frequency set and RX entry
 * 
 * @return 0 on success (always returns success as firmware assumed present)
 */
int lr1121_flash_elrs_firmware(void)
{
  DEBUGOUT("[LR1121] ELRS firmware flash skipped (assuming pre-flashed)\n");
  return 0;  /* Success - firmware already flashed */
}
