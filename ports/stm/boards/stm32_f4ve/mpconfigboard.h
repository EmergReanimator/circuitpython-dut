/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Lucian Copeland for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Micropython setup

#define MICROPY_HW_BOARD_NAME       "STM32F4_F4VE"
#define MICROPY_HW_MCU_NAME         "STM32F407VE"

#define FLASH_SIZE                  (0x80000)   // 512 KiB
#define FLASH_PAGE_SIZE             (0x4000)

#define HSE_VALUE ((uint32_t)8000000)
#define LSE_VALUE ((uint32_t)32768)
#define BOARD_HAS_LOW_SPEED_CRYSTAL (1)

// On-board flash
#define SPI_FLASH_MOSI_PIN          (&pin_PB05)
#define SPI_FLASH_MISO_PIN          (&pin_PB04)
#define SPI_FLASH_SCK_PIN           (&pin_PB03)
#define SPI_FLASH_CS_PIN            (&pin_PB00)

#define BOARD_NO_VBUS_SENSE         (1)
#define BOARD_NO_USB_OTG_ID_SENSE   (1)

// Bootloader only
#ifdef UF2_BOOTLOADER_ENABLED
    #define BOARD_VTOR_DEFER (1) // Leave VTOR relocation to bootloader
#endif

#define DEFAULT_I2C_BUS_SCL         (&pin_PB08)
#define DEFAULT_I2C_BUS_SDA         (&pin_PB09)

#define DEFAULT_SPI_BUS_SCK         (&pin_PB13)
#define DEFAULT_SPI_BUS_MOSI        (&pin_PB15)
#define DEFAULT_SPI_BUS_MISO        (&pin_PB14)

#define DEFAULT_UART_BUS_RX         (&pin_PA03)
#define DEFAULT_UART_BUS_TX         (&pin_PA02)

#if (0)
#define DEBUG_UART_RX               (&pin_PA10)
#define DEBUG_UART_TX               (&pin_PA09)
#endif
