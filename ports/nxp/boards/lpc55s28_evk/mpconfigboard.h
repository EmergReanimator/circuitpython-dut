/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
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

#if !defined(PORTS_NXP_BOARDS_LPC55S28_EVK_MPCONFIGBOARD_H_)
#define PORTS_NXP_BOARDS_LPC55S28_EVK_MPCONFIGBOARD_H_

#define MICROPY_HW_BOARD_NAME "NXP LPC55S28 EVK"
#define MICROPY_HW_MCU_NAME "lpc55s28"

#define CIRCUITPY_INTERNAL_NVM_SIZE                 (0)
#define CIRCUITPY_INTERNAL_FLASH_FILESYSTEM_SIZE    (128 * 1024)

/* TODO: Move memory definitions into linker file */
/* FIXME: First 8 bytes are overwritten by a ghost as soon as
 * dcd_reg->DEVCMDSTAT has been written in dcd_init. */
#define MAGIC_SIZE                                  (16)
#define SRAM_ADDR                                   (0x20000000 + MAGIC_SIZE)
#define SRAM_SIZE                                   (256 * 1024 - MAGIC_SIZE)
#define AHBRAM_ADDR                                 (0x04000000)
#define AHBRAM_SIZE                                 (32 * 1024)
#define RAM_ADDR                                    (SRAM_ADDR)
#define RAM_SIZE                                    (SRAM_SIZE + AHBRAM_SIZE)
#define CIRCUITPY_DEFAULT_STACK_SIZE                (8 * 1024)
#define CIRCUITPY_DEFAULT_HEAP_SIZE                 (AHBRAM_SIZE - CIRCUITPY_DEFAULT_STACK_SIZE)

#if (1)
#define CIRCUITPY_CONSOLE_UART_TX                     (&pin_P0_30)
#define CIRCUITPY_CONSOLE_UART_RX                     (&pin_P0_29)
#endif

#if (1)
#define CIRCUITPY_STATUS_LED_POWER                  (&pin_P1_4)
#define MICROPY_HW_LED_STATUS                       (&pin_P1_7)
#endif

#endif // PORTS_NXP_BOARDS_LPC55S28_EVK_MPCONFIGBOARD_H_
