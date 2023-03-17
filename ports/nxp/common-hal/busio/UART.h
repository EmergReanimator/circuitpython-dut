/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 microDev
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

#ifndef MICROPY_INCLUDED_NXP_COMMON_HAL_BUSIO_UART_H
#define MICROPY_INCLUDED_NXP_COMMON_HAL_BUSIO_UART_H

#include "cmsis5/CMSIS/Driver/Include/Driver_USART.h"
#include "common-hal/microcontroller/Pin.h"
#include "py/ringbuf.h"
#include "py/obj.h"

typedef struct {
    uint8_t tx;
    uint8_t rx;
    uint8_t cts;
    uint8_t rts;
} uart_pin_set_t;

typedef volatile ringbuf_t uart_ringbuf_t;

typedef struct {
    const size_t id;
    bool is_used;
    const ARM_DRIVER_USART *driver;
    const uart_pin_set_t *pin_map;
    const size_t pin_map_len;
    uint8_t buf[1U];
    uart_ringbuf_t ringbuf;
} uart_inst_t;

typedef struct {
    mp_obj_base_t base;
    uart_inst_t *uart_instance;
    const mcu_pin_obj_t *tx;
    const mcu_pin_obj_t *rx;
    const mcu_pin_obj_t *cts;
    const mcu_pin_obj_t *rts;
    uint32_t baudrate;
    uint32_t timeout_ms;
} busio_uart_obj_t;

extern void reset_uart(void);
extern void never_reset_uart(uint8_t num);

#endif // MICROPY_INCLUDED_NXP_COMMON_HAL_BUSIO_UART_H
