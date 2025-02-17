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

#ifndef MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGIN_H
#define MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGIN_H

#include "common-hal/microcontroller/Pin.h"
#include "driver/adc.h"

#include "py/obj.h"


typedef struct _adc_pin_set {
    const uint8_t pin;
    const uint8_t func;
} adc_pin_set_t;

typedef struct _adc_inst_t
{
    const size_t id;
    bool is_used;
    adc_dev_t *adc_dev;
    const adc_pin_set_t *pin_map;
    const size_t pin_map_len;
} adc_inst_t;

typedef struct {
    mp_obj_base_t base;
    size_t adc_channel;
    adc_inst_t *adc_instance;
    const mcu_pin_obj_t *pin;
    uint32_t timeout;
} analogio_analogin_obj_t;

void analogin_init(void);

#endif // MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGIN_H
