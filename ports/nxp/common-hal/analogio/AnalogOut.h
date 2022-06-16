/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Scott Shawcroft
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

#ifndef MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGOUT_H
#define MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGOUT_H

#include "common-hal/microcontroller/Pin.h"
#include "driver/dac.h"

#include "py/obj.h"


typedef struct _dac_pin_set {
    const uint8_t pin;
    const uint8_t func;
} dac_pin_set_t;

typedef struct _dac_inst_t
{
    const size_t id;
    bool is_used;
    dac_dev_t *dac_dev;
    const dac_pin_set_t *pin_map;
    const size_t pin_map_len;
} dac_inst_t;

typedef struct {
    mp_obj_base_t base;
    dac_inst_t *dac_instance;
    const mcu_pin_obj_t *pin;
} analogio_analogout_obj_t;


#endif // MICROPY_INCLUDED_NXP_COMMON_HAL_ANALOGIO_ANALOGOUT_H
