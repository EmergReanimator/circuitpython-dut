/*
 * This file is part of the Micro Python project, http://micropython.org/
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

// DO NOT include this file directly. Use shared-bindings/microcontroller/Pin.h instead to ensure
// that all necessary includes are already included.

#ifndef MICROPY_INCLUDED_NXP_PERIPHERALS_PINS_H
#define MICROPY_INCLUDED_NXP_PERIPHERALS_PINS_H

#include "py/obj.h"

// Used to track the pin usage
typedef struct {
    const uint32_t available_pin_mask;
    uint32_t reserved_pin_mask;
    uint32_t used_pin_mask;
} gpio_port_obj_t;

typedef struct {
    mp_obj_base_t base;
    // TODO: Consider to replace (port, number) tuple with single designator
    uint8_t port : 3;
    uint8_t number : 5;
} mcu_pin_obj_t;

#define NXP_PORT_GPIO_PIN(port, number)  ((port << 5) | (number))
#define NXP_PORT_GPIO_PIN_PORT(pin)      ((pin >> 5))
#define NXP_PORT_GPIO_PIN_NUMBER(pin)    ((pin & 0x1FU))

// This macro is used to simplify pin definition in boards/<board>/pins.c
#define PIN(p_port, p_number) \
    const mcu_pin_obj_t pin_P##p_port##_##p_number = { \
        { &mcu_pin_type }, \
        .port = p_port, \
        .number = p_number \
    }

extern const mp_obj_type_t mcu_pin_type;

typedef enum _gpio_pin_mode
{
    GPIO_Mode_PullUp    = 0U,
    GPIO_Mode_PullDown  = 1U,
    GPIO_Mode_PullNone  = 2U,
    GPIO_Mode_OpenDrain = 3U,
} gpio_pin_mode_t;


typedef struct _pin_config
{
    bool input;
    bool outputLogic;
    gpio_pin_mode_t pinMode;
} pin_config_t;


int gpio_pin_init(uint8_t port, uint8_t number, pin_config_t *config);
int gpio_pin_dir(uint8_t port, uint8_t number, bool input);
int gpio_pin_write(uint8_t port, uint8_t number, bool value);
bool gpio_pin_read(uint8_t port, uint8_t number);


#if defined(LPC1700)
#include "lpc1700/pins.h"
#elif defined(LPC55S00)
#include "lpc55s00/pins.h"
#endif

#endif  // MICROPY_INCLUDED_NXP_PERIPHERALS_PINS_H
