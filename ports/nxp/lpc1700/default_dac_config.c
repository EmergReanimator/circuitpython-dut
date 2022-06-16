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

#include "common-hal/analogio/AnalogIn.h"
#include "lpc1700/dac_config.h"
#include "nxp_lpcopen/lpc175x_6x/lpc_chip_175x_6x/inc/chip.h"


const STATIC dac_pin_set_t __DAC0_pin_set[] =
{
    /* Pin set 0 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 26),
        .func = 2U,
    },
};


STATIC dac_inst_t __dac_instances[DAC_INSTANCES_NUM] = {
    /*  Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .dac_dev = (dac_dev_t *)LPC_DAC,
        .pin_map = &__DAC0_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(__DAC0_pin_set),
    },
};


dac_inst_t *get_dac_instance(const size_t n) {
    if (n < DAC_INSTANCES_NUM) {
        return &__dac_instances[n];
    } else {
        return (dac_inst_t *)NULL;
    }
}

void dac_enable(dac_inst_t *dac_instance) {

    if (0U == dac_instance->id) {
        const uint8_t pin = __DAC0_pin_set[0U].pin;
        const uint8_t func = __DAC0_pin_set[0U].func;
        const uint8_t port = NXP_PORT_GPIO_PIN_PORT(pin);
        const uint8_t pin_number = NXP_PORT_GPIO_PIN_NUMBER(pin);
        Chip_IOCON_PinMux(LPC_IOCON, port, pin_number, 0U, func);

        /* ... Clocks are enabled by Chip_DAC_Init function */
    }

    return;
}

void dac_disable(dac_inst_t *dac_instance) {
    if (0U == dac_instance->id) {
        const uint8_t pin = __DAC0_pin_set[0U].pin;
        const uint8_t port = NXP_PORT_GPIO_PIN_PORT(pin);
        const uint8_t pin_number = NXP_PORT_GPIO_PIN_NUMBER(pin);

        /* ... Configure as GPIO */
        Chip_IOCON_PinMux(LPC_IOCON, port, pin_number, 0U, 0U);

        /* ... Clocks are disabled by Chip_DAC_DeInit function */
    }

    return;
}
