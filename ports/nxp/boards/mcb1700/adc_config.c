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
#include "boards/mcb1700/adc_config.h"
#include "nxp_lpcopen/lpc175x_6x/lpc_chip_175x_6x/inc/chip.h"


const STATIC adc_pin_set_t __ADC0_pin_set[] =
{
    /* Pin set 0 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 23),
        .func = 1U,
    },

    /* Pin set 1 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 24),
        .func = 1U,
    },

    /* Pin set 2 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 25),
        .func = 1U,
    },

    /* Pin set 3 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 26),
        .func = 1U,
    },

    /* Pin set 4 */
    {
        .pin = NXP_PORT_GPIO_PIN(1, 30),
        .func = 3U,
    },

    /* Pin set 5 */
    {
        .pin = NXP_PORT_GPIO_PIN(1, 31),
        .func = 3U,
    },

    #if (1)
    /* P0.2 and P0.3 are connected to COM1 only.
     * It makes no sense to use ADC on this pins. */
    #else
    /* Pin set 6 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 3),
        .func = 2U,
    },

    /* Pin set 7 */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 2),
        .func = 2U,
    },
    #endif
};


STATIC adc_inst_t __adc_instances[ADC_INSTANCES_NUM] = {
    /*  Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .adc_dev = (adc_dev_t *)LPC_ADC,
        .pin_map = &__ADC0_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(__ADC0_pin_set),
    },
};


adc_inst_t *get_adc_instance(const size_t n) {
    if (n < ADC_INSTANCES_NUM) {
        return &__adc_instances[n];
    } else {
        return (adc_inst_t *)NULL;
    }
}

void adc_enable(adc_inst_t *adc_instance, size_t channel) {

    if ((0U == adc_instance->id) && (MP_ARRAY_SIZE(__ADC0_pin_set) > channel)) {
        const uint8_t pin = __ADC0_pin_set[channel].pin;
        const uint8_t func = __ADC0_pin_set[channel].func;
        const uint8_t port = NXP_PORT_GPIO_PIN_PORT(pin);
        const uint8_t pin_number = NXP_PORT_GPIO_PIN_NUMBER(pin);
        Chip_IOCON_PinMux(LPC_IOCON, port, pin_number, 0U, func);

        /* ... Clocks are enabled by Chip_ADC_Init function */
    }

    return;
}

void adc_disable(adc_inst_t *adc_instance, size_t channel) {
    if ((0U == adc_instance->id) && (MP_ARRAY_SIZE(__ADC0_pin_set) > channel)) {
        const uint8_t pin = __ADC0_pin_set[channel].pin;
        const uint8_t port = NXP_PORT_GPIO_PIN_PORT(pin);
        const uint8_t pin_number = NXP_PORT_GPIO_PIN_NUMBER(pin);

        /* ... Configure as GPIO */
        Chip_IOCON_PinMux(LPC_IOCON, port, pin_number, 0U, 0U);

        /* ... Clocks are disabled by Chip_ADC_DeInit function */
    }

    return;
}
