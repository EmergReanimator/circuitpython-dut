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

#include "lpc55s00/adc_config.h"
#include "lpc55s00/pin_mux.h"
#include "common-hal/analogio/AnalogIn.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"


const STATIC adc_pin_set_t __ADC0_pin_set[] =
{
    /* Pin set 0, Channel 0A */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 23),
        .func = 0U,
    },

    /* Pin set 1, CHannel 1A */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 10),
        .func = 0U,
    },

    /* Pin set 2, Channel 2A */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 15),
        .func = 0U,
    },

    /* Pin set 3, Channel 3A */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 31),
        .func = 0U,
    },

    /* Pin set 4, Channel 4A */
    {
        .pin = NXP_PORT_GPIO_PIN(1, 8),
        .func = 0U,
    },

    /* Pin set 5, Channel 0B */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 16),
        .func = 0U,
    },

    /* Pin set 6, Channel 1B */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 11),
        .func = 0U,
    },

    /* Pin set 7, Channel 2B */
    {
        .pin = NXP_PORT_GPIO_PIN(0, 12),
        .func = 0U,
    },

    /* Pin set 8, Channel 3B */
    {
        .pin = NXP_PORT_GPIO_PIN(1, 0),
        .func = 0U,
    },

    /* Pin set 9, Channel 4B */
    {
        .pin = NXP_PORT_GPIO_PIN(1, 9),
        .func = 0U,
    },
};


STATIC adc_inst_t __adc_instances[ADC_INSTANCES_NUM] = {
    /*  Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .adc_dev = (adc_dev_t *)ADC0,
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

        const uint32_t pin_config = (/* Pin is configured as ADC0_x */
            func |
            /* No addition pin function */
            IOCON_PIO_MODE_INACT |
            /* Standard mode, output slew rate control is enabled */
            IOCON_PIO_SLEW_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables analog function */
            IOCON_PIO_ANALOG_EN |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* Analog switch is closed (enabled) */
            IOCON_PIO_ASW_EN);

        /* Configure as ADC0_x */
        IOCON_PinMuxSet(IOCON, port, pin_number, pin_config);

        /* ... Clocks are enabled by adc_init function */
    }

    return;
}

void adc_disable(adc_inst_t *adc_instance, size_t channel) {
    if ((0U == adc_instance->id) && (MP_ARRAY_SIZE(__ADC0_pin_set) > channel)) {
        const uint8_t pin = __ADC0_pin_set[channel].pin;
        const uint8_t port = NXP_PORT_GPIO_PIN_PORT(pin);
        const uint8_t pin_number = NXP_PORT_GPIO_PIN_NUMBER(pin);

        const uint32_t pin_config = (/* Pin is configured as ADC0_x */
            IOCON_PIO_FUNC0 |
            /* No addition pin function */
            IOCON_PIO_MODE_INACT |
            /* Standard mode, output slew rate control is enabled */
            IOCON_PIO_SLEW_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI);

        /* ... Configure as GPIO */
        IOCON_PinMuxSet(IOCON, port, pin_number, pin_config);

        /* ... Clocks are disabled by adc_deinit function */
    }

    return;
}
