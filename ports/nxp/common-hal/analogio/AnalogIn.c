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

#include "adc_config.h"

#include "common-hal/analogio/AnalogIn.h"
#include "shared-bindings/analogio/AnalogIn.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "py/runtime.h"
#include "supervisor/shared/translate/translate.h"

#include <math.h>

STATIC uint32_t __channel_status = 0U;


STATIC bool __match_adc_instance(const adc_pin_set_t *pin_set, const mcu_pin_obj_t *pin) {
    bool is_matched = true;

    uint8_t adc_pin = NXP_PORT_GPIO_PIN(pin->port, pin->number);

    is_matched = is_matched && (pin_set->pin == adc_pin);

    return is_matched;
}


STATIC bool __validate_pins(const mcu_pin_obj_t *adc_in) {
    bool is_free = true;

    is_free = is_free && pin_number_is_free(adc_in->port, adc_in->number);

    return is_free;
}


STATIC size_t __lookup_matching_free_adc_instance(size_t *channel, const mcu_pin_obj_t *pin) {
    bool valid_pin_set = false;

    size_t n;
    for (n = 0U; n < ADC_INSTANCES_NUM; ++n) {
        /* ... loop over all CAN pin set for given CAN instance */
        adc_inst_t *instance = get_adc_instance(n);
        const adc_pin_set_t *pin_set = instance->pin_map;
        const size_t M = instance->pin_map_len;
        for (size_t m = 0U; m < M; ++m) {
            valid_pin_set = __match_adc_instance(&pin_set[m], pin);
            if (valid_pin_set) {
                *channel = m;
                break;
            }
        }
        if (valid_pin_set && !(instance->is_used)) {
            break;
        }
    }

    return valid_pin_set ? n : SIZE_MAX;
}


void common_hal_analogio_analogin_construct(analogio_analogin_obj_t *self, const mcu_pin_obj_t *pin) {

    if (NULL == self->adc_instance) {
        self->adc_instance = (adc_inst_t *)NULL;
        self->pin = (const mcu_pin_obj_t *)NULL;

        bool valid_pin_set = __validate_pins(pin);
        size_t adc_channel = SIZE_MAX;
        size_t instance_idx = __lookup_matching_free_adc_instance(&adc_channel, pin);
        adc_inst_t *adc_instance = get_adc_instance(instance_idx);

        adc_dev_t *adc_dev = adc_instance->adc_dev;

        if (valid_pin_set && (SIZE_MAX > instance_idx) && !(adc_instance->is_used)) {
            self->adc_channel = adc_channel;
            adc_enable(adc_instance, adc_channel);

            int rc = 0;
            if (0 == __channel_status) {
                rc = adc_init(adc_dev);
            }

            if (!rc) {
                adc_caibrate(adc_dev);
                common_hal_mcu_pin_claim(pin);

                __channel_status |= 1U << adc_channel;
                self->adc_instance = adc_instance;
                self->pin = pin;
            } else {
                mp_raise_ValueError(translate("ADC Init Error"));
            }
        } else {
            mp_raise_ValueError(translate("Pin does not have ADC capabilities"));
        }
    } else {
        mp_raise_ValueError(translate("ADC peripheral in use"));
    }

    return;
}


void common_hal_analogio_analogin_deinit(analogio_analogin_obj_t *self) {
    if (!common_hal_analogio_analogin_deinited(self)) {
        adc_inst_t *adc_instance = self->adc_instance;
        adc_dev_t *adc_dev = adc_instance->adc_dev;

        uint32_t adc_channel = self->adc_channel;
        adc_disable(adc_instance, adc_channel);
        __channel_status &= (1U << adc_channel);

        if (0U == __channel_status) {
            adc_deinit(adc_dev);
        }

        common_hal_reset_pin(self->pin);
        self->adc_channel = SIZE_MAX;
        self->adc_instance = (adc_inst_t *)NULL;
        self->pin = NULL;
        self->timeout = 0U;
    }

    return;
}


bool common_hal_analogio_analogin_deinited(analogio_analogin_obj_t *self) {
    return NULL == self->adc_instance;
}


uint16_t common_hal_analogio_analogin_get_value(analogio_analogin_obj_t *self) {

    adc_inst_t *adc_instance = self->adc_instance;
    adc_dev_t *adc_dev = adc_instance->adc_dev;

    uint32_t channel = self->adc_channel;
    adc_enable_channel(adc_dev, channel);

    const uint32_t timeout = self->timeout;
    adc_value_t value = 0U;
    adc_read(adc_dev, &value, channel, timeout);

    adc_disable_channel(adc_dev, channel);

    return (uint16_t)value;
}


float common_hal_analogio_analogin_get_reference_voltage(analogio_analogin_obj_t *self) {
    float ref_voltage = NAN;

    uint32_t adc_voltage = 0U;
    int rc = adc_ref_voltage(self->adc_instance->adc_dev, &adc_voltage);
    if (!rc) {
        ref_voltage = (float)adc_voltage;
    }

    return ref_voltage;
}
