/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Dan Halbert for Adafruit Industries
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

#include "dac_config.h"

#include "shared-bindings/analogio/AnalogOut.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"


STATIC bool __match_dac_instance(const dac_pin_set_t *pin_set, const mcu_pin_obj_t *pin) {
    bool is_matched = true;

    uint8_t dac_pin = NXP_PORT_GPIO_PIN(pin->port, pin->number);

    is_matched = is_matched && (pin_set->pin == dac_pin);

    return is_matched;
}


STATIC bool __validate_pins(const mcu_pin_obj_t *dac_in) {
    bool is_free = true;

    is_free = is_free && pin_number_is_free(dac_in->port, dac_in->number);

    return is_free;
}


STATIC size_t __lookup_matching_free_dac_instance(const mcu_pin_obj_t *pin) {
    bool valid_pin_set = false;

    size_t n;
    for (n = 0U; n < DAC_INSTANCES_NUM; ++n) {
        /* ... loop over all CAN pin set for given CAN instance */
        dac_inst_t *instance = get_dac_instance(n);
        const dac_pin_set_t *pin_set = instance->pin_map;
        const size_t M = instance->pin_map_len;
        for (size_t m = 0U; m < M; ++m) {
            valid_pin_set = __match_dac_instance(&pin_set[m], pin);
            if (valid_pin_set) {
                break;
            }
        }
        if (valid_pin_set && !(instance->is_used)) {
            break;
        }
    }

    return valid_pin_set ? n : SIZE_MAX;
}


void common_hal_analogio_analogout_construct(analogio_analogout_obj_t *self, const mcu_pin_obj_t *pin) {
    if (DAC_INSTANCES_NUM > 0U) {
        if (NULL == self->dac_instance) {
            self->dac_instance = (dac_inst_t *)NULL;
            self->pin = (const mcu_pin_obj_t *)NULL;

            bool valid_pin_set = __validate_pins(pin);
            size_t instance_idx = __lookup_matching_free_dac_instance(pin);
            dac_inst_t *dac_instance = get_dac_instance(instance_idx);

            dac_dev_t *dac_dev = dac_instance->dac_dev;

            if (valid_pin_set && (SIZE_MAX > instance_idx) && !(dac_instance->is_used)) {
                int rc = dac_init(dac_dev);
                if (!rc) {
                    self->dac_instance = dac_instance;
                    self->pin = pin;
                } else {
                    mp_raise_ValueError(translate("DAC Init Error"));
                }
            }
        } else {
            mp_raise_ValueError(translate("DAC peripheral in use"));
        }
    } else {
        mp_raise_RuntimeError(translate("AnalogOut functionality not supported"));
    }

    return;
}


bool common_hal_analogio_analogout_deinited(analogio_analogout_obj_t *self) {
    return NULL == self->dac_instance;
}


void common_hal_analogio_analogout_deinit(analogio_analogout_obj_t *self) {
    /* FIXME: Implement common_hal_analogio_analogout_deinit */
    return;
}


void common_hal_analogio_analogout_set_value(analogio_analogout_obj_t *self, uint16_t value) {

    dac_inst_t *dac_instance = self->dac_instance;
    dac_dev_t *dac_dev = dac_instance->dac_dev;

    dac_value_t dac_value = (dac_value_t)value;

    dac_write(dac_dev, &dac_value);

    return;
}
