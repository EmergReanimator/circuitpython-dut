/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Scott Shawcroft for Adafruit Industries
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

#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"

#include "common-hal/microcontroller/Pin.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "supervisor/shared/translate/translate.h"

#include "peripherals/pins.h"

digitalinout_result_t
common_hal_digitalio_digitalinout_construct(digitalio_digitalinout_obj_t *self, const mcu_pin_obj_t *pin) {
    claim_pin(pin->port, pin->number);
    self->pin = pin;
    self->output = false;
    self->open_drain = false;

    // Set to input. No output value.
    pin_config_t pin_config;
    pin_config.input = true;
    pin_config.pinMode = GPIO_Mode_PullNone;
    pin_config.outputLogic = true;

    gpio_pin_init(pin->port, pin->number, &pin_config);
    return DIGITALINOUT_OK;
}

void
common_hal_digitalio_digitalinout_never_reset(digitalio_digitalinout_obj_t *self) {
    never_reset_pin_number(self->pin->port, self->pin->number);
}

bool
common_hal_digitalio_digitalinout_deinited(digitalio_digitalinout_obj_t *self) {
    return NULL == self->pin;
}

void
common_hal_digitalio_digitalinout_deinit(digitalio_digitalinout_obj_t *self) {
    if (!common_hal_digitalio_digitalinout_deinited(self)) {
        reset_pin_number(self->pin->port, self->pin->number);
        self->pin = NULL;
    }

    return;
}

digitalinout_result_t
common_hal_digitalio_digitalinout_switch_to_input(digitalio_digitalinout_obj_t *self, digitalio_pull_t pull) {
    self->output = false;
    // This also sets direction to input.
    common_hal_digitalio_digitalinout_set_pull(self, pull);
    (void)gpio_pin_dir(self->pin->port, self->pin->number, true);

    return DIGITALINOUT_OK;
}

digitalinout_result_t
common_hal_digitalio_digitalinout_switch_to_output(digitalio_digitalinout_obj_t *self, bool value, digitalio_drive_mode_t drive_mode) {
    pin_config_t pin_config;
    pin_config.input = false;
    pin_config.outputLogic = true;
    pin_config.pinMode = GPIO_Mode_PullNone;
    gpio_pin_init(self->pin->port, self->pin->number, &pin_config);
    gpio_pin_dir(self->pin->port, self->pin->number, false);

    self->output = true;
    self->open_drain = (DRIVE_MODE_OPEN_DRAIN == drive_mode);

    // Pin direction is ultimately set in set_value. We don't need to do it here.
    common_hal_digitalio_digitalinout_set_value(self, value);

    return DIGITALINOUT_OK;
}

digitalio_direction_t
common_hal_digitalio_digitalinout_get_direction(digitalio_digitalinout_obj_t *self) {
    return self->output ? DIRECTION_OUTPUT : DIRECTION_INPUT;
}

void
common_hal_digitalio_digitalinout_set_value(digitalio_digitalinout_obj_t *self, bool value) {
    (void)gpio_pin_write(self->pin->port, self->pin->number, value);

    return;
}

bool
common_hal_digitalio_digitalinout_get_value(digitalio_digitalinout_obj_t *self) {
    return gpio_pin_read(self->pin->port, self->pin->number);
}

digitalinout_result_t
common_hal_digitalio_digitalinout_set_drive_mode(digitalio_digitalinout_obj_t *self, digitalio_drive_mode_t drive_mode) {
    #if (1)
    return DIGITALINOUT_PIN_BUSY;
    #else
    const uint8_t pin = self->pin->number;
    bool value = common_hal_digitalio_digitalinout_get_value(self);
    self->open_drain = drive_mode == DRIVE_MODE_OPEN_DRAIN;
    // True is implemented differently between modes so reset the value to make
    // sure it's correct for the new mode.
    if (value) {
        common_hal_digitalio_digitalinout_set_value(self, value);
    }
    return DIGITALINOUT_OK;
    #endif
}

digitalio_drive_mode_t
common_hal_digitalio_digitalinout_get_drive_mode(digitalio_digitalinout_obj_t *self) {
    if (self->open_drain) {
        return DRIVE_MODE_OPEN_DRAIN;
    } else {
        return DRIVE_MODE_PUSH_PULL;
    }
}

digitalinout_result_t
common_hal_digitalio_digitalinout_set_pull(digitalio_digitalinout_obj_t *self, digitalio_pull_t pull) {
    pin_config_t pin_config;
    pin_config.input = (false == self->output);
    pin_config.outputLogic = true;

    if (PULL_NONE == pull) {
        pin_config.pinMode = GPIO_Mode_PullNone;
    } else if (PULL_DOWN == pull) {
        pin_config.pinMode = GPIO_Mode_PullDown;
    } else if (PULL_UP == pull) {
        pin_config.pinMode = GPIO_Mode_PullUp;
    }

    (void)gpio_pin_init(self->pin->port, self->pin->number, &pin_config);

    return DIGITALINOUT_OK;
}

digitalio_pull_t
common_hal_digitalio_digitalinout_get_pull(digitalio_digitalinout_obj_t *self) {
    #if (1)
    return PULL_NONE;
    #else
    uint32_t pin = self->pin->number;
    if (self->output) {
        mp_raise_AttributeError(translate("Cannot get pull while in output mode"));
        return PULL_NONE;
    } else {
        if (gpio_is_pulled_up(pin)) {
            return PULL_UP;
        } else if (gpio_is_pulled_down(pin)) {
            return PULL_DOWN;
        }
    }
    return PULL_NONE;
    #endif
}
