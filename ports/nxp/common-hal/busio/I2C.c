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

#include "shared-bindings/busio/I2C.h"

#include "i2c_config.h"

#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/runtime.h"

#include "supervisor/board.h"
#include "common-hal/microcontroller/Pin.h"
#include "shared-bindings/microcontroller/Pin.h"


#define CRITICAL_SECTION_ENTER()    {}
#define CRITICAL_SECTION_LEAVE()    {}

STATIC uint32_t __i2c_events[I2C_INSTANCES_NUM];

typedef void (*cb_func)(uint32_t event);

STATIC void __I2C_cb(size_t n, uint32_t event) {
    if (n < sizeof(__i2c_events) / sizeof(__i2c_events[0U])) {
        __i2c_events[n] = event;
    }

    return;
}

#define CALLBACK(n) \
    STATIC void __I2C_cb##n(uint32_t event) \
    { \
        __I2C_cb(n, event); \
    }

#define CALLBACK1 \
    CALLBACK(0)

#define CALLBACK2 \
    CALLBACK1 \
    CALLBACK(1)

#define CALLBACK3 \
    CALLBACK2 \
    CALLBACK(2) \

#define CALLBACK4 \
    CALLBACK3 \
    CALLBACK(3)

#define CALLBACK5 \
    CALLBACK3 \
    CALLBAC(4)


#define MAKE_CALLBACKS(n) \
    CALLBACK##n


#define INIT_CALLBACK(n) \
    __I2C_cb##n,

#define INIT_CALLBACK1 \
    INIT_CALLBACK(0)

#define INIT_CALLBACK2 \
    INIT_CALLBACK1 \
    INIT_CALLBACK(1)

#define INIT_CALLBACK3 \
    INIT_CALLBACK2 \
    INIT_CALLBACK(2)

#define INIT_CALLBACK4 \
    INIT_CALLBACK3 \
    INIT_CALLBACK(3)

#define INIT_CALLBACK5 \
    INIT_CALLBACK4 \
    INIT_CALLBACK(4)

#define INIT_CALLBACKS(n) \
    INIT_CALLBACK##n


#if (I2C_INSTANCES_NUM == 1U)
MAKE_CALLBACKS(1);
#elif (I2C_INSTANCES_NUM == 2U)
MAKE_CALLBACKS(2);
#elif (I2C_INSTANCES_NUM == 3U)
MAKE_CALLBACKS(3);
#elif (I2C_INSTANCES_NUM == 4U)
MAKE_CALLBACKS(4);
#elif (I2C_INSTANCES_NUM == 5U)
MAKE_CALLBACKS(5);
#endif

const STATIC cb_func __cb[I2C_INSTANCES_NUM] =
{
    #if (I2C_INSTANCES_NUM == 1U)
    INIT_CALLBACKS(1)
    #elif (I2C_INSTANCES_NUM == 2U)
    INIT_CALLBACKS(2)
    #elif (I2C_INSTANCES_NUM == 3U)
    INIT_CALLBACKS(3)
    #elif (I2C_INSTANCES_NUM == 4U)
    INIT_CALLBACKS(4)
    #elif (I2C_INSTANCES_NUM == 5U)
    INIT_CALLBACKS(4)
    #endif
};


STATIC bool __match_i2c_instance(const i2c_pin_set_t *pin_set, const mcu_pin_obj_t *scl, const mcu_pin_obj_t *sda) {
    bool is_matched = true;

    uint8_t scl_pin = NXP_PORT_GPIO_PIN(scl->port, scl->number);
    uint8_t sda_pin = NXP_PORT_GPIO_PIN(sda->port, sda->number);

    is_matched = is_matched && (pin_set->scl == scl_pin);
    is_matched = is_matched && (pin_set->sda == sda_pin);

    return is_matched;
}


STATIC bool __validate_pins(const mcu_pin_obj_t *scl, const mcu_pin_obj_t *sda) {
    bool is_free = true;

    is_free = is_free && pin_number_is_free(scl->port, scl->number);
    is_free = is_free && pin_number_is_free(sda->port, sda->number);

    return is_free;
}


STATIC size_t __lookup_matching_free_i2c_instance(const mcu_pin_obj_t *scl, const mcu_pin_obj_t *sda) {
    bool valid_pin_set = false;

    size_t n;
    for (n = 0U; n < I2C_INSTANCES_NUM; ++n) {
        /* ... loop over all I2C pin set for given I2C instance */
        i2c_inst_t *instance = get_i2c_instance(n);
        const i2c_pin_set_t *pin_set = instance->pin_map;
        const size_t M = instance->pin_map_len;
        for (size_t m = 0U; m < M; ++m) {
            valid_pin_set = __match_i2c_instance(&pin_set[m], scl, sda);
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


STATIC int __i2c_init(busio_i2c_obj_t *self, ARM_DRIVER_I2C *i2c_drv, cb_func cb, const uint32_t frequency) {
    int rc = 0;

    int32_t status = i2c_drv->Initialize(cb);

    if (ARM_DRIVER_OK == status) {
        i2c_drv->PowerControl(ARM_POWER_FULL);
        if (100000u == frequency) {
            i2c_drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
        } else if (400000u == frequency) {
            i2c_drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
        } else if (1000000u == frequency) {
            i2c_drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
        } else if (3400000u == frequency) {
            i2c_drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_HIGH);
        } else {
            i2c_drv->Uninitialize();
            rc = -MP_EINVAL;
        }
    } else {
        rc = -MP_ENXIO;
    }

    return rc;
}


STATIC void __i2c_deinit(busio_i2c_obj_t *self) {
    // TODO: Implement __i2c_deinit
    return;
}

void port_reset_i2c(void) {
    // TODO: Implement port_reset_i2c
    #if (0)
    busio_i2c_obj_t *self;
    common_hal_reset_pin(self->scl);
    common_hal_reset_pin(self->sda);
    #endif
    return;
}


void common_hal_busio_i2c_construct(busio_i2c_obj_t *self,
    const mcu_pin_obj_t *scl, const mcu_pin_obj_t *sda, uint32_t frequency, uint32_t timeout) {

    if (NULL == self->i2c_instance) {
        self->i2c_instance = (i2c_inst_t *)NULL;
        self->scl = (const mcu_pin_obj_t *)NULL;
        self->sda = (const mcu_pin_obj_t *)NULL;
        self->frequency = 0;
        self->has_lock = false;

        bool valid_pin_set = __validate_pins(scl, sda);
        size_t instance_idx = __lookup_matching_free_i2c_instance(scl, sda);
        i2c_inst_t *i2c_instance = get_i2c_instance(instance_idx);

        if (valid_pin_set && (SIZE_MAX > instance_idx)) {
            common_hal_mcu_pin_claim(scl);
            self->scl = scl;

            common_hal_mcu_pin_claim(sda);
            self->sda = sda;

            i2c_instance->is_used = true;
            ARM_DRIVER_I2C *i2c_drv = i2c_instance->driver;

            self->i2c_instance = i2c_instance;

            i2c_enable(i2c_instance);

            #if CIRCUITPY_REQUIRE_I2C_PULLUPS
            #error "CIRCUITPY_REQUIRE_I2C_PULLUPS option is not yet supported"
            #else
            #if (0)
            mp_raise_RuntimeError(translate("No pull up found on SDA or SCL; check your wiring"));
            #endif
            #endif

            int rc = __i2c_init(self, i2c_drv, __cb[instance_idx], frequency);
            if (rc < 0) {
                /* ... Roll-back the changes */
                i2c_disable(i2c_instance);

                common_hal_reset_pin(scl);
                self->scl = (const mcu_pin_obj_t *)NULL;

                common_hal_reset_pin(sda);
                self->sda = (const mcu_pin_obj_t *)NULL;

                i2c_instance->is_used = false;
                self->i2c_instance = (i2c_inst_t *)NULL;

                if (-MP_EINVAL == rc) {
                    mp_raise_ValueError(translate("Unsupported baudrate"));
                } else {
                    mp_raise_RuntimeError(translate("I2C Init Error"));
                }
            }
        } else {
            mp_raise_ValueError(translate("Invalid pins"));
        }
    } else {
        mp_raise_ValueError(translate("I2C peripheral in use"));
    }

    return;
}


bool common_hal_busio_i2c_deinited(busio_i2c_obj_t *self) {
    return NULL == self->i2c_instance;
}


void common_hal_busio_i2c_deinit(busio_i2c_obj_t *self) {
    /* TODO: Implement common_hal_busio_i2c_deinit */
    return;
}


bool common_hal_busio_i2c_probe(busio_i2c_obj_t *self, uint8_t addr) {
    uint8_t dummy = 0u;
    return common_hal_busio_i2c_write(self, addr, &dummy, 1u) == 0;
}


bool common_hal_busio_i2c_try_lock(busio_i2c_obj_t *self) {
    /* TODO: Introduce common_hal_busio_try_lock function */
    bool grabbed_lock = false;
    CRITICAL_SECTION_ENTER();
    if (!self->has_lock) {
        grabbed_lock = true;
        self->has_lock = true;
    }
    CRITICAL_SECTION_LEAVE();
    return grabbed_lock;
}


bool common_hal_busio_i2c_has_lock(busio_i2c_obj_t *self) {
    return self->has_lock;
}


void common_hal_busio_i2c_unlock(busio_i2c_obj_t *self) {
    self->has_lock = false;
}


STATIC uint8_t __common_hal_busio_i2c_write(busio_i2c_obj_t *self, uint16_t addr,
    const uint8_t *data, size_t len, bool transmit_stop_bit) {

    i2c_inst_t *i2c_instance = self->i2c_instance;
    ARM_DRIVER_I2C *i2c_drv = i2c_instance->driver;
    __i2c_events[i2c_instance->id] = 0U;

    int32_t drv_err = i2c_drv->MasterTransmit(addr, data, len, transmit_stop_bit);

    uint8_t err = 0u;
    if (!drv_err) {
        ARM_I2C_STATUS status;
        do {
            RUN_BACKGROUND_TASKS;
            status = i2c_drv->GetStatus();
        } while (status.busy);

        if (__i2c_events[i2c_instance->id] & ARM_I2C_EVENT_ADDRESS_NACK) {
            err = MP_EIO;
        }
    } else {
        if (ARM_DRIVER_ERROR_PARAMETER == drv_err) {
            err = MP_ENODEV;
        } else if (ARM_DRIVER_ERROR == drv_err) {
            err = MP_ENXIO;
        } else if (ARM_DRIVER_ERROR_BUSY == drv_err) {
            err = MP_EBUSY;
        } else {
            err = MP_EIO;
        }
    }

    return err;
}


uint8_t common_hal_busio_i2c_write(busio_i2c_obj_t *self, uint16_t addr,
    const uint8_t *data, size_t len) {
    return __common_hal_busio_i2c_write(self, addr, data, len, true);
}


uint8_t common_hal_busio_i2c_read(busio_i2c_obj_t *self, uint16_t addr,
    uint8_t *data, size_t len) {
    bool transmit_stop_bit = false;
    i2c_inst_t *i2c_instance = self->i2c_instance;
    ARM_DRIVER_I2C *i2c_drv = i2c_instance->driver;
    __i2c_events[i2c_instance->id] = 0U;

    int32_t drv_err = i2c_drv->MasterReceive(addr, data, len, transmit_stop_bit);

    uint8_t err = 0u;
    if (!drv_err) {
        ARM_I2C_STATUS status;
        do {
            RUN_BACKGROUND_TASKS;
            status = i2c_drv->GetStatus();
        } while (status.busy);

        if (__i2c_events[i2c_instance->id] & ARM_I2C_EVENT_ADDRESS_NACK) {
            err = MP_EIO;
        }
    } else {
        if (ARM_DRIVER_ERROR_PARAMETER == drv_err) {
            err = MP_ENODEV;
        } else if (ARM_DRIVER_ERROR == drv_err) {
            err = MP_ENXIO;
        } else if (ARM_DRIVER_ERROR_BUSY == drv_err) {
            err = MP_EBUSY;
        } else if (ARM_DRIVER_OK == drv_err) {
            err = MP_EIO;
        }
    }

    return err;
}


uint8_t common_hal_busio_i2c_write_read(busio_i2c_obj_t *self, uint16_t addr,
    uint8_t *out_data, size_t out_len, uint8_t *in_data, size_t in_len) {
    uint8_t result = __common_hal_busio_i2c_write(self, addr, out_data, out_len, false);
    if (result != 0) {
        return result;
    }

    return common_hal_busio_i2c_read(self, addr, in_data, in_len);
}

void common_hal_busio_i2c_never_reset(busio_i2c_obj_t *self) {
    self->i2c_instance->is_used = true;
    common_hal_never_reset_pin(self->scl);
    common_hal_never_reset_pin(self->sda);

    return;
}
