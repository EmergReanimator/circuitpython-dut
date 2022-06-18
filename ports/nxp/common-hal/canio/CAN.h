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

#if !defined(PORTS_NXP_COMMON_HAL_CANIO_CAN_H_)
#define PORTS_NXP_COMMON_HAL_CANIO_CAN_H_

#include "py/obj.h"
#include "py/ringbuf.h"

#include "shared-bindings/canio/__init__.h"
#include "shared-bindings/canio/CAN.h"

#include "common-hal/microcontroller/Pin.h"
#include "common-hal/canio/__init__.h"

#include "shared-module/canio/Message.h"

#include "cmsis5/CMSIS/Driver/Include/Driver_CAN.h"

typedef volatile ringbuf_t ring_buffer_t;

#define rb_alloc(rb,len) \
    ringbuf_alloc((ringbuf_t *)rb,len,true)

#define rb_init(rb,buf,len) \
    ringbuf_init((ringbuf_t *)rb,(uint8_t *)buf,len)

#define rb_push(rb,buf,len) \
    ringbuf_put_n((ringbuf_t *)rb,(uint8_t *)buf,len)

#define rb_pop(rb,buf,len) \
    ringbuf_get_n((ringbuf_t *)rb,(uint8_t *)buf,len)

#define rb_is_empty(rb) \
    (ringbuf_num_empty((ringbuf_t *)rb) == ringbuf_capacity((ringbuf_t *)rb))

#define rb_is_full(rb) \
    (ringbuf_num_filled((ringbuf_t *)rb) == ringbuf_capacity((ringbuf_t *)rb))


typedef struct can_pin_set {
    uint8_t tx;
    uint8_t rx;
} can_pin_set_t;

typedef struct can_inst {
    const size_t id;
    bool is_used;
    const ARM_DRIVER_CAN *driver;
    const can_pin_set_t *pin_map;
    const size_t pin_map_len;

    uint32_t tx_id;
    uint32_t rx_id;

    volatile bool tx_busy   : 1;
    volatile bool rx_lost   : 1;

    ring_buffer_t rx_msg_pool;
    ring_buffer_t rx_msg_queue;
} can_inst_t;

typedef struct canio_can_obj {
    mp_obj_base_t base;
    can_inst_t *can_instance;
    canio_can_state_t *state;

    uint32_t bitrate;
    bool loopback           : 1;
    bool silent             : 1;
    bool auto_recovery      : 1;

    const mcu_pin_obj_t *rx;
    const mcu_pin_obj_t *tx;

} canio_can_obj_t;

#endif // PORTS_NXP_COMMON_HAL_CANIO_CAN_H_
