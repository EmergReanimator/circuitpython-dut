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


#include <math.h>
#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "shared/runtime/interrupt_char.h"

#include "common-hal/canio/__init__.h"
#include "common-hal/canio/Listener.h"
#include "shared-bindings/canio/Listener.h"
#include "shared-bindings/util.h"
#include "supervisor/shared/tick.h"
#include "supervisor/shared/safe_mode.h"


typedef canio_message_obj_t *canio_message_ptr_t;

void common_hal_canio_listener_construct(canio_listener_obj_t *self, canio_can_obj_t *can, size_t nmatch, canio_match_obj_t **matches, float timeout) {
    assert((NULL != can));
    self->can = can;

    can_inst_t *can_instance = can->can_instance;
    assert((NULL != can_instance));

    ARM_DRIVER_CAN *can_drv = can_instance->driver;
    assert((NULL != can_drv));

    ARM_CAN_OBJ_CAPABILITIES capabilities = can_drv->ObjectGetCapabilities(can_instance->rx_id);

    if (nmatch > 1U && !(capabilities.multiple_filters)) {
        mp_raise_ValueError(translate("Filters too complex"));
    }

    const size_t num_of_msg = 8U;
    canio_message_ptr_t *pool = m_new(canio_message_ptr_t, num_of_msg);
    canio_message_ptr_t *queue = m_new(canio_message_ptr_t, num_of_msg);

    ring_buffer_t *pool_rb = &can_instance->rx_msg_pool;
    memset(pool, (int)NULL, sizeof(canio_message_ptr_t *) * num_of_msg);
    rb_init(pool_rb, pool, sizeof(canio_message_ptr_t *) * num_of_msg);

    ring_buffer_t *queue_rb = &can_instance->rx_msg_queue;
    memset(queue, (int)NULL, sizeof(canio_message_ptr_t *) * num_of_msg);
    rb_init(queue_rb, queue, sizeof(canio_message_ptr_t *) * num_of_msg);

    for (size_t n = 0; n < num_of_msg; ++n) {
        canio_message_obj_t *msg = m_new_ll_obj(canio_message_obj_t);
        rb_push(pool_rb, &msg, sizeof(msg));
    }

    canio_match_obj_t *match = matches[0U];
    if (ARM_CAN_STANDARD_ID(-1) == match->mask) {
        uint32_t id = match->id;
        id = (match->extended) ? ARM_CAN_EXTENDED_ID(id) : ARM_CAN_STANDARD_ID(id);
        int32_t rc = can_drv->ObjectSetFilter(can_instance->rx_id, ARM_CAN_FILTER_ID_EXACT_ADD, id, 0U);

        #if (0)
        can_drv->ObjectSetFilter(obj_idx, ARM_CAN_FILTER_ID_RANGE_ADD, id_low, id_high);
        #endif
    } else {
        mp_raise_ValueError(translate("Filters too complex"));
    }

    common_hal_canio_listener_set_timeout(self, timeout);

    return;
}

void common_hal_canio_listener_set_timeout(canio_listener_obj_t *self, float timeout) {
    self->timeout_ms = (uint32_t)MICROPY_FLOAT_C_FUN(ceil)((timeout * 1000.0f));
    return;
}

float common_hal_canio_listener_get_timeout(canio_listener_obj_t *self) {
    return ((float)self->timeout_ms) / 1000.0f;
}

void common_hal_canio_listener_check_for_deinit(canio_listener_obj_t *self) {
    if (!self->can) {
        raise_deinited_error();
    }
    common_hal_canio_can_check_for_deinit(self->can);
    return;
}

int common_hal_canio_listener_in_waiting(canio_listener_obj_t *self) {
    canio_can_obj_t *can = self->can;
    assert((NULL != can));

    ring_buffer_t *queue = &can->can_instance->rx_msg_queue;

    return !rb_is_empty(queue);
}

mp_obj_t common_hal_canio_listener_receive(canio_listener_obj_t *self) {

    bool is_waiting = !common_hal_canio_listener_in_waiting(self);
    if (is_waiting) {
        const uint64_t deadline = supervisor_ticks_ms64() + (uint64_t)self->timeout_ms;

        while (is_waiting) {
            uint64_t timestamp = supervisor_ticks_ms64();
            if (timestamp <= deadline) {
                RUN_BACKGROUND_TASKS;

                /* Allow user to break out of a timeout with a KeyboardInterrupt. */
                bool is_interrupted = mp_hal_is_interrupted();
                bool is_ready = common_hal_canio_listener_in_waiting(self);

                if (is_interrupted || is_ready) {
                    is_waiting = false;
                }
            } else {
                is_waiting = false;
            }
        }
    }

    canio_can_obj_t *can = self->can;
    can_inst_t *can_instance = can->can_instance;
    ring_buffer_t *pool = &can_instance->rx_msg_pool;
    ring_buffer_t *queue = &can_instance->rx_msg_queue;
    canio_message_obj_t *msg = (canio_message_obj_t *)NULL;
    if (!rb_is_empty(queue) && !rb_is_full(pool)) {
        ARM_DRIVER_CAN *can_drv = can_instance->driver;

        canio_message_obj_t *msg_container = (canio_message_obj_t *)NULL;
        rb_pop(queue, &msg_container, sizeof(msg));
        msg = m_new_obj(canio_message_obj_t);
        msg->base.type = msg_container->base.type;
        memcpy(msg->data, msg_container->data, 8U);
        msg->extended = msg_container->extended;
        msg->id = msg_container->id;
        msg->size = msg_container->size;
        rb_push(pool, msg_container, sizeof(msg));
    }

    return msg;
}

void common_hal_canio_listener_deinit(canio_listener_obj_t *self) {
    /* TODO: Implement common_hal_canio_listener_deinit */

    return;
}
