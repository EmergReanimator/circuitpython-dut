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


#include "can_config.h"

#include "py/runtime.h"
#include "py/mperrno.h"

#include "shared-bindings/util.h"   /* raise_deinited_error */
#include "shared-bindings/canio/CAN.h"
#include "shared-bindings/microcontroller/Pin.h"

#include "common-hal/canio/CAN.h"
#include "common-hal/microcontroller/Pin.h"

#include "supervisor/shared/tick.h"


#if defined(NDEBUG)
#define __ASSERT(expr) assert(expr)
#else
#define __ASSERT(expr) (void)(expr)
#endif

// CAN standard frame format bits (without datafield)
// SOF BASEID SRR           RTR r1     r0         DLC DATA CRC STUF-BITS CRCDEL ACK EOF IFS
#define CAN_STD_FRAME_BITS            (1 + 11 + 1 + 1 + 1 + 1 + 4 + 15 + 24 + 1 + 2 + 7 + 3)

// CAN extended frame format bits (without datafield)
// SOF BASEID SRR IDE IDEXT RTR r1     r0         DLC DATA CRC STUF-BITS CRCDEL ACK EOF
#define CAN_EXT_FRAME_BITS            (1 + 11 + 1 + 1 + 18 + 1 + 1 + 1 + 4 + 15 + 29 + 1 + 2 + 7)

// CAN FD extended frame format bits sent at NOMINAL bitrate
// SOF BASEID SRR IDE IDEXT     r1 EDL r0 BRS ESI DLC DATA CRC CRCDEL ACK EOF
#define CAN_EXT_FRAME_BITS_NOMINAL    (1 + 11 + 1 + 1 + 18 + 1 + 1 + 1 + 1 + 2 + 7)

// CAN FD extended frame format bits sent at FD_DATA bitrate (without datafield)
// SOF BASEID SRR IDE IDEXT     r1 EDL r0 BRS ESI DLC DATA CRC STUF-BITS CRCDEL ACK EOF
#define CAN_EXT_FRAME_BITS_FD_DATA    (+1 + 4 + 15 + 29 + 1)


#if (0)
#include "genhdr/candata.h"
#endif

#if !defined(FALL_THROUGH)
#if (defined(__GNUC__) && (__GNUC__ >= 7)) || (defined(__clang__) && (__clang_major__ >= 12))
#define FALL_THROUGH __attribute__ ((fallthrough))    /* fall through */
#else
#define FALL_THROUGH ((void)0)    /* fall through */
#endif /* __GNUC__ >= 7 */
#endif /* !defined(FALL_THROUGH) */



#define CRITICAL_SECTION_ENTER()    {}
#define CRITICAL_SECTION_LEAVE()    {}

typedef void (*object_cb_func)(uint32_t obj_idx, uint32_t event);

STATIC void __CAN_object_cb(size_t n, uint32_t obj_idx, uint32_t event) {
    can_inst_t *can_instance = get_can_instance(n);

    if ((ARM_CAN_EVENT_SEND_COMPLETE == event) && (obj_idx == can_instance->rx_id)) {
        can_instance->tx_busy = false;
    } else if ((ARM_CAN_EVENT_RECEIVE == event) && (obj_idx == can_instance->rx_id)) {
        ring_buffer_t *pool = &can_instance->rx_msg_pool;
        ring_buffer_t *queue = &can_instance->rx_msg_queue;

        bool is_pool_empty = rb_is_empty(pool);
        bool is_queue_full = rb_is_full(queue);
        if (pool && !is_pool_empty && !is_queue_full) {
            canio_message_obj_t *msg = (canio_message_obj_t *)NULL;
            rb_pop(pool, &msg, sizeof(msg));
            ARM_DRIVER_CAN *can_drv = can_instance->driver;
            ARM_CAN_MSG_INFO msg_info;

            int32_t rc = can_drv->MessageRead(obj_idx, &msg_info, &msg->data[0U], MP_ARRAY_SIZE(msg->data));
            if (rc > 0) {
                msg->extended = (bool)(msg_info.id & ARM_CAN_ID_IDE_Msk);
                msg->base.type = msg_info.rtr ? &canio_remote_transmission_request_type : &canio_message_type;
                msg->id = msg_info.id & ~ARM_CAN_ID_IDE_Msk;
                msg->size = (size_t)msg_info.dlc;
                rb_push(queue, &msg, sizeof(msg));
            }
        }
    } else if ((ARM_CAN_EVENT_RECEIVE_OVERRUN == event) && (obj_idx == can_instance->rx_id)) {
        can_instance->rx_lost = true;
    }

    return;
}

#define CALLBACK(n) \
    STATIC void __CAN_object_cb##n(uint32_t obj_idx, uint32_t event) \
    { \
        __CAN_object_cb(n, obj_idx, event); \
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
    __CAN_object_cb##n,

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


#if (CAN_INSTANCES_NUM == 1U)
MAKE_CALLBACKS(1);
#elif (CAN_INSTANCES_NUM == 2U)
MAKE_CALLBACKS(2);
#elif (CAN_INSTANCES_NUM == 3U)
MAKE_CALLBACKS(3);
#elif (CAN_INSTANCES_NUM == 4U)
MAKE_CALLBACKS(4);
#elif (CAN_INSTANCES_NUM == 5U)
MAKE_CALLBACKS(5);
#endif

const STATIC object_cb_func __object_cb[CAN_INSTANCES_NUM] =
{
    #if (CAN_INSTANCES_NUM == 1U)
    INIT_CALLBACKS(1)
    #elif (CAN_INSTANCES_NUM == 2U)
    INIT_CALLBACKS(2)
    #elif (CAN_INSTANCES_NUM == 3U)
    INIT_CALLBACKS(3)
    #elif (CAN_INSTANCES_NUM == 4U)
    INIT_CALLBACKS(4)
    #elif (CAN_INSTANCES_NUM == 5U)
    INIT_CALLBACKS(4)
    #endif
};

STATIC bool __match_can_instance(const can_pin_set_t *pin_set, const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx) {
    bool is_matched = true;

    uint8_t tx_pin = NXP_PORT_GPIO_PIN(tx->port, tx->number);
    uint8_t rx_pin = NXP_PORT_GPIO_PIN(rx->port, rx->number);

    is_matched = is_matched && (pin_set->tx == tx_pin);
    is_matched = is_matched && (pin_set->rx == rx_pin);

    return is_matched;
}

STATIC bool __validate_pins(const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx) {
    bool is_free = true;

    is_free = is_free && pin_number_is_free(tx->port, tx->number);
    is_free = is_free && pin_number_is_free(rx->port, rx->number);

    return is_free;
}

STATIC size_t __lookup_matching_free_can_instance(const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx) {
    bool valid_pin_set = false;

    size_t n;
    for (n = 0U; n < CAN_INSTANCES_NUM; ++n) {
        /* ... loop over all CAN pin set for given CAN instance */
        can_inst_t *instance = get_can_instance(n);
        const can_pin_set_t *pin_set = instance->pin_map;
        const size_t M = instance->pin_map_len;
        for (size_t m = 0U; m < M; ++m) {
            valid_pin_set = __match_can_instance(&pin_set[m], tx, rx);
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

STATIC int __can_set_mode(ARM_DRIVER_CAN *can_drv, bool loopback, bool silent) {
    int drv_err = ARM_DRIVER_ERROR;

    if (loopback) {
        if (silent) {
            drv_err = can_drv->SetMode(ARM_CAN_MODE_LOOPBACK_INTERNAL);
        } else {
            drv_err = can_drv->SetMode(ARM_CAN_MODE_LOOPBACK_EXTERNAL);
        }
    } else {
        drv_err = can_drv->SetMode(ARM_CAN_MODE_NORMAL);
    }

    return drv_err;
}

STATIC int __can_init(canio_can_obj_t *self, object_cb_func cb, uint32_t baudrate, bool loopback, bool silent) {
    int32_t rc = ARM_DRIVER_OK;

    can_inst_t *can_instance = self->can_instance;
    ARM_DRIVER_CAN *can_drv = can_instance->driver;

    /* TODO: Implement unit callback to track CAN bus state changes. */
    rc = can_drv->Initialize(NULL, cb);
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = can_drv->PowerControl(ARM_POWER_FULL);
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = can_drv->SetMode(ARM_CAN_MODE_INITIALIZATION);
    __ASSERT((ARM_DRIVER_OK == rc));

    uint32_t bitrate = (uint32_t)(baudrate);
    rc = can_drv->SetBitrate(ARM_CAN_BITRATE_NOMINAL, bitrate,
        ARM_CAN_BIT_PROP_SEG(4)
        | ARM_CAN_BIT_PHASE_SEG1(10U)
        | ARM_CAN_BIT_PHASE_SEG2(5U)
        | ARM_CAN_BIT_SJW(3U));
    if (ARM_DRIVER_OK != rc) {
        can_drv->PowerControl(ARM_POWER_OFF);
        mp_raise_OSError(MP_EINVAL); // bitrate cannot be attained (16kHz or something is lower bound, should never happen)
    }

    ARM_CAN_CAPABILITIES can_dev_info = can_drv->GetCapabilities();
    const size_t num_of_channels = can_dev_info.num_objects;

    ARM_CAN_OBJ_CAPABILITIES chan_info;
    can_instance->tx_id = UINT32_MAX;
    can_instance->rx_id = UINT32_MAX;

    for (size_t n = 0U; n < num_of_channels; ++n) {
        chan_info = can_drv->ObjectGetCapabilities(n);

        if ((UINT32_MAX == can_instance->tx_id) && (chan_info.tx)) {
            can_instance->tx_id = n;
        }

        if ((UINT32_MAX == can_instance->rx_id) && (chan_info.rx)) {
            can_instance->rx_id = n;
        }
    }

    __ASSERT((UINT32_MAX != can_instance->tx_id));
    __ASSERT((UINT32_MAX != can_instance->rx_id));

    rc = can_drv->ObjectConfigure(can_instance->tx_id, ARM_CAN_OBJ_TX);
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = can_drv->ObjectConfigure(can_instance->rx_id, ARM_CAN_OBJ_RX);
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = __can_set_mode(can_drv, loopback, silent);
    __ASSERT((ARM_DRIVER_OK == rc));

    return rc;
}

void common_hal_canio_reset(void) {
    #if (1)
    #if (1)
    // TODO: Implement common_hal_canio_reset
    #else
    mp_raise_RuntimeError(translate("common_hal_canio_reset functionality is missing"));
    #endif

    #else
    canio_can_obj_t *self = NULL;
    __ASSERT((NULL != self));

    can_inst_t *can_instnace = self->can_instance;
    __ASSERT((NULL != can_instnace));

    ARM_DRIVER_CAN *can_drv = can_instnace->driver;
    int rc = can_drv->SetMode(ARM_CAN_MODE_INITIALIZATION);
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = __can_set_mode(can_drv, self->loopback, self->silent);
    __ASSERT((ARM_DRIVER_OK == rc));
    #endif

    return;
}

void common_hal_canio_can_construct(canio_can_obj_t *self,
    const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx,
    int baudrate, bool loopback, bool silent) {

    if (NULL == self->can_instance) {
        self->can_instance = (can_inst_t *)NULL;
        self->tx = (const mcu_pin_obj_t *)NULL;
        self->rx = (const mcu_pin_obj_t *)NULL;

        bool valid_pin_set = __validate_pins(tx, rx);
        size_t instance_idx = __lookup_matching_free_can_instance(tx, rx);
        can_inst_t *can_instance = get_can_instance(instance_idx);

        if (valid_pin_set && (SIZE_MAX > instance_idx) && !(can_instance->is_used)) {
            common_hal_mcu_pin_claim(tx);
            common_hal_reset_pin(tx);
            self->tx = tx;

            common_hal_mcu_pin_claim(rx);
            common_hal_reset_pin(rx);
            self->rx = rx;

            can_instance->is_used = true;
            ARM_DRIVER_CAN *can_drv = can_instance->driver;

            self->can_instance = can_instance;

            can_enable(can_instance);
            __can_init(self, __object_cb[instance_idx], baudrate, loopback, silent);

            /* RX buffer will be allocated in listener */
            rb_init(&self->can_instance->rx_msg_pool, NULL, 0U);
            rb_init(&self->can_instance->rx_msg_queue, NULL, 0U);

            self->bitrate = (uint32_t)baudrate;
            self->can_instance->tx_busy = false;
            self->can_instance->rx_lost = false;
            self->loopback = loopback;
            self->silent = silent;
            self->auto_recovery = false;
        } else {
            mp_raise_ValueError(translate("Invalid pins"));
        }
    } else {
        mp_raise_ValueError(translate("CAN peripheral in use"));
    }

    return;
}

void common_hal_canio_can_deinit(canio_can_obj_t *self) {
    if (!common_hal_canio_can_deinited(self)) {
        ARM_DRIVER_CAN *can_drv = self->can_instance->driver;
        (void)can_drv->PowerControl(ARM_POWER_OFF);
        (void)can_drv->Uninitialize();

        self->can_instance->is_used = false;
        self->can_instance = (can_inst_t *)NULL;

        common_hal_reset_pin(self->tx);
        common_hal_reset_pin(self->rx);
    }

    return;
}

bool common_hal_canio_can_deinited(canio_can_obj_t *self) {
    return NULL == self->can_instance;
}

void common_hal_canio_can_check_for_deinit(canio_can_obj_t *self) {
    if (common_hal_canio_can_deinited(self)) {
        raise_deinited_error();
    }
    return;
}

int common_hal_canio_can_baudrate_get(canio_can_obj_t *self) {
    return (int)self->bitrate;
}

bool common_hal_canio_can_silent_get(canio_can_obj_t *self) {
    return self->silent;
}

bool common_hal_canio_can_loopback_get(canio_can_obj_t *self) {
    return self->loopback;
}

void common_hal_canio_can_restart(canio_can_obj_t *self) {
    ARM_DRIVER_CAN *can_drv = self->can_instance->driver;
    __ASSERT((NULL != can_drv));

    int32_t rc = can_drv->PowerControl(ARM_POWER_OFF);
    (void)rc;
    __ASSERT((ARM_DRIVER_OK == rc));

    rc = can_drv->PowerControl(ARM_POWER_FULL);
    __ASSERT((ARM_DRIVER_OK == rc));

    return;
}

bool common_hal_canio_can_auto_restart_get(canio_can_obj_t *self) {
    return self->auto_recovery;
}

void common_hal_canio_can_auto_restart_set(canio_can_obj_t *self, bool value) {
    self->auto_recovery = value;
    return;
}

int common_hal_canio_can_transmit_error_count_get(canio_can_obj_t *self) {
    ARM_DRIVER_CAN *can_drv = self->can_instance->driver;
    __ASSERT((NULL != can_drv));

    ARM_CAN_STATUS status = can_drv->GetStatus();
    int tx_err_cnt = (int)(status.tx_error_count);

    return tx_err_cnt;
}

int common_hal_canio_can_receive_error_count_get(canio_can_obj_t *self) {
    ARM_DRIVER_CAN *can_drv = self->can_instance->driver;
    __ASSERT((NULL != can_drv));

    ARM_CAN_STATUS status = can_drv->GetStatus();
    int rx_err_cnt = (int)(status.rx_error_count);

    return rx_err_cnt;
}

canio_bus_state_t common_hal_canio_can_state_get(canio_can_obj_t *self) {
    ARM_DRIVER_CAN *can_drv = self->can_instance->driver;
    __ASSERT((NULL != can_drv));

    ARM_CAN_STATUS status = can_drv->GetStatus();

    canio_bus_state_t canio_bus_state = BUS_STATE_OFF;
    uint32_t unit_state = status.unit_state;
    switch (unit_state) {
        case (ARM_CAN_UNIT_STATE_ACTIVE): { canio_bus_state = BUS_STATE_ERROR_ACTIVE;
                                            break;
        }

        case (ARM_CAN_UNIT_STATE_INACTIVE):
            FALL_THROUGH;
        /* no break */
        case (ARM_CAN_UNIT_STATE_PASSIVE): { canio_bus_state = BUS_STATE_ERROR_PASSIVE;
                                             break;
        }

        case (ARM_CAN_UNIT_STATE_BUS_OFF):
            FALL_THROUGH;
        /* no break */
        default: { canio_bus_state = BUS_STATE_OFF;
                   break;
        }
    }

    return canio_bus_state;
}

void common_hal_canio_can_send(canio_can_obj_t *self, mp_obj_t message_in) {
    can_inst_t *can_instance = self->can_instance;
    __ASSERT((NULL != can_instance));

    ARM_DRIVER_CAN *can_drv = can_instance->driver;
    __ASSERT((NULL != can_drv));

    canio_message_obj_t *msg = (canio_message_obj_t *)message_in;
    __ASSERT((NULL != msg));

    #if (0)
    bool rtr = message->base.type == &canio_remote_transmission_request_type;
    #endif

    size_t dlc = (msg->size < 8U) ? msg->size : 8U;

    ARM_CAN_MSG_INFO msg_info;
    msg_info.id = (msg->extended) ? ARM_CAN_EXTENDED_ID(msg->id) : ARM_CAN_STANDARD_ID(msg->id);
    msg_info.rtr = 0U;
    msg_info.edl = 0U;
    msg_info.brs = 0U;
    msg_info.esi = 0U;
    msg_info.dlc = dlc;
    msg_info.reserved = 0U;

    can_instance->tx_busy = true;
    int32_t rc = can_drv->MessageSend(self->can_instance->tx_id, &msg_info, &msg->data[0U], dlc);
    __ASSERT((dlc == rc));

    /* T ms = Frame bits + 8 bit/byte * DLC bytes * 1000 ms/s */
    uint64_t timeout_ms = 8ULL * ((uint64_t)dlc);
    timeout_ms += (msg->extended) ? ((uint64_t)CAN_EXT_FRAME_BITS) : ((uint64_t)CAN_STD_FRAME_BITS);
    timeout_ms += 1000ULL;
    timeout_ms /= ((uint64_t)self->bitrate);

    uint64_t deadline = supervisor_ticks_ms64() + timeout_ms;
    bool is_waiting = true;
    while (is_waiting) {
        uint64_t timestamp = supervisor_ticks_ms64();

        if (timestamp <= deadline) {
            RUN_BACKGROUND_TASKS;

            #if (1)
            /* ... No need to interrupt the busy loop due to negligible timeout values. */
            const bool is_interrupted = false;
            #else
            /* Allow user to break out of a timeout with a KeyboardInterrupt. */
            bool is_interrupted = mp_hal_is_interrupted();
            #endif
            bool is_ready = !self->can_instance->tx_busy;

            if (is_interrupted || is_ready) {
                is_waiting = false;
            }
        } else {
            is_waiting = false;
        }
    }

    return;
}
