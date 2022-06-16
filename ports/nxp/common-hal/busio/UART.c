/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 microDev
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

#include "shared-bindings/busio/UART.h"

#include "uart_config.h"

#include "py/stream.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "supervisor/shared/tick.h"
#include "shared/runtime/interrupt_char.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "common-hal/microcontroller/Pin.h"
#include <string.h> /* memset */


typedef void (*cb_func)(uint32_t event);

STATIC void __copy_into_ringbuf(uart_ringbuf_t *r, ARM_DRIVER_USART *uart_drv, const uint8_t *buf, size_t buflen) {
    if (ringbuf_num_empty((ringbuf_t *)r) > 0) {
        const size_t rx_cnt = uart_drv->GetRxCount();
        (void)ringbuf_put_n((ringbuf_t *)r, (uint8_t *)buf, rx_cnt);
    }

    return;
}

STATIC void __UART_cb(size_t n, uint32_t event) {
    if (n < UART_INSTANCES_NUM) {
        uart_inst_t *uart_instance = get_uart_instance(n);

        if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
            uint8_t *buf = uart_instance->buf;
            const size_t buflen = MP_ARRAY_SIZE(uart_instance->buf);
            __copy_into_ringbuf(&uart_instance->ringbuf, uart_instance->driver, buf, buflen);
            (void)uart_instance->driver->Receive(buf, buflen);
        }
    }

    return;
}

#define CALLBACK(n) \
    STATIC void __UART_cb##n(uint32_t event) \
    { \
        __UART_cb(n, event); \
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
    __UART_cb##n,

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


#if (UART_INSTANCES_NUM == 1U)
MAKE_CALLBACKS(1);
#elif (UART_INSTANCES_NUM == 2U)
MAKE_CALLBACKS(2);
#elif (UART_INSTANCES_NUM == 3U)
MAKE_CALLBACKS(3);
#elif (UART_INSTANCES_NUM == 4U)
MAKE_CALLBACKS(4);
#elif (UART_INSTANCES_NUM == 5U)
MAKE_CALLBACKS(5);
#endif

const STATIC cb_func __cb[UART_INSTANCES_NUM] =
{
    #if (UART_INSTANCES_NUM == 1U)
    INIT_CALLBACKS(1)
    #elif (UART_INSTANCES_NUM == 2U)
    INIT_CALLBACKS(2)
    #elif (UART_INSTANCES_NUM == 3U)
    INIT_CALLBACKS(3)
    #elif (UART_INSTANCES_NUM == 4U)
    INIT_CALLBACKS(4)
    #elif (UART_INSTANCES_NUM == 5U)
    INIT_CALLBACKS(4)
    #endif
};

STATIC bool __match_uart_instance(
    const uart_pin_set_t *pin_set,
    const mcu_pin_obj_t *tx,
    const mcu_pin_obj_t *rx,
    const mcu_pin_obj_t *rts,
    const mcu_pin_obj_t *cts) {
    bool is_matched = true;

    uint8_t tx_pin = (tx) ? NXP_PORT_GPIO_PIN(tx->port, tx->number) : COMMON_HAL_MCU_NO_PIN;
    uint8_t rx_pin = (rx) ? NXP_PORT_GPIO_PIN(rx->port, rx->number) : COMMON_HAL_MCU_NO_PIN;
    uint8_t rts_pin = (rts) ? NXP_PORT_GPIO_PIN(rts->port, rts->number) : COMMON_HAL_MCU_NO_PIN;
    uint8_t cts_pin = (cts) ? NXP_PORT_GPIO_PIN(cts->port, cts->number) : COMMON_HAL_MCU_NO_PIN;

    is_matched = is_matched && (pin_set->tx == tx_pin);
    is_matched = is_matched && ((rx) ? (pin_set->rx == rx_pin) : true);
    is_matched = is_matched && ((rts) ? (pin_set->rts == rts_pin) : true);
    is_matched = is_matched && ((cts) ? (pin_set->cts == cts_pin) : true);

    return is_matched;
}

STATIC bool __validate_pins(
    const mcu_pin_obj_t *tx,
    const mcu_pin_obj_t *rx,
    const mcu_pin_obj_t *rts,
    const mcu_pin_obj_t *cts) {
    bool is_free = true;

    is_free = is_free && ((tx) ? pin_number_is_free(tx->port, tx->number) : true);
    is_free = is_free && ((rx) ? pin_number_is_free(rx->port, rx->number) : true);
    is_free = is_free && ((rts) ? pin_number_is_free(rts->port, rts->number) : true);
    is_free = is_free && ((cts) ? pin_number_is_free(cts->port, cts->number) : true);

    return is_free;
}

STATIC size_t __lookup_matching_free_uart_instance(
    const mcu_pin_obj_t *tx_pin,
    const mcu_pin_obj_t *rx_pin,
    const mcu_pin_obj_t *cts_pin,
    const mcu_pin_obj_t *rts_pin) {
    bool valid_pin_set = false;

    size_t n;
    for (n = 0U; n < UART_INSTANCES_NUM; ++n) {
        /* ... loop over all UART pin set for given UART instance */
        uart_inst_t *instance = get_uart_instance(n);
        const uart_pin_set_t *pin_set = instance->pin_map;
        const size_t M = instance->pin_map_len;
        for (size_t m = 0U; m < M; ++m) {
            valid_pin_set = __match_uart_instance(pin_set, tx_pin, rx_pin, rts_pin, cts_pin);
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

STATIC void __uart_init(
    busio_uart_obj_t *self,
    ARM_DRIVER_USART *uart_drv,
    cb_func cb,
    uint32_t baudrate,
    bool rx_enable) {
    int32_t drv_err = uart_drv->Initialize(cb);
    if (ARM_DRIVER_OK == drv_err) {
        uart_drv->PowerControl(ARM_POWER_FULL);
        uint32_t control = ARM_USART_MODE_ASYNCHRONOUS
            | ARM_USART_DATA_BITS_8
            | ARM_USART_PARITY_NONE
            | ARM_USART_STOP_BITS_1
            | ARM_USART_FLOW_CONTROL_NONE;

        drv_err = uart_drv->Control(control, baudrate);

        (void)uart_drv->Control(ARM_USART_CONTROL_TX, 1);

        if (rx_enable) {
            uart_drv->Control(ARM_USART_CONTROL_RX, 1);
        }
    } else {
        uart_drv->Uninitialize();
        mp_raise_ValueError(translate("UART Init Error"));
    }

    return;
}

void reset_uart(void) {
    // TODO: Implement reset_uart
    return;
}

void never_reset_uart(uint8_t num) {
    return;
}

void common_hal_busio_uart_construct(busio_uart_obj_t *self,
    const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx,
    const mcu_pin_obj_t *rts, const mcu_pin_obj_t *cts,
    const mcu_pin_obj_t *rs485_dir, bool rs485_invert,
    uint32_t baudrate, uint8_t bits, busio_uart_parity_t parity, uint8_t stop,
    mp_float_t timeout, uint16_t receiver_buffer_size, byte *receiver_buffer,
    bool sigint_enabled) {

    if (NULL == self->uart_instance) {
        if ((rts != NULL) || (cts != NULL) || (rs485_dir != NULL) || (rs485_invert == true)) {
            mp_raise_NotImplementedError(translate("RTS/CTS/RS485 Not yet supported on this device"));
        }

        self->uart_instance = (uart_inst_t *)NULL;
        self->rx = (const mcu_pin_obj_t *)NULL;
        self->tx = (const mcu_pin_obj_t *)NULL;
        self->cts = (const mcu_pin_obj_t *)NULL;
        self->rts = (const mcu_pin_obj_t *)NULL;

        bool valid_pin_set = __validate_pins(tx, rx, rts, cts);
        size_t instance_idx = __lookup_matching_free_uart_instance(tx, rx, rts, cts);
        uart_inst_t *uart_instance = get_uart_instance(instance_idx);

        if (valid_pin_set && (SIZE_MAX > instance_idx) && !(uart_instance->is_used)) {
            self->tx = tx;
            common_hal_mcu_pin_claim(tx);
            common_hal_reset_pin(tx);

            bool have_rx = (NULL != rx);
            uart_instance->is_used = true;
            memset(&uart_instance->buf[0U], 0U, MP_ARRAY_SIZE(uart_instance->buf));

            if (have_rx) {
                self->rx = rx;
                common_hal_mcu_pin_claim(rx);
                common_hal_reset_pin(rx);
            }

            if (NULL != rts) {
                self->rts = rts;
                common_hal_mcu_pin_claim(rts);
                common_hal_reset_pin(rts);
            }

            if (NULL != cts) {
                self->cts = cts;
                common_hal_mcu_pin_claim(cts);
                common_hal_reset_pin(cts);
            }

            ARM_DRIVER_USART *uart_drv = uart_instance->driver;

            self->uart_instance = uart_instance;

            /* Pins must be reset before UART has been initialised */
            uart_enable(uart_instance);
            __uart_init(self, uart_drv, __cb[instance_idx], baudrate, have_rx);

            self->timeout_ms = timeout * 1000;

            if (have_rx) {
                if (0 == receiver_buffer_size) {
                    mp_raise_ValueError(translate("Invalid buffer size"));
                }

                // Initially allocate the UART's buffer in the long-lived part of the
                // heap. UARTs are generally long-lived objects, but the "make long-
                // lived" machinery is incapable of moving internal pointers like
                // self->buffer, so do it manually.  (However, as long as internal
                // pointers like this are NOT moved, allocating the buffer
                // in the long-lived pool is not strictly necessary)
                // (This is a macro.)

                bool is_initialised = false;
                if (NULL != receiver_buffer) {
                    is_initialised = ringbuf_init((ringbuf_t *)&uart_instance->ringbuf, receiver_buffer, receiver_buffer_size);
                } else {
                    is_initialised = ringbuf_alloc((ringbuf_t *)&uart_instance->ringbuf, receiver_buffer_size, true);
                }

                if (!is_initialised) {
                    mp_raise_msg(&mp_type_MemoryError, translate("UART Buffer allocation error"));
                }

                (void)uart_drv->Receive(&uart_instance->buf[0U], MP_ARRAY_SIZE(uart_instance->buf));
            }
        } else {
            mp_raise_ValueError(translate("Invalid pins"));
        }
    } else {
        mp_raise_ValueError(translate("UART peripheral in use"));
    }

    return;
}

bool common_hal_busio_uart_deinited(busio_uart_obj_t *self) {
    return NULL == self->uart_instance;
}

void common_hal_busio_uart_deinit(busio_uart_obj_t *self) {
    if (!common_hal_busio_uart_deinited(self)) {

        uart_inst_t *uart_instance = self->uart_instance;
        (void)uart_instance->driver->Control(ARM_USART_ABORT_TRANSFER, 0U);
        (void)uart_instance->driver->PowerControl(ARM_POWER_OFF);
        (void)uart_instance->driver->Uninitialize();
        uart_instance->is_used = false;

        self->uart_instance = (uart_inst_t *)NULL;

        if (self->tx) {
            common_hal_reset_pin(self->tx);
            self->tx = NULL;
        }

        if (self->rx) {
            common_hal_reset_pin(self->rx);
            self->rx = NULL;
        }

        if (self->rts) {
            common_hal_reset_pin(self->rts);
            self->rts = NULL;
        }

        if (self->cts) {
            common_hal_reset_pin(self->cts);
            self->cts = NULL;
        }

        ringbuf_free((ringbuf_t *)&uart_instance->ringbuf);
    }

    return;
}

size_t common_hal_busio_uart_write(busio_uart_obj_t *self, const uint8_t *data, size_t len, int *errcode) {
    if (!self->tx) {
        mp_raise_ValueError(translate("No TX pin"));
    }

    ARM_DRIVER_USART *uart_drv = self->uart_instance->driver;

    int32_t drv_err = uart_drv->Send(data, len);

    if (ARM_DRIVER_OK == drv_err) {
        ARM_USART_STATUS status;
        do {
            status = uart_drv->GetStatus();
            RUN_BACKGROUND_TASKS;
        } while (status.tx_busy);
    } else {
        len = 0U;
    }

    return len;
}

size_t common_hal_busio_uart_read(busio_uart_obj_t *self, uint8_t *data, size_t len, int *errcode) {
    if (!self->rx) {
        mp_raise_ValueError(translate("No RX pin"));
    }

    if (len == 0) {
        // Nothing to read.
        return 0;
    }

    uart_ringbuf_t *ringbuf = &self->uart_instance->ringbuf;

    // Copy as much received data as available, up to len bytes.
    size_t total_read = ringbuf_get_n((ringbuf_t *)ringbuf, &data[0U], len);

    // Check if we still need to read more data.
    if (len > total_read) {
        len -= total_read;
        uint64_t start_ticks = supervisor_ticks_ms64();
        // Busy-wait until timeout or until we've read enough chars.
        while (len > 0 && (supervisor_ticks_ms64() - start_ticks < self->timeout_ms)) {
            size_t num_filled = ringbuf_num_filled((ringbuf_t *)ringbuf);
            if (num_filled) {
                size_t num_read = ringbuf_get_n((ringbuf_t *)ringbuf, &data[total_read], len);

                // Adjust the counters.
                len -= num_read;
                total_read += num_read;

                // Reset the timeout on every character read.
                start_ticks = supervisor_ticks_ms64();
            }
            RUN_BACKGROUND_TASKS;
            // Allow user to break out of a timeout with a KeyboardInterrupt.
            if (mp_hal_is_interrupted()) {
                break;
            }
        }
    }

    if (0U == total_read) {
        *errcode = EAGAIN;
        return MP_STREAM_ERROR;
    }

    return total_read;
}

uint32_t common_hal_busio_uart_get_baudrate(busio_uart_obj_t *self) {
    return self->baudrate;
}

void common_hal_busio_uart_set_baudrate(busio_uart_obj_t *self, uint32_t baudrate) {
    self->baudrate = baudrate;
    return;
}

mp_float_t common_hal_busio_uart_get_timeout(busio_uart_obj_t *self) {
    return ((mp_float_t)self->timeout_ms) / 1000.0f;
}

void common_hal_busio_uart_set_timeout(busio_uart_obj_t *self, mp_float_t timeout) {
    self->timeout_ms = (uint32_t)(timeout * 1000.0f);
    return;
}

uint32_t common_hal_busio_uart_rx_characters_available(busio_uart_obj_t *self) {
    return ringbuf_num_filled((ringbuf_t *)&self->uart_instance->ringbuf);
}

void common_hal_busio_uart_clear_rx_buffer(busio_uart_obj_t *self) {
    uart_inst_t *uart_instance = self->uart_instance;
    (void)uart_instance->driver->Control(ARM_USART_ABORT_RECEIVE, 0U);
    memset(&uart_instance->buf[0U], 0U, MP_ARRAY_SIZE(uart_instance->buf));
    ringbuf_clear((ringbuf_t *)&self->uart_instance->ringbuf);
    return;
}

bool common_hal_busio_uart_ready_to_tx(busio_uart_obj_t *self) {
    if (!self->tx) {
        return false;
    }
    return true;
}

void common_hal_busio_uart_never_reset(busio_uart_obj_t *self) {
    if (self->tx) {
        common_hal_never_reset_pin(self->tx);
    }
    if (self->rx) {
        common_hal_never_reset_pin(self->rx);
    }
    if (self->rts) {
        common_hal_never_reset_pin(self->rts);
    }
    if (self->cts) {
        common_hal_never_reset_pin(self->cts);
    }

    return;
}
