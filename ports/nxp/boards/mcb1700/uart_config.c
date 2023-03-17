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


#include "shared-bindings/microcontroller/Pin.h" /* COMMON_HAL_MCU_NO_PIN */

#include "common-hal/busio/UART.h"
#include "boards/mcb1700/uart_config.h"
#include "boards/mcb1700/CMSIS/Driver/Config/RTE_Device.h"

#include "device.h"


extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;

const static uart_pin_set_t UART0_pin_set[] =
{
    /* Pin set 0 */
    {
        .tx = NXP_PORT_GPIO_PIN(RTE_UART0_TX_PORT, RTE_UART0_TX_BIT),
        .rx = NXP_PORT_GPIO_PIN(RTE_UART0_RX_PORT, RTE_UART0_RX_BIT),
        .cts = COMMON_HAL_MCU_NO_PIN,
        .rts = COMMON_HAL_MCU_NO_PIN,
    },
};

const static uart_pin_set_t UART1_pin_set[] =
{
    /* Pin set 0 */
    {
        .tx = NXP_PORT_GPIO_PIN(RTE_UART1_TX_PORT, RTE_UART1_TX_BIT),
        .rx = NXP_PORT_GPIO_PIN(RTE_UART1_RX_PORT, RTE_UART1_RX_BIT),
        .cts = COMMON_HAL_MCU_NO_PIN,
        .rts = COMMON_HAL_MCU_NO_PIN,
    },
};

static uart_inst_t uart_instances[UART_INSTANCES_NUM] = {
    /* UART Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .driver = &Driver_USART0,
        .pin_map = &UART0_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(UART0_pin_set),
    },

    /* UART Instance 1 */
    {
        .id = 1U,
        .is_used = false,
        .driver = &Driver_USART1,
        .pin_map = &UART1_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(UART1_pin_set),
    },
};

uart_inst_t *get_uart_instance(const size_t n) {
    if (n < UART_INSTANCES_NUM) {
        return &uart_instances[n];
    } else {
        return (uart_inst_t *)NULL;
    }
}

#define PCUART0  (1U << 3)
#define PCUART1  (1U << 4)

void uart_enable(uart_inst_t *uart_instance) {
    if (0U == uart_instance->id) {
        LPC_SC->PCONP |= PCUART0;
    } else if (1U == uart_instance->id) {
        LPC_SC->PCONP |= PCUART1;
    }

    return;
}

void uart_disable(uart_inst_t *uart_instance) {
    if (0U == uart_instance->id) {
        LPC_SC->PCONP &= ~PCUART0;
    } else if (1U == uart_instance->id) {
        LPC_SC->PCONP &= ~PCUART1;
    }

    return;
}
