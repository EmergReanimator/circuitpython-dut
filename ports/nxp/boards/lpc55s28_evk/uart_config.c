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

#include "boards/lpc55s28_evk/CMSIS/Driver/Config/RTE_Device.h"
#include "boards/lpc55s28_evk/uart_config.h"
#include "common-hal/busio/UART.h"
#include "fsl_clock.h"

#include "shared-bindings/microcontroller/Pin.h" /* COMMON_HAL_MCU_NO_PIN */
#include "device.h"


extern ARM_DRIVER_USART Driver_USART0;

const static uart_pin_set_t UART0_pin_set[] =
{
    /* Pin set 0 */
    {
        .tx = NXP_PORT_GPIO_PIN(RTE_USART0_TX_PORT, RTE_USART0_TX_BIT),
        .rx = NXP_PORT_GPIO_PIN(RTE_USART0_RX_PORT, RTE_USART0_RX_BIT),
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
};

uart_inst_t *get_uart_instance(const size_t n) {
    if (n < UART_INSTANCES_NUM) {
        return &uart_instances[n];
    } else {
        return (uart_inst_t *)NULL;
    }
}

void uart_enable(uart_inst_t *uart_instance) {
    if (0U == uart_instance->id) {
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
    }

    return;
}

void uart_disable(uart_inst_t *uart_instance) {
    if (0U == uart_instance->id) {
        CLOCK_AttachClk(kNONE_to_FLEXCOMM0);
    }

    return;
}

uint32_t USART0_GetFreq(void) {
    return CLOCK_GetFlexCommClkFreq(0U);
}


#include "boards/lpc55s28_evk/pin_mux.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_common.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_gpio.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_inputmux.h"
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : USART0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void USART0_InitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
        IOCON_PIO_FUNC1 |
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
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
        IOCON_PIO_FUNC1 |
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
    /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : USART0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void USART0_DeinitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin29_config = (/* Pin is configured as PIO0_29 */
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
    /* PORT0 PIN29 (coords: 92) is configured as PIO0_29 */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    const uint32_t port0_pin30_config = (/* Pin is configured as PIO0_30 */
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
    /* PORT0 PIN30 (coords: 94) is configured as PIO0_30 */
    IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);
}
