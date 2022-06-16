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


// FIXME: Remove dependency from the board header file
#include "boards/lpc55s28_evk/pin_mux.h"

#include "py/obj.h"
#include "py/mphal.h"
#include "peripherals/pins.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_common.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_gpio_cmsis.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_gpio.h"
#include "RTE_Device.h"


#if defined(CPU_LPC55S28JBD100)
// ... GPIO0
PIN(0,0);   //
PIN(0,1);   //
PIN(0,2);   //
PIN(0,3);   //
PIN(0,4);   //
PIN(0,5);   //
PIN(0,6);   //
PIN(0,7);   //
PIN(0,8);   //
PIN(0,9);   //
PIN(0,10);  //
PIN(0,11);  //
PIN(0,12);  //
PIN(0,13);  //
PIN(0,14);  //
PIN(0,15);  //
PIN(0,16);  //
PIN(0,17);  //
PIN(0,18);  //
PIN(0,19);  //
PIN(0,20);  //
PIN(0,21);  //
PIN(0,22);  //
PIN(0,23);  //
PIN(0,24);  //
PIN(0,25);  //
PIN(0,26);  //
PIN(0,27);  //
PIN(0,28);  //
PIN(0,29);  //
PIN(0,30);  //
PIN(0,31);  //

// ... GPIO1
PIN(1,0);   //
PIN(1,1);   //
PIN(1,2);   //
PIN(1,3);   //
PIN(1,4);   //
PIN(1,5);   //
PIN(1,6);   //
PIN(1,7);   //
PIN(1,8);   //
PIN(1,9);   //
PIN(1,10);  //
PIN(1,11);  //
PIN(1,12);  //
PIN(1,13);  //
PIN(1,14);  //
PIN(1,15);  //
PIN(1,16);  //
PIN(1,17);  //
PIN(1,18);  //
PIN(1,19);  //
PIN(1,20);  //
PIN(1,21);  //
PIN(1,22);  //
PIN(1,23);  //
PIN(1,24);  //
PIN(1,25);  //
PIN(1,26);  //
PIN(1,27);  //
PIN(1,28);  //
PIN(1,29);  //
PIN(1,30);  //
PIN(1,31);  //

#else
#error "CPU_LPC55S28JBD100 must be set"

#endif // CPU_LPC55S28JBD100


#if defined(RTE_GPIO_PORT0) && RTE_GPIO_PORT0
extern ARM_DRIVER_GPIO Driver_GPIO_PORT0;
#else
#define Driver_GPIO_PORT0 *(ARM_DRIVER_GPIO *)(NULL)
#endif

#if defined(RTE_GPIO_PORT1) && RTE_GPIO_PORT1
extern ARM_DRIVER_GPIO Driver_GPIO_PORT1;
#else
#define Driver_GPIO_PORT1 *(ARM_DRIVER_GPIO *)(NULL)
#endif


gpio_port_obj_t gpio_ports[2U] =
{
    // P0
    {
        .available_pin_mask = 0xFFFFFFFFU,
        // P0_10 - SWO
        // P0_11 - SWCLK
        // P0_12 - SWDIO
        // P0_12 - USB_VBUS
        .reserved_pin_mask = ((1U << 22) | (1U << 12) | (1U << 11) | (1U << 10)),
        .used_pin_mask = 0x00000000U,
    },

    // P1
    {
        .available_pin_mask = 0x000000D0U,
        .reserved_pin_mask = 0xFFFFFF2FU,
        .used_pin_mask = 0x00000000U,
    },
};


int gpio_pin_init(uint8_t port, uint8_t number, pin_config_t *config) {
    const size_t port_cnt = sizeof(gpio_ports) / sizeof(gpio_ports[0U]);
    ARM_DRIVER_GPIO *gpio_port = NULL;

    int rc = -1;

    if (port < port_cnt) {
        if (0U == port) {
            gpio_port = &Driver_GPIO_PORT0;
        } else if (1U == port) {
            gpio_port = &Driver_GPIO_PORT1;
        }

        assert((NULL != gpio_port));

        gpio_port->Initialize();

        const uint32_t pin_config = (/* Pin is configured as GPIO */
            IOCON_PIO_FUNC_GPIO |
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

        gpio_pin_dir(port, number, config->input);
        IOCON_PinMuxSet(IOCON, port, number, pin_config);

        rc = 0;
    }

    return rc;
}


int gpio_pin_dir(uint8_t port, uint8_t number, bool input) {
    ARM_DRIVER_GPIO *gpio_port = NULL;
    if (0U == port) {
        gpio_port = &Driver_GPIO_PORT0;
    } else if (1U == port) {
        gpio_port = &Driver_GPIO_PORT1;
    }

    if (input) {
        gpio_port->InitPinAsInput(number, ARM_GPIO_INTERRUPT_NONE, NULL);
    } else {
        gpio_port->InitPinAsOutput(number, 1U);
    }

    return 0;
}


int gpio_pin_write(uint8_t port, uint8_t number, bool value) {
    if (0U == port) {
        Driver_GPIO_PORT0.PinWrite(number, value);
    } else if (1U == port) {
        Driver_GPIO_PORT1.PinWrite(number, value);
    }

    return 0;
}


bool gpio_pin_read(uint8_t port, uint8_t number) {
    bool value = false;

    if (0U == port) {
        value = (bool)Driver_GPIO_PORT0.PinRead(number);
    } else if (1U == port) {
        value = (bool)Driver_GPIO_PORT1.PinRead(number);
    }

    return value;
}
