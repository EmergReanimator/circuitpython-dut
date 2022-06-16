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

#include "common-hal/busio/I2C.h"
#include "boards/lpc55s28_evk/i2c_config.h"
#include "boards/lpc55s28_evk/CMSIS/Driver/Config/RTE_Device.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_clock.h"


extern ARM_DRIVER_I2C Driver_I2C1;
extern ARM_DRIVER_I2C Driver_I2C4;

const static i2c_pin_set_t I2C1_pin_set[] =
{
    /* Pin set 0 */
    {
        .scl = NXP_PORT_GPIO_PIN(RTE_I2C1_SCL_PORT, RTE_I2C1_SCL_PIN),
        .sda = NXP_PORT_GPIO_PIN(RTE_I2C1_SDA_PORT, RTE_I2C1_SDA_PIN),
    },
};

const static i2c_pin_set_t I2C4_pin_set[] =
{
    /* Pin set 0 */
    {
        .scl = NXP_PORT_GPIO_PIN(RTE_I2C4_SCL_PORT, RTE_I2C4_SCL_PIN),
        .sda = NXP_PORT_GPIO_PIN(RTE_I2C4_SDA_PORT, RTE_I2C4_SDA_PIN),
    },
};

static i2c_inst_t i2c_instances[I2C_INSTANCES_NUM] = {
    /* I2C Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .driver = &Driver_I2C1,
        .pin_map = &I2C1_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(I2C1_pin_set),
    },

    /* I2C Instance 1 */
    {
        .id = 0U,
        .is_used = false,
        .driver = &Driver_I2C4,
        .pin_map = &I2C4_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(I2C4_pin_set),
    },
};

i2c_inst_t *get_i2c_instance(const size_t n) {
    if (n < I2C_INSTANCES_NUM) {
        return &i2c_instances[n];
    } else {
        return (i2c_inst_t *)NULL;
    }
}

void i2c_enable(i2c_inst_t *i2c_instance) {
    if (1U == i2c_instance->id) {
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
    } else if (2U == i2c_instance->id) {
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    }

    return;
}

void i2c_disable(i2c_inst_t *i2c_instance) {
    if (1U == i2c_instance->id) {
        CLOCK_AttachClk(kNONE_to_FLEXCOMM1);
    } else if (2U == i2c_instance->id) {
        CLOCK_AttachClk(kNONE_to_FLEXCOMM4);
    }

    return;
}

uint32_t I2C1_GetFreq(void) {
    return CLOCK_GetFlexCommClkFreq(1U);
}

uint32_t I2C4_GetFreq(void) {
    return CLOCK_GetFlexCommClkFreq(4U);
}

#include "boards/lpc55s28_evk/pin_mux.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_common.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_gpio.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_inputmux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void I2C1_InitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin14_config = (/* Pin is configured as FC1_RTS_SCLX_SSEL1 */
        IOCON_PIO_FUNC1 |
        /* Selects pull-up function */
        IOCON_PIO_MODE_PULLUP |
        /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_SLEW_STANDARD |
        /* Input function is not inverted */
        IOCON_PIO_INV_DI |
        /* Enables digital function */
        IOCON_PIO_DIGITAL_EN |
        /* Open drain is enabled */
        IOCON_PIO_OPENDRAIN_EN);
    /* PORT0 PIN14 (coords: ?) is configured as FC4_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 14U, port1_pin14_config);

    const uint32_t port1_pin13_config = (/* Pin is configured as FC1_CTS_SDAX_SSEL0 */
        IOCON_PIO_FUNC1 |
        /* Selects pull-up function */
        IOCON_PIO_MODE_PULLUP |
        /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_SLEW_STANDARD |
        /* Input function is not inverted */
        IOCON_PIO_INV_DI |
        /* Enables digital function */
        IOCON_PIO_DIGITAL_EN |
        /* Open drain is enabled */
        IOCON_PIO_OPENDRAIN_EN);
    /* PORT0 PIN13 (coords: ?) is configured as FC4_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 13U, port1_pin13_config);
}

void I2C4_InitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin20_config = (/* Pin is configured as FC4_TXD_SCL_MISO_WS */
        IOCON_PIO_FUNC5 |
        /* Selects pull-up function */
        IOCON_PIO_MODE_PULLUP |
        /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_SLEW_STANDARD |
        /* Input function is not inverted */
        IOCON_PIO_INV_DI |
        /* Enables digital function */
        IOCON_PIO_DIGITAL_EN |
        /* Open drain is enabled */
        IOCON_PIO_OPENDRAIN_EN);
    /* PORT1 PIN20 (coords: 4) is configured as FC4_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

    const uint32_t port1_pin21_config = (/* Pin is configured as FC4_RXD_SDA_MOSI_DATA */
        IOCON_PIO_FUNC5 |
        /* Selects pull-up function */
        IOCON_PIO_MODE_PULLUP |
        /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_SLEW_STANDARD |
        /* Input function is not inverted */
        IOCON_PIO_INV_DI |
        /* Enables digital function */
        IOCON_PIO_DIGITAL_EN |
        /* Open drain is enabled */
        IOCON_PIO_OPENDRAIN_EN);
    /* PORT1 PIN21 (coords: 30) is configured as FC4_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void I2C1_DeinitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin14_config = (/* Pin is configured as PIO1_20 */
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
    /* PORT0 PIN14 (coords: 4) is configured as PIO1_20 */
    IOCON_PinMuxSet(IOCON, 0U, 14U, port1_pin14_config);

    const uint32_t port1_pin13_config = (/* Pin is configured as PIO1_21 */
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
    /* PORT0 PIN13 (coords: ?) is configured as PIO1_21 */
    IOCON_PinMuxSet(IOCON, 0U, 13U, port1_pin13_config);
}

void I2C4_DeinitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port1_pin20_config = (/* Pin is configured as PIO1_20 */
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
    /* PORT1 PIN20 (coords: 4) is configured as PIO1_20 */
    IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

    const uint32_t port1_pin21_config = (/* Pin is configured as PIO1_21 */
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
    /* PORT1 PIN21 (coords: 30) is configured as PIO1_21 */
    IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
}
