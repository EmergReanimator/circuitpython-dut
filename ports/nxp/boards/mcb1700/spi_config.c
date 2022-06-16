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

#include "common-hal/busio/SPI.h"
#include "boards/mcb1700/spi_config.h"
#include "boards/mcb1700/CMSIS/Driver/Config/RTE_Device.h"

#include "device.h"


extern ARM_DRIVER_SPI Driver_SPI0;
extern ARM_DRIVER_SPI Driver_SPI1;
extern ARM_DRIVER_SPI Driver_SPI2;

const static spi_pin_set_t SPI0_pin_set[] =
{
    /* Pin set 0 */
    {
        .clock = NXP_PORT_GPIO_PIN(RTE_SSP0_SCK_PORT,  RTE_SSP0_SCK_BIT),
        .mosi = NXP_PORT_GPIO_PIN(RTE_SSP0_MOSI_PORT, RTE_SSP0_MOSI_BIT),
        .miso = NXP_PORT_GPIO_PIN(RTE_SSP0_MISO_PORT, RTE_SSP0_MISO_BIT),
    },
};

const static spi_pin_set_t SPI1_pin_set[] =
{
    /* Pin set 0 */
    {
        .clock = NXP_PORT_GPIO_PIN(RTE_SSP1_SCK_PORT,  RTE_SSP1_SCK_BIT),
        .mosi = NXP_PORT_GPIO_PIN(RTE_SSP1_MOSI_PORT, RTE_SSP1_MOSI_BIT),
        .miso = NXP_PORT_GPIO_PIN(RTE_SSP1_MISO_PORT, RTE_SSP1_MISO_BIT),
    },
};

const static spi_pin_set_t SPI2_pin_set[] =
{
    /* Pin set 0 */
    {
        .clock = NXP_PORT_GPIO_PIN(RTE_SPI_SCK_PORT,  RTE_SPI_SCK_BIT),
        .mosi = NXP_PORT_GPIO_PIN(RTE_SPI_MOSI_PORT, RTE_SPI_MOSI_BIT),
        .miso = NXP_PORT_GPIO_PIN(RTE_SPI_MISO_PORT, RTE_SPI_MISO_BIT),
    },
};

static spi_inst_t spi_instances[SPI_INSTANCES_NUM] = {
    /* SPI Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .driver = &Driver_SPI0,
        .pin_map = &SPI0_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(SPI0_pin_set),
    },

    /* SPI Instance 1 */
    {
        .id = 1U,
        .is_used = false,
        .driver = &Driver_SPI1,
        .pin_map = &SPI1_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(SPI1_pin_set),
    },

    /* SPI Instance 2 */
    {
        .id = 2U,
        .is_used = false,
        .driver = &Driver_SPI2,
        .pin_map = &SPI2_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(SPI2_pin_set),
    },
};

spi_inst_t *get_spi_instance(const size_t n) {
    if (n < SPI_INSTANCES_NUM) {
        return &spi_instances[n];
    } else {
        return (spi_inst_t *)NULL;
    }
}

void spi_enable(spi_inst_t *spi_instance) {
    /* No need to peripheral here.
     * Clock is handled by the SPI driver itself. */

    return;
}

void spi_disable(spi_inst_t *spi_instance) {
    /* No need to peripheral here.
     * Clock is handled by the SPI driver itself. */

    return;
}
