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

#include "common-hal/canio/CAN.h"
#include "boards/mcb1700/can_config.h"
#include "boards/mcb1700/CMSIS/Driver/Config/RTE_Device.h"

#include "device.h"


extern ARM_DRIVER_CAN Driver_CAN1;
extern ARM_DRIVER_CAN Driver_CAN2;

const static can_pin_set_t CAN0_pin_set[] =
{
    /* Pin set 0 */
    {
        .tx = NXP_PORT_GPIO_PIN(RTE_CAN1_TD_PORT, RTE_CAN1_TD_BIT),
        .rx = NXP_PORT_GPIO_PIN(RTE_CAN1_RD_PORT, RTE_CAN1_RD_BIT),
    },
};

const static can_pin_set_t CAN1_pin_set[] =
{
    /* Pin set 0 */
    {
        .tx = NXP_PORT_GPIO_PIN(RTE_CAN2_TD_PORT, RTE_CAN2_TD_BIT),
        .rx = NXP_PORT_GPIO_PIN(RTE_CAN2_RD_PORT, RTE_CAN2_RD_BIT),
    },
};

static can_inst_t can_instances[CAN_INSTANCES_NUM] = {
    /* CAN Instance 0 */
    {
        .id = 0U,
        .is_used = false,
        .driver = &Driver_CAN1,
        .pin_map = &CAN0_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(CAN0_pin_set),
    },

    /* CAN Instance 1 */
    {
        .id = 1U,
        .is_used = false,
        .driver = &Driver_CAN2,
        .pin_map = &CAN1_pin_set[0U],
        .pin_map_len = MP_ARRAY_SIZE(CAN1_pin_set),
    },
};

can_inst_t *get_can_instance(const size_t n) {
    if (n < CAN_INSTANCES_NUM) {
        return &can_instances[n];
    } else {
        return (can_inst_t *)NULL;
    }
}

void can_enable(can_inst_t *can_instance) {
    /* No need to peripheral here.
     * Clock is handled by the CAN driver itself. */

    return;
}

void can_disable(can_inst_t *can_instance) {
    /* No need to peripheral here.
     * Clock is handled by the CAN driver itself. */

    return;
}
