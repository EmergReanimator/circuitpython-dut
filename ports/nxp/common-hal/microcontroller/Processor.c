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

#include "py/mphal.h"
#include "common-hal/microcontroller/Processor.h"
#include "shared-bindings/microcontroller/ResetReason.h"
#include "device.h"

#include "driver/iap.h"


float common_hal_mcu_processor_get_temperature(void) {
    #if (1)
    /* TODO: Implement CPU temperature acquisition. */
    return NAN;
    #else
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    uint16_t value = adc_read();
    adc_set_temp_sensor_enabled(false);
    float voltage = value * 3.3 / (1 << 12);
    // TODO: turn the ADC back off
    return 27 - (voltage - 0.706) / 0.001721;
    #endif
}

float common_hal_mcu_processor_get_voltage(void) {
    return NAN;
}

uint32_t common_hal_mcu_processor_get_frequency(void) {
    return SystemCoreClock;
}

void common_hal_mcu_processor_get_uid(uint8_t raw_id[]) {
    assert((COMMON_HAL_MCU_PROCESSOR_UID_LENGTH >= 8U));
    memset(&raw_id[0U], 0U, COMMON_HAL_MCU_PROCESSOR_UID_LENGTH);

    NXP_IAP_ReadUniqueID(&raw_id[0], COMMON_HAL_MCU_PROCESSOR_UID_LENGTH);

    return;
}

mcu_reset_reason_t common_hal_mcu_processor_get_reset_reason(void) {
    /* TODO: Get the reason from RSID */
    return RESET_REASON_UNKNOWN;
}
