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

#include "driver/adc.h"
#include "py/mperrno.h"

#include "nxp_lpcopen/lpc175x_6x/lpc_chip_175x_6x/inc/chip.h"


STATIC uint32_t __channel_status = 0U;


int adc_init(adc_dev_t *adc) {
    LPC_ADC_T *adc_dev = (LPC_ADC_T *)adc;
    ADC_CLOCK_SETUP_T ADCSetup;
    ADCSetup.adcRate = 100U;
    ADCSetup.bitsAccuracy = 16U;
    ADCSetup.burstMode = false;

    /* ... Clocks are enabled by Chip_ADC_Init function */
    Chip_ADC_Init(adc_dev, &ADCSetup);

    return 0;
}


int adc_deinit(adc_dev_t *adc) {
    LPC_ADC_T *adc_dev = (LPC_ADC_T *)adc;

    /* ... Clocks are disabled by Chip_ADC_DeInit function */
    Chip_ADC_DeInit(adc_dev);

    return 0;
}


int adc_caibrate(adc_dev_t *adc) {
    /* ... Factory calibrated */
    return 0;
}


int adc_enable_channel(adc_dev_t *adc, const uint32_t channel) {
    int rc = -MP_EINVAL;

    if (channel <= ((uint32_t)ADC_CH7)) {
        LPC_ADC_T *adc_dev = (LPC_ADC_T *)adc;
        ADC_CHANNEL_T adc_channel = (ADC_CHANNEL_T)channel;
        Chip_ADC_EnableChannel(adc_dev, adc_channel, ENABLE);

        if (!__channel_status) {
            Chip_ADC_SetStartMode(adc_dev, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
        }

        __channel_status |= (1U << channel);

        rc = 0;
    }

    return rc;
}


int adc_disable_channel(adc_dev_t *adc, const uint32_t channel) {
    int rc = -MP_EINVAL;

    if (channel <= ((uint32_t)ADC_CH7)) {
        LPC_ADC_T *adc_dev = (LPC_ADC_T *)adc;
        ADC_CHANNEL_T adc_channel = (ADC_CHANNEL_T)channel;
        Chip_ADC_EnableChannel(adc_dev, adc_channel, DISABLE);

        __channel_status &= ~(1U << channel);

        if (!__channel_status) {
            Chip_ADC_SetStartMode(adc_dev, ADC_NO_START, ADC_TRIGGERMODE_RISING);
        }

        rc = 0;
    }

    return rc;
}


int adc_read(adc_dev_t *adc, adc_value_t *value, const uint32_t channel, uint32_t timeout) {
    int rc = -MP_EINVAL;

    if (channel <= ((uint32_t)ADC_CH7)) {
        LPC_ADC_T *adc_dev = (LPC_ADC_T *)adc;
        ADC_CHANNEL_T adc_channel = (ADC_CHANNEL_T)channel;

        {
            FlagStatus flags = Chip_ADC_ReadStatus(adc_dev, adc_channel, ADC_DR_DONE_STAT);

            /* TODO: Timeout must be handled here */
            while (!flags) {
                flags = Chip_ADC_ReadStatus(adc_dev, adc_channel, ADC_DR_DONE_STAT);
            }
        }

        uint16_t adc_value = 0U;
        Status status = Chip_ADC_ReadValue(adc_dev, adc_channel, &adc_value);

        if (SUCCESS == status) {
            const uint32_t max = ADC_DR_RESULT(UINT32_MAX);

            adc_value += 1U;
            adc_value *= (UINT16_MAX / max);
            adc_value -= 1U;
            *value = (adc_value_t)adc_value;

            rc = 0;
        } else {
            rc = -MP_EIO;
        }
    }

    return rc;
}


int adc_ref_voltage(adc_dev_t *adc, uint32_t *voltage) {
    /* FIXME: Obtain board dependent reference voltage. */
    voltage = 0U;

    return -ENOSYS;
}
