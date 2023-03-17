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

#include "mcux-sdk/drivers/lpadc/fsl_lpadc.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_clock.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_power.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"
#include "lpc55s00/pin_mux.h"


int adc_init(adc_dev_t *adc) {
    int rc = -MP_EINVAL;

    /* Maximum ADC frequency is 24 MHz. Chose 12 MHz */
    const uint32_t adc_freq = 12000000U;
    uint32_t adc_div = 0U;

    if (adc_freq < SystemCoreClock) {
        adc_div = SystemCoreClock / adc_freq;
    }

    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, adc_div, true);
    CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK);

    /* Disable LDOGPADC power down */
    POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);

    ADC_Type *adc_dev = (ADC_Type *)adc;
    lpadc_config_t mLpadcConfigStruct;

    LPADC_GetDefaultConfig(&mLpadcConfigStruct);
    mLpadcConfigStruct.enableAnalogPreliminary = true;
    /* VREFH = VDDA pin */
    mLpadcConfigStruct.referenceVoltageSource = kLPADC_ReferenceVoltageAlt2;
    mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;
    LPADC_Init(adc_dev, &mLpadcConfigStruct);

    rc = 0;

    return rc;
}


int adc_deinit(adc_dev_t *adc) {
    ADC_Type *adc_dev = (ADC_Type *)adc;
    LPADC_Deinit(adc_dev);

    return 0;
}


int adc_caibrate(adc_dev_t *adc) {
    ADC_Type *adc_dev = (ADC_Type *)adc;

    uint32_t LPADC_OFFSET_VALUE_A, LPADC_OFFSET_VALUE_B;
    LPADC_OFFSET_VALUE_A = LPADC_OFFSET_VALUE_B = 0x10;

    LPADC_SetOffsetValue(adc_dev, LPADC_OFFSET_VALUE_A, LPADC_OFFSET_VALUE_B);
    LPADC_DoAutoCalibration(adc_dev);

    return 0;
}


int adc_enable_channel(adc_dev_t *adc, const uint32_t channel) {
    lpadc_conv_command_config_t mLpadcCommandConfigStruct;
    lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
    ADC_Type *adc_dev = (ADC_Type *)adc;
    const uint32_t channel_idx = channel + 1U;

    LPADC_GetDefaultConvCommandConfig(&mLpadcCommandConfigStruct);
    mLpadcCommandConfigStruct.channelNumber = channel;
    mLpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    LPADC_SetConvCommandConfig(adc_dev, channel_idx, &mLpadcCommandConfigStruct);

    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
    mLpadcTriggerConfigStruct.targetCommandId = channel_idx;
    LPADC_SetConvTriggerConfig(adc_dev, channel, &mLpadcTriggerConfigStruct);

    return 0;
}


int adc_disable_channel(adc_dev_t *adc, const uint32_t channel) {
    lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
    ADC_Type *adc_dev = (ADC_Type *)adc;

    LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
    LPADC_SetConvTriggerConfig(adc_dev, channel, &mLpadcTriggerConfigStruct);

    return 0;
}


int adc_read(adc_dev_t *adc, adc_value_t *value, const uint32_t channel, uint32_t timeout) {
    ADC_Type *adc_dev = (ADC_Type *)adc;

    LPADC_DoSoftwareTrigger(adc_dev, 1U); /* 1U is trigger0 mask. */

    lpadc_conv_result_t mLpadcResultConfigStruct;
    while (!LPADC_GetConvResult(adc_dev, &mLpadcResultConfigStruct, 0U)) {
    }

    *value = (adc_value_t)mLpadcResultConfigStruct.convValue;

    return 0;
}


int adc_ref_voltage(adc_dev_t *adc, uint32_t *voltage) {
    return -ENOSYS;
}
