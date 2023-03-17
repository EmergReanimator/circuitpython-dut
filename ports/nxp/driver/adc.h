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

#if !defined(PORTS_NXP_DRIVER_ADC_H_)
#define PORTS_NXP_DRIVER_ADC_H_


#include <stdint.h>

typedef struct {} adc_dev_t;
typedef uint32_t adc_value_t;


int adc_init(adc_dev_t *adc);
int adc_deinit(adc_dev_t *adc);
int adc_caibrate(adc_dev_t *adc);
int adc_enable_channel(adc_dev_t *adc, const uint32_t channel);
int adc_disable_channel(adc_dev_t *adc, const uint32_t channel);
int adc_read(adc_dev_t *adc, adc_value_t *value, const uint32_t channel, uint32_t timeout);
int adc_ref_voltage(adc_dev_t *adc, uint32_t *voltage);


#endif // PORTS_NXP_DRIVER_ADC_H_
