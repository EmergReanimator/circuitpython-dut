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

#include "py/mpprint.h"
#include "diag/Trace.h"
#include <stdbool.h>


STATIC void swo_write_substring(const char *text, uint32_t length) {
    trace_write(text, length);
}

STATIC void mp_hal_swo_tx_strn(const char *str, size_t len) {
    swo_write_substring(str, len);
}

// Send "cooked" string of given length, where every occurrence of
// LF character is replaced with CR LF.
STATIC void mp_hal_swo_tx_strn_cooked(const char *str, size_t len) {
    bool last_cr = false;
    while (len > 0) {
        size_t i = 0;
        if (str[0] == '\n' && !last_cr) {
            mp_hal_swo_tx_strn("\r", 1);
            i = 1;
        }
        // Lump all characters on the next line together.
        while ((last_cr || str[i] != '\n') && i < len) {
            last_cr = str[i] == '\r';
            i++;
        }
        mp_hal_swo_tx_strn(str, i);
        str = &str[i];
        len -= i;
    }
}

STATIC void swo_print_strn(void *env, const char *str, size_t len) {
    (void)env;
    mp_hal_swo_tx_strn_cooked(str, len);
}

const mp_print_t mp_swo_print = {NULL, swo_print_strn};
