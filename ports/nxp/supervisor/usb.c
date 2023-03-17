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

#include "lib/tinyusb/src/device/usbd.h"
#include "supervisor/background_callback.h"
#include "supervisor/usb.h"
#include "py/mpconfig.h"    /* STATIC */

#include "cmsis5/CMSIS/Driver/Include/Driver_USBD.h"

extern ARM_DRIVER_USBD Driver_USBD0;

#if (0)
#include "src/rp2_common/hardware_irq/include/hardware/irq.h"
#include "src/rp2_common/pico_platform/include/pico/platform.h"
#include "src/rp2040/hardware_regs/include/hardware/regs/intctrl.h"
#endif

/* FIXME: IRQ handlers are different for different MCUs */
void CPY_USB0_IRQHandler(void) {
    usb_irq_handler(CIRCUITPY_USB_DEVICE_INSTANCE);
}

STATIC void __USBD_SignalDeviceEvent(uint32_t event) {
    return;
}

STATIC void __USBD_SignalEndpointEvent(uint8_t ep_addr, uint32_t event) {
    return;
}

void init_usb_hardware(void) {
    ARM_DRIVER_USBD *usb_drv = &Driver_USBD0;

    /* TODO: Enable USB here */
    usb_drv->Initialize(__USBD_SignalDeviceEvent, __USBD_SignalEndpointEvent);

    usb_drv->PowerControl(ARM_POWER_FULL);

    return;
}

void post_usb_init(void) {
    #if (1)
    return;
    #else
    irq_set_enabled(USBCTRL_IRQ, false);

    irq_handler_t usb_handler = irq_get_exclusive_handler(USBCTRL_IRQ);
    if (usb_handler) {
        irq_remove_handler(USBCTRL_IRQ, usb_handler);
    }
    irq_set_exclusive_handler(USBCTRL_IRQ, usb_irq_handler);

    irq_set_enabled(USBCTRL_IRQ, true);
    #endif
}
