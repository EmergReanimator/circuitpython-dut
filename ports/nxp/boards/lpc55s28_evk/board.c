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

#include "supervisor/board.h"

#include "driver/fsl_cmsis/LPC55S28/fsl_common.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_clock.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_power.h"
#include "driver/fsl_cmsis/LPC55S28/fsl_iocon.h"
#include "boards/lpc55s28_evk/clock_config.h"
#include "boards/lpc55s28_evk/pin_mux.h"

#include "device.h"


STATIC void __BOARD_InitPins(void) {
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin22_config = (/* Pin is configured as USB0_VBUS */
        IOCON_PIO_FUNC7 |
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
    /* PORT0 PIN22 (coords: 78) is configured as USB0_VBUS */
    IOCON_PinMuxSet(IOCON, 0U, 22U, port0_pin22_config);

    const uint32_t port0_pin10_config = (/* Pin is configured as SWO */
        IOCON_PIO_FUNC6 |
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
    /* PORT0 PIN8 (coords: 21) is configured as SWO */
    IOCON_PinMuxSet(IOCON, 0U, 10U, port0_pin10_config);

    return;
}


void board_init(void) {
    __BOARD_InitPins();

    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);

    /* Board initialization */
    __BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    SystemCoreClockUpdate();

    NVIC_ClearPendingIRQ(USB0_IRQn);
    NVIC_ClearPendingIRQ(USB0_NEEDCLK_IRQn);

    POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*< Turn on USB0 Phy */

    /* reset the IP to make sure it's in reset state. */
    RESET_PeripheralReset(kUSB0D_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HSL_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB0HMR_RST_SHIFT_RSTn);

    #if (1)
    POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*< Turn on USB0 Phy */

    // Enable USB Clock Adjustments to trim the FRO for the full speed controller
    ANACTRL->FRO192M_CTRL |= ANACTRL_FRO192M_CTRL_USBCLKADJ_MASK;

    CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 1, false);
    CLOCK_AttachClk(kFRO_HF_to_USB0_CLK);

    /* enable usb0 host clock */
    CLOCK_EnableClock(kCLOCK_Usbhsl0);

    /*According to reference manual, device mode setting has to be set by access usb host register */
    #if (0)
    *((uint32_t *)(USBFSH_BASE + 0x5C)) |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
    #else
    USBFSH->PORTMODE |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
    #endif

    /* disable usb0 host clock */
    CLOCK_DisableClock(kCLOCK_Usbhsl0);
    #endif

    /* Initialise the USB peripheral clock */
    /* enable USB IP clock */
    CLOCK_EnableUsbfs0DeviceClock(kCLOCK_UsbfsSrcFro, CLOCK_GetFroHfFreq());

    #if defined(FSL_FEATURE_USB_USB_RAM) && (FSL_FEATURE_USB_USB_RAM)
    for (size_t i = 0; i < FSL_FEATURE_USB_USB_RAM; i++)
    {
        ((uint8_t *)FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
    }
    #endif

    return;
}

bool board_requests_safe_mode(void) {
    return false;
}

void reset_board(void) {
    return;
}
