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

#include <stdint.h>
#include <string.h> // memset

#include "Driver_USBD.h"
#include "device.h"

#define USBD_DRIVER_INITIALIZED         (1U << 0)
#define USBD_DRIVER_POWERED             (1U << 1)

extern ARM_DRIVER_USBD Driver_USBD0;

// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,0)

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
    1U, // VBUS Detection
    1U, // Event VBUS On
    1U  // Event VBUS Off
    #if (defined(ARM_USBD_API_VERSION) && (ARM_USBD_API_VERSION >= 0x202U))
    , 0U
    #endif
};

static uint8_t usb_state = 0U;
static uint8_t usb_role = ARM_USB_ROLE_NONE;
static ARM_USBD_STATE usbd_state;

static ARM_USBD_SignalDeviceEvent_t SignalDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t SignalEndpointEvent;

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion(void) {
    return usbd_driver_version;
}


/**
  \fn          ARM_USBD_CAPABILITIES USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities(void) {
    return usbd_driver_capabilities;
}


/**
  \fn          int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                        ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBD_Initialize(ARM_USBD_SignalDeviceEvent_t cb_device_event,
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

    if ((usb_state & USBD_DRIVER_INITIALIZED) != 0U) {
        return ARM_DRIVER_OK;
    } else if ((NULL == cb_device_event) || (NULL == cb_endpoint_event)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    SignalDeviceEvent = cb_device_event;
    SignalEndpointEvent = cb_endpoint_event;

    usb_role = ARM_USB_ROLE_DEVICE;

    #if (0)
    if (USB_PinsConfigure() == -1) {
        usb_role = ARM_USB_ROLE_NONE;
        return ARM_DRIVER_ERROR;
    }
    #endif

    usb_state |= USBD_DRIVER_INITIALIZED;

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize(void) {
    #if (0)
    if (USB_PinsUnconfigure() == -1) {
        return ARM_DRIVER_ERROR;
    }
    #endif
    usb_role = ARM_USB_ROLE_NONE;
    usb_state = 0U;

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl(ARM_POWER_STATE state) {

    switch (state) {
        case ARM_POWER_OFF:
            NVIC_DisableIRQ(USB0_IRQn);                  // Disable interrupt
            NVIC_ClearPendingIRQ(USB0_IRQn);             // Clear pending interrupt
            usb_state &= ~USBD_DRIVER_POWERED;          // Clear powered flag
                                                        // Reset variables
            #if (0)
            setup_received = 0U;
            #endif

            memset((void *)&usbd_state, 0, sizeof(usbd_state));

            #if (0)
            LPC_SC->PCONP |= (1UL << 31);               // USB PCLK -> enable USB Peripheral
            #if   defined(RTE_Drivers_USBH0)
            LPC_USB->OTGClkCtrl |= 0x1AU;               // Enable AHB, OTG and Device clocks
            while ((LPC_USB->OTGClkSt & 0x1AU) != 0x1AU) {
                ;                                       // Wait for AHB, OTG and Device clocks
            }
            #else
            LPC_USB->OTGClkCtrl |= 0x12U;               // Enable AHB and Device clocks
            while ((LPC_USB->OTGClkSt & 0x12U) != 0x12U) {
                ;                                       // Wait for AHB and Device clocks
            }
            #endif
            LPC_USB->USBClkCtrl |= 0x12U;               // Enable AHB and Device clocks
            while ((LPC_USB->USBClkSt & 0x12U) != 0x12U) {
                ;
            }

            LPC_USB->DevIntEn = 0U;                     // Disable USB Controller Device interrupts

            LPC_USB->USBClkCtrl &= ~0x02U;              // Disable Device clock
            while ((LPC_USB->USBClkSt & 0x02U) != 0U) {
                ;
            }

            if ((usb_state & 0xF0U) == 0U) {            // If Host is not enabled
                LPC_USB->USBClkCtrl &= ~0x10U;          // Disable AHB clock
                while ((LPC_USB->USBClkSt & 0x10U) != 0U) {
                    ;
                }
                #if     defined(RTE_Drivers_USBH0)
                LPC_USB->OTGClkCtrl &= ~0x1AU;          // Disable AHB, OTG and Device clocks
                while ((LPC_USB->OTGClkSt & 0x1AU) != 0U) {
                    ;                                   // Wait for AHB, OTG and Device clocks disabled
                }
                #else
                LPC_USB->OTGClkCtrl &= ~0x12U;          // Disable AHB and Device clock
                while ((LPC_USB->OTGClkSt & 0x12U) != 0U) {
                    ;                                   // Wait for AHB and Device clock disabled
                }
                #endif
                LPC_SC->PCONP &= ~(1UL << 31);          // USB PCLK -> disable USB Peripheral
            } else {                                    // If Host is enabled
                NVIC_EnableIRQ(USB_IRQn);               // Enable interrupt
            }
            #endif
            break;

        case ARM_POWER_FULL:
            if ((usb_state & USBD_DRIVER_INITIALIZED) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            if ((usb_state & USBD_DRIVER_POWERED) != 0U) {
                return ARM_DRIVER_OK;
            }

            #if (0)
            LPC_SC->PCONP |= (1UL << 31);               // USB PCLK -> enable USB Peripheral
            #if   defined(RTE_Drivers_USBH0)
            LPC_USB->OTGClkCtrl |= 0x1AU;               // Enable AHB, OTG and Device clocks
            while ((LPC_USB->OTGClkSt & 0x1AU) != 0x1AU) {
                ;                                       // Wait for AHB, OTG and Device clocks
            }
            #else
            LPC_USB->OTGClkCtrl |= 0x12U;               // Enable AHB and Device clocks
            while ((LPC_USB->OTGClkSt & 0x12U) != 0x12U) {
                ;                                       // Wait for AHB and Device clocks
            }
            #endif
            LPC_USB->USBClkCtrl |= 0x12U;               // Enable AHB and Device clocks
            while ((LPC_USB->USBClkSt & 0x12U) != 0x12U) {
                ;                                       // Wait for AHB and Device clocks

            }
            #if  (defined(LPC175x_6x) && defined(RTE_Drivers_USBH0))
            LPC_USB->StCtrl &= ~0x01U;                  // Reset port function
            #elif defined(LPC177x_8x)
            LPC_USB->USBClkCtrl |= 0x08U;               // Enable port select register clocks
            while ((LPC_USB->USBClkSt & 0x08U) == 0U) {
                ;
            }
            LPC_USB->StCtrl &= ~0x03U;                  // Reset port function
            if (LPC_USB->StCtrl != RTE_USB_PORT_CFG) {
                LPC_USB->StCtrl |= RTE_USB_PORT_CFG;    // Select port function
            }
            #if ((RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
            USB_I2C_Initialize();                       // Initialize I2C for OTG Transceiver
            #endif
            #endif
            LPC_USB->DevIntEn = USBD_DEV_STAT_INT | USBD_EP_SLOW_INT;

            usb_state |= USBD_DRIVER_POWERED;           // Set powered flag
            #endif
            NVIC_EnableIRQ(USB0_IRQn);                  // Enable interrupt
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceConnect(void) {
    #if (0)
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    SIE_WrCmdData(USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(USBD_DEV_CON));

    #if (defined(LPC177x_8x) && (RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
    USB_I2C_DpPullUp(true);
    #endif

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif
    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect(void) {

    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    #if (0)
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    #if (defined(LPC177x_8x) && (RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
    USB_I2C_DpPullUp(false);
    #endif

    SIE_WrCmdData(USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(0));

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif
    return ARM_DRIVER_OK;
}


/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState(void) {
    return usbd_state;
}


/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup(void) {

    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    #if (0)
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    USBD_PowerControl(ARM_POWER_FULL);

    SIE_WrCmdData(USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(SIE_RdCmdData(USBD_CMD_GET_DEV_STAT) & (~USBD_DEV_SUS)));

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBD_DeviceSetAddress(uint8_t dev_addr) {

    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    #if (0)
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    SIE_WrCmdData(USBD_CMD_SET_ADDR, USBD_DAT_WR_BYTE(USBD_DEV_EN | dev_addr));
    SIE_WrCmdData(USBD_CMD_SET_ADDR, USBD_DAT_WR_BYTE(USBD_DEV_EN | dev_addr));

    // Set device configured
    SIE_WrCmdData(USBD_CMD_CFG_DEV,  USBD_DAT_WR_BYTE(USBD_CONF_DVICE));

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket(uint8_t *setup) {

    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    #if (0)
    if (setup_received == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    setup_received = 0U;
    USBD_HW_ReadEP(0, setup);

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }

    if (setup_received != 0U) {         // If new setup packet was received while this was being read
        return ARM_DRIVER_ERROR;
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                               uint8_t  ep_type,
                                               uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBD_EndpointConfigure(uint8_t ep_addr,
    uint8_t ep_type,
    uint16_t ep_max_packet_size) {
    #if (0)
    ENDPOINT_t *ptr_ep;
    uint32_t ep_phy;

    ep_phy = EP_PHY(ep_addr);
    ptr_ep = &ep[ep_phy];
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return ARM_DRIVER_ERROR;
    }
    switch (ep_type) {
        case ARM_USB_ENDPOINT_CONTROL:
            if ((USBD_CTRL_ENDPOINT_MASK & (1U << ep_phy)) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            break;
        case ARM_USB_ENDPOINT_ISOCHRONOUS:
            if ((USBD_ISO_ENDPOINT_MASK & (1U << ep_phy)) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            break;
        case ARM_USB_ENDPOINT_BULK:
            if ((USBD_BULK_ENDPOINT_MASK & (1U << ep_phy)) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            break;
        case ARM_USB_ENDPOINT_INTERRUPT:
            if ((USBD_INT_ENDPOINT_MASK & (1U << ep_phy)) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            break;
        default:
            return ARM_DRIVER_ERROR;
    }
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    // Clear Endpoint settings in memory
    memset((void *)ptr_ep, 0, sizeof(ENDPOINT_t));

    // Store Endpoint settings in memory
    ptr_ep->type = ep_type;
    ptr_ep->max_packet_size = ep_max_packet_size;
    if (ep_type == ARM_USB_ENDPOINT_ISOCHRONOUS) {
        // For isochronous endpoints transfers are handled on SOF
        ep_iso_cfg_mask |= (1U << ep_phy);
        if (ep_iso_cfg_mask != 0U) {
            LPC_USB->DevIntEn |= USBD_FRAME_INT;
        }
    }

    USBD_HW_ConfigEP(ep_phy, ep_max_packet_size);
    SIE_WrCmdData(USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(0));
    USBD_HW_EnableEP(ep_phy);

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointUnconfigure(uint8_t ep_addr) {
    #if (0)
    ENDPOINT_t *ptr_ep;
    uint32_t ep_phy;

    ep_phy = EP_PHY(ep_addr);
    ptr_ep = &ep[ep_phy];
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return ARM_DRIVER_ERROR;
    }
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    USBD_HW_DisableEP(ep_phy);

    if (ptr_ep->type == ARM_USB_ENDPOINT_ISOCHRONOUS) {
        ep_iso_cfg_mask &= ~(1U << ep_phy);
        if (ep_iso_cfg_mask == 0U) {
            LPC_USB->DevIntEn &= ~USBD_FRAME_INT;
        }
    }

    // Clear Endpoint settings in memory
    memset((void *)ptr_ep, 0, sizeof(ENDPOINT_t));

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif
    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBD_EndpointStall(uint8_t ep_addr, bool stall) {
    #if (0)
    uint32_t ep_phy;

    ep_phy = EP_PHY(ep_addr);
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return ARM_DRIVER_ERROR;
    }
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    if (stall != 0U) {
        SIE_WrCmdData(USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(USBD_EP_STAT_ST));
    } else {
        SIE_WrCmdData(USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(0));
    }

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransfer(uint8_t ep_addr, uint8_t *data, uint32_t num) {
    #if (0)
    ENDPOINT_t *ptr_ep;
    uint32_t ep_phy, ep_msk;

    ep_phy = EP_PHY(ep_addr);
    ptr_ep = &ep[ep_phy];
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return ARM_DRIVER_ERROR;
    }
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (ptr_ep->active != 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    ep_msk = 1U << ep_phy;

    ptr_ep->data = data;
    ptr_ep->num = num;
    ptr_ep->num_transferred_total = 0U;
    ptr_ep->num_transferring = 0U;

    if ((ep_iso_cfg_mask & ep_msk) != 0U) {
        ep_iso_act_mask |= ep_msk;
    }

    // Ignore OUT ZLP request as it is automatically received by hardware and next SETUP
    // can over-write it before it was activated for transfer
    if ((ep_phy != 0U) ||
        ((ep_phy == 0U) && (num != 0U))) {
        if ((ep_addr & 0x80U) != 0U) {                                    // for IN Endpoint
            ptr_ep->active = 1U;
            ptr_ep->num_transferring = (uint16_t)USBD_HW_WriteEP(ep_phy >> 1, ptr_ep->data, ptr_ep->num);
        } else {                                                          // for OUT Endpoint
            ptr_ep->active = 1U;
            if (ptr_ep->out_irq_pending != 0U) {                          // if something was already received
                ptr_ep->out_irq_pending = 0U;
                LPC_USB->EpIntSet = ep_msk;                               // force interrupt to handle already received data
            }
        }
    }

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBD_EndpointTransferGetResult(uint8_t ep_addr) {
    #if (0)
    uint32_t ep_phy;

    ep_phy = EP_PHY(ep_addr);
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return 0U;
    }

    return ep[ep_phy].num_transferred_total;
    #else
    return 0U;
    #endif
}


/**
  \fn          int32_t USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransferAbort(uint8_t ep_addr) {
    #if (0)
    uint32_t ep_phy;

    ep_phy = EP_PHY(ep_addr);
    if (ep_phy > ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U)) {
        return ARM_DRIVER_ERROR;
    }
    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    ep[ep_phy].active = 0U;
    if ((ep_iso_act_mask & (1U << ep_phy)) != 0U) {
        if ((ep_addr & 0x80) != 0U) {
            USBD_HW_WriteEP(ep_phy >> 1, NULL, 0);
        }
        ep_iso_act_mask &= ~(1U << ep_phy);
    }

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return ARM_DRIVER_OK;
}


/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber(void) {
    uint16_t val;

    if ((usb_state & USBD_DRIVER_POWERED) == 0U) {
        return 0U;
    }
    #if (1)
    val = 0U;
    #else
    if (check_and_set_var(&usbd_sie_busy) == 0U) {
        return 0U;
    }

    if (usbd_int_active == 0U) {
        NVIC_DisableIRQ(USB_IRQn);
    }

    SIE_WrCmd(USBD_CMD_RD_FRAME);
    val = (uint16_t)SIE_RdCmdData(USBD_DAT_RD_FRAME);
    val |= SIE_RdCmdData(USBD_DAT_RD_FRAME) << 8;

    usbd_sie_busy = 0U;
    if (usbd_int_active == 0U) {
        NVIC_EnableIRQ(USB_IRQn);
    }
    #endif

    return val;
}


/**
  \fn          void USB0_IRQHandler (void)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USB0_IRQHandler(void) {
    SignalEndpointEvent(0U, 0U);
}


ARM_DRIVER_USBD Driver_USBD0 = {
    USBD_GetVersion,
    USBD_GetCapabilities,
    USBD_Initialize,
    USBD_Uninitialize,
    USBD_PowerControl,
    USBD_DeviceConnect,
    USBD_DeviceDisconnect,
    USBD_DeviceGetState,
    USBD_DeviceRemoteWakeup,
    USBD_DeviceSetAddress,
    USBD_ReadSetupPacket,
    USBD_EndpointConfigure,
    USBD_EndpointUnconfigure,
    USBD_EndpointStall,
    USBD_EndpointTransfer,
    USBD_EndpointTransferGetResult,
    USBD_EndpointTransferAbort,
    USBD_GetFrameNumber
};
