/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RTE_DEVICE_H
#define _RTE_DEVICE_H

/* Driver name mapping. */
/* User needs to provide the implementation of USARTX_GetFreq/USARTX_InitPins/USARTX_DeinitPins for the enabled USART instance. */
extern void USART0_InitPins();
extern void USART0_DeinitPins();
extern void BOARD_InitSW2(void); /* Function assigned for the Cortex-M33 (Core #0) */
extern void BOARD_InitLEDBlue(void); /* Function assigned for the Cortex-M33 (Core #0) */

/* User needs to provide the implementation of I2CX_GetFreq/I2CX_InitPins/I2CX_DeinitPins for the enabled I2C instance. */
extern void I2C0_InitPins();
extern void I2C0_DeinitPins();


// -------- <<< Use Configuration Wizard in Context Menu >>> --------------------
// <e> USB Controller [Driver_USBD and Driver_USBH]
// <i> Configuration settings for Driver_USBD in component ::Drivers:USB Device
// <i> Configuration settings for Driver_USBH in component ::Drivers:USB Host
#define   RTE_USB_USB0                  1

//   <h> Pin Configuration
//     <o> USB_PPWR (Host) <0=>Not used <1=>P1_19
//     <i> VBUS drive signal (towards external charge pump or power management unit).
#define   RTE_USB_PPWR_ID               0
#if      (RTE_USB_PPWR_ID == 0)
  #define RTE_USB_PPWR_PIN_EN           0
#elif    (RTE_USB_PPWR_ID == 1)
  #define RTE_USB_PPWR_PIN_EN           1
#else
  #error "Invalid RTE_USB_PPWR Pin Configuration!"
#endif

//     <o> USB_PWRD (Host) <0=>Not used <1=>P1_22
//     <i> Power Status for USB port.
#define   RTE_USB_PWRD_ID               0
#if      (RTE_USB_PWRD_ID == 0)
  #define RTE_USB_PWRD_PIN_EN           0
#elif    (RTE_USB_PWRD_ID == 1)
  #define RTE_USB_PWRD_PIN_EN           1
#else
  #error "Invalid RTE_USB_PWRD Pin Configuration!"
#endif

//     <o> USB_OVRCR (Host) <0=>Not used <1=>P1_27
//     <i> Port power fault signal indicating overcurrent condition.
//     <i> This signal monitors over-current on the USB bus
//        (external circuitry required to detect over-current condition).
#define   RTE_USB_OVRCR_ID              0
#if      (RTE_USB_OVRCR_ID == 0)
  #define RTE_USB_OVRCR_PIN_EN          0
#elif    (RTE_USB_OVRCR_ID == 1)
  #define RTE_USB_OVRCR_PIN_EN          1
#else
  #error "Invalid RTE_USB_OVRCR Pin Configuration!"
#endif

//     <o> USB_CONNECT (Device) <0=>Not used <1=>P2_9
//     <i> SoftConnect control signal
#define   RTE_USB_CONNECT_ID            1
#if      (RTE_USB_CONNECT_ID == 0)
  #define RTE_USB_CONNECT_PIN_EN        0
#elif    (RTE_USB_CONNECT_ID == 1)
  #define RTE_USB_CONNECT_PIN_EN        1
#else
  #error "Invalid RTE_USB_CONNECT Pin Configuration!"
#endif

//     <o> USB_VBUS (Device) <0=>Not used <1=>P1_30
//     <i> VBUS status input.
//     <i> When this function is not enabled via its corresponding PINSEL register, it is driven HIGH internally.
#define   RTE_USB_VBUS_ID               1
#if      (RTE_USB_VBUS_ID == 0)
  #define RTE_USB_VBUS_PIN_EN           0
#elif    (RTE_USB_VBUS_ID == 1)
  #define RTE_USB_VBUS_PIN_EN           1
#else
  #error "Invalid RTE_USB_VBUS Pin Configuration!"
#endif

//     <o> USB_UP_LED <0=>Not used <1=>P1_18
//     <i> GoodLink LED control signal.
#define   RTE_USB_UP_LED_ID             1
#if      (RTE_USB_UP_LED_ID == 0)
  #define RTE_USB_UP_LED_PIN_EN         0
#elif    (RTE_USB_UP_LED_ID == 1)
  #define RTE_USB_UP_LED_PIN_EN         1
#else
  #error "Invalid RTE_USB_UP_LED Pin Configuration!"
#endif

//  </h> Pin Configuration
// </e> USB Controller [Driver_USBD and Driver_USBH]


// <e> USART0 (Universal synchronous/asynchronous receiver transmitter)
#define RTE_USART0                  1

//   <o> USART0_TX Pin <0=>Not used <1=>P0_25 <2=>P0_30 <3=>P1_6
//   <i> USART0 Serial Output pin
#define RTE_USART0_TX_ID                 2
#if    (RTE_USART0_TX_ID == 0)
#define RTE_USART0_TX_PIN_EN             0
#elif  (RTE_USART0_TX_ID == 1)
#define RTE_USART0_TX_PORT               0
#define RTE_USART0_TX_BIT                25
#define RTE_USART0_TX_FUNC               1
#elif  (RTE_USART0_TX_ID == 2)
#define RTE_USART0_TX_PORT               0
#define RTE_USART0_TX_BIT                30
#define RTE_USART0_TX_FUNC               1
#elif  (RTE_USART0_TX_ID == 4)
#define RTE_USART0_TX_PORT               1
#define RTE_USART0_TX_BIT                6
#define RTE_USART0_TX_FUNC               1
#else
#error "Invalid USART0_TX Pin Configuration!"
#endif
#ifndef RTE_USART0_TX_PIN_EN
#define RTE_USART0_TX_PIN_EN             1
#endif

//   <o> USART0_RX Pin <0=>Not used <1=>P0_24 <2=>P0_29 <3=>P1_5
//   <i> USART0 Serial Input pin
#define RTE_USART0_RX_ID                 2
#if    (RTE_USART0_RX_ID == 0)
#define RTE_USART0_RX_PIN_EN             0
#elif  (RTE_USART0_RX_ID == 1)
#define RTE_USART0_RX_PORT               0
#define RTE_USART0_RX_BIT                24
#define RTE_USART0_RX_FUNC               1
#elif  (RTE_USART0_RX_ID == 2)
#define RTE_USART0_RX_PORT               0
#define RTE_USART0_RX_BIT                29
#define RTE_USART0_RX_FUNC               1
#elif  (RTE_USART0_RX_ID == 3)
#define RTE_USART0_RX_PORT               1
#define RTE_USART0_RX_BIT                5
#define RTE_USART0_RX_FUNC               1
#else
#error "Invalid USART0_RX Pin Configuration!"
#endif
#ifndef RTE_USART0_RX_PIN_EN
#define RTE_USART0_RX_PIN_EN             1
#endif

// <h> DMA
//   <e> Tx
//     <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//   </e>
#define RTE_USART0_DMA_EN           0
#define RTE_USART0_DMA_TX_CH        5

//   <e> Rx
//     <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//   </e>
#define RTE_USART0_DMA_EN           0
#define RTE_USART0_DMA_RX_CH        4
// </h>
// </e> USART0 (Universal synchronous/asynchronous receiver transmitter)

// <e> I2C0 (Inter-integrated Circuit Interface 0) [Driver_I2C0]
// <i> Configuration settings for Driver_I2C0 in component ::Drivers:I2C
#define RTE_I2C0                    0
#define RTE_I2C0_PIN_INIT           I2C0_InitPins
#define RTE_I2C0_PIN_DEINIT         I2C0_DeinitPins
#define RTE_I2C0_DMA_EN             1

// I2C configuration
#define RTE_I2C0_Master_DMA_BASE    DMA0
#define RTE_I2C0_Master_DMA_CH      13
// </e>

// <e> I2C1 (Inter-integrated Circuit Interface 4) [Driver_I2C1]
// <i> Configuration settings for Driver_I2C1 in component ::Drivers:I2C
#define RTE_I2C1                        1

//   <o> I2C1_SCL Pin <0=>P0_28
#define RTE_I2C1_SCL_PORT_ID            0
#if    (RTE_I2C1_SCL_PORT_ID == 0)
#define RTE_I2C1_SCL_PORT               0
#define RTE_I2C1_SCL_PIN                14
#define RTE_I2C1_SCL_FUNC               1
#else
#error "Invalid I2C1_SCL Pin Configuration!"
#endif

//   <o> I2C1_SDA Pin <0=>P0_27
#define RTE_I2C1_SDA_PORT_ID            0
#if    (RTE_I2C1_SDA_PORT_ID == 0)
#define RTE_I2C1_SDA_PORT               0
#define RTE_I2C1_SDA_PIN                13
#define RTE_I2C1_SDA_FUNC               1
#else
#error "Invalid I2C1_SDA Pin Configuration!"
#endif

#define RTE_I2C1_PIN_INIT           I2C1_InitPins
#define RTE_I2C1_PIN_DEINIT         I2C1_DeinitPins
#define RTE_I2C1_DMA_EN             0

// I2C configuration
#define RTE_I2C1_Master_DMA_BASE    DMA0
#define RTE_I2C1_Master_DMA_CH      17
// </e>

// <e> I2C4 (Inter-integrated Circuit Interface 4) [Driver_I2C4]
// <i> Configuration settings for Driver_I2C4 in component ::Drivers:I2C
#define RTE_I2C4                        1

//   <o> I2C4_SCL Pin <0=>P0_28
#define RTE_I2C4_SCL_PORT_ID            0
#if    (RTE_I2C4_SCL_PORT_ID == 0)
#define RTE_I2C4_SCL_PORT               1
#define RTE_I2C4_SCL_PIN                20
#define RTE_I2C4_SCL_FUNC               5
#else
#error "Invalid I2C4_SCL Pin Configuration!"
#endif

//   <o> I2C4_SDA Pin <0=>P0_27
#define RTE_I2C4_SDA_PORT_ID            0
#if    (RTE_I2C4_SDA_PORT_ID == 0)
#define RTE_I2C4_SDA_PORT               1
#define RTE_I2C4_SDA_PIN                21
#define RTE_I2C4_SDA_FUNC               5
#else
#error "Invalid I2C4_SDA Pin Configuration!"
#endif

#define RTE_I2C4_PIN_INIT           I2C4_InitPins
#define RTE_I2C4_PIN_DEINIT         I2C4_DeinitPins
#define RTE_I2C4_DMA_EN             0

// I2C configuration
#define RTE_I2C4_Master_DMA_BASE    DMA0
#define RTE_I2C4_Master_DMA_CH      17
// </e>

// <e> SPI0 (Serial Peripheral Interface 0) [Driver_SPI0]
// <i> Configuration settings for Driver_SPI0 in component ::Drivers:SPI
#define RTE_SPI0                    0
// </e>

// <e> SPI1 (Serial Peripheral Interface 1) [Driver_SPI1]
// <i> Configuration settings for Driver_SPI1 in component ::Drivers:SPI
#define RTE_SPI1                    0
// </e>

// <e> SPI2 (Serial Peripheral Interface 2) [Driver_SPI2]
// <i> Configuration settings for Driver_SPI2 in component ::Drivers:SPI
#define RTE_SPI2                    0
// </e>

// <e> SPI3 (Serial Peripheral Interface 2) [Driver_SPI3]
// <i> Configuration settings for Driver_SPI3 in component ::Drivers:SPI
#define RTE_SPI3                        1

//   <h> Pin Configuration
//     <o> SPI3_SSEL <0=>Not used <1=>P0_1 <2=>P0_4 <3=>P0_5 <4=>P0_7 <5=>P0_8 <6=>P0_9 <7=>P0_20 <8=>P0_21 <9=>P1_23 <10=>P1_24
//     <i> Slave Select for SPI3
#define   RTE_SPI3_SSEL_PIN_SEL         2
#if      (RTE_SPI3_SSEL_PIN_SEL == 0)
#define   RTE_SPI3_SSEL_PIN_EN          0
#elif    (RTE_SPI3_SSEL_PIN_SEL == 1)   // SSEL0
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             1
  #define RTE_SPI3_SSEL_FUNC            2
#elif    (RTE_SPI3_SSEL_PIN_SEL == 2)   // SSEL0
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             4
  #define RTE_SPI3_SSEL_FUNC            8
#elif    (RTE_SPI3_SSEL_PIN_SEL == 3)   // SSEL1
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             5
  #define RTE_SPI3_SSEL_FUNC            8
#elif    (RTE_SPI3_SSEL_PIN_SEL == 4)   // SSEL1
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             7
  #define RTE_SPI3_SSEL_FUNC            1
#elif    (RTE_SPI3_SSEL_PIN_SEL == 5)   // SSEL3
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             8
  #define RTE_SPI3_SSEL_FUNC            1
#elif    (RTE_SPI3_SSEL_PIN_SEL == 6)   // SSEL2
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             9
  #define RTE_SPI3_SSEL_FUNC            1
#elif    (RTE_SPI3_SSEL_PIN_SEL == 7)   // SSEL0
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             20
  #define RTE_SPI3_SSEL_FUNC            1
#elif    (RTE_SPI3_SSEL_PIN_SEL == 8)   // SSEL1
  #define RTE_SPI3_SSEL_PORT            0
  #define RTE_SPI3_SSEL_BIT             21
  #define RTE_SPI3_SSEL_FUNC            1
#elif    (RTE_SPI3_SSEL_PIN_SEL == 9)   // SSEL2
  #define RTE_SPI3_SSEL_PORT            1
  #define RTE_SPI3_SSEL_BIT             23
  #define RTE_SPI3_SSEL_FUNC            5
#elif    (RTE_SPI3_SSEL_PIN_SEL == 9)   // SSEL3
  #define RTE_SPI3_SSEL_PORT            1
  #define RTE_SPI3_SSEL_BIT             24
  #define RTE_SPI3_SSEL_FUNC            5
#else
  #error "Invalid SPI3 SPI3_SSEL Pin Configuration!"
#endif
#ifndef   RTE_SPI3_SSEL_PIN_EN
#define   RTE_SPI3_SSEL_PIN_EN          1
#endif

//     <o> SPI3_SCK <0=>P0_0 <1=>P0_6
//     <i> Serial clock for SPI3
#define   RTE_SPI3_SCK_PIN_SEL          1
#if      (RTE_SPI3_SCK_PIN_SEL == 0)
  #define RTE_SPI3_SCK_PORT             0
  #define RTE_SPI3_SCK_BIT              0
  #define RTE_SPI3_SCK_FUNC             2
#elif    (RTE_SPI3_SCK_PIN_SEL == 1)
  #define RTE_SPI3_SCK_PORT             0
  #define RTE_SPI3_SCK_BIT              6
  #define RTE_SPI3_SCK_FUNC             1
#else
  #error "Invalid SPI3 SPI3_SCK Pin Configuration!"
#endif

//     <o> SPI3_MISO <0=>P0_2 <1=>P0_12
//     <i> Master In Slave Out for SPI3
#define   RTE_SPI3_MISO_PIN_SEL         0
#if      (RTE_SPI3_MISO_PIN_SEL == 0)
  #define RTE_SPI3_MISO_PORT            0
  #define RTE_SPI3_MISO_BIT             2
  #define RTE_SPI3_MISO_FUNC            1
#elif    (RTE_SPI3_MISO_PIN_SEL == 1)
  #define RTE_SPI3_MISO_PORT            0
  #define RTE_SPI3_MISO_BIT             12
  #define RTE_SPI3_MISO_FUNC            1
#else
  #error "Invalid SPI3 SPI3_MISO Pin Configuration!"
#endif

//     <o> SPI3_MOSI <0=>P0_3 <1=>P1_1
//     <i> Master Out Slave In for SPI3
#define   RTE_SPI3_MOSI_PIN_SEL         0
#if      (RTE_SPI3_MOSI_PIN_SEL == 0)
  #define RTE_SPI3_MOSI_PORT            0
  #define RTE_SPI3_MOSI_BIT             3
  #define RTE_SPI3_MOSI_FUNC            1
#elif    (RTE_SPI3_MOSI_PIN_SEL == 1)
  #define RTE_SPI3_MOSI_PORT            1
  #define RTE_SPI3_MOSI_BIT             1
  #define RTE_SPI3_MOSI_FUNC            1
#else
  #error "Invalid SPI3 SPI3_MOSI Pin Configuration!"
#endif
//   </h>
// </e>

// <e> SPI7 (Serial Peripheral Interface 7) [Driver_SPI7]
// <i> Configuration settings for Driver_SPI7 in component ::Drivers:SPI
#define RTE_SPI7                    0
// </e>

// <e> SPI8 (High Speed Serial Peripheral Interface 8) [Driver_SPI8]
// <i> Configuration settings for Driver_SPI8 in component ::Drivers:SPI
#define RTE_SPI8                        1

//   <h> Pin Configuration
//     <o> SPI8_SSEL <0=>Not used <1=>P0_20 <2=>P1_1 <3=>P1_12 <4=>P1_26
//     <i> Slave Select for SPI8
#define   RTE_SPI8_SSEL_PIN_SEL         2
#if      (RTE_SPI8_SSEL_PIN_SEL == 0)
#define   RTE_SPI8_SSEL_PIN_EN          0
#elif    (RTE_SPI8_SSEL_PIN_SEL == 1)
  #define RTE_SPI8_SSEL_PORT            0
  #define RTE_SPI8_SSEL_BIT             20
  #define RTE_SPI8_SSEL_FUNC            8
#elif    (RTE_SPI8_SSEL_PIN_SEL == 2)
  #define RTE_SPI8_SSEL_PORT            1
  #define RTE_SPI8_SSEL_BIT             1
  #define RTE_SPI8_SSEL_FUNC            5
#elif    (RTE_SPI8_SSEL_PIN_SEL == 3)
  #define RTE_SPI8_SSEL_PORT            1
  #define RTE_SPI8_SSEL_BIT             12
  #define RTE_SPI8_SSEL_FUNC            5
#elif    (RTE_SPI8_SSEL_PIN_SEL == 4)
  #define RTE_SPI8_SSEL_PORT            1
  #define RTE_SPI8_SSEL_BIT             26
  #define RTE_SPI8_SSEL_FUNC            5
#else
  #error "Invalid SPI8 SPI8_SSEL Pin Configuration!"
#endif
#ifndef   RTE_SPI8_SSEL_PIN_EN
#define   RTE_SPI8_SSEL_PIN_EN          1
#endif

//     <o> SPI8_SCK <0=>P1_2
//     <i> Serial clock for SPI8
#define   RTE_SPI8_SCK_PIN_SEL          0
#if      (RTE_SPI8_SCK_PIN_SEL == 0)
  #define RTE_SPI8_SCK_PORT             1
  #define RTE_SPI8_SCK_BIT              5
  #define RTE_SPI8_SCK_FUNC             9
#else
  #error "Invalid SPI8 SPI8_SCK Pin Configuration!"
#endif

//     <o> SPI8_MISO <0=>P1_3
//     <i> Master In Slave Out for SPI8
#define   RTE_SPI8_MISO_PIN_SEL         0
#if      (RTE_SPI8_MISO_PIN_SEL == 0)
  #define RTE_SPI8_MISO_PORT            1
  #define RTE_SPI8_MISO_BIT             3
  #define RTE_SPI8_MISO_FUNC            6
#else
  #error "Invalid SPI8 SPI8_MISO Pin Configuration!"
#endif

//     <o> SPI8_MOSI <0=>P0_26
//     <i> Master Out Slave In for SPI8
#define   RTE_SPI8_MOSI_PIN_SEL         0
#if      (RTE_SPI8_MOSI_PIN_SEL == 0)
  #define RTE_SPI8_MOSI_PORT            0
  #define RTE_SPI8_MOSI_BIT             26
  #define RTE_SPI8_MOSI_FUNC            9
#else
  #error "Invalid SPI8 SPI8_MOSI Pin Configuration!"
#endif
//   </h>
// </e>

// -------- <<< end of configuration section >>> --------------------

#define RTE_USART0_DMA_TX_DMA_BASE  DMA0
#define RTE_USART0_DMA_RX_DMA_BASE  DMA0

/* USART configuration. */
#define USART_RX_BUFFER_LEN         64
#define USART0_RX_BUFFER_ENABLE     1
#define USART1_RX_BUFFER_ENABLE     0
#define USART2_RX_BUFFER_ENABLE     0
#define USART3_RX_BUFFER_ENABLE     0
#define USART4_RX_BUFFER_ENABLE     0
#define USART5_RX_BUFFER_ENABLE     0
#define USART6_RX_BUFFER_ENABLE     0
#define USART7_RX_BUFFER_ENABLE     0

#define RTE_USART0_PIN_INIT        USART0_InitPins
#define RTE_USART0_PIN_DEINIT      USART0_DeinitPins

#if (0)
#include "board.h"

#else
#ifndef BOARD_LED_RED_GPIO
#define BOARD_LED_RED_GPIO GPIO
#endif
#define BOARD_LED_RED_GPIO_PORT 1U
#ifndef BOARD_LED_RED_GPIO_PIN
#define BOARD_LED_RED_GPIO_PIN 6U
#endif

#ifndef BOARD_LED_BLUE_GPIO
#define BOARD_LED_BLUE_GPIO GPIO
#endif
#define BOARD_LED_BLUE_GPIO_PORT 1U
#ifndef BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN 4U
#endif

#ifndef BOARD_LED_GREEN_GPIO
#define BOARD_LED_GREEN_GPIO GPIO
#endif
#define BOARD_LED_GREEN_GPIO_PORT 1U
#ifndef BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN 7U
#endif

#ifndef BOARD_SW1_GPIO
#define BOARD_SW1_GPIO GPIO
#endif
#define BOARD_SW1_GPIO_PORT 0U
#ifndef BOARD_SW1_GPIO_PIN
#define BOARD_SW1_GPIO_PIN 5U
#endif
#define BOARD_SW1_NAME        "SW1"
#define BOARD_SW1_IRQ         PIN_INT0_IRQn
#define BOARD_SW1_IRQ_HANDLER PIN_INT0_IRQHandler

#ifndef BOARD_SW2_GPIO
#define BOARD_SW2_GPIO GPIO
#endif
#define BOARD_SW2_GPIO_PORT 1U
#ifndef BOARD_SW2_GPIO_PIN
#define BOARD_SW2_GPIO_PIN 18U
#endif
#define BOARD_SW2_NAME              "SW2"
#define BOARD_SW2_IRQ               PIN_INT1_IRQn
#define BOARD_SW2_IRQ_HANDLER       PIN_INT1_IRQHandler
#define BOARD_SW2_GPIO_PININT_INDEX 1

#endif


/*----- PORT 0 Configuration */
#define RTE_GPIO_PORT0             1
#define RTE_GPIO_PORT0_SIZE_OF_MAP 0
#define RTE_GPIO_PORT0_MAPS                                                \
    {                                                                      \
    }
#define RTE_GPIO_PORT0_MAX_INTERRUPT_CONTEXTS 0

/*----- PORT 1 Configuration */
#define RTE_GPIO_PORT1             1
#define RTE_GPIO_PORT1_SIZE_OF_MAP 0
#define RTE_GPIO_PORT1_MAPS                                                \
    {                                                                      \
    }
#define RTE_GPIO_PORT1_MAX_INTERRUPT_CONTEXTS 1


#endif /* _RTE_DEVICE_H */
