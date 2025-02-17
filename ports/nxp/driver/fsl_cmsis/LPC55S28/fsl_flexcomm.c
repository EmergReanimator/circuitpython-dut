/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexcomm.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.flexcomm"
#endif

/*!
 * @brief Used for conversion between `void*` and `uint32_t`.
 */
typedef union pvoid_to_u32
{
    void *pvoid;
    uint32_t u32;
} pvoid_to_u32_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*! @brief Set the FLEXCOMM mode . */
static status_t FLEXCOMM_SetPeriph(FLEXCOMM_Type *base, FLEXCOMM_PERIPH_T periph, int lock);

/*! @brief check whether flexcomm supports peripheral type */
static bool FLEXCOMM_PeripheralIsPresent(FLEXCOMM_Type *base, FLEXCOMM_PERIPH_T periph);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Pointers to real IRQ handlers installed by drivers for each instance. */
static flexcomm_irq_handler_t s_flexcommIrqHandler[FSL_FEATURE_SOC_FLEXCOMM_COUNT];

/*! @brief Pointers to handles for each instance to provide context to interrupt routines */
static void *s_flexcommHandle[FSL_FEATURE_SOC_FLEXCOMM_COUNT];

/*! @brief Array to map FLEXCOMM instance number to IRQ number. */
IRQn_Type const kFlexcommIrqs[] = FLEXCOMM_IRQS;

/*! @brief Array to map FLEXCOMM instance number to base address. */
static const uint32_t s_flexcommBaseAddrs[FSL_FEATURE_SOC_FLEXCOMM_COUNT] = FLEXCOMM_BASE_ADDRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief IDs of clock for each FLEXCOMM module */
static const clock_ip_name_t s_flexcommClocks[] = FLEXCOMM_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_FEATURE_FLEXCOMM_HAS_NO_RESET) && FSL_FEATURE_FLEXCOMM_HAS_NO_RESET)
/*! @brief Pointers to FLEXCOMM resets for each instance. */
static const reset_ip_name_t s_flexcommResets[] = FLEXCOMM_RSTS;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/* check whether flexcomm supports peripheral type */
static bool FLEXCOMM_PeripheralIsPresent(FLEXCOMM_Type *base, FLEXCOMM_PERIPH_T periph) {
    if (periph == FLEXCOMM_PERIPH_NONE) {
        return true;
    } else if (periph <= FLEXCOMM_PERIPH_I2S_TX) {
        return (base->PSELID & (1UL << ((uint32_t)periph + 3U))) > 0UL ? true : false;
    } else if (periph == FLEXCOMM_PERIPH_I2S_RX) {
        return (base->PSELID & (1U << 7U)) > (uint32_t)0U ? true : false;
    } else {
        return false;
    }
}

/* Get the index corresponding to the FLEXCOMM */
/*! brief Returns instance number for FLEXCOMM module with given base address. */
uint32_t FLEXCOMM_GetInstance(void *base) {
    uint32_t i;
    pvoid_to_u32_t BaseAddr;
    BaseAddr.pvoid = base;

    for (i = 0U; i < (uint32_t)FSL_FEATURE_SOC_FLEXCOMM_COUNT; i++)
    {
        if (BaseAddr.u32 == s_flexcommBaseAddrs[i]) {
            break;
        }
    }

    assert(i < (uint32_t)FSL_FEATURE_SOC_FLEXCOMM_COUNT);
    return i;
}

/* Changes FLEXCOMM mode */
static status_t FLEXCOMM_SetPeriph(FLEXCOMM_Type *base, FLEXCOMM_PERIPH_T periph, int lock) {
    /* Check whether peripheral type is present */
    if (!FLEXCOMM_PeripheralIsPresent(base, periph)) {
        return kStatus_OutOfRange;
    }

    /* Flexcomm is locked to different peripheral type than expected  */
    if (((base->PSELID & FLEXCOMM_PSELID_LOCK_MASK) != 0U) &&
        ((base->PSELID & FLEXCOMM_PSELID_PERSEL_MASK) != (uint32_t)periph)) {
        return kStatus_Fail;
    }

    /* Check if we are asked to lock */
    if (lock != 0) {
        base->PSELID = (uint32_t)periph | FLEXCOMM_PSELID_LOCK_MASK;
    } else {
        base->PSELID = (uint32_t)periph;
    }

    return kStatus_Success;
}

/*! brief Initializes FLEXCOMM and selects peripheral mode according to the second parameter. */
status_t FLEXCOMM_Init(void *base, FLEXCOMM_PERIPH_T periph) {
    uint32_t idx = FLEXCOMM_GetInstance(base);

    #if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the peripheral clock */
    CLOCK_EnableClock(s_flexcommClocks[idx]);
    #endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    #if !(defined(FSL_FEATURE_FLEXCOMM_HAS_NO_RESET) && FSL_FEATURE_FLEXCOMM_HAS_NO_RESET)
    /* Reset the FLEXCOMM module */
    RESET_PeripheralReset(s_flexcommResets[idx]);
    #endif

    /* Set the FLEXCOMM to given peripheral */
    return FLEXCOMM_SetPeriph((FLEXCOMM_Type *)base, periph, 0);
}

/*! brief Sets IRQ handler for given FLEXCOMM module. It is used by drivers register IRQ handler according to FLEXCOMM
 * mode */
void FLEXCOMM_SetIRQHandler(void *base, flexcomm_irq_handler_t handler, void *flexcommHandle) {
    uint32_t instance;

    /* Look up instance number */
    instance = FLEXCOMM_GetInstance(base);

    /* Clear handler first to avoid execution of the handler with wrong handle */
    s_flexcommIrqHandler[instance] = NULL;
    s_flexcommHandle[instance] = flexcommHandle;
    s_flexcommIrqHandler[instance] = handler;
    SDK_ISR_EXIT_BARRIER;
}

/* IRQ handler functions overloading weak symbols in the startup */
#if defined(FLEXCOMM0)
void FLEXCOMM0_DriverIRQHandler(void);
void FLEXCOMM0_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[0]);
    s_flexcommIrqHandler[0]((uint32_t *)s_flexcommBaseAddrs[0], s_flexcommHandle[0]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM1)
void FLEXCOMM1_DriverIRQHandler(void);
void FLEXCOMM1_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[1]);
    s_flexcommIrqHandler[1]((uint32_t *)s_flexcommBaseAddrs[1], s_flexcommHandle[1]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM2)
void FLEXCOMM2_DriverIRQHandler(void);
void FLEXCOMM2_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[2]);
    s_flexcommIrqHandler[2]((uint32_t *)s_flexcommBaseAddrs[2], s_flexcommHandle[2]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM3)
void FLEXCOMM3_DriverIRQHandler(void);
void FLEXCOMM3_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[3]);
    s_flexcommIrqHandler[3]((uint32_t *)s_flexcommBaseAddrs[3], s_flexcommHandle[3]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM4)
void FLEXCOMM4_DriverIRQHandler(void);
void FLEXCOMM4_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[4]);
    s_flexcommIrqHandler[4]((uint32_t *)s_flexcommBaseAddrs[4], s_flexcommHandle[4]);
    SDK_ISR_EXIT_BARRIER;
}

#endif

#if defined(FLEXCOMM5)
void FLEXCOMM5_DriverIRQHandler(void);
void FLEXCOMM5_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[5]);
    s_flexcommIrqHandler[5]((uint32_t *)s_flexcommBaseAddrs[5], s_flexcommHandle[5]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM6)
void FLEXCOMM6_DriverIRQHandler(void);
void FLEXCOMM6_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[6]);
    s_flexcommIrqHandler[6]((uint32_t *)s_flexcommBaseAddrs[6], s_flexcommHandle[6]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM7)
void FLEXCOMM7_DriverIRQHandler(void);
void FLEXCOMM7_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[7]);
    s_flexcommIrqHandler[7]((uint32_t *)s_flexcommBaseAddrs[7], s_flexcommHandle[7]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM8)
void FLEXCOMM8_DriverIRQHandler(void);
void FLEXCOMM8_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[8]);
    s_flexcommIrqHandler[8]((uint32_t *)s_flexcommBaseAddrs[8], s_flexcommHandle[8]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM9)
void FLEXCOMM9_DriverIRQHandler(void);
void FLEXCOMM9_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[9]);
    s_flexcommIrqHandler[9]((uint32_t *)s_flexcommBaseAddrs[9], s_flexcommHandle[9]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM10)
void FLEXCOMM10_DriverIRQHandler(void);
void FLEXCOMM10_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[10]);
    s_flexcommIrqHandler[10]((uint32_t *)s_flexcommBaseAddrs[10], s_flexcommHandle[10]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM11)
void FLEXCOMM11_DriverIRQHandler(void);
void FLEXCOMM11_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[11]);
    s_flexcommIrqHandler[11]((uint32_t *)s_flexcommBaseAddrs[11], s_flexcommHandle[11]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM12)
void FLEXCOMM12_DriverIRQHandler(void);
void FLEXCOMM12_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[12]);
    s_flexcommIrqHandler[12]((uint32_t *)s_flexcommBaseAddrs[12], s_flexcommHandle[12]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM13)
void FLEXCOMM13_DriverIRQHandler(void);
void FLEXCOMM13_DriverIRQHandler(void) {
    assert(s_flexcommIrqHandler[13]);
    s_flexcommIrqHandler[13]((uint32_t *)s_flexcommBaseAddrs[13], s_flexcommHandle[13]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM14)
void FLEXCOMM14_DriverIRQHandler(void);
void FLEXCOMM14_DriverIRQHandler(void) {
    uint32_t instance;

    /* Look up instance number */
    instance = FLEXCOMM_GetInstance(FLEXCOMM14);
    assert(s_flexcommIrqHandler[instance]);
    s_flexcommIrqHandler[instance]((uint32_t *)s_flexcommBaseAddrs[instance], s_flexcommHandle[instance]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM15)
void FLEXCOMM15_DriverIRQHandler(void);
void FLEXCOMM15_DriverIRQHandler(void) {
    uint32_t instance;

    /* Look up instance number */
    instance = FLEXCOMM_GetInstance(FLEXCOMM15);
    assert(s_flexcommIrqHandler[instance]);
    s_flexcommIrqHandler[instance]((uint32_t *)s_flexcommBaseAddrs[instance], s_flexcommHandle[instance]);
    SDK_ISR_EXIT_BARRIER;
}
#endif

#if defined(FLEXCOMM16)
void FLEXCOMM16_DriverIRQHandler(void);
void FLEXCOMM16_DriverIRQHandler(void) {
    uint32_t instance;

    /* Look up instance number */
    instance = FLEXCOMM_GetInstance(FLEXCOMM16);
    assert(s_flexcommIrqHandler[instance]);
    s_flexcommIrqHandler[instance]((uint32_t *)s_flexcommBaseAddrs[instance], s_flexcommHandle[instance]);
    SDK_ISR_EXIT_BARRIER;
}
#endif
