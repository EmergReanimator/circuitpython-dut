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

#include "py/mperrno.h"
#include "driver/iap.h"
#include "mcux-sdk/drivers/iap1/fsl_iap_ffr.h"
#include "mcux-sdk/drivers/iap1/fsl_iap.h"

#include "device.h"

STATIC bool __initialised = false;
STATIC flash_config_t __flash_config;

int NXP_IAP_Init(void) {
    int ret = 0;

    if (!__initialised) {
        status_t status = FFR_Init(&__flash_config);
        ret = (status == kStatus_Success) ? 0 : MP_EIO;
        __initialised = true;
    }

    return ret;
}

int NXP_IAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector) {
    #if (1)
    return MP_EIO;
    #else
    uint8_t result = Chip_IAP_PreSectorForReadWrite(startSector, endSector);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock) {
    #if (1)
    return MP_EIO;
    #else
    (void)systemCoreClock;

    uint8_t result = Chip_IAP_ErasePage(startPage, endPage);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock) {
    #if (1)
    return MP_EIO;
    #else
    (void)systemCoreClock;

    uint8_t result = Chip_IAP_EraseSector(startSector, endSector);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_BlankCheckSector(uint32_t startSector, uint32_t endSector) {
    #if (1)
    return MP_EIO;
    #else
    uint8_t result = Chip_IAP_BlankCheckSector(startSector, endSector);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock) {
    #if (1)
    return MP_EIO;
    #else
    (void)systemCoreClock;

    uint8_t result = Chip_IAP_CopyRamToFlash(dstAddr, srcAddr, numOfBytes);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes) {
    #if (1)
    return MP_EIO;
    #else
    uint8_t result = Chip_IAP_Compare(dstAddr, (uint32_t)srcAddr, numOfBytes);

    return (IAP_CMD_SUCCESS == result) ? 0 : MP_EIO;
    #endif
}

int NXP_IAP_ReadUniqueID(uint8_t *uniqueID, uint32_t numOfBytes) {
    assert((__initialised));
    assert((NULL != uniqueID));
    size_t count = MIN(128U, numOfBytes);

    status_t status = FFR_GetUUID(&__flash_config, uniqueID);

    return (status == kStatus_Success) ? 0 : MP_EIO;
}
