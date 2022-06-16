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

#if !defined(PORTS_NXP_DRIVER_IAP_H_)
#define PORTS_NXP_DRIVER_IAP_H_

int NXP_IAP_Init(void);
int NXP_IAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector);
int NXP_IAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock);
int NXP_IAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock);
int NXP_IAP_BlankCheckSector(uint32_t startSector, uint32_t endSector);
int NXP_IAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock);
int NXP_IAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes);
int NXP_IAP_ReadUniqueID(uint8_t *uniqueID, uint32_t numOfBytes);

#endif // PORTS_NXP_DRIVER_IAP_H_
