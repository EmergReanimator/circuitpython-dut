/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * SPDX-FileCopyrightText: Copyright (c) 2013, 2014 Damien P. George
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

#include "supervisor/internal_flash.h"

#include <stdint.h>
#include <string.h>

#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mperrno.h"    // MP_E*
#include "lib/oofatfs/ff.h"

#include "device.h"
#include "driver/iap.h"
#include "supervisor/flash.h"
#include "supervisor/shared/safe_mode.h"

#if !(defined(DISABLE_FILESYSTEM) && (DISABLE_FILESYSTEM == 1))

#define COMPILER_SECTION(a) __attribute__((__section__(a)))

typedef struct {
    uint32_t base_address;
    uint16_t sector_size;
    uint16_t sector_count;
} flash_layout_t;

#define INVALID_SECTOR_INDEX    UINT32_MAX
#define EMPTY_CACHE            UINT32_MAX

/*------------------------------------------------------------------*/
/* Internal Flash API
 *------------------------------------------------------------------*/
STATIC const flash_layout_t port_flash_layout[] = {
    { .base_address = 0x00001000U, .sector_size = 0x1000U, .sector_count = 15U },
    { .base_address = 0x00010000U, .sector_size = 0x8000U, .sector_count = 16U },
};


typedef uint8_t __attribute__((aligned(256U))) * port_flash_page_ptr_t;

STATIC COMPILER_SECTION(".bss.cache") uint8_t port_flash_cache[0x1000U] __attribute__((aligned(256U)));

STATIC uint32_t __flash_cache_addr = EMPTY_CACHE;

STATIC uint32_t __flash_get_sector_info(uint32_t addr, uint32_t *start_addr, uint32_t *size) {
    const uint32_t base_address = port_flash_layout[0].base_address;

    if (addr >= base_address) {
        const uint32_t sector_size = port_flash_layout[0].sector_size;
        uint32_t sector_index = base_address / sector_size;
        if (MP_ARRAY_SIZE(port_flash_layout) == 1) {
            sector_index = (addr - base_address) / sector_size;
            const uint32_t sector_count = port_flash_layout[0].sector_count;
            if (sector_index >= sector_count) {
                return INVALID_SECTOR_INDEX;
            }
            if (start_addr) {
                *start_addr = base_address + (sector_index * sector_size);
            }
            if (size) {
                *size = sector_size;
            }
            return sector_index;
        }

        for (uint8_t i = 0; i < MP_ARRAY_SIZE(port_flash_layout); ++i) {
            for (uint8_t j = 0; j < port_flash_layout[i].sector_count; ++j) {
                uint32_t sector_start_next = port_flash_layout[i].base_address
                    + (j + 1) * port_flash_layout[i].sector_size;
                if (addr < sector_start_next) {
                    if (start_addr != NULL) {
                        *start_addr = port_flash_layout[i].base_address
                            + j * port_flash_layout[i].sector_size;
                    }
                    if (size != NULL) {
                        *size = port_flash_layout[i].sector_size;
                    }
                    return sector_index;
                }
                ++sector_index;
            }
        }
    }

    return INVALID_SECTOR_INDEX;
}

STATIC int32_t __convert_block_to_flash_addr(uint32_t block) {
    if (0 <= block && block < INTERNAL_FLASH_PART1_NUM_BLOCKS) {
        // a block in partition 1
        return CIRCUITPY_INTERNAL_FLASH_FILESYSTEM_START_ADDR + block * FILESYSTEM_BLOCK_SIZE;
    }
    // bad block
    return -MP_ERANGE;
}

STATIC mp_uint_t __flash_erase_verify_sectors(uint32_t start_index, uint32_t end_index) {
    mp_uint_t ret = MP_EIO;

    common_hal_mcu_disable_interrupts();
    int rc = NXP_IAP_PrepareSectorForWrite(start_index, end_index);
    if (!rc) {
        rc = NXP_IAP_EraseSector(start_index, end_index, SystemCoreClock);
    }
    common_hal_mcu_enable_interrupts();

    if (!rc) {
        rc = NXP_IAP_BlankCheckSector(start_index, end_index);
    }
    ret = -rc;

    return ret;
}

STATIC mp_uint_t __flash_write_verfiy_sectors(uint32_t start_index, uint32_t end_index, uint32_t start_addr, size_t num_bytes, port_flash_page_ptr_t *src) {
    mp_uint_t ret = MP_EIO;

    common_hal_mcu_disable_interrupts();
    int rc = NXP_IAP_PrepareSectorForWrite(start_index, end_index);
    if (!rc) {
        rc = NXP_IAP_CopyRamToFlash(start_addr, (uint32_t *)src, num_bytes, SystemCoreClock);
    }
    common_hal_mcu_enable_interrupts();

    if (!rc) {
        rc = NXP_IAP_Compare(start_addr, (uint32_t *)src, num_bytes);
    }
    ret = -rc;

    return ret;
}

void supervisor_flash_init(void) {
    return;
}

uint32_t supervisor_flash_get_block_size(void) {
    return FILESYSTEM_BLOCK_SIZE;
}

uint32_t supervisor_flash_get_block_count(void) {
    return INTERNAL_FLASH_PART1_NUM_BLOCKS;
}

void port_internal_flash_flush(void) {
    if (EMPTY_CACHE != __flash_cache_addr) {
        // get the sector information
        uint32_t sector_size;
        uint32_t sector_start_addr = UINT32_MAX;
        int32_t start_index = __flash_get_sector_info(__flash_cache_addr, &sector_start_addr, &sector_size);

        if (sector_size <= sizeof(port_flash_cache) && (UINT32_MAX != sector_start_addr)) {
            // Skip if data is the same
            if (memcmp(&port_flash_cache[0U], (void *)__flash_cache_addr, sector_size) != 0) {

                const uint32_t end_index = start_index + 1U;
                mp_uint_t ret = __flash_erase_verify_sectors(start_index, end_index);
                if (!ret) {
                    port_flash_page_ptr_t *cache_addr = (port_flash_page_ptr_t *)&port_flash_cache[0];
                    ret = __flash_write_verfiy_sectors(start_index, end_index, sector_start_addr, sector_size, cache_addr);

                    if (ret) {
                        reset_into_safe_mode(FLASH_WRITE_FAIL);
                    }
                } else {
                    reset_into_safe_mode(FLASH_WRITE_FAIL);
                }
            }
        } else {
            reset_into_safe_mode(FLASH_WRITE_FAIL);
        }
    }

    return;
}

void supervisor_flash_release_cache(void) {
    __flash_cache_addr = EMPTY_CACHE;
    return;
}

mp_uint_t supervisor_flash_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    mp_uint_t ret = MP_EIO;

    int32_t start_addr = __convert_block_to_flash_addr(block_num);

    if (start_addr > 0) {
        // Determine whether the read is contained within the sector
        uint32_t sector_size;
        uint32_t sector_start_addr;
        mp_uint_t start_index = __flash_get_sector_info(start_addr, &sector_start_addr, &sector_size);

        if (INVALID_SECTOR_INDEX != start_index) {
            // Count how many blocks are left in the sector
            uint32_t count = (sector_size - (start_addr - sector_start_addr)) / FILESYSTEM_BLOCK_SIZE;
            count = MIN(num_blocks, count);

            const size_t data_size = FILESYSTEM_BLOCK_SIZE * num_blocks;
            if (count < num_blocks && __flash_cache_addr == sector_start_addr) {
                // Read is contained in the cache, so just read cache
                memcpy(dest, (port_flash_cache + (start_addr - sector_start_addr)), data_size);
            } else {
                // The read spans multiple sectors or is in another sector
                // Must write out anything in cache before trying to read.
                supervisor_flash_flush();
                memcpy(dest, (uint8_t *)start_addr, data_size);
            }

            // success
            ret = 0;
        }
    }

    return ret;
}

mp_uint_t supervisor_flash_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks) {
    mp_uint_t ret = MP_EINVAL;

    while (num_blocks) {
        int32_t start_addr = __convert_block_to_flash_addr(block_num);

        if (start_addr > 0) {
            // Determine whether the read is contained within the sector
            uint32_t sector_size;
            uint32_t sector_start_addr;
            int start_index = __flash_get_sector_info(start_addr, &sector_start_addr, &sector_size);

            if (INVALID_SECTOR_INDEX != start_index) {
                #if (1)
                assert((sector_size <= sizeof(port_flash_cache)));
                #else
                reset_into_safe_mode(FLASH_WRITE_FAIL);
                #endif

                // Find how many blocks are left in the sector
                uint32_t block_size = sector_size - (start_addr - sector_start_addr);
                uint32_t count = block_size / FILESYSTEM_BLOCK_SIZE;
                count = MIN(num_blocks, count);
                block_size = count * FILESYSTEM_BLOCK_SIZE;

                if (__flash_cache_addr != sector_start_addr) {
                    // Write out anything in cache before overwriting it.
                    supervisor_flash_flush();

                    __flash_cache_addr = sector_start_addr;

                    // Copy the current contents of the entire sector into the flash cache.
                    memcpy(&port_flash_cache[0U], (void *)sector_start_addr, sector_size);
                }

                // Overwrite part or all of the flash cache with the src data.
                memcpy(&port_flash_cache[0U] + (start_addr - sector_start_addr), src, block_size);

                // adjust for next run
                block_num += count;
                src += block_size;
                num_blocks -= count;
            } else {
                num_blocks = 0U;
                ret = MP_ERANGE;
            }
        } else {
            num_blocks = 0U;
        }
    }

    return ret;
}
#endif
