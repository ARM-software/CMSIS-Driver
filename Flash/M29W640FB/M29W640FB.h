/*
 * Copyright (c) 2013-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * -----------------------------------------------------------------------
 *
 * $Date:        18. April 2018
 * $Revision:    V1.4
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Description for M29W640FB (16-bit Bus)
 * -------------------------------------------------------------------- */

#define FLASH_SECTOR_COUNT      ((uint32_t)135) /* Number of sectors */
#define FLASH_SECTOR_SIZE       ((uint32_t)0)   /* FLASH_SECTORS information used */
#define FLASH_PAGE_SIZE         ((uint32_t)2)   /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)2)   /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF) /* Contents of erased memory */

#define FLASH_SECTORS {                                            \
  ARM_FLASH_SECTOR_INFO(0x000000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x002000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x004000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x006000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x008000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x00A000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x00C000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x00E000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x010000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x020000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x030000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x040000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x050000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x060000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x070000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x080000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x090000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x0F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x100000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x110000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x120000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x130000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x140000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x150000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x160000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x170000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x180000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x190000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x1F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x200000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x210000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x220000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x230000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x240000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x250000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x260000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x270000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x280000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x290000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x2F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x300000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x310000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x320000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x330000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x340000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x350000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x360000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x370000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x380000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x390000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x3F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x400000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x410000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x420000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x430000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x440000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x450000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x460000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x470000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x480000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x490000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x4F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x500000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x510000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x520000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x530000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x540000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x550000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x560000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x570000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x580000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x590000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x5F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x600000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x610000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x620000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x630000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x640000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x650000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x660000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x670000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x680000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x690000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x6F0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x700000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x710000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x720000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x730000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x740000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x750000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x760000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x770000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x780000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x790000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7A0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7B0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7C0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7D0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7E0000UL, 0x10000UL), /* Sector size 64kB */ \
  ARM_FLASH_SECTOR_INFO(0x7F0000UL, 0x10000UL)  /* Sector size 64kB */ \
}
