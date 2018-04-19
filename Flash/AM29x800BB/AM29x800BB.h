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
 * $Date:        19. April 2018
 * $Revision:    V1.4
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Description for AM29x800BB (16-bit Bus)
 * -------------------------------------------------------------------- */

#define FLASH_SECTOR_COUNT      ((uint32_t)22)  /* Number of sectors */
#define FLASH_SECTOR_SIZE       ((uint32_t)0)   /* FLASH_SECTORS information used */
#define FLASH_PAGE_SIZE         ((uint32_t)2)   /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)2)   /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF) /* Contents of erased memory */

#define FLASH_SECTORS {                                             \
  ARM_FLASH_SECTOR_INFO(0x000000UL, 0x04000UL), /* Sector size 16kB */ \
  ARM_FLASH_SECTOR_INFO(0x004000UL, 0x08000UL), /* Sector size 32kB */ \
  ARM_FLASH_SECTOR_INFO(0x00C000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x00E000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x010000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x012000UL, 0x02000UL), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x014000UL, 0x08000UL), /* Sector size 32kB */ \
  ARM_FLASH_SECTOR_INFO(0x01C000UL, 0x04000UL), /* Sector size 16kB */ \
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
  ARM_FLASH_SECTOR_INFO(0x0F0000UL, 0x10000UL)  /* Sector size 64kB */ \
}
