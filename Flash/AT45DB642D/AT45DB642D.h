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
 * $Revision:    V1.3
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Description for Atmel DataFlash AT45DB642D (SPI)
 * -------------------------------------------------------------------- */

#define FLASH_SECTOR_COUNT      ((uint32_t)64)   /* Number of Sectors */
#define FLASH_SECTOR_SIZE       ((uint32_t)0)    /* FLASH_SECTORS information used */
#define FLASH_PAGE_SIZE         ((uint32_t)1056) /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)1)    /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF)  /* Contents of erased memory */

#define FLASH_SECTORS {                                                 \
  ARM_FLASH_SECTOR_INFO(0x000000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x021000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x042000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x063000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x084000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x0A5000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x0C6000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x0E7000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x108000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x129000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x14A000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x16B000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x18C000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x1AD000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x1CE000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x1EF000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x210000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x231000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x252000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x273000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x294000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x2B5000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x2D6000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x2F7000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x318000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x339000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x35A000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x37B000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x39C000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x3BD000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x3DE000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x3FF000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x420000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x441000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x462000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x483000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x4A4000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x4C5000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x4E6000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x507000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x528000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x549000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x56A000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x58B000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x5AC000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x5CD000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x5EE000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x60F000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x630000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x651000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x672000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x693000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x6B4000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x6D5000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x6F6000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x717000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x738000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x759000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x77A000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x79B000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x7BC000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x7DD000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x7FE000UL, 0x21000UL), /* Sector size 132kB */ \
  ARM_FLASH_SECTOR_INFO(0x81F000UL, 0x21000UL)  /* Sector size 132kB */ \
}
