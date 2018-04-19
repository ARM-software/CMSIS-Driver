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
 * $Revision:    V1.0
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Description - Template
 * -------------------------------------------------------------------- */

#define FLASH_SECTOR_COUNT      4           /* Number of sectors */
#define FLASH_SECTOR_SIZE       0           /* FLASH_SECTORS information used */
#define FLASH_PAGE_SIZE         1           /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      1           /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      0xFF        /* Contents of erased memory */

#define FLASH_SECTORS                                              \
  ARM_FLASH_SECTOR_INFO(0x000000, 0x08000), /* Sector size 32kB */ \
  ARM_FLASH_SECTOR_INFO(0x008000, 0x04000), /* Sector size 16kB */ \
  ARM_FLASH_SECTOR_INFO(0x00C000, 0x02000), /* Sector size  8kB */ \
  ARM_FLASH_SECTOR_INFO(0x00E000, 0x02000)  /* Sector size  8kB */
