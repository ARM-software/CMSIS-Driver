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
 * Project:      Flash Device Description for M29EW 128Mb (16-bit)
 *               Part Number: JS28F128M29EW, PC28F128M29EW, RC28F128M29EW
 * -------------------------------------------------------------------- */

#define FLASH_SECTOR_COUNT      ((uint32_t)128)    /* Number of sectors */
#define FLASH_SECTOR_SIZE       ((uint32_t)131072) /* Uniform sectors of 128kB */
#define FLASH_PAGE_SIZE         ((uint32_t)2)      /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)2)      /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF)    /* Contents of erased memory */
