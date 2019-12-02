/* -----------------------------------------------------------------------------
 * Copyright (c) 2019 Arm Limited (or its affiliates). All rights reserved.
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
 *
 * $Date:        4. October 2019
 * $Revision:    V1.0
 *
 * Project:      Memory management for Inventek ISM43362 WiFi Driver
 *               header file
 * -------------------------------------------------------------------------- */

#ifndef __WIFI_ISM43362_MEM_H__
#define __WIFI_ISM43362_MEM_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


typedef struct WIFI_ISM43362_MEM_t {    // Memory pool management structure
  struct WIFI_ISM43362_MEM_t *next;     // Next memory block in the pool
  uint32_t                    len;      // Length of data in the block
  uint8_t                     data[];   // Memory block data
} WIFI_ISM43362_MEM_t;


/// \brief       Initialize memory pool
extern void WiFi_ISM43362_MemoryInitialize (void);

/// \brief       Uninitialize memory pool
extern void WiFi_ISM43362_MemoryUninitialize (void);

/// \brief       Get memory pool size
/// \return      size of memory pool in bytes
extern uint32_t WiFi_ISM43362_MemoryGetPoolSize (void);

/// \brief       Allocate a piece of memory (block) of required size from the memory pool
/// \param[in]   sz       Size of memory to be allocated from the memory pool
/// \return               Pointer to allocated memory or NULL in case not enough memory is available
/// \return        value != 0:   pointer to allocated memory
/// \return        value = NULL: not enough memory is available
extern void *WiFi_ISM43362_MemoryAllocate (uint32_t sz);

/// \brief       Free memory and return it to the memory pool
/// \param[in]   ptr_mem  Pointer to allocated memory to be freed
/// \return      true     freeing succeeded
/// \return      false    freeing failed
extern bool WiFi_ISM43362_MemoryFree (const void *ptr_mem);

#endif  // __WIFI_ISM43362_MEM_H__
