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
 *
 * --------------------------------------------------------------------------
 * Important note:
 * Size of memory pool must be at least =
 * (maximum number of elements * 8) + (sum of all element data) + 4
 * Example: two 8 byte packets => (2*8)+(8+8)+4 = 36
 * -------------------------------------------------------------------------- */

#include "WiFi_ISM43362_Mem.h"

#include "WiFi_ISM43362_Config.h"       // Driver configuration settings


// Short names for easier local usage
#define  MEM_t                          WIFI_ISM43362_MEM_t

// Local variables
static uint32_t mem_pool[(WIFI_ISM43362_MEM_POOL_SIZE + 3) / 4];


/// \brief       Initialize memory pool
void WiFi_ISM43362_MemoryInitialize (void) {
  MEM_t *ptr_first;

  memset ((void *)mem_pool, 0, sizeof(mem_pool));

  ptr_first             = (MEM_t *)mem_pool;
  ptr_first->next       = (MEM_t *)((uint32_t)mem_pool + sizeof(mem_pool) - sizeof(struct MEM_t *));
  ptr_first->next->next = NULL;
  ptr_first->len        = 0U;
}

/// \brief       Uninitialize memory pool
void WiFi_ISM43362_MemoryUninitialize (void) {

  memset ((void *)mem_pool, 0, sizeof(mem_pool));
}

/// \brief       Get memory pool size
/// \return      size of memory pool in bytes
uint32_t WiFi_ISM43362_MemoryGetPoolSize (void) {

  return sizeof(mem_pool);
}

/// \brief       Allocate a piece of memory (block) of required size from the memory pool
/// \param[in]   sz       Size of memory to be allocated from the memory pool
/// \return               Pointer to allocated memory or NULL in case not enough memory is available
/// \return        value != 0:   pointer to allocated memory
/// \return        value = NULL: not enough memory is available
void *WiFi_ISM43362_MemoryAllocate (uint32_t sz) {
  MEM_t    *ptr_curr, *ptr_new;
  void     *ptr_data;
  uint32_t  total_sz;
  uint32_t  hole_sz;

  // Add the memory handling overhead to 'sz'
  // Overhead is pointer to next (4 bytes) and length of data (4 bytes) = 8 bytes per block
  total_sz = sz + sizeof(struct MEM_t *) + sizeof(uint32_t);
  // Make sure that block is 4-byte aligned
  total_sz = (total_sz + 3U) & ~0x00000003U;

  ptr_curr = (MEM_t *)mem_pool;
  for (;;) {
    hole_sz  = (uint32_t)(ptr_curr->next) - (uint32_t)ptr_curr;
    hole_sz -= ptr_curr->len;
    if (hole_sz >= total_sz) {
      // If the hole size is big enough stop the search as we found place for new block
      break;
    }
    ptr_curr = ptr_curr->next;
    if (ptr_curr->next == NULL) {
      // Failed, we are at the end of the list and there was not enough place for new block
      return NULL;
    }
  }

  if (ptr_curr->len == 0U) {
    // No block is allocated, set the Length of the first element
    ptr_curr->len  = total_sz;
    ptr_data       = (void *)ptr_curr->data;
  } else {
    // Insert a new list element into the memory list
    ptr_new        = (MEM_t *)((uint32_t)ptr_curr + ptr_curr->len);
    ptr_new->next  = ptr_curr->next;
    ptr_curr->next = ptr_new;
    ptr_new->len   = total_sz;
    ptr_data       = (void *)ptr_new->data;
  }

  return ptr_data;
}

/// \brief       Free memory and return it to the memory pool
/// \param[in]   ptr_mem  Pointer to allocated memory to be freed
/// \return      true     freeing succeeded
/// \return      false    freeing failed
bool WiFi_ISM43362_MemoryFree (const void *ptr_mem) {
  const MEM_t *ptr_to_free;
        MEM_t *ptr_prev, *ptr_curr;

  if (ptr_mem == NULL) {
    return false;
  }

  ptr_to_free = (MEM_t *)((uint32_t)ptr_mem - (sizeof(struct MEM_t *) + sizeof(uint32_t)));

  // Find block previous to one being freed
  ptr_curr = (MEM_t *)mem_pool;
  ptr_prev = ptr_curr;
  while (ptr_curr != ptr_to_free) {
    ptr_prev = ptr_curr;
    ptr_curr = ptr_curr->next;
    if (ptr_curr == NULL) {
      // If this is not a valid memory block pointer
      return false;
    }
    if ((((uint32_t)(ptr_curr->next)) & 3U) != 0U) {
      // If next pointer was overwritten or memory is corrupted
      return false;
    }
  }
  if (ptr_curr == (MEM_t *)mem_pool) {
    // First element to be set free, only set Length to 0
    ptr_curr->len  = 0U;
  } else {
    // Discard list element
    ptr_prev->next = ptr_curr->next;
  }

  return true;
}
