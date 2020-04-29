/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2020 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        22. January 2020
 * $Revision:    V1.1
 *
 * Project:      Variable size data block buffer 
 *               for Inventek ISM43362 WiFi Driver
 * -------------------------------------------------------------------------- */

#include "WiFi_ISM43362_Buf.h"

#include "WiFi_ISM43362_Config.h"       // Driver configuration settings
#include "WiFi_ISM43362_Mem.h"


// Short names for easier local usage
#define  MEM_t                          WIFI_ISM43362_MEM_t
#define  BUF_t                          WIFI_ISM43362_BUF_t
#define  BUF_ENT_t                      WIFI_ISM43362_BUF_ENT_t

// Local variables
static bool  mem_init = false;
static BUF_t buf_mgmt[WIFI_ISM43362_SOCKETS_NUM];


/// \brief       Initialize buffer
/// \param[in]   idx      Index of buffer to initialize (0 ..)
/// \return      true     Buffer initialized successfully
/// \return      false    Buffer initialization failed
bool WiFi_ISM43362_BufferInitialize (uint8_t idx) {

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return false;
  }

  if (mem_init == false) {
    WiFi_ISM43362_MemoryInitialize();
    mem_init = true;
  }

  memset ((void *)&buf_mgmt[idx], 0, sizeof(BUF_t));
  buf_mgmt[idx].init = 1U;

  return true;
}

/// \brief       Uninitialize buffer
/// \param[in]   idx      Index of buffer to uninitialize (0 ..)
/// \return      true     Buffer uninitialized successfully
/// \return      false    Buffer uninitialization failed
bool WiFi_ISM43362_BufferUninitialize (uint8_t idx) {
  uint32_t i;

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return false;
  }

  if (WiFi_ISM43362_BufferFlush (idx) == false) {
    return false;
  }

  memset ((void *)&buf_mgmt[idx], 0, sizeof(BUF_t));

  for (i = 0U; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
    if (buf_mgmt[i].init == 1U) {
      break;
    }
  }
  if (i == WIFI_ISM43362_SOCKETS_NUM) {
    // If all buffers were uninitialized, uninitialize the memory pool also
    if (mem_init == true) {
      WiFi_ISM43362_MemoryUninitialize();
      mem_init = false;
    }
  }

  return true;
}

/// \brief       Put data block content into buffer
/// \param[in]   idx      Index of buffer in which to put data block (0 ..)
/// \param[in]   ptr_data Pointer to data
/// \param[in]   len      Length of data
/// \return      true     Data put into buffer successfully
/// \return      false    Data not put into buffer
bool WiFi_ISM43362_BufferPut (uint8_t idx, const void *ptr_data, uint32_t len) {
  void      *ptr_mem;
  BUF_ENT_t *ptr_buf_ent_new;
  BUF_ENT_t *ptr_buf_ent_curr;

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return false;
  }
  if (buf_mgmt[idx].init == 0U) {
    return false;
  }
  
  ptr_mem = WiFi_ISM43362_MemoryAllocate(len + sizeof(BUF_ENT_t));
  if (ptr_mem == NULL) {
    return false;
  }

  ptr_buf_ent_curr = buf_mgmt[idx].head;
  ptr_buf_ent_new  = (BUF_ENT_t *)ptr_mem;

  // Update head pointer
  if (ptr_buf_ent_curr != NULL) {       // If this is first head entry
    ptr_buf_ent_curr->next = ptr_buf_ent_new;
  }
  ptr_buf_ent_new->next = NULL;

  // Put data into entry, and set len and ofs
  ptr_buf_ent_new->len  = len;
  ptr_buf_ent_new->ofs  = 0U;
  memcpy(ptr_buf_ent_new->data, ptr_data, len);

  buf_mgmt[idx].head = ptr_buf_ent_new;

  // Update tail pointer
  if (buf_mgmt[idx].tail == NULL) {     // If there is no tail entry
    buf_mgmt[idx].tail = ptr_buf_ent_new;
  }

  return true;
}

/// \brief       Get oldest unread data block content from buffer
/// \param[in]   idx      Index of buffer from which to get the data block (0 ..)
/// \param[in]   ptr_data Pointer where data will be returned
/// \param[in]   max_len  Maximum length of data to be returned
/// \return               Number of bytes returned
uint32_t WiFi_ISM43362_BufferGet (uint8_t idx, void *ptr_data, uint32_t max_len) {
  uint32_t   len;
  BUF_ENT_t *ptr_buf_ent_curr;
  BUF_ENT_t *ptr_buf_ent_next;

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return 0U;
  }
  if (buf_mgmt[idx].init == 0U) {
    return 0U;
  }
  if (buf_mgmt[idx].tail == NULL) {
    // If no message is available
    return 0U;
  }

  ptr_buf_ent_curr = (BUF_ENT_t *)buf_mgmt[idx].tail;
  len = ptr_buf_ent_curr->len - ptr_buf_ent_curr->ofs;
  if (len > max_len) {
    len = max_len;
  }
  if (len > 0) {
    memcpy(ptr_data, ptr_buf_ent_curr->data + ptr_buf_ent_curr->ofs, len);
    ptr_buf_ent_curr->ofs += len;
    if (ptr_buf_ent_curr->ofs == ptr_buf_ent_curr->len) {
      // If all data was read from this entry, remove it
      ptr_buf_ent_next = ptr_buf_ent_curr->next;
      memset(ptr_buf_ent_curr, 0, sizeof(BUF_ENT_t));
      WiFi_ISM43362_MemoryFree (ptr_buf_ent_curr);
      if (buf_mgmt[idx].head == buf_mgmt[idx].tail) {
        buf_mgmt[idx].head = NULL;
      }
      buf_mgmt[idx].tail = ptr_buf_ent_next;
    }
  }

  return len;
}

/// \brief       Check if buffer contains any unread data
/// \param[in]   idx      Index of buffer from which to get the data block (0 ..)
/// \return      true     Buffer is not empty
/// \return      false    Buffer is empty
bool WiFi_ISM43362_BufferNotEmpty (uint8_t idx) {

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return false;
  }
  if (buf_mgmt[idx].init == 0U) {
    return false;
  }
  if (buf_mgmt[idx].tail == NULL) {
    // If no message is in the buffer
    return false;
  }

  return true;
}

/// \brief       Flush data blocks from buffer
/// \param[in]   idx      Index of buffer from which to flush data blocks (0 ..)
/// \return      true     Data blocks flushed successfully
/// \return      false    Data blocks flushing failed
bool WiFi_ISM43362_BufferFlush (uint8_t idx) {
  BUF_ENT_t *ptr_buf_ent_curr;
  BUF_ENT_t *ptr_buf_ent_next;

  if (idx >= WIFI_ISM43362_SOCKETS_NUM) {
    return false;
  }
  if (buf_mgmt[idx].init == 0U) {
    return false;
  }
  if (buf_mgmt[idx].tail == NULL) {
    // If no message is in the buffer
    return true;
  }

  ptr_buf_ent_curr = (BUF_ENT_t *)buf_mgmt[idx].tail;
  while (ptr_buf_ent_curr != NULL) {
    ptr_buf_ent_next = ptr_buf_ent_curr->next;
    memset(ptr_buf_ent_curr, 0, sizeof(BUF_ENT_t));
    if (WiFi_ISM43362_MemoryFree (ptr_buf_ent_curr) == false) {
      return false;
    }
    ptr_buf_ent_curr = ptr_buf_ent_next;
  }

  buf_mgmt[idx].tail = NULL;
  buf_mgmt[idx].head = NULL;

  return true;
}
