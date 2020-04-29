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
 * Project:      Message buffer for Inventek ISM43362 WiFi Driver
 *               header file
 * -------------------------------------------------------------------------- */

#ifndef __WIFI_ISM43362_BUF_H__
#define __WIFI_ISM43362_BUF_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct WIFI_ISM43362_BUF_ENT_t {// Message buffer entry structure (packed in memory block data)
  struct WIFI_ISM43362_BUF_ENT_t *next; // Next message entry
  uint32_t                       len;   // Message data length
  uint32_t                       ofs;   // Offset to already read data
  uint8_t                        data[];// Message data
} WIFI_ISM43362_BUF_ENT_t;

typedef struct WIFI_ISM43362_BUF_t {    // Message buffer management structure
  uint32_t                        init; // Initialization status
  struct WIFI_ISM43362_BUF_ENT_t *tail; // Pointer to head entry (latest)
  struct WIFI_ISM43362_BUF_ENT_t *head; // Pointer to tail entry (oldest)
} WIFI_ISM43362_BUF_t;


/// \brief       Initialize buffer
/// \param[in]   idx      Index of buffer to initialize (0 ..)
/// \return      true     Buffer initialized successfully
/// \return      false    Buffer initialization failed
extern bool WiFi_ISM43362_BufferInitialize (uint8_t idx);

/// \brief       Uninitialize buffer
/// \param[in]   idx      Index of buffer to uninitialize (0 ..)
/// \return      true     Buffer uninitialized successfully
/// \return      false    Buffer uninitialization failed
extern bool WiFi_ISM43362_BufferUninitialize (uint8_t idx);

/// \brief       Put data block content into buffer
/// \param[in]   idx      Index of buffer in which to put data block (0 ..)
/// \param[in]   ptr_data Pointer to data
/// \param[in]   len      Length of data
/// \return      true     Data put into buffer successfully
/// \return      false    Data not put into buffer
extern bool WiFi_ISM43362_BufferPut (uint8_t idx, const void *ptr_data, uint32_t len);

/// \brief       Get oldest unread data block content from buffer
/// \param[in]   idx      Index of buffer from which to get the data block (0 ..)
/// \param[in]   ptr_data Pointer where data will be returned
/// \param[in]   max_len  Maximum length of data to be returned
/// \return               Number of bytes returned
extern uint32_t WiFi_ISM43362_BufferGet (uint8_t idx, void *ptr_data, uint32_t max_len);

/// \brief       Check if buffer contains any unread data
/// \param[in]   idx      Index of buffer from which to get the data block (0 ..)
/// \return      true     Buffer is not empty
/// \return      false    Buffer is empty
extern bool WiFi_ISM43362_BufferNotEmpty (uint8_t idx);

/// \brief       Flush data blocks from buffer
/// \param[in]   idx      Index of buffer from which to flush data blocks (0 ..)
/// \return      true     Data blocks flushed successfully
/// \return      false    Data blocks flushing failed
extern bool WiFi_ISM43362_BufferFlush (uint8_t idx);

#endif  // __WIFI_ISM43362_BUF_H__
