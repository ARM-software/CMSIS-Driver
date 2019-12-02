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
 * $Date:        12. November 2019
 * $Revision:    V1.0
 *
 * Project:      Buffering using CMSIS-RTOS2 memory pools as storage
 * -------------------------------------------------------------------------- */

#ifndef BUFLIST_H__
#define BUFLIST_H__

#include <stdint.h>
#include "LinkList.h"

typedef struct {
  uint16_t wr_idx; /* Buffer write index */
  uint16_t rd_idx; /* Buffer read index  */
  uint8_t  data[]; /* Buffer data array  */
} BUF_MEM;

typedef struct {
  List_t   list;   /* Linked list            */
  void    *mutex;  /* Buffer access mutex    */
  void    *mp_id;  /* Memory pool id         */
  uint16_t bl_sz;  /* Memory pool block size */
  uint16_t rsvd;   /* Reserved               */
} BUF_LIST;

/**
  Initialize buffer list.
*/
extern int32_t BufInit (void *mp_id, void *mutex, BUF_LIST *p);

/**
  Uninitialize buffer list.
*/
extern int32_t BufUninit (BUF_LIST *p);

/**
  Allocate new buffer, add it into the list and return buffer information.
*/
extern BUF_MEM *BufAlloc (BUF_LIST *p);

/**
  Remove current buffer from the list and return information about next buffer in list.
*/
extern BUF_MEM *BufFree  (BUF_LIST *p);

/**
  Retrieve current write buffer (last buffer added to the list).
*/
extern BUF_MEM *BufGetTail (BUF_LIST *p);

/**
  Retrieve common buffer size valid for all buffers in the list.

  Common buffer size is the memory pool block size reduced for handling header.
*/
extern uint16_t BufGetSize (BUF_LIST *p);

/**
  Retrieve total amount of free space for give buffer list.

  Total amount of free space consists of free space in allocated blocks
  and free memory pool blocks.
*/
extern uint32_t BufGetFree (BUF_LIST *p);

/**
  Retrieve number of bytes in the buffer.
*/
extern uint32_t BufGetCount (BUF_LIST *p);

/**
  Read a byte from the list buffer.

  \return byte read or -1 if buffer empty
*/
extern int32_t BufReadByte (BUF_LIST *p);

/**
  Peek a byte from the list buffer.

  \return byte read or -1 if buffer empty
*/
extern int32_t BufPeekByte (BUF_LIST *p);

/**
  Peek a byte with the specified offset from current position.
  
  \return byte value or -1 if buffer empty
*/
extern int32_t BufPeekOffs (uint32_t offs, BUF_LIST *p);

/**
  Flush a byte from the list buffer.

  \return byte flushed or -1 if buffer error.
*/
extern int32_t BufFlushByte (BUF_LIST *p);

/**
  Write a byte into the list buffer.
*/
extern int32_t BufWriteByte (uint8_t data, BUF_LIST *p);

/**
  Read num of bytes into buf from the list buffer.
  
  \param[out]   buf   data buffer
  \param[in]    num   number of bytes to read
  \param[in]    p     link buffer structure pointer
  
  \return number of bytes read
*/
extern int32_t BufRead (uint8_t *buf, uint32_t num, BUF_LIST *p);

/**
  Write num of bytes from buf into the list buffer.
*/
extern int32_t BufWrite (uint8_t *buf, uint32_t num, BUF_LIST *p);

/**
  Copy num of bytes from the source list buffer into the destination list buffer.
  
  \return number of bytes copied
*/
extern uint32_t BufCopy (BUF_LIST *dst, BUF_LIST *src, uint32_t num);

/**
  Flush num of bytes from the list buffer. List buffer is flushed completely when num equals to zero.

  This function increments list buffer read pointer.
*/
extern uint32_t BufFlush (uint32_t num, BUF_LIST *p);

/**
  Find the first occurence of a data byte in the list buffer and return its offset from current position.

  This function does not move list buffer read pointer.
  \return non-zero if data byte was found, -1 otherwise
*/
extern int32_t BufFindByte (uint8_t data, BUF_LIST *p);

/**
  Find the first occurence of a data sequence in the list buffer and return its offset from current position.

  This function does not move list buffer read pointer.

  \param[in]  data    data sequence
  \param[in]  num     number of bytes from data to compare
  \param[in]  p       list buffer pointer
  \return   >=0: offset from the start of data
             -1: no match
*/
extern int32_t BufFind (const uint8_t *data, uint32_t num, BUF_LIST *p);

/**
  Compare string with the data in the list buffer.

  This function does not move list buffer read pointer.

  \param[in]  string  string to compare
  \param[in]  offs    offset from current position
  \param[in]  p       list buffer pointer
  \return   >0: match, string length not including null terminator
             0: no match
            -1: no match, end of buffer
*/
extern int32_t BufCompareString (const char *string, uint32_t offs, BUF_LIST *p);

#endif /* BUFLIST_H__ */
