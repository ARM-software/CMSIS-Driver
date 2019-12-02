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

#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "BufList.h"

/*
  Head buffer: first in list, contains oldest buffer, where read operation starts
  Tail buffer: last in list,  contains newest buffer, where write operation starts
*/

typedef struct {
  Link_t link;     /* Linked list        */
  uint16_t wri;    /* Buffer write index */
  uint16_t rdi;    /* Buffer read index  */
  uint8_t  data[]; /* Buffered data      */
} MEM_BUF;

/* Lock buffer access */
static void Lock (BUF_LIST *p) {
  if (p->mutex != NULL) {
    osMutexAcquire (p->mutex, osWaitForever);
  }
}

/* Unlock buffer access */
static void Unlock (BUF_LIST *p) {
  if (p->mutex != NULL) {
    osMutexRelease (p->mutex);
  }
}

/* Allocate memory block, put it into buffer list and return pointer to buffer */
static MEM_BUF *Alloc (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  Link_t  *buf_link;

  buf_cb = (MEM_BUF *)osMemoryPoolAlloc (p->mp_id, 0U);

  if (buf_cb != NULL) {
    /* Buffer allocated, add it to list */
    buf_link = (Link_t *)buf_cb;

    ListPut (&p->list, buf_link);

    buf_cb->wri = 0U; //Write index
    buf_cb->rdi = 0U; //Read index
  }

  return (buf_cb);
}

/* Get buffer from list, free memory block and return next buffer in list */
static MEM_BUF *Free (BUF_LIST *p) {
  MEM_BUF *buf_cb;

  buf_cb = (MEM_BUF *)ListGet (&p->list);

  if (osMemoryPoolFree (p->mp_id, buf_cb) == osOK) {
    /* Peek next */
    buf_cb = (MEM_BUF *)ListPeekHead(&p->list);
  }

  return (buf_cb);
}

/* Get the size of one block in buffer memory pool */
static uint16_t Size (BUF_LIST *p) {
  uint16_t sz;

  sz = p->bl_sz;

  if (sz != 0U) {
    /* Memory pool block size reduced for buffer header */
    sz -= sizeof(MEM_BUF);
  }

  return (sz);
}

/**
  Initialize buffer list.
*/
int32_t BufInit (void *mp_id, void *mutex, BUF_LIST *p) {
  int32_t  rval;
  uint32_t bl_sz;

  if ((p == NULL) || (mp_id == NULL)) {
    /* Buffer list or memory pool invalid */
    rval = -1;
  }
  else {
    bl_sz = osMemoryPoolGetBlockSize (mp_id);

    if (bl_sz > UINT16_MAX) {
      /* Not supported */
      rval = -1;
    }
    else {
      p->mutex = mutex;
      p->mp_id = mp_id;
      p->bl_sz = (uint16_t)bl_sz;

      ListInit (&p->list);

      rval = 0;
    }
  }

  return (rval);
}

/**
  Uninitialize buffer list.
*/
int32_t BufUninit (BUF_LIST *p) {
  int32_t rval;
  MEM_BUF *buf_cb;

  if (p == NULL) {
    rval = -1;
  }
  else {
    /* Free all blocks */
    do {
      buf_cb = Free (p);
    }
    while (buf_cb != NULL);

    ListInit (&p->list);

    rval = 0;
  }
  return (rval);
}


BUF_MEM *BufAlloc (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  BUF_MEM *usr_cb; //User control block

  Lock(p);

  buf_cb = Alloc (p);

  if (buf_cb != NULL) {
    /* Set pointer to write index */
    usr_cb = (BUF_MEM *)&buf_cb->wri;
  } else {
    /* Out of memory */
    usr_cb = NULL;
  }

  Unlock(p);

  return (usr_cb);
}


BUF_MEM *BufFree (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  BUF_MEM *usr_cb; //User control block

  Lock(p);

  buf_cb = Free (p);

  if (buf_cb != NULL) {
    /* Set pointer to write index */
    usr_cb = (BUF_MEM *)&buf_cb->wri;
  } else {
    /* Should never happen */
    usr_cb = NULL;
  }

  Unlock(p);

  return (usr_cb);
}

/*
  - Retrieve current buffer
  -- If current buffer is not allocated, allocate it
  -- If current buffer is full, allocate new
  -- If out of memory, return NULL
*/
BUF_MEM *BufGetTail (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  BUF_MEM *usr_cb; //User control block

  Lock(p);

  usr_cb = NULL;
  buf_cb = (MEM_BUF *)ListPeekTail(&p->list);

  if (buf_cb != NULL) {
    if (buf_cb->wri < Size (p)) {
      /* Set pointer to wri member */
      usr_cb = (BUF_MEM *)&buf_cb->wri;
    }
  }

  if (usr_cb == NULL) {
    /* Allocate new buffer */
    usr_cb = BufAlloc (p);
  }

  Unlock(p);

  return (usr_cb);
}


uint16_t BufGetSize (BUF_LIST *p) {
  uint16_t sz;

  Lock(p);

  sz = Size (p);

  Unlock(p);

  return ((uint16_t)sz);
}

uint32_t BufGetFree (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t maxi, sz;

  Lock(p);

  /* Size of one buffer (mem pool block - header */
  maxi = Size (p);

  /* Multiplied by the number of free memory pool blocks */
  sz = maxi * osMemoryPoolGetSpace (p->mp_id);

  /* Add number of bytes available in the tail buffer */
  buf_cb = (MEM_BUF *)ListPeekTail(&p->list);

  if (buf_cb != NULL) {
    sz += (maxi - buf_cb->wri);
  }

  Unlock(p);

  return (sz);
}

/**
  Retrieve number of bytes in the buffer.
*/
uint32_t BufGetCount (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t n;

  Lock(p);

  n = 0U;
  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  while (buf_cb != NULL) {
    /* Determine number of bytes in the current buffer */
    n += (buf_cb->wri - buf_cb->rdi);

    /* Load next buffer */
    buf_cb = (MEM_BUF *)ListPeekNext ((Link_t *)buf_cb);
  }

  Unlock(p);

  /* Return total number of bytes in the buffer */
  return (n);
}


int32_t BufReadByte (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  int32_t  rval;

  Lock(p);

  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer, free it */
      if (buf_cb->wri == Size(p)) {
        buf_cb = Free(p);
      } else {
        buf_cb = NULL;
      }
    }
  }

  if (buf_cb != NULL) {
    /* Return current byte */
    rval = buf_cb->data[buf_cb->rdi++];
  }
  else {
    /* End of chain */
    rval = -1;
  }

  Unlock(p);

  return (rval);
}

int32_t BufPeekByte (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  int32_t  rval;

  Lock(p);

  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer, free it */
      if (buf_cb->wri == Size(p)) {
        buf_cb = Free(p);
      } else {
        buf_cb = NULL;
      }
    }
  }

  if (buf_cb != NULL) {
    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer */
      rval = -1;
    }
    else {
      rval = buf_cb->data[buf_cb->rdi];
    }
  }
  else {
    /* No data */
    rval = -1;
  }

  Unlock(p);

  return (rval);
}

int32_t BufPeekOffs (uint32_t offs, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t rdi = 0;
  int32_t  n;

  Lock(p);

  n = -1; //End of buffer
  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    rdi = buf_cb->rdi;
  }

  while (buf_cb != NULL) {

    if (rdi == buf_cb->wri) {
      /* End of current buffer, peek next */
      buf_cb = (MEM_BUF *)ListPeekNext ((Link_t *)buf_cb);

      if (buf_cb == NULL) {
        /* End of buffer */
        n = -1;
      }

      /* Reset read index */
      rdi = 0U;
    }
    else {
      if (offs != 0U) {
        offs--;
        rdi++;
      }
      else {
        /* Return byte at specified offset */
        n = buf_cb->data[rdi];
        break;
      }
    }
  }

  Unlock(p);

  return (n);
}

int32_t BufFlushByte (BUF_LIST *p) {
  MEM_BUF *buf_cb;
  int32_t  rval;

  Lock(p);

  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer, free it */
      if (buf_cb->wri == Size(p)) {
        buf_cb = Free(p);
      } else {
        buf_cb = NULL;
      }
    }
  }

  if (buf_cb != NULL) {
    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer */
      rval = -1;
    }
    else {
      /* Read byte and increment read index */
      rval = buf_cb->data[buf_cb->rdi++];
    }
  }
  else {
    /* End of chain */
    rval = -1;
  }

  Unlock(p);

  return (rval);
}

int32_t BufWriteByte (uint8_t data, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t maxi;
  int32_t  rval;

  Lock(p);

  maxi = Size (p);

  buf_cb = (MEM_BUF *)ListPeekTail(&p->list);

  if ((buf_cb == NULL) || (buf_cb->wri == maxi)) {
    /* Buffer full, allocate new */
    buf_cb = Alloc (p);
  }

  if (buf_cb != NULL) {
    buf_cb->data[buf_cb->wri++] = data;

    rval = data;
  }
  else {
    rval = -1;
  }

  Unlock(p);

  return (rval);
}

/*
  Read num of bytes into buf and return number of bytes actually read.
*/
int32_t BufRead (uint8_t *buf, uint32_t num, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t n;

  Lock(p);

  n = 0;
  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  while (buf_cb != NULL) {

    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer, free current, get next one */
      if (buf_cb->wri == Size(p)) {
        buf_cb = Free(p);
      } else {
        buf_cb = NULL;
      }
    }
    else {
      while (n < num) {
        buf[n++] = buf_cb->data[buf_cb->rdi++];

        if (buf_cb->rdi == buf_cb->wri) {
          /* End of buffer, go back and reload */
          break;
        }
      }

      if (n == num) {
        break;
      }
    }
  }

  Unlock(p);

  return ((int32_t)n);
}


/*
  Write num of bytes from buf and return number of bytes actually written.
*/
int32_t BufWrite (uint8_t *buf, uint32_t num, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t maxi;
  uint32_t n;

  Lock(p);

  maxi = Size (p);

  n = 0U;
  buf_cb = (MEM_BUF *)ListPeekTail(&p->list);

  do {
    if ((buf_cb == NULL) || (buf_cb->wri == maxi)) {
      /* Buffer full, allocate new */
      buf_cb = Alloc (p);
    }

    if (buf_cb != NULL) {

      while (n < num) {
        buf_cb->data[buf_cb->wri++] = buf[n++];

        if (buf_cb->wri == maxi) {
          /* End of buffer, go back */
          break;
        }
      }

      if (n == num) {
        /* All bytes written */
        break;
      }
    }
  } while (buf_cb != NULL);

  Unlock(p);

  return ((int32_t)n);
}


/*
  Copy num of bytes from src to dst and return number of bytes actually copied.
*/
uint32_t BufCopy (BUF_LIST *dst, BUF_LIST *src, uint32_t num) {
  MEM_BUF *dst_cb, *src_cb;
  uint32_t sz_d, sz_s;
  uint32_t i;
  uint32_t maxi;

  Lock(dst);
  Lock(src);

  i    = 0U;
  maxi = Size (dst);

  dst_cb = (MEM_BUF *)ListPeekTail(&dst->list);
  src_cb = (MEM_BUF *)ListPeekHead(&src->list);

  while (i < num) {
    if (src_cb == NULL) {
      /* End of source buffer */
      break;
    }

    if (dst_cb == NULL) {
      /* Allocate new destination buffer */
      dst_cb = Alloc (dst);

      if (dst_cb == NULL) {
        break;
      }
    }
    /* Determine free space in current buffers */
    sz_d = maxi - dst_cb->wri;
    sz_s = src_cb->wri - src_cb->rdi;

    while ((sz_d > 0) && (sz_s > 0) && (i < num)) {
      dst_cb->data[dst_cb->wri++] = src_cb->data[src_cb->rdi++];

      /* Decrement number of available space/data */
      sz_d--;
      sz_s--;

      /* Increment number of copied bytes */
      i++;
    }

    if (sz_d == 0) {
      /* Destination buffer is full */
      dst_cb = NULL;
    }

    if (sz_s == 0) {
      /* Source buffer is empty */
      if (src_cb->wri == Size(src)) {
        src_cb = Free(src);
      } else {
        src_cb = NULL;
      }
    }
  }

  Unlock(src);
  Unlock(dst);

  /* Return number of copied bytes */
  return (i);
}


/*
  Flush num of bytes from the list buffer. List buffer is flushed completely when num equals to zero.
*/
uint32_t BufFlush (uint32_t num, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t n;

  Lock(p);

  n = 0U;
  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  while (buf_cb != NULL) {

    if (buf_cb->rdi == buf_cb->wri) {
      /* End of current buffer, free current, get next one */
      if (buf_cb->wri == Size(p)) {
        buf_cb = Free(p);
      } else {
        buf_cb = NULL;
      }
    }
    else {
      buf_cb->rdi++;
      /* Increment number of bytes flushed */
      n++;
    }

    if (num != 0U) {
      /* Check if required number of bytes flushed */
      if (n == num) {
        break;
      }
    }
  }

  Unlock(p);

  return (n);
}


/*
  Find the first occurence of a data byte in the list buffer and return its offset from current position.
*/
int32_t BufFindByte (uint8_t data, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t offs, rdi;
  int32_t n;

  Lock(p);

  n    = -1;
  rdi  = 0U;
  offs = 0U;

  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    rdi = buf_cb->rdi;
  }

  while (buf_cb != NULL) {

    if (rdi == buf_cb->wri) {
      /* End of current buffer, peek next */
      buf_cb = (MEM_BUF *)ListPeekNext ((Link_t *)buf_cb);

      /* Reset read index */
      rdi = 0U;
    }
    else {
      /* Check current buffer */
      while (rdi < buf_cb->wri) {
        /* Compare data with current buffer content */
        if (data == buf_cb->data[rdi]) {
          /* Equal data byte found */
          n = (int32_t)offs;
          break;
        }
        offs++;
        rdi++;
      }

      if (n != -1) {
        break;
      }
    }
  }

  Unlock(p);

  return (n);
}


/*
  Find the first occurence of a data sequence in the list buffer and return its offset from current position.
  
  num   number of bytes from data to compare
*/
int32_t BufFind (const uint8_t *data, uint32_t num, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t offs, rdi;
  uint32_t i;
  int32_t n;

  Lock(p);

  n    = -1; //No match
  i    = 0U;
  rdi  = 0U;
  offs = 0U;

  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    rdi = buf_cb->rdi;
  }

  while (buf_cb != NULL) {

    if (rdi == buf_cb->wri) {
      /* End of current buffer, peek next */
      buf_cb = (MEM_BUF *)ListPeekNext ((Link_t *)buf_cb);

      /* Reset read index */
      rdi = 0U;
    }
    else {
      /* Check current buffer */
      while (rdi < buf_cb->wri) {
        /* Compare data with current buffer content */
        if (data[i] != buf_cb->data[rdi]) {
          /* Adjust offset for number of matches */
          if (i == 0) { offs += 1; }
          else        { offs += i; }

          /* Reset number of matches */
          i = 0U;
        }

        if (data[i] == buf_cb->data[rdi]) {
          /* Equal data byte found */
          i++;
        }

        rdi++;
        if (i == num) {
          /* Compared sequence matches */
          n = (int32_t)offs;
          break;
        }
      }

      if (n != -1) {
        break;
      }
    }
  }

  Unlock(p);

  return (n);
}


/*
  Compare string with buffered data
  
  \note Does not move buffer pointers
*/
int32_t BufCompareString (const char *string, uint32_t offs, BUF_LIST *p) {
  MEM_BUF *buf_cb;
  uint32_t rdi = 0U;
  int32_t  n;

  Lock(p);

  n = 0; //No match
  buf_cb = (MEM_BUF *)ListPeekHead(&p->list);

  if (buf_cb != NULL) {
    rdi = buf_cb->rdi;
  }

  while (buf_cb != NULL) {

    if (rdi == buf_cb->wri) {
      /* End of current buffer, peek next */
      buf_cb = (MEM_BUF *)ListPeekNext ((Link_t *)buf_cb);

      if (buf_cb == NULL) {
        /* End of buffer */
        n = -1;
      }

      /* Reset read index */
      rdi = 0U;
    }
    else {
      if (offs != 0U) {
        offs--;
        rdi++;
      }
      else {
        while (rdi < buf_cb->wri) {
          if (string[n] == '\0') {
            /* End of string */
            break;
          }
          /* Compare string with content of current buffer */
          if (string[n] != buf_cb->data[rdi]) {
            /* No match */
            n = 0;
            break;
          }
          n++;
          rdi++;
        }

        if ((n <= 0) || (string[n] == '\0')) {
          /* No match or end of string */
          break;
        }
      }
    }
  }

  Unlock(p);

  return (n);
}
