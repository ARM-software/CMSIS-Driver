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
 * Project:      ESP8266 WiFi Driver
 * -------------------------------------------------------------------------- */

#include "WiFi_ESP8266_Os.h"

#define THREAD_CC_ATTR     __attribute__((section(".bss.os.thread.cb")))
#define MEMPOOL_CC_ATTR    __attribute__((section(".bss.os.mempool.cb")))
#define EVENTFLAGS_CC_ATTR __attribute__((section(".bss.os.evflags.cb")))
#define MUTEX_CC_ATTR      __attribute__((section(".bss.os.mutex.cb")))

/* --------------------------------------------------------------------------*/

#define WIFI_THREAD_STACK_ARR_SIZE   WIFI_THREAD_STACK_SIZE

/* Stack memory */
static uint8_t WiFi_ThreadCb[OS_THREAD_CB_SIZE]                __ALIGNED(4) THREAD_CC_ATTR;
static uint8_t WiFi_ThreadStackArr[WIFI_THREAD_STACK_ARR_SIZE] __ALIGNED(8);

/* WiFi thread */
const osThreadAttr_t WiFi_Thread_Attr = {
  .name       = "WiFi Thread",
  .attr_bits  = osThreadDetached,
  .cb_mem     = WiFi_ThreadCb,
  .cb_size    = sizeof(WiFi_ThreadCb),
  .stack_mem  = WiFi_ThreadStackArr,
  .stack_size = sizeof (WiFi_ThreadStackArr),
  .priority   = WIFI_THREAD_PRIORITY,
  .tz_module  = 0,
};

/* --------------------------------------------------------------------------*/

/* Memory Pool array size */
#define SOCKET_MEMPOOL_ARR_SIZE    OS_MEMPOOL_MEM_SIZE(SOCKET_BUFFER_BLOCK_COUNT, SOCKET_BUFFER_BLOCK_SIZE)

/* Memory Pool control block and memory space */
static uint8_t Socket_MemPoolCb[OS_MEMPOOL_CB_SIZE]       __ALIGNED(4) MEMPOOL_CC_ATTR;
static uint8_t Socket_MemPoolArr[SOCKET_MEMPOOL_ARR_SIZE] __ALIGNED(4);

/* Memory pool for socket data storage */
const osMemoryPoolAttr_t Socket_MemPool_Attr = {
  .name      = "WiFi Socket",
  .attr_bits = 0U,
  .cb_mem    = Socket_MemPoolCb,
  .cb_size   = sizeof(Socket_MemPoolCb),
  .mp_mem    = Socket_MemPoolArr,
  .mp_size   = sizeof(Socket_MemPoolArr)
};

/* --------------------------------------------------------------------------*/

#define WIFI_MUTEX_ATTRIBUTES  osMutexPrioInherit

static uint8_t Socket_MutexCb[OS_MUTEX_CB_SIZE] __ALIGNED(4) MUTEX_CC_ATTR;

const osMutexAttr_t Socket_Mutex_Attr = {
  .name      = "WiFi Socket",
  .attr_bits = WIFI_MUTEX_ATTRIBUTES,
  .cb_mem    = Socket_MutexCb,
  .cb_size   = sizeof(Socket_MutexCb)
};

/* --------------------------------------------------------------------------*/

#define ATPARSER_MEMPOOL_ARR_SIZE     OS_MEMPOOL_MEM_SIZE(PARSER_BUFFER_BLOCK_COUNT, PARSER_BUFFER_BLOCK_SIZE)

/* Memory Pool control block and memory space */
static uint8_t AT_Parser_MemPoolCb[OS_MEMPOOL_CB_SIZE]         __ALIGNED(4) MEMPOOL_CC_ATTR;
static uint8_t AT_Parser_MemPoolArr[ATPARSER_MEMPOOL_ARR_SIZE] __ALIGNED(4) ;

/* Memory Pool for AT command parser */
const osMemoryPoolAttr_t AT_Parser_MemPool_Attr = {
  .name      = "WiFi Parser",
  .attr_bits = 0U,
  .cb_mem    = AT_Parser_MemPoolCb,
  .cb_size   = sizeof(AT_Parser_MemPoolCb),
  .mp_mem    = AT_Parser_MemPoolArr,
  .mp_size   = sizeof(AT_Parser_MemPoolArr),
};

/* --------------------------------------------------------------------------*/

static uint8_t WiFi_EventFlagsCb[OS_EVENTFLAGS_CB_SIZE] __ALIGNED(4) EVENTFLAGS_CC_ATTR;

const osEventFlagsAttr_t WiFi_EventFlags_Attr = {
  .name    = "WiFi Wait",
  .cb_mem  = WiFi_EventFlagsCb,
  .cb_size = sizeof(WiFi_EventFlagsCb)
};

/* --------------------------------------------------------------------------*/

static uint8_t BufList_MutexCb[OS_MUTEX_CB_SIZE] __ALIGNED(4) MUTEX_CC_ATTR;

const osMutexAttr_t BufList_Mutex_Attr = {
  .name      = "BufList",
  .attr_bits = osMutexPrioInherit,
  .cb_mem    = BufList_MutexCb,
  .cb_size   = sizeof(BufList_MutexCb)
};
