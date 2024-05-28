/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------------
 *
 * $Date:       28. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host EHCI Controller Driver
 *              for customized EHCI with internal Transaction Translator (TT)
 *              (with full/low speed support)
 *
 * -----------------------------------------------------------------------------
 */

/* History:
 *  Version 1.0
 *    Initial release
 */

#include "USBH_EHCI_TT.h"

#include <stdio.h>
#include <string.h>

#include "USBH_EHCI_Config.h"
#include "USBH_EHCI_TT_Regs.h"
#include "USBH_EHCI_HW.h"

#include "cmsis_os2.h"
#include "cmsis_compiler.h"

// Driver Version *************************************************************
static const ARM_DRIVER_VERSION usbh_driver_version = { ARM_USBH_API_VERSION, ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) };
// ****************************************************************************

// Compile-time configuration *************************************************

// Configuration depending on the USBH_EHCI_Config.h

#if    (USBH1_EHCI_ENABLED == 1)
#define USBH_EHCI_TT_INSTANCES         (2U)
#else
#define USBH_EHCI_TT_INSTANCES         (1U)
#endif

// ****************************************************************************

// Driver Capabilities ********************************************************
static ARM_USBH_CAPABILITIES usbh_driver_capabilities[USBH_EHCI_TT_INSTANCES] = {
  {
    0x3FFFU,    /* Root HUB available Ports Mask   */
    0U,         /* Automatic SPLIT packet handling */
    1U,         /* Signal Connect event            */
    1U,         /* Signal Disconnect event         */
    1U,         /* Signal Overcurrent event        */
    0U          /* reserved bits                   */
  }
#if (USBH_EHCI_TT_INSTANCES >= 2)
, {
    0x3FFFU,    /* Root HUB available Ports Mask   */
    0U,         /* Automatic SPLIT packet handling */
    1U,         /* Signal Connect event            */
    1U,         /* Signal Disconnect event         */
    1U,         /* Signal Overcurrent event        */
    0U          /* reserved bits                   */
  }
#endif
};
// ****************************************************************************

// Macros

#define USBHn_EHCI_COM_AREA_SECTION_(x) __attribute__((section(x)))

#if    (USBH0_EHCI_COM_AREA_RELOC == 1)
#define USBH0_EHCI_COM_AREA_SECTION(x)  USBHn_EHCI_COM_AREA_SECTION_(x)
#else 
#define USBH0_EHCI_COM_AREA_SECTION(x)
#endif

#if    (USBH1_EHCI_ENABLED == 1)
#if    (USBH1_EHCI_COM_AREA_RELOC == 1)
#define USBH1_EHCI_COM_AREA_SECTION(x)  USBHn_EHCI_COM_AREA_SECTION_(x)
#else 
#define USBH1_EHCI_COM_AREA_SECTION(x)
#endif
#endif

#ifndef USBH_EHCI_MEM_PFL_SIZE
#define USBH_EHCI_MEM_PFL_SIZE          (4096U)
#endif
#ifndef USBH_EHCI_MEM_QH_SIZE
#define USBH_EHCI_MEM_QH_SIZE           (USBH_EHCI_MAX_PIPES * 64U)
#endif
#ifndef USBH_EHCI_MEM_QTD_SIZE
#define USBH_EHCI_MEM_QTD_SIZE          (USBH_EHCI_MAX_PIPES * 32U)
#endif
#ifndef USBH_EHCI_MEM_SITD_SIZE
#define USBH_EHCI_MEM_SITD_SIZE         (2U * (USBH_EHCI_MAX_PIPES - 1) * 32U)
#endif

// Transfer information structure
typedef struct {
  uint32_t             *ptr_transfer;                       // pointer to transfer structure
  uint32_t             *ptr_pipe;                           // pointer to pipe structure
  uint32_t              packet;                             // packet
  uint8_t              *data;                               // pointer to data
  uint32_t              num;                                // number of bytes to transfer
  uint32_t              num_transferred_total;              // total number of bytes transferred
  uint32_t              num_to_transfer;                    // number of bytes to transfer in single transfer
  uint32_t              active;                             // activity flag
  uint32_t             *iso_ptr_pipe1;                      // iso pointer to additional pipe structure
  uint8_t              *iso_data1;                          // iso pointer to data for second transfer
  uint16_t              iso_max_transfer_size;              // iso maximum transfer size per frame
  uint16_t              iso_interval;                       // iso endpoint interval in (u)SOFs
  uint32_t              iso_last_frame_index;               // iso last transfer frame index
  uint32_t              iso_frame_index[2];                 // iso transfer frame index
} USBH_TransferInfo_t;

// Pipe event structure
typedef struct {                                            // Additional Pipe Event parameters
  const uint8_t        *buf;                                // Data buffer start address
        uint32_t        len;                                // Number of transferred data bytes
} USBH_PipeEventInfo_t;

// Structure containing configuration values for EHCI Compliant Controller
typedef struct {
  uint32_t              ctrl;                               // controller index (used for hardware specific driver)
  uint32_t             *ptr_EHCI;                           // pointer to memory mapped reg base address
  uint16_t              max_qH;                             // maximum queue Heads
  uint16_t              max_qTD;                            // maximum queue Transfer Descriptors
  uint16_t              max_iTD;                            // maximum Isochronous Transfer Descriptors
  uint16_t              max_siTD;                           // maximum split Isochronous Transfer Descriptors
  uint16_t              max_FSTN;                           // maximum periodic Frame Span Traversal Nodes
  uint16_t              pad0;                               // explicit padding
  uint32_t             *ptr_PFL;                            // pointer to Periodic Frame List memory
  uint32_t             *ptr_qH;                             // pointer to qH memory start
  uint32_t             *ptr_qTD;                            // pointer to qTD memory start
  uint32_t             *ptr_iTD;                            // pointer to iTD memory start
  uint32_t             *ptr_siTD;                           // pointer to siTD memory start
  uint32_t             *ptr_FSTN;                           // pointer to FSTN memory start
  USBH_TransferInfo_t  *ptr_TI;                             // pointer to Transfer Info (TI) array start
  USBH_PipeEventInfo_t *ptr_PEI;                            // pointer to Pipe Event Info (PEI)
  USBH_EHCI_Interrupt_t irq_handler;                        // pointer to EHCI Interrupt Handler Routine
} USBH_EHCI_t;

// Communication structure
typedef struct {
  uint32_t              pfl  [USBH_EHCI_MEM_PFL_SIZE  / 4];
  uint32_t              qh   [USBH_EHCI_MEM_QH_SIZE   / 4];
  uint32_t              qtd  [USBH_EHCI_MEM_QTD_SIZE  / 4];
  uint32_t              sitd [USBH_EHCI_MEM_SITD_SIZE / 4];
} USBH_EHCI_ComArea_t;

// Local functions prototypes
static void Driver_USBH0_IRQ_Handler (void);
#if   (USBH1_EHCI_ENABLED == 1)
static void Driver_USBH1_IRQ_Handler (void);
#endif

// USB Host 0 information
static USBH_TransferInfo_t  usbh0_transfer_info    [USBH_EHCI_MAX_PIPES];
static USBH_PipeEventInfo_t usbh0_pipe_evt_info;
static USBH_EHCI_ComArea_t  usbh0_ehci_com_area     USBH0_EHCI_COM_AREA_SECTION(USBH0_EHCI_COM_AREA_SECTION_NAME) __ALIGNED(USBH_EHCI_MEM_PFL_SIZE);
static const USBH_EHCI_t    usbh0_ehci = {          USBH0_EHCI_DRV_NUM,
                                                   (uint32_t *)USBH0_EHCI_BASE_ADDR,
                                                    USBH_EHCI_MAX_PIPES,
                                                    USBH_EHCI_MAX_PIPES,
                                                    0U,
                                                    2*(USBH_EHCI_MAX_PIPES-1),
                                                    1024U,
                                                    0U, // padding byte
                                                   &usbh0_ehci_com_area.pfl[0],
                                                   &usbh0_ehci_com_area.qh[0],
                                                   &usbh0_ehci_com_area.qtd[0],
                                                    NULL,
                                                   &usbh0_ehci_com_area.sitd[0],
                                                    NULL,
                                                   &usbh0_transfer_info[0],
                                                   &usbh0_pipe_evt_info, 
                                                    Driver_USBH0_IRQ_Handler
                                                 };

// USB Host 1 information
#if   (USBH1_EHCI_ENABLED == 1)
static USBH_TransferInfo_t  usbh1_transfer_info    [USBH_EHCI_MAX_PIPES];
static USBH_PipeEventInfo_t usbh1_pipe_evt_info;
static USBH_EHCI_ComArea_t  usbh1_ehci_com_area     USBH1_EHCI_COM_AREA_SECTION(USBH1_EHCI_COM_AREA_SECTION_NAME) __ALIGNED(USBH_EHCI_MEM_PFL_SIZE);
static const USBH_EHCI_t    usbh1_ehci = {          USBH1_EHCI_DRV_NUM,
                                                   (uint32_t *)USBH1_EHCI_BASE_ADDR,
                                                    USBH_EHCI_MAX_PIPES,
                                                    USBH_EHCI_MAX_PIPES,
                                                    0U,
                                                    2*(USBH_EHCI_MAX_PIPES-1),
                                                    1024U,
                                                    0U, // padding byte
                                                   &usbh1_ehci_com_area.pfl[0],
                                                   &usbh1_ehci_com_area.qh[0],
                                                   &usbh1_ehci_com_area.qtd[0],
                                                    NULL,
                                                   &usbh1_ehci_com_area.sitd[0],
                                                    NULL,
                                                   &usbh1_transfer_info[0],
                                                   &usbh1_pipe_evt_info,
                                                    Driver_USBH1_IRQ_Handler
                                                 };
#endif // (USBH1_EHCI_ENABLED == 1)

// USB Hosts information
static const USBH_EHCI_t * const usbh_ehci_ptr[USBH_EHCI_TT_INSTANCES] = {
       &usbh0_ehci
#if   (USBH_EHCI_TT_INSTANCES >= 2)
     , &usbh1_ehci
#endif
};

static USBH_EHCI_Registers_t      *usbh_ehci_reg_ptr[USBH_EHCI_TT_INSTANCES];
static ARM_USBH_SignalPortEvent_t  signal_port_event[USBH_EHCI_TT_INSTANCES];
static ARM_USBH_SignalPipeEvent_t  signal_pipe_event[USBH_EHCI_TT_INSTANCES];

static uint32_t pfl_size[USBH_EHCI_TT_INSTANCES];
static uint32_t port_act[USBH_EHCI_TT_INSTANCES];

/* USBH Transfer and Endpoint Helper Functions ------------*/

/**
  \fn          uint16_t USBH_EHCI_PFL_GetSize (uint8_t ctrl)
  \brief       Get size (number of entries) of Periodic Frame List
  \param[in]   ctrl        Index of USB Host controller
  \return      size of Periodic Frame List (256, 512 or 1024)
*/
static uint16_t USBH_EHCI_PFL_GetSize (uint8_t ctrl) {
  uint32_t  usbcmd;
  uint32_t  i;
  uint16_t  sz;

  usbcmd = usbh_ehci_reg_ptr[ctrl]->USBCMD;
  sz  =   1024;
  i   = ((usbcmd >> 13) & 0x04U) | ((usbcmd >> 2) & 0x03U);
  while ((i--) != 0U) {
    sz >>= 1;
  }
  return sz;
}

/**
  \fn          void USBH_EHCI_PFL_Clear (uint8_t ctrl)
  \brief       Clear Periodic Frame List
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_EHCI_PFL_Clear (uint8_t ctrl) {
  uint32_t *ptr_uint32_t;
  uint32_t  sz;

  sz = USBH_EHCI_PFL_GetSize(ctrl);
  ptr_uint32_t = (usbh_ehci_ptr[ctrl])->ptr_PFL;
  while ((sz--) != 0U) {
    *ptr_uint32_t = 1U;                 // Set T bit to 1, thus disable entry
     ptr_uint32_t++;
  }
}

/**
  \fn          void USBH_EHCI_IOC (uint32_t *ptr_pfl_entry)
  \brief       Process periodic transfers chain and disable IoC (Interrupt On Complete) on all but last periodic transfer
  \param[in]   ptr_pfl_entry    Pointer to Periodic Frame List entry to start from
  \return      none
*/
static void USBH_EHCI_IOC (uint32_t *ptr_pfl_entry) {
  USBH_EHCI_COMMON *ptr_curr;
  USBH_EHCI_COMMON *ptr_int_td;

  if (ptr_pfl_entry == NULL)       { return; }
  if ((*ptr_pfl_entry & 1U) != 0U) { return; }          // If terminate bit is 1 (not a valid entry)

  // If terminate bit is 0 (valid entry)
  ptr_curr = (USBH_EHCI_COMMON *)(*ptr_pfl_entry & (~0x1FU));

  // Process all isochronous (siTDs) first
  while ((ptr_curr->DW0.LinkPtr != 0U) && 
         (ptr_curr->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_siTD) && 
         (ptr_curr->DW0.T == 0U)) {
    ((USBH_EHCI_siTD *)ptr_curr)->DW3.IOC = 0U;         // Clear IOC (for current siTD)
    ptr_curr = (USBH_EHCI_COMMON *)(ptr_curr->DW0.LinkPtr << 5);
  }

  if ((ptr_curr->DW0.LinkPtr != 0U) && 
      (ptr_curr->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_qH) && 
      (ptr_curr->DW0.T == 0U)) {
    // Process all interrupt (qH)
    do {
      ptr_int_td = ptr_curr;
      while ((ptr_int_td->DW0.LinkPtr != 0U) && 
             (ptr_int_td->DW0.T == 0U)) {
        ((USBH_EHCI_qTD *)ptr_int_td)->DW2.IOC = 0U;      // Clear IOC (for current qTD)
        ptr_int_td = (USBH_EHCI_COMMON *)(ptr_int_td->DW0.LinkPtr << 5);
      }
    } while ((ptr_curr->DW0.LinkPtr != 0U) && 
             (ptr_curr->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_qH) && 
             (ptr_curr->DW0.T == 0U));
    ((USBH_EHCI_siTD *)ptr_int_td)->DW3.IOC = 1U;         // Set IOC (for last qTD)
  } else {
    // There are no interrupt qH in chain, so ptr_curr points to last siTD
    ((USBH_EHCI_siTD *)ptr_curr)->DW3.IOC = 1U;           // Set IOC (for last siTD)
  }
}

/**
  \fn          bool USBH_EHCI_StartStop (uint8_t ctrl, uint8_t type, bool start)
  \brief       Start or stop asynchronous or periodic schedule processing
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   type        Type of processing to start
                             USBH_EHCI_ASYNC_SCHED:     asynchronous schedule
                             USBH_EHCI_PERIODIC_SCHED:  periodic schedule
  \param[in]   start       Start or stop
                             true:                      start schedule processing
                             false:                     stop schedule processing
  \return      true = success, false = fail
*/
static bool USBH_EHCI_StartStop (uint8_t ctrl, uint8_t type, bool start) {
  uint32_t usbcmd, usbsts;
  uint32_t tout;
  uint32_t msk;

  if (type == USBH_EHCI_ASYNC_SCHED) {            // Asynchronous type
    msk = (1UL << 5);
  } else {                                        // Periodic type
    msk = (1UL << 4);
  }
                                                
  // Wait for status = command (max 1 second)
  for (tout = 10100U; ; tout-- ){
    usbcmd = usbh_ehci_reg_ptr[ctrl]->USBCMD;
    usbsts = usbh_ehci_reg_ptr[ctrl]->USBSTS;
    if (((usbcmd ^ (usbsts >> 10)) & msk) == 0U) {
      break;
    }
    if (tout ==   0U) {
      return false;
    }
    if (tout <= 100U) {
      (void)osDelay(10U);
    }
  }

  if (start) {
    if ((usbh_ehci_reg_ptr[ctrl]->USBCMD & msk) == 0U) {        // If schedule disabled
      usbh_ehci_reg_ptr[ctrl]->USBCMD |=  msk;                  // enable schedule
    }
  } else {
    if ((usbh_ehci_reg_ptr[ctrl]->USBCMD & msk) != 0U) {        // If schedule enabled
      usbh_ehci_reg_ptr[ctrl]->USBCMD &= ~msk;                  // disable schedule
    }
  }

  // Wait for status = command (max 1 second)
  for (tout = 10100U; ; tout-- ){
    usbcmd = usbh_ehci_reg_ptr[ctrl]->USBCMD;
    usbsts = usbh_ehci_reg_ptr[ctrl]->USBSTS;
    if (((usbcmd ^ (usbsts >> 10)) & msk) == 0U) {
      break;
    }
    if (tout ==   0U) {
      return false;
    }
    if (tout <= 100U) {
      (void)osDelay(10U);
    }
  }

  return true;
}

/**
  \fn          bool USBH_EHCI_DoHandshake (uint8_t ctrl)
  \brief       Do handshake for asynchronous schedule
  \param[in]   ctrl        Index of USB Host controller
  \return      true = success, false = fail
*/
#if 0
static bool USBH_EHCI_DoHandshake (uint8_t ctrl) {
  uint32_t tout;

  // Activate Interrupt on Async Advance Doorbell
  usbh_ehci_reg_ptr[ctrl]->USBCMD &= ~(1 << 6);

  tout = 10100;
  while (!((usbh_ehci_reg_ptr[ctrl]->USBSTS >> 5) & 1)){
    if (tout-- <= 100) {
      if (tout == 0) return false;
      (void)osDelay(10U);
    }
  }
  usbh_ehci_reg_ptr[ctrl]->USBSTS = (1 << 5);     // Clear Interrupt on Async Advance

  return true;
}
#endif

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_GetFree (uint8_t ctrl)
  \brief       Get free queue element Transfer Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free queue element Transfer Descriptor, no free or function failed
               value > 0:  pointer to first free queue element Transfer Descriptor
               value = 0:  no free queue element Transfer Descriptor exists or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_GetFree (uint8_t ctrl) {
  USBH_EHCI_qTD *ptr_qTD;
  uint32_t       i;

  ptr_qTD = (USBH_EHCI_qTD *)((uint32_t)((usbh_ehci_ptr[ctrl])->ptr_qTD));

  if (ptr_qTD == NULL) { return NULL; }

  for (i = 0; i < (usbh_ehci_ptr[ctrl])->max_qTD; i++) {
    if (ptr_qTD->DW2.IOC == 0U) {
      return ptr_qTD;
    }
    ptr_qTD++;
  }

  return NULL;
}

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_GetNext_qTD (const USBH_EHCI_qTD *ptr_qTD)
  \brief       Get next queue element Transfer Descriptor from requested queue element Transfer Descriptor
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor
  \return                  Pointer to next queue element Transfer Descriptor, no next or function failed
               value > 0:  pointer to next queue element Transfer Descriptor
               value = 0:  no next queue element Transfer Descriptor exists or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_GetNext_qTD (const USBH_EHCI_qTD *ptr_qTD) {
  uint32_t addr;

  if (ptr_qTD == NULL) { return NULL; }

  if (!(ptr_qTD->DW0.T)) {              // If terminate is 0
    addr = (uint32_t)ptr_qTD->DW0.NextPtr << 5;
    return (USBH_EHCI_qTD *)addr;
  }

  return NULL;
}

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_GetPrevious (uint8_t ctrl, const USBH_EHCI_qTD *ptr_qTD)
  \brief       Get previous linked queue element Transfer Descriptor linked to this one (before it)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor
  \return                  Pointer to previous linked queue element Transfer Descriptor, no previous or function failed
               value > 0:  pointer to previous linked queue element Transfer Descriptor
               value = 0:  no previous linked queue element Transfer Descriptor exists or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_GetPrevious (uint8_t ctrl, const USBH_EHCI_qTD *ptr_qTD) {
  USBH_EHCI_qTD *ptr_prev_qTD;
  uint32_t       i;

  if (ptr_qTD == NULL) { return NULL; }

  ptr_prev_qTD = (USBH_EHCI_qTD *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qTD);

  if (ptr_prev_qTD == NULL) { return NULL; }

  for (i = 0U; i < (usbh_ehci_ptr[ctrl])->max_qTD; i++) {
    if (ptr_qTD == USBH_EHCI_qTD_GetNext_qTD(ptr_prev_qTD)) {
      return ptr_prev_qTD;
    }
    ptr_prev_qTD++;
  }

  return NULL;
}

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_GetNext_qH (const USBH_EHCI_qH *ptr_qH)
  \brief       Get next queue element Transfer Descriptor for queue Head
  \param[in]   ptr_qH      Pointer to queue Head
  \return                  Pointer to next queue element Transfer Descriptor for queue Head, no next or function failed
               value > 0:  pointer to next queue element Transfer Descriptor for queue Head
               value = 0:  no next queue element Transfer Descriptor exists for queue Head or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_GetNext_qH (const USBH_EHCI_qH *ptr_qH) {
  uint32_t addr;

  if (ptr_qH == NULL) { return NULL; }

  if (ptr_qH->DW4.T == 0U) {            // If terminate is 0
    addr = (uint32_t)ptr_qH->DW4.NextPtr << 5;
    return (USBH_EHCI_qTD *)addr;
  }

  return NULL;
}

/**
  \fn          bool USBH_EHCI_qTD_IsLinkedIn_qH (const USBH_EHCI_qH *ptr_qH, const USBH_EHCI_qTD *ptr_qTD)
  \brief       Check if Transfer Descriptor is already linked in queue Head
  \param[in]   ptr_qH      Pointer to queue Head
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor
  \return      true = already linked, false = not linked

*/
static bool USBH_EHCI_qTD_IsLinkedIn_qH (const USBH_EHCI_qH *ptr_qH, const USBH_EHCI_qTD *ptr_qTD) {
  USBH_EHCI_qTD *ptr_curr_qTD;
  
  if (ptr_qH  == NULL) { return NULL; }
  if (ptr_qTD == NULL) { return NULL; }

  if ((ptr_qH->DW4.T == 0U) && ((ptr_qH->DW4.NextPtr << 5) != 0U)) {
    ptr_curr_qTD = (USBH_EHCI_qTD *)(ptr_qH->DW4.NextPtr << 5);
    while ((ptr_curr_qTD->DW0.T == 0U) && ((ptr_curr_qTD->DW0.NextPtr << 5) != 0U)) {
      if ((((uint32_t)ptr_curr_qTD->DW0.NextPtr) << 5) == (uint32_t)ptr_qTD) {
        return true;
      }
      ptr_curr_qTD = (USBH_EHCI_qTD *)(ptr_qH->DW4.NextPtr << 5);
    }
  }

  return false;
}

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_GetLast (const USBH_EHCI_qH *ptr_qH)
  \brief       Get last queue element Transfer Descriptor for queue Head
  \param[in]   ptr_qH      Pointer to queue Head
  \return                  Pointer to last queue element Transfer Descriptor, no last or function failed
               value > 0:  pointer to last queue element Transfer Descriptor
               value = 0:  no last queue element Transfer Descriptor exists or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_GetLast (const USBH_EHCI_qH *ptr_qH) {
  USBH_EHCI_qTD *ptr_qTD;
  USBH_EHCI_qTD *ptr_next_qTD;

  if (ptr_qH == NULL) { return NULL; }

  ptr_qTD = USBH_EHCI_qTD_GetNext_qH(ptr_qH);

  if (ptr_qTD == NULL) { return NULL; }

  do {
    ptr_next_qTD = USBH_EHCI_qTD_GetNext_qTD(ptr_qTD);
    ptr_qTD = ptr_next_qTD;
  } while (ptr_next_qTD != NULL);

  return ptr_qTD;
}

/**
  \fn          USBH_EHCI_qTD *USBH_EHCI_qTD_Clear (USBH_EHCI_qTD *ptr_qTD)
  \brief       Clear queue element Transfer Descriptor and return pointer to next linked queue element Transfer Descriptor
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor to be cleared
  \return                  Pointer to next linked queue element Transfer Descriptor, no next or function failed
               value > 0:  pointer to next linked queue element Transfer Descriptor
               value = 0:  no next linked queue element Transfer Descriptor exists or function failed
*/
static USBH_EHCI_qTD *USBH_EHCI_qTD_Clear (USBH_EHCI_qTD *ptr_qTD) {
  USBH_EHCI_qTD *ptr_next_qTD;

  if (ptr_qTD == NULL) { return NULL; }

  ptr_next_qTD = USBH_EHCI_qTD_GetNext_qTD(ptr_qTD);
  memset (ptr_qTD, 0, sizeof(USBH_EHCI_qTD));
  ptr_qTD->DW0.T = 1U;                  // Set Next qTD pointer T bit to 1
  ptr_qTD->DW1.T = 1U;                  // Set Alternate Next qTD pointer T bit to 1

  return ptr_next_qTD;
}

/**
  \fn          void USBH_EHCI_qTD_ClearAll (uint8_t ctrl)
  \brief       Clear all queue element Transfer Descriptors
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_EHCI_qTD_ClearAll (uint8_t ctrl) {
  USBH_EHCI_qTD *ptr_qTD;
  uint32_t       i;

  ptr_qTD = (USBH_EHCI_qTD *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qTD);
  if (ptr_qTD != NULL) {
    for (i = 0U; i < (usbh_ehci_ptr[ctrl])->max_qTD; i++) {
      (void)USBH_EHCI_qTD_Clear (ptr_qTD);
      ptr_qTD++;
    }
  }
}

/**
  \fn          bool USBH_EHCI_qTD_Enqueue (uint8_t              ctrl,
                                           USBH_TransferInfo_t *ptr_TI,
                                           USBH_EHCI_qH        *ptr_qH,
                                           USBH_EHCI_qTD       *ptr_qTD,
                                           uint32_t             packet,
                                           uint8_t             *data,
                                           uint32_t             num)
  \brief       Enqueue Control/Bulk/Interrupt Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_TI      Pointer to Transfer Info
  \param[in]   ptr_qH      Pointer to queue Head
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor to enqueue
  \param[in]   packet      Packet information
  \param[in]   data        Pointer to buffer with data to send or for data to receive
  \param[in]   num         Number of data bytes to transfer
  \return      true = success, false = fail
*/
static bool USBH_EHCI_qTD_Enqueue (uint8_t ctrl, USBH_TransferInfo_t *ptr_TI, USBH_EHCI_qH *ptr_qH, USBH_EHCI_qTD *ptr_qTD, uint32_t packet, uint8_t *data, uint32_t num) {
  USBH_EHCI_qTD *ptr_last_qTD;
  uint32_t       sz;

  (void)ctrl;

  if (ptr_qH  == NULL) { return false; }
  if (ptr_qTD == NULL) { return false; }

  // Fill queue Element Transfer Descriptor fields
  switch (packet & ARM_USBH_PACKET_TOKEN_Msk) {
    case ARM_USBH_PACKET_IN:
      ptr_qTD->DW2.PID = 1U;            // PID Code = IN Token
      break;
    case ARM_USBH_PACKET_OUT:
      ptr_qTD->DW2.PID = 0U;            // PID Code = OUT Token
      break;
    case ARM_USBH_PACKET_SETUP:
      ptr_qTD->DW2.PID = 2U;            // PID Code = SETUP Token
      ptr_qH->DW6.DT   = 0U;            // Force Data Toggle for SETUP packet to DATA0
      break;
    case ARM_USBH_PACKET_PING:
      ptr_qTD->DW2.Status |= 1U;        // Do PING
      break;
    default:
      return false;
  }
  switch (packet & ARM_USBH_PACKET_DATA_Msk) {
    case ARM_USBH_PACKET_DATA0:
      ptr_qH->DW6.DT = 0U;
      break;
    case ARM_USBH_PACKET_DATA1:
      ptr_qH->DW6.DT = 1U;
      break;
    default:                            // Keep previous data toggle
      break;
  }

  ptr_qTD->DW2.TBT    = (uint16_t)num;  // Total Bytes to Transfer
  ptr_qTD->DW2.IOC    = 1U;             // Interrupt On Complete
  ptr_qTD->DW2.C_Page = 0U;             // Current Page
  ptr_qTD->DW2.CERR   = 3U;             // Error Counter
  ptr_qTD->DW0.T      = 1U;
  ptr_qTD->DW1.T      = 1U;
  if (num == 0U) {                      // For 0 size packet
    ptr_qTD->DW3.CurOfs = 0U;
    ptr_qTD->DW3.BufPtr = 0U;
  } else if (num <= 16536U) {
    ptr_qTD->DW3.CurOfs = (uint16_t)((uint32_t)data & 0xFFFU);
    ptr_qTD->DW3.BufPtr =            (uint32_t)data >> 12;
    sz = (((uint32_t)(data) & 0xFFFU) + num);
    if (sz >       (1U*4096U)) { ptr_qTD->DW4.BufPtr = ptr_qTD->DW3.BufPtr + 1U;
      if (sz >     (2U*4096U)) { ptr_qTD->DW5.BufPtr = ptr_qTD->DW3.BufPtr + 2U;
        if (sz >   (3U*4096U)) { ptr_qTD->DW6.BufPtr = ptr_qTD->DW3.BufPtr + 3U;
          if (sz > (4U*4096U)) { ptr_qTD->DW7.BufPtr = ptr_qTD->DW3.BufPtr + 4U;
          }
        }
      }
    }
  } else {
    return false;
  }

  if ((ptr_qH->DW6.Status & 0x40U) != 0U) {     // If qH status is Halted
    ptr_qH->DW6.Status = 0U;            // Clear Halted status
  }
  ptr_qTD->DW2.Status = 0x80U;          // Status Active bit set to 1
  ptr_TI->active = 1U;                  // Set active bit in Transfer Info

  // Check if qTD is already linked in qH, if it is not we need to link it in
  if (USBH_EHCI_qTD_IsLinkedIn_qH(ptr_qH, ptr_qTD) == false) {
    ptr_last_qTD = USBH_EHCI_qTD_GetLast(ptr_qH);
    if (ptr_last_qTD != NULL) {         // If there are qTDs linked to qH
                                        // Set Next qTD pointer
      ptr_last_qTD->DW0.NextPtr = (uint32_t)(ptr_qTD) >> 5;
      ptr_last_qTD->DW0.T       = 0U;   // T-bit = 0 => activate transfer described by TD
    } else {
                                        // Set Next qTD pointer
      ptr_qH->DW4.NextPtr       = (uint32_t)(ptr_qTD) >> 5;
      ptr_qH->DW4.T             = 0U;   // T-bit = 0 => activate transfer described by TD
    }
  }

  return true;
}

/**
  \fn          bool USBH_EHCI_IsoTransferActivate (uint8_t              ctrl,
                                                   USBH_TransferInfo_t *ptr_TI,
                                                   void                *ptr_pe,
                                                   uint32_t             packet,
                                                   uint8_t             *data,
                                                   uint32_t             num)
  \brief       Activate Isochronous Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_TI      Pointer to Transfer Info
  \param[in]   ptr_pe      Pointer to Periodic Element (iTD/siTD)
  \param[in]   data        Pointer to buffer with data to send or for data to receive
  \param[in]   num         Number of data bytes to transfer
  \return      true = success, false = fail
*/
static bool USBH_EHCI_IsoTransferActivate (uint8_t ctrl, USBH_TransferInfo_t *ptr_TI, void *ptr_pe, uint8_t *data, uint32_t num) {
  USBH_EHCI_siTD    *ptr_siTD;
  USBH_EHCI_siTD    *ptr_curr_siTD;
  uint32_t          *ptr_uint32_t;
  uint32_t           pfl_index, ti_index;

  if (ptr_TI == NULL) { return false; }
  if (ptr_pe == NULL) { return false; }

  switch (((USBH_EHCI_COMMON *)ptr_pe)->DW0.Typ) {
    case USBH_EHCI_ELEMENT_TYPE_iTD:
      // High-speed isochronous transfers not supported
      return false;
    case USBH_EHCI_ELEMENT_TYPE_qH:
      return false;
    case USBH_EHCI_ELEMENT_TYPE_siTD:

      ptr_siTD = (USBH_EHCI_siTD *)ptr_pe;

      if (ptr_TI->ptr_pipe == (uint32_t *)ptr_siTD) {
        ti_index = 0U;
      } else {
        ti_index = 1U;
      }

      // Load transfer related parameters into siTD structure
      ptr_siTD->DW0.T    = 1U;              // Link to next is invalid
      ptr_siTD->DW0.NextLinkPtr = 0U;       // Next Link Pointer is invalid (0)
      ptr_siTD->DW3.TBT  = (uint16_t)num;   // Total Bytes to Transfer
      ptr_siTD->DW3.IOC  = 0U;              // Clear Interrupt On Complete
      ((USBH_EHCI_siTD_DW *)ptr_siTD)->DW[4] = (uint32_t)data;  // Set DW4.BufPtr and DW4.CurOfs
      ptr_siTD->DW5.BufPtr = ptr_siTD->DW4.BufPtr + 1U;
      ptr_siTD->DW3.Status = 0x80U;         // Status Active bit set to 1

      // Beginning of the Periodic Frame List
      ptr_uint32_t = (uint32_t *)((usbh_ehci_ptr[ctrl])->ptr_PFL);

      if (ptr_TI->active == 0U) {
        // If this is first iso transfer, offset it by 5 frames from current frame
        pfl_index = ((usbh_ehci_reg_ptr[ctrl]->FRINDEX) >> 3) + 5U;
      } else {
        // Otherwise increment from last frame number
        pfl_index = ptr_TI->iso_last_frame_index + ptr_TI->iso_interval;
      }

      // pfl_index is Periodic Frame List entry index for new transfer
      // ptr_uint32_t pints to Periodic Frame List entry for new transfer
      pfl_index &= (pfl_size[ctrl] - 1U);
      ptr_uint32_t += pfl_index;
      ptr_TI->iso_last_frame_index = pfl_index;
      ptr_TI->iso_frame_index[ti_index] = pfl_index;

      // Insert siTD into PFL, if entry already exists append this new transfer to 
      // last existing isochronous transfer
      if (((*ptr_uint32_t & 0x1FU) == 4U) &&    // If T bit is 0 and Type is siTD, meaning it is valid entry
          ((*ptr_uint32_t & (~0x1FU)) != 0U)) { // and if NextLinkPointer is != 0
        // Find last iso entry and add new one behind it
        ptr_curr_siTD = (USBH_EHCI_siTD *)(*ptr_uint32_t & ~0x1FU);
        while ((ptr_curr_siTD->DW0.NextLinkPtr != 0U) && 
               (ptr_curr_siTD->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_siTD) && 
               (ptr_curr_siTD->DW0.T == 0U)) {
          ptr_curr_siTD = (USBH_EHCI_siTD *)(ptr_curr_siTD->DW0.NextLinkPtr << 5);
        }
        // Add new siTD after ptr_curr_siTD which is currently last siTD
        ptr_siTD->DW0.NextLinkPtr = ptr_curr_siTD->DW0.NextLinkPtr;
        ptr_siTD->DW0.T = ptr_curr_siTD->DW0.T;
        ptr_curr_siTD->DW0.NextLinkPtr = ((uint32_t)ptr_siTD) >> 5;
        ptr_curr_siTD->DW0.T = 0U;
      } else {
        // If this is first siTD entry in frame
        // Link existing PFL entry pointer to this siTD
        ptr_siTD->DW0.NextLinkPtr = *ptr_uint32_t >> 5;
        ptr_siTD->DW0.T = (*ptr_uint32_t & 1U);

        // Set PFL entry to point to this new siTD
        *ptr_uint32_t = (uint32_t)ptr_siTD | 4U;
      }
      // Enable only IoC on last periodic transfer, remove on all previous
      USBH_EHCI_IOC(ptr_uint32_t);

      ptr_TI->active = 1U;                  // Set active status in Transfer Info
      break;
    case USBH_EHCI_ELEMENT_TYPE_FSTN:
      return false;
  }

  return true;
}

/**
  \fn          bool USBH_EHCI_qTD_Dequeue (uint8_t        ctrl,
                                           USBH_EHCI_qH  *ptr_qH,
                                           USBH_EHCI_qTD *ptr_qTD)
  \brief       Dequeue Control/Bulk/Interrupt Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to queue Head
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor to dequeue
  \return      true = success, false = fail
*/
static bool USBH_EHCI_qTD_Dequeue (uint8_t ctrl, USBH_EHCI_qH *ptr_qH, USBH_EHCI_qTD *ptr_qTD) {
  USBH_EHCI_qTD *ptr_prev_qTD;
  USBH_EHCI_qTD *ptr_next_qTD;
  uint8_t        was_active;

  if (ptr_qH  == NULL) { return false; }
  if (ptr_qTD == NULL) { return false; }

  was_active = 0U;
  if ((ptr_qH->DW6.Status & (1UL << 7)) != 0U) {
    was_active = 1U;
    if (ptr_qH->DW2.ISM != 0U) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, false);
    } else {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    false);
    }
    ptr_qH->DW6.Status = 0U;
    while ((ptr_qH->DW6.Status & (1UL << 7)) != 0U) { __NOP(); }        // Wait for deactivate
  }

  ptr_next_qTD = USBH_EHCI_qTD_Clear (ptr_qTD);
  ptr_prev_qTD = USBH_EHCI_qTD_GetPrevious (ctrl, ptr_qTD);
  if ((ptr_prev_qTD != NULL) && (ptr_next_qTD != NULL)) {       // If previous and next TD exists
    ptr_prev_qTD->DW0.NextPtr = (uint32_t)ptr_next_qTD >> 5;
    ptr_prev_qTD->DW0.T = 0U;
    ptr_prev_qTD->DW1.T = 1U;
  } else if (ptr_prev_qTD != NULL) {    // If only previous TD exists
    ptr_prev_qTD->DW0.T = 1U;
    ptr_prev_qTD->DW1.T = 1U;
  } else if (ptr_next_qTD != NULL) {    // If only next TD exists
    ptr_qH->DW4.NextPtr = (uint32_t)(ptr_next_qTD) >> 5;
    ptr_qH->DW4.T       = 0U;           // T-bit = 0 => activate transfer described by TD
  } else {
    // both prev and next qTD are NULL, nothing to do
  }
  if (was_active == 1U) {
    if (ptr_qH->DW2.ISM != 0U) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);
    } else {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    true);
    }
  }

  return true;
}

/**
  \fn          USBH_EHCI_qH *USBH_EHCI_qH_Clear (USBH_EHCI_qH *ptr_qH)
  \brief       Clear queue Head and return pointer to next linked queue Head
  \param[in]   ptr_qH      Pointer to queue Head
  \return                  Pointer to next linked queue Head, no next or function failed
               value > 0:  pointer to next linked queue Head
               value = 0:  no next linked queue Head exists or function failed
*/
static USBH_EHCI_qH *USBH_EHCI_qH_Clear (USBH_EHCI_qH *ptr_qH) {
  USBH_EHCI_qH *ptr_next_qH;
  uint32_t      addr;

  if (ptr_qH == NULL) { return NULL; }

  if (ptr_qH->DW0.T == 0U) {
    addr = (uint32_t)ptr_qH->DW0.HorizLinkPtr << 5;
    ptr_next_qH = (USBH_EHCI_qH *)addr;
  } else {
    ptr_next_qH =  NULL;
  }

  memset (ptr_qH, 0, sizeof(USBH_EHCI_qH));
  ptr_qH->DW4.T = 1U;                   // Set Next qTD pointer T bit to 1
  ptr_qH->DW5.T = 1U;                   // Set Alternate Next qTD pointer T bit to 1

  return ptr_next_qH;
}

/**
  \fn          void *USBH_EHCI_PE_Clear (void *ptr_pe)
  \brief       Clear Periodic Element (qH/iTD/siTD) and return pointer to next linked Periodic Element
  \param[in]   ptr_PE      Pointer to qH/iTD/siTD
  \return                  Pointer to next linked qH/iTD/siTD, no next or function failed
               value > 0:  pointer to next linked qH/iTD/siTD
               value = 0:  no next linked qH/iTD/siTD exists or function failed
*/
static void *USBH_EHCI_PE_Clear (void *ptr_pe) {
  USBH_EHCI_COMMON *ptr_curr;
  USBH_EHCI_COMMON *ptr_next;
  uint32_t          len = 0U;

  if (ptr_pe == NULL) { return NULL; }

  ptr_curr = (USBH_EHCI_COMMON *)ptr_pe;

  if (ptr_curr->DW0.T == 0U) {
    ptr_next = (USBH_EHCI_COMMON *)(*((uint32_t *)ptr_pe) & 0xFFFFFFE0U);
  } else {
    ptr_next = NULL;
  }

  switch (ptr_curr->DW0.Typ) {
    case USBH_EHCI_ELEMENT_TYPE_iTD:
      len = sizeof(USBH_EHCI_iTD);
      break;
    case USBH_EHCI_ELEMENT_TYPE_qH:
      len = sizeof(USBH_EHCI_qH);
      break;
    case USBH_EHCI_ELEMENT_TYPE_siTD:
      len = sizeof(USBH_EHCI_siTD);
      break;
    case USBH_EHCI_ELEMENT_TYPE_FSTN:
      len = sizeof(USBH_EHCI_FSTN);
      break;
  }

  memset (ptr_curr, 0, len);

  return ptr_next;
}

/**
  \fn          void USBH_EHCI_qH_ClearAll (uint8_t ctrl)
  \brief       Clear all queue Heads
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_EHCI_qH_ClearAll (uint8_t ctrl) {
  USBH_EHCI_qH *ptr_qH;
  uint32_t      cnt;

  ptr_qH = (USBH_EHCI_qH *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qH); if (ptr_qH == NULL) { return; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qH;
  while (cnt != 0U) {
    cnt--;
    memset (ptr_qH, 0, sizeof(USBH_EHCI_qH));
    ptr_qH->DW4.T = 1U;                 // Set Next qTD pointer T bit to 1
    ptr_qH->DW5.T = 1U;                 // Set Alternate Next qTD pointer T bit to 1
    ptr_qH++;
  }
}

/**
  \fn          USBH_EHCI_qH *USBH_EHCI_qH_GetFree (uint8_t ctrl)
  \brief       Get free queue Head
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free queue Head, no free or function failed
               value > 0:  pointer to first free queue Head
               value = 0:  no free queue Head exists or function failed
*/
static USBH_EHCI_qH *USBH_EHCI_qH_GetFree (uint8_t ctrl) {
  USBH_EHCI_qH *ptr_qH;
  uint32_t      cnt;

  ptr_qH = (USBH_EHCI_qH *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qH); if (ptr_qH == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qH;
  while (cnt != 0U) {
    cnt--;
    if (ptr_qH->DW1.MaxPcktLen == 0U) { return ptr_qH; }
    ptr_qH++;
  }

  return NULL;
}

/**
  \fn          void *USBH_EHCI_PE_GetFree (uint8_t ctrl, uint8_t type)
  \brief       Get free Periodic Element (qH/iTD/siTD)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   type        Type of Periodic Element
  \return                  Pointer to first free qH/iTD/siTD, no next or function failed
               value > 0:  pointer to first free qH/iTD/siTD
               value = 0:  no free qH/iTD/siTD exists or function failed
*/
static void *USBH_EHCI_PE_GetFree (uint8_t ctrl, uint8_t type) {
  USBH_EHCI_qH   *ptr_qH;
  USBH_EHCI_iTD  *ptr_iTD;
  USBH_EHCI_siTD *ptr_siTD;
  uint32_t        cnt;

  switch (type & 3U) {
    case USBH_EHCI_ELEMENT_TYPE_iTD:
      ptr_iTD = (USBH_EHCI_iTD *)((usbh_ehci_ptr[ctrl])->ptr_iTD);
      if (ptr_iTD == NULL) { return NULL; }
      cnt = (usbh_ehci_ptr[ctrl])->max_iTD;
      while (cnt != 0U) {
        cnt--;
        if (ptr_iTD->DW9.DevAddr == 0U) { return ptr_iTD; }
        ptr_iTD++;
      }
      break;
    case USBH_EHCI_ELEMENT_TYPE_qH:
      ptr_qH = (USBH_EHCI_qH *)((usbh_ehci_ptr[ctrl])->ptr_qH);
      if (ptr_qH == NULL) { return NULL; }
      cnt = (usbh_ehci_ptr[ctrl])->max_qH;
      while (cnt != 0U) {
        cnt--;
        if (ptr_qH->DW1.MaxPcktLen == 0U) { return ptr_qH; }
        ptr_qH++;
      }
      break;
    case USBH_EHCI_ELEMENT_TYPE_siTD:
      ptr_siTD = (USBH_EHCI_siTD *)((usbh_ehci_ptr[ctrl])->ptr_siTD);
      if (ptr_siTD == NULL) { return NULL; }
      cnt = (usbh_ehci_ptr[ctrl])->max_siTD;
      while (cnt != 0U) {
        cnt--;
        if (ptr_siTD->DW1.DevAddr == 0U) { return ptr_siTD; }
        ptr_siTD++;
      }
      break;
    case USBH_EHCI_ELEMENT_TYPE_FSTN:
      break;
  }

  return NULL;
}

/**
  \fn          USBH_EHCI_qH *USBH_EHCI_qH_GetPrevious (uint8_t ctrl, USBH_EHCI_qH *ptr_qH)
  \brief       Get previous queue Head linked to this one (before it)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to queue Head to start search from
  \return                  Pointer to previous queue Head, no previous or function failed
               value > 0:  pointer to previous queue Head
               value = 0:  no previous queue Head exists or function failed
*/
static USBH_EHCI_qH *USBH_EHCI_qH_GetPrevious (uint8_t ctrl, USBH_EHCI_qH *ptr_qH) {
  USBH_EHCI_qH *ptr_prev_qH;
  uint32_t      cnt;

  if (ptr_qH == NULL) { return NULL; }

  ptr_prev_qH = (USBH_EHCI_qH *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qH); if (ptr_prev_qH == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qH;
  while (cnt != 0U) {
    cnt--;
    if ((ptr_prev_qH->DW0.T == 0U) && (((uint32_t)ptr_prev_qH->DW0.HorizLinkPtr << 5) == (uint32_t)ptr_qH)) { return ptr_prev_qH; }
    ptr_prev_qH++;
  }

  return NULL;
}

/**
  \fn          USBH_EHCI_qH *USBH_EHCI_qH_GetLast (USBH_EHCI_qH *ptr_qH)
  \brief       Get last queue Head starting from requested one
  \param[in]   ptr_qH      Pointer to queue Head to start search from
  \return                  Pointer to last chained queue Head starting from requested one, no last or function failed
               value > 0:  pointer to last chained queue Head starting from requested one
               value = 0:  no last last chained queue Head starting from requested one or function failed
*/
static USBH_EHCI_qH *USBH_EHCI_qH_GetLast (USBH_EHCI_qH *ptr_qH) {
  USBH_EHCI_qH *ptr_qH_;
  uint32_t      addr;

  if (ptr_qH == NULL) { return NULL; }
  ptr_qH_ = ptr_qH;

  addr = (uint32_t)ptr_qH_->DW0.HorizLinkPtr << 5;
  while ((ptr_qH_->DW0.T == 0U) && (ptr_qH_->DW0.HorizLinkPtr != 0U) && ((((USBH_EHCI_qH *)addr)->DW1.H) == 0U)) {
    ptr_qH_ = (USBH_EHCI_qH *)addr;
    addr    = (uint32_t)ptr_qH_->DW0.HorizLinkPtr << 5;
  }

  return ptr_qH_;
}

/**
  \fn          USBH_EHCI_qH *USBH_EHCI_qH_Unlink (USBH_EHCI_qH *ptr_first_qH, USBH_EHCI_qH *ptr_qH)
  \brief       Unlink queue Head from queue Head linked list and
               return pointer to beginning of the linked list queue Head
  \param[in]   ptr_first_qH  Pointer to first queue Head in the linked list
  \param[in]   ptr_qH        Pointer to queue Head
  \return                    Pointer to beginning of the linked list, queue Head not found or function failed
               value > 0:    pointer to beginning of the linked list
               value = 0:    queue Head not found or function failed
*/
static USBH_EHCI_qH *USBH_EHCI_qH_Unlink (USBH_EHCI_qH *ptr_first_qH, USBH_EHCI_qH *ptr_qH) {
  const USBH_EHCI_qH *ptr_qH_;
        USBH_EHCI_qH *ptr_first_qH_, *ptr_curr_qH, *ptr_next_qH;
        uint32_t      addr;

  if (ptr_first_qH->DW0.T == 0U) {
    addr          = (uint32_t)ptr_first_qH & ~0x1FU;
    ptr_first_qH_ = (USBH_EHCI_qH *)addr;
  } else {
    ptr_first_qH_ = NULL;
  }
  addr         = (uint32_t)ptr_qH & ~0x1FU;
  ptr_qH_      = (USBH_EHCI_qH *)addr;

  if ((ptr_first_qH_ == NULL) || (ptr_qH_ == NULL)) { return ptr_first_qH_; }

  ptr_curr_qH  =  ptr_first_qH_;
  if (ptr_curr_qH->DW0.T == 0U) {
    addr        = (uint32_t)ptr_curr_qH->DW0.HorizLinkPtr << 5;
    ptr_next_qH = (USBH_EHCI_qH *)addr;
  } else {
    ptr_next_qH = NULL;
  }

  // Handle special case if first element is one that needs to be unlinked
  if (ptr_curr_qH == ptr_qH_) {         // If first element is requested one
    if ((ptr_next_qH == ptr_curr_qH) || // If element is linked to itself or
        (ptr_next_qH == NULL)) {        // if this is last element in list
      return NULL;                      // Start of the list is now 0
    } else {                            // There is a valid next element
      ptr_first_qH_ = ptr_next_qH;
      ptr_curr_qH   = ptr_next_qH;
      if (ptr_curr_qH->DW0.T == 0U) {
        addr        = (uint32_t)ptr_curr_qH->DW0.HorizLinkPtr << 5;
        ptr_next_qH = (USBH_EHCI_qH *)addr;
      } else {
        ptr_next_qH = NULL;
      }
    }
  }

  while (ptr_next_qH != NULL) {         // While next element is valid
    if (ptr_next_qH == ptr_qH_) {       // If next element is requested one
      if (ptr_next_qH->DW0.T == 0U) {   // If next element exists
        addr        = (uint32_t)ptr_next_qH->DW0.HorizLinkPtr << 5;
        ptr_next_qH = (USBH_EHCI_qH *)addr;
        ptr_curr_qH->DW0.HorizLinkPtr = (uint32_t)(ptr_next_qH) >> 5;
        ptr_curr_qH->DW0.T = ptr_next_qH->DW0.T;
      } else {                          // If next element does not exist make curr last
        ptr_curr_qH->DW0.HorizLinkPtr = 0U;
        ptr_curr_qH->DW0.T = 1U;
      }
      break;
    }
    ptr_curr_qH = ptr_next_qH;
    if (ptr_next_qH->DW0.T == 0U) {
      addr        = (uint32_t)ptr_next_qH->DW0.HorizLinkPtr << 5;
      ptr_next_qH = (USBH_EHCI_qH *)addr;
    } else {
      ptr_next_qH = NULL;
    }
  }

  return ptr_first_qH_;
}

/**
  \fn          void *USBH_EHCI_PE_Unlink (void *ptr_first_pe, void *ptr_pe)
  \brief       Unlink Periodic Element (qH/iTD/siTD) from Periodic Element (qH/iTD/siTD) linked list and
               return pointer to beginning of the linked list Periodic Element (qH/iTD/siTD)
  \param[in]   ptr_first_pe  Pointer to first Periodic Element (qH/iTD/siTD) in the linked list
  \param[in]   ptr_pe        Pointer to Periodic Element (qH/iTD/siTD)
  \return                    Pointer to beginning of the linked list, Periodic Element (qH/iTD/siTD) not found or function failed
               value > 0:    pointer to beginning of the linked list
               value = 0:    Periodic Element (qH/iTD/siTD) not found or function failed
*/
static void *USBH_EHCI_PE_Unlink (void *ptr_first_pe, void *ptr_pe) {
  USBH_EHCI_COMMON *ptr_curr, *ptr_next;
  void             *ptr_new_first;

  if ((ptr_first_pe == NULL) || (ptr_pe == NULL)) {
    return NULL;
  }

  ptr_curr      = (USBH_EHCI_COMMON *)ptr_first_pe;
  ptr_new_first = ptr_first_pe;

  if (ptr_first_pe == ptr_pe) {
    // If first element should be unlinked
    if (ptr_curr->DW0.T == 0U) {
      ptr_new_first = (void *)(ptr_curr->DW0.LinkPtr << 5);
      ptr_curr->DW0.T = 1U;
    } else {
      ptr_new_first = NULL;
    }
  } else {
    // If we need to find element to unlink
    while ((ptr_curr->DW0.T == 0U) && (ptr_curr->DW0.LinkPtr != 0U) && ((((uint32_t)ptr_curr->DW0.LinkPtr) << 5) != (uint32_t)ptr_pe)) {
      ptr_curr = (USBH_EHCI_COMMON *)(ptr_curr->DW0.LinkPtr << 5);
    }
    if ((ptr_curr->DW0.T == 0U) && (ptr_curr->DW0.LinkPtr != 0U) && ((((uint32_t)ptr_curr->DW0.LinkPtr) << 5) == (uint32_t)ptr_pe)) {
      // If next of current shows to the one we want to remove
      ptr_next = (USBH_EHCI_COMMON *)(ptr_curr->DW0.LinkPtr << 5);
      ptr_curr->DW0.LinkPtr = ptr_next->DW0.LinkPtr;
      ptr_curr->DW0.T       = ptr_next->DW0.T;
    }
  }

  return ptr_new_first;
}

/**
  \fn          uint32_t USBH_EHCI_qH_CountIntEntries (uint8_t ctrl, USBH_EHCI_qH *ptr_qH)
  \brief       Count number of times Interrupt queue Head is found in Periodic Frame List
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to Interrupt queue Head
  \return                  Number of times Interrupt queue Head is found in Periodic Frame List
*/
static uint32_t USBH_EHCI_qH_CountIntEntries (uint8_t ctrl, const USBH_EHCI_qH *ptr_qH) {
  const uint32_t     *ptr_uint32_t;
  const USBH_EHCI_qH *ptr_tmp_qH;
        uint32_t      pfl_sz;
        uint32_t      i, cnt;
        uint32_t      addr;

  if (ptr_qH == NULL) { return 0U; }

  pfl_sz = USBH_EHCI_PFL_GetSize(ctrl);
  cnt    = 0U;

  ptr_uint32_t = (usbh_ehci_ptr[ctrl])->ptr_PFL;
  for (i = 0U; i < pfl_sz; i++) {
    if ((*ptr_uint32_t & 1U) == 0U) {   // If T is cleared then this is a valid pointer
      addr       = *ptr_uint32_t & (~0x1FU);
      ptr_tmp_qH = (USBH_EHCI_qH *)addr;
      while (ptr_tmp_qH != NULL) {
        if (ptr_tmp_qH == ptr_qH) { cnt++; }
        if (ptr_tmp_qH->DW0.T == 0U) {
          addr       =  (uint32_t)ptr_qH->DW0.HorizLinkPtr << 5;
          ptr_tmp_qH = (USBH_EHCI_qH *)addr;
        } else {
          ptr_tmp_qH =  NULL;
        }
      }
    }
    ptr_uint32_t++;
  }

  return cnt;
}

/**
  \fn          bool USBH_EHCI_qH_ClearIntEntries (uint8_t ctrl, USBH_EHCI_qH *ptr_qH)
  \brief       Clear entries in Periodic Frame List for Interrupt queue Head
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to queue Head
  \return      true = success, false = fail
*/
static bool USBH_EHCI_qH_ClearIntEntries (uint8_t ctrl, USBH_EHCI_qH *ptr_qH) {
  USBH_EHCI_qH *ptr_first_qH;
  uint32_t     *ptr_uint32_t;
  uint32_t      pfl_sz;

  if (ptr_qH == NULL) { return false; }

  pfl_sz       = USBH_EHCI_PFL_GetSize(ctrl); if (pfl_sz == 0U) { return false; }
  ptr_uint32_t = (usbh_ehci_ptr[ctrl])->ptr_PFL;
  while (pfl_sz != 0U) {
    pfl_sz--;
    if ((*ptr_uint32_t & 1U) == 0U) {             // If T is 0, meaning pointer is valid
      ptr_first_qH = USBH_EHCI_qH_Unlink ((USBH_EHCI_qH *)(*ptr_uint32_t), ptr_qH);
      if (ptr_first_qH == NULL) {
        *ptr_uint32_t = 1U;                       // T = 1, no qH linked
      } else {
        *ptr_uint32_t = (*ptr_uint32_t & 0x1FU) | (uint32_t)(ptr_first_qH);
      }
    }
    ptr_uint32_t++;
  }

  return true;
}

/**
  \fn          bool USBH_EHCI_PFL_PE_Remove (uint8_t ctrl, void *ptr_pe)
  \brief       Remove entries from Periodic Frame List for Periodic Element (qH/iTD/siTD)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_pe      Pointer to Periodic Element (qH/iTD/siTD)
  \return      true = success, false = fail
*/
static bool USBH_EHCI_PFL_PE_Remove (uint8_t ctrl, void *ptr_pe) {
  void     *ptr_new_first;
  uint32_t *ptr_uint32_t;
  uint32_t  pfl_sz;

  if (ptr_pe == NULL) { return false; }

  pfl_sz       = USBH_EHCI_PFL_GetSize(ctrl); if (pfl_sz == 0U) { return false; }
  ptr_uint32_t = (usbh_ehci_ptr[ctrl])->ptr_PFL;
  while (pfl_sz != 0U) {
    pfl_sz--;
    if ((*ptr_uint32_t & 1U) == 0U) {             // If T is 0, meaning pointer is valid
      ptr_new_first = USBH_EHCI_PE_Unlink ((void *)((*ptr_uint32_t) & (~0x1FU)), ptr_pe);
      if (ptr_new_first == NULL) {
        *ptr_uint32_t = 1U;                       // T = 1, no Periodic Element linked
      } else {
        *ptr_uint32_t = (uint32_t)(ptr_new_first);
      }
    }
    ptr_uint32_t++;
  }

  return true;
}

/**
  \fn          uint32_t USBH_EHCI_CalcIntEntriesNum (uint8_t ctrl, uint8_t interval)
  \brief       Calculate number of entries in Periodic Frame List for Interrupt queue Head
               required to achieve requested polling interval
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   interval    Endpoint interval (in SOF or uSOF intervals)
  \return      Number of entries in Periodic Frame List required to achieve requested polling interval
*/
static uint32_t USBH_EHCI_CalcIntEntriesNum (uint8_t ctrl, uint32_t interval) {
  uint32_t pfl_sz;

  pfl_sz = USBH_EHCI_PFL_GetSize(ctrl);

  if (interval > pfl_sz) {              // Special case only 1 entry
    return 1U;
  }
  if (interval <= 1U) {                 // Special case all entries
    return pfl_sz;
  }

  while (((pfl_sz % interval) != 0U) && (interval > 1U)) {
    interval--;
  }
  return (pfl_sz / interval);
}

/**
  \fn          bool USBH_EHCI_qH_SetIntEntries (uint8_t ctrl, USBH_EHCI_qH *ptr_qH, uint8_t ep_speed, uint8_t ep_interval)
  \brief       Set queue Head entries in Periodic Frame List for Interrupt Endpoint
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to queue Head
  \param[in]   ep_speed    Endpoint speed
  \param[in]   ep_interval Endpoint polling interval
  \return      true = success, false = fail
*/
static bool USBH_EHCI_qH_SetIntEntries (uint8_t ctrl, USBH_EHCI_qH *ptr_qH, uint8_t ep_speed, uint8_t ep_interval) {
  USBH_EHCI_qH *ptr_tmp_qH;
  USBH_EHCI_qH *ptr_prev_qH;
  uint32_t     *ptr_uint32_t;
  uint32_t      num, step, interval, i;
  uint32_t      addr;

  if (USBH_EHCI_qH_ClearIntEntries (ctrl, ptr_qH) == false) { return false; }

  if (ep_interval == 0U) {
    interval = 1U;
  } else {
    switch (ep_speed) {
      case ARM_USB_SPEED_LOW:
      case ARM_USB_SPEED_FULL:
        interval = ep_interval;
        break;
      case ARM_USB_SPEED_HIGH:
        if (ep_interval > 16U) {
          // Out of bounds value
          interval = 32768U;
        } else {
          // Convert bInterval to interval in uSOFs
          interval = 2UL << ((uint32_t)ep_interval - 1U);
        }
        break;
      default:
        return false;
    }
  }

  num  = USBH_EHCI_CalcIntEntriesNum (ctrl, interval);
  step = USBH_EHCI_PFL_GetSize (ctrl) / num;

  ptr_prev_qH = NULL;

  // Search for qH which has same number or more entries then requested qH,
  // and if found link requested qH in front of found qH, otherwise link
  // requested qH after last qH
  ptr_uint32_t = (uint32_t *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_PFL);
  addr         = *ptr_uint32_t & (~0x1FU);
  ptr_tmp_qH   = (USBH_EHCI_qH *)addr;
  if ((*ptr_uint32_t & 1U) == 0U) {
    // If starting table pointer is not empty find where new qH should be added
    for (;;) {
      if (USBH_EHCI_qH_CountIntEntries (ctrl, ptr_tmp_qH) >= num) {
        // We need to put new qH in front of current one
        ptr_prev_qH = USBH_EHCI_qH_GetPrevious (ctrl, ptr_tmp_qH);
        ptr_qH->DW0.HorizLinkPtr = (uint32_t)ptr_tmp_qH >> 5;
        ptr_qH->DW0.T = 0U;               // T = 0, not last entry
        if (ptr_prev_qH != NULL) {
          ptr_prev_qH->DW0.HorizLinkPtr = (uint32_t)ptr_qH >> 5;
        }
        break;
      } else if (ptr_tmp_qH->DW0.HorizLinkPtr != 0U) {
        // Go to next qH in chain as we have not found where to put new qH yet
        addr       = (uint32_t)ptr_tmp_qH->DW0.HorizLinkPtr << 5;
        ptr_tmp_qH = (USBH_EHCI_qH *)addr;
      } else {
        // We need to put new qH at the end
        ptr_prev_qH = ptr_tmp_qH;
        ptr_tmp_qH->DW0.HorizLinkPtr = (uint32_t)ptr_qH >> 5;
        ptr_tmp_qH->DW0.T = 0U;         // T = 0, is not last entry any more
        ptr_qH->DW0.T     = 1U;         // T = 1, is last entry
        break;
      }
    }
  }

  // Find any corresponding table pointer that should start with requested qH
  ptr_uint32_t = (uint32_t *)((usbh_ehci_ptr[ctrl])->ptr_PFL);
  for (i = 0U; i < num; i++) {
    if (((*ptr_uint32_t & 1U) == 1U) ||                 // If empty pointer (T == 1) or
         (ptr_prev_qH == NULL)) {                       // if no previous qH
      *ptr_uint32_t = (uint32_t)ptr_qH | (1UL << 1);    // Typ = QH, T = 0
    }
    ptr_uint32_t += step;
  }

  return true;
}

/**
  \fn          void USBH_EHCI_TI_ClearAll (uint8_t ctrl)
  \brief       Clear all Transfer Info entries
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_EHCI_TI_ClearAll (uint8_t ctrl) {

  memset((usbh_ehci_ptr[ctrl])->ptr_TI, 0, (USBH_EHCI_MAX_PIPES)*sizeof(USBH_TransferInfo_t));
}

/**
  \fn          USBH_TransferInfo_t *USBH_EHCI_TI_GetFree (uint8_t ctrl)
  \brief       Get Free Transfer Info entry
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free Transfer Info entry, no free or function failed
               value > 0:  pointer to first free Transfer Info entry
               value = 0:  no free Transfer Info entry or function failed
*/
static USBH_TransferInfo_t *USBH_EHCI_TI_GetFree (uint8_t ctrl) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  ptr_TI = (usbh_ehci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qTD;
  while (cnt != 0U) {
    cnt--;
    if (ptr_TI->ptr_pipe == NULL) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

/**
  \fn          USBH_TransferInfo_t *USBH_EHCI_TI_Find_qTD (uint8_t ctrl, USBH_EHCI_qTD *ptr_qTD)
  \brief       Get Transfer Info entry referring to queue element Transfer Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qTD     Pointer to queue element Transfer Descriptor to find
  \return                  Pointer to Transfer Info entry, no corresponding entry or function failed
               value > 0:  pointer to Transfer Info entry
               value = 0:  no Transfer Info entry found or function failed
*/
static USBH_TransferInfo_t *USBH_EHCI_TI_Find_qTD (uint8_t ctrl, USBH_EHCI_qTD *ptr_qTD) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  if (ptr_qTD == NULL) { return NULL; }

  ptr_TI = (usbh_ehci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qTD;
  while (cnt != 0U) {
    cnt--;
    if (((uint32_t)ptr_TI->ptr_transfer & ~0x1FU) == (uint32_t)ptr_qTD) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

/**
  \fn          USBH_TransferInfo_t *USBH_EHCI_TI_Find_qH (uint8_t ctrl, USBH_EHCI_qH *ptr_qH)
  \brief       Get Transfer Info entry referring to queue Head
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to queue Head to find
  \return                  Pointer to Transfer Info entry, no corresponding entry or function failed
               value > 0:  pointer to Transfer Info entry
               value = 0:  no Transfer Info entry found or function failed
*/
static USBH_TransferInfo_t *USBH_EHCI_TI_Find_qH (uint8_t ctrl, USBH_EHCI_qH *ptr_qH) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  if (ptr_qH == NULL) { return NULL; }

  ptr_TI = (usbh_ehci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qH;
  while (cnt != 0U) {
    cnt--;
    if ((uint32_t)ptr_TI->ptr_pipe == (uint32_t)ptr_qH) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

/**
  \fn          USBH_TransferInfo_t *USBH_EHCI_TI_Find_PE (uint8_t ctrl, void *ptr_pe)
  \brief       Get Transfer Info entry referring to Periodic Element (qH/iTD/siTD)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_pe      Pointer to Periodic Element (qH/iTD/siTD) to find
  \return                  Pointer to Transfer Info entry, no corresponding entry or function failed
               value > 0:  pointer to Transfer Info entry
               value = 0:  no Transfer Info entry found or function failed
*/
static USBH_TransferInfo_t *USBH_EHCI_TI_Find_PE (uint8_t ctrl, void *ptr_pe) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  if (ptr_pe == NULL) { return NULL; }

  ptr_TI = (usbh_ehci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (usbh_ehci_ptr[ctrl])->max_qH;
  while (cnt != 0U) {
    cnt--;
    if ((uint32_t)ptr_TI->ptr_pipe      == (uint32_t)ptr_pe) { return ptr_TI; }
    if ((uint32_t)ptr_TI->iso_ptr_pipe1 == (uint32_t)ptr_pe) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

// Driver functions ***********************************************************

/**
  \fn          ARM_DRIVER_VERSION USBH_HW_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBH0_HW_GetVersion (void) { return usbh_driver_version; }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static ARM_DRIVER_VERSION USBH1_HW_GetVersion (void) { return usbh_driver_version; }
#endif

/**
  \fn          ARM_USBH_CAPABILITIES USBH_HW_GetCapabilities (uint8_t ctrl)
  \brief       Get driver capabilities.
  \param[in]   ctrl  Index of USB Host controller
  \return      \ref ARM_USBH_CAPABILITIES
*/
static ARM_USBH_CAPABILITIES USBH_HW_GetCapabilities (uint8_t ctrl) {
  uint32_t port_num = (usbh_ehci_reg_ptr[ctrl]->HCSPARAMS & 0xFU);

  if (port_num == 0U) {
    port_num = 1U;
  }

  usbh_driver_capabilities[ctrl].port_mask = (1U << port_num) - 1U;

  return usbh_driver_capabilities[ctrl];
}
static ARM_USBH_CAPABILITIES USBH0_HW_GetCapabilities (void)         { return USBH_HW_GetCapabilities (0); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static ARM_USBH_CAPABILITIES USBH1_HW_GetCapabilities (void)         { return USBH_HW_GetCapabilities (1); }
#endif

/**
  \fn          int32_t USBH_HW_Initialize (uint8_t                    ctrl,
                                           ARM_USBH_SignalPortEvent_t cb_port_event,
                                           ARM_USBH_SignalPipeEvent_t cb_pipe_event)
  \brief       Initialize USB Host Interface.
  \param[in]   ctrl           Index of USB Host controller
  \param[in]   cb_port_event  Pointer to \ref ARM_USBH_SignalPortEvent
  \param[in]   cb_pipe_event  Pointer to \ref ARM_USBH_SignalPipeEvent
  \return      \ref execution_status
*/
static int32_t USBH_HW_Initialize (uint8_t ctrl, ARM_USBH_SignalPortEvent_t cb_port_event, ARM_USBH_SignalPipeEvent_t cb_pipe_event) {
  int32_t ret;

  if (ctrl >= USBH_EHCI_TT_INSTANCES) { return ARM_DRIVER_ERROR; }

  usbh_ehci_reg_ptr[ctrl] = (USBH_EHCI_Registers_t *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_EHCI);

  signal_port_event[ctrl] = cb_port_event;
  signal_pipe_event[ctrl] = cb_pipe_event;

  (void)USBH_HW_GetCapabilities(ctrl);

  port_act[ctrl] = 0U;
  pfl_size[ctrl] = USBH_EHCI_PFL_GetSize(ctrl);

  USBH_EHCI_TI_ClearAll (ctrl);

  ret = USBH_EHCI_HW_Initialize((uint8_t)(usbh_ehci_ptr[ctrl])->ctrl, (usbh_ehci_ptr[ctrl])->irq_handler);

  if (ret != 0) {
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_Initialize (ARM_USBH_SignalPortEvent_t cb_port_event, ARM_USBH_SignalPipeEvent_t cb_pipe_event) { return USBH_HW_Initialize (0, cb_port_event, cb_pipe_event); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_Initialize (ARM_USBH_SignalPortEvent_t cb_port_event, ARM_USBH_SignalPipeEvent_t cb_pipe_event) { return USBH_HW_Initialize (1, cb_port_event, cb_pipe_event); }
#endif

/**
  \fn          int32_t USBH_HW_Uninitialize (uint8_t ctrl)
  \brief       De-initialize USB Host Interface.
  \param[in]   ctrl  Index of USB Host controller
  \return      \ref execution_status
*/
static int32_t USBH_HW_Uninitialize (uint8_t ctrl) {
  int32_t ret;

  ret = USBH_EHCI_HW_Uninitialize((uint8_t)(usbh_ehci_ptr[ctrl])->ctrl);

  if (ret != 0) {
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_Uninitialize (void) { return USBH_HW_Uninitialize(0); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_Uninitialize (void) { return USBH_HW_Uninitialize(1); }
#endif

/**
  \fn          int32_t USBH_HW_PowerControl (uint8_t ctrl, ARM_POWER_STATE state)
  \brief       Control USB Host Interface Power.
  \param[in]   ctrl   Index of USB Host controller
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBH_HW_PowerControl (uint8_t ctrl, ARM_POWER_STATE state) {
  int32_t  status;
  uint32_t tout;

  switch (state) {
    case ARM_POWER_OFF:
      usbh_ehci_reg_ptr[ctrl]->USBINTR = 0U;    // Disable all EHCI interrupts
      usbh_ehci_reg_ptr[ctrl]->USBCMD  = 0U;    // Host Controller Stop

      // Wait for stop, max 1 sec
      tout = 10100;
      while ((usbh_ehci_reg_ptr[ctrl]->USBSTS & (1UL << 12)) == 0U){  // While not halted
        if (tout <= 100U) {
          if (tout == 0U) { return ARM_DRIVER_ERROR; }
          (void)osDelay(10U);
        }
        tout--;
      }

      status = USBH_EHCI_HW_PowerControl((uint8_t)(usbh_ehci_ptr[ctrl])->ctrl, 0U);
      if (status != 0) {
        return ARM_DRIVER_ERROR;
      }
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR;

    case ARM_POWER_FULL:
      status = USBH_EHCI_HW_PowerControl((uint8_t)(usbh_ehci_ptr[ctrl])->ctrl, 1U);
      if (status != 0) {
        return ARM_DRIVER_ERROR;
      }

      // Initialize memories
      USBH_EHCI_PFL_Clear   (ctrl);
      USBH_EHCI_qTD_ClearAll(ctrl);
      USBH_EHCI_qH_ClearAll (ctrl);
      USBH_EHCI_TI_ClearAll (ctrl);

      usbh_ehci_reg_ptr[ctrl]->USBCMD |= (1UL << 1);    // Host Controller Reset

      // Wait for reset to finish, max 1 sec
      tout = 10100;
      while ((usbh_ehci_reg_ptr[ctrl]->USBCMD & (1UL << 1)) != 0U){
        if (tout <= 100U) {
          if (tout == 0U) { return ARM_DRIVER_ERROR; }
          (void)osDelay(10U);
        }
        tout--;
      }
      usbh_ehci_reg_ptr[ctrl]->PORTSC[9] = 0x23U;       // Controller Mode=Host Mode, VBPS Hi
      usbh_ehci_reg_ptr[ctrl]->USBINTR   = 7U;          // Enable needed interrupts
      usbh_ehci_reg_ptr[ctrl]->PERIODICLISTBASE = (uint32_t)(usbh_ehci_ptr[ctrl])->ptr_PFL; // PFL

      // Wait for run to activate, max 1 sec
      usbh_ehci_reg_ptr[ctrl]->USBCMD |= 1U;            // Host Controller Run
      tout = 10100;
      while ((usbh_ehci_reg_ptr[ctrl]->USBSTS & (1UL << 12)) != 0U){  // While halted
        if (tout <= 100U) {
          if (tout == 0U) { return ARM_DRIVER_ERROR; }
          (void)osDelay(10U);
        }
        tout--;
      }
      usbh_ehci_reg_ptr[ctrl]->TTCTRL = 0U;             // Hub not used for FS and LS devices
      break;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PowerControl (ARM_POWER_STATE state) { return USBH_HW_PowerControl (0, state); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PowerControl (ARM_POWER_STATE state) { return USBH_HW_PowerControl (1, state); }
#endif

/**
  \fn          int32_t USBH_HW_PortVbusOnOff (uint8_t ctrl, uint8_t port, bool vbus)
  \brief       Root HUB Port VBUS on/off.
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \param[in]   vbus
                - \b false VBUS off
                - \b true  VBUS on
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortVbusOnOff (uint8_t ctrl, uint8_t port, bool vbus) {

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) == 0U) {
    // If port is not available
    return ARM_DRIVER_ERROR;
  }

  if (vbus) {
    usbh_ehci_reg_ptr[ctrl]->PORTSC[port] |=  USBH_EHCI_PORTSC_PP(1);
  } else {
    usbh_ehci_reg_ptr[ctrl]->PORTSC[port] &= ~USBH_EHCI_PORTSC_PP_MSK;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortVbusOnOff (uint8_t port, bool vbus) { return USBH_HW_PortVbusOnOff(0, port, vbus); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PortVbusOnOff (uint8_t port, bool vbus) { return USBH_HW_PortVbusOnOff(1, port, vbus); }
#endif

/**
  \fn          int32_t USBH_HW_PortReset (uint8_t ctrl, uint8_t port)
  \brief       Do Root HUB Port Reset.
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortReset (uint8_t ctrl, uint8_t port) {
  uint32_t tout;

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) == 0U) {
    // If port is not available
    return ARM_DRIVER_ERROR;
  }

  usbh_ehci_reg_ptr[ctrl]->PORTSC[port] &= ~USBH_EHCI_PORTSC_PEC_MSK;    // Clear Port Enable
  usbh_ehci_reg_ptr[ctrl]->PORTSC[port] |=  USBH_EHCI_PORTSC_PR(1);      // Port Reset

  tout = 10U;
  while ((usbh_ehci_reg_ptr[ctrl]->PORTSC[port] & USBH_EHCI_PORTSC_PR_MSK) != 0U){
    if (tout <= 10U) {
      if (tout == 0U) { return ARM_DRIVER_ERROR; }
      (void)osDelay(10U);
    }
    tout--;
  }

  signal_port_event[ctrl](port, ARM_USBH_EVENT_RESET);

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortReset (uint8_t port) { return USBH_HW_PortReset (0, port); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PortReset (uint8_t port) { return USBH_HW_PortReset (1, port); }
#endif

/**
  \fn          int32_t USBH_HW_PortSuspend (uint8_t ctrl, uint8_t port)
  \brief       Suspend Root HUB Port (stop generating SOFs).
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortSuspend (uint8_t ctrl, uint8_t port) {

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) == 0U) {
    // If port is not available
    return ARM_DRIVER_ERROR;
  }

  usbh_ehci_reg_ptr[ctrl]->PORTSC[port] |= USBH_EHCI_PORTSC_SUSP_MSK; // Set Suspend

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortSuspend (uint8_t port) { return USBH_HW_PortSuspend (0, port); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PortSuspend (uint8_t port) { return USBH_HW_PortSuspend (1, port); }
#endif

/**
  \fn          int32_t USBH_HW_PortResume (uint8_t ctrl, uint8_t port)
  \brief       Resume Root HUB Port (start generating SOFs).
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortResume (uint8_t ctrl, uint8_t port) {

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) == 0U) {
    // If port is not available
    return ARM_DRIVER_ERROR;
  }

  usbh_ehci_reg_ptr[ctrl]->PORTSC[port] |= USBH_EHCI_PORTSC_FPR_MSK;  // Force Port Resume

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortResume (uint8_t port) { return USBH_HW_PortResume (0, port); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PortResume (uint8_t port) { return USBH_HW_PortResume (1, port); }
#endif

/**
  \fn          ARM_USBH_PORT_STATE USBH_HW_PortGetState (uint8_t ctrl, uint8_t port)
  \brief       Get current Root HUB Port State.
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \return      Port State \ref ARM_USBH_PORT_STATE
*/
static ARM_USBH_PORT_STATE USBH_HW_PortGetState (uint8_t ctrl, uint8_t port) {
  ARM_USBH_PORT_STATE port_state = { 0U, 0U, 0U, 0U };
  uint32_t            PORTSC_val;

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) == 0U) {
    // If port is not available
    return port_state;
  }

  PORTSC_val  = usbh_ehci_reg_ptr[ctrl]->PORTSC[port];

  if ((PORTSC_val & USBH_EHCI_PORTSC_CCS_MSK) != 0U) {
    port_state.connected = 1U;
  } else {
    port_state.connected = 0U;
  }
  if ((PORTSC_val & USBH_EHCI_PORTSC_OCA_MSK) != 0U) {
    port_state.overcurrent = 1U;
  } else {
    port_state.overcurrent = 0U;
  }
  switch ((PORTSC_val & USBH_EHCI_PORTSC_PSPD_MSK) >> 26) {
    case 1:                             // Low-speed
     port_state.speed = ARM_USB_SPEED_LOW;
     break;
    case 0:                             // Full-speed
     port_state.speed = ARM_USB_SPEED_FULL;
     break;
    case 2:                             // High-speed
     port_state.speed = ARM_USB_SPEED_HIGH;
     break;
    default:
     port_state.speed = ARM_USB_SPEED_LOW;
     break;
  }

  return port_state;
}
static ARM_USBH_PORT_STATE USBH0_HW_PortGetState (uint8_t port) { return USBH_HW_PortGetState (0, port); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static ARM_USBH_PORT_STATE USBH1_HW_PortGetState (uint8_t port) { return USBH_HW_PortGetState (1, port); }
#endif

/**
  \fn          ARM_USBH_PIPE_HANDLE USBH_HW_PipeCreate (uint8_t  ctrl,
                                                        uint8_t  dev_addr,
                                                        uint8_t  dev_speed,
                                                        uint8_t  hub_addr,
                                                        uint8_t  hub_port,
                                                        uint8_t  ep_addr,
                                                        uint8_t  ep_type,
                                                        uint16_t ep_max_packet_size,
                                                        uint8_t  ep_interval)
  \brief       Create Pipe in System.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   dev_addr   Device Address
  \param[in]   dev_speed  Device Speed
  \param[in]   hub_addr   Hub Address
  \param[in]   hub_port   Hub Port
  \param[in]   ep_addr    Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type    Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \param[in]   ep_interval        Endpoint Polling Interval
  \return      Pipe Handle \ref ARM_USBH_PIPE_HANDLE
*/
static ARM_USBH_PIPE_HANDLE USBH_HW_PipeCreate (uint8_t ctrl, uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t ep_interval) {
  USBH_EHCI_qH        *ptr_qH;
  USBH_EHCI_qH        *ptr_first_qH;
  USBH_EHCI_qH        *ptr_last_qH;
  USBH_EHCI_qTD       *ptr_qTD;
  USBH_EHCI_siTD      *ptr_siTD;
  USBH_EHCI_siTD      *ptr_siTD1;
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             addr;
  uint8_t              restart_flags;

  (void)hub_addr;
  (void)hub_port;

  switch (ep_type) {                            // Check Endpoint Type is valid
    case ARM_USB_ENDPOINT_CONTROL:
    case ARM_USB_ENDPOINT_BULK:
    case ARM_USB_ENDPOINT_INTERRUPT:
    case ARM_USB_ENDPOINT_ISOCHRONOUS:
      break;
    default:                                    // Unknown endpoint type
      return 0U;
  }

  if (ep_type != ARM_USB_ENDPOINT_ISOCHRONOUS){ // If Control, Bulk or Interrupt pipe

    // Get free qH and TI
    ptr_qH  = USBH_EHCI_qH_GetFree  (ctrl); if (ptr_qH  == NULL) { return 0U; }
    ptr_qTD = USBH_EHCI_qTD_GetFree (ctrl); if (ptr_qTD == NULL) { return 0U; }
    ptr_TI  = USBH_EHCI_TI_GetFree  (ctrl); if (ptr_TI  == NULL) { return 0U; }
    memset(ptr_TI, 0, sizeof(USBH_TransferInfo_t));

    restart_flags = 0;
    switch (ep_type) {                            // Endpoint Type
      case ARM_USB_ENDPOINT_CONTROL:
      case ARM_USB_ENDPOINT_BULK:
        (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    false);
        restart_flags |= 1U;
        break;
      case ARM_USB_ENDPOINT_INTERRUPT:
        (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, false);
        restart_flags |= (1U << 1);
        break;
      default:                                    // Unknown endpoint type
        return 0U;
    }

    // Fill in all fields from Endpoint Descriptor to internal element (qH)
    ptr_qH->DW1.MaxPcktLen = ep_max_packet_size;
    switch (dev_speed) {
      case ARM_USB_SPEED_LOW:
        ptr_qH->DW1.EPS = 1U;
        break;
      case ARM_USB_SPEED_FULL:
        ptr_qH->DW1.EPS = 0U;
        break;
      case ARM_USB_SPEED_HIGH:
        ptr_qH->DW1.EPS = 2U;
        break;
      default:                                  // Unknown speed
        return 0U;
    }

    ptr_qH->DW1.EndPt   = ep_addr  & 0x0FU;     // Endpoint Number
    ptr_qH->DW1.DevAddr = dev_addr & 0x7FU;     // Device Address

    ptr_qH->DW2.Mult    = 1U;                   // Pipe Multiplier = 1

    switch (ep_type) {                          // Endpoint Type
      case ARM_USB_ENDPOINT_CONTROL:
        ptr_qH->DW1.RL  = 15U;                  // NAK Count Reload = 15
        ptr_qH->DW1.C   = 1U;                   // Control Endpoint Flag
        break;
      case ARM_USB_ENDPOINT_BULK:
        ptr_qH->DW1.RL  = 15U;                  // NAK Count Reload = 15
        break;
      case ARM_USB_ENDPOINT_INTERRUPT:
        ptr_qH->DW1.RL  = 1U;                   // NAK Count Reload = 1
        ptr_qH->DW2.ISM = 1U;                   // Interrupt Schedule Mask Flag
        break;
      default:                                  // Unknown endpoint type
        return 0U;
    }

    ptr_qH->DW4.T = 1U;                         // Set Next qTD T bit to 1
    ptr_qH->DW5.T = 1U;                         // Set Alternate Next qTD T bit to 1

    // Get previous qH that should be linked to this one or if this is the only
    // qH in the list link it to itself
    switch (ep_type) {                          // Endpoint Type
      case ARM_USB_ENDPOINT_CONTROL:
      case ARM_USB_ENDPOINT_BULK:
        addr         = usbh_ehci_reg_ptr[ctrl]->ASYNCLISTADDR & ~0x1FU;
        ptr_first_qH = (USBH_EHCI_qH *)addr;
        if (!ptr_first_qH) {                    // If this is first qH
          ptr_qH->DW1.H            =  1U;       // Set Head of Reclamation List (H)
          ptr_qH->DW0.HorizLinkPtr = (uint32_t)ptr_qH >> 5;
          ptr_qH->DW0.Typ          =  1U;       // Typ = QH
          ptr_qH->DW0.T            =  0U;       // T   = 0
          usbh_ehci_reg_ptr[ctrl]->ASYNCLISTADDR = (uint32_t)ptr_qH & ~0x1FU;
        } else {                                // If this is not first qH
          ptr_last_qH              =  USBH_EHCI_qH_GetLast (ptr_first_qH);
          ptr_qH->DW0.HorizLinkPtr = (uint32_t)ptr_first_qH >> 5;
          ptr_qH->DW0.Typ          =  1U;       // Typ = QH
          ptr_qH->DW0.T            =  0U;       // T   = 0
          ptr_last_qH->DW0.HorizLinkPtr = (uint32_t)ptr_qH >> 5;
          ptr_last_qH->DW0.Typ     =  1U;       // Typ = QH
          ptr_last_qH->DW0.T       =  0U;       // T   = 0
        }
        break;
      case ARM_USB_ENDPOINT_INTERRUPT:
        ptr_qH->DW0.Typ            =  1U;       // Typ = QH
        ptr_qH->DW0.T              =  1U;
        (void)USBH_EHCI_qH_SetIntEntries (ctrl, ptr_qH, dev_speed, ep_interval);
        break;
      default:                                  // Unknown endpoint type
        return 0U;
    }

    ptr_qTD->DW0.T = 1U;                        // Set Next qTD T bit to 1
    ptr_qTD->DW1.T = 1U;                        // Set Alternate Next qTD T bit to 1
    ptr_qTD->DW2.IOC = 1U;                      // Set IOC to 1, it denotes qTD as used

    ptr_TI->ptr_transfer = (uint32_t *)((uint32_t)ptr_qTD);
    ptr_TI->ptr_pipe     = (uint32_t *)((uint32_t)ptr_qH);

    if (((usbh_ehci_reg_ptr[ctrl]->USBCMD & (1UL << 5)) == 0U) && ((restart_flags & 1U) != 0U)) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    true);
    }
    if (((usbh_ehci_reg_ptr[ctrl]->USBCMD & (1UL << 4)) == 0U) && ((restart_flags & 2U) != 0U)) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);
    }

    return ((ARM_USBH_PIPE_HANDLE)(ptr_qH));
  } else {                                      // If Isochronous pipe

    if (dev_speed == ARM_USB_SPEED_HIGH) {      // High-speed isochronous
      // High-speed isochronous pipe not supported
      return 0U;
    }

    // Endpoint interval must be in limits 1..16
    if ((ep_interval < 1U) || (ep_interval > 16U)) {
      return 0U;
    }

    // Get free TI (Transfer Info)
    ptr_TI = USBH_EHCI_TI_GetFree (ctrl); if (ptr_TI == NULL) { return 0U; }
    memset(ptr_TI, 0, sizeof(USBH_TransferInfo_t));

    // Store interval
    ptr_TI->iso_interval = (uint16_t)(1U << (ep_interval - 1U));
    if (ptr_TI->iso_interval > (pfl_size[ctrl] / 2)) {
      // Iso interval cannot be longer than size of PFL / 2
      ptr_TI->iso_interval = (uint16_t)(pfl_size[ctrl] / 2);
    }
    // Store max packet size
    ptr_TI->iso_max_transfer_size = ep_max_packet_size;

    // Get 2 free siTDs (double buffering)
    ptr_siTD  = (USBH_EHCI_siTD *)USBH_EHCI_PE_GetFree (ctrl, USBH_EHCI_ELEMENT_TYPE_siTD);
    if (ptr_siTD == NULL) { return 0U; }

    ptr_siTD->DW1.DevAddr = dev_addr & 0x7FU; // Device Address

    ptr_siTD1 = (USBH_EHCI_siTD *)USBH_EHCI_PE_GetFree (ctrl, USBH_EHCI_ELEMENT_TYPE_siTD);
    if (ptr_siTD1 == NULL) { return 0U; }

    ptr_siTD->DW0.Typ = 2U;                   // Typ = siTD
    ptr_siTD->DW0.T   = 1U;                   // Link to next is invalid

    ptr_siTD->DW1.IO  = (ep_addr >> 7) & 1U;  // Direction
    ptr_siTD->DW1.EndPt= ep_addr  & 0x0FU;    // Endpoint Number

    ptr_siTD->DW2.SCM = 1U;                   // uFrame C-mask
    ptr_siTD->DW2.SSM = 1U;                   // uFrame S-mask

    // Copy all settings from first siTD to second siTD
    memcpy(ptr_siTD1, ptr_siTD, sizeof(USBH_EHCI_siTD));

    ptr_TI->ptr_pipe      = (uint32_t *)((uint32_t)ptr_siTD);
    ptr_TI->iso_ptr_pipe1 = (uint32_t *)((uint32_t)ptr_siTD1);

    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);

    return ((ARM_USBH_PIPE_HANDLE)(ptr_siTD));
  }
}
static ARM_USBH_PIPE_HANDLE USBH0_HW_PipeCreate (uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t  ep_interval) { return USBH_HW_PipeCreate (0, dev_addr, dev_speed, hub_addr, hub_port, ep_addr, ep_type, ep_max_packet_size, ep_interval); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static ARM_USBH_PIPE_HANDLE USBH1_HW_PipeCreate (uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t  ep_interval) { return USBH_HW_PipeCreate (1, dev_addr, dev_speed, hub_addr, hub_port, ep_addr, ep_type, ep_max_packet_size, ep_interval); }
#endif

/**
  \fn          int32_t USBH_HW_PipeModify (uint8_t              ctrl,
                                           ARM_USBH_PIPE_HANDLE pipe_hndl,
                                           uint8_t              dev_addr,
                                           uint8_t              dev_speed,
                                           uint8_t              hub_addr,
                                           uint8_t              hub_port,
                                           uint16_t             ep_max_packet_size)
  \brief       Modify Pipe in System.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \param[in]   dev_addr   Device Address
  \param[in]   dev_speed  Device Speed
  \param[in]   hub_addr   Hub Address
  \param[in]   hub_port   Hub Port
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBH_HW_PipeModify (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl, uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint16_t ep_max_packet_size) {
  USBH_EHCI_qH *ptr_qH;

  (void)hub_addr;
  (void)hub_port;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  if (((USBH_EHCI_COMMON *)pipe_hndl)->DW0.Typ != USBH_EHCI_ELEMENT_TYPE_qH) {
    // Not supported for isochronous pipe
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  ptr_qH = (USBH_EHCI_qH *)(pipe_hndl);
  if (ptr_qH->DW2.ISM != 0U) {
    // Interrupt Endpoint type
    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, false);
  } else {
    // Control or Bulk Endpoint type
    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    false);
  }

  // Update only device address, device speed and maximum packet size
  ptr_qH->DW1.MaxPcktLen = ep_max_packet_size;
  switch (dev_speed) {
    case ARM_USB_SPEED_LOW:
      ptr_qH->DW1.EPS = 1U;
      break;
    case ARM_USB_SPEED_FULL:
      ptr_qH->DW1.EPS = 0U;
      break;
    case ARM_USB_SPEED_HIGH:
      ptr_qH->DW1.EPS = 2U;
      break;
    default:                                    // Unknown endpoint type
      return ARM_DRIVER_ERROR_PARAMETER;
  }

  ptr_qH->DW1.DevAddr = dev_addr & 0x7FU;       // Device Address

  if (ptr_qH->DW2.ISM != 0U) {
    // Interrupt Endpoint type
    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);
  } else {
    // Control or Bulk Endpoint type
    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED, true);
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeModify (ARM_USBH_PIPE_HANDLE pipe_hndl, uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint16_t ep_max_packet_size) { return USBH_HW_PipeModify (0, pipe_hndl, dev_addr, dev_speed, hub_addr, hub_port, ep_max_packet_size); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PipeModify (ARM_USBH_PIPE_HANDLE pipe_hndl, uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint16_t ep_max_packet_size) { return USBH_HW_PipeModify (1, pipe_hndl, dev_addr, dev_speed, hub_addr, hub_port, ep_max_packet_size); }
#endif

/**
  \fn          int32_t USBH_HW_PipeTransferAbort (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl)
  \brief       Abort current USB Pipe transfer.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \return      \ref execution_status
*/
static int32_t USBH_HW_PipeTransferAbort (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl) {
  USBH_EHCI_qH        *ptr_qH;
  USBH_TransferInfo_t *ptr_TI;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  if (((USBH_EHCI_COMMON *)pipe_hndl)->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_qH) {
    // If Control, Bulk or Interrupt pipe

    ptr_qH = (USBH_EHCI_qH *)pipe_hndl;

    ptr_TI = USBH_EHCI_TI_Find_qH (ctrl, ptr_qH); if (ptr_TI == NULL) { return ARM_DRIVER_OK; }
    if (ptr_TI->active == 0U) { return ARM_DRIVER_OK; }   // If transfer is not active

    if (!USBH_EHCI_qTD_Dequeue (ctrl, ptr_qH, (USBH_EHCI_qTD *)((uint32_t)ptr_TI->ptr_transfer))) {
      return ARM_DRIVER_ERROR;
    }
  } else {

    // For Isochronous pipe
    ptr_TI = USBH_EHCI_TI_Find_PE (ctrl, (void *)pipe_hndl); if (ptr_TI == NULL) { return ARM_DRIVER_ERROR; }
    if (ptr_TI->active == 0U) { return ARM_DRIVER_OK; }   // If transfer is not active

    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, false);

    // Remove isochronous TD (siTDs)
    if (ptr_TI->iso_ptr_pipe1 != NULL) {
      (void)USBH_EHCI_PFL_PE_Remove (ctrl, (void *)ptr_TI->iso_ptr_pipe1);
      ptr_TI->iso_data1 = NULL;
    }
    if (ptr_TI->ptr_pipe != NULL) {
      (void)USBH_EHCI_PFL_PE_Remove (ctrl, (void *)ptr_TI->ptr_pipe);
      ptr_TI->data = NULL;
    }

    (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);
  }

  ptr_TI->active = 0U;

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeTransferAbort (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferAbort (0, pipe_hndl); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PipeTransferAbort (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferAbort (1, pipe_hndl); }
#endif

/**
  \fn          int32_t USBH_HW_PipeDelete (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl)
  \brief       Delete Pipe from System.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \return      \ref execution_status
*/
static int32_t USBH_HW_PipeDelete (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl) {
        USBH_EHCI_qH        *ptr_qH;
        USBH_EHCI_qH        *ptr_prev_qH;
        USBH_TransferInfo_t *ptr_TI;
  const uint32_t            *ptr_uint32_t;
        uint32_t             cnt;
        uint8_t              restart_flags;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  if (((USBH_EHCI_COMMON *)pipe_hndl)->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_qH) {
    // For Control, Bulk and Interrupt pipe

    ptr_qH = (USBH_EHCI_qH *)pipe_hndl;
    ptr_TI = USBH_EHCI_TI_Find_qH (ctrl, ptr_qH); if (ptr_TI == NULL) { return ARM_DRIVER_ERROR; }
    if (ptr_TI->active != 0U) {
      return ARM_DRIVER_ERROR_BUSY;
    }

    if (ptr_qH->DW2.ISM != 0U) {
      // Interrupt Endpoint type
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, false);
    } else {
      // Control or Bulk Endpoint type
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    false);
    }

    // If next queue Head Pointer points to the queue Head that is being
    // processed if so set next queue Head Pointer to the queue Head
    // Horizontal Link Pointer
    restart_flags = 0;
    if (ptr_qH->DW2.ISM != 0U) {
      // Interrupt Endpoint
      (void)USBH_EHCI_qH_ClearIntEntries (ctrl, ptr_qH);

      ptr_uint32_t = (usbh_ehci_ptr[ctrl])->ptr_PFL;
      cnt = USBH_EHCI_PFL_GetSize(ctrl);
      while (cnt != 0U) {
        cnt--;
        if ((*ptr_uint32_t & 1U) == 0U) {
          restart_flags |= (1U << 1);
          break;
        }
        ptr_uint32_t++;
      }
    } else {
      // Control or Bulk Endpoint
      if (((uint32_t)usbh_ehci_reg_ptr[ctrl]->ASYNCLISTADDR & ~0x1FU) == ((uint32_t)ptr_qH & ~0x1FU)) {
        if (((uint32_t)ptr_qH->DW0.HorizLinkPtr << 5) == ((uint32_t)ptr_qH & ~0x1FU)) { // If only 1
          usbh_ehci_reg_ptr[ctrl]->ASYNCLISTADDR = 0;
        } else {
          // Point to next
          usbh_ehci_reg_ptr[ctrl]->ASYNCLISTADDR = (uint32_t)ptr_qH->DW0.HorizLinkPtr << 5;
          ptr_qH->DW1.H  = 1;
          restart_flags |=  1U;
        }
      }
    }

    // If previous qH exists link it to next qH
    ptr_prev_qH = USBH_EHCI_qH_GetPrevious (ctrl, ptr_qH);
    if (ptr_prev_qH != NULL) {
      ptr_prev_qH->DW0 = ptr_qH->DW0;
    }

    (void)USBH_EHCI_qTD_Clear((USBH_EHCI_qTD *)ptr_TI->ptr_transfer);   // Clear qTD
    (void)USBH_EHCI_qH_Clear (ptr_qH);    // Clear queue Head

    if ((restart_flags & 1U) != 0U) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_ASYNC_SCHED,    true);
    }
    if ((restart_flags & 2U) != 0U) {
      (void)USBH_EHCI_StartStop (ctrl, USBH_EHCI_PERIODIC_SCHED, true);
    }
  } else {
    // For Isochronous pipe
    USBH_HW_PipeTransferAbort(ctrl, pipe_hndl);

    ptr_TI = USBH_EHCI_TI_Find_PE (ctrl, (void *)pipe_hndl); if (ptr_TI == NULL) { return ARM_DRIVER_ERROR; }

    (void)USBH_EHCI_PE_Clear((void *)ptr_TI->ptr_pipe);
    (void)USBH_EHCI_PE_Clear((void *)ptr_TI->iso_ptr_pipe1);
  }

  memset(ptr_TI, 0, sizeof(USBH_TransferInfo_t));

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeDelete (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeDelete (0, pipe_hndl); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PipeDelete (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeDelete (1, pipe_hndl); }
#endif

/**
  \fn          int32_t USBH_HW_PipeReset (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl)
  \brief       Reset Pipe.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \return      \ref execution_status
*/
static int32_t USBH_HW_PipeReset (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl) {

  (void)ctrl;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  if (((USBH_EHCI_COMMON *)pipe_hndl)->DW0.Typ != USBH_EHCI_ELEMENT_TYPE_qH) {
    // Not supported for isochronous pipe
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  ((USBH_EHCI_qH *)pipe_hndl)->DW6.DT = 0U;     // Reset data toggle bit to 0

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeReset (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeReset (0, pipe_hndl); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PipeReset (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeReset (1, pipe_hndl); }
#endif

/**
  \fn          int32_t USBH_HW_PipeTransfer (uint8_t              ctrl,
                                             ARM_USBH_PIPE_HANDLE pipe_hndl,
                                             uint32_t             packet,
                                             uint8_t             *data,
                                             uint32_t             num)
  \brief       Transfer packets through USB Pipe.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \param[in]   packet     Packet information
  \param[in]   data       Pointer to buffer with data to send or for data to receive
  \param[in]   num        Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBH_HW_PipeTransfer (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl, uint32_t packet, uint8_t *data, uint32_t num) {
  USBH_EHCI_COMMON    *ptr_curr;
  USBH_EHCI_qTD       *ptr_qTD;
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             num_to_transfer, j;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  ptr_TI   = USBH_EHCI_TI_Find_PE (ctrl, (USBH_EHCI_qH *)pipe_hndl); if (ptr_TI  == NULL) { return ARM_DRIVER_ERROR; }
  ptr_curr = (USBH_EHCI_COMMON *)pipe_hndl;

  if (ptr_curr->DW0.Typ == USBH_EHCI_ELEMENT_TYPE_qH){     // If Control, Bulk or Interrupt pipe

    if (ptr_TI->active != 0U) {
      return ARM_DRIVER_ERROR_BUSY;
    }

    ptr_qTD = (USBH_EHCI_qTD *)ptr_TI->ptr_transfer; if (ptr_qTD == NULL) { return ARM_DRIVER_ERROR; }

    ptr_TI->packet                = packet;
    ptr_TI->data                  = data;
    ptr_TI->num                   = num;
    ptr_TI->num_transferred_total = 0;
    if (num >= 16384U) {
      ptr_TI->num_to_transfer     = 16384U;
    } else {
      ptr_TI->num_to_transfer     = num;
    }

    // LED indicator IND0 control
    if ((((USBH_EHCI_qH *)((uint32_t)ptr_TI->ptr_pipe))->DW2.ISM) == 0U) {
      for (j = 0U; j < 15U; j++) {
        if ((port_act[ctrl] & (1UL << j)) != 0U) {
          usbh_ehci_reg_ptr[ctrl]->PORTSC[j] |=  USBH_EHCI_PORTSC_PIC(1);   // Activate IND0
        }
      }
    }

    if (!USBH_EHCI_qTD_Enqueue (ctrl, ptr_TI, (USBH_EHCI_qH *)pipe_hndl, ptr_qTD, packet, data, ptr_TI->num_to_transfer)) {
      return ARM_DRIVER_ERROR;
    }
  } else {                                      // If Isochronous pipe

    if (num >= ptr_TI->iso_max_transfer_size) {
      num_to_transfer = ptr_TI->iso_max_transfer_size;
    } else {
      num_to_transfer = num;
    }

    if ((ptr_TI->data == NULL) && (ptr_TI->ptr_pipe != NULL)) {                 // If first transfer element is free
      ptr_TI->data = data;
      if (!USBH_EHCI_IsoTransferActivate (ctrl, ptr_TI, (void *)pipe_hndl, data, num_to_transfer)) {
        ptr_TI->data = NULL;
        return ARM_DRIVER_ERROR;
      }
    } else if ((ptr_TI->iso_data1 == NULL) && (ptr_TI->iso_ptr_pipe1 != NULL)){ // If second transfer element is free
      ptr_TI->iso_data1 = data;
      if (!USBH_EHCI_IsoTransferActivate (ctrl, ptr_TI, (void *)ptr_TI->iso_ptr_pipe1, data, num_to_transfer)) {
        ptr_TI->iso_data1 = NULL;
        return ARM_DRIVER_ERROR;
      }
    } else {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeTransfer (ARM_USBH_PIPE_HANDLE pipe_hndl, uint32_t packet, uint8_t *data, uint32_t num) { return USBH_HW_PipeTransfer (0, pipe_hndl, packet, data, num); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static int32_t USBH1_HW_PipeTransfer (ARM_USBH_PIPE_HANDLE pipe_hndl, uint32_t packet, uint8_t *data, uint32_t num) { return USBH_HW_PipeTransfer (1, pipe_hndl, packet, data, num); }
#endif

/**
  \fn          uint32_t USBH_HW_PipeTransferGetResult (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl)
  \brief       Get result of USB Pipe transfer.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \return      number of successfully transferred data bytes
*/
static uint32_t USBH_HW_PipeTransferGetResult (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl) {
  const USBH_TransferInfo_t *ptr_TI;

  if (pipe_hndl == 0U) { return 0U; }

  if (((USBH_EHCI_COMMON *)pipe_hndl)->DW0.Typ != USBH_EHCI_ELEMENT_TYPE_qH) {
    // Not supported for isochronous pipe
    return 0U;
  }

  ptr_TI = USBH_EHCI_TI_Find_qH (ctrl, (USBH_EHCI_qH *)pipe_hndl); if (ptr_TI == NULL) { return 0U; }

  return (ptr_TI->num_transferred_total);
}
static uint32_t USBH0_HW_PipeTransferGetResult (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferGetResult (0, pipe_hndl); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static uint32_t USBH1_HW_PipeTransferGetResult (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferGetResult (1, pipe_hndl); }
#endif

/**
  \fn          uint16_t USBH_HW_GetFrameNumber (uint8_t ctrl)
  \brief       Get current USB Frame Number.
  \param[in]   ctrl     Index of USB Host controller
  \return      Frame Number
*/
static uint16_t USBH_HW_GetFrameNumber (uint8_t ctrl) {

  return ((uint16_t)(usbh_ehci_reg_ptr[ctrl]->FRINDEX >> 3));
}
static uint16_t USBH0_HW_GetFrameNumber (void) { return USBH_HW_GetFrameNumber (0); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static uint16_t USBH1_HW_GetFrameNumber (void) { return USBH_HW_GetFrameNumber (1); }
#endif

/// \brief         Interrupt handling routine
/// \param[in]     ctrl          index of USB Host controller
static void USBH_HW_IRQ_Handler (uint8_t ctrl) {
  ARM_USBH_PIPE_HANDLE  pipe_handle;
  USBH_EHCI_qTD        *ptr_qTD;
  USBH_EHCI_siTD       *ptr_siTD;
  USBH_TransferInfo_t  *ptr_TI;
  USBH_PipeEventInfo_t *ptr_PEI;
  uint32_t              usbsts;
  uint32_t              usbintr;
  uint32_t              frindex;
  uint32_t              done_pfl_index;
  uint32_t              portsc;
  uint32_t              status;
  uint32_t              ep_event;
  uint32_t              transferred;
  uint32_t             *ptr_uint32_t;
  uint32_t              dw0;
  const uint8_t        *buf;
  uint8_t               i, j;

  // Read all registers for interrupt handling
  usbsts  = usbh_ehci_reg_ptr[ctrl]->USBSTS;
  usbh_ehci_reg_ptr[ctrl]->USBSTS = usbsts;
  usbintr = usbh_ehci_reg_ptr[ctrl]->USBINTR;
  frindex = usbh_ehci_reg_ptr[ctrl]->FRINDEX;

  // Analyze interrupt
  if ((usbsts & usbintr) != 0U) {       // Check if not spurious Interrupt
    if ((usbsts & USBH_EHCI_USBSTS_PCI_MSK) != 0U) {
                                        // Port Change Detect
      for (i = 0U; i < 15U; i ++) {     // Go through all ports
        if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << i)) != 0U) {
                                        // If port is enabled
          portsc = usbh_ehci_reg_ptr[ctrl]->PORTSC[i];
          usbh_ehci_reg_ptr[ctrl]->PORTSC[i] = portsc;
          if ((portsc & USBH_EHCI_PORTSC_OCC_MSK) != 0U) {
                                        // Over-current Change
                                        // Clear Connect Status Change
            if ((portsc & USBH_EHCI_PORTSC_OCA_MSK) != 0U) {
//              USBH_HW_PortVbusOnOff (ctrl, i, false);
//              signal_port_event[ctrl](i, ARM_USBH_EVENT_OVERCURRENT);
            }
          }
          if ((portsc & USBH_EHCI_PORTSC_CSC_MSK) != 0U) {
                                        // Connect Status Change
                                        // Clear Connect Status Change
            if ((portsc & USBH_EHCI_PORTSC_CCS_MSK) != 0U) {
                                        // Device Connected
              port_act[ctrl] |=  (1UL << i);
              usbh_ehci_reg_ptr[ctrl]->PORTSC[i] |= USBH_EHCI_PORTSC_PIC(2);    // Activate IND1
              signal_port_event[ctrl](i, ARM_USBH_EVENT_CONNECT);
            } else {
                                        // Device Disconnected
              port_act[ctrl] &= ~(1UL << i);
              usbh_ehci_reg_ptr[ctrl]->PORTSC[i] &= ~USBH_EHCI_PORTSC_PIC_MSK;  // Deactivate LEDs
              signal_port_event[ctrl](i, ARM_USBH_EVENT_DISCONNECT);
            }
          }
          if ((portsc & USBH_EHCI_PORTSC_PE_MSK) != 0U) {
                                        // Port Enable is set
            if ((portsc & USBH_EHCI_PORTSC_FPR_MSK) != 0U) {
                                        // Port Remote Wake-up
              signal_port_event[ctrl](i, ARM_USBH_EVENT_REMOTE_WAKEUP);
            }
          }
        } else {
          if ((usbh_driver_capabilities[ctrl].port_mask & ~((1UL << i) - 1U)) == 0U) {
            break;
          }
        }
      }
    }

    if ((usbsts & USBH_EHCI_USBSTS_UI_MSK) != 0U) {     // If USB Transaction Completed

      // Process Isochronous transfers
      if ((usbh_ehci_ptr[ctrl])->ptr_siTD != NULL) {
        // Last processed Periodic Frame List entry is one before current frame index
        // Calculate previous PFL entry index
        done_pfl_index  = (frindex >> 3) - 2U;
        done_pfl_index &= pfl_size[ctrl] - 1;

        // PFL entry that was processed
        ptr_uint32_t = (uint32_t *)((usbh_ehci_ptr[ctrl])->ptr_PFL) + done_pfl_index;

        if ((*ptr_uint32_t != 1U) && (((*ptr_uint32_t >> 1U) & 3U) == USBH_EHCI_ELEMENT_TYPE_siTD)) {
          // If entry is valid and pointing to siTD
          ptr_siTD = (USBH_EHCI_siTD *)(*ptr_uint32_t & (~0x1FU));
          while (ptr_siTD != NULL) {
            ep_event = 0U;

            // Process all chained siTDs
            ptr_TI = USBH_EHCI_TI_Find_PE (ctrl, (void *)ptr_siTD);
            buf = NULL;
            transferred = 0U;
            pipe_handle = 0U;
            if (ptr_TI && ptr_TI->active) {
              if (ptr_TI->ptr_pipe == (uint32_t *)ptr_siTD) {
                // If first transfer is active
                if ((ptr_TI->data != NULL) && (ptr_TI->iso_frame_index[0] == done_pfl_index)) {
                  // If first transfer in TI has finished
                  buf = ptr_TI->data;
                  ptr_TI->data = NULL;
                }
              } else if (ptr_TI->iso_ptr_pipe1 == (uint32_t *)ptr_siTD) {
                // If second transfer is active
                if ((ptr_TI->iso_data1 != NULL) && (ptr_TI->iso_frame_index[1] == done_pfl_index)) {
                  // If second transfer in TI has finished
                  buf = ptr_TI->iso_data1;
                  ptr_TI->iso_data1 = NULL;
                }
              }

              if (buf != NULL) {
                // If data transfer has finished
                pipe_handle = (ARM_USBH_PIPE_HANDLE)ptr_TI->ptr_pipe;

                // Determine status of transfer
                if ((ptr_siTD->DW3.Status & (3UL<<6)) == 0U) {  // If no error
                  ep_event = ARM_USBH_EVENT_TRANSFER_COMPLETE;
                } else {                                        // If error
                  ep_event = ARM_USBH_EVENT_BUS_ERROR;
                }

                // Calculate number of transferred data bytes
                if (ptr_siTD->DW3.P != 0U) {
                  transferred = (uint32_t)((ptr_siTD->DW5.BufPtr << 12) + ptr_siTD->DW4.CurOfs) - (uint32_t)buf;
                } else {
                  transferred = (uint32_t)((ptr_siTD->DW4.BufPtr << 12) + ptr_siTD->DW4.CurOfs) - (uint32_t)buf;
                }
              }
            }

            // Progress to next siTD if available, and disable current
            if (ptr_siTD->DW0.T == 0U) {
              // If there is next transfer descriptor linked to this one
              dw0 = ((USBH_EHCI_siTD_DW *)ptr_siTD)->DW[0];
              ptr_siTD->DW0.T = 1U;     // Terminate the current siTD
              ptr_siTD = (USBH_EHCI_siTD *)(dw0 & (~0x1FU));
              if (ptr_siTD->DW0.Typ != USBH_EHCI_ELEMENT_TYPE_siTD) {
                // If next linked transfer descriptor is not siTD then it can only be interrupt QH
                // so we put it into PFL
                *ptr_uint32_t = dw0;
                 ptr_siTD = NULL;       // Exit the loop
              }
              // If next linked transfer descriptor is siTD then we continue processing
            } else {
              // No next linked transfer descriptor
              *ptr_uint32_t = 1U;       // Terminate PFL entry (T = 1)
               ptr_siTD = NULL;         // Exit the loop
            }

            if (ep_event != 0U) {
              ptr_PEI = (usbh_ehci_ptr[ctrl])->ptr_PEI;
              ptr_PEI->buf = buf;
              ptr_PEI->len = transferred;
              ep_event |= (uint32_t)(ptr_PEI);
              signal_pipe_event[ctrl](pipe_handle, ep_event); // Signal Pipe Event
            }
          }
        }
      }

      // Process Bulk, Control and Interrupt transfers
      ptr_qTD = (USBH_EHCI_qTD *)((uint32_t)(usbh_ehci_ptr[ctrl])->ptr_qTD); if (ptr_qTD == NULL) { return; }
      for (i = 0U; i < (usbh_ehci_ptr[ctrl])->max_qTD; i++) {
        if ((ptr_qTD->DW2.Status & (1UL << 7)) != 0U) {         // If qTD status is Active
          ptr_qTD++;
          continue;
        }
        ptr_TI   = USBH_EHCI_TI_Find_qTD (ctrl, ptr_qTD);
        ep_event = 0;
        if (ptr_TI && ptr_TI->active) {
          pipe_handle = (ARM_USBH_PIPE_HANDLE)ptr_TI->ptr_pipe;
          status      = ptr_qTD->DW2.Status;
          transferred = ptr_TI->num_to_transfer - ptr_qTD->DW2.TBT;
          ptr_TI->num_transferred_total += transferred;
          if (((status & (1UL << 7)) == 0U) && 
              ((status & (1UL << 6)) == 0U)) {                  // No Error
            if (ptr_TI->num == ptr_TI->num_transferred_total) { // All data was transferred
              ep_event = ARM_USBH_EVENT_TRANSFER_COMPLETE;
            } else {
              if ((ptr_qTD->DW2.PID == 1U) &&                   // Receiving from IN endpoint
                  (transferred != ptr_TI->num_to_transfer)) {   // Short packet or ZLP
                ep_event = ARM_USBH_EVENT_TRANSFER_COMPLETE;
              }
            }
            if (!ep_event) {
              // Restart transfer of remaining data
              if ((ptr_TI->num - ptr_TI->num_transferred_total) >= 16384U) {
                ptr_TI->num_to_transfer = 16384U;
              } else {
                ptr_TI->num_to_transfer = (ptr_TI->num - ptr_TI->num_transferred_total);
              }
              (void)USBH_EHCI_qTD_Enqueue (ctrl, ptr_TI, (USBH_EHCI_qH *)((uint32_t)ptr_TI->ptr_pipe), (USBH_EHCI_qTD *)((uint32_t)ptr_TI->ptr_transfer), ptr_TI->packet, ptr_TI->data + ptr_TI->num_transferred_total, ptr_TI->num_to_transfer);
            } else {
              // Transfer completed

              // LED indicator IND0 control
              if (((USBH_EHCI_qH *)((uint32_t)ptr_TI->ptr_pipe))->DW2.ISM != 0U) {
                for (j = 0U; j < 15U; j++) {
                  if ((port_act[ctrl] & (1UL << j)) != 0U) {
                    usbh_ehci_reg_ptr[ctrl]->PORTSC[j] ^=  USBH_EHCI_PORTSC_PIC(1);   // Change IND0 state
                  }
                }
              } else {
                for (j = 0U; j < 15U; j++) {
                  if ((port_act[ctrl] & (1UL << j)) != 0U) {
                    usbh_ehci_reg_ptr[ctrl]->PORTSC[j] &= ~USBH_EHCI_PORTSC_PIC(1);   // Deactivate IND0
                  }
                }
              }
            }
          } else {
            if ((status & (1UL << 6)) != 0U) {          // Halted
              if        ((status & (1UL << 4)) != 0U) { // Babble Detected
                ep_event = ARM_USBH_EVENT_BUS_ERROR;
              } else if ((status & (1UL << 3)) != 0U) { // Transaction Error
                ep_event = ARM_USBH_EVENT_BUS_ERROR;
              } else if ((status & (1UL << 5)) != 0U) { // Data Buffer Error
                ep_event = ARM_USBH_EVENT_BUS_ERROR;
              } else {
                ep_event = ARM_USBH_EVENT_HANDSHAKE_STALL;
              }
            }
          }
          if (ep_event != 0U) {
            ptr_TI->active = 0U;
            signal_pipe_event[ctrl](pipe_handle, ep_event);
          }
        }
        ptr_qTD++;
      }
    }
  }
}
static void Driver_USBH0_IRQ_Handler (void) { USBH_HW_IRQ_Handler(0U); }
#if   (USBH_EHCI_TT_INSTANCES >= 2)
static void Driver_USBH1_IRQ_Handler (void) { USBH_HW_IRQ_Handler(1U); }
#endif

ARM_DRIVER_USBH USBHn_DRIVER(USBH0_EHCI_DRV_NUM) = {
  USBH0_HW_GetVersion,
  USBH0_HW_GetCapabilities,
  USBH0_HW_Initialize,
  USBH0_HW_Uninitialize,
  USBH0_HW_PowerControl,
  USBH0_HW_PortVbusOnOff,
  USBH0_HW_PortReset,
  USBH0_HW_PortSuspend,
  USBH0_HW_PortResume,
  USBH0_HW_PortGetState,
  USBH0_HW_PipeCreate,
  USBH0_HW_PipeModify,
  USBH0_HW_PipeDelete,
  USBH0_HW_PipeReset,
  USBH0_HW_PipeTransfer,
  USBH0_HW_PipeTransferGetResult,
  USBH0_HW_PipeTransferAbort,
  USBH0_HW_GetFrameNumber
};

#if (USBH_EHCI_TT_INSTANCES >= 2)
ARM_DRIVER_USBH USBHn_DRIVER(USBH1_EHCI_DRV_NUM) = {
  USBH1_HW_GetVersion,
  USBH1_HW_GetCapabilities,
  USBH1_HW_Initialize,
  USBH1_HW_Uninitialize,
  USBH1_HW_PowerControl,
  USBH1_HW_PortVbusOnOff,
  USBH1_HW_PortReset,
  USBH1_HW_PortSuspend,
  USBH1_HW_PortResume,
  USBH1_HW_PortGetState,
  USBH1_HW_PipeCreate,
  USBH1_HW_PipeModify,
  USBH1_HW_PipeDelete,
  USBH1_HW_PipeReset,
  USBH1_HW_PipeTransfer,
  USBH1_HW_PipeTransferGetResult,
  USBH1_HW_PipeTransferAbort,
  USBH1_HW_GetFrameNumber
};
#endif
