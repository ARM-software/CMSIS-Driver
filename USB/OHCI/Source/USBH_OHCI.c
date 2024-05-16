/* -----------------------------------------------------------------------------
 * Copyright (c) 2024 Arm Limited (or its affiliates).
 * All rights reserved.
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
 * $Date:        16. May 2024
 * $Revision:    V1.0
 *
 * Project:      USB Host OHCI Controller Driver
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0
 *    Initial release
 */

#include "USBH_OHCI.h"

#include <stdio.h>
#include <string.h>

#include "USBH_OHCI_Config.h"
#include "USBH_OHCI_Regs.h"
#include "USBH_OHCI_HW.h"

#include "cmsis_os2.h"
#include "cmsis_compiler.h"

// Driver Version *************************************************************
static const ARM_DRIVER_VERSION usbh_driver_version = { ARM_USBH_API_VERSION, ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) };
// ****************************************************************************

// Compile-time configuration *************************************************

// Configuration depending on the USBH_OHCI_Config.h

#if    (USBH1_OHCI_ENABLED == 1)
#define USBH_OHCI_INSTANCES            (2U)
#else
#define USBH_OHCI_INSTANCES            (1U)
#endif

// ****************************************************************************

// Driver Capabilities ********************************************************
static ARM_USBH_CAPABILITIES usbh_driver_capabilities[USBH_OHCI_INSTANCES] = {
  {
    0x3FFFU,    /* Root HUB available Ports Mask   */
    0U,         /* Automatic SPLIT packet handling */
    1U,         /* Signal Connect event            */
    1U,         /* Signal Disconnect event         */
    1U,         /* Signal Overcurrent event        */
    0U          /* reserved bits                   */
  }
#if (USBH_OHCI_INSTANCES >= 2)
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

#define USBHn_OHCI_HCCA_SECTION_(x)     __attribute__((section(x)))

#if    (USBH0_OHCI_HCCA_RELOC == 1)
#define USBH0_OHCI_HCCA_SECTION(x)      USBHn_OHCI_HCCA_SECTION_(x)
#else 
#define USBH0_OHCI_HCCA_SECTION(x)
#endif

#if    (USBH1_OHCI_ENABLED == 1)
#if    (USBH1_OHCI_HCCA_RELOC == 1)
#define USBH1_OHCI_HCCA_SECTION(x)      USBHn_OHCI_HCCA_SECTION_(x)
#else 
#define USBH1_OHCI_HCCA_SECTION(x)
#endif
#endif

#ifndef USBH_OHCI_MEM_HCCA_SIZE
#define USBH_OHCI_MEM_HCCA_SIZE         (256U)
#endif
#ifndef USBH_OHCI_MEM_ED_SIZE
#define USBH_OHCI_MEM_ED_SIZE           (USBH_OHCI_MAX_PIPES * 16U)
#endif
#ifndef USBH_OHCI_MEM_TD_SIZE
#define USBH_OHCI_MEM_TD_SIZE           (USBH_OHCI_MAX_PIPES * 16U)
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
} USBH_TransferInfo_t;

// Structure containing configuration values for OHCI Compliant Controller
typedef struct {
  uint32_t              ctrl;                               // controller index (used for hardware specific driver)
  uint32_t             *ptr_OHCI;                           // pointer to memory mapped reg base address
  uint16_t              max_ED;                             // maximum Endpoint Descriptors
  uint16_t              max_TD;                             // maximum Transfer Descriptors
  uint16_t              max_ITD;                            // maximum Isochronous Transfer Descriptors
  uint16_t              pad0;                               // explicit padding
  uint32_t             *ptr_HCCA;                           // pointer to HCCA memory start
  uint32_t             *ptr_ED;                             // pointer to ED memory start
  uint32_t             *ptr_TD;                             // pointer to TD memory start
  uint32_t             *ptr_ITD;                            // pointer to ITD memory start
  USBH_TransferInfo_t  *ptr_TI;                             // pointer to Transfer Info (TI) array start
  USBH_OHCI_Interrupt_t irq_handler;                        // pointer to OHCI Interrupt Handler Routine
} USBH_OHCI_t;

// HCCA Communication structure
typedef struct {
  uint32_t              hcca [USBH_OHCI_MEM_HCCA_SIZE / 4];
  uint32_t              ed   [USBH_OHCI_MEM_ED_SIZE   / 4];
  uint32_t              td   [USBH_OHCI_MEM_TD_SIZE   / 4];
} USBH_OHCI_HCCA_t;

// Local functions prototypes
static void Driver_USBH0_IRQ_Handler (void);
#if   (USBH1_OHCI_ENABLED == 1)
static void Driver_USBH1_IRQ_Handler (void);
#endif

// USB Host 0 information
static USBH_TransferInfo_t  usbh0_transfer_info    [USBH_OHCI_MAX_PIPES];
static USBH_OHCI_HCCA_t     usbh0_ohci_hcca         USBH0_OHCI_HCCA_SECTION(USBH0_OHCI_HCCA_SECTION_NAME) __ALIGNED(USBH_OHCI_MEM_HCCA_SIZE);
static const USBH_OHCI_t    usbh0_ohci = {          USBH0_OHCI_DRV_NUM,
                                                   (uint32_t *)USBH0_OHCI_BASE_ADDR,
                                                    USBH_OHCI_MAX_PIPES,
                                                    USBH_OHCI_MAX_PIPES,
                                                    0U,
                                                    0U, // padding byte
                                                   &usbh0_ohci_hcca.hcca[0],
                                                   &usbh0_ohci_hcca.ed[0],
                                                   &usbh0_ohci_hcca.td[0],
                                                    NULL,
                                                   &usbh0_transfer_info[0],
                                                    Driver_USBH0_IRQ_Handler
                                                 };

// USB Host 1 information
#if   (USBH1_OHCI_ENABLED == 1)
static USBH_TransferInfo_t  usbh1_transfer_info    [USBH_OHCI_MAX_PIPES];
static USBH_OHCI_HCCA_t     usbh1_ohci_hcca         USBH1_OHCI_HCCA_SECTION(USBH1_OHCI_HCCA_SECTION_NAME) __ALIGNED(USBH_OHCI_MEM_HCCA_SIZE);
static const USBH_OHCI_t    usbh1_ohci = {          USBH1_OHCI_DRV_NUM,
                                                   (uint32_t *)USBH1_OHCI_BASE_ADDR,
                                                    USBH_OHCI_MAX_PIPES,
                                                    USBH_OHCI_MAX_PIPES,
                                                    0U,
                                                    0U, // padding byte
                                                   &usbh1_ohci_hcca.hcca[0],
                                                   &usbh1_ohci_hcca.ed[0],
                                                   &usbh1_ohci_hcca.td[0],
                                                    NULL,
                                                   &usbh1_transfer_info[0],
                                                    Driver_USBH1_IRQ_Handler
                                                 };
#endif // (USBH1_OHCI_ENABLED == 1)

// USB Hosts information
static const USBH_OHCI_t * const usbh_ohci_ptr[USBH_OHCI_INSTANCES] = {
       &usbh0_ohci
#if   (USBH_OHCI_INSTANCES >= 2)
     , &usbh1_ohci
#endif
};

static USBH_OHCI_Registers_t      *usbh_ohci_reg_ptr[USBH_OHCI_INSTANCES];
static ARM_USBH_SignalPortEvent_t  signal_port_event[USBH_OHCI_INSTANCES];
static ARM_USBH_SignalPipeEvent_t  signal_pipe_event[USBH_OHCI_INSTANCES];

/* USBH Transfer and Endpoint Helper Functions ------------*/

/**
  \fn          void USBH_OHCI_HCCA_Clear (uint8_t ctrl)
  \brief       Clear Host Controller Communication Area
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_OHCI_HCCA_Clear (uint8_t ctrl) {

  memset ((void *)((usbh_ohci_ptr[ctrl])->ptr_HCCA), 0, 256);
}

/**
  \fn          USBH_OHCI_TD *USBH_OHCI_TD_GetFree (uint8_t ctrl)
  \brief       Get free general Transfer Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free general Transfer Descriptor, no free or function failed
               value > 0:  pointer to first free general Transfer Descriptor
               value = 0:  no free general Transfer Descriptor exists or function failed
*/
static USBH_OHCI_TD *USBH_OHCI_TD_GetFree (uint8_t ctrl) {
  USBH_OHCI_TD *ptr_TD;
  uint32_t      cnt;

  ptr_TD = (USBH_OHCI_TD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_TD);

  if (ptr_TD == NULL) { return NULL; }

  cnt = (usbh_ohci_ptr[ctrl])->max_TD;
  while (cnt != 0U) {
    cnt--;
    if (ptr_TD->DW0.SCR == 0U) { return ptr_TD; }
    ptr_TD++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_TD *USBH_OHCI_TD_GetPrevious (uint8_t ctrl, USBH_OHCI_TD *ptr_TD)
  \brief       Get previous linked general Transfer Descriptor linked to this one (before it)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_TD      Pointer to general Transfer Descriptor
  \return                  Pointer to previous linked general Transfer Descriptor, no previous or function failed
               value > 0:  pointer to previous linked general Transfer Descriptor
               value = 0:  no previous linked general Transfer Descriptor exists or function failed
*/
static USBH_OHCI_TD *USBH_OHCI_TD_GetPrevious (uint8_t ctrl, const USBH_OHCI_TD *ptr_TD) {
  USBH_OHCI_TD  *ptr_prev_TD;
  uint32_t       i;

  if (ptr_TD == NULL) { return NULL; }

  ptr_prev_TD = (USBH_OHCI_TD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_TD);

  if (ptr_prev_TD == NULL) { return NULL; }

  for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_TD; i++) {
    if (ptr_prev_TD->NextTD == (uint32_t)ptr_TD) {
      return ptr_prev_TD;
    }
    ptr_prev_TD++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_TD *USBH_OHCI_TD_GetLast (USBH_OHCI_ED *ptr_ED)
  \brief       Get last general Transfer Descriptor for Endpoint Descriptor
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \return                  Pointer to last general Transfer Descriptor, no last or function failed
               value > 0:  pointer to last general Transfer Descriptor
               value = 0:  no last general Transfer Descriptor exists or function failed
*/
static USBH_OHCI_TD *USBH_OHCI_TD_GetLast (const USBH_OHCI_ED *ptr_ED) {
  USBH_OHCI_TD *ptr_TD;
  uint32_t      addr;

  if (ptr_ED == NULL) { return NULL; }

  addr   = (uint32_t)ptr_ED->DW2.HeadP << 4;
  ptr_TD = (USBH_OHCI_TD *)addr;

  if (ptr_TD == NULL) { return NULL; }

  while (ptr_TD->NextTD != 0U) {
    ptr_TD = (USBH_OHCI_TD *)ptr_TD->NextTD;
  }

  return ptr_TD;
}

/**
  \fn          USBH_OHCI_TD *USBH_OHCI_TD_Clear (USBH_OHCI_TD *ptr_TD)
  \brief       Clear general Transfer Descriptor and return pointer to next linked general Transfer Descriptor
  \param[in]   ptr_TD      Pointer to general Transfer Descriptor to be cleared
  \return                  Pointer to next linked general Transfer Descriptor, no next or function failed
               value > 0:  pointer to next linked general Transfer Descriptor
               value = 0:  no next linked general Transfer Descriptor exists or function failed
*/
static USBH_OHCI_TD *USBH_OHCI_TD_Clear (USBH_OHCI_TD *ptr_TD) {
  USBH_OHCI_TD *ptr_next_TD;

  if (ptr_TD == NULL) { return NULL; }

  ptr_next_TD = (USBH_OHCI_TD *)ptr_TD->NextTD;
  memset (ptr_TD, 0, sizeof(USBH_OHCI_TD));

  return ptr_next_TD;
}

/**
  \fn          void USBH_OHCI_TD_ClearAll (uint8_t ctrl)
  \brief       Clear all general Transfer Descriptors
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_OHCI_TD_ClearAll (uint8_t ctrl) {
  USBH_OHCI_TD *ptr_TD;
  uint32_t      i;

  ptr_TD = (USBH_OHCI_TD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_TD);
  if (ptr_TD != NULL) {
    for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_TD; i++) {
      (void)USBH_OHCI_TD_Clear (ptr_TD);
      ptr_TD++;
    }
  }
}

/**
  \fn          bool USBH_OHCI_TD_Enqueue (uint8_t       ctrl,
                                          USBH_OHCI_ED *ptr_ED,
                                          USBH_OHCI_TD *ptr_TD,
                                          uint32_t      packet,
                                          uint8_t      *data,
                                          uint32_t      num)
  \brief       Enqueue Control/Bulk/Interrupt Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \param[in]   ptr_TD      Pointer to general Transfer Descriptor to enqueue
  \param[in]   packet      Packet information
  \param[in]   data        Pointer to buffer with data to send or for data to receive
  \param[in]   num         Number of data bytes to transfer
  \return      true = success, false = fail
*/
static bool USBH_OHCI_TD_Enqueue (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, USBH_OHCI_TD *ptr_TD, uint32_t packet, uint8_t *data, uint32_t num) {
  USBH_OHCI_TD *ptr_last_TD;

  (void)ctrl;

  if (ptr_ED == NULL) { return false; }
  if (ptr_TD == NULL) { return false; }

  // Fill Transfer Descriptor fields
  ptr_TD->DW0.SCR = 1U;

  ptr_TD->DW0.R   = 1U;
  switch (packet & ARM_USBH_PACKET_TOKEN_Msk) {
    case ARM_USBH_PACKET_IN:
      ptr_TD->DW0.DP = 2U;
      break;
    case ARM_USBH_PACKET_OUT:
      ptr_TD->DW0.DP = 1U;
      break;
    case ARM_USBH_PACKET_SETUP:
      ptr_TD->DW0.DP = 0U;
      ptr_ED->DW2.C  = 0U;              // Force Data Toggle for SETUP packet to DATA0
      break;
    default:
      return false;
  }
  switch (packet & ARM_USBH_PACKET_DATA_Msk) {
    case ARM_USBH_PACKET_DATA0:
      ptr_ED->DW2.C = 0U;
      break;
    case ARM_USBH_PACKET_DATA1:
      ptr_ED->DW2.C = 1U;
      break;
    default:                            // Keep previous data toggle
      break;
  }

  ptr_TD->DW0.DI = 0U;
  ptr_TD->DW0.T  = 0U;
  ptr_TD->DW0.EC = 0U;
  ptr_TD->DW0.CC = USBH_OHCI_CC_NOTACCESSED;
  if (num == 0U) {                      // For 0 size packet
    ptr_TD->CBP    = 0U;
    ptr_TD->NextTD = 0U;
    ptr_TD->BE     = 0U;
  } else {
    ptr_TD->CBP    = (uint32_t)(data);
    ptr_TD->NextTD = 0U;
    ptr_TD->BE     = (uint32_t)(data) + num - 1U;
  }

  if ((ptr_ED->DW2.HeadP ^ (ptr_ED->TailP >> 4)) == 0U) {
    ptr_ED->DW2.HeadP = (uint32_t)ptr_TD >> 4;
    ptr_ED->TailP = 0U;
  } else {
    ptr_last_TD = USBH_OHCI_TD_GetLast (ptr_ED);
    if (ptr_last_TD != NULL) {
      ptr_last_TD->NextTD  = (uint32_t)ptr_TD;
    }
    ptr_ED->TailP = 0U;
  }
  ptr_ED->DW2.H = 0U;                   // Clear halted status of endpoint
  ptr_ED->DW0.K = 0U;                   // Clear skip bit of endpoint descriptor (start processing it)

  return true;
}

/**
  \fn          bool USBH_OHCI_TD_Dequeue (uint8_t       ctrl,
                                          USBH_OHCI_ED *ptr_ED,
                                          USBH_OHCI_TD *ptr_TD)
  \brief       Dequeue Control/Bulk/Interrupt Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \param[in]   ptr_TD      Pointer to general Transfer Descriptor to dequeue
  \return      true = success, false = fail
*/
static bool USBH_OHCI_TD_Dequeue (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, USBH_OHCI_TD *ptr_TD) {
  const USBH_OHCI_TD *ptr_prev_TD;

  if (ptr_ED == NULL) { return false; }
  if (ptr_TD == NULL) { return false; }

  ptr_ED->DW0.K   = 1U;                 // Set skip bit of endpoint descriptor (stop processing it)
  (void)osDelay(1U);                    // Wait ~1 ms to be sure that endpoint descriptor is not being processed

  ptr_prev_TD = USBH_OHCI_TD_GetPrevious (ctrl, ptr_TD);
  if (ptr_prev_TD == NULL) {
    if (((uint32_t)ptr_ED->DW2.HeadP << 4) == (uint32_t)ptr_TD) {
      ptr_ED->DW2.HeadP = ptr_TD->NextTD >> 4;
    }
  }
  (void)USBH_OHCI_TD_Clear (ptr_TD);

  return true;
}

#if 0
/**
  \fn          USBH_OHCI_ITD *USBH_OHCI_ITD_GetFree (uint8_t ctrl)
  \brief       Get free Isochronous Transfer Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free Isochronous Transfer Descriptor, no free or function failed
               value > 0:  pointer to first free Isochronous Transfer Descriptor
               value = 0:  no free Isochronous Transfer Descriptor exists or function failed
*/
static USBH_OHCI_ITD *USBH_OHCI_ITD_GetFree (uint8_t ctrl) {
  USBH_OHCI_ITD *ptr_ITD;
  uint32_t       i;

  ptr_ITD = (USBH_OHCI_ITD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ITD);

  if (ptr_ITD == NULL) { return NULL; }

  for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_ITD; i++) {
    if ((ptr_ITD->DW0.CC == 0U) && (ptr_ITD->DW1.BP0 == 0U) && (ptr_ITD->NextTD == 0U) && (ptr_ITD->BE == 0U)) {
      return ptr_ITD;
    }
    ptr_ITD++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_ITD *USBH_OHCI_ITD_GetPrevious (uint8_t ctrl, USBH_OHCI_ITD *ptr_ITD)
  \brief       Get previous linked Isochronous Transfer Descriptor linked to this one (before it)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ITD     Pointer to Isochronous Transfer Descriptor
  \return                  Pointer to previous linked Isochronous Transfer Descriptor, no previous or function failed
               value > 0:  pointer to previous linked Isochronous Transfer Descriptor
               value = 0:  no previous linked Isochronous Transfer Descriptor exists or function failed
*/
static USBH_OHCI_ITD *USBH_OHCI_ITD_GetPrevious (uint8_t ctrl, USBH_OHCI_ITD *ptr_ITD) {
  USBH_OHCI_ITD *ptr_prev_ITD;
  uint32_t       i;

  if (ptr_ITD == NULL) { return NULL; }

  ptr_prev_ITD = (USBH_OHCI_ITD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ITD);

  if (ptr_prev_ITD == NULL) { return NULL; }

  for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_ITD; i++) {
    if (ptr_prev_ITD->NextTD == (uint32_t)ptr_ITD) {
      return ptr_prev_ITD;
    }
    ptr_prev_ITD++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_ITD *USBH_OHCI_ITD_GetLast (USBH_OHCI_ED *ptr_ED)
  \brief       Get last Isochronous Transfer Descriptor for Endpoint Descriptor
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \return                  Pointer to last Isochronous Transfer Descriptor, no last or function failed
               value > 0:  pointer to last Isochronous Transfer Descriptor
               value = 0:  no last Isochronous Transfer Descriptor exists or function failed
*/
static USBH_OHCI_ITD *USBH_OHCI_ITD_GetLast (const USBH_OHCI_ED *ptr_ED) {
  USBH_OHCI_ITD *ptr_ITD;
  uint32_t       addr;

  if (ptr_ED == NULL) { return NULL; }

  addr = ptr_ED->DW2.HeadP << 4;
  ptr_ITD = (USBH_OHCI_ITD *)addr;

  if (ptr_ITD == NULL) { return NULL; }

  while (ptr_ITD->NextTD != 0U) {
    ptr_ITD = (USBH_OHCI_ITD *)ptr_ITD->NextTD;
  }

  return ptr_ITD;
}

/**
  \fn          USBH_OHCI_ITD *USBH_OHCI_ITD_Clear (USBH_OHCI_ITD *ptr_ITD)
  \brief       Clear Isochronous Transfer Descriptor and return pointer to next linked general Transfer Descriptor
  \param[in]   ptr_ITD     Pointer to Isochronous Transfer Descriptor to be cleared
  \return                  Pointer to next linked Isochronous Transfer Descriptor, no next or function failed
               value > 0:  pointer to next linked Isochronous Transfer Descriptor
               value = 0:  no next linked Isochronous Transfer Descriptor exists or function failed
*/
static USBH_OHCI_ITD *USBH_OHCI_ITD_Clear (USBH_OHCI_ITD *ptr_ITD) {
  USBH_OHCI_ITD *ptr_next_ITD;

  if (ptr_ITD == NULL) { return NULL; }

  ptr_next_ITD = (USBH_OHCI_ITD *)ptr_ITD->NextTD;
  memset (ptr_ITD, 0, sizeof(USBH_OHCI_ITD));

  return ptr_next_ITD;
}

/**
  \fn          void USBH_OHCI_ITD_ClearAll (uint8_t ctrl)
  \brief       Clear all Isochronous Transfer Descriptors
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_OHCI_ITD_ClearAll (uint8_t ctrl) {
  USBH_OHCI_ITD *ptr_ITD;
  uint32_t       i;

  ptr_ITD = (USBH_OHCI_ITD *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ITD);
  if (ptr_ITD != NULL) {
    for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_ITD; i++) {
      (void)USBH_OHCI_ITD_Clear (ptr_ITD);
      ptr_ITD++;
    }
  }
}

/**
  \fn          bool USBH_OHCI_ITD_Enqueue (uint8_t        ctrl,
                                           USBH_OHCI_ED  *ptr_ED,
                                           USBH_OHCI_ITD *ptr_ITD,
                                           uint32_t       packet,
                                           uint8_t       *data,
                                           uint32_t       num)
  \brief       Enqueue Isochronous Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \param[in]   ptr_ITD     Pointer to Isochronous Transfer Descriptor to enqueue
  \param[in]   packet      Packet information
  \param[in]   data        Pointer to buffer with data to send or for data to receive
  \param[in]   num         Number of data bytes to transfer
  \return      true = success, false = fail
*/
static bool USBH_OHCI_ITD_Enqueue (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, USBH_OHCI_ITD *ptr_ITD, uint32_t packet, uint8_t *data, uint32_t num) {
#ifdef __DEBUG
  #warning "function USBH_OHCI_ITD_Enqueue: Not implemented yet!"
#endif
  (void)ctrl;
  (void)ptr_ED;
  (void)ptr_ITD;
  (void)packet;
  (void)data;
  (void)num;

  return false;
}

/**
  \fn          bool USBH_OHCI_ITD_Dequeue (uint8_t       ctrl,
                                           USBH_OHCI_ED *ptr_ED,
                                           USBH_OHCI_TD *ptr_ITD)
  \brief       Dequeue Isochronous Transfer
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \param[in]   ptr_ITD     Pointer to Isochronous Transfer Descriptor to dequeue
  \return      true = success, false = fail
*/
static bool USBH_OHCI_ITD_Dequeue (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, USBH_OHCI_ITD *ptr_ITD) {
#ifdef __DEBUG
  #warning "function USBH_OHCI_ITD_Dequeue: Not implemented yet!"
#endif
  (void)ctrl;
  (void)ptr_ED;
  (void)ptr_ITD;

  return false;
}
#endif

/**
  \fn          USBH_OHCI_ED *USBH_OHCI_ED_Clear (USBH_OHCI_ED *ptr_ED)
  \brief       Clear Endpoint Descriptor and return pointer to next linked Endpoint Descriptor
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \return                  Pointer to next linked Endpoint Descriptor, no next or function failed
               value > 0:  pointer to next linked Endpoint Descriptor
               value = 0:  no next linked Endpoint Descriptor exists or function failed
*/
static USBH_OHCI_ED *USBH_OHCI_ED_Clear (USBH_OHCI_ED *ptr_ED) {
  USBH_OHCI_ED *ptr_next_ED;

  if (ptr_ED == NULL) { return NULL; }

  ptr_next_ED = (USBH_OHCI_ED *)ptr_ED->NextED;
  if (ptr_ED == ptr_next_ED) {          // If ED is pointing as next to itself
    ptr_next_ED = NULL;
  }

  memset(ptr_ED, 0, sizeof(USBH_OHCI_ED));

  return ptr_next_ED;
}

/**
  \fn          void USBH_OHCI_ED_ClearAll (uint8_t ctrl)
  \brief       Clear all Endpoint Descriptors
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_OHCI_ED_ClearAll (uint8_t ctrl) {
  USBH_OHCI_ED *ptr_ED;
  uint32_t      i;

  ptr_ED = (USBH_OHCI_ED *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ED);
  if (ptr_ED != NULL) {
    for (i = 0U; i < (usbh_ohci_ptr[ctrl])->max_ED; i++) {
      (void)USBH_OHCI_ED_Clear (ptr_ED);
      ptr_ED++;
    }
  }
}

/**
  \fn          USBH_OHCI_ED *USBH_OHCI_ED_GetFree (uint8_t ctrl)
  \brief       Get free general Endpoint Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free Endpoint Descriptor, no free or function failed
               value > 0:  pointer to first free Endpoint Descriptor
               value = 0:  no free Endpoint Descriptor exists or function failed
*/
static USBH_OHCI_ED *USBH_OHCI_ED_GetFree (uint8_t ctrl) {
  USBH_OHCI_ED *ptr_ED;
  uint32_t      cnt;

  ptr_ED = (USBH_OHCI_ED *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ED); if (ptr_ED == NULL) { return NULL; }

  cnt = (usbh_ohci_ptr[ctrl])->max_ED;
  while (cnt != 0U) {
    cnt--;
    if (ptr_ED->DW0.SCR == 0U) { return ptr_ED; }
    ptr_ED++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_ED *USBH_OHCI_ED_GetPrevious (uint8_t ctrl, USBH_OHCI_ED *ptr_ED)
  \brief       Get previous Endpoint Descriptor linked to this one (before it)
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_qH      Pointer to Endpoint Descriptor to start search from
  \return                  Pointer to previous Endpoint Descriptor, no previous or function failed
               value > 0:  pointer to previous Endpoint Descriptor
               value = 0:  no previous Endpoint Descriptor exists or function failed
*/
static USBH_OHCI_ED *USBH_OHCI_ED_GetPrevious (uint8_t ctrl, USBH_OHCI_ED *ptr_ED) {
  USBH_OHCI_ED *ptr_prev_ED;
  uint32_t      cnt;

  if (ptr_ED == NULL) { return NULL; }

  ptr_prev_ED = (USBH_OHCI_ED *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_ED); if (ptr_prev_ED == NULL) { return NULL; }

  cnt = (usbh_ohci_ptr[ctrl])->max_ED;
  while (cnt != 0U) {
    cnt--;
    if (ptr_prev_ED->NextED == (uint32_t)ptr_ED) { return ptr_prev_ED; }
    ptr_prev_ED++;
  }

  return NULL;
}

/**
  \fn          USBH_OHCI_ED *USBH_OHCI_ED_GetLast (USBH_OHCI_ED *ptr_ED)
  \brief       Get last Endpoint Descriptor starting from requested one
  \param[in]   ptr_qH      Pointer to Endpoint Descriptor to start search from
  \return                  Pointer to last chained Endpoint Descriptor starting from requested one, no last or function failed
               value > 0:  pointer to last chained Endpoint Descriptor starting from requested one
               value = 0:  no last last chained Endpoint Descriptor starting from requested one or function failed
*/
static USBH_OHCI_ED *USBH_OHCI_ED_GetLast (USBH_OHCI_ED *ptr_ED) {
  USBH_OHCI_ED *ptr_ED_;

  if (ptr_ED == NULL) { return NULL; }
  ptr_ED_ = ptr_ED;

  while (ptr_ED_->NextED != 0U) {
    ptr_ED_ = (USBH_OHCI_ED *)ptr_ED_->NextED;
  }

  return ptr_ED_;
}

/**
  \fn          USBH_OHCI_ED *USBH_OHCI_ED_Unlink (USBH_OHCI_ED *ptr_first_ED, USBH_OHCI_ED *ptr_ED)
  \brief       Unlink Endpoint Descriptor form Endpoint Descriptor linked list and
               return pointer to beginning of the linked list Endpoint Descriptor
  \param[in]   ptr_first_ED  Pointer to first Endpoint Descriptor in linked list
  \param[in]   ptr_ED        Pointer to Endpoint Descriptor
  \return                    Pointer to beginning ot the linked list, Endpoint Descriptor not found or function failed
               value > 0:    pointer to beginning ot the linked list
               value = 0:    Endpoint Descriptor not found or function failed
*/
static USBH_OHCI_ED *USBH_OHCI_ED_Unlink (USBH_OHCI_ED *ptr_first_ED, USBH_OHCI_ED *ptr_ED) {
        USBH_OHCI_ED *ptr_first_ED_, *ptr_curr_ED, *ptr_next_ED;
  const USBH_OHCI_ED *ptr_ED_;
        uint32_t      addr;

  addr          = (uint32_t)(ptr_first_ED) & ~0x0FU;
  ptr_first_ED_ = (USBH_OHCI_ED *)addr;
  addr          = (uint32_t)(ptr_ED)       & ~0x0FU;
  ptr_ED_       = (USBH_OHCI_ED *)addr;

  if ((ptr_first_ED_ == NULL) || (ptr_ED_ == NULL)) { return ptr_first_ED_; }

  ptr_curr_ED   =  ptr_first_ED_;
  addr          = (uint32_t)(ptr_curr_ED->NextED) & ~0x0FU;
  ptr_next_ED   = (USBH_OHCI_ED *)addr;

  // Handle special case if first element is one that needs to be unlinked
  if (ptr_curr_ED == ptr_ED_) {         // If first element is requested one
    if ((ptr_next_ED == ptr_curr_ED) || // If element is linked to itself or
        (ptr_next_ED == NULL)) {        // if this is last element in list
      return NULL;                      // Start of the list is now 0
    } else {                            // There is a valid next element
      ptr_first_ED_ = ptr_next_ED;
      ptr_curr_ED   = ptr_next_ED;
      addr          = (uint32_t)(ptr_curr_ED->NextED) & ~0x0FU;
      ptr_next_ED   = (USBH_OHCI_ED *)addr;
    }
  }

  while (ptr_next_ED != NULL) {         // While next element is valid
    if (ptr_next_ED == ptr_ED_) {       // If next element is requested one
      addr        = (uint32_t)(ptr_next_ED->NextED) & ~0x0FU;
      ptr_next_ED = (USBH_OHCI_ED *)addr;
      ptr_curr_ED->NextED = (uint32_t)ptr_next_ED;
      if (ptr_next_ED == NULL) { break; }
    }
    ptr_curr_ED = ptr_next_ED;
    addr        = (uint32_t)(ptr_next_ED->NextED) & ~0x0FU;
    ptr_next_ED = (USBH_OHCI_ED *)addr;
  }

  return ptr_first_ED_;
}

/**
  \fn          uint32_t USBH_OHCI_ED_CountIntEntries (uint8_t ctrl, USBH_OHCI_ED *ptr_ED)
  \brief       Count number of times Endpoint Descriptor for interrupt endpoint is found in periodic table
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \return                  Number of times Endpoint Descriptor is found in periodic table
*/
static uint32_t USBH_OHCI_ED_CountIntEntries (uint8_t ctrl, const USBH_OHCI_ED *ptr_ED) {
  const uint32_t     *ptr_uint32_t;
  const USBH_OHCI_ED *ptr_tmp_ED;
        uint32_t      cnt;
        uint32_t      i;
        uint32_t      addr;

  if (ptr_ED == NULL) { return 0U; }

  cnt = 0U;

  ptr_uint32_t = (usbh_ohci_ptr[ctrl])->ptr_HCCA;
  for (i = 0U; i < 32U; i++) {
    if (*ptr_uint32_t != 0U) {
      addr       = *ptr_uint32_t & ~0x0FU;
      ptr_tmp_ED = (USBH_OHCI_ED *)addr;
      while (ptr_tmp_ED != NULL) {
        if (ptr_tmp_ED == ptr_ED) { cnt++; }
        if (ptr_tmp_ED->NextED != 0U) {
          ptr_tmp_ED = (USBH_OHCI_ED *)((uint32_t)ptr_tmp_ED->NextED);
        } else {
          ptr_tmp_ED =  NULL;
        }
      }
    }
    ptr_uint32_t++;
  }

  return cnt;
}

/**
  \fn          bool USBH_OHCI_ED_ClearIntEntries (uint8_t ctrl, USBH_OHCI_ED *ptr_ED)
  \brief       Clear entries in Interrupt Table for Interrupt Endpoint
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \return      true = success, false = fail
*/
static bool USBH_OHCI_ED_ClearIntEntries (uint8_t ctrl, USBH_OHCI_ED *ptr_ED) {
  uint32_t *ptr_uint32_t;
  uint32_t  i;

  if (ptr_ED == NULL) { return false; }

  ptr_uint32_t = (usbh_ohci_ptr[ctrl])->ptr_HCCA;
  if (*ptr_uint32_t != 0U) {
    for (i = 0U; i < 32U; i++) {
      if (*ptr_uint32_t != 0U) {
        *ptr_uint32_t = (uint32_t)(USBH_OHCI_ED_Unlink ((USBH_OHCI_ED *)(*ptr_uint32_t), ptr_ED));
      }
      ptr_uint32_t++;
    }
  }

  return true;
}

/**
  \fn          uint32_t USBH_OHCI_CalcIntEntriesNum (uint8_t ctrl, uint8_t interval)
  \brief       Calculate number of entries in Interrupt Table for Interrupt Endpoint
               required to achieve requested polling interval
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   interval    Endpoint interval (in ms)
  \return      Number of entries in Interrupt Table required to achieve requested polling interval
*/
static uint32_t USBH_OHCI_CalcIntEntriesNum (uint8_t ctrl, uint8_t interval) {

  (void)ctrl;

  if (interval > 32U) {                 // Special case only 1 entry
    return 1U;
  }
  if (interval <= 1U) {                 // Special case all entries
    return 32U;
  }

  while (((32U % interval) != 0U) && (interval > 1U)) {
    interval--;
  }

  return (32U / (uint32_t)interval);
}

/**
  \fn          bool USBH_OHCI_ED_SetIntEntries (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, uint8_t ep_speed, uint8_t ep_interval)
  \brief       Set Endpoint Descriptor entries in Interrupt Table for Interrupt Endpoint
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor
  \param[in]   ep_speed    Endpoint speed
  \param[in]   ep_interval Endpoint polling interval
  \return      true = success, false = fail
*/
static bool USBH_OHCI_ED_SetIntEntries (uint8_t ctrl, USBH_OHCI_ED *ptr_ED, uint8_t ep_speed, uint8_t ep_interval) {
  USBH_OHCI_ED   *ptr_tmp_ED;
  USBH_OHCI_ED   *ptr_prev_ED;
  uint32_t       *ptr_uint32_t;
  uint32_t        num, step, i;
  uint32_t        addr;

  (void)ep_speed;

  if (USBH_OHCI_ED_ClearIntEntries (ctrl, ptr_ED) == false) { return false; }

  num  = USBH_OHCI_CalcIntEntriesNum (ctrl, ep_interval);
  step = 32U / num;

  ptr_prev_ED = NULL;

  // Search for ED which has same number or more entries then requested ED,
  // and if found link requested ED in front of found ED, otherwise link
  // requested ED after last ED
  ptr_uint32_t = (uint32_t *)((usbh_ohci_ptr[ctrl])->ptr_HCCA);
  addr         = *ptr_uint32_t & ~0x0FU;
  ptr_tmp_ED   = (USBH_OHCI_ED *)addr;
  if (*ptr_uint32_t != 0U) {
    // If starting table pointer is not empty find where new qH should be added
    for (;;) {
      if (USBH_OHCI_ED_CountIntEntries (ctrl, ptr_tmp_ED) >= num) {
        // We need to put new ED in front of current one
        ptr_prev_ED = USBH_OHCI_ED_GetPrevious (ctrl, ptr_tmp_ED);
        ptr_ED->NextED = (uint32_t)(ptr_tmp_ED);
        if (ptr_prev_ED != NULL) {
          ptr_prev_ED->NextED = (uint32_t)(ptr_ED);
        }
        break;
      } else if (ptr_tmp_ED->NextED != 0U) {
        // Go to next ED in chain as we have not found where to put new ED yet
        ptr_tmp_ED = (USBH_OHCI_ED *)(ptr_tmp_ED->NextED);
      } else {
        // We need to put new ED at the end
        ptr_prev_ED = ptr_tmp_ED;
        ptr_tmp_ED->NextED = (uint32_t)(ptr_ED);
        break;
      }
    }
  }

  // Find any corresponding table pointer that should start with requested ED
  ptr_uint32_t = (uint32_t *)((usbh_ohci_ptr[ctrl])->ptr_HCCA);
  for (i = 0U; i < num; i++) {
    if ((*ptr_uint32_t == 0U) ||                  // If empty pointer or
        (ptr_prev_ED   == NULL)) {                // if no previous ED
      *ptr_uint32_t = (uint32_t)ptr_ED;
    }
    ptr_uint32_t += step;
  }

  return true;
}

/**
  \fn          void USBH_OHCI_TI_ClearAll (uint8_t ctrl)
  \brief       Clear all Transfer Info entries
  \param[in]   ctrl        Index of USB Host controller
*/
static void USBH_OHCI_TI_ClearAll (uint8_t ctrl) {

  memset((usbh_ohci_ptr[ctrl])->ptr_TI, 0, (USBH_OHCI_MAX_PIPES)*sizeof(USBH_TransferInfo_t));
}

/**
  \fn          USBH_TransferInfo_t *USBH_OHCI_TI_GetFree (uint8_t ctrl)
  \brief       Get Free Transfer Info entry
  \param[in]   ctrl        Index of USB Host controller
  \return                  Pointer to first free Transfer Info entry, no free or function failed
               value > 0:  pointer to first free Transfer Info entry
               value = 0:  no free Transfer Info entry or function failed
*/
static USBH_TransferInfo_t *USBH_OHCI_TI_GetFree (uint8_t ctrl) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  ptr_TI = (usbh_ohci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (uint32_t)(usbh_ohci_ptr[ctrl])->max_TD + (uint32_t)(usbh_ohci_ptr[ctrl])->max_ITD;
  while (cnt != 0U) {
    cnt--;
    if (ptr_TI->ptr_pipe == NULL) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

/**
  \fn          USBH_TransferInfo_t *USBH_OHCI_TI_Find_TD (uint8_t ctrl, USBH_OHCI_TD *ptr_TD)
  \brief       Get Transfer Info entry referring to general Transfer Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_TD      Pointer to general Transfer Descriptor to find
  \return                  Pointer to Transfer Info entry, no corresponding entry or function failed
               value > 0:  pointer to Transfer Info entry
               value = 0:  no Transfer Info entry found or function failed
*/
static USBH_TransferInfo_t *USBH_OHCI_TI_Find_TD (uint8_t ctrl, USBH_OHCI_TD *ptr_TD) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  if (ptr_TD == NULL) { return NULL; }

  ptr_TI = (usbh_ohci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (uint32_t)(usbh_ohci_ptr[ctrl])->max_TD + (uint32_t)(usbh_ohci_ptr[ctrl])->max_ITD;
  while (cnt != 0U) {
    cnt--;
    if (((uint32_t)ptr_TI->ptr_transfer & ~0x0FU) == (uint32_t)ptr_TD) { return ptr_TI; }
    ptr_TI++;
  }

  return NULL;
}

/**
  \fn          USBH_TransferInfo_t *USBH_OHCI_TI_Find_ED (uint8_t ctrl, USBH_OHCI_ED ptr_ED)
  \brief       Get Transfer Info entry referring to Endpoint Descriptor
  \param[in]   ctrl        Index of USB Host controller
  \param[in]   ptr_ED      Pointer to Endpoint Descriptor to find
  \return                  Pointer to Transfer Info entry, no corresponding entry or function failed
               value > 0:  pointer to Transfer Info entry
               value = 0:  no Transfer Info entry found or function failed
*/
static USBH_TransferInfo_t *USBH_OHCI_TI_Find_ED (uint8_t ctrl, USBH_OHCI_ED *ptr_ED) {
  USBH_TransferInfo_t *ptr_TI;
  uint32_t             cnt;

  if (ptr_ED == NULL) { return NULL; }

  ptr_TI = (usbh_ohci_ptr[ctrl])->ptr_TI; if (ptr_TI == NULL) { return NULL; }

  cnt = (usbh_ohci_ptr[ctrl])->max_ED;
  while (cnt != 0U) {
    cnt--;
    if ((uint32_t)ptr_TI->ptr_pipe == (uint32_t)ptr_ED) { return ptr_TI; }
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
#if   (USBH_OHCI_INSTANCES >= 2)
static ARM_DRIVER_VERSION USBH1_HW_GetVersion (void) { return usbh_driver_version; }
#endif

/**
  \fn          ARM_USBH_CAPABILITIES USBH_HW_GetCapabilities (void)
  \brief       Get driver capabilities.
  \param[in]   ctrl  Index of USB Host controller
  \return      \ref ARM_USBH_CAPABILITIES
*/
static ARM_USBH_CAPABILITIES USBH_HW_GetCapabilities  (uint8_t ctrl) {
  uint32_t port_num = (usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorA & 0xFU);

  if (port_num == 0U) {
    port_num = 1U;
  }

  usbh_driver_capabilities[ctrl].port_mask = (1U << port_num) - 1U;

  return usbh_driver_capabilities[ctrl];
}
static ARM_USBH_CAPABILITIES USBH0_HW_GetCapabilities (void)         { return USBH_HW_GetCapabilities (0); }
#if   (USBH_OHCI_INSTANCES >= 2)
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

  if (ctrl >= USBH_OHCI_INSTANCES) { return ARM_DRIVER_ERROR; }

  usbh_ohci_reg_ptr[ctrl] = (USBH_OHCI_Registers_t *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_OHCI);

  signal_port_event[ctrl] = cb_port_event;
  signal_pipe_event[ctrl] = cb_pipe_event;

  (void)USBH_HW_GetCapabilities(ctrl);

  USBH_OHCI_TI_ClearAll (ctrl);

  ret = USBH_OHCI_HW_Initialize((uint8_t)(usbh_ohci_ptr[ctrl])->ctrl, (usbh_ohci_ptr[ctrl])->irq_handler);

  if (ret != 0) {
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_Initialize (ARM_USBH_SignalPortEvent_t cb_port_event, ARM_USBH_SignalPipeEvent_t cb_pipe_event) { return USBH_HW_Initialize (0, cb_port_event, cb_pipe_event); }
#if   (USBH_OHCI_INSTANCES >= 2)
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

  ret = USBH_OHCI_HW_Uninitialize((uint8_t)(usbh_ohci_ptr[ctrl])->ctrl);

  if (ret != 0) {
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_Uninitialize (void) { return USBH_HW_Uninitialize(0); }
#if   (USBH_OHCI_INSTANCES >= 2)
static int32_t USBH1_HW_Uninitialize (void) { return USBH_HW_Uninitialize(1); }
#endif

/**
  \fn          int32_t USBH_HW_PowerControl (uint8_t ctrl, ARM_POWER_STATE state)
  \brief       Control USB Host Interface Power.
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBH_HW_PowerControl (uint8_t ctrl, ARM_POWER_STATE state) {
  int32_t status;

  switch (state) {
    case ARM_POWER_OFF:
      // Disable OHCI interrupts
      usbh_ohci_reg_ptr[ctrl]->HcInterruptDisable = USBH_OHCI_HcInterruptEnable_MIE | USBH_OHCI_HcInterruptStatus_ALL;

      // HCFS = USB Suspend state
      usbh_ohci_reg_ptr[ctrl]->HcControl          = ((usbh_ohci_reg_ptr[ctrl]->HcControl & ~USBH_OHCI_HcControl_HCFS) | USBH_OHCI_FS_USB_SUSPEND);

      // USB Reset state
      usbh_ohci_reg_ptr[ctrl]->HcControl          = 0;

      // Put reset values to all of the OHCI registers
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorA   &=~USBH_OHCI_HcRhDescriptorA_PSM;
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorA   |= USBH_OHCI_HcRhDescriptorA_NPS;
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorB   &=~0xFFFF0000U;
      usbh_ohci_reg_ptr[ctrl]->HcCommandStatus    = 0;
      usbh_ohci_reg_ptr[ctrl]->HcInterruptStatus  = 0;
      usbh_ohci_reg_ptr[ctrl]->HcInterruptDisable = USBH_OHCI_HcInterruptEnable_MIE | USBH_OHCI_HcInterruptStatus_ALL;
      usbh_ohci_reg_ptr[ctrl]->HcHCCA             = 0;
      usbh_ohci_reg_ptr[ctrl]->HcControlHeadED    = 0;
      usbh_ohci_reg_ptr[ctrl]->HcControlCurrentED = 0;
      usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED       = 0;
      usbh_ohci_reg_ptr[ctrl]->HcBulkCurrentED    = 0;
      usbh_ohci_reg_ptr[ctrl]->HcFmInterval       = USBH_OHCI_HcFmInterval_FSMPS(0x2778) | USBH_OHCI_HcFmInterval_FI(0x2EDF);
      usbh_ohci_reg_ptr[ctrl]->HcPeriodicStart    = 0;
      usbh_ohci_reg_ptr[ctrl]->HcLSThreshold      = USBH_OHCI_HcLSThreshold_LST(0x0628);

      status = USBH_OHCI_HW_PowerControl((uint8_t)(usbh_ohci_ptr[ctrl])->ctrl, 0U);
      if (status != 0) {
        return ARM_DRIVER_ERROR;
      }
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR;

    case ARM_POWER_FULL:
      status = USBH_OHCI_HW_PowerControl((uint8_t)(usbh_ohci_ptr[ctrl])->ctrl, 1U);
      if (status != 0) {
        return ARM_DRIVER_ERROR;
      }

      // Initialize memories
      USBH_OHCI_HCCA_Clear     (ctrl);
      USBH_OHCI_TD_ClearAll    (ctrl);
    //USBH_OHCI_ITD_ClearAll   (ctrl);
      USBH_OHCI_ED_ClearAll    (ctrl);
      USBH_OHCI_TI_ClearAll    (ctrl);

      usbh_ohci_reg_ptr[ctrl]->HcControl       = 0;   // Host Controller Reset
      usbh_ohci_reg_ptr[ctrl]->HcControlHeadED = 0;   // Initialize Control list head
      usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED    = 0;   // Initialize Bulk list head

      usbh_ohci_reg_ptr[ctrl]->HcCommandStatus = USBH_OHCI_HcCommandStatus_HCR;   // Host Ctrl Reset
      usbh_ohci_reg_ptr[ctrl]->HcFmInterval    = USBH_OHCI_HcFmInterval_FSMPS(0x2778)|// Max Pckt Sz
                               USBH_OHCI_HcFmInterval_FI(0x2EDF);                 // Frm Intrvl
      usbh_ohci_reg_ptr[ctrl]->HcPeriodicStart = 0x2A2F;  // 10% before end of HcFmInterval

      usbh_ohci_reg_ptr[ctrl]->HcHCCA          = (uint32_t)((usbh_ohci_ptr[ctrl])->ptr_HCCA);   // HCCA
      usbh_ohci_reg_ptr[ctrl]->HcInterruptDisable=USBH_OHCI_HcInterruptEnable_MIE;// Master int dis
      usbh_ohci_reg_ptr[ctrl]->HcInterruptStatus = usbh_ohci_reg_ptr[ctrl]->HcInterruptStatus;// Clear int stat

      // HCFS = USB Operational state
      usbh_ohci_reg_ptr[ctrl]->HcControl       = ((usbh_ohci_reg_ptr[ctrl]->HcControl & ~USBH_OHCI_HcControl_HCFS) | USBH_OHCI_FS_USB_OPERATIONAL);

      // Enable individually powered ports
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorB|= 0xFFFF0000U;
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorA&=~USBH_OHCI_HcRhDescriptorA_NPS;
      usbh_ohci_reg_ptr[ctrl]->HcRhDescriptorA|= USBH_OHCI_HcRhDescriptorA_PSM;

      // Enable interrupts
      usbh_ohci_reg_ptr[ctrl]->HcInterruptEnable= (USBH_OHCI_HcInterruptEnable_MIE |  // Master int en
                               USBH_OHCI_HcInterruptStatus_RHSC|// Root hub stat
                               USBH_OHCI_HcInterruptStatus_WDH |// Writeback done
                               USBH_OHCI_HcInterruptStatus_SF );// Start of Frame
      break;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PowerControl (ARM_POWER_STATE state) { return USBH_HW_PowerControl (0, state); }
#if   (USBH_OHCI_INSTANCES >= 2)
static int32_t USBH1_HW_PowerControl (ARM_POWER_STATE state) { return USBH_HW_PowerControl (1, state); }
#endif

/**
  \fn          int32_t USBH_HW_PortVbusOnOff (uint8_t port, bool vbus)
  \brief       Root HUB Port VBUS on/off.
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \param[in]   vbus
                - \b false VBUS off
                - \b true  VBUS on
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortVbusOnOff (uint8_t ctrl, uint8_t port, bool vbus) {

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) != 0U) {
    if (vbus) {
      usbh_ohci_reg_ptr[ctrl]->HcRhStatus           |= USBH_OHCI_HcRhStatus_LPSC;   // For global power power
      usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port] |= USBH_OHCI_HcRhPortStatus_PPS;
    } else {
      usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port] |= USBH_OHCI_HcRhPortStatus_LSDA;
      usbh_ohci_reg_ptr[ctrl]->HcRhStatus           |= USBH_OHCI_HcRhStatus_LPS;    // For global power power
    }
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortVbusOnOff (uint8_t port, bool vbus) { return USBH_HW_PortVbusOnOff(0, port, vbus); }
#if   (USBH_OHCI_INSTANCES >= 2)
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

  if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << port)) != 0U) {
    usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port] |=  USBH_OHCI_HcRhPortStatus_PRS;      // Port Reset
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortReset (uint8_t port) { return USBH_HW_PortReset (0, port); }
#if   (USBH_OHCI_INSTANCES >= 2)
static int32_t USBH1_HW_PortReset (uint8_t port) { return USBH_HW_PortReset (1, port); }
#endif

/**
  \fn          int32_t USBH_HW_PortSuspend (uint8_t port)
  \brief       Suspend Root HUB Port (stop generating SOFs).
  \param[in]   ctrl  Index of USB Host controller
  \param[in]   port  Root HUB Port Number
  \return      \ref execution_status
*/
static int32_t USBH_HW_PortSuspend (uint8_t ctrl, uint8_t port) {

  usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port] |= USBH_OHCI_HcRhPortStatus_PSS;  // Port Suspend

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortSuspend (uint8_t port) { return USBH_HW_PortSuspend (0, port); }
#if   (USBH_OHCI_INSTANCES >= 2)
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

  usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port] |= USBH_OHCI_HcRhPortStatus_POCI; // Port Resume
  (void)osDelay(20U);

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PortResume (uint8_t port) { return USBH_HW_PortResume (0, port); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  ARM_USBH_PORT_STATE port_state;
  uint32_t            HcRhPortStatus_val;

  HcRhPortStatus_val  = usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[port];

  if ((HcRhPortStatus_val & USBH_OHCI_HcRhPortStatus_CCS ) != 0U) {
    port_state.connected   = 1U;
  } else {
    port_state.connected   = 0U;
  }
  if ((HcRhPortStatus_val & USBH_OHCI_HcRhPortStatus_POCI) != 0U) {
    port_state.overcurrent = 1U;
  } else {
    port_state.overcurrent = 0U;
  }
  if ((HcRhPortStatus_val & USBH_OHCI_HcRhPortStatus_LSDA) != 0U) {
    port_state.speed = ARM_USB_SPEED_LOW;
  } else {
    port_state.speed = ARM_USB_SPEED_FULL;
  }

  return port_state;
}
static ARM_USBH_PORT_STATE USBH0_HW_PortGetState (uint8_t port) { return USBH_HW_PortGetState (0, port); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  USBH_OHCI_ED        *ptr_ED;
  USBH_OHCI_ED        *ptr_prev_ED;
  USBH_OHCI_ED        *ptr_first_ED;
  USBH_TransferInfo_t *ptr_TI;

  (void)hub_addr;
  (void)hub_port;

  // Get free ED and TI
  ptr_ED = USBH_OHCI_ED_GetFree (ctrl); if (ptr_ED == NULL) { return 0U; }
  ptr_TI = USBH_OHCI_TI_GetFree (ctrl); if (ptr_TI == NULL) { return 0U; }

  memset(ptr_TI, 0, sizeof(USBH_TransferInfo_t));

  // Fill in all fields of Endpoint Descriptor
  // Use scratch pad 0 area for endpoint type information
  ptr_ED->DW0.SCR   = ep_type | (1U << 4);
  ptr_ED->DW0.MPS   = ep_max_packet_size;
  if (ep_type == (uint32_t)ARM_USB_ENDPOINT_ISOCHRONOUS) {
    ptr_ED->DW0.F   = 1U;
  } else {
    ptr_ED->DW0.F   = 0U;
  }
  ptr_ED->DW0.K     = 1U;               // Set skip bit of endpoint descriptor (stop processing it)
  if (dev_speed == (uint32_t)ARM_USB_SPEED_LOW) {
    ptr_ED->DW0.S   = 1U;
  } else {
    ptr_ED->DW0.S   = 0U;
  }
  if (ep_type == (uint32_t)ARM_USB_ENDPOINT_CONTROL) {
    ptr_ED->DW0.D   = 0U;
  } else {
    ptr_ED->DW0.D   = 1U + ((ep_addr >> 7) & 0x01U);
  }
  ptr_ED->DW0.EN    = ep_addr  & 0x0FU;
  ptr_ED->DW0.FA    = dev_addr & 0x7FU;
  ptr_ED->TailP     = 0U;
  ptr_ED->DW2.C     = 0U;
  ptr_ED->DW2.H     = 0U;
  ptr_ED->DW2.HeadP = 0U;
  ptr_ED->NextED    = 0U;

  // Find last Endpoint Descriptor and correct it to point to this one
  switch (ep_type) {                    // Endpoint Type
    case ARM_USB_ENDPOINT_CONTROL:
      ptr_first_ED = (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcControlHeadED);
      if (ptr_first_ED == NULL) {
        usbh_ohci_reg_ptr[ctrl]->HcControlHeadED = (uint32_t)ptr_ED;
      } else {
        ptr_prev_ED = USBH_OHCI_ED_GetLast (ptr_first_ED);
        ptr_prev_ED->NextED = (uint32_t)ptr_ED;
      }
      usbh_ohci_reg_ptr[ctrl]->HcControl       |=  USBH_OHCI_HcControl_CLE;
      break;
    case ARM_USB_ENDPOINT_ISOCHRONOUS:
      usbh_ohci_reg_ptr[ctrl]->HcControl       |=  USBH_OHCI_HcControl_IE;
      break;
    case ARM_USB_ENDPOINT_BULK:
      ptr_first_ED = (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED);
      if (ptr_first_ED == NULL) {
        usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED = (uint32_t)ptr_ED;
      } else {
        ptr_prev_ED = USBH_OHCI_ED_GetLast (ptr_first_ED);
        ptr_prev_ED->NextED = (uint32_t)ptr_ED;
      }
      usbh_ohci_reg_ptr[ctrl]->HcCommandStatus |=  USBH_OHCI_HcCommandStatus_BLF;
      usbh_ohci_reg_ptr[ctrl]->HcControl       |=  USBH_OHCI_HcControl_BLE;
      break;
    case ARM_USB_ENDPOINT_INTERRUPT:
      (void)USBH_OHCI_ED_SetIntEntries (ctrl, ptr_ED, dev_speed, ep_interval);
      usbh_ohci_reg_ptr[ctrl]->HcControl       |=  USBH_OHCI_HcControl_PLE;
      break;
    default:
      return 0U;
  }

  ptr_TI->ptr_pipe = (uint32_t *)((uint32_t)ptr_ED);

  return ((ARM_USBH_PIPE_HANDLE)(ptr_ED));
}
static ARM_USBH_PIPE_HANDLE USBH0_HW_PipeCreate (uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t  ep_interval) { return USBH_HW_PipeCreate (0, dev_addr, dev_speed, hub_addr, hub_port, ep_addr, ep_type, ep_max_packet_size, ep_interval); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  USBH_OHCI_ED *ptr_ED;

  (void)ctrl;
  (void)hub_addr;
  (void)hub_port;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ED = (USBH_OHCI_ED *)(pipe_hndl);

  // Update only maximum packet size, device address and device speed
  // Use scratch pad 0 area for endpoint type information
  ptr_ED->DW0.K   = 1U;                 // Set skip bit of endpoint descriptor (stop processing it)
  (void)osDelay(1U);                    // Wait ~1 ms to be sure that endpoint descriptor is not being processed

  ptr_ED->DW0.MPS = ep_max_packet_size;
  if (dev_speed == (uint32_t)ARM_USB_SPEED_LOW) {
    ptr_ED->DW0.S = 1U;
  } else {
    ptr_ED->DW0.S = 0U;
  }
  ptr_ED->DW0.FA  = dev_addr & 0x7FU;
  ptr_ED->DW0.K   = 0U;                 // Clear skip bit of endpoint descriptor (start processing it)

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeModify (ARM_USBH_PIPE_HANDLE pipe_hndl, uint8_t dev_addr, uint8_t dev_speed, uint8_t hub_addr, uint8_t hub_port, uint16_t ep_max_packet_size) { return USBH_HW_PipeModify (0, pipe_hndl, dev_addr, dev_speed, hub_addr, hub_port, ep_max_packet_size); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  USBH_OHCI_ED        *ptr_ED;
  USBH_TransferInfo_t *ptr_TI;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }
  ptr_ED = (USBH_OHCI_ED *)pipe_hndl;

  ptr_TI = USBH_OHCI_TI_Find_ED (ctrl, ptr_ED); if (ptr_TI == NULL) { return ARM_DRIVER_OK;    }
  if (ptr_TI->active == 0U) { return ARM_DRIVER_OK; }   // If transfer is not active

  if (!USBH_OHCI_TD_Dequeue (ctrl, ptr_ED, (USBH_OHCI_TD *)((uint32_t)ptr_TI->ptr_transfer))) {
    return ARM_DRIVER_ERROR;
  }
  ptr_TI->active = 0U;

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeTransferAbort (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferAbort (0, pipe_hndl); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  USBH_OHCI_ED        *ptr_ED;
  USBH_TransferInfo_t *ptr_TI;
  USBH_OHCI_ED        *ptr_prev_ED;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ED = (USBH_OHCI_ED *)pipe_hndl;
  ptr_TI = USBH_OHCI_TI_Find_ED (ctrl, ptr_ED); if (ptr_TI == NULL) { return ARM_DRIVER_ERROR; }
  if (ptr_TI->active != 0U) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  // If NextED exists check if currently this ED is being processed if so set
  // currently processed ED to NextED
  ptr_ED->DW0.K   = 1U;                 // Set skip bit of endpoint descriptor (stop processing it)
  (void)osDelay(1U);                    // Wait ~1 ms to be sure that endpoint descriptor is not being processed

  switch(ptr_ED->DW0.SCR & 3U) {        // Endpoint Type
    case ARM_USB_ENDPOINT_CONTROL:
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcControlCurrentED)) {
        usbh_ohci_reg_ptr[ctrl]->HcControlCurrentED = ptr_ED->NextED;
      }
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcControlHeadED)) {
        usbh_ohci_reg_ptr[ctrl]->HcControlHeadED = ptr_ED->NextED;
        if (ptr_ED->NextED == 0U) {
          usbh_ohci_reg_ptr[ctrl]->HcControl &= ~USBH_OHCI_HcControl_CLE;
        }
      }
      break;
    case ARM_USB_ENDPOINT_ISOCHRONOUS:
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcPeriodCurrentED)) {
        usbh_ohci_reg_ptr[ctrl]->HcPeriodCurrentED = ptr_ED->NextED;
      }
      if ((uint32_t *)((usbh_ohci_ptr[ctrl])->ptr_HCCA) == 0U) {
        // If first interrupt table entry is empty (there are no more isochronous pipes)
        usbh_ohci_reg_ptr[ctrl]->HcControl &= ~USBH_OHCI_HcControl_IE;
      }
      break;
    case ARM_USB_ENDPOINT_BULK:
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcBulkCurrentED)) {
        usbh_ohci_reg_ptr[ctrl]->HcBulkCurrentED = ptr_ED->NextED;
      }
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED)) {
        usbh_ohci_reg_ptr[ctrl]->HcBulkHeadED = ptr_ED->NextED;
        if (ptr_ED->NextED == 0U) {
          usbh_ohci_reg_ptr[ctrl]->HcControl &= ~USBH_OHCI_HcControl_BLE;
        }
      }
      break;
    case ARM_USB_ENDPOINT_INTERRUPT:
      if (ptr_ED == (USBH_OHCI_ED *)(usbh_ohci_reg_ptr[ctrl]->HcPeriodCurrentED)) {
        usbh_ohci_reg_ptr[ctrl]->HcPeriodCurrentED = ptr_ED->NextED;
      }
      (void)USBH_OHCI_ED_ClearIntEntries (ctrl, ptr_ED);
      if ((uint32_t *)((usbh_ohci_ptr[ctrl])->ptr_HCCA) == 0U) {
        // If first interrupt table entry is empty (there are no more interrupt pipes)
        usbh_ohci_reg_ptr[ctrl]->HcControl &= ~USBH_OHCI_HcControl_PLE;
      }
      break;
    default:
      return ARM_DRIVER_ERROR;
  }

  // If previous ED exists link it to next ED
  ptr_prev_ED = USBH_OHCI_ED_GetPrevious (ctrl, ptr_ED);
  if (ptr_prev_ED != NULL) {
    ptr_prev_ED->NextED = ptr_ED->NextED;
  }

  (void)USBH_OHCI_ED_Clear (ptr_ED);    // Clear Endpoint Descriptor

  memset(ptr_TI, 0, sizeof(USBH_TransferInfo_t));

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeDelete (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeDelete (0, pipe_hndl); }
#if   (USBH_OHCI_INSTANCES >= 2)
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

  ((USBH_OHCI_ED *)pipe_hndl)->DW2.C = 0U;

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeReset (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeReset (0, pipe_hndl); }
#if   (USBH_OHCI_INSTANCES >= 2)
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
  USBH_OHCI_TD        *ptr_TD;
  USBH_TransferInfo_t *ptr_TI;

  if (pipe_hndl == 0U) { return ARM_DRIVER_ERROR; }

  ptr_TD = USBH_OHCI_TD_GetFree (ctrl);                            if (ptr_TD == NULL) { return ARM_DRIVER_ERROR; }
  ptr_TI = USBH_OHCI_TI_Find_ED (ctrl, (USBH_OHCI_ED *)pipe_hndl); if (ptr_TI == NULL) { return ARM_DRIVER_ERROR; }
  if (ptr_TI->active != 0U) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  ptr_TI->ptr_transfer          = (uint32_t *)((uint32_t)ptr_TD);
  ptr_TI->packet                = packet;
  ptr_TI->data                  = data;
  ptr_TI->num                   = num;
  ptr_TI->packet                = packet;
  ptr_TI->num_transferred_total = 0;
  if (num >= 4096U) {
    ptr_TI->num_to_transfer     = 4096U;
  } else {
    ptr_TI->num_to_transfer     = num;
  }

  if (!USBH_OHCI_TD_Enqueue (ctrl, (USBH_OHCI_ED *)pipe_hndl, ptr_TD, packet, data, ptr_TI->num_to_transfer)) {
    return ARM_DRIVER_ERROR;
  }

  ptr_TI->active                = 1U;

  // Activate control or bulk transfer
  switch (((USBH_OHCI_ED *)pipe_hndl)->DW0.SCR & 3U) {          // Endpoint Type
    case ARM_USB_ENDPOINT_CONTROL:
      usbh_ohci_reg_ptr[ctrl]->HcCommandStatus |=  USBH_OHCI_HcCommandStatus_CLF;
      break;
    case ARM_USB_ENDPOINT_ISOCHRONOUS:
      break;
    case ARM_USB_ENDPOINT_BULK:
      usbh_ohci_reg_ptr[ctrl]->HcCommandStatus |=  USBH_OHCI_HcCommandStatus_BLF;
      break;
    case ARM_USB_ENDPOINT_INTERRUPT:
      break;
    default:
      return false;
  }

  return ARM_DRIVER_OK;
}
static int32_t USBH0_HW_PipeTransfer (ARM_USBH_PIPE_HANDLE pipe_hndl, uint32_t packet, uint8_t *data, uint32_t num) { return USBH_HW_PipeTransfer (0, pipe_hndl, packet, data, num); }
#if   (USBH_OHCI_INSTANCES >= 2)
static int32_t USBH1_HW_PipeTransfer (ARM_USBH_PIPE_HANDLE pipe_hndl, uint32_t packet, uint8_t *data, uint32_t num) { return USBH_HW_PipeTransfer (1, pipe_hndl, packet, data, num); }
#endif

/**
  \fn          uint32_t USBH_HW_PipeTransferGetResult (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl)
  \brief       Get result of USB Pipe transfer.
  \param[in]   ctrl       Index of USB Host controller
  \param[in]   pipe_hndl  Pipe Handle
  \return      number of successfully transfered data bytes
*/
static uint32_t USBH_HW_PipeTransferGetResult (uint8_t ctrl, ARM_USBH_PIPE_HANDLE pipe_hndl) {
  const USBH_TransferInfo_t *ptr_TI;

  if (pipe_hndl == 0U) { return 0U; }

  ptr_TI = USBH_OHCI_TI_Find_ED (ctrl, (USBH_OHCI_ED *)pipe_hndl); if (ptr_TI == NULL) { return 0U; }

  return (ptr_TI->num_transferred_total);
}
static uint32_t USBH0_HW_PipeTransferGetResult (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferGetResult (0, pipe_hndl); }
#if   (USBH_OHCI_INSTANCES >= 2)
static uint32_t USBH1_HW_PipeTransferGetResult (ARM_USBH_PIPE_HANDLE pipe_hndl) { return USBH_HW_PipeTransferGetResult (1, pipe_hndl); }
#endif

/**
  \fn          uint16_t USBH_HW_GetFrameNumber (uint8_t ctrl)
  \brief       Get current USB Frame Number.
  \param[in]   ctrl     Index of USB Host controller
  \return      Frame Number
*/
static uint16_t USBH_HW_GetFrameNumber (uint8_t ctrl) {

  return ((((USBH_OHCI_HCCA *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_HCCA))->HccaFrameNumber) >> 5);
}
static uint16_t USBH0_HW_GetFrameNumber (void) { return USBH_HW_GetFrameNumber (0); }
#if   (USBH_OHCI_INSTANCES >= 2)
static uint16_t USBH1_HW_GetFrameNumber (void) { return USBH_HW_GetFrameNumber (1); }
#endif

/// \brief         Interrupt handling routine
/// \param[in]     ctrl          index of USB Host controller
static void USBH_HW_IRQ_Handler (uint8_t ctrl) {
  ARM_USBH_PIPE_HANDLE  pipe_handle;
  USBH_OHCI_TD         *ptr_TD, *ptr_TD_next;
  USBH_TransferInfo_t  *ptr_TI;
  uint32_t              int_stat;
  uint32_t              int_en;
  uint32_t              rh_stat;
  uint32_t              rhport_stat;
  uint32_t              hcca_done_head;
  uint32_t              cc;
  uint32_t              ep_event;
  uint32_t              transferred;
  uint8_t               i;

  // Read all registers for interrupt handling
  int_stat       = usbh_ohci_reg_ptr[ctrl]->HcInterruptStatus;
  int_en         = usbh_ohci_reg_ptr[ctrl]->HcInterruptEnable;
  rh_stat        = usbh_ohci_reg_ptr[ctrl]->HcRhStatus;
  hcca_done_head = ((USBH_OHCI_HCCA *)((uint32_t)(usbh_ohci_ptr[ctrl])->ptr_HCCA))->HccaDoneHead;
  usbh_ohci_reg_ptr[ctrl]->HcInterruptStatus = int_stat; // Clear Interrupt Status

  // Analyze interrupt
  if ((int_stat & int_en) != 0U) {      // Check if not spurious Interrupt

    if ((int_stat & USBH_OHCI_HcInterruptStatus_RHSC) != 0U) {
                                        // Root Hub Status Change
      for (i = 0U; i < 15U; i ++) {     // Go through all ports
        if ((usbh_driver_capabilities[ctrl].port_mask & (1UL << i)) != 0U) {
                                        // If port is enabled
          rhport_stat = usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[i];
          usbh_ohci_reg_ptr[ctrl]->HcRhPortStatus[i] = rhport_stat & 0xFFFF0000U;
          if ((rhport_stat & USBH_OHCI_HcRhPortStatus_OCIC) != 0U){
                                        // Port Over-current Indicator Changed
            (void)USBH_HW_PortVbusOnOff (ctrl, i, false);
            signal_port_event[ctrl](i, ARM_USBH_EVENT_OVERCURRENT);
          }
          if ((rhport_stat & USBH_OHCI_HcRhPortStatus_CSC) != 0U){
                                        // Connect Status Change
                                        // Clear Connect Status Change
            if ((rhport_stat & USBH_OHCI_HcRhPortStatus_CCS) != 0U) {
                                        // Device Connected
              if ((rh_stat & USBH_OHCI_HcRhStatus_DRWE) != 0U){
                                        // Remote Wake-up event
                signal_port_event[ctrl](i, ARM_USBH_EVENT_REMOTE_WAKEUP);
              } else {
                signal_port_event[ctrl](i, ARM_USBH_EVENT_CONNECT);
              }
            } else {
                                        // Device Disconnected
              signal_port_event[ctrl](i, ARM_USBH_EVENT_DISCONNECT);
            }
          }
          if ((rhport_stat & USBH_OHCI_HcRhPortStatus_PRSC) != 0U) {
                                        // Port Reset Status Change
                                        // Clear Port Reset Status
            if (!(rhport_stat & USBH_OHCI_HcRhPortStatus_PRS)) {
                                        // Port Reset not Active
              signal_port_event[ctrl](i, ARM_USBH_EVENT_RESET);
            }
          }
          if ((rhport_stat & USBH_OHCI_HcRhPortStatus_PSSC) != 0U){
            if ((rhport_stat & USBH_OHCI_HcRhPortStatus_PSS) != 0U) {
                                        // Port Suspend Status
              signal_port_event[ctrl](i, ARM_USBH_EVENT_SUSPEND);
            } else {
              signal_port_event[ctrl](i, ARM_USBH_EVENT_RESUME);
            }
          }
        } else {
          if ((usbh_driver_capabilities[ctrl].port_mask & ~((1UL << i) - 1U)) == 0U) {
            break;
          }
        }
      }
    }

    if ((int_stat & USBH_OHCI_HcInterruptStatus_WDH) != 0U) {
                                        // Writeback Done Interrupt
      ptr_TD = (USBH_OHCI_TD *)(hcca_done_head & (~1U)); if (ptr_TD == NULL) { return; }

      do {
        ptr_TD_next = ((USBH_OHCI_TD *)ptr_TD->NextTD);
        ptr_TI      = USBH_OHCI_TI_Find_TD (ctrl, ptr_TD);
        cc          = ptr_TD->DW0.CC;
        if (cc == USBH_OHCI_CC_NOTACCESSED) {   // Transfer not finished
          ptr_TD = ptr_TD_next;
          continue;
        }
        ep_event    = 0U;
        if ((ptr_TI != NULL) && (ptr_TI->active != 0U)) {
          pipe_handle = (ARM_USBH_EP_HANDLE)ptr_TI->ptr_pipe;
          cc          = ptr_TD->DW0.CC;
          transferred = 0U;
          if (cc == USBH_OHCI_CC_NOERROR) {
            if (ptr_TD->CBP == 0U) {
              transferred = ptr_TI->num_to_transfer;
            } else {
              if (ptr_TD->BE > ptr_TD->CBP) {
                transferred = ptr_TI->num_to_transfer - (ptr_TD->BE - ptr_TD->CBP + 1U);
              }
            }
          }
          ptr_TI->num_transferred_total += transferred;
          if (!cc) {
            if (ptr_TI->num == ptr_TI->num_transferred_total) {         // All data was transferred
              ep_event = ARM_USBH_EVENT_TRANSFER_COMPLETE;
            } else {
              if ((((USBH_OHCI_TD *)((uint32_t)ptr_TI->ptr_transfer))->DW0.DP == 2U) && // Receiving from IN endpoint
                   (transferred != ptr_TI->num_to_transfer)) {         // Short packet or ZLP
                ep_event = ARM_USBH_EVENT_TRANSFER_COMPLETE;
              }
            }
            if (!ep_event) {
              // Restart transfer of remaining data
              if ((ptr_TI->num - ptr_TI->num_transferred_total) >= 4096U) {
                ptr_TI->num_to_transfer = 4096U;
              } else {
                ptr_TI->num_to_transfer = (ptr_TI->num - ptr_TI->num_transferred_total);
              }
              (void)USBH_OHCI_TD_Enqueue   (ctrl, (USBH_OHCI_ED *)((uint32_t)ptr_TI->ptr_pipe), (USBH_OHCI_TD *)((uint32_t)ptr_TI->ptr_transfer), ptr_TI->packet, ptr_TI->data + ptr_TI->num_transferred_total, ptr_TI->num_to_transfer);
            }
          } else if (cc == USBH_OHCI_CC_STALL) {
            ep_event = ARM_USBH_EVENT_HANDSHAKE_STALL;
          } else {
            ep_event = ARM_USBH_EVENT_BUS_ERROR;
          }
          if (ep_event != 0U) {
            ptr_TD->DW0.SCR = 0U;
            ptr_TI->ptr_transfer = NULL;
            ptr_TI->active       = 0U;
            signal_pipe_event[ctrl](pipe_handle, ep_event);
          }
        }
        ptr_TD = ptr_TD_next;
      } while (ptr_TD != NULL);
    }

//  if (int_stat & USBH_OHCI_HcInterruptStatus_SF) {
//                                      // Start of Frame Interrupt
//  }
  }
}
static void Driver_USBH0_IRQ_Handler (void) { USBH_HW_IRQ_Handler(0U); }
#if   (USBH_OHCI_INSTANCES >= 2)
static void Driver_USBH1_IRQ_Handler (void) { USBH_HW_IRQ_Handler(1U); }
#endif

ARM_DRIVER_USBH USBHn_DRIVER(USBH0_OHCI_DRV_NUM) = {
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

#if (USBH_OHCI_INSTANCES >= 2)
ARM_DRIVER_USBH USBHn_DRIVER(USBH1_OHCI_DRV_NUM) = {
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
