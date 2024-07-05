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
 * Project:     USB Host OHCI Controller Driver Registers header
 *
 * -----------------------------------------------------------------------------
 */

#ifndef USBH_OHCI_REGS_H_
#define USBH_OHCI_REGS_H_

#include <stdint.h>

// Structure Definitions

typedef struct {                        // OHCI Registers
  volatile uint32_t HcRevision;
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile uint32_t HcFmRemaining;
  volatile uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSThreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus[15];
} USBH_OHCI_Registers_t;

typedef struct {                        // Endpoint Descriptor       (ED)
  struct {
    uint32_t FA             :  7;       // ED Function Address       (FA)
    uint32_t EN             :  4;       // ED Endpoint Number        (EN)
    uint32_t D              :  2;       // ED Direction              (D)
    uint32_t S              :  1;       // ED Speed                  (S)
    uint32_t K              :  1;       // ED Skip                   (K)
    uint32_t F              :  1;       // ED Format                 (F)
    uint32_t MPS            : 11;       // ED Maximum Packet Size    (MPS)
    uint32_t SCR            :  5;       // ED Scratch Bit Field      (SCR)
  } DW0;
  uint32_t TailP;                       // ED TD Queue Tail Pointer  (TailP)
  struct {
    volatile uint32_t H     :  1;       // ED Halted                 (H)
    volatile uint32_t C     :  1;       // ED Toggle Carry           (C)
             uint32_t Rsvd0 :  2;       // Reserved - unused bits
    volatile uint32_t HeadP : 28;       // ED TD Queue Head Pointer  (HeadP)
  } DW2;
  uint32_t NextED;                      // ED Next ED                (NextED)
} USBH_OHCI_ED;

typedef struct {                        // Transfer Descriptor       (TD)
  struct {
             uint32_t SCR   : 18;       // TD Scratch Bit Field      (SCR)
             uint32_t R     :  1;       // TD Buffer Rounding        (R)
             uint32_t DP    :  2;       // TD Direction PID          (DP)
             uint32_t DI    :  3;       // TD Delay Interrupt        (DI)
    volatile uint32_t T     :  2;       // TD Data Toggle            (T)
    volatile uint32_t EC    :  2;       // TD Error Count            (EC)
    volatile uint32_t CC    :  4;       // TD Condition Code         (CC)
  } DW0;
  volatile uint32_t CBP;                         // TD Current Buffer Pointer (CBP)
  volatile uint32_t NextTD;                      // TD Next TD                (NextTD)
  uint32_t BE;                          // TD Buffer End             (BE)
} USBH_OHCI_TD;

typedef struct {                        // Isochronous TD            (ITD)
  struct {
    uint32_t SF             : 16;       // ITD Starting Frame        (SF)
    uint32_t Rsvd0          :  5;       // Reserved - unused bits
    uint32_t DI             :  3;       // ITD Delay Interrupt       (DI)
    uint32_t FC             :  3;       // ITD Frame Count           (FC)
    uint32_t Rsvd1          :  1;       // Reserved - unused bit
    volatile uint32_t CC    :  4;       // ITD Condition Code        (CC)
  } DW0;
  struct {
    uint32_t Rsvd0     : 12;            // Reserved - unused bits
    uint32_t BP0       : 20;            // ITD Buffer Page 0         (BP0)
  } DW1;
  volatile uint32_t NextTD;             // ITD Next TD               (NextTD)
  uint32_t BE;                          // ITD Buffer End            (BE)
  volatile uint16_t PSW0;               // ITD Offset0/PSW0          (PSW0)
  volatile uint16_t PSW1;               // ITD Offset1/PSW1          (PSW1)
  volatile uint16_t PSW2;               // ITD Offset2/PSW2          (PSW2)
  volatile uint16_t PSW3;               // ITD Offset3/PSW3          (PSW3)
  volatile uint16_t PSW4;               // ITD Offset4/PSW4          (PSW4)
  volatile uint16_t PSW5;               // ITD Offset5/PSW5          (PSW5)
  volatile uint16_t PSW6;               // ITD Offset6/PSW6          (PSW6)
  volatile uint16_t PSW7;               // ITD Offset7/PSW7          (PSW7)
} USBH_OHCI_ITD;

typedef struct {                        // HC Communication Area     (HCCA)
  uint32_t HccaInterruptTable[32];      // HCCA Interrupt ED Pointers
  volatile uint16_t HccaFrameNumber;    // HCCA Current Frame Number
  volatile uint16_t HccaPad1;           // HCCA Pad
  volatile uint32_t HccaDoneHead;       // HCCA Done Head
  volatile uint32_t Rsvd0[30];          // Reserved - unused bytes
} USBH_OHCI_HCCA;

// Constant Definitions

// HcControl Register bits
#define USBH_OHCI_HcControl_RWE          (1UL << 10)
#define USBH_OHCI_HcControl_RWC          (1UL <<  9)
#define USBH_OHCI_HcControl_IR           (1UL <<  8)
#define USBH_OHCI_HcControl_HCFS         (3UL <<  6)
#define USBH_OHCI_HcControl_BLE          (1UL <<  5)
#define USBH_OHCI_HcControl_CLE          (1UL <<  4)
#define USBH_OHCI_HcControl_IE           (1UL <<  3)
#define USBH_OHCI_HcControl_PLE          (1UL <<  2)
#define USBH_OHCI_HcControl_CBSR         (3U      )

                                         // Functional State (FS)
#define USBH_OHCI_FS_USB_RESET           (0UL)
#define USBH_OHCI_FS_USB_RESUME          (1UL <<  6)
#define USBH_OHCI_FS_USB_OPERATIONAL     (2UL <<  6)
#define USBH_OHCI_FS_USB_SUSPEND         (3UL <<  6)

// HcCommandStatus Register bits
#define USBH_OHCI_HcCommandStatus_SOC    (3UL << 16)
#define USBH_OHCI_HcCommandStatus_OCR    (1UL <<  3)
#define USBH_OHCI_HcCommandStatus_BLF    (1UL <<  2)
#define USBH_OHCI_HcCommandStatus_CLF    (1UL <<  1)
#define USBH_OHCI_HcCommandStatus_HCR    (1UL      )

// HcInterruptEnable Register bits
#define USBH_OHCI_HcInterruptEnable_MIE  (1UL << 31)

// HcInterruptStatus Register bits
#define USBH_OHCI_HcInterruptStatus_ALL  (0x4000007FU)
#define USBH_OHCI_HcInterruptStatus_OC   (1UL << 30)
#define USBH_OHCI_HcInterruptStatus_RHSC (1UL <<  6)
#define USBH_OHCI_HcInterruptStatus_FNO  (1UL <<  5)
#define USBH_OHCI_HcInterruptStatus_UE   (1UL <<  4)
#define USBH_OHCI_HcInterruptStatus_RD   (1UL <<  3)
#define USBH_OHCI_HcInterruptStatus_SF   (1UL <<  2)
#define USBH_OHCI_HcInterruptStatus_WDH  (1UL <<  1)
#define USBH_OHCI_HcInterruptStatus_SO   (1UL      )

// HcRhDescriptorA Register bits
#define USBH_OHCI_HcRhDescriptorA_POTPGT (0xFFUL << 24)
#define USBH_OHCI_HcRhDescriptorA_NOCP   (   1UL << 12)
#define USBH_OHCI_HcRhDescriptorA_OCPM   (   1UL << 11)
#define USBH_OHCI_HcRhDescriptorA_DT     (   1UL << 10)
#define USBH_OHCI_HcRhDescriptorA_NPS    (   1UL <<  9)
#define USBH_OHCI_HcRhDescriptorA_PSM    (   1UL <<  8)
#define USBH_OHCI_HcRhDescriptorA_NDP    (0xFFU)

// HcRhDescriptorB Register bits
#define USBH_OHCI_HcRhDescriptorB_PPCM   (0xFFFFUL << 16)
#define USBH_OHCI_HcRhDescriptorB_DR     (0xFFFFUL      )

// HcFmInterval Register bits
#define USBH_OHCI_HcFmInterval_FIT       (1UL << 31)
#define USBH_OHCI_HcFmInterval_FSMPS(x)  ((x) << 16)
#define USBH_OHCI_HcFmInterval_FI(x)     (x)

// HcLSThreshold Register bits
#define USBH_OHCI_HcLSThreshold_LST(x)   (x)

// HcRhStatus Register bits
#define USBH_OHCI_HcRhStatus_CRWE        (1UL << 31)
#define USBH_OHCI_HcRhStatus_OCIC        (1UL << 17)
#define USBH_OHCI_HcRhStatus_LPSC        (1UL << 16)
#define USBH_OHCI_HcRhStatus_DRWE        (1UL << 15)
#define USBH_OHCI_HcRhStatus_OCI         (1UL <<  1)
#define USBH_OHCI_HcRhStatus_LPS         (1UL      )

// HcRhPortStatus Register bits
#define USBH_OHCI_HcRhPortStatus_PRSC    (1UL << 20)
#define USBH_OHCI_HcRhPortStatus_OCIC    (1UL << 19)
#define USBH_OHCI_HcRhPortStatus_PSSC    (1UL << 18)
#define USBH_OHCI_HcRhPortStatus_PESC    (1UL << 17)
#define USBH_OHCI_HcRhPortStatus_CSC     (1UL << 16)
#define USBH_OHCI_HcRhPortStatus_LSDA    (1UL <<  9)
#define USBH_OHCI_HcRhPortStatus_PPS     (1UL <<  8)
#define USBH_OHCI_HcRhPortStatus_PRS     (1UL <<  4)
#define USBH_OHCI_HcRhPortStatus_POCI    (1UL <<  3)
#define USBH_OHCI_HcRhPortStatus_PSS     (1UL <<  2)
#define USBH_OHCI_HcRhPortStatus_PES     (1UL <<  1)
#define USBH_OHCI_HcRhPortStatus_CCS     (1UL      )

// Transfer Descriptor - Completion Code (CC)
#define USBH_OHCI_CC_NOERROR             ( 0U)
#define USBH_OHCI_CC_CRC                 ( 1U)
#define USBH_OHCI_CC_BITSTUFFING         ( 2U)
#define USBH_OHCI_CC_DATATOGGLEMISMATCH  ( 3U)
#define USBH_OHCI_CC_STALL               ( 4U)
#define USBH_OHCI_CC_DEVICENOTRESPONDING ( 5U)
#define USBH_OHCI_CC_PIDCHECKFAILURE     ( 6U)
#define USBH_OHCI_CC_UNEXPECTEDPID       ( 7U)
#define USBH_OHCI_CC_DATAOVERRUN         ( 8U)
#define USBH_OHCI_CC_DATAUNDERRUN        ( 9U)
#define USBH_OHCI_CC_BUFFEROVERRUN       (12U)
#define USBH_OHCI_CC_BUFFERUNDERRUN      (13U)
#define USBH_OHCI_CC_NOTACCESSED0        (14U)
#define USBH_OHCI_CC_NOTACCESSED         (15U)

#endif /* USBH_OHCI_REGS_H_ */
