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
 * $Date:        15. May 2024
 * $Revision:    V1.0
 *
 * Project:      USB Host EHCI Controller Driver Registers header
 *               for customized EHCI with internal Transaction Translator (TT)
 *               (with full/low speed support)
 * -------------------------------------------------------------------------- */

#ifndef USBH_EHCI_TT_REGS_H_
#define USBH_EHCI_TT_REGS_H_

#include <stdint.h>

// Structure Definitions

typedef struct {                        // EHCI Registers
  volatile uint32_t CAPLENGTH;
  volatile uint32_t HCSPARAMS;
  volatile uint32_t HCCPARAMS;
  volatile uint32_t RESERVED0[5];
  volatile uint32_t DCIVERSION;
  volatile uint32_t RESERVED1[7];
  volatile uint32_t USBCMD;
  volatile uint32_t USBSTS;
  volatile uint32_t USBINTR;
  volatile uint32_t FRINDEX;
  volatile uint32_t CTRLDSSEGMENT;
  volatile uint32_t PERIODICLISTBASE;
  volatile uint32_t ASYNCLISTADDR;
  volatile uint32_t TTCTRL;
  volatile uint32_t BURSTSIZE;
  volatile uint32_t TXFILLTUNING;
  volatile uint32_t RESERVED2[6];
  volatile uint32_t CONFIGFLAG;
  volatile uint32_t PORTSC[15];
} USBH_EHCI_Registers_t;

typedef struct {                        // qH/iTD/siTD Common Link Pointer
  struct {                              // Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
    uint32_t Rsvd0              :  2;   // Reserved - unused bits
    uint32_t LinkPtr            : 27;   // Link Pointer
  } DW0;
} USBH_EHCI_COMMON;

typedef struct {                        // Isochronous Transfer Descriptor (iTD)
  struct {                              // iTD Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
    uint32_t Rsvd0              :  2;   // Reserved - unused bits
    uint32_t NextLinkPtr        : 27;   // Next Link Pointer
  } DW0;
  struct {                              // iTD Double Word 1 (32 bits)
    uint32_t Offset             : 12;   // Transaction 0 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 0 Length
    volatile uint32_t Status    :  4;   // Status
  } DW1;
  struct {                              // iTD Double Word 2 (32 bits)
    uint32_t Offset             : 12;   // Transaction 1 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 1 Length
    volatile uint32_t Status    :  4;   // Status
  } DW2;
  struct {                              // iTD Double Word 3 (32 bits)
    uint32_t Offset             : 12;   // Transaction 2 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 2 Length
    volatile uint32_t Status    :  4;   // Status
  } DW3;
  struct {                              // iTD Double Word 4 (32 bits)
    uint32_t Offset             : 12;   // Transaction 3 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 3 Length
    volatile uint32_t Status    :  4;   // Status
  } DW4;
  struct {                              // iTD Double Word 5 (32 bits)
    uint32_t Offset             : 12;   // Transaction 4 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 4 Length
    volatile uint32_t Status    :  4;   // Status
  } DW5;
  struct {                              // iTD Double Word 6 (32 bits)
    uint32_t Offset             : 12;   // Transaction 5 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 5 Length
    volatile uint32_t Status    :  4;   // Status
  } DW6;
  struct {                              // iTD Double Word 7 (32 bits)
    uint32_t Offset             : 12;   // Transaction 6 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 6 Length
    volatile uint32_t Status    :  4;   // Status
  } DW7;
  struct {                              // iTD Double Word 8 (32 bits)
    uint32_t Offset             : 12;   // Transaction 7 Offset
    uint32_t PG                 :  3;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t Length    : 12;   // Transaction 7 Length
    volatile uint32_t Status    :  4;   // Status
  } DW8;
  struct {                              // iTD Double Word 9 (32 bits)
    uint32_t DevAddr            :  7;   // Device Address
    uint32_t Rsvd0              :  1;   // Reserved
    uint32_t EndPt              :  4;   // Endpoint Number
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
  } DW9;
  struct {                              // iTD Double Word 10 (32 bits)
    uint32_t MaxPcktSz          : 11;   // Maximum Packet Size
    uint32_t IO                 :  1;   // Direction (0 = OUT, 1 = IN)
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
  } DW10;
  struct {                              // iTD Double Word 11 (32 bits)
    uint32_t Mult               :  2;   // Num of transactions per microframe
    uint32_t Rsvd0              : 10;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
  } DW11;
  struct {                              // iTD Double Word 12 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
  } DW12;
  struct {                              // iTD Double Word 13 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
  } DW13;
  struct {                              // iTD Double Word 14 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 5)
  } DW14;
  struct {                              // iTD Double Word 15 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 6)
  } DW15;
} USBH_EHCI_iTD;

typedef struct {                        // Split Transaction Isochronous Transfer Descriptor (siTD)
  struct {                              // siTD Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
    uint32_t Rsvd0              :  2;   // Reserved - unused bits
    uint32_t NextLinkPtr        : 27;   // Next Link Pointer
  } DW0;
  struct {                              // siTD Double Word 1 (32 bits)
    uint32_t DevAddr            :  7;   // Device Address
    uint32_t Rsvd0              :  1;   // Reserved
    uint32_t EndPt              :  4;   // Endpoint Number
    uint32_t Rsvd1              :  4;   // Reserved
    uint32_t HubAddr            :  7;   // HUB Address
    uint32_t Rsvd2              :  1;   // Reserved
    uint32_t PortNum            :  7;   // Port Number
    uint32_t IO                 :  1;   // Direction (0 = OUT, 1 = IN)
  } DW1;
  struct {                              // siTD Double Word 2 (32 bits)
    uint32_t SSM                :  8;   // Split Start Mask (uFrame S-mask)
    uint32_t SCM                :  8;   // Split Completion Mask (uFrame C-Mask)
    uint32_t Rsvd0              : 16;   // Reserved
  } DW2;
  struct {                              // siTD Double Word 3 (32 bits)
    volatile uint32_t Status    :  8;   // Status
    volatile uint32_t CPM       :  8;   // uFrame Complete-split Progress Mask (C-prog-Mask)
    volatile uint32_t TBT       : 10;   // Total Bytes to Transfer
    volatile uint32_t Rsvd0     :  4;   // Reserved
    volatile uint32_t P         :  1;   // Page Select
    uint32_t IOC                :  1;   // Interrupt On Complete
  } DW3;
  struct {                              // siTD Double Word 4 (32 bits)
    volatile uint32_t CurOfs    : 12;   // Current Offset
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
  } DW4;
  struct {                              // siTD Double Word 5 (32 bits)
    volatile uint32_t TC        :  3;   // Transaction Count
    volatile uint32_t TP        :  2;   // Transaction Position
    uint32_t Rsvd0              :  7;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
  } DW5;
  struct {                              // siTD Double Word 6 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Rsvd0              :  4;   // Reserved
    uint32_t BackPtr            : 27;   // siTD Back Pointer
  } DW6;
  uint32_t RsvdDW[1];                   // Reserved to align qH to 32 bytes
} USBH_EHCI_siTD;

typedef struct {                        // Split Transaction Isochronous Transfer Descriptor (siTD)
  uint32_t DW[8];                       // structure allowing direct Double Word access
} USBH_EHCI_siTD_DW;

typedef struct {                        // Queue Element Transfer Desc (qTD)
  struct {                              // qTD Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Rsvd0              :  4;   // Reserved - unused bits
    uint32_t NextPtr            : 27;   // Next Transfer Element Pointer
  } DW0;
  struct {                              // qTD Double Word 1 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Rsvd0              :  4;   // Reserved - unused bits
    uint32_t AltNextPtr         : 27;   // Alternate Next Transfer Element Ptr
  } DW1;
  struct {                              // qTD Double Word 2 (32 bits)
    volatile uint32_t Status    :  8;   // Status
    uint32_t PID                :  2;   // PID Code
    volatile uint32_t CERR      :  2;   // Error Counter
    volatile uint32_t C_Page    :  3;   // Current Page
    uint32_t IOC                :  1;   // Interrupt On Complete
    volatile uint32_t TBT       : 15;   // Total Bytes to Transfer
    volatile uint32_t DT        :  1;   // Data Toggle
  } DW2;
  struct {                              // qTD Double Word 3 (32 bits)
    volatile uint32_t CurOfs    : 12;   // Current Offset
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
  } DW3;
  struct {                              // qTD Double Word 4 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
  } DW4;
  struct {                              // qTD Double Word 5 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
  } DW5;
  struct {                              // qTD Double Word 6 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
  } DW6;
  struct {                              // qTD Double Word 7 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
  } DW7;
} USBH_EHCI_qTD;

typedef struct {                        // Queue Head (qH)
  struct {                              // qH Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // QH, iTD, siTD or FSTN Item Type
    uint32_t Rsvd0              :  2;   // Reserved - unused bits
    uint32_t HorizLinkPtr       : 27;   // Queue Head Horizontal Link Pointer
  } DW0;
  struct {                              // qH Double Word 1 (32 bits)
    uint32_t DevAddr            :  7;   // Device Address
    uint32_t I                  :  1;   // Inactive on Next Transaction
    uint32_t EndPt              :  4;   // Endpoint Number
    uint32_t EPS                :  2;   // Endpoint Speed
    uint32_t DTC                :  1;   // Data Toggle Control
    uint32_t H                  :  1;   // Head of Reclamation List Flag
    uint32_t MaxPcktLen         : 11;   // Maximum Packet Length
    uint32_t C                  :  1;   // Control Endpoint Flag
    uint32_t RL                 :  4;   // NAK Count Reload
  } DW1;
  struct {                              // qH Double Word 2 (32 bits)
    uint32_t ISM                :  8;   // Interrupt Schedule Mask (uFrame S-mask)
    uint32_t SCM                :  8;   // Split Completion Mask (uFrame C-Mask)
    uint32_t HubAddr            :  7;   // HUB address
    uint32_t PortNum            :  7;   // Port Number
    uint32_t Mult               :  2;   // High-Bandwidth Pipe Multiplier
  } DW2;
  volatile struct {                     // qH Double Word 3 (32 bits)
    uint32_t Rsvd0              :  5;   // Reserved
    uint32_t CurrPtr            : 27;   // Current Element Transaction Descriptor Link Pointer
  } DW3;
  volatile struct {                     // qH Double Word 4 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Rsvd0              :  4;   // Reserved
    uint32_t NextPtr            : 27;   // Next Element Transaction Descriptor Link Pointer
  } DW4;
  volatile struct {                     // qH Double Word 5 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t NakCnt             :  4;   // NAK Counter
    uint32_t AltNextPtr         : 27;   // Alternate Next Element Transaction Descriptor Link Pointer
  } DW5;
  volatile struct {                     // qH Double Word 6 (32 bits)
    uint32_t Status             :  8;   // Status
    uint32_t PID                :  2;   // PID Code
    uint32_t CERR               :  2;   // Error Counter
    uint32_t C_Page             :  3;   // Current Page
    uint32_t IOC                :  1;   // Interrupt On Complete
    uint32_t TBT                : 15;   // Total Bytes to Transfer
    uint32_t DT                 :  1;   // Data Toggle
  } DW6;
  volatile struct {                     // qH Double Word 7 (32 bits)
    uint32_t CurOfs             : 12;   // Current Offset
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 0)
  } DW7;
  volatile struct {                     // qH Double Word 8 (32 bits)
    uint32_t CPM                :  8;   // Split-transaction Complete-split Progress (C-prog-mask)
    uint32_t Rsvd0              :  4;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 1)
  } DW8;
  volatile struct {                     // qH Double Word 9 (32 bits)
    uint32_t FrameTag           :  5;   // Split-transaction Frame Tag (Frame Tag)
    uint32_t SBytes             :  7;   // S-bytes
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 2)
  } DW9;
  volatile struct {                     // qH Double Word 10 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 3)
  } DW10;
  volatile struct {                     // qH Double Word 11 (32 bits)
    uint32_t Rsvd0              : 12;   // Reserved
    uint32_t BufPtr             : 20;   // Buffer Pointer (Page 4)
  } DW11;
  uint32_t RsvdDW[4];                   // Reserved to align qH to 64 bytes
} USBH_EHCI_qH;

typedef struct {                        // Periodic Frame Span Traversal Node (FSTN)
  struct {                              // FSTN Double Word 0 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // qH, iTD, siTD or FSTN Item Type
    uint32_t NPLP               : 29;   // Normal Path Link Pointer (NPLP)
  } DW0;
  struct {                              // FSTN Double Word 1 (32 bits)
    uint32_t T                  :  1;   // Terminate
    uint32_t Typ                :  2;   // Must be qH Type
    uint32_t BackPathLinkPtr    : 29;   // Back Path Link Pointer
  } DW1;
} USBH_EHCI_FSTN;

// Constant Definitions

// Capability Registers
#define oCAPLENGTH                      (       0x00U)
#define oHCIVERSION                     (       0x02U)
#define oHCSPARAMS                      (       0x04U)
#define oHCCPARAMS                      (       0x08U)
#define oHCSP_PORTROUTE                 (       0x0CU)

// Operational Registers
#define oUSBCMD                         (       0x40U)
#define oUSBSTS                         (       0x44U)
#define oUSBINTR                        (       0x48U)
#define oFRINDEX                        (       0x4CU)
#define oCTRLDSSEGMENT                  (       0x50U)
#define oPERIODICLISTBASE               (       0x54U)
#define oASYNCLISTADDR                  (       0x58U)
#define oTTCTRL                         (       0x5CU)
#define oBURSTSIZE                      (       0x60U)
#define oTXFILLTUNING                   (       0x64U)
#define oCONFIGFLAG                     (       0x80U)
#define oPORTSC1                        (4U* 0U+0x84U)
#define oPORTSC2                        (4U* 1U+0x84U)
#define oPORTSC3                        (4U* 2U+0x84U)
#define oPORTSC4                        (4U* 3U+0x84U)
#define oPORTSC5                        (4U* 4U+0x84U)
#define oPORTSC6                        (4U* 5U+0x84U)
#define oPORTSC7                        (4U* 6U+0x84U)
#define oPORTSC8                        (4U* 7U+0x84U)
#define oPORTSC9                        (4U* 8U+0x84U)
#define oPORTSC10                       (4U* 9U+0x84U)
#define oPORTSC11                       (4U*10U+0x84U)
#define oPORTSC12                       (4U*11U+0x84U)
#define oPORTSC13                       (4U*12U+0x84U)
#define oPORTSC14                       (4U*13U+0x84U)
#define oPORTSC15                       (4U*14U+0x84U)
#define oUSBMODE                        (       0xA8U)

// Constant Definitions

#define USBH_EHCI_ASYNC_SCHED           (        0U)
#define USBH_EHCI_PERIODIC_SCHED        (        1U)

// HCSPARAMS Register bits
#define USBH_EHCI_HCSPARAMS_N_TT(x)     (((x##U)&0x0FUL) << 24)
#define USBH_EHCI_HCSPARAMS_N_TT_MSK    (        0x0FUL  << 24)
#define USBH_EHCI_HCSPARAMS_N_PTT(x)    (((x##U)&0x0FUL) << 20)
#define USBH_EHCI_HCSPARAMS_N_PTT_MSK   (        0x0FUL  << 20)
#define USBH_EHCI_HCSPARAMS_DPN(x)      (((x##U)&0x0FUL) << 20)
#define USBH_EHCI_HCSPARAMS_DPN_MSK     (        0x0FUL  << 20)
#define USBH_EHCI_HCSPARAMS_PI(x)       (((x##U)&0x01UL) << 16)
#define USBH_EHCI_HCSPARAMS_PI_MSK      (        0x01UL  << 16)
#define USBH_EHCI_HCSPARAMS_N_CC(x)     (((x##U)&0x0FUL) << 12)
#define USBH_EHCI_HCSPARAMS_N_CC_MSK    (        0x0FUL  << 12)
#define USBH_EHCI_HCSPARAMS_N_PCC(x)    (((x##U)&0x0FUL) <<  8)
#define USBH_EHCI_HCSPARAMS_N_PCC_MSK   (        0x0FUL  <<  8)
#define USBH_EHCI_HCSPARAMS_PRR(x)      (((x##U)&0x01UL) <<  7)
#define USBH_EHCI_HCSPARAMS_PRR_MSK     (        0x01UL  <<  7)
#define USBH_EHCI_HCSPARAMS_PPC(x)      (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_HCSPARAMS_PPC_MSK     (        0x01UL  <<  4)
#define USBH_EHCI_HCSPARAMS_N_PORTS(x)  (((x##U)&0x0FUL)      )
#define USBH_EHCI_HCSPARAMS_N_PORTS_MSK (        0x0FUL       )

// HCCPARAMS Register bits
#define USBH_EHCI_HCCPARAMS_EECP(x)     (((x##U)&0xFFUL) <<  8)
#define USBH_EHCI_HCCPARAMS_EECP_MSK    (        0xFFUL  <<  8)
#define USBH_EHCI_HCCPARAMS_IST(x)      (((x##U)&0x0FUL) <<  4)
#define USBH_EHCI_HCCPARAMS_IST_MSK     (        0x0FUL  <<  4)
#define USBH_EHCI_HCCPARAMS_ASP(x)      (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_HCCPARAMS_ASP_MSK     (        0x01UL  <<  2)
#define USBH_EHCI_HCCPARAMS_PFL(x)      (((x##U)&0x01UL) <<  1)
#define USBH_EHCI_HCCPARAMS_PFL_MSK     (        0x01UL  <<  1)
#define USBH_EHCI_HCCPARAMS_ADC(x)      (((x##U)&0x01UL)      )
#define USBH_EHCI_HCCPARAMS_ADC_MSK     (        0x01UL       )

// USBCMD Register bits
#define USBH_EHCI_USBCMD_ITC(x)         (((x##U)&0xFFUL) << 16)
#define USBH_EHCI_USBCMD_ITC_MSK        (        0xFFUL  << 16)
#define USBH_EHCI_USBCMD_FS2(x)         (((x##U)&0x01UL) << 15)
#define USBH_EHCI_USBCMD_FS2_MSK        (        0x01UL  << 15)
#define USBH_EHCI_USBCMD_ASPE(x)        (((x##U)&0x01UL) << 11)
#define USBH_EHCI_USBCMD_ASPE_MSK       (        0x01UL  << 11)
#define USBH_EHCI_USBCMD_ASP1_0(x)      (((x##U)&0x03UL) <<  8)
#define USBH_EHCI_USBCMD_ASP1_0_MSK     (        0x03UL  <<  8)
#define USBH_EHCI_USBCMD_LHCR(x)        (((x##U)&0x01UL) <<  7)
#define USBH_EHCI_USBCMD_LHCR_MSK       (        0x01UL  <<  7)
#define USBH_EHCI_USBCMD_IAA(x)         (((x##U)&0x01UL) <<  6)
#define USBH_EHCI_USBCMD_IAA_MSK        (        0x01UL  <<  6)
#define USBH_EHCI_USBCMD_ASE(x)         (((x##U)&0x01UL) <<  5)
#define USBH_EHCI_USBCMD_ASE_MSK        (        0x01UL  <<  5)
#define USBH_EHCI_USBCMD_PSE(x)         (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_USBCMD_PSE_MSK        (        0x01UL  <<  4)
#define USBH_EHCI_USBCMD_FS1(x)         (((x##U)&0x01UL) <<  3)
#define USBH_EHCI_USBCMD_FS1_MSK        (        0x01UL  <<  3)
#define USBH_EHCI_USBCMD_FS0(x)         (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_USBCMD_FS0_MSK        (        0x01UL  <<  2)
#define USBH_EHCI_USBCMD_FS(x)          (((x##U)&0x03UL) <<  2)
#define USBH_EHCI_USBCMD_FS_MSK         (        0x03UL  <<  2)
#define USBH_EHCI_USBCMD_RST(x)         (((x##U)&0x01UL) <<  1)
#define USBH_EHCI_USBCMD_RST_MSK        (        0x01UL  <<  1)
#define USBH_EHCI_USBCMD_RS(x)          (((x##U)&0x01UL)      )
#define USBH_EHCI_USBCMD_RS_MSK         (        0x01UL       )

// USBSTS Register bits
#define USBH_EHCI_USBSTS_UPI(x)         (((x##U)&0x01UL) << 19)
#define USBH_EHCI_USBSTS_UPI_MSK        (        0x01UL  << 19)
#define USBH_EHCI_USBSTS_UAI(x)         (((x##U)&0x01UL) << 18)
#define USBH_EHCI_USBSTS_UAI_MSK        (        0x01UL  << 18)
#define USBH_EHCI_USBSTS_AS(x)          (((x##U)&0x01UL) << 15)
#define USBH_EHCI_USBSTS_AS_MSK         (        0x01UL  << 15)
#define USBH_EHCI_USBSTS_PS(x)          (((x##U)&0x01UL) << 14)
#define USBH_EHCI_USBSTS_PS_MSK         (        0x01UL  << 14)
#define USBH_EHCI_USBSTS_RCL(x)         (((x##U)&0x01UL) << 13)
#define USBH_EHCI_USBSTS_RCL_MSK        (        0x01UL  << 13)
#define USBH_EHCI_USBSTS_HCH(x)         (((x##U)&0x01UL) << 12)
#define USBH_EHCI_USBSTS_HCH_MSK        (        0x01UL  << 12)
#define USBH_EHCI_USBSTS_SRI(x)         (((x##U)&0x01UL) <<  7)
#define USBH_EHCI_USBSTS_SRI_MSK        (        0x01UL  <<  7)
#define USBH_EHCI_USBSTS_AAI(x)         (((x##U)&0x01UL) <<  5)
#define USBH_EHCI_USBSTS_AAI_MSK        (        0x01UL  <<  5)
#define USBH_EHCI_USBSTS_HSE(x)         (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_USBSTS_HSE_MSK        (        0x01UL  <<  4)
#define USBH_EHCI_USBSTS_FRI(x)         (((x##U)&0x01UL) <<  3)
#define USBH_EHCI_USBSTS_FRI_MSK        (        0x01UL  <<  3)
#define USBH_EHCI_USBSTS_PCI(x)         (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_USBSTS_PCI_MSK        (        0x01UL  <<  2)
#define USBH_EHCI_USBSTS_UEI(x)         (((x##U)&0x01UL) <<  1)
#define USBH_EHCI_USBSTS_UEI_MSK        (        0x01UL  <<  1)
#define USBH_EHCI_USBSTS_UI(x)          (((x##U)&0x01UL)      )
#define USBH_EHCI_USBSTS_UI_MSK         (        0x01UL       )

// USBINTR Register bits
#define USBH_EHCI_USBINTR_UPIA(x)       (((x##U)&0x01UL) << 19)
#define USBH_EHCI_USBINTR_UPIA_MSK      (        0x01UL  << 19)
#define USBH_EHCI_USBINTR_UAIE(x)       (((x##U)&0x01UL) << 18)
#define USBH_EHCI_USBINTR_UAIE_MSK      (        0x01UL  << 18)
#define USBH_EHCI_USBINTR_SRE(x)        (((x##U)&0x01UL) <<  7)
#define USBH_EHCI_USBINTR_SRE_MSK       (        0x01UL  <<  7)
#define USBH_EHCI_USBINTR_AAE(x)        (((x##U)&0x01UL) <<  5)
#define USBH_EHCI_USBINTR_AAE_MSK       (        0x01UL  <<  5)
#define USBH_EHCI_USBINTR_HSE(x)        (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_USBINTR_HSE_MSK       (        0x01UL  <<  4)
#define USBH_EHCI_USBINTR_FRE(x)        (((x##U)&0x01UL) <<  3)
#define USBH_EHCI_USBINTR_FRE_MSK       (        0x01UL  <<  3)
#define USBH_EHCI_USBINTR_PCE(x)        (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_USBINTR_PCE_MSK       (        0x01UL  <<  2)
#define USBH_EHCI_USBINTR_UEE(x)        (((x##U)&0x01UL) <<  1)
#define USBH_EHCI_USBINTR_UEE_MSK       (        0x01UL  <<  1)
#define USBH_EHCI_USBINTR_UE(x)         (((x##U)&0x01UL)      )
#define USBH_EHCI_USBINTR_UE_MSK        (        0x01UL       )

// FRINDEX Register bits
#define USBH_EHCI_FRINDEX(x)            ((x##U)&0x00001FFFU)
#define USBH_EHCI_FRINDEX_MSK           (       0x00001FFFU)

// PERIODICLISTBASE Register bits
#define USBH_EHCI_PERIODICLISTBASE(x)   ((x##U)&0xFFFFF000U)
#define USBH_EHCI_PERIODICLISTBASE_MSK  (       0xFFFFF000U)

// ASYNCLISTBASE Register bits
#define USBH_EHCI_ASYNCLISTBASE(x)      ((x##U)&0xFFFFFFE0U)
#define USBH_EHCI_ASYNCLISTBASE_MSK     (       0xFFFFFFE0U)

// TTCTRL Register bits
#define USBH_EHCI_TTCTRL(x)             (((x##U)&0x3FUL) << 24)
#define USBH_EHCI_TTCTRL_MSK            (        0x3FUL  << 24)

// CONFIGFLAG Register bits
#define USBH_EHCI_CONFIGFLAG_CF(x)      (((x##U)&0x01UL)      )
#define USBH_EHCI_CONFIGFLAG_CF_MSK     (        0x01UL       )

// TXFILLTUNING Register bits
#define USBH_EHCI_TXFILLTUNING_TXFIFOTHRES(x)  (((x##U)&0x3FUL) << 16)
#define USBH_EHCI_TXFILLTUNING_TXFIFOTHRES_MSK (        0x3FUL  << 16)
#define USBH_EHCI_TXFILLTUNING_TXSCHEALTH(x)   (((x##U)&0x1FUL) <<  8)
#define USBH_EHCI_TXFILLTUNING_TXSCHEALTH_MSK  (        0x1FUL  <<  8)
#define USBH_EHCI_TXFILLTUNING_TXSCHOH(x)      (((x##U)&0xFFUL)      )
#define USBH_EHCI_TXFILLTUNING_TXSCHOH_MSK     (        0xFFUL       )

// PORTSCx Register bits
#define USBH_EHCI_PORTSC_PSPD(x)        (((x##U)&0x03UL) << 26)
#define USBH_EHCI_PORTSC_PSPD_MSK       (        0x03UL  << 26)
#define USBH_EHCI_PORTSC_PFSC(x)        (((x##U)&0x01UL) << 24)
#define USBH_EHCI_PORTSC_PFSC_MSK       (        0x01UL  << 24)
#define USBH_EHCI_PORTSC_PHCD(x)        (((x##U)&0x01UL) << 23)
#define USBH_EHCI_PORTSC_PHCD_MSK       (        0x01UL  << 23)
#define USBH_EHCI_PORTSC_WKOC(x)        (((x##U)&0x01UL) << 22)
#define USBH_EHCI_PORTSC_WKOC_MSK       (        0x01UL  << 22)
#define USBH_EHCI_PORTSC_WKDC(x)        (((x##U)&0x01UL) << 21)
#define USBH_EHCI_PORTSC_WKDC_MSK       (        0x01UL  << 21)
#define USBH_EHCI_PORTSC_WKCN(x)        (((x##U)&0x01UL) << 20)
#define USBH_EHCI_PORTSC_WKCN_MSK       (        0x01UL  << 20)
#define USBH_EHCI_PORTSC_PTC(x)         (((x##U)&0x0FUL) << 16)
#define USBH_EHCI_PORTSC_PTC_MSK        (        0x0FUL  << 16)
#define USBH_EHCI_PORTSC_PIC(x)         (((x##U)&0x03UL) << 14)
#define USBH_EHCI_PORTSC_PIC_MSK        (        0x03UL  << 14)
#define USBH_EHCI_PORTSC_PO(x)          (((x##U)&0x01UL) << 13)
#define USBH_EHCI_PORTSC_PO_MSK         (        0x01UL  << 13)
#define USBH_EHCI_PORTSC_PP(x)          (((x##U)&0x01UL) << 12)
#define USBH_EHCI_PORTSC_PP_MSK         (        0x01UL  << 12)
#define USBH_EHCI_PORTSC_LS(x)          (((x##U)&0x03UL) << 10)
#define USBH_EHCI_PORTSC_LS_MSK         (        0x03UL  << 10)
#define USBH_EHCI_PORTSC_PR(x)          (((x##U)&0x01UL) <<  8)
#define USBH_EHCI_PORTSC_PR_MSK         (        0x01UL  <<  8)
#define USBH_EHCI_PORTSC_SUSP(x)        (((x##U)&0x01UL) <<  7)
#define USBH_EHCI_PORTSC_SUSP_MSK       (        0x01UL  <<  7)
#define USBH_EHCI_PORTSC_FPR(x)         (((x##U)&0x01UL) <<  6)
#define USBH_EHCI_PORTSC_FPR_MSK        (        0x01UL  <<  6)
#define USBH_EHCI_PORTSC_OCC(x)         (((x##U)&0x01UL) <<  5)
#define USBH_EHCI_PORTSC_OCC_MSK        (        0x01UL  <<  5)
#define USBH_EHCI_PORTSC_OCA(x)         (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_PORTSC_OCA_MSK        (        0x01UL  <<  4)
#define USBH_EHCI_PORTSC_PEC(x)         (((x##U)&0x01UL) <<  3)
#define USBH_EHCI_PORTSC_PEC_MSK        (        0x01UL  <<  3)
#define USBH_EHCI_PORTSC_PE(x)          (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_PORTSC_PE_MSK         (        0x01UL  <<  2)
#define USBH_EHCI_PORTSC_CSC(x)         (((x##U)&0x01UL) <<  1)
#define USBH_EHCI_PORTSC_CSC_MSK        (        0x01UL  <<  1)
#define USBH_EHCI_PORTSC_CCS(x)         (((x##U)&0x01UL)      )
#define USBH_EHCI_PORTSC_CCS_MSK        (        0x01UL       )

// USBMODE Register bits
#define USBH_EHCI_USBMODE_VBPS(x)       (((x##U)&0x01UL) <<  5)
#define USBH_EHCI_USBMODE_VBPS_MSK      (        0x01UL  <<  5)
#define USBH_EHCI_USBMODE_SDIS(x)       (((x##U)&0x01UL) <<  4)
#define USBH_EHCI_USBMODE_SDIS_MSK      (        0x01UL  <<  4)
#define USBH_EHCI_USBMODE_ES(x)         (((x##U)&0x01UL) <<  2)
#define USBH_EHCI_USBMODE_ES_MSK        (        0x01UL  <<  2)
#define USBH_EHCI_USBMODE_CM(x)         (((x##U)&0x03UL)      )
#define USBH_EHCI_USBMODE_CM_MSK        (        0x03UL       )

#define USBH_EHCI_ELEMENT_TYPE_iTD      (0U)
#define USBH_EHCI_ELEMENT_TYPE_qH       (1U)
#define USBH_EHCI_ELEMENT_TYPE_siTD     (2U)
#define USBH_EHCI_ELEMENT_TYPE_FSTN     (3U)

#endif /* USBH_EHCI_TT_REGS_H_ */
