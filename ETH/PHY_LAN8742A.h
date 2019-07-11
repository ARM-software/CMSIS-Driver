/*
 * Copyright (c) 2013-2019 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------
 *
 * $Date:        6. May 2019
 * $Revision:    V1.3
 *
 * Project:      Ethernet Physical Layer Transceiver (PHY)
 *               Definitions for LAN8742A
 * -------------------------------------------------------------------- */

#ifndef __PHY_LAN8742A_H
#define __PHY_LAN8742A_H

#include "Driver_ETH_PHY.h"

/* Basic Registers */
#define REG_BCR             0           /* Basic Control Register            */
#define REG_BSR             1           /* Basic Status Register             */

/* Extended Registers */
#define REG_PHYIDR1         2           /* PHY Identifier 1                  */
#define REG_PHYIDR2         3           /* PHY Identifier 2                  */
#define REG_ANAR            4           /* Auto-Negotiation Advertisement    */
#define REG_ANLPAR          5           /* Auto-Neg. Link Partner Ability    */
#define REG_ANER            6           /* Auto-Neg. Expansion Register      */
#define REG_ANEG_NP_TX      7           /* Auto-Neg. Next Page Tx            */
#define REG_ANEG_NP_RX      8           /* Auto-Neg. Next Page Rx            */
#define REG_MMD_ACCES_CTRL  13          /* MMD Access Control                */
#define REG_MMD_ACCES_AD    14          /* MMD Access Address/Data           */

/* Vendor-specific Registers */
#define REG_MCSR            17          /* Mode Control/Status Register      */
#define REG_SPEC_MODE       18          /* Special Modes Register            */
#define REG_TDR_PAT_DEL     24          /* TDR Patterns/Delay Control Reg.   */
#define REG_TDR_CTRL_STAT   25          /* TDR Control/Status Register       */
#define REG_SEC             26          /* System Error Counter Register     */
#define REG_SC_SI           27          /* Specifal Control/Status Indication*/
#define REG_CABLE_LEN       28          /* Cable Length Register             */
#define REG_ISF             29          /* Interrupt Source Flag Register    */
#define REG_IM              30          /* Interrupt Mask Register           */
#define REG_PSCS            31          /* PHY Special Ctrl/Status Register  */

/* Basic Control Register */
#define BCR_RESET           0x8000      /* Software Reset                    */
#define BCR_LOOPBACK        0x4000      /* Loopback mode                     */
#define BCR_SPEED_SEL       0x2000      /* Speed Select (1=100Mb/s)          */
#define BCR_ANEG_EN         0x1000      /* Auto Negotiation Enable           */
#define BCR_POWER_DOWN      0x0800      /* Power Down (1=power down mode)    */
#define BCR_ISOLATE         0x0400      /* Isolate Media interface           */
#define BCR_REST_ANEG       0x0200      /* Restart Auto Negotiation          */
#define BCR_DUPLEX          0x0100      /* Duplex Mode (1=Full duplex)       */
#define BCR_COL_TEST        0x0080      /* Enable Collision Test             */

/* Basic Status Register */
#define BSR_100B_T4         0x8000      /* 100BASE-T4 Capable                */
#define BSR_100B_TX_FD      0x4000      /* 100BASE-TX Full Duplex Capable    */
#define BSR_100B_TX_HD      0x2000      /* 100BASE-TX Half Duplex Capable    */
#define BSR_10B_T_FD        0x1000      /* 10BASE-T Full Duplex Capable      */
#define BSR_10B_T_HD        0x0800      /* 10BASE-T Half Duplex Capable      */
#define BSR_100B_T2_FD      0x0400      /* 1000BASE-T2 Full Duplex Capable   */
#define BSR_100B_T2_HD      0x0200      /* 1000BASE-T2 Half Duplex Capable   */
#define BSR_EXTENDED_STAT   0x0100      /* Extended Status in register 15    */
#define BSR_ANEG_COMPL      0x0020      /* Auto Negotiation Complete         */
#define BSR_REM_FAULT       0x0010      /* Remote Fault                      */
#define BSR_ANEG_ABIL       0x0008      /* Auto Negotiation Ability          */
#define BSR_LINK_STAT       0x0004      /* Link Status (1=link us up)        */
#define BSR_JABBER_DET      0x0002      /* Jabber Detect                     */
#define BSR_EXT_CAPAB       0x0001      /* Extended Capabilities             */

/* PHY Identifier Registers */
#define PHY_ID1             0x0007      /* LAN8742A Device Identifier MSB    */
#define PHY_ID2             0xC130      /* LAN8742A Device Identifier LSB    */

/* PHY Special Control/Status Register */
#define PSCS_AUTODONE       0x1000      /* Auto-negotiation is done          */
#define PSCS_DUPLEX         0x0010      /* Duplex Status (1=Full duplex)     */
#define PSCS_SPEED          0x0004      /* Speed10 Status (1=10MBit/s)       */

/* PHY Driver State Flags */
#define PHY_INIT              0x01U     /* Driver initialized                */
#define PHY_POWER             0x02U     /* Driver power is on                */

/* PHY Driver Control Structure */
typedef struct phy_ctrl {
  ARM_ETH_PHY_Read_t  reg_rd;           /* PHY register read function        */
  ARM_ETH_PHY_Write_t reg_wr;           /* PHY register write function       */
  uint16_t            bcr;              /* BCR register value                */
  uint8_t             flags;            /* Control flags                     */
  uint8_t             rsvd;             /* Reserved                          */
} PHY_CTRL;

#endif /* __PHY_LAN8742A_H */
