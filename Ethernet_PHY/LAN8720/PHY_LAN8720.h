/*
 * Copyright (c) 2013-2018 Arm Limited. All rights reserved.
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
 * $Date:        25. May 2018
 * $Revision:    V6.2
 *
 * Project:      Ethernet Physical Layer Transceiver (PHY)
 *               Definitions for LAN8720
 * -------------------------------------------------------------------- */

#ifndef __PHY_LAN8720_H
#define __PHY_LAN8720_H

#include "Driver_ETH_PHY.h"

/* Basic Registers */
#define REG_BMCR            0x00        /* Basic Mode Control Register       */
#define REG_BMSR            0x01        /* Basic Mode Status Register        */
#define REG_PHYIDR1         0x02        /* PHY Identifier 1                  */
#define REG_PHYIDR2         0x03        /* PHY Identifier 2                  */
#define REG_ANAR            0x04        /* Auto-Negotiation Advertisement    */
#define REG_ANLPAR          0x05        /* Auto-Neg. Link Partner Ability    */
#define REG_ANER            0x06        /* Auto-Neg. Expansion Register      */

/* LAN8720 PHY Extended Registers */
#define REG_MCSR            0x11        /* Mode Control/Status Register      */
#define REG_SR              0x12        /* Special Modes Register            */
#define REG_SECR            0x1A        /* Syytem Error Counter Register     */
#define REG_CSIR            0x1B        /* Control/Status Indication Register*/
#define REG_ISR             0x1D        /* Interrupt Source Register         */
#define REG_IMR             0x1E        /* Interrupt Mask Register           */
#define REG_PSCSR           0x1F        /* PHY Special Ctrl/Status Register  */

/* Basic Mode Control Register */
#define BMCR_RESET          0x8000      /* Software Reset                    */
#define BMCR_LOOPBACK       0x4000      /* Loopback mode                     */
#define BMCR_SPEED_SEL      0x2000      /* Speed Select (1=100Mb/s)          */
#define BMCR_ANEG_EN        0x1000      /* Auto Negotiation Enable           */
#define BMCR_POWER_DOWN     0x0800      /* LAN8700 Power Down                */
#define BMCR_ISOLATE        0x0400      /* Isolate Media interface           */
#define BMCR_REST_ANEG      0x0200      /* Restart Auto Negotiation          */
#define BMCR_DUPLEX         0x0100      /* Duplex Mode (1=Full duplex)       */
#define BMCR_COL_TEST       0x0080      /* Enable Collision Test             */

/* Basic Status Register */
#define BMSR_100B_T4        0x8000      /* 100BASE-T4 Capable                */
#define BMSR_100B_TX_FD     0x4000      /* 100BASE-TX Full Duplex Capable    */
#define BMSR_100B_TX_HD     0x2000      /* 100BASE-TX Half Duplex Capable    */
#define BMSR_10B_T_FD       0x1000      /* 10BASE-T Full Duplex Capable      */
#define BMSR_10B_T_HD       0x0800      /* 10BASE-T Half Duplex Capable      */
#define BMSR_ANEG_COMPL     0x0020      /* Auto Negotiation Complete         */
#define BMSR_REM_FAULT      0x0010      /* Remote Fault                      */
#define BMSR_ANEG_ABIL      0x0008      /* Auto Negotiation Ability          */
#define BMSR_LINK_STAT      0x0004      /* Link Status (1=established)       */
#define BMSR_JABBER_DET     0x0002      /* Jaber Detect                      */
#define BMSR_EXT_CAPAB      0x0001      /* Extended Capability               */

/* PHY Identifier Registers */
#define PHY_ID1             0x0007      /* LAN8720 Device Identifier MSB    */
#define PHY_ID2             0xC0F0      /* LAN8720 Device Identifier LSB    */

/* PHY Special Control/Status Register */
#define PSCSR_AUTODONE      0x1000      /* Auto-negotiation is done          */
#define PSCSR_DUPLEX        0x0010      /* Duplex Status (1=Full duplex)     */
#define PSCSR_SPEED         0x0004      /* Speed10 Status (1=10MBit/s)       */

/* PHY Driver State Flags */
#define PHY_INIT            0x01U       /* Driver initialized                */
#define PHY_POWER           0x02U       /* Driver power is on                */

/* PHY Driver Control Structure */
typedef struct phy_ctrl {
  ARM_ETH_PHY_Read_t  reg_rd;           /* PHY register read function        */
  ARM_ETH_PHY_Write_t reg_wr;           /* PHY register write function       */
  uint16_t            bmcr;             /* BMCR register value               */
  uint8_t             flags;            /* Control flags                     */
  uint8_t             rsvd;             /* Reserved                          */
} PHY_CTRL;

#endif /* __PHY_LAN8720_H */
