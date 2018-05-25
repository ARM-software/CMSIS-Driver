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
 *               Definitions for DP83848C
 * -------------------------------------------------------------------- */

#ifndef __PHY_DP83848C_H
#define __PHY_DP83848C_H

#include "Driver_ETH_PHY.h"

/* Basic Registers */
#define REG_BMCR            0x00        /* Basic Mode Control Register       */
#define REG_BMSR            0x01        /* Basic Mode Status Register        */
#define REG_PHYIDR1         0x02        /* PHY Identifier 1                  */
#define REG_PHYIDR2         0x03        /* PHY Identifier 2                  */
#define REG_ANAR            0x04        /* Auto-Negotiation Advertisement    */
#define REG_ANLPAR          0x05        /* Auto-Neg. Link Partner Abitily    */
#define REG_ANER            0x06        /* Auto-Neg. Expansion Register      */
#define REG_ANNPTR          0x07        /* Auto-Neg. Next Page TX            */

/* Extended Registers */
#define REG_PHYSTS          0x10        /* Status Register                   */
#define REG_MICR            0x11        /* MII Interrupt Control Register    */
#define REG_MISR            0x12        /* MII Interrupt Status Register     */
#define REG_FCSCR           0x14        /* False Carrier Sense Counter       */
#define REG_RECR            0x15        /* Receive Error Counter             */
#define REG_PCSR            0x16        /* PCS Sublayer Config. and Status   */
#define REG_RBR             0x17        /* RMII and Bypass Register          */
#define REG_LEDCR           0x18        /* LED Direct Control Register       */
#define REG_PHYCR           0x19        /* PHY Control Register              */
#define REG_BTSCR           0x1A        /* 10Base-T Status/Control Register  */
#define REG_CDBR1           0x1B        /* CD Test Control and BIST Extens.  */
#define REG_EDCR            0x1D        /* Energy Detect Control Register    */

/* Basic Mode Control Register */
#define BMCR_RESET          0x8000      /* Software Reset                    */
#define BMCR_LOOPBACK       0x4000      /* Loopback mode                     */
#define BMCR_SPEED_SEL      0x2000      /* Speed Select (1=100Mb/s)          */
#define BMCR_ANEG_EN        0x1000      /* Auto Negotiation Enable           */
#define BMCR_POWER_DOWN     0x0800      /* Power Down                        */
#define BMCR_ISOLATE        0x0400      /* Isolate Media interface           */
#define BMCR_REST_ANEG      0x0200      /* Restart Auto Negotiation          */
#define BMCR_DUPLEX         0x0100      /* Duplex Mode (1=Full duplex)       */
#define BMCR_COL_TEST       0x0080      /* Collision Test                    */

/* Basic Mode Status Register */
#define BMSR_100B_T4        0x8000      /* 100BASE-T4 Capable                */
#define BMSR_100B_TX_FD     0x4000      /* 100BASE-TX Full Duplex Capable    */
#define BMSR_100B_TX_HD     0x2000      /* 100BASE-TX Half Duplex Capable    */
#define BMSR_10B_T_FD       0x1000      /* 10BASE-T Full Duplex Capable      */
#define BMSR_10B_T_HD       0x0800      /* 10BASE-T Half Duplex Capable      */
#define BMSR_MF_PRE_SUP     0x0040      /* Preamble suppression Capable      */
#define BMSR_ANEG_COMPL     0x0020      /* Auto Negotiation Complete         */
#define BMSR_REM_FAULT      0x0010      /* Remote Fault                      */
#define BMSR_ANEG_ABIL      0x0008      /* Auto Negotiation Ability          */
#define BMSR_LINK_STAT      0x0004      /* Link Status (1=established)       */
#define BMSR_JABBER_DET     0x0002      /* Jaber Detect                      */
#define BMSR_EXT_CAPAB      0x0001      /* Extended Capability               */

/* PHY Identifier Registers */
#define PHY_ID1             0x2000      /* DP83848C Device Identifier MSB    */
#define PHY_ID2             0x5C90      /* DP83848C Device Identifier LSB    */

/* PHY Status Register */
#define PHYSTS_MDI_X        0x4000      /* MDI-X mode enabled by Auto-Negot. */
#define PHYSTS_REC_ERR      0x2000      /* Receive Error Latch               */
#define PHYSTS_POL_STAT     0x1000      /* Polarity Status                   */
#define PHYSTS_FC_SENSE     0x0800      /* False Carrier Sense Latch         */
#define PHYSTS_SIG_DET      0x0400      /* 100Base-TX Signal Detect          */
#define PHYSTS_DES_LOCK     0x0200      /* 100Base-TX Descrambler Lock       */
#define PHYSTS_PAGE_REC     0x0100      /* Link Code Word Page Received      */
#define PHYSTS_MII_INT      0x0080      /* MII Interrupt Pending             */
#define PHYSTS_REM_FAULT    0x0040      /* Remote Fault                      */
#define PHYSTS_JABBER_DET   0x0020      /* Jabber Detect                     */
#define PHYSTS_ANEG_COMPL   0x0010      /* Auto Negotiation Complete         */
#define PHYSTS_LOOPBACK     0x0008      /* Loopback Status                   */
#define PHYSTS_DUPLEX       0x0004      /* Duplex Status (1=Full duplex)     */
#define PHYSTS_SPEED        0x0002      /* Speed10 Status (1=10MBit/s)       */
#define PHYSTS_LINK_STAT    0x0001      /* Link Status (1=established)       */

/* MII Interrupt Control Register */
#define MICR_TINT           0x0004      /* Test Interrupt                    */
#define MICR_INTEN          0x0002      /* Interrupt Enable                  */
#define MICR_INT_OE         0x0001      /* Interrupt Output Enable           */

/* MII Interrupt Status Register */
#define MISR_ED_INT         0x4000      /* Energy Detect Interrupt           */
#define MISR_LINK_INT       0x2000      /* Link Status Change Interrupt      */
#define MISR_SPD_INT        0x1000      /* Speed Status Change Interrupt     */
#define MISR_DUP_INT        0x0800      /* Duplex Status Change Interrupt    */
#define MISR_ANC_INT        0x0400      /* Auto Negotiation Complete Interr. */
#define MISR_FHF_INT        0x0200      /* False Carrier Counter HF Interrupt*/
#define MISR_RHF_INT        0x0100      /* Receive Error Counter HF Interrupt*/
#define MISR_ED_INT_EN      0x0040      /* Endrgy Detect Int.Enable          */
#define MISR_LINK_INT_EN    0x0020      /* Link Status Change Int.Enable     */
#define MISR_SPD_INT_EN     0x0010      /* Speed Status Change Int.Enable    */
#define MISR_DUP_INT_EN     0x0008      /* Duplex Status Change Int.Enable   */
#define MISR_ANC_INT_EN     0x0004      /* Auto Negotiation Complete Int.Ena.*/
#define MISR_FHF_INT_EN     0x0002      /* False Carrier Count.HF Int.Enable */
#define MISR_RHF_INT_EN     0x0001      /* Receive Error Count.HF Int.Enable */

/* 100Mb/s PCS Configuration and Status Register */
#define PCSR_TQ_EN          0x0400      /* 100Mbs True Quiet Mode Enable     */
#define PCSR_SD_FORCE_PMA   0x0200      /* Signal Detect Force PMA           */
#define PCSR_SD_OPTION      0x0100      /* Signal Detect Option              */
#define PCSR_DESC_TIME      0x0080      /* Descrambler Timeout               */
#define PCSR_FORCE_100_OK   0x0020      /* Force 100Mb/s Good Link           */
#define PCSR_NRZI_BYPASS    0x0004      /* NRZI Bypass Enable                */

/* RMII and Bypass Register */
#define RBR_RMII_MODE       0x0020      /* Reduced MII Mode                  */
#define RBR_RMII_REV1_0     0x0010      /* Reduced MII Revision 1.0          */
#define RBR_RX_OVF_STS      0x0008      /* RX FIFO Overflow Status           */
#define RBR_RX_UNF_STS      0x0004      /* RX FIFO Underflow Status          */
#define RBR_ELAST_BUF       0x0003      /* Receive Elasticity Buffer         */

/* LED Direct Control Register */
#define LEDCR_DRV_SPDLED    0x0020      /* Drive SPDLED bit to LED_SPD output*/
#define LEDCR_DRV_LNKLED    0x0010      /* Drive LNKLED bit to LED_LNK output*/
#define LEDCR_DRV_ACTLED    0x0008      /* Drive ACTLED bit to LED_ACT output*/
#define LEDCR_SPDLED        0x0004      /* Value to force on LED_SPD output  */
#define LEDCR_LNKLED        0x0002      /* Value to force on LED_LNK output  */
#define LEDCR_ACTLED        0x0001      /* Value to force on LED_ACT output  */

/* PHY Control Register */
#define PHYCR_MDIX_EN       0x8000      /* Auto MDIX Enable                  */
#define PHYCR_FORCE_MDIX    0x4000      /* Force MDIX                        */
#define PHYCR_PAUSE_RX      0x2000      /* Pause Receive Negotiated          */
#define PHYCR_PAUSE_TX      0x1000      /* Pause Transmit Negotiated         */
#define PHYCR_BIST_FE       0x0800      /* BIST Force Error                  */
#define PHYCR_PSR_15        0x0400      /* BIST Sequence select              */
#define PHYCR_BIST_STATUS   0x0200      /* BIST Test Status                  */
#define PHYCR_BIST_START    0x0100      /* BIST Start                        */
#define PHYCR_BP_STRETCH    0x0080      /* Bypass LED Stretching             */
#define PHYCR_LED_CNFG      0x0060      /* LEDs Configuration                */
#define PHYCR_PHYADDR       0x001F      /* PHY Address for port              */

/* 10Base-T Status/Control Register */
#define BTSCR_10BT_SER      0x8000      /* 10Base-T Serial Mode              */
#define BTSCR_SQUELCH       0x0E00      /* Squelch Configuration             */
#define BTSCR_LOOPB10_DIS   0x0100      /* Loopback 10Base-T Disable         */
#define BTSCR_LP_DIS        0x0080      /* Normal Link Pulse Disable         */
#define BTSCR_FORCE_LNK10   0x0040      /* Force 10Mbs Good Link             */
#define BTSCR_POLARITY      0x0010      /* 10Mbs Polarity Status             */
#define BTSCR_HEARTB_DIS    0x0002      /* Heartbeat Disable                 */
#define BTSCR_JABBER_DIS    0x0001      /* Jabber Disable                    */

/* CD Test and BIST Extensions Register */
#define CDBR1_BIST_ERR_CNTR 0xFF00      /* BIST ERROR Counter                */
#define CDBR1_BIST_CONT_MD  0x0020      /* Packet BIST Continuous Mode       */
#define CDBR1_CDPATEN10     0x0010      /* CD Pattern Enable for 10Mbs       */
#define CDBR1_10MEG_PAT_GAP 0x0004      /* Defines gap between data or NLP   */
#define CDBR1_CDPATTSEL     0x0003      /* CD Pattern Select                 */

/* Energy Detect Control */
#define EDCR_ED_EN          0x8000      /* Energy Detect Enable              */
#define EDCR_ED_AUTO_UP     0x4000      /* Energy Detect Automatic Power Up  */
#define EDCR_ED_AUTO_DOWN   0x2000      /* Energy Detect Automatic Power Down*/
#define EDCR_ED_MAN         0x1000      /* Energy Detect Manual Power Up/Down*/
#define EDCR_ED_BURST_DIS   0x0800      /* Energy Detect Burst Disable       */
#define EDCR_ED_PWR_STATE   0x0400      /* Energy Detect Power State         */
#define EDCR_ED_ERR_MET     0x0200      /* Energy Detect Error Threshold Met */
#define EDCR_ED_DATA_MET    0x0100      /* Energy Detect Data Threshold Met  */
#define EDCR_ED_ERR_CNT     0x00F0      /* Energy Detect Error Threshold     */
#define EDCR_ED_DATA_CNT    0x000F      /* Energy Detect Data Threshold      */

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

#endif /* __PHY_DP83848C_H */
