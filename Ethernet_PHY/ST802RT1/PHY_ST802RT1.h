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
 *               Definitions for ST802RT1
 * -------------------------------------------------------------------- */

#ifndef __PHY_ST802RT1_H
#define __PHY_ST802RT1_H

#include "Driver_ETH_PHY.h"

/* Basic Registers */
#define REG_CNTRL           0x00        /* Control Register                  */
#define REG_STATS           0x01        /* Status Register                   */
#define REG_PHYID1          0x02        /* PHY Identifier register Hi        */
#define REG_PHYID2          0x03        /* PHY Identifier register Lo        */
#define REG_LDADV           0x04        /* Auto-Negotiation Advertisement    */
#define REG_LPADV           0x05        /* Auto-Neg. Link Partner Abitily    */
#define REG_ANEGX           0x06        /* Auto-Neg. Expansion register      */
#define REG_LDNPG           0x07        /* Auto-Neg. Next Page transmit      */
#define REG_LPNPG           0x08        /* Auto-Neg. Link Partner received   */

/* Extended Registers */
#define REG_XCNTL           0x10        /* RMII-TEST Control register        */
#define REG_XSTAT           0x11        /* Rx Config info and Interrupt stat.*/
#define REG_XRCNT           0x12        /* Rx Event interrupts register      */
#define REG_XCCNT           0x13        /* 100Base-TX control register       */
#define REG_XDCNT           0x14        /* Receiver Mode Control register    */
#define REG_AUXCS           0x18        /* Auxiliary control register        */
#define REG_AUXSS           0x19        /* Auxiliary status register         */
#define REG_AUXM2           0x1B        /* Auxiliary mode 2 register         */
#define REG_TSTAT           0x1C        /* 10Base-T error and general status */
#define REG_AMPHY           0x1E        /* Auxiliary PHY register            */

/* Shadow Registers */
#define REG_BTEST           0x1F        /* Shadow Registers enable register  */
#define REG_AUXS2           0x1B        /* MISC/status/error/test shadow reg.*/

/* Control Register */
#define CNTRL_RESET         0x8000      /* Software Reset                    */
#define CNTRL_LOOPBACK      0x4000      /* Loopback mode                     */
#define CNTRL_SPEED_SEL     0x2000      /* Speed Select (1=100Mb/s)          */
#define CNTRL_ANEG_EN       0x1000      /* Auto Negotiation Enable           */
#define CNTRL_POWER_DOWN    0x0800      /* Power Down                        */
#define CNTRL_ISOLATE       0x0400      /* Isolate Media interface           */
#define CNTRL_REST_ANEG     0x0200      /* Restart Auto Negotiation          */
#define CNTRL_DUPLEX        0x0100      /* Duplex Mode (1=Full duplex)       */
#define CNTRL_COL_TEST      0x0080      /* Collision Test                    */

/* Status Register */
#define STATS_100B_T4       0x8000      /* 100BASE-T4 Capable                */
#define STATS_100B_TX_FD    0x4000      /* 100BASE-TX Full Duplex Capable    */
#define STATS_100B_TX_HD    0x2000      /* 100BASE-TX Half Duplex Capable    */
#define STATS_10B_T_FD      0x1000      /* 10BASE-T Full Duplex Capable      */
#define STATS_10B_T_HD      0x0800      /* 10BASE-T Half Duplex Capable      */
#define STATS_MF_PRE_SUP    0x0040      /* Preamble suppression Capable      */
#define STATS_ANEG_COMPL    0x0020      /* Auto Negotiation Complete         */
#define STATS_REM_FAULT     0x0010      /* Remote Fault                      */
#define STATS_ANEG_ABIL     0x0008      /* Auto Negotiation Ability          */
#define STATS_LINK_STAT     0x0004      /* Link Status (1=established)       */
#define STATS_JABBER_DET    0x0002      /* Jaber Detect                      */
#define STATS_EXT_CAPAB     0x0001      /* Extended Capability               */

/* PHY Identifier Registers */
#define PHY_ID1             0x0203      /* ST802RT1 Device Identifier MSB    */
#define PHY_ID2             0x8460      /* ST802RT1 Device Identifier LSB    */

/* RMII-TEST Control Register */
#define XCNTL_MII_EN        0x0200      /* MII Enabled (1=enabled)           */
#define XCNTL_FEF_EN        0x0020      /* Far End Fault enabled             */
#define XCNTL_FIFO_EXT      0x0004      /* FIFO Extend for RMII mode         */
#define XCNTL_RMII_OOBS     0x0002      /* Out-of-band signaling enabled     */

/* Receiver Configuration Information and Interrupt Status Register */
#define XSTAT_FX_MODE       0x0400      /* FX Mode set                       */
#define XSTAT_SPEED         0x0200      /* Speed (1=100MB, 0=10MB)           */
#define XSTAT_DUPLEX        0x0100      /* Duplex (1=Full, 0=Half)           */
#define XSTAT_PAUSE         0x0080      /* Pause enabled                     */
#define XSTAT_ANEG_COMPL    0x0040      /* Auto Negotiation Completed interr.*/
#define XSTAT_REM_FAULT     0x0020      /* Remote Fault Interrupt pending    */
#define XSTAT_LINK_DOWN     0x0010      /* Link Down Interrupt pending       */
#define XSTAT_ANEG_ACK      0x0008      /* Acknowledge Match Interrupt pend. */
#define XSTAT_PAR_FAULT     0x0004      /* Parallel Detection Fault Interrupt*/
#define XSTAT_ANEG_PAGE_REC 0x0002      /* Auto Negotiation Page received Int*/
#define XSTAT_RX_FULL       0x0001      /* Receive Error Buffer Full Interr. */

/* Receiver Event Interrupts Register */
#define XRCNT_INT_OE_N      0x0100      /* Interrupt Output Enable           */
#define XRCNT_INT_EN        0x0080      /* Interrupt Enable                  */
#define XRCNT_AN_CMPL_EN    0x0040      /* Auto Negotiation Completed IE     */
#define XRCNT_REMFLT_DET_EN 0x0020      /* Remote Fault IE                   */
#define XRCNT_LK_DWN_EN     0x0010      /* Link Fail IE                      */
#define XRCNT_AN_ACK_DET_EN 0x0008      /* Auto Negotiation LCW Received IE  */
#define XRCNT_PD_FLT_EN     0x0004      /* Parallel Detection Fault IE       */
#define XRCNT_PG_RCVD_EN    0x0002      /* Auto Negotiation Page Received IE */
#define XRCNT_RX_FULL_EN    0x0001      /* Receive Error Counter Full IE     */

/* 100Base-TX Control Register */
#define XCCNT_RX_ERR_DIS    0x2000      /* RX error counter disable          */
#define XCCNT_ANEG_COMPL    0x1000      /* Auto Negotiation complete         */
#define XCCNT_DC_REST_EN    0x0100      /* DC restoration enable             */
#define XCCNT_NRZ_NRZI_EN   0x0080      /* nrz<->nrzi conversion enable      */
#define XCCNT_TX_ISO        0x0020      /* Transmit MII isolate              */
#define XCCNT_CMODE         0x001C      /* Mode of operation status          */
#define XCCNT_MLT3_DIS      0x0002      /* MLT3 encoder/decoder disable      */
#define XCCNT_SCR_DIS       0x0001      /* Scrambler and descrambler disable */

/* Receiver Mode Control Register */
#define XDCNT_PHY_ADDR      0x00F8      /* Physical Address for MDIO         */
#define XDCNT_PREAM_SUPP    0x0002      /* Preamble suppression receive MDIO */

/* Auxiliary Control Register */
#define AUXCS_JABBER_DIS    0x8000      /* Jabber detection disable (10baseT)*/
#define AUXCS_MDIO_PW_SAVE  0x0010      /* Stop MDC when MDIO is idle        */

/* Auxiliary Status Register */
#define AUXSS_ANEG_COMPL    0x8000      /* Auto Negotiation Completed        */
#define AUXSS_ANEG_ACK      0x4000      /* Auto Negotiation Completed Ack    */
#define AUXSS_ANEG_DETECT   0x2000      /* Auto Negotiation Ack Match compl. */
#define AUXSS_LP_ANEG_ABIL  0x1000      /* Auto Negotiation Ability          */
#define AUXSS_ANEG_PAUSE    0x0800      /* Auto Negotiation Pause enable     */
#define AUXSS_ANEG_HCD      0x0700      /* Auto Negotiation HCD              */
#define AUXSS_PAR_DET_FAULT 0x0080      /* Parallel Detection Fault          */
#define AUXSS_REM_FAULT     0x0040      /* Remote Fault                      */
#define AUXSS_PAGE_REC      0x0020      /* Link Code word received           */
#define AUXSS_LP_ANEG_ABLE  0x0010      /* Link Partner Auto-Negotiation able*/
#define AUXSS_SP100_IND     0x0008      /* Speed is 100Mbs (100Base-TX)      */
#define AUXSS_LINK_STAT     0x0004      /* Link Status (1=up)                */
#define AUXSS_ANEG_EN       0x0002      /* Auto Negotiation Enable           */
#define AUXSS_JABBER_DET    0x0001      /* Jabber Condition Detected         */

/* Auxiliary Mode 2 Register */
#define AUXM2_LED_MODE      0x0200      /* LED Operation Mode                */
#define AUXM2_BLOCK_ECHO    0x0080      /* Disable 10Base-T echo             */
#define AUXM2_MI_SEQ_DIS    0x0008      /* Force sig.quality error (10M/Half)*/

/* 10Base-T Error and General Status Register */
#define TSTAT_MDIX_STAT     0x2000      /* MDI-X Mode active                 */
#define TSTAT_MDIX_SWAP     0x1000      /* MDI-X Force                       */
#define TSTAT_MDIX_DIS      0x0800      /* MDI-X Mode Disable                */
#define TSTAT_JABBER_DET    0x0200      /* Jabber Condition detected         */
#define TSTAT_POL_CHAN      0x0100      /* Polarity Changed event            */

/* Auxiliary PHY Register */
#define AMPHY_HCD100_TX_FDX 0x8000      /* AN 100Base-TX full-duplex selected*/
#define AMPHY_HCD100_T4     0x4000      /* AN 100Base-T4 selected            */
#define AMPHY_HCD100_TX_HDX 0x2000      /* AN 100Base-TX half-duplex select. */
#define AMPHY_HCD10_T_FDX   0x1000      /* AN 10Base-T full-duplex selected  */
#define AMPHY_HCD10_T_HDX   0x0800      /* AN 10Base-T half-duplex selected  */
#define AMPHY_ANEG_REST     0x0100      /* Restarts Auto Negotiaton process  */
#define AMPHY_ANEG_COMPL    0x0080      /* Auto Negotiaton process completed */
#define AMPHY_ANEG_ACK_COM  0x0040      /* AN Ack completed                  */
#define AMPHY_ANEG_ACK      0x0020      /* AN First ack received             */
#define AMPHY_ANEG_ABIL     0x0010      /* AN in Ability Detect state        */
#define AMPHY_SUPER_ISO     0x0008      /* Super Isolate (MII and RX)        */

/* Shadow Registers Enable Register */
#define BTEST_SH_REG_EN     0x0080      /* Whadow Registers Enable           */

/* Misc Status/Error/Test Shadow Register */
#define AUXS2_MLT3_DET      0x8000      /* MLT3 enabled with no errors       */
#define AUXS2_TX_CAB_LEN    0x7000      /* TX100 Cable length (m)            */
#define AUXS2_LED_TEST      0x0400      /* LED frequencies up by 8192 times  */
#define AUXS2_DESC_LOCK     0x0200      /* Descrambler Locked on RX stream   */
#define AUXS2_FALSE_CD      0x0100      /* False Carrier detected            */
#define AUXS2_BAD_ESD       0x0080      /* End-of-stream delimiter missing   */
#define AUXS2_RX_ERROR      0x0040      /* RX error detected (100Base-X)     */
#define AUXS2_LOCK_ERR      0x0010      /* Lock Error detected               */
#define AUXS2_MDL3_ERR      0x0008      /* MLT3 Error detected               */

/* PHY Driver State Flags */
#define PHY_INIT            0x01U       /* Driver initialized                */
#define PHY_POWER           0x02U       /* Driver power is on                */

/* PHY Driver Control Structure */
typedef struct phy_ctrl {
  ARM_ETH_PHY_Read_t  reg_rd;           /* PHY register read function        */
  ARM_ETH_PHY_Write_t reg_wr;           /* PHY register write function       */
  uint16_t            cntrl;            /* CNTRL register value              */
  uint8_t             flags;            /* Control flags                     */
  uint8_t             rsvd;             /* Reserved                          */
} PHY_CTRL;

#endif /* __PHY_ST802RT1_H */
