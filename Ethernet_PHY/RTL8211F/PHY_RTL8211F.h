/*
 * Copyright (c) 2026 Arm Limited. All rights reserved.
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
 * $Date:        29. June 2026
 * $Revision:    V1.0
 *
 * Project:      Ethernet Physical Layer Transceiver (PHY)
 *               Definitions for TL8211F-CG
 * -------------------------------------------------------------------- */

#ifndef __PHY_RTL8211F_H
#define __PHY_RTL8211F_H

#include "Driver_ETH_PHY.h"

#ifndef ARM_ETH_INTERFACE_RGMII
#define ARM_ETH_INTERFACE_RGMII 3U      /* CMSIS-Driver 2.x has no RGMII ID  */
#endif

/* ---------------------------------------------------------------------------
 * Build-time configuration. These mirror the switches used by the ST
 * reference driver (ENABLE_RTL8211F_TXDELAY / ENABLE_RTL8211F_RXDELAY /
 * DISABLE_RTL8211F_EEE).
 * ------------------------------------------------------------------------- */
#ifndef RTL8211_TX_DELAY_ENABLE
#define RTL8211_TX_DELAY_ENABLE  1U     /* Enable RGMII TX internal clock delay */
#endif

#ifndef RTL8211_RX_DELAY_ENABLE
#define RTL8211_RX_DELAY_ENABLE  1U     /* Enable RGMII RX internal clock delay */
#endif

#ifndef RTL8211_DISABLE_EEE
#define RTL8211_DISABLE_EEE      1U     /* Disable Energy-Efficient Ethernet    */
#endif

#ifndef RTL8211_ADVERTISE_1000
#define RTL8211_ADVERTISE_1000   1U     /* Advertise 1000BASE-T full duplex     */
#endif

/* ===========================================================================
 * RTL8211 register map and bit definitions
 * (values taken from the ST "stm32-rtl8211" component driver, rtl8211.h)
 * ======================================================================== */

/* IEEE standard registers (page independent) */
#define RTL8211_BMCR             0x00U  /* Basic Mode Control Register       */
#define RTL8211_BMSR             0x01U  /* Basic Mode Status Register        */
#define RTL8211_PHYID1           0x02U  /* PHY Identifier 1                  */
#define RTL8211_PHYID2           0x03U  /* PHY Identifier 2                  */
#define RTL8211_ANAR             0x04U  /* Auto-Negotiation Advertisement    */
#define RTL8211_ANLPAR           0x05U  /* AN Link Partner Ability           */
#define RTL8211_GBCR             0x09U  /* 1000Base-T Control Register       */
#define RTL8211_GBSR             0x0AU  /* 1000Base-T Status Register        */
#define RTL8211_MACR             0x0DU  /* MMD Access Control Register       */
#define RTL8211_MAADR            0x0EU  /* MMD Access Address/Data Register  */
#define RTL8211_GBESR            0x0FU  /* 1000Base-T Extended Status        */

/* Paged registers */
#define RTL8211_PHYSR1_PA43      0x1AU  /* PHY Specific Status (page 0xA43)  */
#define RTL8211_PAGSR            0x1FU  /* Page Select Register              */
#define RTL8211_EEELCR_PA4B      0x11U  /* EEE LED Control     (page 0xA4B)  */
#define RTL8211_MIICR1_PD08      0x11U  /* MII Control 1 / TXDLY (page 0xD08)*/
#define RTL8211_MIICR2_PD08      0x15U  /* MII Control 2 / RXDLY (page 0xD08)*/

/* Register pages */
#define RTL8211_PAGE_DEFAULT     0x0000U
#define RTL8211_PAGE_A43         0x0A43U
#define RTL8211_PAGE_A4B         0x0A4BU
#define RTL8211_PAGE_D08         0x0D08U

/* Basic Mode Control Register (BMCR) bits */
#define RTL8211_BMCR_RESET           0x8000U
#define RTL8211_BMCR_LOOPBACK        0x4000U
#define RTL8211_BMCR_SPEED_SEL_LSB   0x2000U  /* 100M when set (LSB)         */
#define RTL8211_BMCR_AN_EN           0x1000U
#define RTL8211_BMCR_POWER_DOWN      0x0800U
#define RTL8211_BMCR_ISOLATE         0x0400U
#define RTL8211_BMCR_RESTART_AN      0x0200U
#define RTL8211_BMCR_DUPLEX_MODE     0x0100U
#define RTL8211_BMCR_SPEED_SEL_MSB   0x0040U  /* 1000M when set (MSB)        */

/* Basic Mode Status Register (BMSR) bits */
#define RTL8211_BMSR_AN_COMPLETE     0x0020U
#define RTL8211_BMSR_LINK_STATUS     0x0004U

/* PHY identifier expected values (RTL8211F) */
#define RTL8211_PHYID1_DEFAULT       0x001CU
#define RTL8211_PHYID2_DEFAULT       0xC910U
#define RTL8211_PHYID2_MASK          0xFFF0U  /* Ignore revision nibble     */
#define RTL8211_PHYID_INVALID        0xFFFFU  /* No PHY at this address      */

/* Auto-Negotiation Advertisement Register (ANAR) bits */
#define RTL8211_ANAR_100BTX_FD       0x0100U
#define RTL8211_ANAR_100BTX_HD       0x0080U
#define RTL8211_ANAR_10BT_FD         0x0040U
#define RTL8211_ANAR_10BT_HD         0x0020U
#define RTL8211_ANAR_SELECTOR_802_3  0x0001U

/* 1000Base-T Control Register (GBCR) bits */
#define RTL8211_GBCR_1000BT_FD       0x0200U

/* PHY Specific Status Register 1 (page 0xA43) bits */
#define RTL8211_PHYSR1_SPEED_MASK    0x0030U
#define RTL8211_PHYSR1_SPEED_1000M   0x0020U
#define RTL8211_PHYSR1_SPEED_100M    0x0010U
#define RTL8211_PHYSR1_SPEED_10M     0x0000U
#define RTL8211_PHYSR1_DUPLEX        0x0008U
#define RTL8211_PHYSR1_LINK_RT       0x0004U

/* MII Control Register bits (page 0xD08) */
#define RTL8211_MIICR1_TXDLY_ENABLE  0x0100U
#define RTL8211_MIICR2_RXDLY_ENABLE  0x0008U

/* EEE disable sequence constants (matches ST DISABLE_RTL8211F_EEE) */
#define RTL8211_EEELCR_DISABLE_VAL   0x1110U
#define RTL8211_MMD_AN_DEVICE        0x0007U  /* MMD device 7 (AN)          */
#define RTL8211_MMD_FUNC_DATA        0x4000U  /* MMD function: data access  */
#define RTL8211_MMD_EEE_ADV_ADDR     0x003CU  /* MMD 7.60 EEE advertisement */

/* Polling limits. The CMSIS PHY API provides no system tick, so MDIO
   transactions are counted instead (one MDIO access is ~13 us @ 2.5 MHz). */
#define RTL8211_RESET_TIMEOUT        100000U  /* Software reset poll count   */
#define RTL8211_INIT_SETTLE          4000U    /* Post-reset settle (>=30 ms) */
#define RTL8211_PHY_ADDR_MAX         31U      /* Highest valid PHY address   */

/* PHY Driver State Flags */
#define PHY_INIT                 0x01U  /* Driver initialized                */
#define PHY_POWER                0x02U  /* Driver power is on                */

/* PHY Driver Control Structure */
typedef struct phy_ctrl {
  ARM_ETH_PHY_Read_t  reg_rd;           /* PHY register read function        */
  ARM_ETH_PHY_Write_t reg_wr;           /* PHY register write function       */
  uint16_t            bcr;              /* Cached BMCR value                 */
  uint8_t             addr;             /* Discovered PHY device address     */
  uint8_t             flags;            /* Control flags                     */
} PHY_CTRL;

#endif /* __PHY_RTL8211F_H */
