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
 * Driver:       Driver_ETH_PHYn (default: Driver_ETH_PHY0)
 * Project:      Ethernet Physical Layer Transceiver (PHY)
 *               Driver for PHY_RTL8211F
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                     Value
 *   ---------------------                     -----
 *   Connect to hardware via Driver_ETH_PHY# = n (default: 0)
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.0
 *    Initial release
 */

#include "PHY_RTL8211F.h"

#define ARM_ETH_PHY_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

#ifndef ETH_PHY_NUM
#define ETH_PHY_NUM             0       /* Default driver number             */
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_ETH_PHY_API_VERSION,
  ARM_ETH_PHY_DRV_VERSION
};

static PHY_CTRL PHY = { NULL, NULL, 0U, 0U, 0U };

/**
  \fn          int32_t phy_read (uint8_t reg_addr, uint16_t *data)
  \brief       Read a PHY register at the active device address.
*/
static int32_t phy_read (uint8_t reg_addr, uint16_t *data) {
  return PHY.reg_rd(PHY.addr, reg_addr, data);
}

/**
  \fn          int32_t phy_write (uint8_t reg_addr, uint16_t data)
  \brief       Write a PHY register at the active device address.
*/
static int32_t phy_write (uint8_t reg_addr, uint16_t data) {
  return PHY.reg_wr(PHY.addr, reg_addr, data);
}

/**
  \fn          int32_t SetPage (uint16_t page)
  \brief       Select an RTL8211 register page.
*/
static int32_t SetPage (uint16_t page) {
  return phy_write(RTL8211_PAGSR, page);
}

/**
  \fn          int32_t ReadPhysr (uint16_t *data)
  \brief       Read the PHY Specific Status Register (page 0xA43).
*/
static int32_t ReadPhysr (uint16_t *data) {
  int32_t status;
  int32_t restore;

  status = SetPage(RTL8211_PAGE_A43);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  status  = phy_read(RTL8211_PHYSR1_PA43, data);
  restore = SetPage(RTL8211_PAGE_DEFAULT);
  if (status == ARM_DRIVER_OK) {
    status = restore;
  }

  return status;
}

/**
  \fn          int32_t ScanPhyAddress (void)
  \brief       Find and identify the RTL8211 on the MDIO bus.
  \details     Mirrors the address scan performed by the ST reference driver:
               the first address returning a valid (non 0xFFFF) PHY ID 1 is
               used; the device is then verified to be an RTL8211.
*/
static int32_t ScanPhyAddress (void) {
  uint16_t id1;
  uint16_t id2;
  uint8_t  addr;
  int32_t  status;

  for (addr = 0U; addr <= RTL8211_PHY_ADDR_MAX; addr++) {
    id1 = RTL8211_PHYID_INVALID;
    if (PHY.reg_rd(addr, RTL8211_PHYID1, &id1) != ARM_DRIVER_OK) {
      continue;
    }
    if (id1 == RTL8211_PHYID_INVALID) {
      continue;
    }

    status = PHY.reg_rd(addr, RTL8211_PHYID2, &id2);
    if (status != ARM_DRIVER_OK) {
      return status;
    }

    if ((id1 == RTL8211_PHYID1_DEFAULT) &&
        ((id2 & RTL8211_PHYID2_MASK) == RTL8211_PHYID2_DEFAULT)) {
      PHY.addr = addr;
      return ARM_DRIVER_OK;
    }
  }

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t ResetPhy (void)
  \brief       Software-reset the RTL8211 and wait until it is ready.
  \details     After the reset bit self-clears the RTL8211 needs a
               stabilization period (datasheet minimum 30 ms) before its
               registers and the RGMII interface are reliable. The ST
               reference driver waits explicitly here; with no system tick
               available the delay is produced by counted MDIO reads.
*/
static int32_t ResetPhy (void) {
  uint16_t val;
  uint32_t count;
  int32_t  status;

  status = SetPage(RTL8211_PAGE_DEFAULT);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  status = phy_write(RTL8211_BMCR, RTL8211_BMCR_RESET);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  for (count = RTL8211_RESET_TIMEOUT; count > 0U; count--) {
    status = phy_read(RTL8211_BMCR, &val);
    if (status != ARM_DRIVER_OK) {
      return status;
    }
    if ((val & RTL8211_BMCR_RESET) == 0U) {
      break;
    }
  }

  if (count == 0U) {
    return ARM_DRIVER_ERROR_TIMEOUT;
  }

  for (count = RTL8211_INIT_SETTLE; count > 0U; count--) {
    status = phy_read(RTL8211_BMSR, &val);
    if (status != ARM_DRIVER_OK) {
      return status;
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ConfigureRgmiiDelay (void)
  \brief       Configure the RTL8211 RGMII internal TX/RX clock delays.
  \details     Same register sequence as the ST reference driver: page 0xD08,
               MIICR1[8] for the TX delay and MIICR2[3] for the RX delay.
*/
static int32_t ConfigureRgmiiDelay (void) {
  uint16_t val;
  int32_t  status;

  status = SetPage(RTL8211_PAGE_D08);
  if (status != ARM_DRIVER_OK) { return status; }

  /* TX delay: reg 0x11 bit 8 */
  status = phy_read(RTL8211_MIICR1_PD08, &val);
  if (status != ARM_DRIVER_OK) { (void)SetPage(RTL8211_PAGE_DEFAULT); return status; }
#if (RTL8211_TX_DELAY_ENABLE != 0U)
  val |=  RTL8211_MIICR1_TXDLY_ENABLE;
#else
  val &= (uint16_t)~RTL8211_MIICR1_TXDLY_ENABLE;
#endif
  status = phy_write(RTL8211_MIICR1_PD08, val);
  if (status != ARM_DRIVER_OK) { (void)SetPage(RTL8211_PAGE_DEFAULT); return status; }

  /* RX delay: reg 0x15 bit 3 */
  status = phy_read(RTL8211_MIICR2_PD08, &val);
  if (status != ARM_DRIVER_OK) { (void)SetPage(RTL8211_PAGE_DEFAULT); return status; }
#if (RTL8211_RX_DELAY_ENABLE != 0U)
  val |=  RTL8211_MIICR2_RXDLY_ENABLE;
#else
  val &= (uint16_t)~RTL8211_MIICR2_RXDLY_ENABLE;
#endif
  status = phy_write(RTL8211_MIICR2_PD08, val);
  if (status != ARM_DRIVER_OK) { (void)SetPage(RTL8211_PAGE_DEFAULT); return status; }

  return SetPage(RTL8211_PAGE_DEFAULT);
}

/**
  \fn          int32_t DisableEee (void)
  \brief       Disable Energy-Efficient Ethernet (EEE/LPI).
  \details     Reproduces the ST reference driver DISABLE_RTL8211F_EEE
               sequence. In the LPI idle state the MAC drops frames that
               arrive after an idle gap (such as the DHCP OFFER), which leaves
               the host stranded on a 169.254.x.x address.
*/
static int32_t DisableEee (void) {
  int32_t status;

  /* Clear the EEE LED control bits on page 0xA4B. */
  status = SetPage(RTL8211_PAGE_A4B);
  if (status != ARM_DRIVER_OK) {
    return status;
  }
  status = phy_write(RTL8211_EEELCR_PA4B, RTL8211_EEELCR_DISABLE_VAL);
  if (status != ARM_DRIVER_OK) {
    (void)SetPage(RTL8211_PAGE_DEFAULT);
    return status;
  }
  status = SetPage(RTL8211_PAGE_DEFAULT);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  /* Clear the EEE advertisement via MMD register 7.60. */
  status = phy_write(RTL8211_MACR, RTL8211_MMD_AN_DEVICE);
  if (status != ARM_DRIVER_OK) {
    return status;
  }
  status = phy_write(RTL8211_MAADR, RTL8211_MMD_EEE_ADV_ADDR);
  if (status != ARM_DRIVER_OK) {
    return status;
  }
  status = phy_write(RTL8211_MACR,
                     (uint16_t)(RTL8211_MMD_FUNC_DATA | RTL8211_MMD_AN_DEVICE));
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  return phy_write(RTL8211_MAADR, 0U);
}

/**
  \fn          int32_t ConfigureAutoNegotiation (void)
  \brief       Program the RTL8211 advertisement registers.
*/
static int32_t ConfigureAutoNegotiation (void) {
  uint16_t val;
  int32_t  status;

  status = SetPage(RTL8211_PAGE_DEFAULT);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  val = RTL8211_ANAR_SELECTOR_802_3 |
        RTL8211_ANAR_10BT_HD        |
        RTL8211_ANAR_10BT_FD        |
        RTL8211_ANAR_100BTX_HD      |
        RTL8211_ANAR_100BTX_FD;

  status = phy_write(RTL8211_ANAR, val);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

#if (RTL8211_ADVERTISE_1000 != 0U)
  val = RTL8211_GBCR_1000BT_FD;
#else
  val = 0U;
#endif

  return phy_write(RTL8211_GBCR, val);
}

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          int32_t Initialize (ARM_ETH_PHY_Read_t  fn_read,
                                   ARM_ETH_PHY_Write_t fn_write)
  \brief       Initialize Ethernet PHY Device.
  \param[in]   fn_read   Pointer to \ref ARM_ETH_MAC_PHY_Read
  \param[in]   fn_write  Pointer to \ref ARM_ETH_MAC_PHY_Write
  \return      \ref execution_status
*/
static int32_t Initialize (ARM_ETH_PHY_Read_t fn_read, ARM_ETH_PHY_Write_t fn_write) {

  if ((fn_read == NULL) || (fn_write == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((PHY.flags & PHY_INIT) == 0U) {
    PHY.reg_rd = fn_read;
    PHY.reg_wr = fn_write;
    PHY.bcr    = 0U;
    PHY.addr   = 0U;
    PHY.flags  = PHY_INIT;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet PHY Device.
  \return      \ref execution_status
*/
static int32_t Uninitialize (void) {

  PHY.reg_rd = NULL;
  PHY.reg_wr = NULL;
  PHY.bcr    = 0U;
  PHY.addr   = 0U;
  PHY.flags  = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet PHY Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  uint16_t val;
  int32_t  status;

  switch ((int32_t)state) {
    case ARM_POWER_OFF:
      if ((PHY.flags & PHY_INIT) == 0U) {
        return ARM_DRIVER_ERROR;
      }

      status = SetPage(RTL8211_PAGE_DEFAULT);
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      status = phy_read(RTL8211_BMCR, &val);
      if (status != ARM_DRIVER_OK) {
        val = PHY.bcr;
      }

      PHY.bcr    = val | RTL8211_BMCR_POWER_DOWN;
      PHY.flags &= ~PHY_POWER;

      return phy_write(RTL8211_BMCR, PHY.bcr);

    case ARM_POWER_FULL:
      if ((PHY.flags & PHY_INIT) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((PHY.flags & PHY_POWER) != 0U) {
        return ARM_DRIVER_OK;
      }

      /* Locate and identify the RTL8211 on the MDIO bus. */
      status = ScanPhyAddress();
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      /* Software reset and mandatory post-reset stabilization. */
      status = ResetPhy();
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      /* Take the PHY out of power-down / isolate. */
      status = phy_read(RTL8211_BMCR, &val);
      if (status != ARM_DRIVER_OK) {
        return status;
      }
      PHY.bcr = val & (uint16_t)~(RTL8211_BMCR_POWER_DOWN | RTL8211_BMCR_ISOLATE);
      status  = phy_write(RTL8211_BMCR, PHY.bcr);
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      /* RGMII internal clock delays. */
      status = ConfigureRgmiiDelay();
      if (status != ARM_DRIVER_OK) {
        return status;
      }

#if (RTL8211_DISABLE_EEE != 0U)
      status = DisableEee();
      if (status != ARM_DRIVER_OK) {
        return status;
      }
#endif

      /* Advertise the supported modes and start auto-negotiation so the
         copper link is established (link LED on) regardless of how the MAC
         subsequently calls SetMode. Auto-negotiation is also the only valid
         way to bring up a 1000BASE-T link. */
      status = ConfigureAutoNegotiation();
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      PHY.bcr |= (RTL8211_BMCR_AN_EN | RTL8211_BMCR_RESTART_AN);
      status   = phy_write(RTL8211_BMCR, PHY.bcr);
      if (status != ARM_DRIVER_OK) {
        return status;
      }

      PHY.flags |= PHY_POWER;
      return ARM_DRIVER_OK;

    case ARM_POWER_LOW:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
}

/**
  \fn          int32_t SetInterface (uint32_t interface)
  \brief       Set Ethernet Media Interface.
  \param[in]   interface  Media Interface type
  \return      \ref execution_status
*/
static int32_t SetInterface (uint32_t interface) {

  if ((PHY.flags & PHY_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  switch (interface) {
    case ARM_ETH_INTERFACE_MII:
    case ARM_ETH_INTERFACE_RGMII:
      return ARM_DRIVER_OK;
    case ARM_ETH_INTERFACE_RMII:
    case ARM_ETH_INTERFACE_SMII:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
}

/**
  \fn          int32_t SetMode (uint32_t mode)
  \brief       Set Ethernet PHY Device Operation mode.
  \param[in]   mode  Operation Mode
  \return      \ref execution_status
*/
static int32_t SetMode (uint32_t mode) {
  uint16_t val;
  uint32_t autoneg;
  int32_t  status;

  if ((PHY.flags & PHY_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  val     = PHY.bcr & RTL8211_BMCR_POWER_DOWN;
  autoneg = (mode & ARM_ETH_PHY_AUTO_NEGOTIATE);

  switch (mode & ARM_ETH_PHY_SPEED_Msk) {
    case ARM_ETH_PHY_SPEED_10M:
      break;
    case ARM_ETH_PHY_SPEED_100M:
      val |= RTL8211_BMCR_SPEED_SEL_LSB;
      break;
    case ARM_ETH_PHY_SPEED_1G:
      if ((mode & ARM_ETH_PHY_DUPLEX_Msk) == ARM_ETH_PHY_DUPLEX_HALF) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      /* 1000BASE-T can only be established through auto-negotiation (the
         master/slave relationship is resolved during negotiation); it cannot
         be force-set, so enable auto-negotiation for any gigabit request. */
      autoneg = ARM_ETH_PHY_AUTO_NEGOTIATE;
      val |= RTL8211_BMCR_SPEED_SEL_MSB;
      break;
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  switch (mode & ARM_ETH_PHY_DUPLEX_Msk) {
    case ARM_ETH_PHY_DUPLEX_HALF:
      break;
    case ARM_ETH_PHY_DUPLEX_FULL:
      val |= RTL8211_BMCR_DUPLEX_MODE;
      break;
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  if (autoneg != 0U) {
    val |= (RTL8211_BMCR_AN_EN | RTL8211_BMCR_RESTART_AN);
  }
  if ((mode & ARM_ETH_PHY_LOOPBACK) != 0U) {
    val |= RTL8211_BMCR_LOOPBACK;
  }
  if ((mode & ARM_ETH_PHY_ISOLATE) != 0U) {
    val |= RTL8211_BMCR_ISOLATE;
  }

  status = SetPage(RTL8211_PAGE_DEFAULT);
  if (status != ARM_DRIVER_OK) {
    return status;
  }

  if (autoneg != 0U) {
    status = ConfigureAutoNegotiation();
    if (status != ARM_DRIVER_OK) {
      return status;
    }
  }

  PHY.bcr = val;
  return phy_write(RTL8211_BMCR, PHY.bcr);
}

/**
  \fn          ARM_ETH_LINK_STATE GetLinkState (void)
  \brief       Get Ethernet PHY Device Link state.
  \return      current link status \ref ARM_ETH_LINK_STATE
*/
static ARM_ETH_LINK_STATE GetLinkState (void) {
  uint16_t val;

  val = 0U;

  if ((PHY.flags & PHY_POWER) != 0U) {
    /* Read BMSR twice: the link status bit latches low, so the second read
       reflects the current state (same approach as the ST reference driver). */
    if (phy_read(RTL8211_BMSR, &val) != ARM_DRIVER_OK) {
      val = 0U;
    }
    else if (phy_read(RTL8211_BMSR, &val) != ARM_DRIVER_OK) {
      val = 0U;
    }
  }

  if ((val & RTL8211_BMSR_LINK_STATUS) != 0U) {
    return ARM_ETH_LINK_UP;
  }

  return ARM_ETH_LINK_DOWN;
}

/**
  \fn          ARM_ETH_LINK_INFO GetLinkInfo (void)
  \brief       Get Ethernet PHY Device Link information.
  \return      current link parameters \ref ARM_ETH_LINK_INFO
*/
static ARM_ETH_LINK_INFO GetLinkInfo (void) {
  ARM_ETH_LINK_INFO info;
  uint16_t          val;

  info.speed    = ARM_ETH_SPEED_10M;
  info.duplex   = ARM_ETH_DUPLEX_HALF;
  info.reserved = 0U;
  val           = 0U;

  if ((PHY.flags & PHY_POWER) != 0U) {
    if (ReadPhysr(&val) != ARM_DRIVER_OK) {
      val = 0U;
    }
  }

  switch (val & RTL8211_PHYSR1_SPEED_MASK) {
    case RTL8211_PHYSR1_SPEED_1000M:
      info.speed = ARM_ETH_SPEED_1G;
      break;
    case RTL8211_PHYSR1_SPEED_100M:
      info.speed = ARM_ETH_SPEED_100M;
      break;
    default:
      info.speed = ARM_ETH_SPEED_10M;
      break;
  }

  if ((val & RTL8211_PHYSR1_DUPLEX) != 0U) {
    info.duplex = ARM_ETH_DUPLEX_FULL;
  }

  return info;
}

/* PHY Driver Control Block */
extern
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY_(ETH_PHY_NUM);
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY_(ETH_PHY_NUM) = {
  GetVersion,
  Initialize,
  Uninitialize,
  PowerControl,
  SetInterface,
  SetMode,
  GetLinkState,
  GetLinkInfo
};
