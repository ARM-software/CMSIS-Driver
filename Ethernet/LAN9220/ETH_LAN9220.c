/*
 * Copyright (c) 2013-2022 Arm Limited. All rights reserved.
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
 * $Date:        25. March 2022
 * $Revision:    V1.1
 *
 * Driver:       Driver_ETH_MACn (default: Driver_ETH_MAC0),
 *               Driver_ETH_PHYn (default: Driver_ETH_PHY0)
 * Project:      Ethernet Media Access (MAC) Driver and
 *               Ethernet Physical Layer Transceiver (PHY) Driver
 *               for LAN9220
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                     Value
 *   ---------------------                     -----
 *   Connect to hardware via Driver_ETH_MAC# = n (default: 0)
 *                           Driver_ETH_PHY# = n (default: 0)
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.1
 *    Corrected invalid power status in MAC_PowerControl
 *  Version 1.0
 *    Initial release
 */

#include <string.h>
#include "cmsis_compiler.h"

#include "Driver_ETH_MAC.h"
#include "Driver_ETH_PHY.h"
#include "ETH_LAN9220.h"


#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,1) /* driver version */
#define ARM_ETH_PHY_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,1) /* driver version */


/* Ethernet MAC driver number (default) */
#ifndef ETH_MAC_NUM
#define ETH_MAC_NUM             0
#endif

/* Ethernet PHY driver number (default) */
#ifndef ETH_PHY_NUM
#define ETH_PHY_NUM             0
#endif

/* LAN9220 Base Address (default) */
#ifndef LAN9220_BASE
#define LAN9220_BASE            (0x52000000UL)
#endif


/* LAN9220 Interface Access */
#define LAN9220                 ((LAN9220_TypeDef volatile *)LAN9220_BASE)

#ifdef __clang__
  #define __rbit(v)             __builtin_arm_rbit(v)
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION MAC_DriverVersion = {
  ARM_ETH_MAC_API_VERSION,
  ARM_ETH_MAC_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES MAC_DriverCapabilities = {
  0,                                /* checksum_offload_rx_ip4  */
  0,                                /* checksum_offload_rx_ip6  */
  0,                                /* checksum_offload_rx_udp  */
  0,                                /* checksum_offload_rx_tcp  */
  0,                                /* checksum_offload_rx_icmp */
  0,                                /* checksum_offload_tx_ip4  */
  0,                                /* checksum_offload_tx_ip6  */
  0,                                /* checksum_offload_tx_udp  */
  0,                                /* checksum_offload_tx_tcp  */
  0,                                /* checksum_offload_tx_icmp */
  0,                                /* media_interface          */
  0,                                /* mac_address              */
  0,                                /* event_rx_frame           */
  0,                                /* event_tx_frame           */
  0,                                /* event_wakeup             */
  0,                                /* precision_timer          */
  0                                 /* reserved                 */
};

/* Driver control structure */
static ETH_CTRL ETH = { 0, 0, 0, 0, 0 };

/* Intermediate tx buffer */
static uint8_t Tx_Buf[ETH_BUF_SIZE];

/* Local functions */
static uint32_t init_loop_tout (uint32_t timeout, uint32_t loop_cycles);
static uint32_t reg_rd (uint8_t reg);
static void     reg_wr (uint8_t reg, uint32_t val);
static int32_t  phy_busy (void);
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val);
static uint32_t crc32_data (const uint8_t *data, uint32_t len);

/**
  Initialize loop timeout value.

  \parma[in]   timeout      Timeout value in microseconds
  \param[in]   loop_cycles  Number of instructions within a loop
*/
static uint32_t init_loop_tout (uint32_t timeout, uint32_t loop_cycles) {
  uint32_t tout;

  /* Determine number of cycles within required timeout */
  tout = ETH.Us_Cyc * timeout;

  /* Normalize tout in cycles to loop duration */
  tout = (tout + (loop_cycles-1)) / loop_cycles;

  /* Return normalized timeout */
  return (tout);
}

/**
  Read register value.

  \param[in]   reg  Register to read
*/
static uint32_t reg_rd (uint8_t reg) {
  uint32_t cmd, val, tout;

  /* Set register offset, read mode and busy indicator */
  LAN9220->MAC_CSR_CMD = reg | MAC_CSR_CMD_RNW_Msk | MAC_CSR_CMD_BUSY_Msk;

  /* Wait till busy */
  tout = init_loop_tout(LAN9220_TOUT_REG_RW, 20U);
  do {
    cmd = LAN9220->MAC_CSR_CMD;

    if ((cmd & MAC_CSR_CMD_BUSY_Msk) == 0) {
      break;
    }
    tout--;
  } while (tout != 0U);

  if ((cmd & MAC_CSR_CMD_BUSY_Msk) == 0) {
    val = LAN9220->MAC_CSR_DATA;
  } else {
    val = 0U;
  }

  return (val);
}

/**
  Write register value.

  \param[in]   reg  Register to write
  \param[in]   val  Value to write
*/
static void reg_wr (uint8_t reg, uint32_t val) {
  uint32_t cmd, tout;

  /* Store register value */
  LAN9220->MAC_CSR_DATA = val;
  
  /* Set register offset, write mode and busy indicator */
  LAN9220->MAC_CSR_CMD = reg | MAC_CSR_CMD_BUSY_Msk;

  /* Wait till busy */
  tout = init_loop_tout(LAN9220_TOUT_REG_RW, 20U);
  do {
    cmd = LAN9220->MAC_CSR_CMD;

    if ((cmd & MAC_CSR_CMD_BUSY_Msk) == 0) {
      break;
    }
    tout--;
  } while (tout != 0U);
}

/**
  Wait until PHY clears busy flag.
*/
static int32_t phy_busy (void) {
  uint32_t tout, val;

  tout = init_loop_tout (LAN9220_TOUT_PHY, 20U);
  while (tout--) {
    /* Read MII access register */
    val = reg_rd (REG_MAC_MII_ACC);

    if ((val & MII_ACC_BUSY_Msk) == 0U) {
      /* Busy flag cleared */
      return ARM_DRIVER_OK;
    }
  }
  return ARM_DRIVER_ERROR;
}


/**
  \fn          uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val)
  \brief       Calculate 32-bit CRC (Polynom: 0x04C11DB7, data bit-reversed).
  \param[in]   crc32  CRC initial value
  \param[in]   val    Input value
  \return      Calculated CRC value
*/
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val) {
  uint32_t n;

  crc32 ^= __rbit (val);
  for (n = 8; n; n--) {
    if (crc32 & 0x80000000) {
      crc32 <<= 1;
      crc32  ^= 0x04C11DB7;
    } else {
      crc32 <<= 1;
    }
  }
  return (crc32);
}

/**
  \fn          uint32_t crc32_data (const uint8_t *data, uint32_t len)
  \brief       Calculate standard 32-bit Ethernet CRC.
  \param[in]   data  Pointer to buffer containing the data
  \param[in]   len   Data length in bytes
  \return      Calculated CRC value
*/
static uint32_t crc32_data (const uint8_t *data, uint32_t len) {
  uint32_t crc;

  for (crc = 0xFFFFFFFF; len; len--) {
    crc = crc32_8bit_rev (crc, *data++);
  }
  return (crc);
}


/* Ethernet Driver functions */

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION MAC_GetVersion (void) {
  return MAC_DriverVersion;
}

/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES MAC_GetCapabilities (void) {
  return MAC_DriverCapabilities;
}

/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t MAC_Initialize (ARM_ETH_MAC_SignalEvent_t cb_event) {
  (void)cb_event;

  if (ETH.Flags & ETH_INIT) { return ARM_DRIVER_OK; }

  /* Determine number of CPU cycles within 1us */
  if (SystemCoreClock < 1000000U) {
    ETH.Us_Cyc = 1U;
  } else {
    ETH.Us_Cyc = (SystemCoreClock + (1000000U-1U)) / 1000000U;
  }

  ETH.Flags = ETH_INIT;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t MAC_Uninitialize (void) {

  ETH.Flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t MAC_PowerControl (ARM_POWER_STATE state) {
  uint32_t tout, val;

  switch ((int32_t)state) {
    case ARM_POWER_OFF:
      ETH.Flags &= ~ETH_POWER;

      /* Power down */
      LAN9220->PMT_CTRL = (0x02U << PMT_CTRL_PM_MODE_Pos);
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((ETH.Flags & ETH_INIT) == 0) {
        return ARM_DRIVER_ERROR;
      }
      if (ETH.Flags & ETH_POWER) {
        return ARM_DRIVER_OK;
      }

      /* Check device ID */
      val = (LAN9220->ID_REV & ID_REV_CHIP_ID_Msk) >> ID_REV_CHIP_ID_Pos;

      if (val != 0x9220UL) {
        /* Invalid ID */
        return ARM_DRIVER_ERROR;
      }

      /* Initiate software reset */
      /* Note: device must be read at least once before any write */
      LAN9220->HW_CFG |= HW_CFG_SRST_Msk;

      tout = init_loop_tout (LAN9220_TOUT_RESET, 15U);
      while (tout--) {
        if ((LAN9220->HW_CFG & HW_CFG_SRST_Msk) == 0) {
          /* Software initiated reset completed */
          break;
        }
      }
      if ((LAN9220->HW_CFG & HW_CFG_SRST_Msk) != 0) {
        return ARM_DRIVER_ERROR;
      }

      /* Disable interrupts */
      LAN9220->IRQ_CFG = 0U;
      LAN9220->INT_EN  = 0U;
      LAN9220->INT_STS = 0U;

      /* Clean-up Tx FIFO */
      LAN9220->TX_CFG = TX_CFG_TXS_DUMP_Msk | TX_CFG_TXD_DUMP_Msk;

      /* Clean-up Rx FIFO and ensure 4-byte end alignment */
      LAN9220->RX_CFG = RX_CFG_RX_DUMP_Msk;

      /* Wait until clean-up complete */
      tout = init_loop_tout (LAN9220_TOUT_FIFO_DUMP, 15U);
      while (tout--) {
        if ((LAN9220->RX_CFG & RX_CFG_RX_DUMP_Msk) == 0) {
          break;
        }
      }
      if ((LAN9220->RX_CFG & RX_CFG_RX_DUMP_Msk) != 0) {
        return ARM_DRIVER_ERROR;
      }

      /* Configure Tx FIFO Data Available Level interrupt to 1 frame */
      LAN9220->FIFO_INT = (1536 / 64) << FIFO_INT_TDAL_Pos;

      /* Set TX FIFO size to 4k */
      val  = (ETH_TX_FIFO_FRAME_CNT + 1) << HW_CFG_TX_FIF_Pos;
      val |= HW_CFG_MBO_Msk;

      LAN9220->HW_CFG = val;

      /* Wait until EPC Busy */
      tout = init_loop_tout (LAN9220_TOUT_RESET, 15U);
      while (tout--) {
        if ((LAN9220->E2P_CMD & E2P_CMD_BUSY_Msk) == 0) {
          break;
        }
      }
      if ((LAN9220->E2P_CMD & E2P_CMD_BUSY_Msk) != 0) {
        return ARM_DRIVER_ERROR;
      }

      /* Enable LEDs */
      LAN9220->GPIO_CFG = GPIO_CFG_LEDx_EN_Msk;

      /* Configure automatic flow control */
      LAN9220->AFC_CFG = (192 << AFC_CFG_AFC_HI_Pos)   |
                         (64  << AFC_CFG_AFC_LO_Pos)   |
                         (5   << AFC_CFG_BACK_DUR_Pos) | AFC_CFG_FCANY_Msk;

      ETH.Flags |= ETH_POWER;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t MAC_GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr) {
  uint32_t val;

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((ETH.Flags & ETH_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  val = reg_rd (REG_MAC_ADDRL);
  ptr_addr->b[0] = (uint8_t)val;
  ptr_addr->b[1] = (uint8_t)(val >>  8);
  ptr_addr->b[2] = (uint8_t)(val >> 16);
  ptr_addr->b[3] = (uint8_t)(val >> 24);

  val = reg_rd (REG_MAC_ADDRH);
  ptr_addr->b[4] = (uint8_t)val;
  ptr_addr->b[5] = (uint8_t)(val >> 8);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t MAC_SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr) {
  uint32_t val;

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((ETH.Flags & ETH_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  val  = (uint32_t) ptr_addr->b[0];
  val |= (uint32_t)(ptr_addr->b[1] <<  8);
  val |= (uint32_t)(ptr_addr->b[2] << 16);
  val |= (uint32_t)(ptr_addr->b[3] << 24);

  reg_wr (REG_MAC_ADDRL, val);

  val  = (uint32_t) ptr_addr->b[4];
  val |= (uint32_t)(ptr_addr->b[5] << 8);

  reg_wr (REG_MAC_ADDRH, val);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t MAC_SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr) {
  uint32_t crc, val;
  uint32_t hi, lo;

  if ((ptr_addr == NULL) && (num_addr != 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((ETH.Flags & ETH_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  /* Read MAC control register value */
  val = reg_rd (REG_MAC_CR);

  if (num_addr == 0U) {
    /* Disable multicast hash filtering */
    val |= MAC_CR_MCPAS_Msk;
  }
  else {
    hi = 0U;
    lo = 0U;

    /* Calculate 64-bit Hash table for MAC addresses */
    while (num_addr > 0U) {
      crc = crc32_data (&ptr_addr->b[0], 6) >> 26;
      if (crc & 0x20) {
        hi |= (1 << (crc & 0x1F));
      } else {
        lo |= (1 << crc);
      }

      ptr_addr++;
      num_addr--;
    }

    /* Set hash registers */
    reg_wr (REG_MAC_HASHH, hi);
    reg_wr (REG_MAC_HASHL, lo);

    /* Enable multicast hash filtering (Mode: MAC perfect + hash for multicasts) */
    val &= ~MAC_CR_MCPAS_Msk;
    val |=  MAC_CR_HPFILT_Msk;
  }

  /* Set new MAC control value */
  reg_wr (REG_MAC_CR, val);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t MAC_SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags) {
  uint32_t cmd_a, cmd_b, i;
  uint32_t free;

  if ((frame == NULL) || (len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((ETH.Flags & ETH_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  /* Check Tx FIFO free space */
  free = (LAN9220->TX_FIFO_INF & TX_FIFO_INF_TDFREE_Msk) >> TX_FIFO_INF_TDFREE_Pos;

  if (free < (ETH.Tx_Len + len)) {
    /* Not enough space, FIFO is busy transmitting */
    return ARM_DRIVER_ERROR_BUSY;
  }

  if ((ETH.Tx_Len + len) > ETH_BUF_SIZE) {
    /* Frame size invalid */
    return ARM_DRIVER_ERROR;
  }

  memcpy (&Tx_Buf[ETH.Tx_Len], frame, len);

  ETH.Tx_Len += len;

  if ((flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) == 0) {
    /* Last fragment, send frame */

    /* Increment packet tag */
    ETH.Tx_PacketTag++;

    /* Setup commands */
    cmd_a  = (ETH.Tx_Len << TX_CMDA_BUF_SIZE_Pos) & TX_CMDA_BUF_SIZE_Msk;
    cmd_a |= TX_CMDA_FIRST_SEGMENT_Msk | TX_CMDA_LAST_SEGMENT_Msk;
    
    cmd_b  = (ETH.Tx_Len << TX_CMDB_PACKET_LEN_Pos) & TX_CMDB_PACKET_LEN_Msk;
    cmd_b |= (uint32_t)(ETH.Tx_PacketTag << TX_CMDB_PACKET_TAG_Pos);

    LAN9220->TX_DATA_PORT = cmd_a;
    LAN9220->TX_DATA_PORT = cmd_b;

    /* Align length */
    ETH.Tx_Len = (ETH.Tx_Len + 3U) & ~3U;

    /* Send content from the intermediate buffer */
    for (i = 0U; i < ETH.Tx_Len; i += 4U) {
      LAN9220->TX_DATA_PORT = __UNALIGNED_UINT32_READ(&Tx_Buf[i]);
    }

    ETH.Tx_Len = 0U;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref execution_status 
*/
static int32_t MAC_ReadFrame (uint8_t *frame, uint32_t len) {
  uint32_t stat;
  uint32_t data, sz, cnt;
  int32_t rval;

  if ((frame == NULL) && (len != 0U)) {
    rval = ARM_DRIVER_ERROR_PARAMETER;
  }
  else if ((ETH.Flags & ETH_POWER) == 0U) {
    rval = ARM_DRIVER_ERROR;
  }
  else {
    rval = 0;

    /* Read frame status word */
    stat = LAN9220->RX_STAT_PORT;

    /* Extract packet size */
    sz = (stat & RX_STAT_PACKET_LEN_Msk) >> RX_STAT_PACKET_LEN_Pos;

    if ((frame == NULL) && (len == 0U)) {
      /* Discard received frame */
      if (sz < 32U) {
        /* Manual FIFO read out */
        while (sz) {
          sz--;
          (void)LAN9220->RX_DATA_PORT;
        }
      }
      else {
        /* Use FIFO Fast Forward */
        LAN9220->RX_DP_CTL = RX_DP_CTRL_RX_FFWD_Msk;
      }
    }
    else {
      if (sz > len) {
        rval = ARM_DRIVER_ERROR;
      }
      else {
        rval = (int32_t)sz;

        /* Adjust length */
        sz = len & ~3U;

        /* Copy aligned bytes */
        for (cnt = 0; cnt < sz; cnt += 4) {
          __UNALIGNED_UINT32_WRITE (&frame[cnt], LAN9220->RX_DATA_PORT);
        }

        /* Adjust length */
        sz = (len + 3U) & ~3U;

        /* Copy remaining bytes */
        if (cnt < sz) {
          data = LAN9220->RX_DATA_PORT;

          while (cnt < sz) {
            frame[cnt] = (uint8_t)data;
            data = data >> 8U;
            cnt++;
          }
        }
      }
    }
  }

  return (rval);
}

/**
  \fn          uint32_t GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t MAC_GetRxFrameSize (void) {
  uint32_t stat, len;
  
  if ((ETH.Flags & ETH_POWER) == 0U) {
    return (0U);
  }

  if ((LAN9220->RX_FIFO_INF & RX_FIFO_INF_RXSUSED_Msk) == 0) {
    /* RX Status FIFO empty, no frames */
    len = 0U;
  }
  else {
    /* Peek FIFO status */
    stat = LAN9220->RX_STAT_PEEK;

    /* Extract packet size */
    len = (stat & RX_STAT_PACKET_LEN_Msk) >> RX_STAT_PACKET_LEN_Pos;
    
    if (len > 0U) {
      /* Discard received frame on error */
      if ((stat & RX_STAT_ERROR_STATUS_Msk) != 0U) {
        /* Dummy status word read */
        (void)LAN9220->RX_STAT_PORT;

        if (len < 32U) {
          /* Manual FIFO read out */
          while (len) {
            len--;
            (void)LAN9220->RX_DATA_PORT;
          }
        }
        else {
          /* Use FIFO Fast Forward */
          LAN9220->RX_DP_CTL = RX_DP_CTRL_RX_FFWD_Msk;
        }
        len = 0U;
      }
    }
  }

  return (len);
}

/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t MAC_GetRxFrameTime (ARM_ETH_MAC_TIME *time) {
  (void)time;
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t MAC_GetTxFrameTime (ARM_ETH_MAC_TIME *time) {
  (void)time;
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  Operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t MAC_ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time) {
  (void)control;
  (void)time;
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t MAC_Control (uint32_t control, uint32_t arg) {
  uint32_t mac_cr;
  uint32_t tout, val;

  if ((ETH.Flags & ETH_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  /* Read MAC Control register value */
  mac_cr = reg_rd (REG_MAC_CR);

  switch (control) {
    case ARM_ETH_MAC_CONFIGURE:
      if ((arg & ARM_ETH_MAC_SPEED_Msk) == ARM_ETH_MAC_SPEED_1G) {
        /* 1Gbit is not supported */
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      if ((arg & ARM_ETH_MAC_DUPLEX_Msk) == ARM_ETH_MAC_DUPLEX_FULL) {
        /* Enable full duplex mode */
        mac_cr |=  MAC_CR_FDPX_Msk;
      } else {
        /* Enable half duplex mode */
        mac_cr &= ~MAC_CR_FDPX_Msk;
      }

      if ((arg & ARM_ETH_MAC_LOOPBACK) != 0) {
        /* Enable loopback mode */
        mac_cr |=  MAC_CR_LOOPBK_Msk;
      } else {
        mac_cr &= ~MAC_CR_LOOPBK_Msk;
      }

      if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) ||
          (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)) {
        /* Checksum offload is disabled in the driver */
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      if ((arg & ARM_ETH_MAC_ADDRESS_BROADCAST) != 0) {
        /* Enable broadcast frame receive */
        mac_cr &= ~MAC_CR_BCAST_Msk;
      } else {
        mac_cr |=  MAC_CR_BCAST_Msk;
      }

      if ((arg & ARM_ETH_MAC_ADDRESS_MULTICAST) != 0) {
        /* Enable all multicast frame receive */
        mac_cr |=  MAC_CR_MCPAS_Msk;
      } else {
        mac_cr &= ~MAC_CR_MCPAS_Msk;
      }

      if ((arg & ARM_ETH_MAC_ADDRESS_ALL) != 0) {
        /* Enable all frame receive (Promiscuous mode) */
        mac_cr |=  MAC_CR_PRMS_Msk;
      } else {
        mac_cr &= ~MAC_CR_PRMS_Msk;
      }

      /* Set configuration */
      reg_wr (REG_MAC_CR, mac_cr);
      break;

    case ARM_ETH_MAC_CONTROL_TX:
      /* Enable/disable MAC transmitter */
      mac_cr = reg_rd (REG_MAC_CR);

      if (arg != 0) {
        mac_cr |=  MAC_CR_TXEN_Msk;
      } else {
        mac_cr &= ~MAC_CR_TXEN_Msk;
      }
      reg_wr (REG_MAC_CR, mac_cr);

      LAN9220->TX_CFG = TX_CFG_TX_ON_Msk | TX_CFG_TXSAO_Msk;
      break;

    case ARM_ETH_MAC_CONTROL_RX:
      /* Enable/disable MAC receiver */
      mac_cr = reg_rd (REG_MAC_CR);

      if (arg != 0) {
        mac_cr |=  MAC_CR_RXEN_Msk;
      } else {
        mac_cr &= ~MAC_CR_RXEN_Msk;
      }
      reg_wr (REG_MAC_CR, mac_cr);
      break;

    case ARM_ETH_MAC_FLUSH:
      /* Flush Tx and Rx buffers */
      if ((arg & ARM_ETH_MAC_FLUSH_RX) != 0) {
        mac_cr  = reg_rd (REG_MAC_CR);
        /* Disable receiver */
        mac_cr &= ~MAC_CR_RXEN_Msk;
        reg_wr (REG_MAC_CR, mac_cr);

        /* Force RX Discard */
        LAN9220->RX_CFG = RX_CFG_RX_DUMP_Msk;
        
        tout = init_loop_tout(LAN9220_TOUT_FIFO_DUMP, 15U);
        do {
          if ((LAN9220->RX_CFG & RX_CFG_RX_DUMP_Msk) == 0) {
            break;
          }
          tout--;
        } while (tout);

        if ((LAN9220->RX_CFG & RX_CFG_RX_DUMP_Msk) == 0) {
          /* Enable receiver */
          mac_cr |= MAC_CR_RXEN_Msk;
          reg_wr (REG_MAC_CR, mac_cr);
        }
        else {
          return ARM_DRIVER_ERROR;
        }
      }

      if ((arg & ARM_ETH_MAC_FLUSH_TX) != 0) {
        /* Disable transmitter */
        LAN9220->TX_CFG |= TX_CFG_STOP_TX_Msk;

        /* Wait till transmitter stops */
        tout = init_loop_tout(LAN9220_TOUT_FIFO_DUMP, 30U);
        do {
          if ((LAN9220->TX_CFG & TX_CFG_STOP_TX_Msk) == 0) {
            /* Discard status and data */
            LAN9220->TX_CFG |= TX_CFG_TXS_DUMP_Msk | TX_CFG_TXD_DUMP_Msk;

            val = LAN9220->TX_CFG & (TX_CFG_TXS_DUMP_Msk | TX_CFG_TXD_DUMP_Msk);

            if (val == (TX_CFG_TXS_DUMP_Msk | TX_CFG_TXD_DUMP_Msk)) {
              /* Re-enable transmitter */
              LAN9220->TX_CFG |= TX_CFG_TX_ON_Msk;
            }
          }
          tout--;
        } while (tout);
      }
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}


/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data) {
  int32_t  status;
  uint32_t cmd;

  status = phy_busy();

  if (status == ARM_DRIVER_OK) {
    cmd  = (uint32_t)(phy_addr << MII_ACC_PHYADDR_Pos);
    cmd |= (uint32_t)(reg_addr << MII_ACC_REGIDX_Pos);

    reg_wr (REG_MAC_MII_ACC, cmd);

    status = phy_busy();

    if (status == ARM_DRIVER_OK) {
      *data = (uint16_t)reg_rd (REG_MAC_MII_DATA);
    }
  }

  return (status);
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data) {
  int32_t  status;
  uint32_t cmd;

  status = phy_busy();

  if (status == ARM_DRIVER_OK) {
    reg_wr (REG_MAC_MII_DATA, data);

    cmd  = (uint32_t)(phy_addr << MII_ACC_PHYADDR_Pos);
    cmd |= (uint32_t)(reg_addr << MII_ACC_REGIDX_Pos);
    cmd |= MII_ACC_WRITE_Msk;

    reg_wr (REG_MAC_MII_ACC, cmd);
  }

  return (status);
}


/* MAC Driver Control Block */
extern
ARM_DRIVER_ETH_MAC ARM_Driver_ETH_MAC_(ETH_MAC_NUM);
ARM_DRIVER_ETH_MAC ARM_Driver_ETH_MAC_(ETH_MAC_NUM) = {
  MAC_GetVersion,
  MAC_GetCapabilities,
  MAC_Initialize,
  MAC_Uninitialize,
  MAC_PowerControl,
  MAC_GetMacAddress,
  MAC_SetMacAddress,
  MAC_SetAddressFilter,
  MAC_SendFrame,
  MAC_ReadFrame,
  MAC_GetRxFrameSize,
  MAC_GetRxFrameTime,
  MAC_GetTxFrameTime,
  MAC_ControlTimer,
  MAC_Control,
  PHY_Read,
  PHY_Write
};


/* Driver Version */
static const ARM_DRIVER_VERSION PHY_DriverVersion = {
  ARM_ETH_PHY_API_VERSION,
  ARM_ETH_PHY_DRV_VERSION
};

/**
  \fn          ARM_DRV_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION PHY_GetVersion (void) {
  return PHY_DriverVersion;
}

/**
  \fn          int32_t Initialize (ARM_ETH_PHY_Read_t  fn_read,
                                   ARM_ETH_PHY_Write_t fn_write)
  \brief       Initialize Ethernet PHY Device.
  \param[in]   fn_read   Pointer to \ref ARM_ETH_MAC_PHY_Read
  \param[in]   fn_write  Pointer to \ref ARM_ETH_MAC_PHY_Write
  \return      \ref execution_status
*/
static int32_t PHY_Initialize (ARM_ETH_PHY_Read_t fn_read, ARM_ETH_PHY_Write_t fn_write) {
  /* PHY_Read and PHY_Write will be re-used, no need to register them again */
  (void)fn_read;
  (void)fn_write;
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet PHY Device.
  \return      \ref execution_status
*/
static int32_t PHY_Uninitialize (void) {
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet PHY Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PHY_PowerControl (ARM_POWER_STATE state) {
  uint16_t val;

  if ((ETH.Flags & ETH_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  if (PHY_Read (LAN9220_PHY_ADDR, REG_PHY_BCONTROL, &val) != ARM_DRIVER_OK) {
    return ARM_DRIVER_ERROR;
  }

  switch ((int32_t)state) {
    case ARM_POWER_OFF:
      /* Select Power Saving Mode */
      val |= PHY_BCR_POWER_DOWN_Msk;
      break;
    case ARM_POWER_FULL:
      /* Select Normal Operation Mode */
      val &= ~PHY_BCR_POWER_DOWN_Msk;
      break;
    case ARM_POWER_LOW:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return PHY_Write (LAN9220_PHY_ADDR, REG_PHY_BCONTROL, val);
}

/**
  \fn          int32_t SetInterface (uint32_t interface)
  \brief       Set Ethernet Media Interface.
  \param[in]   interface  Media Interface type
  \return      \ref execution_status
*/
static int32_t PHY_SetInterface (uint32_t interface) {
  /* Not used in this driver */
  (void)interface;
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMode (uint32_t mode)
  \brief       Set Ethernet PHY Device Operation mode.
  \param[in]   mode  Operation Mode
  \return      \ref execution_status
*/
static int32_t PHY_SetMode (uint32_t mode) {
  uint16_t val;

  if ((ETH.Flags & ETH_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  if (PHY_Read (LAN9220_PHY_ADDR, REG_PHY_BCONTROL, &val) != ARM_DRIVER_OK) {
    return ARM_DRIVER_ERROR;
  }

  val &= ~(PHY_BCR_SPEED_SELECT_Msk | PHY_BCR_DUPLEX_MODE_Msk |
           PHY_BCR_AN_ENABLE_Msk    | PHY_BCR_LOOPBACK_Msk);

  switch (mode & ARM_ETH_PHY_SPEED_Msk) {
    case ARM_ETH_PHY_SPEED_10M:
      break;
    case ARM_ETH_PHY_SPEED_100M:
      val |= PHY_BCR_SPEED_SELECT_Msk;
      break;
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  switch (mode & ARM_ETH_PHY_DUPLEX_Msk) {
    case ARM_ETH_PHY_DUPLEX_HALF:
      break;
    case ARM_ETH_PHY_DUPLEX_FULL:
      val |= PHY_BCR_DUPLEX_MODE_Msk;
      break;
  }

  if (mode & ARM_ETH_PHY_AUTO_NEGOTIATE) {
    val |= PHY_BCR_AN_ENABLE_Msk;
  }

  if (mode & ARM_ETH_PHY_LOOPBACK) {
    val |= PHY_BCR_LOOPBACK_Msk;
  }

  if (mode & ARM_ETH_PHY_ISOLATE) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  /* Apply configured mode */
  return PHY_Write (LAN9220_PHY_ADDR, REG_PHY_BCONTROL, val);
}

/**
  \fn          ARM_ETH_LINK_STATE GetLinkState (void)
  \brief       Get Ethernet PHY Device Link state.
  \return      current link status \ref ARM_ETH_LINK_STATE
*/
static ARM_ETH_LINK_STATE PHY_GetLinkState (void) {
  ARM_ETH_LINK_STATE state = ARM_ETH_LINK_DOWN;
  uint16_t val = 0U;

  if (ETH.Flags & ETH_POWER) {
    if (PHY_Read (LAN9220_PHY_ADDR, REG_PHY_BSTATUS, &val) == ARM_DRIVER_OK) {
      if (val & PHY_BSR_LINK_STATUS_Msk) {
        /* Link Good bit is set */
        state = ARM_ETH_LINK_UP;
      }
    }
  }
  return (state);
}

/**
  \fn          ARM_ETH_LINK_INFO GetLinkInfo (void)
  \brief       Get Ethernet PHY Device Link information.
  \return      current link parameters \ref ARM_ETH_LINK_INFO
*/
static ARM_ETH_LINK_INFO PHY_GetLinkInfo (void) {
  ARM_ETH_LINK_INFO info;
  uint32_t speed, duplex;
  uint16_t val = 0U;

  speed  = ARM_ETH_SPEED_10M;
  duplex = ARM_ETH_DUPLEX_HALF;

  if (ETH.Flags & ETH_POWER) {
    if (PHY_Read (LAN9220_PHY_ADDR, REG_PHY_BSTATUS, &val) == ARM_DRIVER_OK) {
      if ((val & (PHY_BSR_100B_FULLD_Msk | PHY_BSR_100B_HALFD_Msk)) != 0U) {
        /* 100Base */
        speed  = ARM_ETH_SPEED_100M;

        if ((val & PHY_BSR_100B_FULLD_Msk) != 0U) {
          /* Full duplex */
          duplex = ARM_ETH_DUPLEX_FULL;
        }
      }
      else {
        /* 10Base */
        if ((val & PHY_BSR_10B_FULLD_Msk) != 0U) {
          /* Full duplex */
          duplex = ARM_ETH_DUPLEX_FULL;
        }
      }
    }
  }
  /* Link must be up to get valid state */
  info.speed  = speed;
  info.duplex = duplex;

  return (info);
}

/* PHY Driver Control Block */
extern
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY_(ETH_PHY_NUM);
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY_(ETH_PHY_NUM) = {
  PHY_GetVersion,
  PHY_Initialize,
  PHY_Uninitialize,
  PHY_PowerControl,
  PHY_SetInterface,
  PHY_SetMode,
  PHY_GetLinkState,
  PHY_GetLinkInfo
};
