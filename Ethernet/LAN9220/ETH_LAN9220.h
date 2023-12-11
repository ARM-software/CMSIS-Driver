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
 * Project:      Register Interface Definitions for LAN9220
 * -------------------------------------------------------------------- */

#ifndef ETH_LAN9220__
#define ETH_LAN9220__

#include <stdint.h>

/* LAN9220 Register Interface */
typedef struct {
  uint32_t  RX_DATA_PORT;          /* Receive FIFO Ports                     Offset: 0x00 */
  uint32_t  RESERVED1[0x7];
  uint32_t  TX_DATA_PORT;          /* Transmit FIFO Ports                    Offset: 0x20 */
  uint32_t  RESERVED2[0x7];
  uint32_t  RX_STAT_PORT;          /* Receive FIFO Status Port               Offset: 0x40 */
  uint32_t  RX_STAT_PEEK;          /* Receive FIFO Status Peek               Offset: 0x44 */
  uint32_t  TX_STAT_PORT;          /* Transmit FIFO Status Port              Offset: 0x48 */
  uint32_t  TX_STAT_PEEK;          /* Transmit FIFO Status Peek              Offset: 0x4C */
  uint32_t  ID_REV;                /* Chip ID and Revision                   Offset: 0x50 */
  uint32_t  IRQ_CFG;               /* Main Interrupt Configuration           Offset: 0x54 */
  uint32_t  INT_STS;               /* Interrupt Status                       Offset: 0x58 */
  uint32_t  INT_EN;                /* Interrupt Enable Register              Offset: 0x5C */
  uint32_t  RESERVED3;             /* Reserved for future use                Offset: 0x60 */
  uint32_t  BYTE_TEST;             /* Read-only byte order testing register  Offset: 0x64 */
  uint32_t  FIFO_INT;              /* FIFO Level Interrupts                  Offset: 0x68 */
  uint32_t  RX_CFG;                /* Receive Configuration                  Offset: 0x6C */
  uint32_t  TX_CFG;                /* Transmit Configuration                 Offset: 0x70 */
  uint32_t  HW_CFG;                /* Hardware Configuration                 Offset: 0x74 */
  uint32_t  RX_DP_CTL;             /* RX Datapath Control                    Offset: 0x78 */
  uint32_t  RX_FIFO_INF;           /* Receive FIFO Information               Offset: 0x7C */
  uint32_t  TX_FIFO_INF;           /* Transmit FIFO Information              Offset: 0x80 */
  uint32_t  PMT_CTRL;              /* Power Management Control               Offset: 0x84 */
  uint32_t  GPIO_CFG;              /* General Purpose IO Configuration       Offset: 0x88 */
  uint32_t  GPT_CFG;               /* General Purpose Timer Configuration    Offset: 0x8C */
  uint32_t  GPT_CNT;               /* General Purpose Timer Count            Offset: 0x90 */
  uint32_t  RESERVED4;             /* Reserved for future use                Offset: 0x94 */
  uint32_t  WORD_SWAP;             /* WORD SWAP Register                     Offset: 0x98 */
  uint32_t  FREE_RUN;              /* Free Run Counter                       Offset: 0x9C */
  uint32_t  RX_DROP;               /* RX Dropped Frames Counter              Offset: 0xA0 */
  uint32_t  MAC_CSR_CMD;           /* MAC CSR Synchronizer Command           Offset: 0xA4 */
  uint32_t  MAC_CSR_DATA;          /* MAC CSR Synchronizer Data              Offset: 0xA8 */
  uint32_t  AFC_CFG;               /* Automatic Flow Control Configuration   Offset: 0xAC */
  uint32_t  E2P_CMD;               /* EEPROM Command                         Offset: 0xB0 */
  uint32_t  E2P_DATA;              /* EEPROM Data                            Offset: 0xB4 */
} LAN9220_TypeDef;


/* LAN9220 MAC Register Indexes */
#define REG_MAC_CR                   (0x01U)
#define REG_MAC_ADDRH                (0x02U)
#define REG_MAC_ADDRL                (0x03U)
#define REG_MAC_HASHH                (0x04U)
#define REG_MAC_HASHL                (0x05U)
#define REG_MAC_MII_ACC              (0x06U)
#define REG_MAC_MII_DATA             (0x07U)
#define REG_MAC_FLOW                 (0x08U)
#define REG_MAC_VLAN1                (0x09U)
#define REG_MAC_VLAN2                (0x0AU)
#define REG_MAC_WUFF                 (0x0BU)
#define REG_MAC_WUCSR                (0x0CU)
#define REG_MAC_COE_CR               (0x0DU)

/* LAN9220 PHY Register Indexes */
#define REG_PHY_BCONTROL             (0U)
#define REG_PHY_BSTATUS              (1U)
#define REG_PHY_ID1                  (2U)
#define REG_PHY_ID2                  (3U)
#define REG_PHY_ANEG_ADV             (4U)
#define REG_PHY_ANEG_LPA             (5U)
#define REG_PHY_ANEG_EXP             (6U)
#define REG_PHY_MCONTROL             (17U)
#define REG_PHY_MSTATUS              (18U)
#define REG_PHY_CSINDICATE           (27U)
#define REG_PHY_INTSRC               (29U)
#define REG_PHY_INTMASK              (30U)
#define REG_PHY_CS                   (31U)


/* Tx command A format definitions */
#define TX_CMDA_INT_COMPLETE_Pos     (31UL)
#define TX_CMDA_INT_COMPLETE_Msk     (1UL     << TX_CMDA_INT_COMPLETE_Pos)
#define TX_CMDA_BUF_END_ALIGN_Pos    (24UL)
#define TX_CMDA_BUF_END_ALIGN_Msk    (0x03UL  << TX_CMDA_BUF_END_ALIGN_Pos)
#define TX_CMDA_DATA_OFFS_Pos        (16UL)
#define TX_CMDA_DATA_OFFS_Msk        (0x1FUL  << TX_CMDA_DATA_OFFS_Pos)
#define TX_CMDA_FIRST_SEGMENT_Pos    (13UL)
#define TX_CMDA_FIRST_SEGMENT_Msk    (1UL     << TX_CMDA_FIRST_SEGMENT_Pos)
#define TX_CMDA_LAST_SEGMENT_Pos     (12UL)
#define TX_CMDA_LAST_SEGMENT_Msk     (1UL     << TX_CMDA_LAST_SEGMENT_Pos)
#define TX_CMDA_BUF_SIZE_Pos         (0UL)
#define TX_CMDA_BUF_SIZE_Msk         (0x7FFUL << TX_CMDA_BUF_SIZE_Pos)

/* Tx command B format definitions */
#define TX_CMDB_PACKET_TAG_Pos       (16UL)
#define TX_CMDB_PACKET_TAG_Msk       (0xFFFFUL << TX_CMDB_PACKET_TAG_Pos)
#define TX_CMDB_CHECKSUM_EN_Pos      (14UL)
#define TX_CMDB_CHECKSUM_EN_Msk      (1UL      << TX_CMDB_CHECKSUM_EN_Pos)
#define TX_CMDB_NO_CRC_Pos           (13UL)
#define TX_CMDB_NO_CRC_Msk           (1UL      << TX_CMDB_NO_CRC_Pos)
#define TX_CMDB_NO_FRAME_PADDING_Pos (12UL)
#define TX_CMDB_NO_FRAME_PADDING_Msk (1UL      << TX_CMDB_NO_FRAME_PADDING_Pos)
#define TX_CMDB_PACKET_LEN_Pos       (0UL)
#define TX_CMDB_PACKET_LEN_Msk       (0x7FFUL  << TX_CMDB_PACKET_LEN_Pos)

/* Tx status format definitions */
#define TX_STAT_PACKET_TAG_Pos       (16UL)
#define TX_STAT_PACKET_TAG_Msk       (0xFFFFUL << TX_STAT_PACKET_TAG_Pos)
#define TX_STAT_ERROR_Pos            (15UL)
#define TX_STAT_ERROR_Msk            (1UL      << TX_STAT_ERROR_Pos)
#define TX_STAT_CARRIER_LOSS_Pos     (11UL)
#define TX_STAT_CARRIER_LOSS_Msk     (1UL      << TX_STAT_CARRIER_LOSS_Pos)
#define TX_STAT_NO_CARRIER_Pos       (10UL)
#define TX_STAT_NO_CARRIER_Msk       (1UL      << TX_STAT_NO_CARRIER_Pos)
#define TX_STAT_LATE_COLLISION_Pos   (9UL)
#define TX_STAT_LATE_COLLISION_Msk   (1UL      << TX_STAT_LATE_COLLISION_Pos)
#define TX_STAT_EXCESS_COLLISION_Pos (8UL)
#define TX_STAT_EXCESS_COLLISION_Msk (1UL      << TX_STAT_EXCESS_COLLISION_Pos)
#define TX_STAT_COLLISION_CNT_Pos    (3UL)
#define TX_STAT_COLLISION_CNT_Msk    (0xFUL    << TX_STAT_COLLISION_CNT_Pos)
#define TX_STAT_EXCESS_DEFERRAL_Pos  (2UL)
#define TX_STAT_EXCESS_DEFERRAL_Msk  (1UL      << TX_STAT_EXCESS_DEFERRAL_Pos)
#define TX_STAT_DEFERRED_Pos         (0UL)
#define TX_STAT_DEFERRED_Msk         (1UL      << TX_STAT_DEFERRED_Pos)

/* Rx status format definitions */
#define RX_STAT_FILTER_FAIL_Pos      (30UL)
#define RX_STAT_FILTER_FAIL_Msk      (1UL      << RX_STAT_FILTER_FAIL_Pos)
#define RX_STAT_PACKET_LEN_Pos       (16UL)
#define RX_STAT_PACKET_LEN_Msk       (0x3FFFUL << RX_STAT_PACKET_LEN_Pos)
#define RX_STAT_ERROR_STATUS_Pos     (15UL)
#define RX_STAT_ERROR_STATUS_Msk     (1UL      << RX_STAT_ERROR_STATUS_Pos)
#define RX_STAT_BROADCAST_Pos        (13UL)
#define RX_STAT_BROADCAST_Msk        (1UL      << RX_STAT_BROADCAST_Pos)
#define RX_STAT_LENGTH_ERROR_Pos     (12UL)
#define RX_STAT_LENGTH_ERROR_Msk     (1UL      << RX_STAT_LENGTH_ERROR_Pos)
#define RX_STAT_RUNT_FRAME_Pos       (11UL)
#define RX_STAT_RUNT_FRAME_Msk       (1UL      << RX_STAT_RUNT_FRAME_Pos)
#define RX_STAT_MULTICAST_Pos        (10UL)
#define RX_STAT_MULTICAST_Msk        (1UL      << RX_STAT_MULTICAST_Pos)
#define RX_STAT_FRAME_TO_LONG_Pos    (7UL)
#define RX_STAT_FRAME_TO_LONG_Msk    (1UL      << RX_STAT_FRAME_TO_LONG_Pos)
#define RX_STAT_COLLISION_SEEN_Pos   (6UL)
#define RX_STAT_COLLISION_SEEN_Msk   (1UL      << RX_STAT_COLLISION_SEEN_Pos)
#define RX_STAT_FRAME_TYPE_Pos       (5UL)
#define RX_STAT_FRAME_TYPE_Msk       (1UL      << RX_STAT_FRAME_TYPE_Pos)
#define RX_STAT_RX_WDT_TOUT_Pos      (4UL)
#define RX_STAT_RX_WDT_TOUT_Msk      (1UL      << RX_STAT_RX_WDT_TOUT_Pos)
#define RX_STAT_MII_ERROR_Pos        (3UL)
#define RX_STAT_MII_ERROR_Msk        (1UL      << RX_STAT_MII_ERROR_Pos)
#define RX_STAT_DRIBBLING_Pos        (2UL)
#define RX_STAT_DRIBBLING_Msk        (1UL      << RX_STAT_DRIBBLING_Pos)
#define RX_STAT_CRC_ERROR_Pos        (1UL)
#define RX_STAT_CRC_ERROR_Msk        (1UL      << RX_STAT_CRC_ERROR_Pos)


/* Direct Address Registers */

/* Chip ID and revision (ID_REV) */
#define ID_REV_CHIP_REV_Pos          (0UL)
#define ID_REV_CHIP_REV_Msk          (0xFFFFUL << ID_REV_CHIP_REV_Pos)
#define ID_REV_CHIP_ID_Pos           (16UL)
#define ID_REV_CHIP_ID_Msk           (0xFFFFUL << ID_REV_CHIP_ID_Pos)

/* FIFO Level Interrupts (FIFO_INT) */
#define FIFO_INT_RXSL_Pos            (0U)
#define FIFO_INT_RXSL_Msk            (0xFFUL << FIFO_INT_RXSL_Pos)
#define FIFO_INT_TXSL_Pos            (16U)
#define FIFO_INT_TXSL_Msk            (0xFFUL << FIFO_INT_TXSL_Pos)
#define FIFO_INT_TDAL_Pos            (24U)
#define FIFO_INT_TDAL_Msk            (0xFFUL << FIFO_INT_TDAL_Pos)

/* Receive Configuration Register (RX_CFG) */
#define RX_CFG_RXDOFF_Pos            (8UL)
#define RX_CFG_RXDOFF_Msk            (0x1FUL << RX_CFG_RXDOFF_Pos)
#define RX_CFG_RX_DUMP_Pos           (15UL)
#define RX_CFG_RX_DUMP_Msk           (1UL << RX_CFG_RX_DUMP_Pos)
#define RX_CFG_RX_DMA_CNT_Pos        (16UL)
#define RX_CFG_RX_DMA_CNT_Msk        (0xFFFUL << RX_CFG_RX_DMA_CNT_Pos)
#define RX_CFG_END_ALIGNMENT_Pos     (30UL)
#define RX_CFG_END_ALIGNMENT_        (3UL << RX_CFG_END_ALIGNMENT_Pos)
                                     
/* Transmit Configuration Register (TX_CFG) */
#define TX_CFG_STOP_TX_Pos           (0UL)
#define TX_CFG_STOP_TX_Msk           (1UL << TX_CFG_STOP_TX_Pos)
#define TX_CFG_TX_ON_Pos             (1UL)
#define TX_CFG_TX_ON_Msk             (1UL << TX_CFG_TX_ON_Pos)
#define TX_CFG_TXSAO_Pos             (2UL)
#define TX_CFG_TXSAO_Msk             (1UL << TX_CFG_TXSAO_Pos)
#define TX_CFG_TXD_DUMP_Pos          (14UL)
#define TX_CFG_TXD_DUMP_Msk          (1UL << TX_CFG_TXD_DUMP_Pos)
#define TX_CFG_TXS_DUMP_Pos          (15UL)
#define TX_CFG_TXS_DUMP_Msk          (1UL << TX_CFG_TXS_DUMP_Pos)

/* Hardware Configuration Register (HW_CFG) */
#define HW_CFG_SRST_Pos              (0UL)
#define HW_CFG_SRST_Msk              (1UL << HW_CFG_SRST_Pos)
#define HW_CFG_SRST_TO_Pos           (1UL)
#define HW_CFG_SRST_TO_Msk           (1UL << HW_CFG_SRST_TO_Pos)
#define HW_CFG_TX_FIF_Pos            (16UL)
#define HW_CFG_TX_FIF_SZ_Msk         (0xFUL << HW_CFG_TX_FIF_Pos)
#define HW_CFG_MBO_Pos               (20UL)
#define HW_CFG_MBO_Msk               (1UL << HW_CFG_MBO_Pos)

/* Receive Datapath Control Register (RX_DP_CTRL) */
#define RX_DP_CTRL_RX_FFWD_Pos       (31UL)
#define RX_DP_CTRL_RX_FFWD_Msk       (1UL << RX_DP_CTRL_RX_FFWD_Pos)

/* Receive FIFO Information Register (RX_FIFO_INF) */
#define RX_FIFO_INF_RXDUSED_Pos      (0UL)
#define RX_FIFO_INF_RXDUSED_Msk      (0xFFFFUL << RX_FIFO_INF_RXDUSED_Pos)
#define RX_FIFO_INF_RXSUSED_Pos      (16UL)
#define RX_FIFO_INF_RXSUSED_Msk      (0xFFUL << RX_FIFO_INF_RXSUSED_Pos)

/* Transmit FIFO Information Register (TX_FIFO_INF) */
#define TX_FIFO_INF_TDFREE_Pos       (0UL)
#define TX_FIFO_INF_TDFREE_Msk       (0xFFFFUL << TX_FIFO_INF_TDFREE_Pos)
#define TX_FIFO_INF_TXSUSED_Pos      (16UL)
#define TX_FIFO_INF_TXSUSED_Msk      (0x00FFUL << TX_FIFO_INF_TXSUSED_Pos)

/* Power Management Control Register (PMT_CTRL) */
#define PMT_CTRL_READY_Pos           (0UL)
#define PMT_CTRL_READY_Msk           (1UL << PMT_CTRL_READY_Pos)
#define PMT_CTRL_PM_MODE_Pos         (12UL)
#define PMT_CTRL_PM_MODE_Msk         (3UL << PMT_CTRL_PM_MODE_Pos)

/* General Purpose IO Configuration Register (GPIO_CFG) */
#define GPIO_CFG_LEDx_EN_Pos         (28UL)
#define GPIO_CFG_LEDx_EN_Msk         (0x7UL << GPIO_CFG_LEDx_EN_Pos)

/* MAC CSR Synchronizer Command Register (MAC_CSR_CMD) */
#define MAC_CSR_ADDR_Pos             (0UL)
#define MAC_CSR_ADDR_Msk             (0xFFUL << MAC_CSR_ADDR_Pos)
#define MAC_CSR_CMD_RNW_Pos          (30UL)
#define MAC_CSR_CMD_RNW_Msk          (1UL << MAC_CSR_CMD_RNW_Pos)
#define MAC_CSR_CMD_BUSY_Pos         (31UL)
#define MAC_CSR_CMD_BUSY_Msk         (1UL << MAC_CSR_CMD_BUSY_Pos)

/* Automatic Flow Control Configuration Register (AFC_CFG) */
#define AFC_CFG_FCANY_Pos            (0UL)
#define AFC_CFG_FCANY_Msk            (1UL << AFC_CFG_FCANY_Pos)
#define AFC_CFG_FCADD_Pos            (1UL)
#define AFC_CFG_FCADD_Msk            (1UL << AFC_CFG_FCADD_Pos)
#define AFC_CFG_FCBRD_Pos            (2UL)
#define AFC_CFG_FCBRD_Msk            (1UL << AFC_CFG_FCBRD_Pos)
#define AFC_CFG_FCMULT_Pos           (3UL)
#define AFC_CFG_FCMULT_Msk           (1UL << AFC_CFG_FCMULT_Pos)
#define AFC_CFG_BACK_DUR_Pos         (4UL)
#define AFC_CFG_BACK_DUR_Msk         (0xFUL << AFC_CFG_BACK_DUR_Pos)
#define AFC_CFG_AFC_LO_Pos           (8UL)
#define AFC_CFG_AFC_LO_Msk           (0xFFUL << AFC_CFG_AFC_LO_Pos)
#define AFC_CFG_AFC_HI_Pos           (16UL)
#define AFC_CFG_AFC_HI_Msk           (0xFFUL << AFC_CFG_AFC_HI_Pos)

/* EEPROM Command Register (E2P_CMD) */
#define E2P_CMD_BUSY_Pos             (31UL)
#define E2P_CMD_BUSY_Msk             (1UL << E2P_CMD_BUSY_Pos)


/* MAC Control and Status Registers */

/* MAC control register */
#define MAC_CR_RXEN_Pos              (2UL)
#define MAC_CR_RXEN_Msk              (1UL << MAC_CR_RXEN_Pos)
#define MAC_CR_TXEN_Pos              (3UL)
#define MAC_CR_TXEN_Msk              (1UL << MAC_CR_TXEN_Pos)
#define MAC_CR_BCAST_Pos             (11UL)
#define MAC_CR_BCAST_Msk             (1UL << MAC_CR_BCAST_Pos)
#define MAC_CR_HPFILT_Pos            (13UL)
#define MAC_CR_HPFILT_Msk            (1UL << MAC_CR_HPFILT_Pos)
#define MAC_CR_HO_Pos                (15UL)
#define MAC_CR_HO_Msk                (1UL << MAC_CR_HO_Pos)
#define MAC_CR_PRMS_Pos              (18UL)
#define MAC_CR_PRMS_Msk              (1UL << MAC_CR_PRMS_Pos)
#define MAC_CR_MCPAS_Pos             (19UL)
#define MAC_CR_MCPAS_Msk             (1UL << MAC_CR_MCPAS_Pos)
#define MAC_CR_FDPX_Pos              (20UL)
#define MAC_CR_FDPX_Msk              (1UL << MAC_CR_FDPX_Pos)
#define MAC_CR_LOOPBK_Pos            (21UL)
#define MAC_CR_LOOPBK_Msk            (1UL << MAC_CR_LOOPBK_Pos)
#define MAC_CR_RXALL_Pos             (31UL)
#define MAC_CR_RXALL_Msk             (1UL << MAC_CR_RXALL_Pos)

/* MII Access Register BitMasks (MII_ACC) */
#define MII_ACC_BUSY_Pos             (0UL)
#define MII_ACC_BUSY_Msk             (1UL << MII_ACC_BUSY_Pos)
#define MII_ACC_WRITE_Pos            (1UL)
#define MII_ACC_WRITE_Msk            (1UL << MII_ACC_WRITE_Pos)
#define MII_ACC_REGIDX_Pos           (6UL)
#define MII_ACC_REGIDX_Msk           (0x1FUL << MII_ACC_REGIDX_Pos)
#define MII_ACC_PHYADDR_Pos          (11UL)
#define MII_ACC_PHYADDR_Msk          (0x1FUL << MII_ACC_PHYADDR_Pos)

/* Checksum offload engine control register */
#define COE_CR_RXCOE_EN_Pos          (0UL)
#define COE_CR_RXCOE_EN_Msk          (1UL << COE_CR_RXCOE_EN_Pos)
#define COE_CR_RXCOE_MODE_Pos        (1UL)
#define COE_CR_RXCOE_MODE_Msk        (1UL << COE_CR_RXCOE_MODE_Pos)
#define COE_CR_TXCOE_EN_Pos          (16)
#define COE_CR_TXCOE_EN_Msk          (1UL << COE_CR_TXCOE_EN_Pos)


/* PHY Registers */

/* PHY: Basic Control Register */
#define PHY_BCR_COLLISION_TST_Pos    (7UL)
#define PHY_BCR_COLLISION_TST_Msk    (1UL << PHY_BCR_COLLISION_TST_Pos)
#define PHY_BCR_DUPLEX_MODE_Pos      (8UL)
#define PHY_BCR_DUPLEX_MODE_Msk      (1UL << PHY_BCR_DUPLEX_MODE_Pos)
#define PHY_BCR_RESTART_AN_Pos       (9UL)
#define PHY_BCR_RESTART_AN_Msk       (1UL << PHY_BCR_RESTART_AN_Pos)
#define PHY_BCR_POWER_DOWN_Pos       (11UL)
#define PHY_BCR_POWER_DOWN_Msk       (1UL << PHY_BCR_POWER_DOWN_Pos)
#define PHY_BCR_AN_ENABLE_Pos        (12UL)
#define PHY_BCR_AN_ENABLE_Msk        (1UL << PHY_BCR_AN_ENABLE_Pos)
#define PHY_BCR_SPEED_SELECT_Pos     (13UL)
#define PHY_BCR_SPEED_SELECT_Msk     (1UL << PHY_BCR_SPEED_SELECT_Pos)
#define PHY_BCR_LOOPBACK_Pos         (14UL)
#define PHY_BCR_LOOPBACK_Msk         (1UL << PHY_BCR_LOOPBACK_Pos)
#define PHY_BCR_RESET_Pos            (15UL)
#define PHY_BCR_RESET_Msk            (1UL << PHY_BCR_RESET_Pos)

/* PHY: Basic Status Register */
#define PHY_BSR_LINK_STATUS_Pos      (2UL)
#define PHY_BSR_LINK_STATUS_Msk      (1UL << PHY_BSR_LINK_STATUS_Pos)
#define PHY_BSR_AN_COMPLETE_Pos      (5UL)
#define PHY_BSR_AN_COMPLETE_Msk      (1UL << PHY_BSR_AN_COMPLETE_Pos)
#define PHY_BSR_10B_HALFD_Pos        (11UL)
#define PHY_BSR_10B_HALFD_Msk        (1UL << PHY_BSR_10B_HALFD_Pos)
#define PHY_BSR_10B_FULLD_Pos        (12UL)
#define PHY_BSR_10B_FULLD_Msk        (1UL << PHY_BSR_10B_FULLD_Pos)
#define PHY_BSR_100B_HALFD_Pos       (13UL)
#define PHY_BSR_100B_HALFD_Msk       (1UL << PHY_BSR_100B_HALFD_Pos)
#define PHY_BSR_100B_FULLD_Pos       (12UL)
#define PHY_BSR_100B_FULLD_Msk       (1UL << PHY_BSR_100B_FULLD_Pos)


/* Various Definitions */

/* Timeouts */
#define LAN9220_TOUT_REG_RW          (20UL)     /* 20us  */
#define LAN9220_TOUT_FIFO_DUMP       (100000UL) /* 100ms */
#define LAN9220_TOUT_PHY             (100000UL) /* 100ms */
#define LAN9220_TOUT_RESET           (500000UL) /* 500ms */

/* PHY address should be always 1 */
#define LAN9220_PHY_ADDR             (1U)

/* ETH Driver state flags */
#define ETH_INIT                     (0x01UL)   /* Driver initialized */
#define ETH_POWER                    (0x02UL)   /* Driver powered     */

/* Number of frames that fit into TX FIFO (1536 byte per frame) */
#define ETH_TX_FIFO_FRAME_CNT        (3UL)

/* Tx Packet buffer size */
#define ETH_BUF_SIZE                 (1536UL)

/* ETH Driver Control Information */
typedef struct {
  uint32_t Us_Cyc;
  uint32_t Tx_Len;
  uint16_t Tx_PacketTag;
  uint8_t  Flags;
  uint8_t  Rsvd;
} ETH_CTRL;

/* System Core Clock is used to initialize loop timeouts */
extern uint32_t SystemCoreClock;

#endif /* ETH_LAN9220__ */
