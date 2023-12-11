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
 * $Revision:    V6.7
 *
 * Project:      Register Interface Definitions for KSZ8851SNL
 * -------------------------------------------------------------------- */

#ifndef __ETH_KSZ8851SNL
#define __ETH_KSZ8851SNL

#include "Driver_ETH.h"

/* Device access SPI commands */
#define SPI_CMD_REG_READ     0x00           // Read register command
#define SPI_CMD_REG_WRITE    0x40           // Write register command
#define SPI_CMD_RXQ_FIFO_RD  0x80           // Read RX Queue data
#define SPI_CMD_TXQ_FIFO_WR  0xC0           // Write TX Queue data

/* ETH Driver state flags */
#define ETH_INIT             0x01           // Driver initialized
#define ETH_POWER            0x02           // Driver power on

/* Family and Chip ID */
#define PHY_ID               0x8870

/* Register definitions */
#define REG_CCR              0x08           // Chip Configuration Register

#define REG_MARL             0x10           // Host MAC Address Register Low
#define REG_MARM             0x12           // Host MAC Address Register Middle
#define REG_MARH             0x14           // Host MAC Address Register High

#define REG_OBCR             0x20           // On-Chip Bus Control Register
#define REG_EEPCR            0x22           // EEPROM Control Register
#define REG_MBIR             0x24           // Memory BIST Info Register
#define REG_GRR              0x26           // Global Reset Register
#define REG_WFCR             0x2A           // Wakeup Frame Control Register

#define REG_WF0CRC0          0x30           // Wakeup Frame 0 CRC0 Register
#define REG_WF0CRC1          0x32           // Wakeup Frame 0 CRC1 Register
#define REG_WF0BM0           0x34           // Wakeup Frame 0 Byte Mask 0 Register
#define REG_WF0BM1           0x36           // Wakeup Frame 0 Byte Mask 1 Register
#define REG_WF0BM2           0x38           // Wakeup Frame 0 Byte Mask 2 Register
#define REG_WF0BM3           0x3A           // Wakeup Frame 0 Byte Mask 3 Register

#define REG_WF1CRC0          0x40           // Wakeup Frame 1 CRC0 Register
#define REG_WF1CRC1          0x42           // Wakeup Frame 1 CRC1 Register
#define REG_WF1BM0           0x44           // Wakeup Frame 1 Byte Mask 0 Register
#define REG_WF1BM1           0x46           // Wakeup Frame 1 Byte Mask 1 Register
#define REG_WF1BM2           0x48           // Wakeup Frame 1 Byte Mask 2 Register
#define REG_WF1BM3           0x4A           // Wakeup Frame 1 Byte Mask 3 Register

#define REG_WF2CRC0          0x50           // Wakeup Frame 2 CRC0 Register
#define REG_WF2CRC1          0x52           // Wakeup Frame 2 CRC1 Register
#define REG_WF2BM0           0x54           // Wakeup Frame 2 Byte Mask 0 Register
#define REG_WF2BM1           0x56           // Wakeup Frame 2 Byte Mask 1 Register
#define REG_WF2BM2           0x58           // Wakeup Frame 2 Byte Mask 2 Register
#define REG_WF2BM3           0x5A           // Wakeup Frame 2 Byte Mask 3 Register

#define REG_WF3CRC0          0x60           // Wakeup Frame 3 CRC0 Register
#define REG_WF3CRC1          0x62           // Wakeup Frame 3 CRC1 Register
#define REG_WF3BM0           0x64           // Wakeup Frame 3 Byte Mask 0 Register
#define REG_WF3BM1           0x66           // Wakeup Frame 3 Byte Mask 1 Register
#define REG_WF3BM2           0x68           // Wakeup Frame 3 Byte Mask 2 Register
#define REG_WF3BM3           0x6A           // Wakeup Frame 3 Byte Mask 3 Register

#define REG_TXCR             0x70           // Transmit Control Register
#define REG_TXSR             0x72           // Transmit Status Register
#define REG_RXCR1            0x74           // Receive Control Register 1
#define REG_RXCR2            0x76           // Receive Control Register 2
#define REG_TXMIR            0x78           // TXQ Memory Information Register
#define REG_RXFHSR           0x7C           // Receive Frame Header Status Register
#define REG_RXFHBCR          0x7E           // Receive Frame Header Byte Count Register

#define REG_TXQCR            0x80           // TXQ Command Register
#define REG_RXQCR            0x82           // RXQ Command Register
#define REG_TXFDPR           0x84           // TX Frame Data Pointer Register
#define REG_RXFDPR           0x86           // RX Frame Data Pointer Register
#define REG_RXDTTR           0x8C           // RX Duration Timer Threshold Register
#define REG_RXDBCTR          0x8E           // RX Data Byte Count Threshold Register

#define REG_IER              0x90           // Interrupt Enable Register
#define REG_ISR              0x92           // Interrupt Status Register
#define REG_RXFCTR           0x9C           // RX Frame Count & Threshold Register
#define REG_TXNTFSR          0x9E           // TX Next Total Frames Size Register

#define REG_MAHTR0           0xA0           // MAC Address Hash Table Register 0
#define REG_MAHTR1           0xA2           // MAC Address Hash Table Register 1
#define REG_MAHTR2           0xA4           // MAC Address Hash Table Register 2
#define REG_MAHTR3           0xA6           // MAC Address Hash Table Register 3

#define REG_FCLWR            0xB0           // Flow Control Low Watermark Register
#define REG_FCHWR            0xB2           // Flow Control High Watermark Register
#define REG_FCOWR            0xB4           // Flow Control Overrun Watermark Register

#define REG_CIDER            0xC0           // Chip ID and Enable Register
#define REG_CGCR             0xC6           // Chip Global Control Register
#define REG_IACR             0xC8           // Indirect Access Control Register

#define REG_IADLR            0xD0           // Indirect Access Data Low Register
#define REG_IADHR            0xD2           // Indirect Access Data High Register
#define REG_PMECR            0xD4           // Power Management Event Control Register
#define REG_GSWUTR           0xD6           // Go-Sleep & Wake-Up Timer Register
#define REG_PHYRR            0xD8           // PHY Reset Register

#define REG_P1MBCR           0xE4           // PHY 1 MII-Register Basic Control Register
#define REG_P1MBSR           0xE6           // PHY 1 MII-Register Basic Status Register
#define REG_PHY1ILR          0xE8           // PHY 1 PHY ID Low Register
#define REG_PHY1IHR          0xEA           // PHY 1 PHY ID High Register
#define REG_P1ANAR           0xEC           // PHY 1 Auto-Negotiation Advertisement Register
#define REG_P1ANLPR          0xEE           // PHY 1 Auto-Negotiation Ling Partner Ability Register

#define REG_P1SCLMD          0xF4           // Port 1 PHY Special Control/Status, LinkMD
#define REG_P1CR             0xF6           // Port 1 Control Register
#define REG_P1SR             0xF8           // Port 1 Status Register

/* Global Reset Register */
#define REG_GRR_GSR          (1 <<  0)      // Global Soft Reset
#define REG_GRR_QMSR         (1 <<  1)      // QMU Module Soft Reset

/* Transmit Control Register */
#define REG_TXCR_TXE         (1 <<  0)      // Transmit Enable
#define REG_TXCR_TXCE        (1 <<  1)      // Transmit CRC Enable
#define REG_TXCR_TXPE        (1 <<  2)      // Transmit Padding Enable
#define REG_TXCR_TXFCE       (1 <<  3)      // Transmit Flow Control Enable
#define REG_TXCR_FTXQ        (1 <<  4)      // Flush Transmit Queue
#define REG_TXCR_TCGIP       (1 <<  5)      // Transmit Checksum Generation for IP
#define REG_TXCR_TCGTCP      (1 <<  6)      // Transmit Checksum Generation for TCP
#define REG_TXCR_TCGUDP      (1 <<  7)      // Transmit Checksum Generation for UDP
#define REG_TXCR_TCGICMP     (1 <<  8)      // Transmit Checksum Generation for ICMP

/* Receive Control Register 1 */
#define REG_RXCR1_RXE        (1 <<  0)      // Receive Enable
#define REG_RXCR1_RXINVF     (1 <<  1)      // Receive Inverse Filtering
#define REG_RXCR1_RXAE       (1 <<  4)      // Receive All Enable
#define REG_RXCR1_RXUE       (1 <<  5)      // Receive Unicast Enable
#define REG_RXCR1_RXME       (1 <<  6)      // Receive Multicast Enable
#define REG_RXCR1_RXBE       (1 <<  7)      // Receive Broadcast Enable
#define REG_RXCR1_RXMAFMA    (1 <<  8)      // Receive Multicast Address Filtering with MAC Address Enable
#define REG_RXCR1_RXEFE      (1 <<  9)      // Receive Error Frame Enable
#define REG_RXCR1_RXFCE      (1 << 10)      // Receive Flow Control Enable
#define REG_RXCR1_RXPAFMA    (1 << 11)      // Receive Physical Address Filtering with MAC Address Enable
#define REG_RXCR1_RXIPFCC    (1 << 12)      // Receive IP Frame Checksum Check Enable
#define REG_RXCR1_RXTCPFCC   (1 << 13)      // Receive TCP Frame Checksum Check Enable
#define REG_RXCR1_RXUDPFCC   (1 << 14)      // Receive UDP Frame Checksum Check Enable
#define REG_RXCR1_FRXQ       (1 << 15)      // Flush Receive Queue

/* Receive Control Register 2 */
#define REG_RXCR2_RXSAF      (1 <<  0)      // Receive Source Address Filtering
#define REG_RXCR2_RXICMPFCC  (1 <<  1)      // Receive ICMP Frame Checksum Check Enable
#define REG_RXCR2_UDPLFE     (1 <<  2)      // UDP Lite Frame Enable
#define REG_RXCR2_RXIUFCEZ   (1 <<  3)      // Receive IPV4/IPV6/UDP Frame Checksum Equal Zero
#define REG_RXCR2_IUFFP      (1 <<  4)      // IPV4/IPV6/UDP Fragment Frame Pass
#define REG_RXCR2_SRDBL_4    (0 <<  5)      // 4 bytes data burst
#define REG_RXCR2_SRDBL_8    (1 <<  5)      // 8 bytes data burst
#define REG_RXCR2_SRDBL_16   (2 <<  5)      // 16 bytes data burst
#define REG_RXCR2_SRDBL_32   (3 <<  5)      // 32 bytes data burst
#define REG_RXCR2_SRDBL_FRM  (4 <<  5)      // Single frame data burst

/* TXQ Command Register */
#define REG_TXQCR_METFE      (1 <<  0)      // Manual Enqueue TXQ Frame Enable
#define REG_TXQCR_TXQMAM     (1 <<  1)      // TXQ Memory Available Monitor
#define REG_TXQCR_AETFE      (1 <<  2)      // Auto-Enqueue TXQ Frame Enable

/* RXQ Command Register */
#define REG_RXQCR_RRXEF      (1 <<  0)      // Release RX Error Frame
#define REG_RXQCR_SDA        (1 <<  3)      // Start DMA Access
#define REG_RXQCR_ADRFE      (1 <<  4)      // Auto-Dequeue RXQ Frame Enable
#define REG_RXQCR_RXFCTE     (1 <<  5)      // RX Frame Count Threshold Enable
#define REG_RXQCR_RXDBCTE    (1 <<  6)      // RX Data Byte Count Threshold Enable
#define REG_RXQCR_RXDTTE     (1 <<  7)      // RX Duration Timer Threshold Enable
#define REG_RXQCR_RXIPHTOE   (1 <<  9)      // RX IP Header Two-Byte Offset Enable
#define REG_RXQCR_RXFCTS     (1 << 10)      // RX Frame Count Threshold Status
#define REG_RXQCR_RXDBCTS    (1 << 11)      // RX Data Byte Count Threshold Status
#define REG_RXQCR_RXDTTS     (1 << 12)      // RX Duration Timer Threshold Status

/* TX Frame Data Pointer Register */
#define REG_TXFDPR_TXFPAI    (1 << 14)      // TX Frame Data Pointer Auto Increment

/* Receive Frame Header Status Register */
#define REG_RXFHSR_RXCE      (1 <<  0)      // Receive CRC Error
#define REG_RXFHSR_RXRF      (1 <<  1)      // Receive Runt Frame
#define REG_RXFHSR_RXFTL     (1 <<  2)      // Receive Frame Too Long
#define REG_RXFHSR_RXFT      (1 <<  3)      // Receive Frame Type
#define REG_RXFHSR_RXMR      (1 <<  4)      // Receive MII Error
#define REG_RXFHSR_RXUF      (1 <<  5)      // Receive Unicast Frame
#define REG_RXFHSR_RXMF      (1 <<  6)      // Receive Multicast Frame
#define REG_RXFHSR_RXBF      (1 <<  7)      // Receive Broadcast Frame
#define REG_RXFHSR_RXUDPFCS  (1 << 10)      // Receive UDP Frame Checksum Status
#define REG_RXFHSR_RXTCPFCS  (1 << 11)      // Receive TCP Frame Checksum Status
#define REG_RXFHSR_RXIPFCS   (1 << 12)      // Receive IP Frame Checksum Status
#define REG_RXFHSR_RXICMPFCS (1 << 13)      // Receive ICMP Frame Checksum Status
#define REG_RXFHSR_RXFV      (1 << 15)      // Receive Frame Valid

/* RX Frame Data Pointer Register */
#define REG_RXFDPR_RXFPAI    (1 << 14)      // RX Frame Pointer Auto Increment

/* Interrupt Enable Register */
#define REG_IER_DEDIE        (1 <<  0)      // Delay Energy Detect Interrupt Enable
#define REG_IER_SPIBEIE      (1 <<  1)      // SPI Bus Error Interrupt Enable
#define REG_IER_EDIE         (1 <<  2)      // Energy Detect Interrupt Enable
#define REG_IER_LDIE         (1 <<  3)      // Linkup Detect Interrupt Enable
#define REG_IER_RXMPDIE      (1 <<  4)      // Receive Magic Packet Detect Interrupt Enable
#define REG_IER_RXWFDIE      (1 <<  5)      // Receive Wake-up Frame Detect Interrupt Enable
#define REG_IER_TXSAIE       (1 <<  6)      // Transmit Space Available Interrupt Enable
#define REG_IER_RXPSIE       (1 <<  8)      // Receive Process Stopped Interrupt Enable
#define REG_IER_TXPSIE       (1 <<  9)      // Transmit Process Stopped Interrupt Enable
#define REG_IER_RXOIE        (1 << 11)      // Receive Overrun Interrupt Enable
#define REG_IER_RXIE         (1 << 13)      // Receive Interrupt Enable
#define REG_IER_TXIE         (1 << 14)      // Transmit Interrupt Enable
#define REG_IER_LCIE         (1 << 15)      // Link Change Interrupt Enable

/* Interrupt Status Register */
#define REG_ISR_SPIBEIS      (1 <<  1)      // SPI Bus Error Interrupt Status
#define REG_ISR_EDIS         (1 <<  2)      // Energy Detect Interrupt Status
#define REG_ISR_LDIS         (1 <<  3)      // Linkup Detect Interrupt Status
#define REG_ISR_RXMPDIS      (1 <<  4)      // Receive Magic Packet Detect Interrupt Status
#define REG_ISR_RXWFDIS      (1 <<  5)      // Receive Wakeup Frame Detect Interrupt Status
#define REG_ISR_TXSAIS       (1 <<  6)      // Transmit Space Available Interrupt Status
#define REG_ISR_RXPSIS       (1 <<  8)      // Receive Process Stopped Interrupt Status
#define REG_ISR_TXPSIS       (1 <<  9)      // Transmit Process Stopped Interrupt Status
#define REG_ISR_RXOIS        (1 << 11)      // Receive Overrun Interrupt Status
#define REG_ISR_RXIS         (1 << 13)      // Receive Interrupt Status
#define REG_ISR_TXIS         (1 << 14)      // Transmit Interrupt Status
#define REG_ISR_LCIS         (1 << 15)      // Link Change Interrupt Status

/* Power Management Event Control Register */
#define REG_PMECR_PMASK      (0x3<< 0)      // Power Management Mode mask
#define REG_PMECR_PNORMAL     (0 << 0)      // - Normal Operation Mode
#define REG_PMECR_ENDET       (1 << 0)      // - Energy Detect Mode
#define REG_PMECR_SOFTPD      (2 << 0)      // - Soft Power Down Mode
#define REG_PMECR_PSAVE       (3 << 0)      // - Power Saving Mode
#define REG_PMECR_SEDI       (1 <<  2)      // Signal Energy Detected event Indication
#define REG_PMECR_LCUPI      (1 <<  3)      // Link Change to up event Indication
#define REG_PMECR_RXMPI      (1 <<  4)      // Received Magic packet event Indication
#define REG_PMECR_RXWUI      (1 <<  5)      // Received Wake-up frame event Indication
#define REG_PMECR_WUNOM      (1 <<  6)      // Wake-up to Normal Operation Mode
#define REG_PMECR_AWUE       (1 <<  7)      // Auto Wake-up Enable
#define REG_PMECR_SEDE       (1 <<  8)      // Assert on Signal energy detected enable
#define REG_PMECR_LCUPE      (1 <<  9)      // Assert on Link change to up enable
#define REG_PMECR_RXMPE      (1 << 10)      // Assert on received Magic packet enable
#define REG_PMECR_RXWUE      (1 << 11)      // Assert on received wake-up frame enable
#define REG_PMECR_OPOL       (1 << 12)      // Output polarity
#define REG_PMECR_ODEN       (1 << 14)      // Output delay enable

/* PHY 1 MII-Register Basic Control Register */
#define REG_P1MBCR_DISLED    (1 <<  0)      // Disable LED
#define REG_P1MBCR_DISTX     (1 <<  1)      // Disable Transmit
#define REG_P1MBCR_DISMDIX   (1 <<  3)      // Disable MDI-X
#define REG_P1MBCR_FORCEMDIX (1 <<  4)      // Force MDI-X
#define REG_P1MBCR_HPMDIX    (1 <<  5)      // HP_mdix (Auto MDI-X mode)
#define REG_P1MBCR_FORCEFD   (1 <<  8)      // Force Full Duplex
#define REG_P1MBCR_RESAN     (1 <<  9)      // Restart AN
#define REG_P1MBCR_ANEN      (1 << 12)      // AN Enable
#define REG_P1MBCR_FORCE100  (1 << 13)      // Force 100
#define REG_P1MBCR_LLB       (1 << 14)      // Local (far-end) loopback

/* Port 1 Special Control/Status Register */
#define REG_P1SCLMD_VCTRES   (0x3<<13)      // VCT test result
#define REG_P1SCLMD_VCTEN    (1 << 12)      // VCT test enable
#define REG_P1SCLMD_FORCELNK (1 << 11)      // Force link
#define REG_P1SCLMD_RLB      (1 <<  9)      // Remote (near-end) loopback
#define REG_P1SCLMD_VCTFCNT  (0x1FF<<0)     // VCT fault count

/* Port 1 Control Register */
#define REG_P1CR_ADV10BTHD   (1 <<  0)      // Advertised 10BT half-duplex capability
#define REG_P1CR_ADV10BTFD   (1 <<  1)      // Advertised 10BT full-duplex capability
#define REG_P1CR_ADV100BTHD  (1 <<  2)      // Advertised 100BT half-duplex capability
#define REG_P1CR_ADV100BTFD  (1 <<  3)      // Advertised 100BT full-duplex capability
#define REG_P1CR_ADVFLOWCTRL (1 <<  4)      // Advertised flow control capability
#define REG_P1CR_FORCEDUPLEX (1 <<  5)      // Force (full) Duplex
#define REG_P1CR_FORCESPEED  (1 <<  6)      // Force (100BT) Speed
#define REG_P1CR_AUTONEGEN   (1 <<  7)      // Auto Negotiation Enable
#define REG_P1CR_FORCEMDIX   (1 <<  9)      // Force MDI-X
#define REG_P1CR_DISAUTOMDIX (1 << 10)      // Disable auto MDI/MDI-X
#define REG_P1CR_RESTARTAN   (1 << 13)      // Restart AN
#define REG_P1CR_TXIDS       (1 << 14)      // Txids (disable port's transmitter
#define REG_P1CR_LEDOFF      (1 << 15)      // LED Off

/* Port 1 Status Register */
#define REG_P1SR_PAR10BTHD   (1 <<  0)      // Partner 10BT half-duplex capability
#define REG_P1SR_PAR10BTFD   (1 <<  1)      // Partner 10BT full-duplex capability
#define REG_P1SR_PAR100BTHD  (1 <<  2)      // Partner 100BT half-duplex capability
#define REG_P1SR_PAR100BTFD  (1 <<  3)      // Partner 100BT full-duplex capability
#define REG_P1SR_PARFLOWCTRL (1 <<  4)      // Partner flow control capability
#define REG_P1SR_LINKGOOD    (1 <<  5)      // Link Good
#define REG_P1SR_ANDONE      (1 <<  6)      // AN Done
#define REG_P1SR_MDIXSTATUS  (1 <<  7)      // MDI-X Status
#define REG_P1SR_OPDUPLEX    (1 <<  9)      // Operation (full) Duplex
#define REG_P1SR_OPSPEED     (1 << 10)      // Operation (100Mbps) Speed
#define REG_P1SR_POLREV      (1 << 13)      // Polarity Reverse
#define REG_P1SR_HPMDIX      (1 << 15)      // HP_mdix (HP Auto MDI-X mode)

/* DMA Tx Header */
#define DMA_HEADER_TXIC      (1 << 15)      // Tx Interrupt on Completion

/* ETH Driver Control Information */
typedef struct {
  uint8_t  flags;                           // Control and state flags
  uint8_t  rx_count;                        // Receive descriptor index
  uint8_t  tx_id;                           // Transmit frame id
  uint16_t tx_len;                          // Transmit length of data
  uint16_t txcr;                            // TXCR shadow register
  uint16_t rxcr1;                           // RXCR1 shadow register
  uint16_t rxcr2;                           // RXCR2 shadow register
  uint16_t rxqcr;                           // RXQCR shadow register
} ETH_CTRL;

#endif /* __ETH_KSZ8851SNL */
