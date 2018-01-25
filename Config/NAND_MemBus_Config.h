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
 * $Date:        17. April 2014
 * $Revision:    V1.00
 *  
 * Driver:       Driver_NAND# (default: Driver_NAND0)
 * Project:      NAND Flash Device connected to Memory Bus Configuration
 * -------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>NAND Driver Configuration

//   <o>Connect to hardware via Driver_NAND# <0-255>
//   <i>Select driver control block for hardware interface
#define NAND_DRIVER             0

//   <e>NAND Device 0
//     <h>Memory Map
//       <o1>Base Address
//       <o2>ALE Address
//       <o3>CLE Address
//     </h>
//     <o4>Data Bus Width <8=>8-bit <16=>16-bit
//     <e5>Ready/Busy Pin
//     <i>Select to enable driver rb_monitor capability.
//     <i>Implement Driver_NANDx_GetDeviceBusy function which returns the Ready/Busy pin state.
//       <q6>Rising Edge Interrupt
//       <i> Select to enable driver event_device_ready capability.
//       <i> Call Driver_NANDx_Event_DeviceReady function on Ready/Busy pin rising edge.
//     </e5>
//   </e>
#define NAND_DEV0               1
#define NAND_DEV0_ADDR_BASE     0x80000000
#define NAND_DEV0_ADDR_ALE      0x80010000
#define NAND_DEV0_ADDR_CLE      0x80020000
#define NAND_DEV0_DATA_WIDTH    8
#define NAND_DEV0_RB_PIN        1
#define NAND_DEV0_RB_PIN_IRQ    1

//   <e>NAND Device 1
//     <h>Memory Map
//       <o1>Base Address
//       <o2>ALE Address
//       <o3>CLE Address
//     </h>
//     <o4>Data Bus Width <8=>8-bit <16=>16-bit
//     <e5>Ready/Busy Pin
//     <i>Select to enable driver rb_monitor capability.
//     <i>Implement Driver_NANDx_GetDeviceBusy function which returns the Ready/Busy pin state.
//       <q6>Rising Edge Interrupt
//       <i> Select to enable driver event_device_ready capability.
//       <i> Call Driver_NANDx_Event_DeviceReady function on Ready/Busy pin rising edge.
//     </e5>
//   </e>
#define NAND_DEV1               0
#define NAND_DEV1_ADDR_BASE     0x81000000
#define NAND_DEV1_ADDR_ALE      0x81010000
#define NAND_DEV1_ADDR_CLE      0x81020000
#define NAND_DEV1_DATA_WIDTH    8
#define NAND_DEV1_RB_PIN        0
#define NAND_DEV1_RB_PIN_IRQ    0

//   <e>NAND Device 2
//     <h>Memory Map
//       <o1>Base Address
//       <o2>ALE Address
//       <o3>CLE Address
//     </h>
//     <o4>Data Bus Width <8=>8-bit <16=>16-bit
//     <e5>Ready/Busy Pin
//     <i>Select to enable driver rb_monitor capability.
//     <i>Implement Driver_NANDx_GetDeviceBusy function which returns the Ready/Busy pin state.
//       <q6>Rising Edge Interrupt
//       <i> Select to enable driver event_device_ready capability.
//       <i> Call Driver_NANDx_Event_DeviceReady function on Ready/Busy pin rising edge.
//     </e5>
//   </e>
#define NAND_DEV2               0
#define NAND_DEV2_ADDR_BASE     0x82000000
#define NAND_DEV2_ADDR_ALE      0x82010000
#define NAND_DEV2_ADDR_CLE      0x82020000
#define NAND_DEV2_DATA_WIDTH    8
#define NAND_DEV2_RB_PIN        0
#define NAND_DEV2_RB_PIN_IRQ    0

//   <e>NAND Device 3
//     <h>Memory Map
//       <o1>Base Address
//       <o2>ALE Address
//       <o3>CLE Address
//     </h>
//     <o4>Data Bus Width <8=>8-bit <16=>16-bit
//     <e5>Ready/Busy Pin
//     <i>Select to enable driver rb_monitor capability.
//     <i>Implement Driver_NANDx_GetDeviceBusy function which returns the Ready/Busy pin state.
//       <q6>Rising Edge Interrupt
//       <i> Select to enable driver event_device_ready capability.
//       <i> Call Driver_NANDx_Event_DeviceReady function on Ready/Busy pin rising edge.
//     </e5>
//   </e>
#define NAND_DEV3               0
#define NAND_DEV3_ADDR_BASE     0x83000000
#define NAND_DEV3_ADDR_ALE      0x83010000
#define NAND_DEV3_ADDR_CLE      0x83020000
#define NAND_DEV3_DATA_WIDTH    8
#define NAND_DEV3_RB_PIN        0
#define NAND_DEV3_RB_PIN_IRQ    0

// </h>
