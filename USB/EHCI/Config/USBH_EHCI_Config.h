/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------------
 *
 * $Date:       27. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host EHCI Controller Driver Configuration definitions
 *
 * -----------------------------------------------------------------------------
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef USBH_EHCI_CONFIG_H_
#define USBH_EHCI_CONFIG_H_

// <h>USB Host Controller 0
//   <o>Export control block Driver_USBH# <0-255>
//     <i>Specifies the exported driver control block number
//     <i>For example, for value 0 the exported driver control block will be Driver_USBH0
#define USBH0_EHCI_DRV_NUM              0

//   <o>EHCI Registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which EHCI controller registers are located
#define USBH0_EHCI_BASE_ADDR            0x402E0100

//   <e>Relocate EHCI Communication Area
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script)
#define USBH0_EHCI_COM_AREA_RELOC       0

//     <s.64>Section name
//       <i>Specifies the section name of the EHCI communication area
//       <i>(for positioning via the linker script)
#define USBH0_EHCI_COM_AREA_SECTION_NAME ".driver.usbh0.ehci_com_area"
//   </e>
// </h>

// <e>USB Host Controller 1
//   <i>Enables or disables USB Host Controller 1 Driver
#define USBH1_EHCI_ENABLED              0

//   <o>Export control block Driver_USBH# <0-255>
//     <i>Specifies the exported driver control block number
//     <i>For example, for value 1 the exported driver control block will be Driver_USBH1
#define USBH1_EHCI_DRV_NUM              1

//   <o>EHCI Registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which EHCI controller registers are located
#define USBH1_EHCI_BASE_ADDR            0x402E0300

//   <e>Relocate EHCI Communication Area
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script)
#define USBH1_EHCI_COM_AREA_RELOC       0

//     <s.64>Section name
//       <i>Specifies the section name of the EHCI communication area
//       <i>(for positioning via the linker script)
#define USBH1_EHCI_COM_AREA_SECTION_NAME ".driver.usbh1.ehci_com_area"
//   </e>
// </e>

// <o>Maximum number of Pipes <1-15>
//   <i>Specifies the maximum number of pipes that the driver will support (per controller)
#define USBH_EHCI_MAX_PIPES             4

#endif /* USBH_EHCI_CONFIG_H_ */
