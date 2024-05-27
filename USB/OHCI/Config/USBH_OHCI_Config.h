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
 * Driver:      Driver_USBH# (default: Driver_USBH0)
 * Project:     USB Host OHCI Controller Driver Configuration definitions
 *
 * -----------------------------------------------------------------------------
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef USBH_OHCI_CONFIG_H_
#define USBH_OHCI_CONFIG_H_

// <h>USB Host Controller 0
//   <o>Export control block Driver_USBH# <0-255>
//     <i>Specifies the exported driver control block number
//     <i>For example, for value 0 the exported driver control block will be Driver_USBH0
#define USBH0_OHCI_DRV_NUM              0

//   <o>OHCI Registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which OHCI controller registers are located
#define USBH0_OHCI_BASE_ADDR            0x5000C000

//   <e>Relocate OHCI Communication Area (HCCA)
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script)
#define USBH0_OHCI_HCCA_RELOC           0

//     <s.64>Section name
//       <i>Specifies the section name of the OHCI communication area
//       <i>(for positioning via the linker script)
#define USBH0_OHCI_HCCA_SECTION_NAME    ".driver.usbh0.ohci_hcca"
//   </e>
// </h>

// <e>USB Host Controller 1
//   <i>Enables or disables USB Host Controller 1 Driver
#define USBH1_OHCI_ENABLED              0

//   <o>Export control block Driver_USBH# <0-255>
//     <i>Specifies the exported driver control block number
//     <i>For example, for value 1 the exported driver control block will be Driver_USBH1
#define USBH1_OHCI_DRV_NUM              1

//   <o>OHCI Registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which OHCI controller registers are located
#define USBH1_OHCI_BASE_ADDR            0x00000000

//   <e>Relocate OHCI Communication Area (HCCA)
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script)
#define USBH1_OHCI_HCCA_RELOC           0

//     <s.64>Section name
//       <i>Specifies the section name of the OHCI communication area
//       <i>(for positioning via the linker script)
#define USBH1_OHCI_HCCA_SECTION_NAME    ".driver.usbh1.ohci_hcca"
//   </e>
// </e>

// <o>Maximum number of Pipes <1-15>
//   <i>Specifies the maximum number of pipes that the driver will support (per controller)
#define USBH_OHCI_MAX_PIPES             4

#endif /* USBH_OHCI_CONFIG_H_ */
