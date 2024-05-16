/* -----------------------------------------------------------------------------
 * Copyright (c) 2024 Arm Limited (or its affiliates).
 * All rights reserved.
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
 *
 * $Date:        16. May 2024
 * $Revision:    V1.0
 *
 * Project:      Config file for USB Host EHCI Controller Driver
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef USBH_EHCI_CONFIG_H_
#define USBH_EHCI_CONFIG_H_

// <h>USB Host Controller 0
//   <o>Driver number <0-3>
//     <i>Specifies the exported Driver_USBH# instance
//     <i>For example for value 0 the exported driver will be Driver_USBH0
#define USBH0_EHCI_DRV_NUM              0

//   <o>EHCI registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which EHCI controller registers are located
#define USBH0_EHCI_BASE_ADDR            0x402E0100

//   <e>Relocated EHCI communication area
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script).
#define USBH0_EHCI_COM_AREA_RELOC       0

//     <s.64>EHCI communication area section name
//       <i>Specifies the section name of the EHCI communication area
#define USBH0_EHCI_COM_AREA_SECTION_NAME ".driver.usbh0.ehci_com_area"
//   </e>
// </h>

// <e>USB Host Controller 1
//   <i>Enables or disables USB Host Driver 1
#define USBH1_EHCI_ENABLED              0

//   <o>Driver number <0-3>
//     <i>Specifies the exported Driver_USBH# instance
//     <i>For example for value 1 the exported driver will be Driver_USBH1
#define USBH1_EHCI_DRV_NUM              1

//   <o>EHCI Host Controller registers base address <0-0xFFFFFFFF>
//     <i>Specifies the absolute address at which EHCI controller registers are located
#define USBH1_EHCI_BASE_ADDR            0x402E0300

//   <e>Relocated EHCI communication area
//     <i>Specifies if the communication area is located at a specific address
//     <i>(via the linker script).
#define USBH1_EHCI_COM_AREA_RELOC       0

//     <s.64>EHCI communication area section name
//       <i>Specifies the section name of the EHCI communication area
#define USBH1_EHCI_COM_AREA_SECTION_NAME ".driver.usbh1.ehci_com_area"
//   </e>
// </e>

// <o>Maximum number of pipes (per controller) <1-15>
#define USBH_EHCI_MAX_PIPES             4

#endif /* USBH_EHCI_CONFIG_H_ */
