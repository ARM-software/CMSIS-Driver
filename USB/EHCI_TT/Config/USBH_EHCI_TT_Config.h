/******************************************************************************
 * @file     USBH_EHCI_TT_Config.h
 * @brief    Config file for USB Host EHCI Controller Driver
 *           for customized EHCI with internal Transaction Translator (TT)
 *           (with full/low speed support)
 * @version  V1.0
 * @date     8. May 2024
 ******************************************************************************/
/*
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
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef USBH_EHCI_TT_CONFIG_H_
#define USBH_EHCI_TT_CONFIG_H_

// <h>USBH0 Driver
//   <o>HCI Driver Number <0-2>
//     <i>Specifies instance of the Host Controller Interface (HCI) sub-driver
#define USBH0_EHCI_TT_HCI_DRV_NUM       0

//   <o>EHCI registers base address <0-0xFFFFFFFF>
//     <i>Specifies absolute address at which EHCI controller registers are located
#define USBH0_EHCI_TT_BASE_ADDR         0x40006100
// </h>

// <e>USBH1 Driver
//   <i>Enables or disables USBH1 Driver (Driver_USBH1)
#define USBH1_EHCI_TT_ENABLED           1

//   <o>HCI Driver Number <0-2>
//     <i>Specifies instance of the Host Controller Interface (HCI) sub-driver
#define USBH1_EHCI_TT_HCI_DRV_NUM       1

//   <o>EHCI Host Controller registers base address <0-0xFFFFFFFF>
//     <i>Specifies absolute address at which EHCI controller registers are located
#define USBH1_EHCI_TT_BASE_ADDR         0x40007100
// </e>

// Configuration not using Configuration Wizard (should be edited manually if necessary)

// Maximum number of pipes supported by this driver (default: 4)
#define USBH_EHCI_TT_MAX_PIPES         (4)

#endif /* USBH_EHCI_TT_CONFIG_H_ */
