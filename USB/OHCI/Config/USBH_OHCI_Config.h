/******************************************************************************
 * @file     USBH_OHCI_Config.h
 * @brief    Config file for USB Host OHCI Controller Driver
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

#ifndef USBH_OHCI_CONFIG_H_
#define USBH_OHCI_CONFIG_H_

// <h>USBH0 Driver
//   <o>HCI Driver Number <0-2>
//     <i>Specifies instance of the Host Controller Interface (HCI) sub-driver
#define USBH0_OHCI_HCI_DRV_NUM          0

//   <o>OHCI registers base address <0-0xFFFFFFFF>
//     <i>Specifies absolute address at which OHCI controller registers are located
#define USBH0_OHCI_BASE_ADDR            0x5000C000
// </h>

// Configuration not using Configuration Wizard (should be edited manually if necessary)

// USBH1 Driver configuration
#define USBH1_OHCI_ENABLED              0
#define USBH1_OHCI_HCI_DRV_NUM          1
#define USBH1_OHCI_BASE_ADDR            0x00000000

// Maximum number of pipes supported by this driver (default: 4)
#define USBH_OHCI_MAX_PIPES            (4)

#endif /* USBH_OHCI_CONFIG_H_ */
