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
 * $Date:       28. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host OHCI Controller Driver header
 *
 * -----------------------------------------------------------------------------
 */

#ifndef USBH_OHCI_H_
#define USBH_OHCI_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_USBH.h"
#include "USBH_OHCI_Config.h"

// Configuration macros *******************************************************

#define USBHn_DRIVER_(n)              Driver_USBH##n
#define USBHn_DRIVER(n)               USBHn_DRIVER_(n)

// Global driver structures ***************************************************

extern  ARM_DRIVER_USBH USBHn_DRIVER(USBH0_OHCI_DRV_NUM);

#if    (USBH1_OHCI_ENABLED == 1)
extern  ARM_DRIVER_USBH USBHn_DRIVER(USBH1_OHCI_DRV_NUM);
#endif

#ifdef  __cplusplus
}
#endif

#endif /* USBH_OHCI_H_ */
