/******************************************************************************
 * @file     USBH_EHCI_TT.h
 * @brief    USB Host EHCI Controller Driver header
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

/* History:
 *  Version 1.0
 *    Initial release
 */

#ifndef USBH_EHCI_TT_H_
#define USBH_EHCI_TT_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_USBH.h"
#include "USBH_EHCI_TT_Config.h"

// Global driver structures ***************************************************

extern    ARM_DRIVER_USBH Driver_USBH0;

#if      (USBH1_EHCI_TT_ENABLED == 1)
extern    ARM_DRIVER_USBH Driver_USBH1;
#endif

#ifdef  __cplusplus
}
#endif

#endif /* USBH_EHCI_TT_H_ */
