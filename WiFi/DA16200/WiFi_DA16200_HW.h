/* -----------------------------------------------------------------------------
 * Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        13. June 2022
 * $Revision:    V1.0
 *
 * Project:      WiFi Driver Hardware specific header file for
 *               Renesas DA16200 WiFi Module
 * -------------------------------------------------------------------------- */

#include <stdint.h>

extern void WiFi_DA16200_Pin_Initialize   (void);
extern void WiFi_DA16200_Pin_Uninitialize (void);
extern void WiFi_DA16200_Pin_RTC_PWR_KEY  (uint8_t state);
extern void WiFi_DA16200_Pin_RTC_WAKE_UP1 (uint8_t state);
