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
 * Project:      WiFi Driver Hardware specific implementation for 
 *               Renesas DA16200 WiFi Module
 * -------------------------------------------------------------------------- */

#include <stdint.h>

// Add device specific include files here
// <code WiFi_DA16200_include_files>

// </code>

/**
  \fn          void WiFi_DA16200_Pin_Initialize (void)
  \brief       Initialize pin(s).
  \return      none
*/
void WiFi_DA16200_Pin_Initialize (void) {
  // Add code for initializing RTC_PWR_KEY pin and RTC_WAKE_UP1 here
  // <code WiFi_DA16200_Pin_Initialize>

  // </code>
}

/**
  \fn          void WiFi_DA16200_Pin_Uninitialize (void)
  \brief       De-initialize pin(s).
  \return      none
*/
void WiFi_DA16200_Pin_Uninitialize (void) {
  // Add code for deinitializing RTC_PWR_KEY pin and RTC_WAKE_UP1 here
  // <code WiFi_DA16200_Pin_Uninitialize>

  // </code>
}

/**
  \fn          void WiFi_DA16200_Pin_RTC_PWR_KEY (uint8_t state)
  \brief       Drive RTC_PWR_KEY line.
  \param[in]   active
                 - value = 0: Drive RTC_PWR_KEY line not active state
                 - value = 1: Drive RTC_PWR_KEY line active state
  \return      none
*/
void WiFi_DA16200_Pin_RTC_PWR_KEY (uint8_t state) {
  // Add code for driving RTC_PWR_KEY pin here
  // <code WiFi_DA16200_Pin_RTC_PWR_KEY>

  // </code>
}

/**
  \fn          void WiFi_DA16200_Pin_RTC_WAKE_UP1 (uint8_t state)
  \brief       Drive RTC_WAKE_UP1 line for Wake up from DPM Sleep.
  \param[in]   active
                 - value = 0: Drive RTC_WAKE_UP1 line not active state
                 - value = 1: Drive RTC_WAKE_UP1 line active state
  \return      none
*/
void WiFi_DA16200_Pin_RTC_WAKE_UP1 (uint8_t state) {
  // Add code for driving RTC_WAKE_UP1 pin here
  // <code WiFi_DA16200_Pin_RTC_WAKE_UP1>

  // </code>
}

