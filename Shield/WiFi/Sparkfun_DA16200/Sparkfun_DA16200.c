/*---------------------------------------------------------------------------
 * Copyright (c) 2023 Arm Limited (or its affiliates).
 * All rights reserved.
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
 *---------------------------------------------------------------------------*/

#include CMSIS_target_header

// Shield Setup
int32_t shield_setup (void) {
  // Setup pins (DA16200 driver is not calling WiFi_DA16200_Pin_Initialize)
  Driver_GPIO0.Setup(ARDUINO_UNO_D4, NULL);  // RTC_PWR_KEY
  Driver_GPIO0.SetDirection(ARDUINO_UNO_D4, ARM_GPIO_OUTPUT);
  return 0;
}
