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
 * $Date:        28. April 2022
 *
 * Project:      DA16200 DPM Features
 * -------------------------------------------------------------------------- */

#ifndef DA16200_DPM_H__
#define DA16200_DPM_H__

/**
*   DPM Sleep Mode Test
*   1. define DA16x00_USE_DPM
*   2. define DA16x00_DPM_Sleep_mode_3_Test or DA16x00_DPM_Sleep_mode_2_Test
*   3. It need to DPM Test API
*       API means calling the Driver_WiFi0.PowerControl(ARM_POWER_LOW) when user process loop is done.
*/

#undef  DA16x00_USE_DPM

#define DA16x00_KEEP_ALIVE         30000           /* millisecond  0 ~ 600000 */
#define DA16x00_TIM_WAKEUP         10              /* count 1 ~ 65535 */
#define DA16x00_DPM_PERIOD         120             /* default period  for DPM Sleep 2 */

#define DA16x00_DPM_SLEEP1         1
#define DA16x00_DPM_SLEEP2         2
#define DA16x00_DPM_SLEEP3         3

#undef  DA16x00_DPM_Sleep_mode_3_Test
#undef  DA16x00_DPM_Sleep_mode_2_Test

extern char cmsis_da16x00_dpm_set_flag;

void DA16x00_DPM_Off(void);
char GetDPMStatus(void);

#endif /* DA16200_DPM_H__ */
