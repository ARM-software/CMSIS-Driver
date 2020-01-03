/* -----------------------------------------------------------------------------
 * Copyright (c) 2019 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        3. January 2020
 * $Revision:    V1.1
 *
 * Project:      Simple serial buffer
 * -------------------------------------------------------------------------- */

#ifndef ESP8266_SERIAL_H__
#define ESP8266_SERIAL_H__

#include <stdint.h>

/* Callback events */
#define SERIAL_CB_RX_DATA_AVAILABLE    1U
#define SERIAL_CB_TX_DATA_COMPLETED    2U
#define SERIAL_CB_RX_ERROR             4U
#define SERIAL_CB_TX_ERROR             8U

int32_t  Serial_Initialize (void);
int32_t  Serial_Uninitialize (void);
int32_t  Serial_SetBaudrate (uint32_t baudrate);
int32_t  Serial_SendBuf (const uint8_t *buf, uint32_t len);
int32_t  Serial_ReadBuf(uint8_t *buf, uint32_t len);
uint32_t Serial_GetRxCount(void);
uint32_t Serial_GetTxCount(void);
uint32_t Serial_GetTxFree (void);

void Serial_Cb (uint32_t cb_event);

#endif /* ESP8266_SERIAL_H__ */
