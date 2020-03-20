/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2020 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        11. February 2020
 *
 * Project:      Simple serial buffer
 * -------------------------------------------------------------------------- */

#ifndef WIZFI360_SERIAL_H__
#define WIZFI360_SERIAL_H__

#include <stdint.h>

/* Callback events */
#define SERIAL_CB_RX_DATA_AVAILABLE    1U
#define SERIAL_CB_TX_DATA_COMPLETED    2U
#define SERIAL_CB_RX_ERROR             4U
#define SERIAL_CB_TX_ERROR             8U

/* Serial interface mode */
typedef struct {
  uint32_t baudrate;      /* Configured baud rate */
  uint8_t  databits;      /* 5:5-bit data, 6:6-bit data, 7:7-bit data, 8:8-bit data */
  uint8_t  stopbits;      /* 1:1-stop bit, 2:2-stop bits */
  uint8_t  parity;        /* 0:none, 1:odd, 2:even */
  uint8_t  flow_control;  /* 0:none, 1:RTS, 2:CTS, 3:RTS/CTS */
} SERIAL_MODE;

int32_t  Serial_Initialize (void);
int32_t  Serial_Uninitialize (void);
int32_t  Serial_GetMode (SERIAL_MODE *mode);
int32_t  Serial_SetMode (SERIAL_MODE *mode);
int32_t  Serial_SendBuf (const uint8_t *buf, uint32_t len);
int32_t  Serial_ReadBuf(uint8_t *buf, uint32_t len);
uint32_t Serial_GetRxCount(void);
uint32_t Serial_GetTxCount(void);
uint32_t Serial_GetTxFree (void);

void Serial_Cb (uint32_t cb_event);

#endif /* WIZFI360_SERIAL_H__ */
