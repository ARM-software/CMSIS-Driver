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
 * $Date:        12. November 2019
 * $Revision:    V1.0
 *
 * Project:      ESP8266 WiFi Driver
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>ESP8266 WiFi Driver Configuration

// <o> WiFi Driver Number (Driver_WiFi#) <0-255>
// <i> Defines exported WiFi driver control block number (Driver_WiFi#)
// <i> Default: 0
#define WIFI_ESP8266_DRIVER_NUMBER          0

// <o>Connect to hardware via Driver_USART# <0-255>
// <i>Defines the serial driver control block number (Driver_USART#)
// <i> Default: 0
#define WIFI_ESP8266_SERIAL_DRIVER          0

// <o> Serial interface baudrate <115200=>115200
//                               <230400=>230400
//                               <460800=>460800
//                               <921600=>921600
// <i> Defines the serial interface baudrate.
// <i> Default: 115200
#define WIFI_ESP8266_SERIAL_BAUDRATE        115200

// <o> WiFi thread priority <0=>osPriorityLow
//                          <1=>osPriorityBelowNormal
//                          <2=>osPriorityNormal
//                          <3=>osPriorityAboveNormal
//                          <4=>osPriorityHigh
//                          <5=>osPriorityRealtime
// <i> Defines the WiFi driver thread priority.
// <i> The priority of the WiFi thread should be higher as application thread priority.
// <i> Default: 3
#define WIFI_ESP8266_THREAD_PRIORITY        3

// <o> WiFi thread stack size [bytes] <96-1073741824:8>
// <i> Defines stack size for the WiFi Thread.
// <i> Default: 512
#define WIFI_ESP8266_THREAD_STACK_SIZE      512

// <o> Socket buffer block size <128-16384:128>
// <i> Defines the size of one memory block used for socket data buffering.
// <i> Socket buffering consists of multiple blocks which are distributed across multiple sockets.
// <i> Default: 512
#define WIFI_ESP8266_SOCKET_BLOCK_SIZE      512

// <o> Socket buffer block count <5-256>
// <i> Defines the total number of memory blocks used for socket data buffering.
// <i> Socket buffering consists of multiple blocks which are distributed across multiple sockets.
// <i> Default: 8
#define WIFI_ESP8266_SOCKET_BLOCK_COUNT     8

// <o> Serial parser buffer block size
// <i> Defines the size of one memory block in serial parser buffer.
// <i> The total size of serial parser buffer is defined by memory block size and number of blocks.
// <i> Default: 256
#define WIFI_ESP8266_PARSER_BLOCK_SIZE      256

// <o> Serial parser buffer block count
// <i> Defines the number of memory blocks in serial parser buffer.
// <i> The total size of serial parser buffer is defined by memory block size and number of blocks.
// <i> Default: 8
#define WIFI_ESP8266_PARSER_BLOCK_COUNT     8

// </h>

//------------- <<< end of configuration section >>> -------------------------
