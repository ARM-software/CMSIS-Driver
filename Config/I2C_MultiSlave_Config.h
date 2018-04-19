/*
 * Copyright (c) 2018 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------
 *
 * $Date:        20. February 2018
 * $Revision:    V1.0.0
 *
 * Driver:       Driver_I2C# (default: Driver_I2C0)
 * Project:      I2C Master to Multi-Slave Wrapper Configuration
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_I2C# = n (default: 0)
 * -------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------

// <h>I2C Multi-Slave Driver Configuration

//   <o>Connect to hardware via Driver_I2C# <0-255>
//   <i>Select driver control block for hardware interface
#define I2C_DRIVER              0

//   <e0>Slave Device 0
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_0      1
#define I2C_DRIVER_SLAVE_0      10

//   <e0>Slave Device 1
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_1      1
#define I2C_DRIVER_SLAVE_1      11

//   <e0>Slave Device 2
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_2      0
#define I2C_DRIVER_SLAVE_2      12

//   <e0>Slave Device 3
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_3      0
#define I2C_DRIVER_SLAVE_3      13

//   <e0>Slave Device 4
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_4      0
#define I2C_DRIVER_SLAVE_4      14

//   <e0>Slave Device 5
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_5      0
#define I2C_DRIVER_SLAVE_5      15

//   <e0>Slave Device 6
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_6      0
#define I2C_DRIVER_SLAVE_6      16

//   <e0>Slave Device 7
//   <i>Enable or disable slave device
//     <o1>Export control block Driver_I2C# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define I2C_ENABLE_SLAVE_7      0
#define I2C_DRIVER_SLAVE_7      17

// </h>

//------------- <<< end of configuration section >>> -------------------------
