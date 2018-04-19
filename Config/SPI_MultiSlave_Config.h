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
 * Driver:       Driver_SPI# (default: Driver_SPI0)
 * Project:      SPI Master to Multi-Slave Wrapper Configuration
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_SPI# = n (default: 0)
 * -------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------

// <h>SPI Multi-Slave Driver Configuration

//   <o>Connect to hardware via Driver_SPI# <0-255>
//   <i>Select driver control block for hardware interface
#define SPI_DRIVER              0

//   <e0>Slave Device 0
//   <i>Enable or disable slave device
//   <i>SPI_Control_SlaveSelect function must be implemented to assert or deassert Slave Select pin.
//     <o1>Export control block Driver_SPI# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define SPI_ENABLE_SLAVE_0      1
#define SPI_DRIVER_SLAVE_0      10

//   <e0>Slave Device 1
//   <i>Enable or disable slave device
//   <i>SPI_Control_SlaveSelect function must be implemented to assert or deassert Slave Select pin.
//     <o1>Export control block Driver_SPI# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define SPI_ENABLE_SLAVE_1      1
#define SPI_DRIVER_SLAVE_1      11

//   <e0>Slave Device 2
//   <i>Enable or disable slave device
//   <i>SPI_Control_SlaveSelect function must be implemented to assert or deassert Slave Select pin.
//     <o1>Export control block Driver_SPI# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define SPI_ENABLE_SLAVE_2      0
#define SPI_DRIVER_SLAVE_2      12

//   <e0>Slave Device 3
//   <i>Enable or disable slave device
//   <i>SPI_Control_SlaveSelect function must be implemented to assert or deassert Slave Select pin.
//     <o1>Export control block Driver_SPI# <0-255>
//     <i>Define exported driver control block number
//   </e>
#define SPI_ENABLE_SLAVE_3      0
#define SPI_DRIVER_SLAVE_3      13

// </h>

//------------- <<< end of configuration section >>> -------------------------
