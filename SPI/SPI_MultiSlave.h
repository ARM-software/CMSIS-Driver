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
 * $Revision:    V1.0.1
 *
 * Driver:       Driver_SPI# (default: Driver_SPI0)
 * Project:      SPI Master to Multi-Slave Wrapper
 * -------------------------------------------------------------------- */

#include "Driver_SPI.h"

/**
  Slave select pin control function.

  This function must be implemented in application to switch slave select
  pin to required state.

  Implementation example:
  \code
  void SPI_Control_SlaveSelect (uint32_t device, uint32_t ss_state) {
    GPIO    *port;
    uint32_t pin;
 
    switch (device) {
      case 0: // Select LCD_CS pin
        port = GPIO0;
        pin  = 0;
        break;
 
      case 1: // Select MicroSD_CS pin
        port = GPIO1;
        pin  = 7;
        break;
 
      case 2: // Nonexisting slaves
      case 3:
        return;
    }
 
    if (ss_state == ARM_SPI_SS_INACTIVE) {
      GPIO_PinSet (port, pin);   // Set GPIO pin high
    } else {
      GPIO_PinReset (port, pin); // Set GPIO pin low
    }
  }
  \endcode

  \param[in]    device    Slave device number
  \param[in]    ss_state  Slave Select signal state (ARM_SPI_SS_INACTIVE | ARM_SPI_SS_ACTIVE)
*/
extern void SPI_Control_SlaveSelect (uint32_t device, uint32_t ss_state);
