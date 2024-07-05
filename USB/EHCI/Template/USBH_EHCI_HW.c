/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------------
 *
 * $Date:       28. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host EHCI Controller Driver Hardware-specific template
 *
 * -----------------------------------------------------------------------------
 */

#include "USBH_EHCI_HW.h"

static USBH_EHCI_Interrupt_t EHCI_IRQ_Handler;

/**
  \fn          int32_t USBH_EHCI_HW_Initialize (uint8_t ctrl, USBH_EHCI_Interrupt_t interrupt_handler)
  \brief       Initialize USB Host EHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   interrupt_handler  Pointer to Interrupt Handler Routine
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_Initialize (uint8_t ctrl, USBH_EHCI_Interrupt_t interrupt_handler) {

  // Add hardware-specific initialization code

  EHCI_IRQ_Handler = interrupt_handler; // Register EHCI interrupt handler

  return 0;
}

/**
  \fn          int32_t USBH_EHCI_HW_Uninitialize (uint8_t ctrl)
  \brief       De-initialize USB Host EHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_Uninitialize (uint8_t ctrl) {

  // Add hardware-specific uninitialization code

  return 0;
}

/**
  \fn          int32_t USBH_EHCI_HW_PowerControl (uint8_t ctrl, uint32_t state)
  \brief       Control USB Host EHCI Interface Power.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   state              Power state (0 = power off, 1 = power on)
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_PowerControl (uint8_t ctrl, uint32_t state) {

  if (state == 0U) {                    // If power off requested
    // Add hardware-specific code to power off the Host interface
  } else {                              // If power on requested
    // Add hardware-specific code to power on the Host interface
  }

  return 0;
}

/**
  \fn          void USBH0_IRQ (void)
  \brief       USB0 Host Interrupt Routine (IRQ).
*/
void USBH0_IRQ (void) {
  EHCI_IRQ_Handler();                   // Call registered EHCI interrupt handler from hardware USB interrupt handler
}
