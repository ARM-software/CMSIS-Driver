/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2019 Arm Limited
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        7. March 2019
 * $Revision:    V1.0
 *
 * Project:      WiFi Driver Hardware specific implementation for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * -------------------------------------------------------------------------- */

/**
  \fn          void WiFi_ISM43362_Pin_Initialize (void)
  \brief       Initialize pin(s).
  \return      none
*/
void WiFi_ISM43362_Pin_Initialize (void) {
  // Add code for initializing Reset, Slave Select and Data Ready pins here
  // (for Data Ready pin external interrupt can also be setup here)
  // <code WiFi_ISM43362_Pin_Initialize>

  // </code>
}

/**
  \fn          void WiFi_ISM43362_Pin_Uninitialize (void)
  \brief       De-initialize pin(s).
  \return      none
*/
void WiFi_ISM43362_Pin_Uninitialize (void) {
  // Add code for deinitializing Reset, Slave Select and Data Ready pins here
  // <code WiFi_ISM43362_Pin_Uninitialize>

  // </code>
}

/**
  \fn          void WiFi_ISM43362_Pin_RSTN (uint8_t rstn)
  \brief       Drive Reset line.
  \param[in]   rstn
                 - value = 0: Drive Reset line not active state
                 - value = 1: Drive Reset line active state
  \return      none
*/
void WiFi_ISM43362_Pin_RSTN (uint8_t rstn) {
  // Add code for driving Reset pin here
  // <code WiFi_ISM43362_Pin_RSTN>

  // </code>
}

/**
  \fn          void WiFi_ISM43362_Pin_SSN (uint8_t ssn)
  \brief       Drive Slave Select line.
  \param[in]   ssn
                 - value = 0: Drive Slave Select line not active state
                 - value = 1: Drive Slave Select line active state
  \return      none
*/
void WiFi_ISM43362_Pin_SSN (uint8_t ssn) {
  // Add code for driving Slave Select pin here
  // <code WiFi_ISM43362_Pin_SSN>

  // </code>
}

/**
  \fn          uint8_t WiFi_ISM43362_Pin_DATARDY (void)
  \brief       Get Data Ready line state.
  \return      Data Ready line state
                 - 0: Data Ready line is not active state
                 - 1: Data Ready line is active state
*/
uint8_t WiFi_ISM43362_Pin_DATARDY (void) {
  // Add code for retrieving Data Ready pin state here
  // <code WiFi_ISM43362_Pin_DATARDY>

  // </code>
}
