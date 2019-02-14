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
 * $Date:        30. January 2019
 * $Revision:    V1.0
 *
 * Project:      WiFi Driver Hardware specific implementation for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * -------------------------------------------------------------------------- */

#include <stdbool.h>

/**
  \fn          void WiFi_Pin_RSTN (bool rstn)
  \brief       Drive Reset line.
  \param[in]   rstn
                 - \b false Drive Reset line not active state
                 - \b true  Drive Reset line active state
  \return      none
*/
void WiFi_ISM43362_Pin_RSTN (bool rstn) {
  // Add code for driving Reset pin here
}

/**
  \fn          void WiFi_Pin_SSN (bool ssn)
  \brief       Drive Slave Select line.
  \param[in]   ssn
                 - \b false Drive Slave Select line not active state
                 - \b true  Drive Slave Select line active state
  \return      none
*/
void WiFi_ISM43362_Pin_SSN (bool ssn) {
  // Add code for driving Slave Select pin here
}

/**
  \fn          bool WiFi_Pin_DATARDY (void)
  \brief       Get Data Ready line state.
  \return      Data Ready line state
                 - \b false Data Ready line is not active state
                 - \b true  Data Ready line is active state
*/
bool WiFi_ISM43362_Pin_DATARDY (void) {
  // Add code for retrieving Data Ready pin state here
}
