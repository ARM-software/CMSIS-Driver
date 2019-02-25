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
 * $Date:        25. February 2019
 * $Revision:    V1.0
 *
 * Project:      WiFi Driver Configuration for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __WIFI_ISM43362_CONFIG_H
#define __WIFI_ISM43362_CONFIG_H

// <h> WiFi Inventek ISM43362 Driver Configuration (SPI)

//   <o> WiFi Driver Number
//   <i> Configuration settings specifying driver number (default: Driver_WiFi0)
#define WIFI_ISM43362_DRV_NUM               0

//   <o> SPI Driver Number
//   <i> Configuration settings specifying SPI driver number used (if Module uses SPI interface)
#define WIFI_ISM43362_SPI_DRV_NUM           3

// </h>

// Number of sockets supported by Module (default and maximum: 4)
#define WIFI_ISM43362_SOCKETS_NUM          (4)

// SPI mutex acquire timeout in ms (default: 1000)
#define WIFI_ISM43362_SPI_TIMEOUT          (1000)

// SPI receive transfer size (default: 32)
// This setting specifies chunk size in which SPI read is done
#define WIFI_ISM43362_SPI_RECEIVE_SIZE     (32)

// SPI command timeout in ms (default: 30000)
// Maximum time that command can keep the SPI DATARDY line busy
#define WIFI_ISM43362_CMD_TIMEOUT          (30000)

// Asynchronous thread polling time interval in ms (default: 1000)
// Interval in which asynchronous events are polled and also interval 
// in which long blocking socket receive is split
#define WIFI_ISM43362_ASYNC_INTERVAL       (1000)

// Asynchronous thread priority (default: osPriorityAboveNormal)
// This priority should be above user threads
#define WIFI_ISM43362_ASYNC_PRIORITY       (osPriorityAboveNormal)

#endif // __WIFI_ISM43362_CONFIG_H
