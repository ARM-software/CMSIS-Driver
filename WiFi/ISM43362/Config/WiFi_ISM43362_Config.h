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
 * Project:      WiFi Driver Configuration for 
 *               Inventek ISM43362-M3G-L44 WiFi Module
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __WIFI_ISM43362_CONFIG_H
#define __WIFI_ISM43362_CONFIG_H

// <h> WiFi Inventek ISM43362 Driver Configuration (SPI variant)

//   <o> WiFi Driver Number
//   <i> Configuration settings specifying index of driver exported structure name (default: Driver_WiFi0)
#define WIFI_ISM43362_DRIVER_INDEX          0

//   <o> SPI Driver Number
//   <i> Configuration settings specifying index of SPI driver used (if Module uses SPI interface)
#define WIFI_ISM43362_SPI_DRV_NUM           3

// </h>

// Number of sockets supported by Module (default and maximum: 4)
#define WIFI_ISM43362_SOCKETS_NUM          (4)

// Socket send/receive default timeout (default: 10000ms)
#define WIFI_ISM43362_SOCKET_DEF_TIMEOUT   (10000)

// SPI mutex acquire timeout (default: 1000 ms)
#define WIFI_ISM43362_SPI_TIMEOUT          (1000)

// SPI command timeout (default: 30000 ms)
#define WIFI_ISM43362_CMD_TIMEOUT          (30000)

// Asynchronous thread polling time interval (default: 1000 ms)
#define WIFI_ISM43362_ASYNC_INTERVAL       (1000)

// Asynchronous thread priority (default: osPriorityAboveNormal)
#define WIFI0_ISM43362_ASYNC_PRIORITY      (osPriorityAboveNormal)

// SPI Receive transfer size (default: 32)
#define WIFI_ISM43362_SPI_RECEIVE_SIZE     (32)

#endif // __WIFI_ISM43362_CONFIG_H
