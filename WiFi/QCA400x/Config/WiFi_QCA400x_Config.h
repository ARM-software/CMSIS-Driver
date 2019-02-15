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
 * $Date:        14. February 2019
 * $Revision:    V1.0
 *
 * Project:      WiFi Driver Configuration for 
 *               Qualcomm QCA400x WiFi Module
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __WIFI_QCA400x_CONFIG_H
#define __WIFI_QCA400x_CONFIG_H

#include "QCA400x_Config.h"

// <h> WiFi Qualcomm QCA400x Driver Configuration
//   <i> QCA400x SDK can be configured in QCA400x_Config.h
//   <i> QCA400x SDK hardware specific implementation can be adapted in QCA400x_HW.c
//   <o> WiFi Driver Number
//   <i> Specifies exported driver index (default: Driver_WiFi0)
#define WIFI_QCA400x_DRIVER_INDEX           0

// </h>

// WiFi Connect/Disconnect timeout (default: 30000 ms)
#define WIFI_QCA400x_CON_DISCON_TIMEOUT     (30000U)

// Hostname resolve timeout (default: 5000 ms)
#define WIFI_QCA400x_RESOLVE_TIMEOUT        (5000U)

// Initial socket send/receive timeout (default: 10000 ms)
#define WIFI_QCA400x_SOCKET_DEF_TIMEOUT     (10000U)

// Maximum packet length - in bytes (default: 1576)
#define WIFI_QCA400x_MAX_PACKET_LEN         (1576)

// Scan Buffer size - in bytes (default: 128)
#define WIFI_QCA400x_SCAN_BUF_LEN           (128)

// Maximum number of stations that can connect to AP (default: 8)
#define WIFI_QCA400x_AP_MAX_NUM             (8)

#endif // __WIFI_QCA400x_CONFIG_H
