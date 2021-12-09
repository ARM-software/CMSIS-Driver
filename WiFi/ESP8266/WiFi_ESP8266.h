/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2021 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        6. June 2021
 *
 * Project:      ESP8266 WiFi Driver
 * -------------------------------------------------------------------------- */

#ifndef WIFI_ESP8266_H__
#define WIFI_ESP8266_H__

#include <string.h>

#include "Driver_WiFi.h"                // ::CMSIS Driver:WiFi
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

#include "ESP8266.h"

#define WIFI_SERIAL_BAUDRATE        WIFI_ESP8266_SERIAL_BAUDRATE
#define WIFI_DRIVER_NUMBER          WIFI_ESP8266_DRIVER_NUMBER

/* Command response timeout [ms] (default) */
#ifndef WIFI_RESP_TIMEOUT
#define WIFI_RESP_TIMEOUT           (5000)
#endif

/* Connection open timeout [ms] (default) */
#ifndef WIFI_CONNOPEN_TIMEOUT
#define WIFI_CONNOPEN_TIMEOUT       (20000)
#endif

/* Socket accept timeout */
#ifndef WIFI_SOCKET_ACCEPT_TIMEOUT
#define WIFI_SOCKET_ACCEPT_TIMEOUT  (0)
#endif

/* WiFi thread pooling interval [ms] */
#ifndef WIFI_THREAD_POOLING_TIMEOUT
#define WIFI_THREAD_POOLING_TIMEOUT (20)
#endif 

/* Access point default channel (used when channel not specified in Activate) */
#ifndef WIFI_AP_CHANNEL
#define WIFI_AP_CHANNEL             (2)
#endif

/* AT response echo enable/disable */
#ifndef WIFI_AT_ECHO
#define WIFI_AT_ECHO                (0)
#endif

/* WIFI interface definitions */
#define WIFI_INTERFACE_STATION      0
#define WIFI_INTERFACE_AP           1

/* WIFI thread flags */
#define WIFI_THREAD_RX_DATA         0x01    // Data received
#define WIFI_THREAD_RX_ERROR        0x02    // Data receive error
#define WIFI_THREAD_TX_DONE         0x04    // Data transmitted
#define WIFI_THREAD_TERMINATE       0x08    // Terminate thread
#define WIFI_THREAD_KICK            0x10    // Wake-up

#define WIFI_THREAD_FLAGS          (WIFI_THREAD_RX_DATA  | \
                                    WIFI_THREAD_RX_ERROR | \
                                    WIFI_THREAD_TX_DONE  | \
                                    WIFI_THREAD_TERMINATE| \
                                    WIFI_THREAD_KICK)

/* WIFI wait flags */
#define WIFI_WAIT_RESP_GENERIC      (1U <<   0)
#define WIFI_WAIT_TX_REQUEST        (1U <<   1)
#define WIFI_WAIT_CONN_ACCEPT       (1U <<   2)
#define WIFI_WAIT_TX_DONE           (1U <<   3)
#define WIFI_WAIT_RX_DONE(n)        (1U << ( 4 + (n))) /* n == [0,4] */
#define WIFI_WAIT_CONN_OPEN(n)      (1U << ( 9 + (n))) /* n == [0,4] */
#define WIFI_WAIT_CONN_CLOSE(n)     (1U << (14 + (n))) /* n == [0,4] */

/* Socket State */
#define SOCKET_STATE_FREE           0U
#define SOCKET_STATE_CREATED        1U
#define SOCKET_STATE_BOUND          2U
#define SOCKET_STATE_LISTEN         3U
#define SOCKET_STATE_CONNECTREQ     4U
#define SOCKET_STATE_CONNECTED      5U
#define SOCKET_STATE_CLOSING        6U
#define SOCKET_STATE_CLOSED         7U
#define SOCKET_STATE_SERVER         8U

/* Socket flags */
#define SOCKET_FLAGS_NONBLOCK       (1U << 0)
#define SOCKET_FLAGS_KEEPALIVE      (1U << 1)
#define SOCKET_FLAGS_TOUT           (1U << 2)

/* Socket control block */
typedef struct {
  BUF_LIST mem;                 /* Socket memory           */
  int32_t  type;                /* Socket type: TCP, DGRAM */
  int32_t  protocol;            /* Protocol type (TCP,UDP) */
  uint32_t tout_rx;             /* Rx timeout              */
  uint32_t tout_tx;             /* Tx timeout              */
  volatile
  uint8_t state;                /* Socket State            */
  uint8_t flags;                /* Socket Flags            */
  uint8_t server;               /* Server socket                */
  uint8_t backlog;              /* Next in backlog socket list  */
  volatile
  uint32_t conn_id;             /* Socket connection id              */
  uint32_t rx_len;              /* Number of bytes in current packet */

  uint16_t r_port;              /* Remote port */
  uint16_t l_port;              /* Local port  */
  uint8_t  r_ip[4];             /* Remote ip   */
  uint8_t  l_ip[4];             /* Local ip    */
} WIFI_SOCKET;

/* WIFI driver options */
typedef struct {
  uint8_t  st_ip[4];
  uint8_t  st_gateway[4];
  uint8_t  st_netmask[4];
  uint8_t  st_bssid[6];         /* BSSID of the AP to connect to */

  uint8_t  ap_mac[6];
  uint8_t  ap_ip[4];
  uint8_t  ap_gateway[4];
  uint8_t  ap_netmask[4];
  uint8_t  ap_dhcp_pool_start[4];
  uint8_t  ap_dhcp_pool_end[4];
  uint32_t ap_dhcp_lease;
} WIFI_OPTIONS;

/* WIFI driver state flags */
#define WIFI_FLAGS_INIT               (1U << 0)
#define WIFI_FLAGS_POWER              (1U << 1)
#define WIFI_FLAGS_CONN_INFO_POOLING  (1U << 2)
#define WIFI_FLAGS_AP_ACTIVE          (1U << 3)
#define WIFI_FLAGS_STATION_ACTIVE     (1U << 4)
#define WIFI_FLAGS_STATION_CONNECTED  (1U << 5)
#define WIFI_FLAGS_STATION_GOT_IP     (1U << 6)
#define WIFI_FLAGS_STATION_STATIC_IP  (1U << 7)
#define WIFI_FLAGS_AP_STATIC_IP       (1U << 8)
#define WIFI_FLAGS_STATION_BSSID_SET  (1U << 9)

#define SOCKET_INVALID                0xFF
#define CONN_ID_INVALID               5

/* WIFI driver descriptor */
typedef struct {
  ARM_WIFI_SignalEvent_t cb_event;    /* Event callback              */
  osThreadId_t           thread_id;   /* Data processing thread id   */
  osEventFlagsId_t       evflags_id;  /* Event flags object id       */
  osMemoryPoolId_t       mempool_id;  /* Socket memory pool id       */
  osMutexId_t            mutex_id;    /* Socket access guard         */
  osMutexId_t            memmtx_id;   /* Memory access mutex         */
  WIFI_OPTIONS           options;     /* Set/GetOption value storage */
  uint32_t               lp_timer;    /* Deep sleep time in seconds  */
  uint8_t                tx_power;    /* Stored TX_POWER value       */
  uint8_t                conn_id;     /* Connection identifier state */
  uint8_t                ap_ecn;      /* AP encryption method        */
  char                   ap_pass[33]; /* AP password                 */
  uint16_t               packdump;    /* Number of dumped rx packets */
  uint16_t               flags;       /* Driver state flags          */
} WIFI_CTRL;

extern ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_DRIVER_NUMBER);

/* Static functions */
static ARM_DRIVER_VERSION    ARM_WIFI_GetVersion (void);
static ARM_WIFI_CAPABILITIES ARM_WIFI_GetCapabilities (void);
static int32_t  ARM_WIFI_Initialize (ARM_WIFI_SignalEvent_t cb_event);
static int32_t  ARM_WIFI_Uninitialize (void);
static int32_t  ARM_WIFI_PowerControl (ARM_POWER_STATE state);
static int32_t  ARM_WIFI_GetModuleInfo (char *module_info, uint32_t max_len);
static int32_t  ARM_WIFI_SetOption (uint32_t interface, uint32_t option, const void *data, uint32_t len);
static int32_t  ARM_WIFI_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len);
static int32_t  ARM_WIFI_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], uint32_t max_num);
static int32_t  ARM_WIFI_Activate (uint32_t interface, const ARM_WIFI_CONFIG_t *config);
static int32_t  ARM_WIFI_Deactivate (uint32_t interface);
static uint32_t ARM_WIFI_IsConnected (void);
static int32_t  ARM_WIFI_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info);
static int32_t  ARM_WIFI_BypassControl (uint32_t interface, uint32_t mode);
static int32_t  ARM_WIFI_EthSendFrame (uint32_t interface, const uint8_t *frame, uint32_t len);
static int32_t  ARM_WIFI_EthReadFrame (uint32_t interface, uint8_t *frame, uint32_t len);
static uint32_t ARM_WIFI_EthGetRxFrameSize (uint32_t interface);
static int32_t  ARM_WIFI_SocketCreate (int32_t af, int32_t type, int32_t protocol);
static int32_t  ARM_WIFI_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port);
static int32_t  ARM_WIFI_SocketListen (int32_t socket, int32_t backlog);
static int32_t  ARM_WIFI_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t  ARM_WIFI_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port);
static int32_t  ARM_WIFI_SocketRecv (int32_t socket, void *buf, uint32_t len);
static int32_t  ARM_WIFI_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t  ARM_WIFI_SocketSend (int32_t socket, const void *buf, uint32_t len);
static int32_t  ARM_WIFI_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port);
static int32_t  ARM_WIFI_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t  ARM_WIFI_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t  ARM_WIFI_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len);
static int32_t  ARM_WIFI_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len);
static int32_t  ARM_WIFI_SocketClose (int32_t socket);
static int32_t  ARM_WIFI_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len);
static int32_t  ARM_WIFI_Ping (const uint8_t *ip, uint32_t ip_len);

/* Static helpers */
static void     WiFi_Thread        (void *arg) __attribute__((noreturn));
static int32_t  WiFi_Wait          (uint32_t event, uint32_t timeout);
static int32_t  ResetModule        (void);
static int32_t  SetupCommunication (void);
static int32_t  LoadOptions        (void);
static int32_t  IsUnspecifiedIP    (const uint8_t ip[]);
static int32_t  GetCurrentMAC      (uint32_t interface, uint8_t mac[]);
static int32_t  GetCurrentIpAddr   (uint32_t interface, uint8_t ip[], uint8_t gw[], uint8_t mask[]);
static int32_t  GetCurrentDhcpPool (uint32_t *t_lease, uint8_t ip_start[], uint8_t ip_end[]);
static int32_t  GetCurrentDnsAddr  (uint32_t interface, uint8_t dns0[], uint8_t dns1[]);
static uint32_t GetOpt             (const void *opt_val, uint32_t opt_len);
static uint32_t SetOpt             (void *opt_val, uint32_t val, uint32_t opt_len);
static uint32_t ConnId_Alloc       (void);
static void     ConnId_Free        (uint32_t conn_id);
static void     ConnId_Accept      (uint32_t conn_id);

#endif /* WIFI_ESP8266_H__ */
