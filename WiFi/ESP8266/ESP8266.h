/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2020 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        11. February 2020
 *
 * Project:      ESP8266 WiFi Driver
 * -------------------------------------------------------------------------- */

#ifndef ESP8266_H__
#define ESP8266_H__

#include "BufList.h"

#include "WiFi_ESP8266_Config.h"

/* AT command set version and variant used */
#ifndef AT_VERSION
#define AT_VERSION                      0x01060200
#endif
#ifndef AT_VARIANT
#define AT_VARIANT                      AT_VARIANT_ESP
#endif

/* AT command set version definition               */
/* Version as major.minor.patch.build:  0xMMmmppbb */
#define AT_VERSION_2_0_0_0              0x02000000
#define AT_VERSION_1_9_0_0              0x01090000
#define AT_VERSION_1_8_0_0              0x01080000
#define AT_VERSION_1_7_0_0              0x01070000
#define AT_VERSION_1_6_0_0              0x01060000
#define AT_VERSION_1_5_0_0              0x01050000
#define AT_VERSION_1_4_0_0              0x01040000
#define AT_VERSION_1_3_0_0              0x01030000
#define AT_VERSION_1_2_0_0              0x01020000
#define AT_VERSION_1_1_0_0              0x01010000
#define AT_VERSION_1_0_0_0              0x01000000

/* AT command set variants */
#define AT_VARIANT_ESP                  0   /* ESP8266  */
#define AT_VARIANT_ESP32                1   /* ESP32    */
#define AT_VARIANT_WIZ                  2   /* WizFi360 */

/* Callback events codes */
#define AT_NOTIFY_EXECUTE               0  /* Serial data available, execute parser */
#define AT_NOTIFY_CONNECTED             1  /* Local station connected to an AP      */
#define AT_NOTIFY_GOT_IP                2  /* Local station got IP address          */
#define AT_NOTIFY_DISCONNECTED          3  /* Local station disconnected from an AP */
#define AT_NOTIFY_CONNECTION_OPEN       4  /* Network connection is established     */
#define AT_NOTIFY_CONNECTION_CLOSED     5  /* Network connection is closed          */
#define AT_NOTIFY_STATION_CONNECTED     6  /* Station connects to the local AP      */
#define AT_NOTIFY_STATION_DISCONNECTED  7  /* Station disconnects from the local AP */
#define AT_NOTIFY_CONNECTION_RX_INIT    8  /* Connection will start receiving data  */
#define AT_NOTIFY_CONNECTION_RX_DATA    9  /* Connection data is ready to read      */
#define AT_NOTIFY_REQUEST_TO_SEND       10 /* Request to load data to transmit      */
#define AT_NOTIFY_RESPONSE_GENERIC      11 /* Received generic command response     */
#define AT_NOTIFY_TX_DONE               12 /* Serial transmit completed             */
#define AT_NOTIFY_OUT_OF_MEMORY         13 /* Serial parser is out of memory        */
#define AT_NOTIFY_ERR_CODE              14 /* Received "ERR_CODE" response          */
#define AT_NOTIFY_READY                 15 /* The AT firmware is ready              */

/**
  AT parser notify callback function.

  \param[in]  event   Callback event code (AT_NOTIFY codes)
  \param[in]  arg     Event argument
*/
extern void AT_Notify (uint32_t event, void *arg);

/* Generic responses */
#define AT_RESP_OK                  0  /* "OK"                */
#define AT_RESP_ERROR               1  /* "ERROR"             */
#define AT_RESP_FAIL                2  /* "FAIL"              */
#define AT_RESP_SEND_OK             3  /* "SEND OK"           */
#define AT_RESP_SEND_FAIL           4  /* "SEND FAIL"         */
#define AT_RESP_BUSY_P              5  /* "busy p..."         */
#define AT_RESP_BUSY_S              6  /* "busy s..."         */
#define AT_RESP_ALREADY_CONNECTED   7  /* "ALREADY CONNECTED" */
#define AT_RESP_WIFI_CONNECTED      8  /* "WIFI CONNECTED"    */
#define AT_RESP_WIFI_GOT_IP         9  /* "WIFI GOT IP"       */
#define AT_RESP_WIFI_DISCONNECT    10  /* "WIFI DISCONNECT"   */
#define AT_RESP_ECHO               11  /* (echo)              */
#define AT_RESP_READY              12  /* "ready"             */
#define AT_RESP_ERR_CODE           13  /* "ERR CODE:0x..."    */
#define AT_RESP_UNKNOWN          0xFF  /* (unknown)           */

/* AT command mode */
#define AT_CMODE_QUERY              0  /* Inquiry command: AT+<x>?      */
#define AT_CMODE_SET                1  /* Set command:     AT+<x>=<...> */
#define AT_CMODE_EXEC               2  /* Execute command: AT+<x>       */
#define AT_CMODE_TEST               3  /* Test command:    AT+<x>=?     */

/* AT_DATA_CWLAP ecn encoding */
#define AT_DATA_ECN_OPEN            0
#define AT_DATA_ECN_WEP             1
#define AT_DATA_ECN_WPA_PSK         2
#define AT_DATA_ECN_WPA2_PSK        3
#define AT_DATA_ECN_WPA_WPA2_PSK    4
#define AT_DATA_ECN_WPA2_E          5

/* List AP */
typedef struct {
  uint8_t ecn;        /* Encryption */
  char    ssid[32+1]; /* SSID string */
  int8_t  rssi;       /* Signal strength */
  uint8_t mac[6];     /* MAC address */
  uint8_t ch;         /* Channel */
  uint16_t freq_offs; /* Frequency offset of AP [kHz] */
} AT_DATA_CWLAP;

/* Configure AP */
typedef struct {
  char *ssid;         /* SSID of AP */
  char *pwd;          /* Password string, length [8:64] bytes, ASCII */
  uint8_t ch;         /* Channel ID */
  uint8_t ecn;        /* Encryption */
  uint8_t max_conn;   /* Max number of stations, range [1,8] */
  uint8_t ssid_hide;  /* 0: SSID is broadcasted (default), 1: SSID is not broadcasted */
} AT_DATA_CWSAP;

/* Connect AP */
typedef struct {
  char    ssid[32+1]; /* SSID string */
  uint8_t bssid[6];   /* BSSID: AP MAC address */
  uint8_t ch;         /* Channel */
  uint8_t rssi;       /* Signal strength */
} AT_DATA_CWJAP;

/* Link Connection */
typedef struct {
  uint8_t  link_id;       /* Connection ID */
  char     type[4];       /* Type, "TCP", "UDP" */
  uint8_t  c_s;           /* ESP runs as 0:client, 1:server */
  uint8_t  remote_ip[4];  /* Remote IP address */
  uint16_t remote_port;   /* Remote port number */
  uint16_t local_port;    /* Local port number */
} AT_DATA_LINK_CONN;

/* Device communication interface */
typedef struct {
  uint32_t baudrate;      /* Configured baud rate */
  uint8_t  databits;      /* 5:5-bit data, 6:6-bit data, 7:7-bit data, 8:8-bit data */
  uint8_t  stopbits;      /* 1:1-stop bit, 2:2-stop bits */
  uint8_t  parity;        /* 0:none, 1:odd, 2:even */
  uint8_t  flow_control;  /* 0:none, 1:RTS, 2:CTS, 3:RTS/CTS */
} AT_PARSER_COM_SERIAL;

/* Device control block */
typedef struct {
  BUF_LIST mem;         /* Parser memory buffer */
  BUF_LIST resp;        /* Response data buffer */
  uint8_t  state;       /* Parser state */
  uint8_t  cmd_sent;    /* Last command sent     */
  uint8_t  gen_resp;    /* Generic response */
  uint8_t  msg_code;    /* Message code          */
  uint8_t  ctrl_code;   /* Control code          */
  uint8_t  resp_code;   /* Response command code */
  uint8_t  resp_len;    /* Response length */
  uint8_t  rsvd[2];     /* Reserved */
  uint32_t ipd_rx;      /* Number of bytes to receive (+IPD) */
} AT_PARSER_HANDLE;

/* AT parser state */
#define AT_STATE_ANALYZE     0
#define AT_STATE_WAIT        1
#define AT_STATE_FLUSH       2
#define AT_STATE_RESP_DATA   3
#define AT_STATE_RESP_GMR    4
#define AT_STATE_RESP_GEN    5
#define AT_STATE_RECV_DATA   6
#define AT_STATE_SEND_DATA   7
#define AT_STATE_RESP_CTRL   8
#define AT_STATE_RESP_ECHO   9

/* AT parser functions */
extern int32_t AT_Parser_Initialize   (void);
extern int32_t AT_Parser_Uninitialize (void);
extern int32_t AT_Parser_GetSerialCfg (AT_PARSER_COM_SERIAL *info);
extern int32_t AT_Parser_SetSerialCfg (AT_PARSER_COM_SERIAL *info);
extern void    AT_Parser_Execute      (void);
extern void    AT_Parser_Reset        (void);

/* Command/Response functions */

/**
  Test AT startup

  Generic response is expected.

  \return
*/
extern int32_t AT_Cmd_TestAT (void);

/**
  Restarts the module

  Generic response is expected.

  \return
*/
extern int32_t AT_Cmd_Reset (void);

/**
  Check version information

  Generic response is expected.

  \return
*/
extern int32_t AT_Cmd_GetVersion (void);

/**
  Get response to GetVersion command

  \param[out] buf
  \param[in]  len
  \return
*/
extern int32_t AT_Resp_GetVersion (uint8_t *buf, uint32_t len);

/**
  Enable or disable command echo.
  
  ESP8266 can echo received commands.
  Generic response is expected.
  
  \param[in]  enable  Echo enable(1) or disable(0)
  \return
  
*/
extern int32_t AT_Cmd_Echo (uint32_t enable);

/**
  Set/Query the current UART configuration

  Supported baudrates:
  - ESP8266: in range of [110, 115200*40]
  - WizFi360: [600:921600, 1000000, 1500000, 2000000]

  \note OK response is sent first (using old UART settings) and then UART config is switched.
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  baudrate
  \param[in]  databits
  \param[in]  stop_par_flowc stopbits[5:4], parity[3:2], flow control[1:0]
  \return
*/
extern int32_t AT_Cmd_ConfigUART (uint32_t at_cmode, uint32_t baudrate, uint32_t databits, uint32_t stop_par_flowc);

/**
  Get response to ConfigUART command

  \param[out] baudrate
  \param[out] databits
  \param[out] stop_par_flowc stopbits[5:4], parity[3:2], flow control[1:0]
  \return
*/
extern int32_t AT_Resp_ConfigUART (uint32_t *baudrate, uint32_t *databits, uint32_t *stop_par_flowc);

/**
  Configure the sleep modes.

  Command: SLEEP

  \note Command can be used only in Station mode. Modem-sleep is the default mode.

  \param[in]  sleep_mode  sleep mode (0: disabled, 1: Light-sleep, 2: Modem-sleep)
  \return 0: ok, -1: error
*/
extern int32_t AT_Cmd_Sleep (uint32_t at_cmode, uint32_t sleep_mode);

/**
  Get response to Sleep command.

  \param[out]   sleep_mode  Pointer to variable where the sleep mode is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_Sleep (uint32_t *sleep_mode);

/**
  Set maximum value of RF TX power [dBm] (set only command).

  Command: RFPOWER

  \note AT query command is not available, only AT set.

  \param[in]  tx_power  power value: range [0:82], units 0.25dBm
*/
extern int32_t AT_Cmd_TxPower (uint32_t tx_power);

/**
  Set current system messages.

  Command: SYSMSG_CUR

  Bit 0: configure the message of quitting passthrough transmission
  Bit 1: configure the message of establishing a network connection

  \note Only AT set command is available.

  \param[in]  n         message configuration bit mask [0:1]
*/
extern int32_t AT_Cmd_SysMessages (uint32_t n);


/**
  Set/Query the current Wi-Fi mode

  Command: CWMODE_CUR

  \note
  When setting mode to value 0, generic response "ERROR" is returned.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 1: station, 2: soft AP, 3: soft AP + station
  \return 0: OK, -1: error (invalid mode, etc)
*/
extern int32_t AT_Cmd_CurrentMode (uint32_t at_cmode, uint32_t mode);

/**
  Get response to CurrentMode command

  \param[in]  mode      Mode, 1: station, 2: soft AP, 3: soft AP + station
  \return
*/
extern int32_t AT_Resp_CurrentMode (uint32_t *mode);


/**
  Set/Query Configures the Name of Station

  Format S: AT+CWHOSTNAME=<hostname>
  Format Q: AT+CWHOSTNAME?

  Response S:
  "OK"
  "ERROR"

  Response Q: Current HostName
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  hostname  the host name of the Station, the maximum length is 32 bytes.
  \return 0: OK, -1: ERROR
*/
extern int32_t AT_Cmd_HostName (uint32_t at_cmode, const char* hostname);

/**
  Get response to HostName command

  Response Q: +CWHOSTNAME:<host	name>
  Example  Q: +CWHOSTNAME:ESP_XXXXXX\r\n\r\nOK

  \param[in]  hostname  the host name of the Station, the maximum length is 32 bytes.
  \return 0: OK, -1: ERROR
*/
extern int32_t AT_Resp_HostName (char* hostname);


/**
  Set/Query connected access point or access point to connect to.
  
  Command: CWJAP_CUR
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ssid
  \param[in]  pwd
  \param[in]  bssid
  \return
*/
extern int32_t AT_Cmd_ConnectAP (uint32_t at_cmode, const char *ssid, const char *pwd, const uint8_t *bssid);

/**
  Response to ConnectAP command.
  
  \param[out]  ap  query result of AP to which the station is already connected

  \return - 1: connection timeout
          - 2: wrong password
          - 3: cannot find the target AP
          - 4: connection failed
*/
extern int32_t AT_Resp_ConnectAP (AT_DATA_CWJAP *ap);


/**
  Disconnect from current access point (execute only command).
*/
extern int32_t AT_Cmd_DisconnectAP (void);


/**
  Configure local access point (SoftAP must be active)

  Command: CWSAP_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  cfg       AP configuration structure
  \return
*/
extern int32_t AT_Cmd_ConfigureAP (uint32_t at_cmode, AT_DATA_CWSAP *cfg);

/**
  Get response to ConfigureAP command

  \param[in]  cfg       AP configuration structure
  \return
*/
extern int32_t AT_Resp_ConfigureAP (AT_DATA_CWSAP *cfg);


/**
  Retrieve the list of all available access points

  Command: CWLAP
*/
extern int32_t AT_Cmd_ListAP (void);

/**
  Get response to ListAP command
  
  \param[out]   ap    Pointer to CWLAP structure
  \param[in] 
  \return execution status
          - negative: error
          - 0: access point list is empty
          - 1: access point list contains more data
          
*/
extern int32_t AT_Resp_ListAP (AT_DATA_CWLAP *ap);


/**
  Set/Query the station MAC address

  Command: CIPSTAMAC_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
  \return 
*/
extern int32_t AT_Cmd_StationMAC (uint32_t at_cmode, const uint8_t mac[]);

/**
  Get response to StationMAC command
  
  \param[out]   mac   Pointer to 6 byte array where MAC address will be stored
  \return execution status
*/
extern int32_t AT_Resp_StationMAC (uint8_t mac[]);


/**
  Set/Query the Access Point MAC address
  
  Command: CIPAPMAC_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
  \return 
*/
extern int32_t AT_Cmd_AccessPointMAC (uint32_t at_cmode, uint8_t mac[]);

/**
  Get response to AccessPointMAC command
  
  \param[out]   mac   Pointer to 6 byte array where MAC address will be stored
  \return execution status
*/
extern int32_t AT_Resp_AccessPointMAC (uint8_t mac[]);


/**
  Set/Query current IP address of the local station

  Command: CIPSTA_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ip        IP address
  \param[in]  gw        Gateway address
  \param[in]  mask      Netmask
  \return 
*/
extern int32_t AT_Cmd_StationIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]);

/**
  Get response to StationIP command

  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_StationIP (uint8_t addr[]);

/**
  Set/Query current IP address of the local access point

  Command: CIPAP_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ip    IP address
  \param[in]  gw    Gateway address
  \param[in]  mask  Netmask
  \return 
*/
extern int32_t AT_Cmd_AccessPointIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]);

/**
  Get response to AccessPointIP command

  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_AccessPointIP (uint8_t addr[]);

/**
  Set user defined DNS servers

  Command: CIPDNS_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    Enable (1) or disable (0) user defined DNS servers
  \param[in]  dns0      Primary DNS server
  \param[in]  dns1      Secondary DNS server
*/
extern int32_t AT_Cmd_DNS (uint32_t at_cmode, uint32_t enable, uint8_t dns0[], uint8_t dns1[]);

#if (AT_VARIANT == AT_VARIANT_ESP32) && (AT_VERSION >= AT_VERSION_2_0_0_0)
/**
  Get response to DNS command

  \param[out]   enable  Pointer to variable where enable flag will be stored
  \param[out]   dns0    Pointer to 4 byte array where DNS 0 address will be stored
  \param[out]   dns1    Pointer to 4 byte array where DNS 1 address will be stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_DNS (uint32_t *enable, uint8_t dns0[], uint8_t dns1[]);
#else
/**
  Get response to DNS command

  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_DNS (uint8_t addr[]);
#endif

#if AT_VARIANT != AT_VARIANT_ESP32
/**
  Set/Query DHCP state

  Command: CWDHCP_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0: set soft-ap, 1: set station, 2: set both (soft-ap and station)
  \param[in]  enable    0: disable DHCP, 1: enable DHCP
*/
extern int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t mode, uint32_t enable);
#else
/**
  Set/Query DHCP state

  Command: CWDHCP

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  operate   0: disable DHCP, 1: enable DHCP
  \param[in]  mode      Bit0: set station, Bit1: set soft-ap
*/
extern int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t operate, uint32_t mode);
#endif

/**
  Get response to DHCP command

  \param[out]   mode    Pointer to variable the DHCP mode is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_DHCP (uint32_t *mode);

/**
  Set/Query DHCP IP address lease time and range for local access point

  Command: CWDHCPS_CUR

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  en_tlease   bit[16] 0: disable setting, 1: enable setting the IP range
                          bits[15:0] lease time in minutes, range [1, 2880]
  \param[in]  ip_start    first IP in range that can be obtained from DHCP server
  \param[in]  ip_end      last IP in range that can be obtained from DHCP server
*/
extern int32_t AT_Cmd_RangeDHCP (uint32_t at_cmode, uint32_t en_tlease, uint8_t ip_start[], uint8_t ip_end[]);

/**
  Get response to RangeDHCP command

  \param[out]  t_lease  lease time in minutes, range[1, 2880]
  \param[out]  ip_start first IP in range that can be obtained from DHCP server
  \param[out]  ip_end   last IP in range that can be obtained from DHCP server

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_RangeDHCP (uint32_t *t_lease, uint8_t ip_start[], uint8_t ip_end[]);

/**
  Set/Query Auto-Connect to the AP

  Command: CWAUTOCONN

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    0:disable, 1:enable auto-connect on power-up

  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_AutoConnectAP (uint32_t at_cmode, uint32_t enable);

/**
  Get response to AutoConnectAP command

  \param[out]   enable  Pointer to variable the enable status is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_AutoConnectAP (uint32_t *enable);

/**
  Retrieve the list of IP (stations) connected to access point (execute only command)
  
  Command: CWLIF
*/
extern int32_t AT_Cmd_ListIP (void);

/**
  Get response to ListIP command

  \param[out] ip  Pointer to 4 byte array where IP address will be stored
  \param[out] mac Pointer to 6 byte array where MAC address will be stored
  \return execution status
          - negative: error
          - 0: list is empty
          - 1: list contains more data
*/
extern int32_t AT_Resp_ListIP (uint8_t ip[], uint8_t mac[]);

/**
  Set send data command.

  Command: CIPSEND
  
  Response: Generic response is expected

  \param[in]  at_cmode    command mode (inquiry, set, exec)
  \param[in]  link_id     connection id
  \param[in]  length      number of bytes to send
  \param[in]  remote_ip   remote IP (UDP transmission)
  \param[in]  remote_port remote port (UDP transmission)
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_SendData (uint32_t at_cmode, uint32_t link_id, uint32_t length, const uint8_t remote_ip[], uint16_t remote_port);

/**
  Set/Query connection type (single, multiple connections)

  Command: CIPMUX

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:single connection, 1:multiple connections
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnectionMux (uint32_t at_cmode, uint32_t mode);

/**
  Get response to ConnectionMux command

  \param[out] mode      0:single connection, 1:multiple connections

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_ConnectionMux (uint32_t *mode);

/**
  Create or delete TCP server.

  Command: CIPSERVER

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:delete server, 1:create server
  \param[in]  port      port number
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_TcpServer (uint32_t at_cmode, uint32_t mode, uint16_t port);

/**
  Set the maximum connection allowed by server.

  Command: CIPSERVERMAXCONN

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  num       maximum number of clients allowed to connect
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_TcpServerMaxConn (uint32_t at_cmode, uint32_t num);

/**
  Get response to TcpServerMaxConn command

  \param[in]  num       maximum number of clients allowed to connect

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_TcpServerMaxConn (uint32_t *num);

/**
  Enable or disable remote IP and port with +IPD.
  
  \param[in]  mode  0:do not show remote info, 1: show remote info
*/
extern int32_t AT_Cmd_RemoteInfo (uint32_t mode);

/**
  Retrieve incomming data.

  Response: +IPD,<link ID>,<len>[,<remote IP>, <remote port>]:<data>

  This response does not have CRLF terminator, <len> is the number of bytes in <data>.

  \param[out]  link_id     connection ID
  \param[out]  len         data length
  \param[out]  remote_ip   remote IP (enabled by command AT+CIPDINFO=1)
  \param[out]  remote_port remote port (enabled by command AT+CIPDINFO=1)
  

  \return 0: ok, len of data shall be read from buffer
          negative: buffer empty or packet incomplete
*/
extern int32_t AT_Resp_IPD (uint32_t *link_id, uint32_t *len, uint8_t *remote_ip, uint16_t *remote_port);

/**
  Get the connection status.
  
  Command: CIPSTATUS
  
  \param[in]  at_cmode    Command mode (exec only)
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_GetStatus (uint32_t at_cmode);

/**
  Get response to GetStatus command

  \param[out] conn         connection info

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_GetStatus (AT_DATA_LINK_CONN *conn);

/**
  Resolve IP address from host name.

  Command: CIPDOMAIN

  \note Domain name string is limited to 64 characters (ESP32, ESP8266 and WizFi360).

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  domain      domain name string (www.xxx.com)

  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_DnsFunction (uint32_t at_cmode, const char *domain);

/**
  Get response to DnsFunction command

  \param[out] ip          IP address

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_DnsFunction (uint8_t ip[]);

/**
  Establish TCP connection.

  Command: CIPSTART

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  link_id     ID of the connection
  \param[in]  r_ip        remote ip number
  \param[in]  r_port      remote port number
  \param[in]  keep_alive  TCP keep alive, 0:disable or [1:7200] in seconds to enable keep alive
  
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnOpenTCP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t keep_alive);

/**
  Establish UDP transmission.

  Command: CIPSTART

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  link_id     ID of the connection
  \param[in]  r_ip        remote ip number
  \param[in]  r_port      remote port number
  \param[in]  l_port      local port
  \param[in]  mode        UDP mode (0:dst peer entity will not change, 1:will change once, 2:allowed to change)
  
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnOpenUDP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t l_port, uint32_t mode);

/**
  Close the TCP/UDP/SSL connection.
  
  Command: CIPCLOSE

  If ID of the connection is 5, all connections will be closed.
  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  link_id   ID of the connection to be closed
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnectionClose (uint32_t at_cmode, uint32_t link_id);

/**
  Get ping response time.

  Command: PING

  Domain can be specified either by IP or by domain name string - only one should be
  specified.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  ip          IP address (xxx.xxx.xxx.xxx)
  \param[in]  domain      domain name string (www.xxx.com)
  
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_Ping (uint32_t at_cmode, const uint8_t ip[], const char *domain);

/**
  Get response to Ping command.

  Argument time indicates ping timeout when MSB is set ((time & 0x80000000) != 0).

  \param[out] time        ping response time in ms

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
extern int32_t AT_Resp_Ping (uint32_t *time);

/**
  Generic response.

  \return response code, see AT_RESP codes
*/
extern int32_t AT_Resp_Generic (void);

/**
  Get +LINK_CONN response parameters (see +SYSMSG_CUR).

  +LINK_CONN:<status_type>,<link_id>,"UDP/TCP/SSL",<c/s>,<remote_ip>,
                          <remote_port>,<local_port>
*/
extern int32_t AT_Resp_LinkConn (uint32_t *status, AT_DATA_LINK_CONN *conn);

/**
  Get connection number from the <conn_id>,CONNECT and <conn_id>,CLOSED response.

  \param[out] conn_id   connection ID
  \return execution status:
          -1: no response (buffer empty)
           0: connection number retrieved
*/
extern int32_t AT_Resp_CtrlConn (uint32_t *conn_id);

/**
  Get +STA_CONNECTED and +STA_DISCONNECTED response (mac).

  +STA_CONNECTED:<sta_mac>
  +STA_DISCONNECTED"<sta_mac>
*/
extern int32_t AT_Resp_StaMac (uint8_t mac[]);

/**
  Get ERR_CODE:0x... response.

  \param[out] err_code    Pointer to 32-bit variable where error code will be stored.
  \return execution status:
          -1: no response (buffer empty)
           0: error code retrieved
*/
extern int32_t AT_Resp_ErrCode (uint32_t *err_code);

/**
  Get number of bytes that can be sent.
*/
extern uint32_t AT_Send_GetFree (void);

/**
  Send data (reply to data transmit request).
*/
extern uint32_t AT_Send_Data (const uint8_t *buf, uint32_t len);

#endif /* ESP8266_H__ */
