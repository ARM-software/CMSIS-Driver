/* -----------------------------------------------------------------------------
 * Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        28. April 2022
 *
 * Project:      DA16200 WiFi Driver
 * -------------------------------------------------------------------------- */

#ifndef DA16200_H__
#define DA16200_H__

#include "BufList.h"

#include "WiFi_DA16200_Config.h"

/* AT command set version and variant used */
#ifndef AT_VERSION
#define AT_VERSION                      0x01000304
#endif
#ifndef AT_VARIANT
#define AT_VARIANT                      AT_VARIANT_DA16
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
#define AT_NOTIFY_WAKEUP                16 /* DPM Wake Up Notice                */


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
  uint8_t sec;
  char *country;
} AT_DATA_CWSAP;

/* Connect AP */
typedef struct {
  char    ssid[32+1]; /* SSID string */
  uint8_t bssid[6];   /* BSSID: AP MAC address */
  uint8_t ch;         /* Channel */
  uint8_t rssi;       /* Signal strength */
  char pwd[33];       /*password string*/
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
  
  \param[in]  enable  Echo enable(1) or disable(0)
  \return
  
*/
extern int32_t AT_Cmd_Echo (uint32_t enable);

/**
  Set/Query the current UART configuration

  Supported baudrates:[9600 ~ 921600] 

  \note 
  Default set is 115200 and Not Support Now
  
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
 
  \return
*/
extern int32_t AT_Resp_ConfigUART (uint32_t *baudrate);


/**
  Set maximum value of RF TX power [dBm] (set only command).

   NOTE : NOT SUPPORT

  \param[in]  tx_power  power value: range [0:82], units 0.25dBm
*/
extern int32_t AT_Cmd_TxPower (uint32_t tx_power);

/**
  Set/Query the current Wi-Fi mode

   Format S: AT+WFMODE=<mode>

  NOTE : SUPPORT only  Station Mode
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 0: station, 1: soft AP
  \return 0: OK, -1: error (invalid mode, etc)
*/
extern int32_t AT_Cmd_CurrentMode (uint32_t at_cmode, uint32_t mode);

/**
  Get response to CurrentMode command

  NOTE : SUPPORT only  Station Mode
  
  \param[in]  mode      Mode, 0: station, 1: soft AP
  \return
*/
extern int32_t AT_Resp_CurrentMode (uint32_t *mode);


/**
  Set/Query connected access point or access point to connect to.
  
  Format Q: AT+WFJAPA=<ssid>,<pwd>
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ssid
  \param[in]  pwd
  \return
*/
extern int32_t AT_Cmd_ConnectAP (uint32_t at_cmode, const char *ssid, const char *pwd, const char *bssid);

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

  Command: WFSAP

  NOTE : NOT SUPPORT NOW

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  cfg       AP configuration structure
  \return
*/
extern int32_t AT_Cmd_ConfigureAP (uint32_t at_cmode, AT_DATA_CWSAP *cfg);

/**
  Get response to ConfigureAP command

  NOTE : NOT SUPPORT NOW
  
  \param[in]  cfg       AP configuration structure
  \return
*/
extern int32_t AT_Resp_ConfigureAP (AT_DATA_CWSAP *cfg);


/**
  Retrieve the list of all available access points

  Command: WFSCAN
*/
extern int32_t AT_Cmd_ListAP (void);


/**
  Get response to ListAP command
  
  \param[out]   ap    Pointer to WFSCAN structure
  \param[in]   max_num Max scaned AP list number
  \return execution status
          - negative: error
          - 0: access point list is empty
          - 1: access point list contains more data
          
*/
extern int32_t AT_Resp_ListAP (AT_DATA_CWLAP *ap, uint8_t max_num);


/**
  Set/Query the station MAC address
 

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
  
    NOTE : NOT SUPPORT NOW

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
  \return 
*/
extern int32_t AT_Cmd_AccessPointMAC (uint32_t at_cmode, uint8_t mac[]);

/**
  Get response to AccessPointMAC command

    NOTE : NOT SUPPORT NOW

  \param[out]   mac   Pointer to 6 byte array where MAC address will be stored
  \return execution status
*/
extern int32_t AT_Resp_AccessPointMAC (uint8_t mac[]);


/**
  Set/Query current IP address of the local station

  Command: NWIP

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ip        IP address
  \param[in]  gw        Gateway address
  \param[in]  mask      Netmask
  \return 
*/
extern int32_t AT_Cmd_StationIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]);

/**
  Get response to StationIP command

  \param[out]   ip    Pointer to 4 byte array where the address is stored
  \param[out]   gateway    Pointer to 4 byte array where the address is stored
  \param[out]   netmask    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_StationIP (uint8_t ip_addr[], uint8_t nm_addr[],uint8_t gw_addr[]); 


/**
  Set/Query current IP address of the local access point

  Command: NWIP

  NOTE : NOT SUPPORT NOW  

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ip    IP address
  \param[in]  gw    Gateway address
  \param[in]  mask  Netmask
  \return 
*/
extern int32_t AT_Cmd_AccessPointIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]);

/**
  Get response to AccessPointIP command

  NOTE : NOT SUPPORT NOW
    
  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
extern int32_t AT_Resp_AccessPointIP (uint8_t ip_addr[], uint8_t nm_addr[],uint8_t gw_addr[]);


/**
  Set user defined DNS servers

    NOTE : NOT SUPPORT NOW
*/
extern int32_t AT_Cmd_DNS (uint32_t at_cmode, uint32_t enable, uint8_t dns0[], uint8_t dns1[]);


/**
  Get response to DNS command

   NOTE : NOT SUPPORT NOW
*/
extern int32_t AT_Resp_DNS (uint8_t addr[]);


/**
  Set/Query DHCP state

  Command: NWDHC

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    0: stop DHCP, 1: start DHCP
*/
extern int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t operate, uint32_t mode);


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

  Command: NWDHS

  NOTE : NOT SUPPORT NOW

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  en_tlease   bit[16] 0: disable setting, 1: enable setting the IP range
                          bits[15:0] lease time in minutes, range [1, 2880]
  \param[in]  ip_start    first IP in range that can be obtained from DHCP server
  \param[in]  ip_end      last IP in range that can be obtained from DHCP server
*/
extern int32_t AT_Cmd_RangeDHCP (uint32_t at_cmode, uint32_t en_tlease, uint8_t ip_start[], uint8_t ip_end[]);

/**
  Get response to RangeDHCP command

  NOTE : NOT SUPPORT NOW

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

  Command: WFDIS

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable     0:enable auto-connect on power-up, 1:disable

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
  Create or delete TCP server.

  Command: TRTS

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:delete server, 1:create server
  \param[in]  port      port number
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_TcpServer (uint32_t at_cmode, uint32_t mode, uint16_t port);


/**
  Retrieve incomming data.

  Response: +TRCTS:<C_ID>,<remote IP>,<remote port>

   \param[out]  remote_ip   remote IP
   \param[out]  remote_port remote port
  

  \return 0: ok, len of data shall be read from buffer
          negative: buffer empty or packet incomplete
*/

extern int32_t AT_Resp_TcpServer (uint8_t ip_addr[], uint16_t *port);


/**
  Retrieve incomming data.

  Response: +TRDTC:<C_ID>,<remote IP>,<remote port>,<len>,<data>
 
   This response does not have CRLF terminator, <len> is the number of bytes in <data>.
   Also note that the format of +IPD is also different in how argument are provided.
 
   \param[out]  link_id     connection ID   
   \param[out]  remote_ip   remote IP 
   \param[out]  remote_port remote port 
   \param[out]  len         data length
  

  \return 0: ok, len of data shall be read from buffer
          negative: buffer empty or packet incomplete
*/
extern int32_t AT_Resp_TRDTC (uint32_t *link_id, uint32_t *len, uint8_t *remote_ip, uint16_t *remote_port);


/**
  Get the connection status.
  
   Format: AT+TRPRT

  \param[in] cid         connection info  0: TCP SERVER, 1: TCP Client, 2: UDP
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_GetStatus (uint32_t at_cmode);

/**
  Get response to GetStatus command

    Format:  +TRPRT:<cid>,<type>,<remote IP>,<remote port>,<local port>    

    \param[out] conn         connection info

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_GetStatus (AT_DATA_LINK_CONN *conn);

/**
  Resolve IP address from host name.

  Format S: AT+NWHOST=<domain name>

  Example S: AT+NWHOST=www.renesas.com

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

  Establish TCP connection.

  Format S: AT+TRTC=<remote IP>,<remote port>
  
  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  ip          remote ip number
  \param[in]  port        remote port number
  
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnOpenTCP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t keep_alive);

/**
  Establish UDP transmission.

  Establish UDP transmission.
  
  Format S: AT+TRUR=<remote IP>,<remote port>

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)

  \param[in]  r_ip        remote ip number
  \param[in]  r_port      remote port number
  \return execution status:
          0: OK, -1 on error
*/
extern int32_t AT_Cmd_ConnUDP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t l_port, uint32_t mode);

/**
  Establish UDP transmission.

  Establish UDP transmission.
  
  Format S: AT+TRUSE=<local port>  
  
  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)  
  \param[in]  l_port      local port number 
  \return execution status:
          0: OK, -1 on error
*/

extern int32_t AT_Cmd_OpenUDP (uint32_t at_cmode, uint16_t l_port);

/**
  Close the TCP/UDP/SSL connection.

  Format S: AT+TRTRM=<c_id>    
  Example: AT+TRTRM=1;  // for TCP Client  

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  c_id     ID of the connection 0: TCP Server, 1: TCP Client, 2: UDP Session

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
extern uint32_t AT_Send_Data (char type, const uint8_t *buf, uint32_t len);


/**
  Check DPM Wake Up Type   
  
*/
extern int32_t AT_Resp_WakeUp(uint8_t *type);

/**
  Set/Query the DPM mode

  NOTE : To take effect, restart is needed.

  Command: AT+DPM=<mode>,<nvm>

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 0: Off, 1: On
  \param[in]  nvm      nvm, 0: not specified, 1: Write DPM mode to nvram only, and not reboot
  \return 0: OK, -1: error (invalid mode, etc)
*/
extern int32_t AT_Cmd_DPM (uint32_t at_cmode, uint8_t mode, uint8_t nvm);

/**
  Get response to dpm command

  \param[in]  mode      Mode, 0: Off, 1: On
  \return
*/
extern int32_t AT_Resp_DPM (uint32_t *mode);

/**
  Set the user application not to enter the DPM sleep

  Format S: AT+CLRDPMSLPEXT

  Response Q: AT_Resp_Gen
  
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_ClrDPMSleep(void);


/**
  Set the user application ready to enter the DPM sleep

  Format S: AT+SETDPMSLPEXT=<sleep_type>,<period>,<retain>

  Response Q: AT_Resp_Gen

  DPM Sleep 1 can be woken up by RTC_WAKE_UP or GPIO which has been assigned as a wake-up source.
  DPM Sleep 2 can be woken up by RTC_WAKE_UP.  
  DPM Sleep 3 can be woken up by an external wake-up signal
  
  \param[in]  sleep_type      1: sleep 1 mode, 2: sleep 2 mode  default :sleep 3 mode
  \param[in]  period          wake up time in second
  \param[in]  retain          1: retain,    2: not retain    
  
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_SetDPMSleep(uint8_t sleep_type, uint32_t period, uint8_t retain );

/**
  Set DPM keepalive period

  Format S: AT+DPMKA=<keepalive>

  Response Q: AT_Resp_Gen

  \param[in]  keepalive      0 ~ 600000 millisecond
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_DPMKA(uint32_t keepalive);

/**
  Set DPM TIM wake-up count

  Format S: AT+DPMTIMWU=<count>

  Response Q: AT_Resp_Gen

  \param[in]  count      1 ~ 65535 count
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_DPMTW(uint32_t count);


/**
  Set DPM user wake-up time

  Format S: AT+DPMUSERWU=<time>

  Response Q: AT_Resp_Gen

  \param[in]  time      0 ~ 86400 sec
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_DPMUW(uint32_t time);


/**
  Notify MCU wake-up done

  Format S: AT+MCUWUDONE

  Response Q: AT_Resp_Gen
  
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_MCUWUDone(void);

/* Socket control variable */

#define TCP_S   0
#define TCP_C   1
#define UDP     2

extern int tcps_link_id;
extern int tcp_link_id;
extern int udp_link_id;

extern char tcpc_session;
extern char tcps_session;
extern char udp_session ;

extern uint8_t r_ip[4];
extern uint16_t r_port;

extern uint8_t ur_ip[4];
extern uint16_t ur_port;
extern uint16_t ul_port;

extern uint16_t tcps_local_port;

#endif /* DA16200_H__ */
