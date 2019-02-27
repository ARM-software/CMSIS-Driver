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
 * $Date:        26. February 2019
 * $Revision:    V1.0 (beta)
 *
 * Driver:       Driver_WiFin (n = WIFI_QCA400x_DRIVER_INDEX value)
 * Project:      WiFi Driver for 
 *               Qualcomm QCA400x based WiFi Module
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   WIFI_QCA400x_DRIVER_INDEX: defines index of driver structure variable
 *     - default value:    0
 *   WIFI_QCA400x_CON_DISCON_TIMEOUT: defines maximum wait on WiFi connect
 *                         or disconnect
 *     - default value:    30000
 *   WIFI_QCA400x_RESOLVE_TIMEOUT: defines maximum wait on hostname resolve
 *     - default value:    5000
 *   WIFI_QCA400x_MAX_PACKET_LEN: defines maximum packet length (in bytes)
 *     - default value:    1576
 *   WIFI_QCA400x_SCAN_BUF_LEN: defines maximum length of buffer for scan 
 *                         result (in bytes)
 *     - default value:    128
 *   WIFI_QCA400x_AP_MAX_NUM: defines maximum number of stations that can 
 *                         connect to Access Point
 *     - default value:    8
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0 (beta)
 *    Initial beta version
 */


#include <stdint.h>
#include <string.h>

#include "a_config.h"
#include "atheros_wifi.h"
#include "atheros_stack_offload.h"
#include "qcom_api.h"

#include "cmsis_compiler.h"

#include "Driver_WiFi.h"

// Configuration definitions
#include "WiFi_QCA400x_Config.h"

// Local configuration definitions
#if   (!defined(DRIVER_CONFIG_ENDIANNESS) || (DRIVER_CONFIG_ENDIANNESS == A_LITTLE_ENDIAN))
#define SWAP(x)                            (__REV(x))
#else
#define SWAP(x)                            (x)
#endif

// Globally provided variable
QCA400x_WiFi wifiDev;


// WiFi Driver *****************************************************************

#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)        // Driver version

// Driver Version
static const ARM_DRIVER_VERSION driver_version = { ARM_WIFI_API_VERSION, ARM_WIFI_DRV_VERSION };

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = { 
  1U,                                   // Station: WiFi Protected Setup (WPS) supported
  1U,                                   // Access Point supported
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
  1U,                                   // Access Point: event generated on Station connect
#else
  0U,                                   // Access Point: event not generated on Station connect
#endif
  0U,                                   // Access Point: event not generated on Station disconnect
#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode supported (internal network stack is bypassed)
  1U,                                   // Bypass or pass-through mode (Ethernet interface) supported
  1U,                                   // Event generated on Ethernet frame reception in bypass mode
#else                                   // Enabled internal network stack (socket functions enabled)
  0U,                                   // Bypass or pass-through mode (Ethernet interface) not supported
  0U,                                   // Event not generated on Ethernet frame reception in bypass mode
#endif
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
  1U,                                   // IP (UDP/TCP) (Socket interface) supported
  0U,                                   // IPv6 (Socket interface) not supported
#else                                   // Pass-through mode supported (internal network stack is bypassed)
  0U,                                   // IP (UDP/TCP) (Socket interface) not supported
  0U,                                   // IPv6 (Socket interface) not supported
#endif
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
  1U,                                   // Ping (ICMP) supported
#else                                   // Pass-through mode supported (internal network stack is bypassed)
  0U,                                   // Ping (ICMP) not supported
#endif
  0U                                    // Reserved (must be zero)
};

typedef struct {                        // Socket structure
   int32_t handle;                      // QCOM socket handle
  uint32_t non_blocking;                // 0 = blocking, non-zero = non-blocking
  uint32_t send_timeout;                // Send Timeout
  uint32_t recv_timeout;                // Receive Timeout
  uint32_t type;                        // Type
  uint16_t local_port;                  // Local  port number
  uint16_t remote_port;                 // Remote port number
  uint8_t  remote_ip[16];               // Remote IP
} socket_t;

// Local variables and structures
static uint8_t                          driver_initialized = 0U;
static ARM_WIFI_SignalEvent_t           signal_event_fn;

static uint32_t                         tx_power;
static uint32_t                         ap_ssid_hidden;
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
static uint8_t                          ap_dhcp_server;
static uint32_t                         ap_ip_dhcp_begin;
static uint32_t                         ap_ip_dhcp_end;
static uint32_t                         ap_ip_dhcp_lease_time;
static uint8_t                          dhcp_client;
static uint32_t                         ip_address;
static uint32_t                         ip_submask;
static uint32_t                         ip_gateway;
static uint32_t                         ip_dns1;
static uint32_t                         ip_dns2;
static uint8_t                          ip6_dns1[16] __ALIGNED(4);
static uint8_t                          ip6_dns2[16] __ALIGNED(4);
static uint8_t                          mac_num;
#endif

static osEventFlagsId_t                 event_con_discon;
static uint8_t                          connected;
static uint8_t                          ap_running;
static uint8_t                          scan_buf[WIFI_QCA400x_SCAN_BUF_LEN] __ALIGNED(4);
static char                             char_buf[65]                        __ALIGNED(4);
static ARM_WIFI_MAC_IP4_t               mac_ip4 [WIFI_QCA400x_AP_MAX_NUM]   __ALIGNED(4);

#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
static socket_t                         socket_arr[MAX_SOCKETS_SUPPORTED];
#endif

#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)
#define NUM_RX_FRAME  16                // must be 2^n; must be < 256
#define NUM_TX_FRAME  4

static volatile uint8_t                 rx_q_head;
static volatile uint8_t                 rx_q_tail;

// Receive netbuf queue
static A_NETBUF *                       rx_netbuf_queue[NUM_RX_FRAME];

typedef struct {
  uint8_t         available;
  A_NATIVE_NETBUF pcb;
} tx_frame_t;

static uint8_t                          tx_buf[NUM_TX_FRAME][1576];
static tx_frame_t                       tx_frame[NUM_TX_FRAME];
static uint32_t                         tx_idx;
#endif

// Function prototypes
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
static int32_t WiFi_SocketRecvFrom (int32_t socket,       void *buf, uint32_t len,       uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t WiFi_SocketSendTo   (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t  ip_len, uint16_t  port);
static int32_t WiFi_SocketClose    (int32_t socket);
#endif


// Helper Functions

/**
  \fn          void ConnectCallback (uint8_t device_id, uint32_t value)
  \brief       Callback that is called when station connects or disconnects.
  \param [in]  value     Connect status
                 - value = 0:    disconnected
                 - value = 0x10: connected
  \param [in]  device_id Device ID
  \param [in]  bssid     Unused
  \param [in]  bssConn   Unused
*/
static void ConnectCallback (uint32_t value, uint8_t device_id, uint8_t *bssid, uint32_t bssConn) {
  (void)bssid;
  (void)bssConn;
  
  if (device_id == 0U) {
    if (value == 0x10U) {
      osEventFlagsSet(event_con_discon, 2U);
    }
    if (value == 0U) {
      osEventFlagsSet(event_con_discon, 1U);
    }
  }
}

#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
/**
  \fn          void AP_DhcpCallback (uint8_t device_id, uint32_t value)
  \brief       Callback that is called when station connects or disconnects.
  \param [in]  device_id Device ID
  \param [in]  value     Connect status
                 - value = 0: disconnected
                 - value = 1: connected
*/
static void AP_DhcpCallback (uint8_t *mac, uint32_t ip) {
  uint32_t val;
  uint8_t  i;

  if ((ap_dhcp_server) && (signal_event_fn != NULL)) {
    for (i = 0; i < WIFI_QCA400x_AP_MAX_NUM; i++) {
      val = *((uint32_t *)mac_ip4[i].mac) | *((uint32_t *)mac_ip4[i].ip4);
      if (val == 0U) {
        memcpy((void *)mac_ip4[i].mac, (void *)mac, 6);
        __UNALIGNED_UINT32_WRITE(mac_ip4[i].ip4, SWAP(ip));
        mac_num++;
        break;
      }
    }
    if (i < WIFI_QCA400x_AP_MAX_NUM) {
      signal_event_fn(ARM_WIFI_EVENT_AP_CONNECT, (void *)mac);
    }
  }
}
#endif

#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)
/**
  \fn          void WiFi_EthFrameReceived (A_NETBUF* a_netbuf_ptr)
  \brief       Callback that is called when frame is received in bypass mode.
  \param [in]  a_netbuf_ptr   Pointer to structure describing received frame
*/
void WiFi_EthFrameReceived (A_NETBUF *a_netbuf_ptr) {

  if (connected == 0U) {
    A_NETBUF_FREE(a_netbuf_ptr);
    return;
  }

  if ((uint8_t)(rx_q_head - rx_q_tail) >= NUM_RX_FRAME) {
    // Rx Frame Queue is full. Dump the Frame.
    A_NETBUF_FREE(a_netbuf_ptr);
  } else {
    // Add to Queue
    rx_netbuf_queue[rx_q_head & (NUM_RX_FRAME - 1)] = a_netbuf_ptr;
    rx_q_head++;

    if (signal_event_fn != NULL) {
      signal_event_fn (ARM_WIFI_EVENT_ETH_RX_FRAME, NULL);
    }
  }
}

/**
  \fn          void Free_TxBuf (void * param)
  \brief       Free Transmit buffer
*/
static void Free_TxBuf (void * param) {
  uint32_t idx = (uint32_t)(((A_NATIVE_NETBUF *) param)->PRIVATE);
  tx_frame[idx].available = 1;
}
#endif


// Driver Functions

/**
  \fn          ARM_DRIVER_VERSION WiFi_GetVersion (void)
  \brief       Get driver version.
  \return      ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION WiFi_GetVersion (void) { return driver_version; }

/**
  \fn          ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      ARM_WIFI_CAPABILITIES
*/
static ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void) { return driver_capabilities; }

/**
  \fn          int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event)
  \brief       Initialize WiFi Interface.
  \param [in]  cb_event Pointer to ARM_WIFI_SignalEvent
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event) {
  int32_t  ret;
#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)
  uint32_t i;
#endif

  ret = ARM_DRIVER_OK;

  signal_event_fn = cb_event;

  if (!driver_initialized) {
    // Initialize all local variables
    connected             = 0U;
    ap_running            = 0U;

    tx_power              = 1U;
    ap_ssid_hidden        = 0U;

    memset((void *)mac_ip4,  0, sizeof(mac_ip4));
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
    dhcp_client           = 1U;
    ap_dhcp_server        = 0U;

    mac_num               = 0U;
    ap_ip_dhcp_begin      = 0U;
    ap_ip_dhcp_end        = 0U;
    ap_ip_dhcp_lease_time = 0U;
    ip_address            = 0U;
    ip_submask            = 0U;
    ip_gateway            = 0U;
    ip_dns1               = 0U;
    ip_dns2               = 0U;

    memset((void *)ip6_dns1, 0, sizeof(ip6_dns1));
    memset((void *)ip6_dns2, 0, sizeof(ip6_dns2));
    memset((void *)socket_arr, 0, sizeof(socket_arr));
#endif
#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)
    rx_q_head = 0U;
    rx_q_tail = 0U;
    tx_idx    = 0U;
    
    for (i = 0; i < NUM_TX_FRAME; i++) {
      tx_frame[i].available            = 1;
      tx_frame[i].pcb.FREE             = &Free_TxBuf;
      tx_frame[i].pcb.PRIVATE          = (void *) i;
      tx_frame[i].pcb.FRAG[0].FRAGMENT = tx_buf[i];
      tx_frame[i].pcb.FRAG[0].LENGTH   = 0;
    }

    wifiDev.FrameReceived_cb = WiFi_EthFrameReceived;
#endif

    event_con_discon      = osEventFlagsNew(NULL);
    if (event_con_discon == NULL) {
      ret = ARM_DRIVER_ERROR;
    }

    if (ret == ARM_DRIVER_OK) {
      if (ATHEROS_WIFI_IF.INIT(&wifiDev) == A_OK) {
        // If initialization succeeded
        driver_initialized = 1U;
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  if (ret != ARM_DRIVER_OK) {
    // If something failed during initialization clean-up (release all resources)
    if (event_con_discon != NULL) {
      if (osEventFlagsDelete(event_con_discon) == osOK) {
        event_con_discon = NULL;
      }
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_Uninitialize (void)
  \brief       De-initialize WiFi Interface.
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Uninitialize (void) {
  int32_t ret;
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
  uint8_t i;
#endif

  ret = ARM_DRIVER_OK;

  if (driver_initialized) {
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
    // Close all open sockets
    for (i = 0U; i < MAX_SOCKETS_SUPPORTED; i++) {
      if (socket_arr[i].handle != NULL) {
        if (WiFi_SocketClose (socket_arr[i].handle) != 0) {
          ret = ARM_DRIVER_ERROR;
          break;
        }
      }
    }
#endif

    if (ret == ARM_DRIVER_OK) {
      if (ATHEROS_WIFI_IF.STOP(&wifiDev) == A_OK) {
        // If uninitialization succeeded
        signal_event_fn    = NULL;
        driver_initialized = 0U;
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }

    if (ret == ARM_DRIVER_OK) {
      if (event_con_discon != NULL) {
        if (osEventFlagsDelete(event_con_discon) == osOK) {
          event_con_discon = NULL;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_PowerControl (ARM_POWER_STATE state)
  \brief       Control WiFi Interface Power.
  \param [in]  state    Power state
                 - ARM_POWER_OFF                : Power off: no operation possible
                 - ARM_POWER_LOW                : Low power mode: retain state, detect and signal wake-up events
                 - ARM_POWER_FULL               : Power on: full operation at maximum performance
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
*/
static int32_t WiFi_PowerControl (ARM_POWER_STATE state) {
  int32_t  ret;

  ret = ARM_DRIVER_OK;

  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    switch (state) {
      case ARM_POWER_OFF:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
      case ARM_POWER_LOW:
        if (qcom_power_set_mode(0, REC_POWER, USER) != A_OK) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_POWER_FULL:
        if (qcom_power_set_mode(0, MAX_PERF_POWER, USER) != A_OK) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SetOption (uint32_t option, const void *data, uint32_t len)
  \brief       Set WiFi Interface Options.
  \param [in]  option   Option to set
  \param [in]  data     Pointer to data relevant to selected option
  \param [in]  len      Length of data (in bytes)
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL data pointer or len less than option specifies)
*/
static int32_t WiFi_SetOption (uint32_t option, const void *data, uint32_t len) {
  int32_t  ret;
  uint32_t val;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((data == NULL) || (len < 4U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (option) {
      case ARM_WIFI_TX_POWER:                           // Station Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_AP_TX_POWER:                        // AP      Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if ((val > 0U) || (val < 18U)) {
            if (qcom_set_tx_power(0U, val) == A_OK) {
              tx_power = val;
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR_PARAMETER;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_AP_SSID_HIDE:                       // AP      Set/Get SSID hide option;                data = &en,       len =  4, en       (uint32_t): 0 = disable (default), non-zero = enable
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_ap_hidden_mode_enable(0U, val) == A_OK) {
            ap_ssid_hidden = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

#if (WIFI_QCA400x_MODE_INT_STACK)                       // Enabled internal network stack (socket functions enabled)
      case ARM_WIFI_AP_IP_DHCP_POOL_BEGIN:              // AP      Set/Get IPv4 DHCP pool begin address;    data = &ip,       len =  4, ip       (uint8_t[4])
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_dhcps_set_pool(0U, val, ap_ip_dhcp_end, ap_ip_dhcp_lease_time) == A_OK) {
            ap_ip_dhcp_begin = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_AP_IP_DHCP_POOL_END:                // AP      Set/Get IPv4 DHCP pool end address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_dhcps_set_pool(0U, ap_ip_dhcp_begin, val, ap_ip_dhcp_lease_time) == A_OK) {
            ap_ip_dhcp_end = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:              // AP      Set/Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t)
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_dhcps_set_pool(0U, ap_ip_dhcp_begin, ap_ip_dhcp_end, val) == A_OK) {
            ap_ip_dhcp_lease_time = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP:                                 // Station Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP:                              // AP      Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
        // Unclear from documentation for AP
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_ipconfig(0U, IPCFG_STATIC, &val, &ip_submask, &ip_gateway) == A_OK) {
            ip_address  = val;
            dhcp_client = 0U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP_SUBNET_MASK:                     // Station Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_AP_IP_SUBNET_MASK:                  // AP      Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
        // Unclear from documentation for AP
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_ipconfig(0U, ((dhcp_client) ? IPCFG_DHCP : IPCFG_STATIC), &ip_address, &val, &ip_gateway) == A_OK) {
            ip_submask = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP_GATEWAY:                         // Station Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_GATEWAY:                      // AP      Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
        // Unclear from documentation for AP
        if (len != 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_ipconfig(0U, ((dhcp_client) ? IPCFG_DHCP : IPCFG_STATIC), &ip_address, &ip_submask, &val) == A_OK) {
            ip_gateway = val;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP_DNS1:                            // Station Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS1:                         // AP      Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (ip_dns1 != 0U) {
            // Remove previous DNS1 address
            if (qcom_dnsc_del_server_address((uint8_t *)&ip_dns1, ATH_AF_INET) != A_OK) {
              ret = ARM_DRIVER_ERROR;
            }
          }
          if (ret == ARM_DRIVER_OK) {
            if (qcom_dnsc_add_server_address((uint8_t *)&val, ATH_AF_INET) == A_OK) {
              if (qcom_dnsc_enable(1) == A_OK) {
                ip_dns1 = val;
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_DNS1:                           // Station Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS1:                        // AP      Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (len == 16U) {
          val = *((uint32_t *)&ip6_dns1[0]) | *((uint32_t *)&ip6_dns1[4]) | *((uint32_t *)&ip6_dns1[8]) | *((uint32_t *)&ip6_dns1[12]);
          if (val != 0U) {
            // Remove previous DNS1 address
            if (qcom_dnsc_del_server_address((uint8_t *)&ip_dns1, ATH_AF_INET6) != A_OK) {
              ret = ARM_DRIVER_ERROR;
            }
          }
          if (ret == ARM_DRIVER_OK) {
            if (qcom_dnsc_add_server_address((uint8_t *)data, ATH_AF_INET6) == A_OK) {
              if (qcom_dnsc_enable(1) == A_OK) {
                memcpy((void *)ip6_dns1, (void *)data, 16);
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP_DNS2:                            // Station Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS2:                         // AP      Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (ip_dns1 != 0U) {
            // Remove previous DNS2 address
            if (qcom_dnsc_del_server_address((uint8_t *)&ip_dns2, ATH_AF_INET) != A_OK) {
              ret = ARM_DRIVER_ERROR;
            }
          }
          if (ret == ARM_DRIVER_OK) {
            if (qcom_dnsc_add_server_address((uint8_t *)&val, ATH_AF_INET) == A_OK) {
              if (qcom_dnsc_enable(1) == A_OK) {
                ip_dns2 = val;
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_DNS2:                           // Station Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS2:                        // AP      Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (len == 16U) {
          val = *((uint32_t *)&ip6_dns2[0]) | *((uint32_t *)&ip6_dns2[4]) | *((uint32_t *)&ip6_dns2[8]) | *((uint32_t *)&ip6_dns2[12]);
          if (val != 0U) {
            // Remove previous DNS2 address
            if (qcom_dnsc_del_server_address((uint8_t *)&ip_dns2, ATH_AF_INET6) != A_OK) {
              ret = ARM_DRIVER_ERROR;
            }
          }
          if (ret == ARM_DRIVER_OK) {
            if (qcom_dnsc_add_server_address((uint8_t *)data, ATH_AF_INET6) == A_OK) {
              if (qcom_dnsc_enable(1) == A_OK) {
                memcpy((void *)ip6_dns2, (void *)data, 16);
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP_DHCP:                            // Station Set/Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (qcom_ipconfig(0U, ((val != 0U) ? IPCFG_DHCP : IPCFG_STATIC), &ip_address, &ip_submask, &ip_gateway) == A_OK) {
            dhcp_client = (val != 0U);
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_AP_IP_DHCP:                         // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (len == 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          if (val != 0U) {
            if (qcom_dhcps_set_pool(0U, ap_ip_dhcp_begin, ap_ip_dhcp_end, ap_ip_dhcp_lease_time) == A_OK) {
              if (qcom_dhcps_register_cb(0, (void *)AP_DhcpCallback) == A_OK) {
                ap_dhcp_server = 1U;
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            if (qcom_dhcps_release_pool(0U) == A_OK) {
              ap_dhcp_server = 0U;
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_DHCP_MODE:                      // Station Set/Get IPv6 DHCPv6 client mode;         data = &mode,     len =  4, mode     (uint32_t): ARM_WIFI_IP6_DHCP_xxx
        if (len != 4U) {
          val = __UNALIGNED_UINT32_READ(data);
          switch (val) {
            case ARM_WIFI_IP6_DHCP_OFF:
            case ARM_WIFI_IP6_DHCP_STATELESS:
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
            case ARM_WIFI_IP6_DHCP_STATEFULL:
              // Only available and supported mode
              break;
            default:
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_GLOBAL:                         // Station Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_LINK_LOCAL:                     // Station Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:              // Station Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_IP6_GATEWAY:                        // Station Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_GLOBAL:                      // AP      Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_LINK_LOCAL:                  // AP      Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_SUBNET_PREFIX_LEN:           // AP      Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_AP_IP6_GATEWAY:                     // AP      Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_MAC:                                // Station Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
      case ARM_WIFI_AP_MAC:                             // AP      Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
#endif
      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_GetOption (uint32_t option, void *data, uint32_t *len)
  \brief       Get WiFi Interface Options.
  \param [in]  option   Option to get
  \param [out] data     Pointer to memory where data for selected option will be returned
  \param [in,
          out] len      Pointer to length of data (input/output)
                 - input: maximum length of data that can be returned (in bytes)
                 - output: length of returned data (in bytes)
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL data or len pointer, or *len less than option specifies)
*/
static int32_t WiFi_GetOption (uint32_t option, void *data, uint32_t *len) {
  uint32_t u32;
  int32_t  ret;
  uint16_t u16;
  uint8_t   u8;
#if (WIFI_QCA400x_MODE_INT_STACK)                       // Enabled internal network stack (socket functions enabled)
  int32_t  val, dummy_val;
  uint8_t  dummy[16];
#endif

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((data == NULL) || (len == NULL) || (*len < 4U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (option) {
      case ARM_WIFI_SSID:                               // Station     Get SSID of connected AP;            data = &ssid,     len<= 33, ssid     (char[32+1]), null-terminated string
        if (*len > 0U) {
          if (qcom_get_ssid(0U, char_buf) == A_OK) {
            u32 = strlen(char_buf);
            if (u32 > *len) {
              u32 = *len;
            }
            if (u32 > 32U) {
              u32 = 32U;
            }
            memcpy((void *)data, (void *)char_buf, u32);
            u32++;
            *((uint8_t *)&data + u32) = 0;
            *len = u32;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_PASS:                               // Station     Get Password of connected AP;        data = &pass,     len<= 65, pass     (char[64+1]), null-terminated string
        // Because qcom_sec_get_passphrase function is not provided in the SDK
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_SECURITY:                           // Station     Get Security Type of connected AP;   data = &security, len =  4, security (uint32_t): ARM_WIFI_SECURITY_xxx
        // Because qcom_sec_get_auth_mode function is not provided in the SDK
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_CHANNEL:                            // Station     Get Channel of connected AP;         data = &ch,       len =  4, ch       (uint32_t)
        if (*len == 4U) {
          if (qcom_get_channel(0U, &u16) == A_OK) {
            __UNALIGNED_UINT32_WRITE(data, (uint32_t)u16);
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_RSSI:                               // Station     Get RSSI of connected AP;            data = &rssi,     len =  4, rssi     (uint32_t)
        if (*len == 4U) {
          if (qcom_sta_get_rssi(0U, &u8) == A_OK) {
            __UNALIGNED_UINT32_WRITE(data, (uint32_t)u8);
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_TX_POWER:                           // Station Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_AP_TX_POWER:                        // AP      Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
        __UNALIGNED_UINT32_WRITE(data, tx_power);
        *len = 4U;
        break;

      case ARM_WIFI_AP_SSID_HIDE:                       // AP      Set/Get SSID hide option;                data = &en,       len =  4, en       (uint32_t): 0 = disable (default), non-zero = enable
        __UNALIGNED_UINT32_WRITE(data, ap_ssid_hidden);
        *len = 4U;
        break;

#if (WIFI_QCA400x_MODE_INT_STACK)                       // Enabled internal network stack (socket functions enabled)
      case ARM_WIFI_AP_IP_DHCP_POOL_BEGIN:              // AP      Set/Get IPv4 DHCP pool begin address;    data = &ip,       len =  4, ip       (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ap_ip_dhcp_begin);
        *len = 4U;
        break;

      case ARM_WIFI_AP_IP_DHCP_POOL_END:                // AP      Set/Get IPv4 DHCP pool end address;      data = &ip,       len =  4, ip       (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ap_ip_dhcp_end);
        *len = 4U;
        break;

      case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:              // AP      Set/Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t)
        __UNALIGNED_UINT32_WRITE(data, ap_ip_dhcp_lease_time);
        *len = 4U;
        break;
#endif

      case ARM_WIFI_BSSID:                              // Station     Get BSSID of connected AP;           data = &bssid,    len =  6, bssid    (uint8_t[6])
      case ARM_WIFI_MAC:                                // Station Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
      case ARM_WIFI_AP_MAC:                             // AP      Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if (*len >= 6U) {
          if (qcom_get_bssid(0U, (uint8_t *)data) == A_OK) {
            *len = 6U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

#if (WIFI_QCA400x_MODE_INT_STACK)                       // Enabled internal network stack (socket functions enabled)
      case ARM_WIFI_IP:                                 // Station Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
        if (qcom_ipconfig(0U, IPCFG_QUERY, &u32, (uint32_t *)&dummy_val, (uint32_t *)&dummy_val) == A_OK) {
          __UNALIGNED_UINT32_WRITE(data, SWAP(u32));
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
        break;

      case ARM_WIFI_AP_IP:                              // AP      Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
        // Unclear from documentation
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_IP_SUBNET_MASK:                     // Station Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ip_submask);
        *len = 4U;
        break;

      case ARM_WIFI_AP_IP_SUBNET_MASK:                  // AP      Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
        // Unclear from documentation
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_IP_GATEWAY:                         // Station Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ip_gateway);
        *len = 4U;
        break;

      case ARM_WIFI_AP_IP_GATEWAY:                      // AP      Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
        // Unclear from documentation
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_IP_DNS1:                            // Station Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS1:                         // AP      Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ip_dns1);
        *len = 4U;
        break;

      case ARM_WIFI_IP_DNS2:                            // Station Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS2:                         // AP      Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        __UNALIGNED_UINT32_WRITE(data, ip_dns2);
        *len = 4U;
        break;

      case ARM_WIFI_IP_DHCP:                            // Station Set/Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (dhcp_client) {
          __UNALIGNED_UINT32_WRITE(data, 1U);
        } else {
          __UNALIGNED_UINT32_WRITE(data, 0U);
        }
        *len = 4U;
        break;

      case ARM_WIFI_AP_IP_DHCP:                         // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (ap_dhcp_server) {
          __UNALIGNED_UINT32_WRITE(data, 1U);
        } else {
          __UNALIGNED_UINT32_WRITE(data, 0U);
        }
        *len = 4U;
        break;

      case ARM_WIFI_IP6_GLOBAL:                         // Station Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_GLOBAL:                      // AP      Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (*len >= 16U) {
          if (qcom_ip6_address_get(0, (uint8_t *)data, (uint8_t *)dummy, (uint8_t *)dummy, (uint8_t *)dummy, &dummy_val, &dummy_val, &dummy_val, &dummy_val) == A_OK) {
            *len = 16U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_LINK_LOCAL:                     // Station Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_LINK_LOCAL:                  // AP      Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (*len >= 16U) {
          if (qcom_ip6_address_get(0, (uint8_t *)dummy, (uint8_t *)data, (uint8_t *)dummy, (uint8_t *)dummy, &dummy_val, &dummy_val, &dummy_val, &dummy_val) == A_OK) {
            *len = 16U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:              // Station Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_AP_IP6_SUBNET_PREFIX_LEN:           // AP      Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
        if (*len >= 16U) {
          if (qcom_ip6_address_get(0, (uint8_t *)dummy, (uint8_t *)dummy, (uint8_t *)dummy, (uint8_t *)dummy, &dummy_val, &val, &dummy_val, &dummy_val) == A_OK) {
            __UNALIGNED_UINT32_WRITE(data, (uint32_t)val);
            *len = 16U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_GATEWAY:                        // Station Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_GATEWAY:                     // AP      Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (*len >= 16U) {
          if (qcom_ip6_address_get(0, (uint8_t *)dummy, (uint8_t *)dummy, (uint8_t *)data, (uint8_t *)dummy, &dummy_val, &dummy_val, &dummy_val, &dummy_val) == A_OK) {
            *len = 16U;
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;

      case ARM_WIFI_IP6_DNS1:                           // Station Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS1:                        // AP      Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (*len >= 16U) {
          memcpy((void *)data, (void*)ip6_dns1, 16);
          *len = 16U;
        }
        break;

      case ARM_WIFI_IP6_DNS2:                           // Station Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS2:                        // AP      Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        if (*len >= 16U) {
          memcpy((void *)data, (void*)ip6_dns2, 16);
          *len = 16U;
        }
        break;

      case ARM_WIFI_IP6_DHCP_MODE:                      // Station Set/Get IPv6 DHCPv6 client mode;         data = &mode,     len =  4, mode     (uint32_t): ARM_WIFI_IP6_DHCP_xxx
        __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_IP6_DHCP_STATEFULL);
        break;

      case ARM_WIFI_AP_IP_DHCP_TABLE:                   // AP          Get IPv4 DHCP table;                 data = &mac_ip4[],len = sizeof(mac_ip4[]), mac_ip4 (array of ARM_WIFI_MAC_IP4_t structures)
        if (*len >= sizeof(ARM_WIFI_MAC_IP4_t)) {
          if (*len > (mac_num * sizeof(ARM_WIFI_MAC_IP4_t))) {
            *len = (mac_num * sizeof(ARM_WIFI_MAC_IP4_t));
          }
          memcpy((void *)data, (void*)mac_ip4, *len);
        }
        break;
#endif

      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_Scan (ARM_WIFI_AP_INFO_t ap_info[], uint32_t max_num)
  \brief       Scan for Access Points in range.
  \param [out] ap_info  Pointer to array of ARM_WIFI_AP_INFO_t structures where Access Point Information will be returned
  \param [in]  max_num  Maximum number of Access Point information structures to return
  \return      number of ARM_WIFI_AP_INFO_t structures returned or error code
                 - value >= 0                   : Number of ARM_WIFI_AP_INFO_t structures returned
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ap_info pointer or max_num equal to 0)
*/
static int32_t WiFi_Scan (ARM_WIFI_AP_INFO_t ap_info[], uint32_t max_num) {
  int32_t                  ret, i;
  int16_t                  num;
  qcom_start_scan_params_t params;
  QCOM_BSS_SCAN_INFO      *ptr_info;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ap_info == NULL) || (max_num == 0U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  } else if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    memset((void *)&params, 0, sizeof(qcom_start_scan_params_t));
    params.forceFgScan = 1;
    if (qcom_set_ssid(0U, " ") == A_OK) {
      if (_qcom_set_scan(0U, &params) == A_OK) {
        if (qcom_get_scan(0U, (QCOM_BSS_SCAN_INFO **)&scan_buf, &num) == A_OK) {
          ptr_info = (QCOM_BSS_SCAN_INFO *)&scan_buf;
          if (num > (int32_t)max_num) {
            num = (int32_t)max_num;
          }
          for (i = 0; i < num; i++) {
            // Extract SSID
            memset((void *)ap_info[i].ssid, 0, sizeof(ap_info[i].ssid));
            memcpy((void *)ap_info[i].ssid, (void *)ptr_info->ssid, ptr_info->ssid_len);

            // Extract BSSID
            memcpy((void *)ap_info[i].bssid, (void *)ptr_info->bssid, 6);

            // Extract security
            switch (ptr_info->wpa_auth) {
              case WLAN_AUTH_NONE:
                ap_info[i].security = ARM_WIFI_SECURITY_OPEN;
                break;
              case WLAN_AUTH_WEP:
                ap_info[i].security = ARM_WIFI_SECURITY_WEP;
                break;
              case WLAN_AUTH_WPA:
              case WLAN_AUTH_WPA_PSK:
              case WLAN_AUTH_WPA_CCKM:
                ap_info[i].security = ARM_WIFI_SECURITY_WPA;
                break;
              case WLAN_AUTH_WPA2:
              case WLAN_AUTH_WPA2_PSK:
              case WLAN_AUTH_WPA2_CCKM:
              case WLAN_AUTH_WPA2_PSK_SHA256:
                ap_info[i].security = ARM_WIFI_SECURITY_WPA2;
                break;
              case WLAN_AUTH_INVALID:
              default:
                ap_info[i].security = 0xFFU;
                break;
            }

            // Extract RSSI
            ap_info[i].rssi = ptr_info->rssi;

            // Extract channel
            ap_info[i].ch = ptr_info->channel;

            ptr_info++;
          }
          ret = num;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_Connect (const char *ssid, const char *pass, uint8_t security, uint8_t ch)
  \brief       Connect Station to Access Point (join the AP).
  \param [in]  ssid     Pointer to Service Set Identifier (SSID) null-terminated string
  \param [in]  pass     Pointer to password null-terminated string
  \param [in]  security Security standard used
                 - ARM_WIFI_SECURITY_OPEN       : Unsecured
                 - ARM_WIFI_SECURITY_WEP        : Wired Equivalent Privacy (WEP)
                 - ARM_WIFI_SECURITY_WPA        : WiFi Protected Access (WPA)
                 - ARM_WIFI_SECURITY_WPA2       : WiFi Protected Access II (WPA2)
  \param [in]  ch       Channel
                 - value = 0: autodetect
                 - value > 0: exact channel to connect on
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported (security type or channel autodetect not supported)
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ssid pointer, or NULL pass pointer if security different then ARM_WIFI_SECURITY_OPEN or invalid security parameter)
*/
static int32_t WiFi_Connect (const char *ssid, const char *pass, uint8_t security, uint8_t ch) {
  int32_t         ret;
  uint32_t        evt;
  WLAN_CRYPT_TYPE crypt_type;
  WLAN_AUTH_MODE  auth_mode;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL)) || (ch > 13U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        break;
      case ARM_WIFI_SECURITY_WEP:
        break;
      case ARM_WIFI_SECURITY_WPA:
        break;
      case ARM_WIFI_SECURITY_WPA2:
        break;
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (qcom_set_connect_callback(0U, &ConnectCallback) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_op_set_mode(0U, QCOM_WLAN_DEV_MODE_STATION) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_set_ssid(0U, (char *)ssid) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_sec_set_passphrase(0U, (char *)pass) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        crypt_type = WLAN_CRYPT_NONE;
        auth_mode  = WLAN_AUTH_NONE;
        break;
      case ARM_WIFI_SECURITY_WEP:
        crypt_type = WLAN_CRYPT_WEP_CRYPT;
        auth_mode  = WLAN_AUTH_WEP;
        break;
      case ARM_WIFI_SECURITY_WPA:
        crypt_type = WLAN_CRYPT_AES_CRYPT;
        auth_mode  = WLAN_AUTH_WPA_PSK;
        break;
      case ARM_WIFI_SECURITY_WPA2:
        crypt_type = WLAN_CRYPT_AES_CRYPT;
        auth_mode  = WLAN_AUTH_WPA2_PSK;
        break;
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
    if ((ret == ARM_DRIVER_OK) && (qcom_sec_set_encrypt_mode(0, crypt_type) != A_OK)) {
      ret = ARM_DRIVER_ERROR;
    }
    if ((ret == ARM_DRIVER_OK) && (qcom_sec_set_auth_mode(0U, auth_mode)) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (ch != 0U) {
      if (qcom_set_channel(0U, (uint16_t)ch) != A_OK) {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_commit(0U) == A_OK) {
      // Wait for connect event
      evt = osEventFlagsWait(event_con_discon, 2U, osFlagsWaitAny, WIFI_QCA400x_CON_DISCON_TIMEOUT);
      // If evt == 2 then connect has succeeded
      if        (evt == 2U) {
        connected  = 1U;
        ap_running = 0U;

#if (WIFI_QCA400x_MODE_INT_STACK)                       // Enabled internal network stack (socket functions enabled)
        // Enable DHCP if needed
        if (qcom_ipconfig(0U, dhcp_client ? IPCFG_DHCP : IPCFG_STATIC, &ip_address, &ip_submask, &ip_gateway) != A_OK) {
          ret = ARM_DRIVER_ERROR;
        }
#endif

      } else if (evt == osFlagsErrorTimeout) {
        ret = ARM_DRIVER_ERROR_TIMEOUT;
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_ConnectWPS (const char *pin)
  \brief       Connect Station to Access Point via WiFi Protected Setup (WPS). Access Point information can be retrieved through 
               GetOption function with ARM_WIFI_INFO_AP option.
  \param [in]  pin      Pointer to pin null-terminated string or push-button connection trigger
                 - value != NULL: pointer to pin null-terminated string
                 - value == NULL: push-button connection trigger
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
*/
static int32_t WiFi_ConnectWPS (const char *pin) {
  int32_t  ret;
  uint32_t evt;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (*pin != NULL) {                 // If pin connection requested
      if (qcom_wps_start(0U, 1U, 0U, (char *)pin) != A_OK) {
        ret = ARM_DRIVER_ERROR;
      }
    } else {                            // If push-button connection requested
      if (qcom_wps_start(0U, 1U, 1U, NULL) != A_OK) {
        ret = ARM_DRIVER_ERROR;
      }
    }

    // Wait for connect event
    evt = osEventFlagsWait(event_con_discon, 2U, osFlagsWaitAny, WIFI_QCA400x_CON_DISCON_TIMEOUT);
    // If evt == 2 then connect has succeeded
    if        (evt == 2U) {
      connected = 1U;
    } else if (evt == osFlagsErrorTimeout) {
      ret = ARM_DRIVER_ERROR_TIMEOUT;
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_Disconnect (void)
  \brief       Disconnect Station from currently connected Access Point.
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Disconnect (void) {
  int32_t  ret;
  uint32_t evt;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (qcom_disconnect(0U) == A_OK) {
      // Wait for disconnect event
      evt = osEventFlagsWait(event_con_discon, 1U, osFlagsWaitAny, WIFI_QCA400x_CON_DISCON_TIMEOUT);
      // If evt == 1 then disconnect has succeeded
      if        (evt == 1U) {
        connected = 0U;
      } else if (evt == osFlagsErrorTimeout) {
        ret = ARM_DRIVER_ERROR_TIMEOUT;
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_IsConnected (void)
  \brief       Check Station connection status.
  \return      connection status
                 - value != 0: connected
                 - value = 0: not connected
*/
static int32_t WiFi_IsConnected (void) {
  return (int32_t)connected;
}

/**
  \fn          int32_t WiFi_AP_Start (const char *ssid, const char *pass, uint8_t security, uint8_t ch)
  \brief       Start Access Point.
  \param [in]  ssid     Pointer to Service Set Identifier (SSID) null-terminated string
  \param [in]  pass     Pointer to password null-terminated string
  \param [in]  security Security standard used
                 - ARM_WIFI_SECURITY_OPEN       : Unsecured
                 - ARM_WIFI_SECURITY_WEP        : Wired Equivalent Privacy (WEP)
                 - ARM_WIFI_SECURITY_WPA        : WiFi Protected Access (WPA)
                 - ARM_WIFI_SECURITY_WPA2       : WiFi Protected Access II (WPA2)
  \param [in]  ch       Channel
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported (security type or channel autodetect not supported)
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ssid pointer, or NULL pass pointer if security different then ARM_WIFI_SECURITY_OPEN or invalid security parameter)
*/
static int32_t WiFi_AP_Start (const char *ssid, const char *pass, uint8_t security, uint8_t ch) {
  int32_t        ret;
  WLAN_AUTH_MODE mode;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL)) || (ch > 13U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        break;
      case ARM_WIFI_SECURITY_WEP:
        break;
      case ARM_WIFI_SECURITY_WPA:
        break;
      case ARM_WIFI_SECURITY_WPA2:
        break;
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (qcom_set_connect_callback(0U, &ConnectCallback) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_op_set_mode(0U, QCOM_WLAN_DEV_MODE_AP) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_set_ssid(0U, (char *)ssid) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_sec_set_passphrase(0U, (char *)pass) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        mode = WLAN_AUTH_NONE;
        break;
      case ARM_WIFI_SECURITY_WEP:
        mode = WLAN_AUTH_WEP;
        break;
      case ARM_WIFI_SECURITY_WPA:
        mode = WLAN_AUTH_WPA_PSK;
        break;
      case ARM_WIFI_SECURITY_WPA2:
        mode = WLAN_AUTH_WPA2_PSK;
        break;
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }
    if ((ret == ARM_DRIVER_OK) && (qcom_sec_set_auth_mode(0U, mode)) != A_OK) {
      ret = ARM_DRIVER_ERROR;
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (ch != 0U) {
      if (qcom_set_channel(0U, (uint16_t)ch) != A_OK) {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }
  if (ret == ARM_DRIVER_OK) {
    if (qcom_commit(0U) == A_OK) {
      ap_running = 1U;
      connected  = 0U;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_AP_Stop (void)
  \brief       Stop Access Point.
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
*/
static int32_t WiFi_AP_Stop (void) {
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t WiFi_AP_IsRunning (void)
  \brief       Check Access Point running status.
  \return      running status
                 - value != 0: running
                 - value = 0: not running
*/
static int32_t WiFi_AP_IsRunning (void) {
  return (int32_t)ap_running;
}

#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)

/**
  \fn          int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol)
  \brief       Create a communication socket.
  \param [in]  af       Address family
  \param [in]  type     Socket type
  \param [in]  protocol Socket protocol
  \return      status information
                 - Socket identification number (>=0)
                 - ARM_SOCKET_EINVAL            : Invalid argument
                 - ARM_SOCKET_ENOTSUP           : Operation not supported
                 - ARM_SOCKET_ENOMEM            : Not enough memory
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol) {
  int32_t ret, handle, family, type_;
  uint8_t i;

  ret = 0;

  // Check parameters
  if (ret == 0) {
    switch (af) {
      case ARM_SOCKET_AF_INET:
        family = ATH_AF_INET;
        break;
      case ARM_SOCKET_AF_INET6:
        family = ATH_AF_INET6;
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
  }
  if (ret == 0) {
    switch (type) {
      case ARM_SOCKET_SOCK_DGRAM:
        type_ = SOCK_DGRAM_TYPE;
        break;
      case ARM_SOCKET_SOCK_STREAM:
        type_ = SOCK_STREAM_TYPE;
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
  }
  if (ret == 0) {
    switch (protocol) {
      case ARM_SOCKET_IPPROTO_TCP:
        break;
      case ARM_SOCKET_IPPROTO_UDP:
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
  }

  if (ret == 0) {
    // Find free local socket entry in socket_arr
    for (i = 0U; i < MAX_SOCKETS_SUPPORTED; i++) {
      if (socket_arr[i].handle == NULL) {
        break;
      }
    }
    if (i == MAX_SOCKETS_SUPPORTED) {
      // No free entry is available
      ret = ARM_SOCKET_ERROR;
    }
  }

  if (ret == 0) {
    handle = qcom_socket(family, type_, 0);
    if (handle > 0) {
      socket_arr[i].handle       = handle;
      socket_arr[i].non_blocking = 0;
      socket_arr[i].local_port   = 0;
      socket_arr[i].remote_port  = 0;
      socket_arr[i].recv_timeout = 0xFFFFFFFF;
      socket_arr[i].send_timeout = 0;
      socket_arr[i].type         = type;
      memset((void *)socket_arr[i].remote_ip, 0, 16);
      ret = i;
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief       Assign a local address to a socket.
  \param [in]  socket   Socket identification number
  \param [in]  ip       Pointer to local IP address
  \param [in]  ip_len   Length of 'ip' address in bytes
  \param [in]  port     Local port number
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (address or socket already bound)
                 - ARM_SOCKET_EADDRINUSE        : Address already in use
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  int32_t        ret, addr_len;
  union {
    SOCKADDR_T   addr;
    SOCKADDR_6_T addr6;
  } addr;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || ((ip_len != 4U) && (ip_len != 16U)))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (ip_len == 4U) {
      addr.addr.sin_family   = ATH_AF_INET;
      addr.addr.sin_port     = port;
      addr.addr.sin_addr     = __UNALIGNED_UINT32_READ(ip);
      addr_len               = sizeof(SOCKADDR_T);
    } else {
      addr.addr6.sin6_family = ATH_AF_INET6;
      addr.addr6.sin6_port   = port;
      memcpy((void *)&addr.addr6.sin6_addr, (void *)ip, ip_len);
      addr_len               = sizeof(SOCKADDR_6_T);
    }
    ret = qcom_bind(socket_arr[socket].handle, (struct sockaddr *)&addr.addr6, addr_len);
    if (ret == 0) {
      socket_arr[socket].local_port = port;
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketListen (int32_t socket, int32_t backlog)
  \brief       Listen for socket connections.
  \param [in]  socket   Socket identification number
  \param [in]  backlog  Number of connection requests that can be queued
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (socket not bound)
                 - ARM_SOCKET_ENOTSUP           : Operation not supported
                 - ARM_SOCKET_EISCONN           : Socket is already connected
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketListen (int32_t socket, int32_t backlog) {
  int32_t ret;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }

  if (ret == 0) {
    ret = qcom_listen(socket_arr[socket].handle, backlog);
    if (ret < 0) {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief       Accept a new connection on a socket.
  \param [in]  socket   Socket identification number
  \param [out] ip       Pointer to buffer where address of connecting socket shall be returned
                        (NULL for none)
  \param [in,
          out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                 - length of supplied 'ip' on input
                 - length of stored 'ip' on output
  \param [out] port     Pointer to buffer where port of connecting socket shall be returned
                        (NULL for none)
  \return      status information
                 - socket identification number of accepted socket (>=0)
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (socket not in listen mode)
                 - ARM_SOCKET_ENOTSUP           : Operation not supported (socket type does not support accepting connections)
                 - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  int32_t        ret, handle, addr_len;
  uint8_t        i;
  union {
    SOCKADDR_T   addr;
    SOCKADDR_6_T addr6;
  } addr;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) || ((ip == NULL) || (ip_len == NULL) || ((*ip_len != 4U) && (*ip_len != 16U)))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    // Find free local socket entry in socket_arr
    for (i = 0U; i < MAX_SOCKETS_SUPPORTED; i++) {
      if (socket_arr[i].handle == NULL) {
        break;
      }
    }
    if (i == MAX_SOCKETS_SUPPORTED) {
      // No free entry is available
      ret = ARM_SOCKET_ERROR;
    }
  }

  if (ret == 0) {
    handle = qcom_accept(socket_arr[socket].handle, (struct sockaddr *)&addr, &addr_len);
    if (handle > 0) {
      socket_arr[i].handle = handle;
      socket_arr[i].non_blocking = socket_arr[socket].non_blocking;
      socket_arr[i].recv_timeout = socket_arr[socket].recv_timeout;
      socket_arr[i].send_timeout = socket_arr[socket].send_timeout;
      ret = i;
      if (addr_len == sizeof(SOCKADDR_T)) {
        __UNALIGNED_UINT32_WRITE(ip, addr.addr.sin_addr);
        *ip_len = 4U;
        *port   = addr.addr.sin_port;
        memcpy((void *)socket_arr[socket].remote_ip, (void *)ip, 4);
        socket_arr[socket].remote_port = *port;
      } else {
        memcpy((void *)ip, (void *)&addr.addr6.sin6_addr, 16);
        *ip_len = 16U;
        *port   = addr.addr6.sin6_port;
        memcpy((void *)socket_arr[socket].remote_ip, (void *)ip, 16);
        socket_arr[socket].remote_port = *port;
      }
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief       Connect a socket to a remote host.
  \param [in]  socket   socket identification number
  \param [in]  ip       Pointer to remote IP address
  \param [in]  ip_len   Length of 'ip' address in bytes
  \param [in]  port     Remote port number
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument
                 - ARM_SOCKET_EALREADY          : Connection already in progress
                 - ARM_SOCKET_EINPROGRESS       : Operation in progress
                 - ARM_SOCKET_EISCONN           : Socket is connected
                 - ARM_SOCKET_ECONNREFUSED      : Connection rejected by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EADDRINUSE        : Address already in use
                 - ARM_SOCKET_ETIMEDOUT         : Operation timed out
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  int32_t        ret, addr_len;
  union {
    SOCKADDR_T   addr;
    SOCKADDR_6_T addr6;
  } addr;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || ((ip_len != 4U) && (ip_len != 16U)))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (ip_len == 4U) {
      addr.addr.sin_family   = ATH_AF_INET;
      addr.addr.sin_port     = port;
      addr.addr.sin_addr     = SWAP(__UNALIGNED_UINT32_READ(ip));
      addr_len               = sizeof(SOCKADDR_T);
    } else {
      addr.addr6.sin6_family = ATH_AF_INET6;
      addr.addr6.sin6_port   = port;
      memcpy((void *)&addr.addr6.sin6_addr, (void *)ip, ip_len);
      addr_len               = sizeof(SOCKADDR_6_T);
    }
    ret = qcom_connect(socket_arr[socket].handle, (struct sockaddr *)&addr, addr_len);
    if (ret == 0) {
      if (ip_len == 4) {
        memcpy((void *)socket_arr[socket].remote_ip, (void *)ip, 4);
        socket_arr[socket].remote_port = port;
      } else {
        memcpy((void *)socket_arr[socket].remote_ip, (void *)ip, 16);
        socket_arr[socket].remote_port = port;
      }
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len)
  \brief       Receive data from a connected socket.
  \param [in]  socket   Socket identification number
  \param [out] buf      Pointer to buffer where data should be stored
  \param [in]  len      Length of buffer (in bytes)
  \return      status information
                 - number of bytes received (>0)
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ENOTCONN          : Socket is not connected
                 - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len) {
  return (WiFi_SocketRecvFrom (socket, buf, len, NULL, NULL, NULL));
}

/**
  \fn          int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief       Receive data from a socket.
  \param [in]  socket   Socket identification number
  \param [out] buf      Pointer to buffer where data should be stored
  \param [in]  len      Length of buffer (in bytes)
  \param [out] ip       Pointer to buffer where remote source address shall be returned
                        (NULL for none)
  \param [in,
          out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                 - length of supplied 'ip' on input
                 - length of stored 'ip' on output
  \param [out] port     Pointer to buffer where remote source port shall be returned
                        (NULL for none
  \return      status information
                 - number of bytes received (>0)
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ENOTCONN          : Socket is not connected
                 - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  int32_t        ret, from_addr_len, len_to_recv, len_recv;
  uint32_t       timeout;
  char          *rx_buf;
  union {
    SOCKADDR_T   addr;
    SOCKADDR_6_T addr6;
  } from_addr;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    timeout = socket_arr[socket].recv_timeout;
    if (socket_arr[socket].non_blocking != 0U) {
      // Non-blocking is emulated with timout of 1 ms
      timeout = 1U;
    }

    len_to_recv = (int32_t)len;
    len_recv    = 0;
    do {
      if ((A_STATUS)t_select(Custom_Api_GetDriverCxt(0), socket_arr[socket].handle, timeout) == A_OK) {
        rx_buf = NULL;
        if (ip != NULL) {
          ret  = qcom_recvfrom(socket_arr[socket].handle, (char **)buf, len_to_recv - len_recv, 0, (struct sockaddr *)&from_addr, &from_addr_len);
        } else {
          ret  = qcom_recv(socket_arr[socket].handle, &rx_buf, len_to_recv - len_recv, 0);
        }
        if (ret > 0) {
          memcpy((void *)((char *)buf + len_recv), (void *)rx_buf, ret);
          len_recv += ret;
          if (from_addr_len == sizeof(SOCKADDR_T)) {
            if (ip != NULL) {
              __UNALIGNED_UINT32_WRITE(ip, from_addr.addr.sin_addr);
            }
            if (ip_len != NULL) {
              *ip_len = 4U;
            }
            if (port != NULL) {
              *port = from_addr.addr.sin_port;
            }
          }
          if (from_addr_len == sizeof(SOCKADDR_6_T)) {
            if (ip != NULL) {
              memcpy((void *)ip, (void *)&from_addr.addr6.sin6_addr, 16);
            }
            if (ip_len != NULL) {
              *ip_len = 16U;
            }
            if (port != NULL) {
              *port = from_addr.addr6.sin6_port;
            }
          }
        } else {
          // If error
          ret = ARM_SOCKET_ERROR;
        }
        if(rx_buf != NULL) {
          zero_copy_free(rx_buf);
        }
        if (ret != 0) {
          break;
        }
      } else {
        // If timeout
        break;
      }
    } while (len_to_recv > len_recv);
  }

  if (ret > 0) {
    ret = len_recv;
  } else if (ret == 0) {
    // If timed out in blocking mode or no data available in non-blocking mode
    ret = ARM_SOCKET_EAGAIN;
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len)
  \brief       Send data to a connected socket.
  \param [in]  socket   Socket identification number
  \param [in]  buf      Pointer to buffer containing data to send
  \param [in]  len      Length of data (in bytes)
  \return      status information
                 - number of bytes sent (>0)
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ENOTCONN          : Socket is not connected
                 - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len) {
  return (WiFi_SocketSendTo (socket, buf, len, NULL, NULL, NULL));
}

/**
  \fn          int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief       Send data to a socket.
  \param [in]  socket   Socket identification number
  \param [in]  buf      Pointer to buffer containing data to send
  \param [in]  len      Length of data (in bytes)
  \param [in]  ip       Pointer to remote destination IP address
  \param [in]  ip_len   Length of 'ip' address in bytes
  \param [in]  port     Remote destination port number
  \return      status information
                 - number of bytes sent (>0)
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ENOTCONN          : Socket is not connected
                 - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                 - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                 - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  int32_t        ret, to_addr_len, len_sent, len_to_send, len_curr;
  char          *tx_buf;
  union {
    SOCKADDR_T   addr;
    SOCKADDR_6_T addr6;
  } to_addr;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (len > WIFI_QCA400x_MAX_PACKET_LEN) {
      tx_buf = custom_alloc (WIFI_QCA400x_MAX_PACKET_LEN);
    } else {
      tx_buf = custom_alloc (len);
    }
    if (ip != NULL) {
      if (ip_len == 4U) {
        to_addr.addr.sin_family   = ATH_AF_INET;
        to_addr.addr.sin_port     = port;
        to_addr.addr.sin_addr     = __UNALIGNED_UINT32_READ(ip);
        to_addr_len               = sizeof(SOCKADDR_T);
      } else {
        to_addr.addr6.sin6_family = ATH_AF_INET6;
        to_addr.addr6.sin6_port   = port;
        memcpy((void *)&to_addr.addr6.sin6_addr, (void *)ip, ip_len);
        to_addr_len               = sizeof(SOCKADDR_6_T);
      }
    }

    len_to_send = (int32_t)len;
    len_sent    = 0U;
    if (tx_buf != NULL) {
      do {
        len_curr = len_to_send - len_sent;
        if (len_curr > WIFI_QCA400x_MAX_PACKET_LEN) {
          len_curr = WIFI_QCA400x_MAX_PACKET_LEN;
        }
        memcpy((void *)tx_buf, (const void *)((const char *)buf + len_sent), len_curr);
        if (ip != NULL) {
          ret = qcom_sendto(socket_arr[socket].handle, (char *)buf, (int32_t)len, 0, (struct sockaddr *)&to_addr, to_addr_len);
        } else {
          ret = qcom_send(socket_arr[socket].handle, tx_buf, (int32_t)len_to_send, 0);
        }
        if (ret > 0) {
          len_sent += ret;
        } else if (ret < 0) {
          // If error
          ret = ARM_SOCKET_ERROR;
          break;
        }
      } while (len_to_send > len_sent);
      if (tx_buf != NULL) {
        custom_free(tx_buf);
      }
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  if (ret > 0) {
    ret = len_sent;
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief       Retrieve local IP address and port of a socket.
  \param [in]  socket   Socket identification number
  \param [out] ip       Pointer to buffer where local address shall be returned
                        (NULL for none)
  \param [in,
          out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                 - length of supplied 'ip' on input
                 - length of stored 'ip' on output
  \param [out] port     Pointer to buffer where local port shall be returned
                        (NULL for none)
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  int32_t  ret, dummy_val;
  uint32_t u32;
  uint8_t  dummy[16];

  ret = 0;

  // Check parameters and state
  if (socket < 0) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len == NULL) || ((*ip_len != 4U) && (*ip_len != 16U))|| (port == NULL))) {
    ret = ARM_SOCKET_EINVAL;
  }

  // Get local IP
  if (ret == 0) {
    if (*ip_len == 4U) {
      if (qcom_ipconfig(0U, IPCFG_QUERY, &u32, (uint32_t *)&dummy_val, (uint32_t *)&dummy_val) == A_OK) {
        __UNALIGNED_UINT32_WRITE(ip, SWAP(u32));
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    } else {
      if (qcom_ip6_address_get(0, (uint8_t *)ip, (uint8_t *)dummy, (uint8_t *)dummy, (uint8_t *)dummy, &dummy_val, &dummy_val, &dummy_val, &dummy_val) != A_OK) {
        ret = ARM_SOCKET_ERROR;
      }
    }
  }

  // Get local port
  if (ret == 0) {
    *port = socket_arr[socket].local_port;
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief       Retrieve remote IP address and port of a socket
  \param [in]  socket   Socket identification number
  \param [out] ip       Pointer to buffer where remote address shall be returned
                        (NULL for none)
  \param [in,
          out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                 - length of supplied 'ip' on input
                 - length of stored 'ip' on output
  \param [out] port     Pointer to buffer where remote port shall be returned
                        (NULL for none)
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                 - ARM_SOCKET_ENOTCONN          : Socket is not connected
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  int32_t ret;

  ret = 0;

  // Check parameters and state
  if (socket < 0) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len == NULL) || ((*ip_len != 4U) && (*ip_len != 16U))|| (port == NULL))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    memcpy((void *)ip, (void *)socket_arr[socket].remote_ip, *ip_len);
    *port = socket_arr[socket].remote_port;
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len)
  \brief       Get socket option.
  \param [in]  socket   Socket identification number
  \param [in]  opt_id   Option identifier
  \param [out] opt_val  Pointer to the buffer that will receive the option value
  \param [in,
          out] opt_len  Pointer to length of the option value
                 - length of buffer on input
                 - length of data on output
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument
                 - ARM_SOCKET_ENOTSUP           : Operation not supported
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len) {
  int32_t ret;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((opt_len == NULL) || (*opt_len < 4U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    switch (opt_id) {
      case ARM_SOCKET_SO_RCVTIMEO:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].recv_timeout);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_SNDTIMEO:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].send_timeout);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_TYPE:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].type);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_KEEPALIVE:
      default:
        ret = ARM_SOCKET_ENOTSUP;
        break;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len)
  \brief       Set socket option.
  \param [in]  socket   Socket identification number
  \param [in]  opt_id   Option identifier
  \param [in]  opt_val  Pointer to the option value
  \param [in]  opt_len  Length of the option value in bytes
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EINVAL            : Invalid argument
                 - ARM_SOCKET_ENOTSUP           : Operation not supported
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len) {
  int32_t  ret;
  uint32_t val;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((opt_len == NULL) || (opt_len != 4U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    val = __UNALIGNED_UINT32_READ(opt_val);
    switch (opt_id) {
      case ARM_SOCKET_IO_FIONBIO:
        socket_arr[socket].non_blocking = (uint8_t)val;
        break;
      case ARM_SOCKET_SO_RCVTIMEO:
        socket_arr[socket].recv_timeout = val;
        break;
      case ARM_SOCKET_SO_SNDTIMEO:
        socket_arr[socket].send_timeout = val;
        break;
      case ARM_SOCKET_SO_KEEPALIVE:
      default:
        ret = ARM_SOCKET_ENOTSUP;
        break;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketClose (int32_t socket)
  \brief       Close and release a socket.
  \param [in]  socket   Socket identification number
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ESOCK             : Invalid socket
                 - ARM_SOCKET_EAGAIN            : Operation would block (may be called again)
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketClose (int32_t socket) {
  int32_t ret;

  ret = 0;

  // Check parameters
  if ((socket < 0) || (socket_arr[socket].handle == 0)) {
    ret = ARM_SOCKET_ESOCK;
  }

  if (ret == 0) {
    ret = qcom_socket_close(socket_arr[socket].handle);
    if (ret == 0) {
      socket_arr[socket].handle = 0;
    } else if (ret < 0) {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len)
  \brief       Retrieve host IP address from host name.
  \param [in]  name     Host name
  \param [in]  af       Address family
  \param [out] ip       Pointer to buffer where resolved IP address shall be returned
  \param [in,
          out] ip_len   Pointer to length of 'ip'
                 - length of supplied 'ip' on input
                 - length of stored 'ip' on output
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_EINVAL            : Invalid argument
                 - ARM_SOCKET_ENOTSUP           : Operation not supported
                 - ARM_SOCKET_ETIMEDOUT         : Operation timed out
                 - ARM_SOCKET_EHOSTNOTFOUND     : Host not found
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len) {
  int32_t    ret;
  SOCKADDR_T dns_ip;
  SOCKADDR_T ip_addr;
  uint32_t   dns_servers[2];
  uint32_t   dns_servers_num;

  ret = 0;

  // Check parameters
  if ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U)) {
    ret = ARM_SOCKET_EINVAL;
  }
  if ((ret == 0) && (!driver_initialized)) {
    ret = ARM_SOCKET_ERROR;
  }
  if ((ret == 0) && (af != ARM_SOCKET_AF_INET)) {
    // IPv6 resolver currently not supported as qcom_dnsc_get_host_by_name and 
    // qcom_dnsc_get_host_by_name2 do not support long host addresses and 
    // qcom_dns_resolver only supports IPv4 addresses
    ret = ARM_SOCKET_ENOTSUP;
  }

  if (ret == 0) {
    memset((void *)dns_servers, 0, sizeof(dns_servers));
    dns_servers_num = 0U;
    qcom_dns_server_address_get(dns_servers, &dns_servers_num);

    dns_ip.sin_addr = SWAP(dns_servers[0]);
    if (qcom_dns_resolver(dns_ip, (char *)name, &ip_addr, WIFI_QCA400x_RESOLVE_TIMEOUT) == A_OK) {
      __UNALIGNED_UINT32_WRITE(ip, ip_addr.sin_addr);
      *ip_len = 4U;
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn          int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len)
  \brief       Probe remote host with Ping command.
  \param [in]  ip       Pointer to remote host IP address
  \param [in]  ip_len   Length of 'ip' address in bytes
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ip pointer or ip_len different than 4 or 16)
*/
static int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len) {
  int32_t  ret;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (ip == NULL){
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (ip_len) {
      case 4:
        if (qcom_ping(__UNALIGNED_UINT32_READ(ip), 1U) != A_OK) {
          ret = ARM_DRIVER_ERROR;
        }
      case 16:
      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  return ret;
}
#endif

#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)

/**
  \fn          int32_t WiFi_BypassControl (uint32_t enable)
  \brief       Enable or disable bypass (pass-through) mode. Transmit and receive Ethernet frames (IP layer bypassed and WiFi/Ethernet translation).
  \param [in]  enable
                 - value != 0: enable
                 - value = 0: disable
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
*/
static int32_t WiFi_BypassControl (uint32_t enable) {
  if (enable) {
    return ARM_DRIVER_OK;
  } else {
    return ARM_DRIVER_ERROR;
  }
}

/**
  \fn          int32_t WiFi_EthSendFrame (const uint8_t *frame, uint32_t len)
  \brief       Send Ethernet frame (in bypass mode only).
  \param [in]  frame    Pointer to frame buffer with data to send
  \param [in]  len      Frame buffer length in bytes
  \return      execution status
                 - ARM_DRIVER_OK                : Operation successful
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_BUSY        : Driver is busy
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL frame pointer)
*/
static int32_t WiFi_EthSendFrame (const uint8_t *frame, uint32_t len) {
  int32_t  ret;
  uint32_t idx;

  ret = 0;

  // Check parameters
  if ((frame == NULL) || (len == 0) || (len > 1576)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == 0) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == 0) {
    ret = ARM_DRIVER_ERROR_BUSY;
    idx = tx_idx;
    if (tx_frame[idx].available == 0) {
      ret = ARM_DRIVER_ERROR_BUSY;
    } else {
      tx_frame[idx].available = 0;
      tx_frame[idx].pcb.FRAG[0].LENGTH = len;
      tx_frame[idx].pcb.FRAG[0].FRAGMENT = tx_buf[idx];
      memcpy(tx_buf[idx], frame, len);

      tx_idx++;
      if (tx_idx == NUM_TX_FRAME) {
        tx_idx = 0;
      }

      if (ATHEROS_WIFI_IF.SEND (&wifiDev, &tx_frame[idx].pcb, 0, 1, 0) == A_OK) {
        ret = ARM_DRIVER_OK;
      } else {
        tx_idx = idx;
        tx_frame[idx].available = 1;
        ret = ARM_DRIVER_ERROR;
      }
    }
  }
  return ret;
}

/**
  \fn          int32_t WiFi_EthReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame (in bypass mode only).
  \param [in]  frame    Pointer to frame buffer for data to read into
  \param [in]  len      Frame buffer length in bytes
  \return      number of data bytes read or error code
                 - value >= 0                   : Number of data bytes read
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                 - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL frame pointer)
*/
static int32_t WiFi_EthReadFrame (uint8_t *frame, uint32_t len) {
  int32_t ret;
  uint32_t sz;

  if ((frame == NULL) && (len != 0)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  } else {
    ret = 0;
    if (rx_q_tail != rx_q_head) {
      // Queue is not empty
      if (frame != NULL) {
        sz = rx_netbuf_queue[rx_q_tail & (NUM_RX_FRAME - 1)]->native.FRAG[0].LENGTH;
        if (sz > len) {
          sz = len;
        }
        memcpy (frame, rx_netbuf_queue[rx_q_tail & (NUM_RX_FRAME - 1)]->native.FRAG[0].FRAGMENT, sz);
        ret = (int32_t)sz;
      }
      A_NETBUF_FREE(rx_netbuf_queue[rx_q_tail & (NUM_RX_FRAME - 1)]);
      rx_q_tail++;
    }
  }

  return ret;
}

/**
  \fn          uint32_t WiFi_EthGetRxFrameSize (void)
  \brief       Get size of received Ethernet frame (in bypass mode only).
  \return      number of bytes in received frame
*/
static uint32_t WiFi_EthGetRxFrameSize (void) {
  uint32_t ret;

  ret = 0U;
  if (rx_q_tail != rx_q_head) {
    // Queue is not empty
    ret = rx_netbuf_queue[rx_q_tail & (NUM_RX_FRAME - 1)]->native.FRAG[0].LENGTH;
  }

  return ret;
}

#endif

// Structure exported by driver Driver_WiFin
extern
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_QCA400x_DRIVER_INDEX);
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_QCA400x_DRIVER_INDEX) = { 
  WiFi_GetVersion,
  WiFi_GetCapabilities,
  WiFi_Initialize,
  WiFi_Uninitialize,
  WiFi_PowerControl,
  WiFi_SetOption,
  WiFi_GetOption,
  WiFi_Scan,
  WiFi_Connect,
  WiFi_ConnectWPS,
  WiFi_Disconnect,
  WiFi_IsConnected,
  WiFi_AP_Start,
  WiFi_AP_Stop,
  WiFi_AP_IsRunning,
#if (WIFI_QCA400x_MODE_PASSTHROUGH)     // Pass-through mode  supported (internal network stack is bypassed)
  WiFi_BypassControl,
  WiFi_EthSendFrame,
  WiFi_EthReadFrame,
  WiFi_EthGetRxFrameSize,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
#endif
#if (WIFI_QCA400x_MODE_INT_STACK)       // Enabled internal network stack (socket functions enabled)
  NULL,
  NULL,
  NULL,
  NULL,
  WiFi_SocketCreate,
  WiFi_SocketBind,
  WiFi_SocketListen,
  WiFi_SocketAccept,
  WiFi_SocketConnect,
  WiFi_SocketRecv,
  WiFi_SocketRecvFrom,
  WiFi_SocketSend,
  WiFi_SocketSendTo,
  WiFi_SocketGetSockName,
  WiFi_SocketGetPeerName,
  WiFi_SocketGetOpt,
  WiFi_SocketSetOpt,
  WiFi_SocketClose,
  WiFi_SocketGetHostByName,
  WiFi_Ping
#endif
};
