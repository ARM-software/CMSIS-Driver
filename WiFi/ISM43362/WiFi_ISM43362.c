/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2022 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        4. April 2022
 * $Revision:    V1.13
 *
 * Driver:       Driver_WiFin (n = WIFI_ISM43362_DRV_NUM value)
 * Project:      WiFi Driver for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * --------------------------------------------------------------------------
 * Use the WiFi_ISM43362_Config.h file for compile time configuration 
 * of this driver.
 *
 * IMPORTANT NOTES:
 * This driver uses SPI for communicating with ISM module, however there are 
 * 3 pins that are not handled by SPI peripheral, and they are:
 *  - RSTN    = reset        (active low)  (output)
 *  - SSN     = slave select (active low)  (output)
 *  - DATARDY = data ready   (active high) (input)
 *
 * SPI transfer buffers (spi_send_buf and spi_recv_buf) can be placed in the
 * appropriate ram by using section ".bss.driver.spin" (n = WIFI_ISM43362_SPI_DRV_NUM)
 * in the linker scatter file. Example: ".bss.driver.spi1".
 *
 * To initialize/uninitialize and drive SSN and RSTN pins, and get state of 
 * DATARDY pin you need to implement following functions 
 * (functions template is available in WiFi_ISM43362_HW.c file and should be 
 * adapted according to hardware):
 *   - void    WiFi_ISM43362_Pin_Initialize   (void)
 *   - void    WiFi_ISM43362_Pin_Uninitialize (void)
 *   - void    WiFi_ISM43362_Pin_RSTN         (uint8_t rstn)
 *   - void    WiFi_ISM43362_Pin_SSN          (uint8_t ssn)
 *   - uint8_t WiFi_ISM43362_Pin_DATARDY      (void)
 *
 * For better performance of the driver DATARDY pin state change should be 
 * interrupt driven and following function should be called when line state 
 * changes from inactive to active (otherwise the driver will use polling 
 * with 1 ms interval to check DATARDY line state change):
 *   - void    WiFi_ISM43362_Pin_DATARDY_IRQ  (void)
 *                                                   
 * Generic ISM43362 Module limitations:
 *  - power off is not supported because it disables host interface
 *  - configuration of local port for client socket is not supported
 *  - socket connect to non-existent port returns error code 
 *    ARM_SOCKET_ETIMEDOUT instead of ARM_SOCKET_ECONNREFUSED
 *  - sometimes connection to remote host fails
 *
 * ISM43362 Module on STMicroelectronics B-L475E-IOT01A1 limitations:
 *  - firmware ISM43362_M3G_L44_SPI_C3.5.2.5.STM:
 *    - SocketConnect does not work if any of IP address octets is 255
 *      (for example IPs like x.y.z.255 or x.y.255.z do not work) or 
 *      if first or last octet is 0 
 *      (for example IPs 0.x.y.z or x.y.z.0 do not work)
 *    - module sometimes returns previous resolve result on request to 
 *      resolve non-existing host address
 *    - CMSIS Driver Validation test for SocketAccept fails if SocketBind and 
 *      SocketListen tests are executed before it, because module stays in some 
 *      odd state and never signals accepted client connection
 *
 * ISM43362 Module on ISMART43362 shield limitations:
 *  - hardware es-WiFi SHIELD Rev. C
 *    - module has reset line connected directly to 
 *      header J6 pin 3 (MICRO_RST_N) together with reset line from the 
 *      microcontroller motherboard (if reset line is activated either by SW2
 *      on the WiFi Shield or by reset button on microcontroller motherboard, 
 *      both the microcontroller and WiFi Shield are reset)
 *      For testing the driver in such combination delay between 
 *      WiFi Initialization and debugger connect has to be introduced and 
 *      WiFi Shield has to be reset manually before starting debug session.
 *  - firmware ISM43362_M3G_L44_SPI_C6.2.1.7.bin is supported
 *    - SocketConnect does not work if certain IP address octets contain 
 *      value 0 or 255
 *      (combinations that do not work: 0.x.y.z, x.y.z.0, 255.x.y.z)
 *  - firmware ISM43362_M3G_L44_SPI_C6.2.1.8.bin is not supported:
 *    - added additional "\r\n" to "OK" response (now 12 bytes instead of 10)
 *      ("\r\n\r\n\r\nOK\r\n> " instead of previously 
 *       "\r\n\r\nOK\r\n> ")
 *    - added additional "\r\n" in front of "OK" in response containing
 *      received data but just for UDP sockets
 *      ("\r\n"DATA"\r\n\r\nOK\r\n> " instead of previously 
 *       "\r\n"DATA"\r\nOK\r\n> ")
 *    - does not return single byte received when requested by R0 command
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.13
 *    - Added configuration for asynchronous thread stack size
 *  Version 1.12
 *    - Enabled placement of SPI transfer buffers in appropriate RAM by using section
 *      ".bss.driver.spin" (n = WIFI_ISM43362_SPI_DRV_NUM) in the linker scatter file
 *  Version 1.11
 *    - Added support for 5 GHz channels on Access Point
 *  Version 1.10
 *    - Fixed socket connect operation for non-blocking mode
 *  Version 1.9
 *    - Corrected Initialize function failure if called shortly after reset
 *    - Corrected default protocol selection in SocketCreate function
 *  Version 1.8
 *    - Corrected SocketConnect function never returning 0 in non-blocking mode
 *    - Corrected SocketRecv/SocketRecvFrom function polling if called without previous Bind
 *    - Corrected delay after module reset
 *  Version 1.7
 *    - Added check that non-STM firmware version is 6.2.1.7, other are not supported
 *  Version 1.6
 *    - Corrected functionality when DATARDY line is used in polling mode
 *  Version 1.5
 *    - API V1.1: SocketSend/SendTo and SocketRecv/RecvFrom (support for polling)
 *  Version 1.4
 *    - Corrected GetModuleInfo return string termination
 *  Version 1.3
 *    - Corrected not setting password in Activate if OPEN security is used
 *  Version 1.2
 *    - Corrected SocketClose functionality
 *    - Updated Initialization function to handle unavailable reset pin
 *  Version 1.1
 *    - Updated functionality to comply with CMSIS WiFi Driver Validation
 *    - Added debug of SPI traffic to Event Recorder
 *  Version 1.0
 *    - Initial version
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "RTE_Components.h"

#include "cmsis_os2.h"
#include "cmsis_compiler.h"

#include "Driver_WiFi.h"

#include "Driver_SPI.h"

#ifdef   RTE_Compiler_EventRecorder
#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder
#endif

#include "WiFi_ISM43362_Config.h"       // Driver configuration settings
#include "WiFi_ISM43362_HW.h"           // Driver hardware specific function prototypes
#include "WiFi_ISM43362_Buf.h"

#ifndef  WIFI_ISM43362_DEBUG_EVR
#define  WIFI_ISM43362_DEBUG_EVR    0
#endif
#if ((WIFI_ISM43362_DEBUG_EVR == 1) && !defined(RTE_Compiler_EventRecorder))
#error For driver debugging enable RTE: Compiler: Event Recorder!
#endif

// Legacy compatibility defines
#ifndef WIFI_ISM43362_SPI_BUS_SPEED
#define WIFI_ISM43362_SPI_BUS_SPEED    (20000000)
#endif
#ifndef WIFI_ISM43362_ASYNC_THREAD_STACK_SIZE
#define WIFI_ISM43362_ASYNC_THREAD_STACK_SIZE  (1024)
#endif


// Hardware dependent functions --------

// Exported hardware dependent function called by user code

extern void    WiFi_ISM43362_Pin_DATARDY_IRQ  (void);

/**
  \fn            void WiFi_ISM43362_Pin_DATARDY_IRQ (void)
  \brief         Interrupt on DATARDY line state changed to active state.
  \detail        This callback function should be called by external user code 
                 on interrupt when DATARDY line changes to active state.
                 Its usage improves driver efficiency as it then uses 
                 event for signaling DATARDY status change instead of polling.
  \return        none
*/
void WiFi_ISM43362_Pin_DATARDY_IRQ (void);


// WiFi Driver *****************************************************************

#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,13)       // Driver version

// Driver Version
static const ARM_DRIVER_VERSION driver_version = { ARM_WIFI_API_VERSION, ARM_WIFI_DRV_VERSION };

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = { 
  1U,                                   // Station supported
  1U,                                   // Access Point supported
  0U,                                   // Concurrent Station and Access Point not supported
  1U,                                   // WiFi Protected Setup (WPS) for Station supported
  0U,                                   // WiFi Protected Setup (WPS) for Access Point not supported
  1U,                                   // Access Point: event generated on Station connect
  0U,                                   // Access Point: event not generated on Station disconnect
  0U,                                   // Event not generated on Ethernet frame reception in bypass mode
  0U,                                   // Bypass or pass-through mode (Ethernet interface) not supported
  1U,                                   // IP (UDP/TCP) (Socket interface) supported
  0U,                                   // IPv6 (Socket interface) not supported
  1U,                                   // Ping (ICMP) supported
  0U                                    // Reserved (must be zero)
};

typedef struct {                        // Socket structure
                                        // Generic socket variables
  uint8_t  state;                       // State
  uint8_t  protocol;                    // Protocol: TCP/UDP
  uint8_t  non_blocking;                // 0 = blocking, non-zero = non-blocking
  uint8_t  reserved0;                   // Reserved
  uint32_t send_timeout;                // Send Timeout
  uint32_t recv_timeout;                // Receive Timeout
  uint32_t keepalive;                   // Keep-alive
  uint16_t local_port;                  // Local       port number
  uint16_t remote_port;                 // Remote host port number
  uint8_t  bound_ip[4];                 // Bound       IP
  uint8_t  remote_ip[4];                // Remote host IP
                                        // Module specific socket variables
  uint32_t recv_time_left;              // Receive Time left until Timeout
  uint32_t start_tick_count;            // Receive start Kernel Tick Count
  uint8_t  client;                      // Socket client running
  uint8_t  server;                      // Socket server running
  uint8_t  bound;                       // Socket bound (server)
  uint8_t  poll_recv;                   // Poll for reception
} socket_t;

// Operating mode
#define OPER_MODE_STATION               (1U     )
#define OPER_MODE_AP                    (1U << 1)

// Socket states
#define SOCKET_STATE_FREE               (0U)
#define SOCKET_STATE_CREATED            (1U)
#define SOCKET_STATE_LISTENING          (2U)
#define SOCKET_STATE_ACCEPTING          (3U)
#define SOCKET_STATE_ACCEPTED           (4U)
#define SOCKET_STATE_CONNECTING         (5U)
#define SOCKET_STATE_CONNECTED          (6U)
#define SOCKET_STATE_DISCONNECTED       (7U)

// Event flags
#define EVENT_SPI_XFER_DONE             (1U     )
#define EVENT_SPI_READY                 (1U << 1)
#define EVENT_ASYNC_POLL                (1U << 2)
#define EVENT_SOCK_RECV                 (1U     )       // Something was received
#define EVENT_SOCK_RECV_TIMEOUT         (1U << 1)       // Reception has timed-out
#define EVENT_SOCK_RECV_DISCON_REMOTE   (1U << 2)       // Remote host closed socket
#define EVENT_SOCK_RECV_CLOSE           (1U << 3)       // Socket close requested locally
#define EVENT_SOCK_RECV_CLOSE_DONE      (1U << 4)       // Socket closed locally
#define EVENT_SOCK_ACCEPTED             (1U << 5)       // Socket accepted new connection
#define EVENT_SOCK_ACCEPTING_CLOSE      (1U << 6)       // Socket accepting close requested locally
#define EVENT_SOCK_ACCEPTING_CLOSE_DONE (1U << 7)       // Socket accepting closed locally

// Local macros
#define SPI_Driver_(n)                  Driver_SPI##n
#define SPI_Driver(n)                   SPI_Driver_(n)
extern ARM_DRIVER_SPI                   SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM);
#define ptrSPI                        (&SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM))

#ifndef SPI_DRIVER_BSS
#define SPI_DRIVER_BSS_STRING(str)      #str
#define SPI_DRIVER_BSS_CREATE(id, n)    SPI_DRIVER_BSS_STRING(id##n)
#define SPI_DRIVER_BSS_SYMBOL(id, n)    SPI_DRIVER_BSS_CREATE(id, n)
#define SPI_DRIVER_BSS                  SPI_DRIVER_BSS_SYMBOL(      \
                                          .bss.driver.spi,          \
                                          WIFI_ISM43362_SPI_DRV_NUM \
                                        )
#endif

#define MAX_DATA_SIZE                  (1460U)

#define TRANSPORT_START                (1U)
#define TRANSPORT_STOP                 (0U)
#define TRANSPORT_RESTART              (2U)
#define TRANSPORT_SERVER               (1U)
#define TRANSPORT_CLIENT               (0U)

// Local variables and structures
static uint8_t                          module_initialized = 0U;
static uint8_t                          driver_initialized = 0U;
static uint8_t                          firmware_stm       = 0U;
static uint32_t                         firmware_version   = 0U;

static uint8_t                          async_thread_stack_mem[WIFI_ISM43362_ASYNC_THREAD_STACK_SIZE] __ALIGNED(8);

static osEventFlagsId_t                 event_flags_id;
static osEventFlagsId_t                 event_flags_sockets_id[WIFI_ISM43362_SOCKETS_NUM];
static osMutexId_t                      mutex_id_spi;
static osMutexId_t                      mutex_id_sockets;
static osThreadId_t                     thread_id_async_poll;

static ARM_WIFI_SignalEvent_t           signal_event_fn;

static uint8_t                          spi_datardy_irq;

static uint8_t                          oper_mode;
static uint32_t                         kernel_tick_freq_in_ms;
static uint8_t                          kernel_tick_freq_shift_to_ms;

static uint8_t                          spi_send_buf[MAX_DATA_SIZE +  8] __ALIGNED(4) __attribute__((section(SPI_DRIVER_BSS)));
static uint8_t                          spi_recv_buf[MAX_DATA_SIZE + 12] __ALIGNED(4) __attribute__((section(SPI_DRIVER_BSS)));
static uint32_t                         spi_recv_len;
static int32_t                          resp_code;

static uint8_t                          recv_buf [WIFI_ISM43362_SOCKETS_NUM][MAX_DATA_SIZE] __ALIGNED(4);
static uint32_t                         recv_len [WIFI_ISM43362_SOCKETS_NUM];

static uint32_t                         sta_lp_time;
static uint8_t                          sta_dhcp_client;
static uint8_t                          ap_beacon_interval;
static uint8_t                          ap_num_connected;
static uint32_t                         ap_dhcp_lease_time;

static uint8_t                          sta_local_ip  [4];
static uint8_t                          ap_local_ip   [4];
static uint8_t                          ap_mac     [8][6];

static socket_t                         socket_arr[2 * WIFI_ISM43362_SOCKETS_NUM];

// Mutex responsible for protecting SPI media access
static const osMutexAttr_t mutex_spi_attr = {
  "Mutex_SPI",                          // Mutex name
  osMutexPrioInherit,                   // attr_bits
  NULL,                                 // Memory for control block
  0U                                    // Size for control block
};

// Mutex responsible for protecting socket local variables access
static const osMutexAttr_t mutex_socket_attr = {
  "Mutex_Socket",                       // Mutex name
  osMutexPrioInherit,                   // attr_bits
  NULL,                                 // Memory for control block
  0U                                    // Size for control block
};

// Thread for polling and processing asynchronous messages
static const osThreadAttr_t thread_async_poll_attr = {
  .name       = "WiFi_ISM43362_Async_Thread",   // name of the thread
  .attr_bits  = osThreadDetached,               // attribute bits
  .cb_mem     = NULL,                           // memory for control block (system allocated)
  .cb_size    = 0U,                             // size of provided memory for control block (system defined)
  .stack_mem  = &async_thread_stack_mem,        // memory for stack
  .stack_size = sizeof(async_thread_stack_mem), // size of stack
  .priority   = WIFI_ISM43362_ASYNC_PRIORITY,   // initial thread priority
  .tz_module  = 0U,                             // TrustZone module identifier
  .reserved   = 0U                              // reserved (must be 0)
};

#if    (WIFI_ISM43362_DEBUG_EVR == 1)
#define EVR_DEBUG_SPI_MAX_LEN          (128)    // Maximum number of bytes of SPI debug message

#define EVR_SPI_ERROR                  (0x3F00 + 0x00)
#define EVR_SPI_DETAIL                 (0x3F00 + 0x01)

static char                             dbg_spi[EVR_DEBUG_SPI_MAX_LEN + 64];
#endif

// Function prototypes
static int32_t SPI_SendReceive     (uint8_t *ptr_send, uint32_t send_len, uint8_t *ptr_recv, uint32_t *ptr_recv_len, int32_t *ptr_resp_code, uint32_t timeout);

static int32_t WiFi_Uninitialize   (void);
static int32_t WiFi_SocketRecvFrom (int32_t socket,       void *buf, uint32_t len,       uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t WiFi_SocketSendTo   (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t  ip_len, uint16_t  port);
static int32_t WiFi_SocketClose    (int32_t socket);


// Helper Functions

/**
  \fn            void ResetVariables (void)
  \brief         Function that resets to all local variables to default values.
*/
static void ResetVariables (void) {
  uint32_t i;

  firmware_stm            = 0U;
  firmware_version        = 0U;

  sta_dhcp_client         = 1U;         // DHCP client is enabled by default

  signal_event_fn         = NULL;

  spi_datardy_irq         = 0U;

  oper_mode               = 0U;

  kernel_tick_freq_in_ms  = osKernelGetTickFreq () / 1000U;
  i                       = kernel_tick_freq_in_ms;
  kernel_tick_freq_shift_to_ms = 0U;
  if (kernel_tick_freq_in_ms > 1U) {
    while(i >= 1U) {
      if (i == 1U) {
        break;
      }
      if((i & 1U) != 0U) {
        kernel_tick_freq_shift_to_ms = 0U;
        break;
      }
      i >>= 1;
      kernel_tick_freq_shift_to_ms++;
    }
  }

  memset((void *)spi_send_buf, 0, sizeof(spi_send_buf));
  memset((void *)spi_recv_buf, 0, sizeof(spi_recv_buf));
  memset((void *)recv_buf, 0, sizeof(recv_buf));
  memset((void *)recv_len, 0, sizeof(recv_len));

  spi_recv_len          = 0U;
  resp_code             = 0U;

  sta_lp_time           = 0U;
  ap_beacon_interval    = 0U;
  ap_num_connected      = 0U;
  ap_dhcp_lease_time    = 0U;

  memset((void *)sta_local_ip, 0, sizeof(sta_local_ip));
  memset((void *)ap_local_ip,  0, sizeof(ap_local_ip));
  memset((void *)ap_mac,       0, sizeof(ap_mac));

  memset((void *)socket_arr,   0, sizeof(socket_arr));
}

/**
  \fn            void WiFi_ISM43362_Pin_DATARDY_IRQ (void)
  \brief         IRQ callback on DATARDY line state change from inactive to active.
  \detail        This function should be called from user IRQ routine when DATARDY pin state changes from inactive to active.
  \return        none
*/
void WiFi_ISM43362_Pin_DATARDY_IRQ (void) {
  osEventFlagsSet(event_flags_id, EVENT_SPI_READY);
}

/**
  \fn            void SPI_SignalEvent (uint32_t event)
  \brief         SPI Signal Event callback, called by SPI driver when an SPI event occurs.
  \param[in]     event    Event signaled by SPI driver
  \return        none
*/
static void SPI_SignalEvent (uint32_t event) {
  if (event & ARM_SPI_EVENT_TRANSFER_COMPLETE) {
    osEventFlagsSet(event_flags_id, EVENT_SPI_XFER_DONE);
  }
}

/**
  \fn            void Wait_us (uint32_t us)
  \brief         Wait a specified number of microseconds (executing NOPs).
  \param[in]     us       Microseconds
  \return        none
*/
static void Wait_us (uint32_t us) {
  uint32_t start_cnt, us_cnt;

  us_cnt    = (osKernelGetSysTimerFreq() / 1000000U) * us;
  start_cnt =  osKernelGetSysTimerCount();

  while ((osKernelGetSysTimerCount() - start_cnt) < us_cnt) {
    __NOP();
  }
}

/**
  \fn            const uint8_t *SkipCommas (uint8_t const *ptr, uint8_t num)
  \brief         Skip requested number of commas in character buffer.
  \param[in]     ptr      Pointer to character buffer
  \param[in]     num      Number of commas to skip
  \return        pointer to first character after requested number of commas, NULL in case of failure
*/
static const uint8_t *SkipCommas (const uint8_t *ptr, uint8_t num) {
  const uint8_t *ptr_tmp;
        char     ch;

  ptr_tmp = (const uint8_t *)ptr;
  if (ptr_tmp != NULL) {
    while (num > 0U) {
      do {
        ch = (char)*ptr_tmp++;
      } while ((ch != ',') && (ch != 0));
      if (ch == 0) {
        return NULL;
      }
      num--;
    }
  }

  return ptr_tmp;
}

/**
  \fn            uint8_t SPI_WaitReady (uint32_t timeout)
  \brief         Wait for SPI ready (DATARDY pin active).
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        SPI ready state
                   - 0: SPI is not ready
                   - 1: SPI is ready
*/
static uint8_t SPI_WaitReady (uint32_t timeout) {
  uint8_t ret;

  if (spi_datardy_irq != 0U) {          // If events are generated by WiFi_ISM43362_Pin_DATARDY_IRQ function
    ret = (uint8_t)((osEventFlagsWait(event_flags_id, EVENT_SPI_READY, osFlagsWaitAll, timeout) & EVENT_SPI_READY) == EVENT_SPI_READY);
  } else {
    do {
      ret = WiFi_ISM43362_Pin_DATARDY();
      if (ret == 0U) {                  // If DATARDY is not ready
        osDelay(1U);
        if (timeout > 0U) {
          timeout--;
        }
      }
    } while ((ret == 0U) && (timeout != 0U));
  }

  return ret;
}

/**
  \fn            void SPI_WaitTransferDone (uint32_t timeout)
  \brief         Wait for SPI transfer to finish.
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        SPI transfer finished state
                   - true: SPI transfer finished successfully
                   - false: SPI transfer did not finish successfully
*/
static bool SPI_WaitTransferDone (uint32_t timeout) {
  uint32_t event_flags;

  event_flags = osEventFlagsWait(event_flags_id, EVENT_SPI_XFER_DONE, osFlagsWaitAll, timeout);

  if ((event_flags & EVENT_SPI_XFER_DONE) != 0U) {
    return true;
  }

  return false;
}

/**
  \fn            int32_t SPI_SendReceive (const uint8_t *ptr_send, uint32_t send_len, uint8_t *ptr_recv, uint32_t *ptr_recv_len, int32_t *ptr_resp_code, uint32_t timeout)
  \brief         Send command and data over SPI interface and receive SPI response on SPI interface. Reception is done in blocks (default 32 bytes).
  \param[in]     ptr_send       Pointer to single buffer containing command and data to be sent
                                Buffer must be 2-byte aligned and size must be multiple of 2 bytes
  \param[in]     send_len       Number of bytes to be sent over the SPI
  \param[in]     ptr_recv       Pointer to buffer where response received over the SPI will be returned
                                Buffer must be 2-byte aligned
  \param[in]     ptr_recv_len   Pointer to value of maximum bytes to be received, updated with actually number of received bytes
  \param[in]     ptr_resp_code  Pointer to where response code will be returned (number of sent bytes, 0 = OK, or error code)
  \param[in]     timeout        Timeout in milliseconds (0 = no timeout)
  \return        execution status
                   - ARM_DRIVER_OK                : Command and data sent successfully and response received
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_SendReceive (uint8_t *ptr_send, uint32_t send_len, uint8_t *ptr_recv, uint32_t *ptr_recv_len, int32_t *ptr_resp_code, uint32_t timeout) {
  int32_t  ret;
  uint32_t len_to_recv, len_recv, len, i, j;
  uint32_t dummy_buf[4];
#if (WIFI_ISM43362_DEBUG_EVR == 1)
  uint32_t dbg_tot_len, dbg_len, ticks, time_in_ms;

  ticks = osKernelGetSysTimerCount();
#endif

  if ((ptr_send == NULL) || (((uint32_t)ptr_send & 1U) == 1U)) {
    // If pointer to send is invalid or not 2-byte aligned
    return ARM_DRIVER_ERROR;
  }
  if ((ptr_recv == NULL) || (((uint32_t)ptr_recv & 1U) == 1U) || (ptr_recv_len == NULL)) {
    // If pointer to receive is invalid or not 2-byte aligned, or pointer to receive length is invalid
    return ARM_DRIVER_ERROR;
  }
  if (send_len == 0U) {
    // If number of bytes to send is 0 or not multiple of 2
    return ARM_DRIVER_ERROR;
  }
  if (*ptr_recv_len == 0U) {
    // If maximum bytes to receive is 0
    return ARM_DRIVER_ERROR;
  }
  if (timeout == 0U) {
    // If timeout is 0, nothing can be sent/received immediately
    return ARM_DRIVER_ERROR;
  }

  if ((send_len & 1U) == 1U) {
    // If number of bytes to send is odd, add trailing padding of '\n' byte
    ptr_send[send_len] = '\n';
    send_len++;
  }

  ret         = ARM_DRIVER_OK;
  len_to_recv = *ptr_recv_len;
  len_recv    = 0U;

  // Send command and data on SPI
  if (SPI_WaitReady(timeout) == 1U) {           // If SPI is ready
    Wait_us(4U);                                // Wait 4 us
    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us
    if (ptrSPI->Send(ptr_send, send_len / 2) == ARM_DRIVER_OK) {
      // If SPI send started successfully
      if (SPI_WaitTransferDone(timeout)) {      // If SPI transfer finished
        WiFi_ISM43362_Pin_SSN(false);           // Deactivate slave select line
        Wait_us(3U);                            // Wait 3 us
      } else {                                  // If SPI transfer timed-out
        ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
        ret = ARM_DRIVER_ERROR_TIMEOUT;
      }
    } else {                                    // If SPI send start failed
      ret = ARM_DRIVER_ERROR;
    }
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (spi_datardy_irq == 0U) {                  // If events are not generated on DATARDY activation
    // Wait for DATARDY to deactivate after command was sent
    for (timeout = WIFI_ISM43362_SPI_TIMEOUT * 16U; timeout != 0U; timeout --) {
      if (WiFi_ISM43362_Pin_DATARDY() == 0U) {
        break;
      }
      Wait_us(32U);                             // Wait 32 us
    }
  }

  // Receive response on SPI
  if (SPI_WaitReady(timeout) == 1U) {           // If SPI is ready
    Wait_us(4U);                                // Wait 4 us
    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us
    do {
      len = len_to_recv;
      if (len > WIFI_ISM43362_SPI_RECEIVE_SIZE) {
        len = WIFI_ISM43362_SPI_RECEIVE_SIZE;
      }
      if (ptrSPI->Receive(ptr_recv + len_recv, (len + 1) / 2) == ARM_DRIVER_OK) {
        // If SPI receive started successfully
        if (SPI_WaitTransferDone(timeout)) {    // If SPI transfer finished
          len_to_recv -= len;
          len_recv    += len;
        } else {                                // If SPI transfer timed-out
          ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      } else {                                  // If SPI receive start failed
        ret = ARM_DRIVER_ERROR;
      }
    } while ((ret == ARM_DRIVER_OK) && (len_to_recv > 0U) && (WiFi_ISM43362_Pin_DATARDY() != 0U));

    // Sometimes module does not deactivate DATARDY line after all expected data was read-out
    // so we keep reading dummy data (0x15) until DATARDY signals chip has finished
    while (WiFi_ISM43362_Pin_DATARDY() != 0U) {
      if (ptrSPI->Receive(dummy_buf, sizeof(dummy_buf) / 2) == ARM_DRIVER_OK) {
        if (SPI_WaitTransferDone(WIFI_ISM43362_CMD_TIMEOUT) == 0U) {
          ret = ARM_DRIVER_ERROR_TIMEOUT;       // If SPI transfer timed out
        }
      } else {                                  // If SPI receive start failed
        ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
        ret = ARM_DRIVER_ERROR;
      }
    }

    // If reception ended with DATARDY line inactive we need to remove trailing 0x15 bytes
    if ((ret == ARM_DRIVER_OK) && (WiFi_ISM43362_Pin_DATARDY() == 0U)) {
      for (i = len_recv; i != 0U; i--) {
        if (ptr_recv[i-1] != 0x15U) {           // If non 0x15 value from end was found
          break;
        }
      }
      len_recv = i;                             // Correct number of received bytes without trailing 0x15 bytes
    }

    // If first character in received buffer is 0x15 then this is a not expected situation 
    // in which module returns no data but only pre-padded 0x15 with terminating "\r\nOK\r\n> "
    if (ptr_recv[0] == 0x15U) {
      for (i = 1U; i < len_recv; i++) {
        if (ptr_recv[i] != 0x15U) {             // If non 0x15 value from beginning was found
          break;
        }
      }
      len_recv -= i;
      // Copy all data after last leading 0x15 byte to beginning of the buffer
      for (j = 0U; j < len_recv; j++, i++) {
        ptr_recv[j] = ptr_recv[i];
      }
    }
    WiFi_ISM43362_Pin_SSN(false);               // Deactivate slave select line
    Wait_us(3U);                                // Wait 3 us
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_OK) {
    *ptr_recv_len = len_recv;
  }

  // Parse response
  if (ptr_resp_code != NULL) {
    if (*ptr_recv_len > 4) {
      if (memcmp((const void *)&ptr_recv[len_recv-8U], (const void *)"\r\nOK\r\n> ", 8) == 0) {
        *ptr_resp_code = 0;                     // OK
      } else {
        for (i = 0U; i < (len_recv - 1U); i++) {
          if (ptr_recv[i] == '\r') {
            if (ptr_recv[i+1] == '\n') {
              break;
            }
          }
        }
        if (i != (len_recv - 1U)) {
          // "\r\n" was found before end extract error code
          if (sscanf((const char *)&ptr_recv[i], "%d", ptr_resp_code) != 1) {
            // Error but we did not extract error code we force value -2
            *ptr_resp_code = -2;                // Error
          }
        } else {
          *ptr_resp_code = -2;                  // Error
        }
      }
    }
  }

#if (WIFI_ISM43362_DEBUG_EVR == 1)
  if (ret == ARM_DRIVER_OK) {
    // Prepare debug data that was sent
    dbg_len = send_len;
    if (dbg_len >= (EVR_DEBUG_SPI_MAX_LEN / 2)) {
      dbg_len = (EVR_DEBUG_SPI_MAX_LEN / 2) - 1U;
    }
    dbg_tot_len = 0U;
    memcpy(&dbg_spi[dbg_tot_len], "Sent[",   5);       dbg_tot_len += 5U;
    memcpy(&dbg_spi[dbg_tot_len], ptr_send,  dbg_len); dbg_tot_len += dbg_len;
    memcpy(&dbg_spi[dbg_tot_len], "],Recv[", 7);       dbg_tot_len += 7U;

    dbg_len = *ptr_recv_len;
    if (dbg_len >= (EVR_DEBUG_SPI_MAX_LEN / 2)) {
      dbg_len = (EVR_DEBUG_SPI_MAX_LEN / 2) - 1U;
    }
    memcpy(&dbg_spi[dbg_tot_len], ptr_recv,  dbg_len); dbg_tot_len += dbg_len;
    if (ptr_resp_code != NULL) {
      memcpy(&dbg_spi[dbg_tot_len], "],Resp[", 7);     dbg_tot_len += 7U;
      dbg_tot_len += (uint32_t)snprintf((char *)&dbg_spi[dbg_tot_len], 11U, "%i]", *ptr_resp_code);
    } else {
      dbg_spi[dbg_tot_len] = ']';                      dbg_tot_len += 1U;
    }

    time_in_ms = (osKernelGetSysTimerCount() - ticks) / (osKernelGetSysTimerFreq() / 1000);
    dbg_tot_len += (uint32_t)snprintf((char *)&dbg_spi[dbg_tot_len], 28U, " [spi recv=%i,%ims]", len_recv, time_in_ms);

    dbg_tot_len++;
    dbg_spi[dbg_tot_len] = 0;   // Terminate string just in case

    EventRecordData(EVR_SPI_DETAIL, dbg_spi, dbg_tot_len);
  } else {
    EventRecord2(EVR_SPI_ERROR, (uint32_t)ret, 0U);
  }
#endif

  return ret;
}

/**
  \fn            int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server)
  \brief         Start or stop transport server or client.
  \param[in]     socket       Socket identification number
  \param[in]     protocol     Protocol (ARM_SOCKET_IPPROTO_TCP or ARM_SOCKET_IPPROTO_UDP)
  \param[in]     ip           Pointer to remote destination IP address
  \param[in]     local_port   Local port number
  \param[in]     remote_ip    Pointer to remote IP4 address
  \param[in]     remote_port  Remote port number
  \param[in]     start        Start/stop request (0 = stop, 1 = start, 2 = restart (only server))
  \param[in]     server       Server/client (0 = client, 1 = server)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server) {
  int32_t  ret, spi_ret;

  ret = 0;

  // Set communication socket number
  snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", socket); spi_recv_len = sizeof(spi_recv_buf);
  spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
  if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
    ret = ARM_SOCKET_ERROR;
  }

  if ((ret == 0) && (start == TRANSPORT_START)) {
    // Select transport protocol
    memcpy((void *)spi_send_buf, (void *)"P1=1\r\n", 6); spi_recv_len = sizeof(spi_recv_buf);
    if (protocol == ARM_SOCKET_IPPROTO_TCP) {
      spi_send_buf[3] = '0';
    }
    spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
      ret = ARM_SOCKET_ERROR;
    }

    if (server != 0U) {
      // Set transport local port number
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P2=%d\r", local_port); spi_recv_len = sizeof(spi_recv_buf);
      spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
        ret = ARM_SOCKET_ERROR;
      }
    } else {
      // Set transport remote host IP address
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P3=%d.%d.%d.%d\r", remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]); spi_recv_len = sizeof(spi_recv_buf);
      spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
        ret = ARM_SOCKET_ERROR;
      }
      // Set transport remote port number
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P4=%d\r", remote_port); spi_recv_len = sizeof(spi_recv_buf);
      spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
        ret = ARM_SOCKET_ERROR;
      }
    }
    // Set read transport packet size
    snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "R1=%d\r", MAX_DATA_SIZE); spi_recv_len = sizeof(spi_recv_buf);
    spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
      ret = ARM_SOCKET_ERROR;
    }
    // Set read transport timeout
    memcpy((void *)spi_send_buf, (void *)"R2=1\r\n", 6); spi_recv_len = sizeof(spi_recv_buf);
    spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
      ret = ARM_SOCKET_ERROR;
    }
  }

  // Server/Client Start/Stop
  if (ret == 0) {
    if (server != 0U) {
      if (protocol == ARM_SOCKET_IPPROTO_TCP) {
        switch (start) {
          case TRANSPORT_START:           // Start multi-accept server
            memcpy((void *)spi_send_buf, (void *)"P5=11\r",  6);
            break;
          case TRANSPORT_STOP:            // Stop server
            memcpy((void *)spi_send_buf, (void *)"P5=0\r\n", 6);
            break;
          case TRANSPORT_RESTART:         // Close and wait for next connection on multi-accept server
            memcpy((void *)spi_send_buf, (void *)"P5=10\r",  6);
            break;
          default:
            memcpy((void *)spi_send_buf, (void *)"P5=0\r\n", 6);
            break;
        }
      } else {
        memcpy((void *)spi_send_buf, (void *)"P5=0\r\n", 6);
        if (start == TRANSPORT_START) {
          spi_send_buf[3] = '1';
        }
      }
    } else {
      memcpy((void *)spi_send_buf, (void *)"P6=0\r\n", 6);
      if (start == TRANSPORT_START) {
        spi_send_buf[3] = '1';
      }
    }
    spi_recv_len = sizeof(spi_recv_buf);
    spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
      ret = ARM_SOCKET_ERROR;
    }
  }

  return ret;
}

/**
  \fn            void WiFi_AsyncMsgProcessThread (void)
  \brief         Thread that reads and processes asynchronous messages from WiFi module.
  \return        none
*/
__NO_RETURN static void WiFi_AsyncMsgProcessThread (void *arg) {
  const uint8_t *ptr_resp_buf;
        uint32_t event_socket[WIFI_ISM43362_SOCKETS_NUM];
        uint32_t ticks, time_in_ms;
        int32_t  spi_ret;
        int32_t  ret;
        uint8_t  u8_arr[6];
        uint8_t  poll_async, poll_recv, check_async, repeat, event_signal;
        uint8_t  poll_nb_con;
        uint8_t  async_prescaler;
        uint16_t u16_val;
        uint8_t  i, hw_socket;

  (void)arg;

  async_prescaler = 1U;

  for (;;) {
    if ((osEventFlagsWait(event_flags_id, EVENT_ASYNC_POLL, osFlagsWaitAny, osWaitForever) & EVENT_ASYNC_POLL) == EVENT_ASYNC_POLL) {
      do {
        if (driver_initialized == 0U) {
          break;
        }

        // Check if thread should poll for asynchronous messages, non-blocking connect or receive in long blocking
        poll_async  = 0U;
        poll_nb_con = 0U;
        poll_recv   = 0U;
        check_async = 0U;
        for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
          if (socket_arr[i].state == SOCKET_STATE_ACCEPTING) {
            poll_async = 1U;
            if (async_prescaler > 0U) {
              async_prescaler --;
              if (async_prescaler == 0U) {
                async_prescaler = 16U;
                check_async     = 1U;
              }
            }
          }
          if ((socket_arr[i].non_blocking != 0U) && (socket_arr[i].state == SOCKET_STATE_CONNECTING)) {
            poll_nb_con = 1U;
          }
          if (socket_arr[i].poll_recv != 0U){
            poll_recv = 1U;
          }
        }
        if (oper_mode == OPER_MODE_AP) {
          poll_async = 1U;
        }

        repeat = 0U;

        if ((poll_async != 0U) || (poll_nb_con != 0U) || (poll_recv != 0U)) {
          event_signal = 0U;
          memset(event_socket, 0, sizeof(event_socket));

          if (osMutexAcquire(mutex_id_sockets, WIFI_ISM43362_ASYNC_INTERVAL) == osOK) { // Lock socket variables
            if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {      // Lock access to SPI interface (acquire mutex)
              if (check_async != 0U) {
                // Send command to read asynchronous message
                memcpy((void *)spi_send_buf, (void *)"MR\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
                spi_ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
                if (spi_ret == ARM_DRIVER_OK) {
                  // If message is asynchronous Accept
                  ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "Accepted ");
                  if (ptr_resp_buf != NULL) {
                    // If message contains "Accepted " string, parse it and extract ip and port
                    ptr_resp_buf += 9U;
                    // Parse IP Address and port
                    if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu:%hu", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3], &u16_val) == 5) {
                      // IP and port read from response correctly
                      // Find which socket is listening on accepted port
                      for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
                        if (socket_arr[i].state == SOCKET_STATE_ACCEPTING) {
                          socket_arr[i].remote_ip[0] = u8_arr[0];
                          socket_arr[i].remote_ip[1] = u8_arr[1];
                          socket_arr[i].remote_ip[2] = u8_arr[2];
                          socket_arr[i].remote_ip[3] = u8_arr[3];

                          // Read remote port by P? command as it is not available in asynchronous response
                          // Select socket
                          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", i); spi_recv_len = sizeof(spi_recv_buf);
                          spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
                          if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
                            // Show Transport Settings
                            memcpy((void *)spi_send_buf, (void *)"P?\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
                            spi_ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
                            if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
                              // Skip protocol, client IP, local port and host IP (4 ',') from response
                              ptr_resp_buf = SkipCommas(spi_recv_buf, 4U);
                              if (ptr_resp_buf != NULL) {
                                sscanf((const char *)ptr_resp_buf, "%hu", &socket_arr[i].remote_port);
                              }
                            }
                          }
                          break;
                        }
                      }
                      if (i != WIFI_ISM43362_SOCKETS_NUM) {
                        // Socket 'i' has accepted connection
                        event_socket[i] |= EVENT_SOCK_ACCEPTED;
                      }
                    }
                  }

                  // If message is asynchronous Assign
                  ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "Assigned ");
                  if (ptr_resp_buf != NULL) {
                    if (ptr_resp_buf != NULL) {
                      // If message contains "Assigned " string, parse it and extract MAC
                      ptr_resp_buf += 9U;
                      // Parse MAC Address
                      if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3], &u8_arr[4], &u8_arr[5]) == 6) {
                        // Check if MAC already exists in mac_ip4 array, if it does ignore it, otherwise add it to array
                        for (i = 0; i < 8; i++) {
                          if (memcmp((const void *)&ap_mac[i][0], (const void *)u8_arr, 6) == 0) {
                            break;
                          }
                        }
                        if ((i <= 8) && (ap_num_connected < 8)) {
                          memcpy((void *)&ap_mac[ap_num_connected][0], (void *)u8_arr, 6);
                          ap_num_connected++;
                          event_signal = 1U;
                        }
                      }
                    }
                  }
                }
              }

              if (poll_nb_con != 0U) {
                // Loop through all sockets
                for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
                  hw_socket = i;
                  if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
                    hw_socket -= WIFI_ISM43362_SOCKETS_NUM;     // Actually used socket of module
                  }
                  if ((socket_arr[i].non_blocking != 0U) && (socket_arr[i].state == SOCKET_STATE_CONNECTING)) {
                    if ((socket_arr[i].bound == 0U) || ((socket_arr[i].bound != 0U) && (socket_arr[i].local_port == 0U))) {
                      ret = SPI_StartStopTransportServerClient (hw_socket, socket_arr[i].protocol, 0U, &socket_arr[i].remote_ip[0], socket_arr[i].remote_port, TRANSPORT_START, TRANSPORT_CLIENT);
                      if (ret == 0) {
                        socket_arr[i].client = 1U;
                        socket_arr[i].state  = SOCKET_STATE_CONNECTED;
                      }
                      if (ret == ARM_SOCKET_ERROR) {
                        socket_arr[i].state = SOCKET_STATE_DISCONNECTED;
                      }
                    } else {
                      socket_arr[i].state = SOCKET_STATE_DISCONNECTED;
                    }
                  }
                }
              }

              if (poll_recv != 0U) {
                // Loop through all sockets
                for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
                  hw_socket = i;
                  if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
                    hw_socket -= WIFI_ISM43362_SOCKETS_NUM;     // Actually used socket of module
                  }
                  if (socket_arr[i].poll_recv != 0U) {
                    if (recv_len[hw_socket] != 0U) {
                      // If there is data that was already received but did not fit into 
                      // the buffer, try to fit it now
                      if (WiFi_ISM43362_BufferPut (hw_socket, &recv_buf[hw_socket][0], recv_len[hw_socket]) == true) {
                        recv_len[hw_socket] = 0U;
                        if (socket_arr[i].recv_time_left != 0U) {
                          event_socket[hw_socket] |= EVENT_SOCK_RECV;
                          socket_arr[i].recv_time_left = 0U;
                        }
                      }
                    } else {
                      // Send command to read data from remote client if socket is using receive in long blocking
                      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", hw_socket); spi_recv_len = sizeof(spi_recv_buf);
                      spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
                      if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
                        // Receive data
                        memcpy((void *)spi_send_buf, (void *)"R0\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
                        spi_ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
                        if (spi_ret == ARM_DRIVER_OK) {
                          if (resp_code == 0) {                 // If response code is OK
                            if (spi_recv_len <= 10U) {          // No data was received
                              if ((socket_arr[i].recv_time_left != 0U) &&           // If recv was called
                                  (socket_arr[i].recv_time_left != 0xFFFFFFFFU)) {  // If not infinite, check for timeout or lost connection
                                ticks = osKernelGetTickCount () - socket_arr[i].start_tick_count;
                                if (kernel_tick_freq_in_ms == 1U) {
                                  time_in_ms = ticks;
                                } else if (kernel_tick_freq_shift_to_ms != 0U) {
                                  time_in_ms = ticks >> kernel_tick_freq_shift_to_ms;
                                } else {
                                  time_in_ms = ticks / kernel_tick_freq_in_ms;
                                }
                                if (time_in_ms >= socket_arr[i].recv_time_left) {
                                  socket_arr[i].recv_time_left = 0U;
                                  event_socket[hw_socket] |= EVENT_SOCK_RECV_TIMEOUT;
                                }
                              }
                            } else {                            // Some data was received
                              // In response first 2 bytes are "\r\n", so these need to be skipped
                              recv_len[hw_socket] = spi_recv_len - 10U;
                              if (WiFi_ISM43362_BufferPut (hw_socket, &spi_recv_buf[2], spi_recv_len - 10U) == true) {
                                recv_len[hw_socket] = 0U;
                                if (socket_arr[i].recv_time_left != 0U) {
                                  socket_arr[i].recv_time_left = 0U;
                                  event_socket[hw_socket] |= EVENT_SOCK_RECV;
                                }
                                repeat = 1U;
                              } else {
                                // If data did not fit into buffer, keep it in intermediate buffer
                                memcpy((void *)&recv_buf[hw_socket][2], (const void *)spi_recv_buf, spi_recv_len - 10U);
                              }
                            }
                          } else if (resp_code == -1) {         // If response code is -1, connection was lost
                            socket_arr[i].poll_recv = 0U;
                            socket_arr[i].recv_time_left = 0U;
                            event_socket[hw_socket] |= EVENT_SOCK_RECV_DISCON_REMOTE;
                          } else {                              // If unknown error happened
                            socket_arr[i].poll_recv = 0U;
                            socket_arr[i].recv_time_left = 0U;
                            event_socket[hw_socket] |= EVENT_SOCK_RECV_DISCON_REMOTE;
                          }
                        } else {                                // If error on SPI happened
                          socket_arr[i].poll_recv = 0U;
                          socket_arr[i].recv_time_left = 0U;
                          event_socket[hw_socket] |= EVENT_SOCK_RECV_DISCON_REMOTE;
                        }
                      }
                    }
                  }
                }
              }
              osMutexRelease(mutex_id_spi);
            }
            osMutexRelease(mutex_id_sockets);
          }

          if ((event_signal != 0U) && (signal_event_fn != NULL)) {
            signal_event_fn(ARM_WIFI_EVENT_AP_CONNECT, (void *)&ap_mac[ap_num_connected-1][0]);
          }
          for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
            if (event_socket[i] != 0U) {
              osEventFlagsSet(event_flags_sockets_id[i], event_socket[i]);
            }
          }
        }

        // Wait async interval unless new event was signaled to process
        if (repeat == 0U) {
          if ((osEventFlagsWait(event_flags_id, EVENT_ASYNC_POLL, osFlagsWaitAny, WIFI_ISM43362_ASYNC_INTERVAL) & (0x80000000U | EVENT_ASYNC_POLL)) == EVENT_ASYNC_POLL) {
            repeat = 1U;
          }
        }
      } while ((poll_async != 0U) || (poll_recv != 0U) || (repeat != 0U));
    }
  }
}

// Driver Functions

/**
  \fn            ARM_DRIVER_VERSION WiFi_GetVersion (void)
  \brief         Get driver version.
  \return        ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION WiFi_GetVersion (void) { return driver_version; }

/**
  \fn            ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void)
  \brief         Get driver capabilities.
  \return        ARM_WIFI_CAPABILITIES
*/
static ARM_WIFI_CAPABILITIES WiFi_GetCapabilities (void) { return driver_capabilities; }

/**
  \fn            int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event)
  \brief         Initialize WiFi Module.
  \param[in]     cb_event Pointer to ARM_WIFI_SignalEvent_t
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Initialize (ARM_WIFI_SignalEvent_t cb_event) {
        int32_t   ret;
  const char     *ptr_str;
        uint32_t  timeout, flags;
        uint8_t   i, u8_arr[4];

  signal_event_fn = cb_event;           // Update pointer to callback function

  if (driver_initialized != 0U) {       // If driver is already initialized
    return ARM_DRIVER_OK;
  }

  ResetVariables();

  ret = ARM_DRIVER_OK;

  // Create resources required by this driver
  mutex_id_spi           = osMutexNew(&mutex_spi_attr);
  mutex_id_sockets       = osMutexNew(&mutex_socket_attr);
  event_flags_id         = osEventFlagsNew(NULL);
  if ((mutex_id_spi      == NULL) || 
      (mutex_id_sockets  == NULL) || 
      (event_flags_id    == NULL)) { 
    // If any of mutex creation has failed
    ret = ARM_DRIVER_ERROR;
  }
  for (i = 0U; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
    event_flags_sockets_id[i] = osEventFlagsNew(NULL);
    if (event_flags_sockets_id[i] == NULL) {
      // If flag creation has failed
      ret = ARM_DRIVER_ERROR;
      break;
    }
  }

  // Initialize additional pins (Reset, Slave Select, Data Ready)
  WiFi_ISM43362_Pin_Initialize();

  // Initialize SPI interface
  if (ret == ARM_DRIVER_OK) {
    ret = ptrSPI->Initialize  (SPI_SignalEvent);
  }
  if (ret == ARM_DRIVER_OK) {
    ret = ptrSPI->PowerControl(ARM_POWER_FULL);
  }
  if (ret == ARM_DRIVER_OK) {
    ret = ptrSPI->Control     (ARM_SPI_SET_DEFAULT_TX_VALUE, 0x0A0AU);
  }
  if (ret == ARM_DRIVER_OK) {
    ret = ptrSPI->Control     (ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(16), WIFI_ISM43362_SPI_BUS_SPEED);
  }

  if (ret == ARM_DRIVER_OK) {
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {

      if (module_initialized == 0U) {

        // Reset WiFi Module
        WiFi_ISM43362_Pin_RSTN(true);
        osDelay(50U);
        WiFi_ISM43362_Pin_RSTN(false);
        osDelay(750U);

        // Initial fetch cursor procedure, read (3 * 16 bits) 6 bytes
        // Do a read independent of return data
        WiFi_ISM43362_Pin_SSN(true);
        Wait_us(15U);
        memset((void *)spi_recv_buf, 0x55, sizeof(spi_recv_buf));
        if (ptrSPI->Receive(spi_recv_buf, 3U) == ARM_DRIVER_OK) {
          if (!SPI_WaitTransferDone(1000U)) {
            ret = ARM_DRIVER_ERROR;       // If transfer timed out
          }
        }
        WiFi_ISM43362_Pin_SSN(false);
        Wait_us(3U);

        // Wait for DATARDY to activate
        for (timeout = WIFI_ISM43362_SPI_TIMEOUT; timeout != 0U; timeout --) {
          if (WiFi_ISM43362_Pin_DATARDY()) {
            break;
          }
        }
        Wait_us(4U);

        module_initialized = 1U;
      }

      // Do a dummy SPI communication so we can determine if event is used for DATARDY state change
      memcpy((void *)spi_send_buf, (void *)"I?\r\n", 4);
      WiFi_ISM43362_Pin_SSN(true);              // Activate slave select line
      Wait_us(15U);                             // Wait 15 us
      if (ptrSPI->Send(spi_send_buf, 2U) == ARM_DRIVER_OK) {
        // If SPI send started successfully
        if (SPI_WaitTransferDone(100U)) {       // If SPI transfer finished
          WiFi_ISM43362_Pin_SSN(false);         // Deactivate slave select line
          Wait_us(3U);                          // Wait 3 us
        } else {                                // If SPI transfer timed-out
          ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      } else {                                  // If SPI send start failed
        ret = ARM_DRIVER_ERROR;
      }

      // Determine if SPI ready is signaled by IRQ
      for (timeout = WIFI_ISM43362_SPI_TIMEOUT; timeout != 0U; timeout --) {
        if (WiFi_ISM43362_Pin_DATARDY()) {
          flags = osEventFlagsWait(event_flags_id, EVENT_SPI_READY, osFlagsWaitAll, 10U);
          if (((flags & 0x80000000U) == 0U) && ((flags & EVENT_SPI_READY) == EVENT_SPI_READY)) {
            spi_datardy_irq = 1U;
          }
          break;
        }
        osDelay(1U);
      }
      if (timeout == 0U) {              // If DATARDY pin did not signal ready in WIFI_ISM43362_SPI_TIMEOUT seconds
        ret = ARM_DRIVER_ERROR;
      }

      // Receive data requested by 'I?' command
      Wait_us(4U);
      WiFi_ISM43362_Pin_SSN(true);
      Wait_us(15U);
      if (ptrSPI->Receive(spi_recv_buf, 64U)  == ARM_DRIVER_OK) {
        if (!SPI_WaitTransferDone(1000U)) {
          ret = ARM_DRIVER_ERROR;       // If transfer timed out
        }
      }
      WiFi_ISM43362_Pin_SSN(false);
      Wait_us(3U);

      if (osMutexRelease(mutex_id_spi) != osOK) {       // If SPI mutex release has failed
        ret = ARM_DRIVER_ERROR;
      }

      if (ret == ARM_DRIVER_OK) {
        ptr_str = strstr ((const char *)spi_recv_buf, ",C");
        if (ptr_str != NULL) {
          if (sscanf(ptr_str + 2U, "%hhu.%hhu.%hhu.%hhu", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3]) == 4) {
            firmware_version = ((uint32_t)u8_arr[0] << 24) | 
                               ((uint32_t)u8_arr[1] << 16) | 
                               ((uint32_t)u8_arr[2] <<  8) | 
                               ((uint32_t)u8_arr[3]      );
          }
        }
        ptr_str = strstr ((const char *)spi_recv_buf, "STM");
        if (ptr_str != NULL) {
          firmware_stm = 1U;
        }
      }
    } else {                            // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if ((firmware_stm == 0U) && (firmware_version != 0x06020107U)) {
    // For non-STM firmware variant only firmware version 6.2.1.7 is supported
    ret = ARM_DRIVER_ERROR;
  }

  // Create asynchronous message processing thread
  if (ret == ARM_DRIVER_OK) {
    thread_id_async_poll = osThreadNew(WiFi_AsyncMsgProcessThread, NULL, &thread_async_poll_attr);
    if (thread_id_async_poll == NULL) { // If thread creation failed
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {           // If initialization succeeded
    driver_initialized = 1U;
  } else {                              // Else if initialization failed -> cleanup
    WiFi_Uninitialize();
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Uninitialize (void)
  \brief         De-initialize WiFi Module.
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
*/
static int32_t WiFi_Uninitialize (void) {
  int32_t ret, ret_sc;
  uint8_t i;

  if (driver_initialized == 0U) {       // If driver is already uninitialized
    return ARM_DRIVER_OK;
  }

  ret = ARM_DRIVER_OK;

  // Close all open sockets
  for (i = 0U; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
    ret_sc = WiFi_SocketClose(i + WIFI_ISM43362_SOCKETS_NUM);
    if ((ret_sc != 0) && (ret_sc != ARM_SOCKET_ESOCK)) {
      ret = ARM_DRIVER_ERROR;
      break;
    }
    ret_sc = WiFi_SocketClose(i);
    if ((ret_sc != 0) && (ret_sc != ARM_SOCKET_ESOCK)) {
      ret = ARM_DRIVER_ERROR;
      break;
    }
  }

  // Release all resources
  if (thread_id_async_poll != NULL) {
    if (osThreadTerminate(thread_id_async_poll) == osOK) {
      thread_id_async_poll = NULL;
    }
  }
  for (i = 0U; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
    if (osEventFlagsDelete(event_flags_sockets_id[i]) == osOK) {
      event_flags_sockets_id[i] = NULL;
    } else {
      ret = ARM_DRIVER_ERROR;
      break;
    }
  }
  if (event_flags_id != NULL) {
    if (osEventFlagsDelete(event_flags_id) == osOK) {
      event_flags_id = NULL;
    }
  }
  if (mutex_id_sockets != NULL) {
    if (osMutexDelete(mutex_id_sockets) == osOK) {
      mutex_id_sockets = NULL;
    }
  }
  if (mutex_id_spi != NULL) {
    if (osMutexDelete(mutex_id_spi) == osOK) {
      mutex_id_spi = NULL;
    }
  }

  if ((ret == ARM_DRIVER_OK)           &&
     ((thread_id_async_poll   != NULL) ||
      (event_flags_id         != NULL) ||
      (mutex_id_sockets       != NULL) ||
      (mutex_id_spi           != NULL))){
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {           // If uninitialization succeeded
    // Uninitialize additional pins (Reset, Slave Select, Data Ready)
    WiFi_ISM43362_Pin_Uninitialize();

    signal_event_fn = NULL;             // Clear pointer to callback function
    ResetVariables();
    driver_initialized = 0U;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_PowerControl (ARM_POWER_STATE state)
  \brief         Control WiFi Module Power.
  \param[in]     state     Power state
                   - ARM_POWER_OFF                : Power off: no operation possible
                   - ARM_POWER_LOW                : Low-power mode: sleep or deep-sleep depending on ARM_WIFI_LP_TIMER option set
                   - ARM_POWER_FULL               : Power on: full operation at maximum performance
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid state)
*/
static int32_t WiFi_PowerControl (ARM_POWER_STATE state) {
  int32_t  ret;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    switch (state) {
      case ARM_POWER_OFF:
        // Module does not support power-off
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_POWER_LOW:
        if ((sta_lp_time != 0U) && (oper_mode == OPER_MODE_STATION)) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "ZP=6,%d\r", sta_lp_time);
        } else if (ap_beacon_interval != 0U) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "ZP=2,%d\r", ap_beacon_interval);
        } else {
          memcpy((void *)spi_send_buf, (void *)"ZP=1,1\r\n", 9);
        }
        break;

      case ARM_POWER_FULL:
        memcpy((void *)spi_send_buf, (void *)"ZP=1,0\r\n", 9);
        break;

#if !defined (__ARMCC_VERSION) || (__ARMCC_VERSION < 6010050)
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
#endif
    }

    // Execute command and receive response
    if (ret == ARM_DRIVER_OK) {
      spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
        ret = ARM_DRIVER_ERROR;
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_GetModuleInfo (char *module_info, uint32_t max_len)
  \brief         Get Module information.
  \param[out]    module_info Pointer to character buffer were info string will be returned
  \param[in]     max_len     Maximum length of string to return (including null terminator)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL module_info pointer or max_len equals to 0)
*/
static int32_t WiFi_GetModuleInfo (char *module_info, uint32_t max_len) {
  int32_t  ret;
  uint32_t copy_len;

  if ((module_info == NULL) || (max_len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  // Show Applications Information
  // Correct response looks like: "\r\nMODULE INFO\r\nOK\r\n> "
  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    memcpy((void *)spi_send_buf, "I?\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
    ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if (ret == ARM_DRIVER_OK) {
      if ((resp_code == 0U) && (spi_recv_len > 10U)) {
        copy_len = spi_recv_len - 10U;
        if (copy_len >= max_len) {
          copy_len = max_len - 1U;
        }
        if (copy_len > 0) {
          memcpy ((void *)module_info, (void *)&spi_recv_buf[2], copy_len);
        }
        module_info[copy_len] = 0;
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SetOption (uint32_t interface, uint32_t option, const void *data, uint32_t len)
  \brief         Set WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to set
  \param[in]     data      Pointer to data relevant to selected option
  \param[in]     len       Length of data (in bytes)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL data pointer or len less than option specifies)
*/
static int32_t WiFi_SetOption (uint32_t interface, uint32_t option, const void *data, uint32_t len) {
        int32_t  ret;
  const uint8_t *ptr_u8_data;
        uint32_t u32;
        uint8_t  exec_cmd;

  if ((interface > 1U) || (data == NULL) || (len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  exec_cmd = 1U;
  ptr_u8_data = (const uint8_t *)data;

  switch (option) {
    case ARM_WIFI_LP_TIMER:                 // Station    Set low-power deep-sleep time;              data = &time,     len =  4, uint32_t [seconds]: 0 = disable (default)
      if (interface == 0U) {
        u32 = *((const uint32_t *)data);
        if (u32 <= 3600000U) {
          sta_lp_time = u32;
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_BEACON:                   //         AP Set beacon interval;                        data = &interval, len =  4, uint32_t [ms]
      if (interface == 1U) {
        u32 = *((const uint32_t *)data);
        if ((u32 >= 1000U) && (u32 <= 60000U)) {
          // Convert to seconds
          ap_beacon_interval = (uint8_t)(u32 / 1000);
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
      } else {
        // Not supported for Station interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_MAC:                      // Station/AP Set MAC;                                    data = &mac,      len =  6, uint8_t[6]
      if (len >= 6U) {
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "Z4=%02X:%02X:%02X:%02X:%02X:%02X\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3], ptr_u8_data[4], ptr_u8_data[5]);
      } else {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      }
      break;

    case ARM_WIFI_IP:                       // Station/AP Set IPv4 static/assigned address;           data = &ip,       len =  4, uint8_t[4]
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      if (interface == 1U) {
        spi_send_buf[0] = 'Z';
      }
      break;

    case ARM_WIFI_IP_SUBNET_MASK:           // Station/AP Set IPv4 subnet mask;                       data = &mask,     len =  4, uint8_t[4]
      if (interface == 0U) {
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C7=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_IP_GATEWAY:               // Station/AP Set IPv4 gateway address;                   data = &ip,       len =  4, uint8_t[4]
      if (interface == 0U) {
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C8=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_IP_DNS1:                  // Station/AP Set IPv4 primary   DNS address;             data = &ip,       len =  4, uint8_t[4]
      if (interface == 0U) {
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C9=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_IP_DNS2:                  // Station/AP Set IPv4 secondary DNS address;             data = &ip,       len =  4, uint8_t[4]
      if (interface == 0U) {
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "CA=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_IP_DHCP:                  // Station/AP Set IPv4 DHCP client/server enable/disable; data = &dhcp,     len =  4, uint32_t: 0 = disable, non-zero = enable (default)
      u32 = *((const uint32_t *)data);
      if (interface == 0U) {
        memcpy((void *)spi_send_buf, "C4=0\r\n", 7);
        if (u32 != 0U) {
          spi_send_buf[3] = '1';
        }
        // Store set value to local variable
        sta_dhcp_client = (uint8_t)u32;
      } else {  // For AP interface
        if (u32 == 0U) {
          // DHCP server cannot be disabled
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        exec_cmd = 0U;  // Do not execute SPI command
      }
      break;

    case ARM_WIFI_IP_DHCP_LEASE_TIME:       //         AP Set IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
      if (interface == 1U) {
        u32 = *((const uint32_t *)data);
        if ((u32 >= (30U * 60U)) &&         // If more then 30 minutes
            (u32 <= (254U * 60U * 60U))) {  // If less then 254 hours
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "AL=%d\r", u32/(60U*60U));
          // Store set value to local variable
          ap_dhcp_lease_time = (u32/(60U*60U))*60U*60U;
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
      } else {
        // Not supported for Station interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  if ((ret == ARM_DRIVER_OK) && (exec_cmd != 0U)) {     // If command should be sent through SPI
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      osMutexRelease(mutex_id_spi);
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  return ret;
}

/**
  \fn            int32_t WiFi_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len)
  \brief         Get WiFi Module Options.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     option    Option to get
  \param[out]    data      Pointer to memory where data for selected option will be returned
  \param[in,out] len       Pointer to length of data (input/output)
                   - input: maximum length of data that can be returned (in bytes)
                   - output: length of returned data (in bytes)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL data or len pointer, or *len less than option specifies)
*/
static int32_t WiFi_GetOption (uint32_t interface, uint32_t option, void *data, uint32_t *len) {
        int32_t   ret;
  const uint8_t  *ptr_resp_buf;
        uint8_t  *ptr_u8_data;
        uint8_t   mutex_acquired;

  if ((interface > 1U) ||  (data == NULL) || (len == NULL) || (*len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  mutex_acquired = 0U;

  if ((option != ARM_WIFI_IP_DHCP_LEASE_TIME) && (option != ARM_WIFI_LP_TIMER)) {
    // For all options except ARM_WIFI_IP_DHCP_LEASE_TIME and ARM_WIFI_LP_TIMER read data through SPI from module is necessary
    // For unsupported commands Network Settings will be read unnecessarily but as unsupported command 
    // is not critical that dummy read is irrelevant
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      mutex_acquired = 1U;
      if        (option == ARM_WIFI_MAC) {
        memcpy((void *)spi_send_buf, "Z5\r\n", 4);   // Get MAC Address
      } else if ((interface == 1U) && (option == ARM_WIFI_IP)) {
        memcpy((void *)spi_send_buf, "A?\r\n", 4);   // Show Access Point Settings
      } else {
        memcpy((void *)spi_send_buf, "C?\r\n", 4);   // Show Network Settings
      }
      spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret != ARM_DRIVER_OK) {
    if (mutex_acquired != 0U) {
      osMutexRelease(mutex_id_spi);
    }
    return ret;
  }

  switch (option) {
    case ARM_WIFI_LP_TIMER:                 // Station    Get low-power deep-sleep time;              data = &time,     len =  4, uint32_t [seconds]: 0 = disable (default)
      if (interface == 0U) {
        *((uint32_t *)data) = sta_lp_time;
        *len = 4U;
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_BEACON:                   //         AP Get beacon interval;                        data = &interval, len =  4, uint32_t [ms]
      if (interface == 1U) {
        // Convert to milliseconds
        *((uint32_t *)data) = (uint32_t)ap_beacon_interval * 1000;
        *len = 4U;
      } else {
        // Not supported for Station interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_MAC:                      // Station/AP Get MAC;                                    data = &mac,      len =  6, uint8_t[6]
      if (*len >= 6U) {
        // Extract MAC from response on "Z5" command
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)&spi_recv_buf[2], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3], &ptr_u8_data[4], &ptr_u8_data[5]) == 6) {
          *len = 6U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      }
      break;

    case ARM_WIFI_IP:                       // Station/AP Get IPv4 static/assigned address;           data = &ip,       len =  4, uint8_t[4]
      if (interface == 0U) {                // For Station interface
        // Skip ssid, password, security, DHCP, IP version (5 ',') from response on "C?" command
        ptr_resp_buf = SkipCommas(spi_recv_buf, 5U);
      } else {                              // For AP interface
        // Skip ssid (1 ',') from response on "A?" command
        ptr_resp_buf = SkipCommas(spi_recv_buf, 1U);
      }
      if (ptr_resp_buf != NULL) {
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) == 4) {
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      break;

    case ARM_WIFI_IP_SUBNET_MASK:           // Station/AP Get IPv4 subnet mask;                       data = &mask,     len =  4, uint8_t[4]
      // Skip ssid, password, security, DHCP, IP version, IP address (6 ',') from response on "C?" command
      ptr_resp_buf = SkipCommas(spi_recv_buf, 6U);
      if (ptr_resp_buf != NULL) {
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) == 4) {
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      break;

    case ARM_WIFI_IP_GATEWAY:               // Station/AP Get IPv4 gateway address;                   data = &ip,       len =  4, uint8_t[4]
      // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask (7 ',') from response on "C?" command
      ptr_resp_buf = SkipCommas(spi_recv_buf, 7U);
      if (ptr_resp_buf != NULL) {
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) == 4) {
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      break;

    case ARM_WIFI_IP_DNS1:                  // Station/AP Get IPv4 primary   DNS address;             data = &ip,       len =  4, uint8_t[4]
      // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP 
      // (8 ',') from response on "C?" command
      ptr_resp_buf = SkipCommas(spi_recv_buf, 8U);
      if (ptr_resp_buf != NULL) {
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) == 4) {
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      break;

    case ARM_WIFI_IP_DNS2:                  // Station/AP Get IPv4 secondary DNS address;             data = &ip,       len =  4, uint8_t[4]
      // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP, primary DNS IP 
      // (9 ',') from response on "C?" command
      ptr_resp_buf = SkipCommas(spi_recv_buf, 9U);
      if (ptr_resp_buf != NULL) {
        ptr_u8_data = (uint8_t *)data;
        if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) == 4) {
          *len = 4U;
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      break;

    case ARM_WIFI_IP_DHCP:                  // Station/AP Get IPv4 DHCP client/server enable/disable; data = &dhcp,     len =  4, uint32_t: 0 = disable, non-zero = enable (default)
      if (interface == 0U) {
        *((uint32_t *)data) = sta_dhcp_client;
      } else {  // For AP interface
        *((uint32_t *)data) = 1U;
      }
      *len = 4U;
      break;

    case ARM_WIFI_IP_DHCP_LEASE_TIME:       //         AP Get IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
      if (interface == 1U) {
        if (ap_dhcp_lease_time == 0U) {
          *((uint32_t *)data) = 30U * 60U;
        } else {
          *((uint32_t *)data) = ap_dhcp_lease_time;
        }
        *len = 4U;
      } else {
        // Not supported for Station interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  if (mutex_acquired != 0U) {
    osMutexRelease(mutex_id_spi);
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], uint32_t max_num)
  \brief         Scan for available networks in range.
  \param[out]    scan_info Pointer to array of ARM_WIFI_SCAN_INFO_t structures where available Scan Information will be returned
  \param[in]     max_num   Maximum number of Network Information structures to return
  \return        number of ARM_WIFI_SCAN_INFO_t structures returned or error code
                   - value >= 0                   : Number of ARM_WIFI_SCAN_INFO_t structures returned
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL scan_info pointer or max_num equal to 0)
*/
static int32_t WiFi_Scan (ARM_WIFI_SCAN_INFO_t scan_info[], uint32_t max_num) {
        int32_t  ret;
  const uint8_t *ptr_resp_buf;
        int32_t  i, int_val;

  i = 0;

  if ((scan_info == NULL) || (max_num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Clear scan_info array
    memset((void *)scan_info, 0, sizeof(ARM_WIFI_SCAN_INFO_t) * max_num);

    // Scan for network access points
    memcpy((void *)spi_send_buf, "F0\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
    ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
      ret = ARM_DRIVER_ERROR;
    }

    // Extract scan data
    if (ret == ARM_DRIVER_OK) {
      ptr_resp_buf = &spi_recv_buf[2];
      while ((ret == ARM_DRIVER_OK) && (ptr_resp_buf != NULL)) {
        if (sscanf((const char *)ptr_resp_buf, "#%d", &i) != 1) {
          break;
        }
        if (i < 0) {
          i = 0;
          break;
        }
        if (i > (int32_t)max_num) {
          i = (int32_t)max_num;
          break;
        }
        i--;
        // Position pointer 1 character after next ',' (skip ",C")
        // Parse SSID
        // Skip index (1 ',') and '"'
        ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
        if (ptr_resp_buf != NULL) {
          ptr_resp_buf++;          // Skip '"'
          // Extract SSID
          if (sscanf((const char *)ptr_resp_buf, "%32[^\"]", scan_info[i].ssid) != 1) {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR;
        }

        // Parse BSSID
        if (ret == ARM_DRIVER_OK) {
          // Skip 1 comma
          ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
          if (ptr_resp_buf != NULL) {
            // Extract BSSID
            if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &scan_info[i].bssid[0], &scan_info[i].bssid[1], &scan_info[i].bssid[2], &scan_info[i].bssid[3], &scan_info[i].bssid[4], &scan_info[i].bssid[5]) != 6) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Parse RSSI
        if (ret == ARM_DRIVER_OK) {
          // Skip 1 comma
          ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
          if (ptr_resp_buf != NULL) {
            // Extract RSSI
            if (sscanf((const char *)ptr_resp_buf, "%d", &int_val) == 1) {
              scan_info[i].rssi = (uint8_t)(int_val + 255);
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Skip bitrate, wireless mode and parse security
        if (ret == ARM_DRIVER_OK) {
          // Skip 3 commas
          ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 3U);
          if (ptr_resp_buf != NULL) {
            // Extract security and cipher
            if        (memcmp((const void *)ptr_resp_buf, (const void *)"WPA2", 4) == 0) {
              scan_info[i].security = ARM_WIFI_SECURITY_WPA2;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"WPA",  3) == 0) {
              scan_info[i].security = ARM_WIFI_SECURITY_WPA;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"WEP",  3) == 0) {
              scan_info[i].security = ARM_WIFI_SECURITY_WEP;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"Open", 4) == 0) {
              scan_info[i].security = ARM_WIFI_SECURITY_OPEN;
            } else {
              scan_info[i].security = ARM_WIFI_SECURITY_UNKNOWN;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Skip frequency and parse channel
        if (ret == ARM_DRIVER_OK) {
          // Skip 2 commas
          ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 2U);
          if (ptr_resp_buf != NULL) {
            // Extract channel
            if (sscanf((const char *)ptr_resp_buf, "%hhu", &scan_info[i].ch) != 1) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Position pointer to next '#'
        while ((*ptr_resp_buf != '#') && (*ptr_resp_buf != 0U)) {
          ptr_resp_buf++;
        }
        if (*ptr_resp_buf != '#') {
          ptr_resp_buf = NULL;
        }
      }
      if (i < (int32_t)max_num) {
        i++;
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = i;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Activate (uint32_t interface, const ARM_WIFI_CONFIG_t *config)
  \brief         Activate interface (Connect to a wireless network or activate an access point).
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     config    Pointer to ARM_WIFI_CONFIG_t structure where Configuration parameters are located
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported (security type, channel autodetect or WPS not supported)
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, NULL config pointer or invalid configuration)
*/
static int32_t WiFi_Activate (uint32_t interface, const ARM_WIFI_CONFIG_t *config) {
        int32_t  ret;
  const uint8_t *ptr_resp_buf;

  if ((interface > 1U) || (config == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (config == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (config->wps_method == ARM_WIFI_WPS_METHOD_NONE) {
    // For station connect if WPS is not used do a sanity check for ssid and security

    // SSID has to be a valid pointer
    if (config->ssid == NULL) {
      return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (config->security) {
      case ARM_WIFI_SECURITY_OPEN:
        break;
      case ARM_WIFI_SECURITY_WEP:
      case ARM_WIFI_SECURITY_WPA:
      case ARM_WIFI_SECURITY_WPA2:
        // Password has to be a valid pointer
        if (config->pass == NULL) {
          return ARM_DRIVER_ERROR_PARAMETER;
        }
        break;
      case ARM_WIFI_SECURITY_UNKNOWN:
      default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
  }

  // Valid channel settings are: 0 for auto, 1 to 13 for 2.4 GHz and
  // 36, 40, 44, 48, 149, 153, 157, 161, 165 for 5 GHz exact channel selection
  if (config->ch > 13U) {
    switch (config->ch) {
      case 36:
      case 40:
      case 44:
      case 48:
      case 149:
      case 153:
      case 157:
      case 161:
      case 165:
        // Allowed 5 GHz channels
        break;
      default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
  }

  switch (config->wps_method) {
    case ARM_WIFI_WPS_METHOD_NONE:
    case ARM_WIFI_WPS_METHOD_PBC:
      break;
    case ARM_WIFI_WPS_METHOD_PIN:
      // PIN has to be a valid pointer
      if (config->wps_pin == NULL) {
        return ARM_DRIVER_ERROR_PARAMETER;
      }
      break;
    default:
      return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    if (interface == 0U) {              // Station
      if (config->wps_method != ARM_WIFI_WPS_METHOD_NONE) {     // If WPS is configured
        switch (config->wps_method) {
          case ARM_WIFI_WPS_METHOD_PBC:
            break;
          case ARM_WIFI_WPS_METHOD_PIN:
            snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "Z7=%s\r", config->wps_pin); spi_recv_len = sizeof(spi_recv_buf);
            ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
            if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
              ret = ARM_DRIVER_ERROR;
            }
            break;
          default:
            ret = ARM_DRIVER_ERROR_PARAMETER;
            break;
        }

        // Prepare WPS command
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)spi_send_buf, (void *)"CW= \r\n", 6);
          switch (config->wps_method) {
            case ARM_WIFI_WPS_METHOD_PBC:
              // Prepare WPS push-button connection command
              spi_send_buf[3] = '1';
              break;
            case ARM_WIFI_WPS_METHOD_PIN:
              // Prepare WPS PIN connection command
              spi_send_buf[3] = '0';
              break;
            default:
              ret = ARM_DRIVER_ERROR_PARAMETER;
              break;
          }
        }
        // Execute command
        if (ret == ARM_DRIVER_OK) {
          spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Check if WPS connection has succeeded
        if (ret == ARM_DRIVER_OK) {
          ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "WPS ");
          if (ptr_resp_buf != NULL) {
            if (strstr((const char *)spi_recv_buf, "No access point found") != NULL) {
              // Check if WPS connection failed
              ret = ARM_DRIVER_ERROR_TIMEOUT;
            } else {
              // If message contains "WPS " string, parse it and extract ip
              // Skip 1 comma
              ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
              if (ptr_resp_buf != NULL) {
                // Extract IP Address
                if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &sta_local_ip[0], &sta_local_ip[1], &sta_local_ip[2], &sta_local_ip[3]) != 4) {
                  ret = ARM_DRIVER_ERROR;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
      } else {                                                  // If WPS is not configured
        if (config->ch != 0U) {
          // Exact channel is not supported only auto
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        // Set network SSID
        if (ret == ARM_DRIVER_OK) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C1=%s\r", config->ssid); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set network passphrase
        if ((ret == ARM_DRIVER_OK) && (config->security != ARM_WIFI_SECURITY_OPEN)) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "C2=%s\r", config->pass); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set network security mode
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)spi_send_buf, "C3= \r\n", 6);
          switch (config->security) {
            case ARM_WIFI_SECURITY_OPEN:
              spi_send_buf[3] = '0';
              break;
            case ARM_WIFI_SECURITY_WEP:
              spi_send_buf[3] = '1';
              break;
            case ARM_WIFI_SECURITY_WPA:
              spi_send_buf[3] = '2';
              break;
            case ARM_WIFI_SECURITY_WPA2:
              spi_send_buf[3] = '3';
              break;
            default:
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
          }
          if (ret == ARM_DRIVER_OK) {
            spi_recv_len = sizeof(spi_recv_buf);
            ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
            if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
              ret = ARM_DRIVER_ERROR;
            }
          }
        }

        // Send command to join a network
        if (ret == ARM_DRIVER_OK) {
          // If IP that we got is 0.0.0.0 then try again for max 3 times to get valid IP
          memcpy((void *)spi_send_buf, "C0\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
          
          // Check if connection has succeeded
          if (ret == ARM_DRIVER_OK) {
            ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "JOIN ");
            if (ptr_resp_buf != NULL) {
              // If message contains "JOIN " string, parse it and extract IP
              // Skip 1 comma
              ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
              if (ptr_resp_buf != NULL) {
                // Extract IP Address
                if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &sta_local_ip[0], &sta_local_ip[1], &sta_local_ip[2], &sta_local_ip[3]) == 4) {
                  if ((sta_local_ip[0] == 0) && (sta_local_ip[1] == 0) && (sta_local_ip[2] == 0) && (sta_local_ip[3] == 0)) {
                    ret = ARM_DRIVER_ERROR;
                  }
                } else {
                  ret = ARM_DRIVER_ERROR;
                }
              }
            }
          } else if (ret == ARM_DRIVER_ERROR) {
            // Check if connection is already established
            ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "Already connected!");
            if (ptr_resp_buf != NULL) {
              ret = ARM_DRIVER_OK;
            }
          }
        }
      }

      if (ret == ARM_DRIVER_OK) {
        oper_mode = OPER_MODE_STATION;
      } else {
        memset((void *)sta_local_ip, 0, 4);
      }
    } else {                            // AP
      if (config->wps_method != ARM_WIFI_WPS_METHOD_NONE) {     // If WPS is configured
        // WPS is not supported for Access Point
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      } else {                                                  // If WPS is not configured
        // Set AP security mode
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)spi_send_buf, "A1= \r\n", 6);
          switch (config->security) {
            case ARM_WIFI_SECURITY_OPEN:
              spi_send_buf[3] = '0';
              break;
            case ARM_WIFI_SECURITY_WEP:
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
            case ARM_WIFI_SECURITY_WPA:
              spi_send_buf[3] = '2';
              break;
            case ARM_WIFI_SECURITY_WPA2:
              spi_send_buf[3] = '3';
              break;
            default:
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
          }
        }
        if (ret == ARM_DRIVER_OK) {
          spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set AP security key (password)
        if (ret == ARM_DRIVER_OK) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "A2=%s\r", config->pass); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set AP channel (0 = autoselect)
        if (ret == ARM_DRIVER_OK) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "AC=%d\r\n", config->ch); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set AP SSID
        if (ret == ARM_DRIVER_OK) {
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "AS=0,%s\r", config->ssid); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Set AP maximum number of clients to maximum which is 4
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)spi_send_buf, "AT=4\r\n", 6); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Activate AP direct connect mode
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)spi_send_buf, "AD\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }

          // Check if AP has started and extract IP address from response
          if (ret == ARM_DRIVER_OK) {
            ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "[AP ");
            if (ptr_resp_buf != NULL) {
              // Skip 2 commas
              ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 2U);
              if (ptr_resp_buf != NULL) {
                // Extract IP Address
                if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ap_local_ip[0], &ap_local_ip[1], &ap_local_ip[2], &ap_local_ip[3]) != 4) {
                  ret = ARM_DRIVER_ERROR;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else if (ret == ARM_DRIVER_ERROR) {
            // Check if AP is already running
            ptr_resp_buf = (uint8_t *)strstr((const char *)spi_recv_buf, "Already running");
            if (ptr_resp_buf != NULL) {
              ret = ARM_DRIVER_OK;
            }
          }
        }

        // Dummy read Access Point Settings as WEB Server can fail in which case 
        // module returns "[WEB SVR] Failed to listen on server socket"
        if (ret == ARM_DRIVER_OK) {
          osDelay(300U);    // Allow time for WEB SVR to start
          memcpy((void *)spi_send_buf, "A?\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
          ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        if (ret == ARM_DRIVER_OK) {
          oper_mode = OPER_MODE_AP;
          osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);
        } else {
          memset((void *)ap_local_ip, 0, 4);
        }
      }
    }

    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Deactivate (uint32_t interface)
  \brief         Deactivate interface (Disconnect from a wireless network or deactivate an access point).
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface)
*/
static int32_t WiFi_Deactivate (uint32_t interface) {
  int32_t ret;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    if ((interface == 0U) && (oper_mode == OPER_MODE_STATION)) {
      // Disconnect from network
      memcpy((void *)spi_send_buf, "CD\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if (ret == ARM_DRIVER_OK) {
        if (resp_code == 0) {
          oper_mode = 0U;
          memset((void *)sta_local_ip, 0, 4);
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }
    }

    if ((interface == 1U) && (oper_mode == OPER_MODE_AP)) {
      // Exit AP direct connect mode
      memcpy((void *)spi_send_buf, "AE\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if (ret == ARM_DRIVER_OK) {
        if (resp_code == 0) {
          oper_mode = 0U;
          memset((void *)ap_local_ip, 0, 4);
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }
    }

    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  return ret;
}

/**
  \fn            uint32_t WiFi_IsConnected (void)
  \brief         Get station connection status.
  \return        station connection status
                   - value != 0: Station connected
                   - value = 0: Station not connected
*/
static uint32_t WiFi_IsConnected (void) {
  int32_t  status;
  uint32_t con;

  if (driver_initialized == 0U) {
    return 0U;
  }

  con = 0U;

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    status = ARM_DRIVER_OK;

    // Check station connection status
    memcpy((void *)spi_send_buf, "CS\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
    status = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((status == ARM_DRIVER_OK) && (resp_code == 0)) {
      if (spi_recv_buf[2] == '1') {
        con = 1U;
      }
    }
    if (con == 0U) {
      if (oper_mode == OPER_MODE_STATION) {
        oper_mode = 0U;
      }
      memset((void *)sta_local_ip, 0, 4);
    }

    osMutexRelease(mutex_id_spi);
  }

  return con;
}

/**
  \fn            int32_t WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info)
  \brief         Get station Network Information.
  \param[out]    net_info  Pointer to ARM_WIFI_NET_INFO_t structure where station Network Information will be returned
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed (station not connected)
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface or NULL net_info pointer)
*/
static int32_t WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info) {
        int32_t  ret;
  const uint8_t *ptr_resp_buf;
        int      int_val;

  if (net_info == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Show Network Settings
    memcpy((void *)spi_send_buf, "C?\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
    ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((ret == ARM_DRIVER_OK) && (resp_code == 0)) {
      ptr_resp_buf = SkipCommas(spi_recv_buf, 14U);
      if (ptr_resp_buf != NULL) {
        if (*ptr_resp_buf == '1') {     // If station is connected
          // Channel is not available so clear it
          net_info->ch = 0U;

          // Extract ssid
          if (ret == ARM_DRIVER_OK) {
            if (sscanf((const char *)&spi_recv_buf[2], "%32[^,]s", net_info->ssid) != 1) {
              ret = ARM_DRIVER_ERROR;
            }
          }

          // Extract password
          // Skip ssid (1 ',')
          if (ret == ARM_DRIVER_OK) {
            ptr_resp_buf = SkipCommas(spi_recv_buf, 1U);
            if (ptr_resp_buf != NULL) {
              if (sscanf((const char *)ptr_resp_buf, "%64[^,]s", net_info->pass) != 1) {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }

          // Extract security and cipher
          // Skip password (1 ',')
          if (ret == ARM_DRIVER_OK) {
            ptr_resp_buf = SkipCommas(ptr_resp_buf, 1U);
            if (ptr_resp_buf != NULL) {
              switch (*ptr_resp_buf) {
                case '0':
                  net_info->security = ARM_WIFI_SECURITY_OPEN;
                  break;
                case '1':
                  net_info->security = ARM_WIFI_SECURITY_UNKNOWN;
                  break;
                case '2':
                  net_info->security = ARM_WIFI_SECURITY_WPA;
                  break;
                case '3':
                  net_info->security = ARM_WIFI_SECURITY_WPA2;
                  break;
                case '4':
                  net_info->security = ARM_WIFI_SECURITY_WPA2;
                  break;
                default:
                  net_info->security = ARM_WIFI_SECURITY_UNKNOWN;
                  break;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }

          // Get RSSI of Associated Access Point
          if (ret == ARM_DRIVER_OK) {
            memcpy((void *)spi_send_buf, "CR\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
            ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
            if (ret == ARM_DRIVER_OK) {
              if (resp_code == 0) {
                if (sscanf((const char *)&spi_recv_buf[2], "%d", &int_val) == 1) {
                  net_info->rssi = (uint8_t)(int_val + 256);
                } else {
                  ret = ARM_DRIVER_ERROR;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            }
          }
        } else {                        // If station is not connected
          ret = ARM_DRIVER_ERROR;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }

    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol)
  \brief         Create a communication socket.
  \param[in]     af       Address family
  \param[in]     type     Socket type
  \param[in]     protocol Socket protocol
  \return        status information
                   - Socket identification number (>=0)
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ENOMEM            : Not enough memory
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketCreate (int32_t af, int32_t type, int32_t protocol) {
  int32_t ret;
  uint8_t i;

  switch (af) {
    case ARM_SOCKET_AF_INET:
      break;
    case ARM_SOCKET_AF_INET6:
      return ARM_SOCKET_ENOTSUP;
    default:
      return ARM_SOCKET_EINVAL;
  }

  switch (type) {
    case ARM_SOCKET_SOCK_DGRAM:
      if (protocol == 0) {              // If default protocol
        protocol = ARM_SOCKET_IPPROTO_UDP;
      }
      if (protocol != ARM_SOCKET_IPPROTO_UDP) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    case ARM_SOCKET_SOCK_STREAM:
      if (protocol == 0) {              // If default protocol
        protocol = ARM_SOCKET_IPPROTO_TCP;
      }
      if (protocol != ARM_SOCKET_IPPROTO_TCP) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  switch (protocol) {
    case ARM_SOCKET_IPPROTO_TCP:
      break;
    case ARM_SOCKET_IPPROTO_UDP:
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Find free socket entry in socket_arr
    for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
      if (socket_arr[i].state == SOCKET_STATE_FREE) {
        break;
      }
    }
    if (i == WIFI_ISM43362_SOCKETS_NUM) {
      ret = ARM_SOCKET_ENOMEM;          // No free socket is available
    }

    if (ret >= 0) {
      osEventFlagsClear(event_flags_sockets_id[ret], 0xFFU);

      WiFi_ISM43362_BufferInitialize (i);

      // If socket creation succeeded
      memset((void *)&socket_arr[i], 0, sizeof(socket_t));
      socket_arr[i].protocol = (uint8_t)protocol;
      socket_arr[i].state    = SOCKET_STATE_CREATED;
      ret = i;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Assign a local address to a socket.
  \param[in]     socket   Socket identification number
  \param[in]     ip       Pointer to local IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Local port number
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (address or socket already bound)
                   - ARM_SOCKET_EADDRINUSE        : Address already in use
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketBind (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
  int32_t ret, i;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len != 4U) || (port == 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }
  if ((socket_arr[socket].state == SOCKET_STATE_CONNECTED) || (socket_arr[socket].state == SOCKET_STATE_CONNECTING)) {
    return ARM_SOCKET_EISCONN;
  }
  if ((socket_arr[socket].bound != 0U) || (socket_arr[socket].state != SOCKET_STATE_CREATED)) {
    return ARM_SOCKET_EINVAL;
  }
  if (                                    (memcmp((const void *)ip, (const void *)"\x00\x00\x00\x00", 4U) != 0)   &&
     ((oper_mode == OPER_MODE_STATION) && (memcmp((const void *)ip, (const void *)sta_local_ip, 4U)       != 0))  &&
     ((oper_mode == OPER_MODE_AP)      && (memcmp((const void *)ip, (const void *)ap_local_ip,  4U)       != 0)))  {
    // If IP is different then 0.0.0.0 (accept all) or 
    // if station is active and requested IP is different then station's IP or 
    // if AP is running and requested IP is different then AP's IP
    return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
      if ((socket_arr[i].bound != 0U) &&
          (memcmp((const void *)ip, (const void *)socket_arr[i].bound_ip, 4U) == 0) && 
          (socket_arr[i].local_port == port)) {
        // If IP and port is already used by another socket
        ret = ARM_SOCKET_EADDRINUSE;
        break;
      }
    }

    if (ret == 0) {
      // Execute functionality on the module through SPI commands
      if ((socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) && (socket_arr[socket].server == 0U)) {
        // For UDP bind sets up filter and in server mode UDP can be received
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, port, NULL, 0U, TRANSPORT_START, TRANSPORT_SERVER);
          if (ret == 0) {
            socket_arr[socket].server = 1U;

            // Start polling for reception
            socket_arr[socket].poll_recv = 1U;
            osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);      // Trigger asynchronous thread read
          }
          osMutexRelease(mutex_id_spi);
        } else {
          ret = ARM_SOCKET_ERROR;
        }
      }
    }

    if (ret == 0) {
      // If socket bind succeeded
      memcpy((void *)socket_arr[socket].bound_ip, (const void *)ip, 4);
      socket_arr[socket].local_port = port;
      socket_arr[socket].bound      = 1U;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketListen (int32_t socket, int32_t backlog)
  \brief         Listen for socket connections.
  \param[in]     socket   Socket identification number
  \param[in]     backlog  Number of connection requests that can be queued
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (socket not bound)
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_EISCONN           : Socket is already connected
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketListen (int32_t socket, int32_t backlog) {
  int32_t ret;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if (backlog != 1) {
    // Only backlog 1 is supported
    return ARM_SOCKET_EINVAL;
  }
  if (socket_arr[socket].state == SOCKET_STATE_LISTENING) {
    return ARM_SOCKET_EINVAL;
  }
  if (socket_arr[socket].protocol != ARM_SOCKET_IPPROTO_TCP) {
    return ARM_SOCKET_ENOTSUP;
  }
  if (socket_arr[socket].bound == 0U) {
    if (socket_arr[socket].state == SOCKET_STATE_CONNECTED) {
      return ARM_SOCKET_EISCONN;
    } else {
      return ARM_SOCKET_EINVAL;
    }
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Execute functionality on the module through SPI commands
    if ((socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_TCP) && (socket_arr[socket].server == 0U)) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, socket_arr[socket].local_port, NULL, 0U, TRANSPORT_START, TRANSPORT_SERVER);
        if (ret == 0) {
          socket_arr[socket].server = 1U;
        }
        osMutexRelease(mutex_id_spi);
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }

    if (ret == 0) {
      // If socket listen succeeded
      socket_arr[socket].state = SOCKET_STATE_LISTENING;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketAccept (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Accept a new connection on a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where address of connecting socket shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where port of connecting socket shall be returned (NULL for none)
  \return        status information
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
  int32_t  ret;
  uint32_t flags, timeout;
  uint8_t  virtual_socket, non_blocking, trigger_async_polling;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (socket_arr[socket].protocol != ARM_SOCKET_IPPROTO_TCP) {
    return ARM_SOCKET_ENOTSUP;
  }
  if ((socket_arr[socket].state != SOCKET_STATE_LISTENING) && (socket_arr[socket].state != SOCKET_STATE_ACCEPTING)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  trigger_async_polling = 0U;
  non_blocking          = 0U;

  virtual_socket = (uint8_t)socket + WIFI_ISM43362_SOCKETS_NUM;
  if (socket_arr[virtual_socket].state != SOCKET_STATE_FREE) {
    // ISM43362 module limitation: only one socket can be processed at a time
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    non_blocking = socket_arr[socket].non_blocking;
    trigger_async_polling = 0U;
    if (socket_arr[socket].state != SOCKET_STATE_ACCEPTING) {
      trigger_async_polling = 1U;
      socket_arr[socket].state = SOCKET_STATE_ACCEPTING;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  if (ret == 0) {
    if (trigger_async_polling != 0U) {
      // If requested activate asynchronous polling
      osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);
    }
    if (non_blocking != 0U) {           // If non-blocking mode
      timeout = 1U;
    } else {                            // If blocking mode
      timeout = osWaitForever;
    }
    flags = osEventFlagsWait(event_flags_sockets_id[socket],    EVENT_SOCK_ACCEPTED | EVENT_SOCK_ACCEPTING_CLOSE, osFlagsWaitAny | osFlagsNoClear, timeout);
    osEventFlagsClear (event_flags_sockets_id[socket], flags & (EVENT_SOCK_ACCEPTED | EVENT_SOCK_ACCEPTING_CLOSE));
    if ((flags & 0x80000000UL) != 0U) { // If error
      if (flags == osFlagsErrorTimeout) {
        ret = ARM_SOCKET_EAGAIN;
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    } else {
      if ((flags & EVENT_SOCK_ACCEPTING_CLOSE) != 0U) { // If abort requested from SocketClose
        ret = ARM_SOCKET_ECONNABORTED;
        osEventFlagsSet(event_flags_sockets_id[socket], EVENT_SOCK_ACCEPTING_CLOSE_DONE);
      }
    }

    // If ret == 0, then accept was signaled from async thread
    if (ret == 0) {
      if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
        // If socket accept succeeded, copy all variables to new (virtual) socket
        memcpy((void *)&socket_arr[virtual_socket], (void *)&socket_arr[socket], sizeof(socket_t));
        socket_arr[socket].state          = SOCKET_STATE_ACCEPTED;
        socket_arr[virtual_socket].state  = SOCKET_STATE_CONNECTED;
        socket_arr[virtual_socket].client = 0U;
        socket_arr[virtual_socket].server = 0U;
        osMutexRelease(mutex_id_sockets);
        if (ip != NULL) {
          ip[0] = socket_arr[virtual_socket].remote_ip[0];
          ip[1] = socket_arr[virtual_socket].remote_ip[1];
          ip[2] = socket_arr[virtual_socket].remote_ip[2];
          ip[3] = socket_arr[virtual_socket].remote_ip[3];
          *ip_len = 4U;
        }
        if (port != NULL) {
          *port = socket_arr[virtual_socket].remote_port;
        }

        // Start polling for reception
        socket_arr[virtual_socket].poll_recv = 1U;
        osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);      // Trigger asynchronous thread read

        // Because on ISM when accept succeeds same socket is used for communication, 
        // to comply with BSD we return different socket id (virtual) which is used for 
        // further communication, this number is considered virtual socket id
        ret = virtual_socket;
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketConnect (int32_t socket, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Connect a socket to a remote host.
  \param[in]     socket   Socket identification number
  \param[in]     ip       Pointer to remote IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Remote port number
  \return        status information
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
  int32_t ret;
  uint8_t dissolve_udp;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len != 4U) || (port == 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_TCP) && (memcmp((const void *)ip, (const void *)"\0\0\0\0", 4) == 0)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  dissolve_udp = 0U;

  if ((ip_len == 4U) && (memcmp((const void *)ip, (const void *)"\0\0\0\0", 4) == 0) && (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP)) {
    // If request to connect to IP 0.0.0.0 for UDP meaning dissolve the connection
    dissolve_udp = 1U;
  }

  // Handling of non-blocking socket connect
  if (socket_arr[socket].non_blocking != 0U) {  // If non-blocking mode
    switch (socket_arr[socket].state) {
      case SOCKET_STATE_CONNECTED:
        if (dissolve_udp) {
          // Dissolve is handled same as for blocking socket (immediately)
          break;
        }
        return ARM_SOCKET_EISCONN;
      case SOCKET_STATE_CONNECTING:
        return ARM_SOCKET_EALREADY;
      case SOCKET_STATE_CREATED:
        if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
          // Store remote host IP and port
          memcpy((void *)socket_arr[socket].remote_ip, ip, 4);
          socket_arr[socket].remote_port = port;

          socket_arr[socket].state = SOCKET_STATE_CONNECTING;
          osMutexRelease(mutex_id_sockets);

          osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);      // Trigger asynchronous thread poll for non-blocking connection

          return ARM_SOCKET_EINPROGRESS;
        }
        return ARM_SOCKET_ERROR;
      case SOCKET_STATE_DISCONNECTED:
        if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
          socket_arr[socket].state = SOCKET_STATE_CREATED;
          osMutexRelease(mutex_id_sockets);
        }
        return ARM_SOCKET_ERROR;
      default:
        return ARM_SOCKET_EINVAL;
    }
  }

  // Handling of blocking socket connect
  switch (socket_arr[socket].state) {
    case SOCKET_STATE_CONNECTED:
      if (dissolve_udp) {
        break;
      }
      return ARM_SOCKET_EISCONN;
    case SOCKET_STATE_CREATED:
      break;
    case SOCKET_STATE_DISCONNECTED:
      break;
    default:
      return ARM_SOCKET_EINVAL;
  }

  // This module does not support setting of local port for client connection 
  // it only supports 0 value and selects the local port randomly

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    if (socket_arr[socket].state != SOCKET_STATE_CONNECTING) {      // If first call of connect
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        if ((socket_arr[socket].bound != 0U) && (socket_arr[socket].server == 1U)) {
          ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, NULL, 0U, TRANSPORT_STOP, TRANSPORT_SERVER);
          if (ret == 0) {
            socket_arr[socket].bound  = 0U;
            socket_arr[socket].server = 0U;
          }
        }

        if (ret == 0) { 
          if (dissolve_udp) {
            if (socket_arr[socket].client == 0U) {
              // Only auto-assigned local port is supported by module
              if ((socket_arr[socket].bound == 0U) || ((socket_arr[socket].bound != 0U) && (socket_arr[socket].local_port == 0U))) {
                ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, ip, port, TRANSPORT_STOP, TRANSPORT_CLIENT);
                if (ret == 0) {
                  socket_arr[socket].client = 0U;
                }
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
          } else {
            if (socket_arr[socket].client == 0U) {
              // Only auto-assigned local port is supported by module
              if ((socket_arr[socket].bound == 0U) || ((socket_arr[socket].bound != 0U) && (socket_arr[socket].local_port == 0U))) {
                ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, ip, port, TRANSPORT_START, TRANSPORT_CLIENT);
                if (ret == 0) {
                  socket_arr[socket].client = 1U;
                  socket_arr[socket].state  = SOCKET_STATE_CONNECTED;
                } else {
                  ret = ARM_SOCKET_ETIMEDOUT;
                }
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
          }
        }
        osMutexRelease(mutex_id_spi);
      } else {
        ret = ARM_SOCKET_ETIMEDOUT;
      }
    }
    // Only for first call of connect in non-blocking mode we return ARM_SOCKET_EINPROGRESS, 
    // otherwise as this module does not give any information on successful connect we 
    // consider that connect was successful and return 0

    if (ret == 0) {
      if (dissolve_udp) {
        // Stop polling for reception
        socket_arr[socket].poll_recv = 0U;

        // Clear remote host IP and port for UDP (datagram)
        memset((void *)socket_arr[socket].remote_ip, 0, 4);
        socket_arr[socket].remote_port = 0U;

        socket_arr[socket].bound = 0U;
        socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
      } else {
        // Start polling for reception
        socket_arr[socket].poll_recv = 1U;
        osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);      // Trigger asynchronous thread read

        // Store remote host IP and port for UDP (datagram)
        memcpy((void *)socket_arr[socket].remote_ip, (const void *)ip, 4);
        socket_arr[socket].remote_port = port;

        // If socket connect succeeded
        socket_arr[socket].bound = 1U;
        socket_arr[socket].state = SOCKET_STATE_CONNECTED;
      }
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len)
  \brief         Receive data or check if data is available on a connected socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes), set len = 0 to check if data is available
  \return        status information
                   - number of bytes received (>=0), if len != 0
                   - 0                            : Data is available (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len) {
  return (WiFi_SocketRecvFrom(socket, buf, len, NULL, NULL, NULL));
}

/**
  \fn            int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Receive data or check if data is available on a socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes), set len = 0 to check if data is available
  \param[out]    ip       Pointer to buffer where remote source address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where remote source port shall be returned (NULL for none)
  \return        status information
                   - number of bytes received (>=0), if len != 0
                   - 0                            : Data is available (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
  int32_t  ret;
  uint32_t flags;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
    return ARM_SOCKET_ENOTCONN;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  hw_socket = (uint8_t)socket;
  if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
    hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Handle if data was already received, and just return data immediately
    if (len != 0U) {
      ret = (int32_t)WiFi_ISM43362_BufferGet (hw_socket, buf, len);
    } else if (WiFi_ISM43362_BufferNotEmpty (hw_socket)) {
      ret = 1;                                                // Set ret to 1, if len requested was 0 and data is available
    }

    if (ret == 0) {
      if (socket_arr[socket].poll_recv == 0U) {               // If reception not started with bind
        socket_arr[socket].poll_recv = 1U;                    // start polling here
      }
      if (socket_arr[socket].non_blocking != 0U) {            // If non-blocking
        socket_arr[socket].recv_time_left = 1U;
      } else {                                                // If blocking
        if (socket_arr[socket].recv_timeout == 0U) {          // If timeout is 0, it means indefinite
          socket_arr[socket].recv_time_left = 0xFFFFFFFFUL;
        } else {                                              // Else set timeout to requested
          socket_arr[socket].recv_time_left = socket_arr[socket].recv_timeout;
        }
      }
      // Store Kernel Tick Count when reception is starting
      socket_arr[socket].start_tick_count = osKernelGetTickCount ();
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  if (ret == 0) {
    osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);      // Trigger asynchronous thread read

    // Wait for reception/timeout or disconnect signaled from asynchronous thread
    // or abort signaled form SocketClose
    flags = osEventFlagsWait(event_flags_sockets_id[hw_socket],    EVENT_SOCK_RECV | EVENT_SOCK_RECV_TIMEOUT | EVENT_SOCK_RECV_CLOSE | EVENT_SOCK_RECV_DISCON_REMOTE, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    osEventFlagsClear (event_flags_sockets_id[hw_socket], flags & (EVENT_SOCK_RECV | EVENT_SOCK_RECV_TIMEOUT | EVENT_SOCK_RECV_CLOSE | EVENT_SOCK_RECV_DISCON_REMOTE));

    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
      if ((flags & 0x80000000UL) != 0U) {                       // If error
        ret = ARM_SOCKET_ERROR;
      } else if ((flags & EVENT_SOCK_RECV_CLOSE) != 0U) {       // If reception was aborted locally by closing socket
        ret = ARM_SOCKET_ECONNABORTED;
        osEventFlagsSet(event_flags_sockets_id[hw_socket], EVENT_SOCK_RECV_CLOSE_DONE);
      } else if ((flags & EVENT_SOCK_RECV_DISCON_REMOTE)!=0U) { // If reception was aborted by remote host closing socket
        socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
        ret = ARM_SOCKET_ECONNRESET;
      } else if ((flags & EVENT_SOCK_RECV) != 0U) {             // If reception has finished and some data was received
        if (len != 0U) {
          ret = (int32_t)WiFi_ISM43362_BufferGet (hw_socket, buf, len);
        } else {
          ret = 1;                                              // Set ret to 1, if len requested was 0 and data is available
        }
      } else if ((flags & EVENT_SOCK_RECV_TIMEOUT) != 0U) {     // If reception has timed-out and nothing was received
        ret = ARM_SOCKET_EAGAIN;
      } else {                                                  // Should never happen
        ret = ARM_SOCKET_EAGAIN;
      }
      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  // Handling of special case read with len 0, if there was data available ret is 1 at this point
  if ((len == 0U) && (ret == 1)) {
    ret = 0;
  }

  if (ret > 0) {
    // If some data was received, load ip and port information (for datagram)
    if ((ip != NULL) && (ip_len != NULL) && (*ip_len >= 4U)) {
      ip[0] = socket_arr[socket].remote_ip[0];
      ip[1] = socket_arr[socket].remote_ip[1];
      ip[2] = socket_arr[socket].remote_ip[2];
      ip[3] = socket_arr[socket].remote_ip[3];
      *ip_len = 4U;
    }
    if (port != NULL) {
      *port = socket_arr[socket].remote_port;
    }
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len)
  \brief         Send data or check if data can be sent on a connected socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes), set len = 0 to check if data can be sent
  \return        status information
                   - number of bytes sent (>=0), if len != 0
                   - 0                            : Data can be sent (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len) {
  return (WiFi_SocketSendTo(socket, buf, len, NULL, 0U, 0U));
}

/**
  \fn            int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Send data or check if data can be sent on a socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes), set len = 0 to check if data can be sent
  \param[in]     ip       Pointer to remote destination IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Remote destination port number
  \return        status information
                   - number of bytes sent (>=0), if len != 0
                   - 0                            : Data can be sent (len = 0)
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ECONNRESET        : Connection reset by the peer
                   - ARM_SOCKET_ECONNABORTED      : Connection aborted locally
                   - ARM_SOCKET_EAGAIN            : Operation would block or timed out (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port) {
        int32_t  ret, spi_ret;
        int32_t  len_sent;
        uint32_t len_to_send, len_tot_sent, len_req;
  const uint8_t *ptr_buf;
        uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) && (len != 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((ip != NULL) && (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_TCP) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
    return ARM_SOCKET_ENOTCONN;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (len == 0U) {
    return 0;
  }

  len_tot_sent = 0U;

  if (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) {
    // For UDP send parameters ip, ip_len and port are used as they were provided on SocketConnect
    if (ip == NULL) {
      ip = socket_arr[socket].remote_ip;
    }
    if (ip_len != 4U) {
      ip_len = 4U;
    }
    if (port == 0U) {
      port = socket_arr[socket].remote_port;
    }
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    hw_socket = (uint8_t)socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }
    len_tot_sent = 0;

    // Execute functionality on the module through SPI commands
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Set communication socket number
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", hw_socket); spi_recv_len = sizeof(spi_recv_buf);
      spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
        ret = ARM_SOCKET_ERROR;
      }

      if (ret == 0) {
        if (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) {    // If UDP socket
          if (socket_arr[socket].client == 0U) {                        // UDP client not running on this socket
            // If call to SendTo for UDP and socket is not bound, do implicit binding
            ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, (const uint8_t *)ip, port, TRANSPORT_START, TRANSPORT_CLIENT);
            if (ret == 0) {
              socket_arr[socket].client = 1U;

              // Start polling for reception
              socket_arr[socket].poll_recv = 1U;
              osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);        // Trigger asynchronous thread read
            }
          }
        }
      }

      // Set transmit timeout (ms)
      if (ret == 0) {
        len_to_send  = len;
        len_tot_sent = 0U;

        do {
          len_req = len_to_send - len_tot_sent;
          if (len_req > MAX_DATA_SIZE) {
            len_req = MAX_DATA_SIZE;
          }
          ptr_buf  = (const uint8_t *)buf;
          ptr_buf += len_tot_sent;

          // Send data
          snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "S3=%04d\r", len_req); spi_recv_len = sizeof(spi_recv_buf);
          memcpy((void *)(&spi_send_buf[8]), (const void *)ptr_buf, len_req);
          spi_ret = SPI_SendReceive(spi_send_buf, len_req + 8U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if (spi_ret == ARM_DRIVER_OK) {
            if (resp_code == 0) {
              if (sscanf((const char *)&spi_recv_buf[2], "%d", &len_sent) == 1) {
                if ((len_sent > 0) || ((len_sent == 0) && (len_to_send == 0U))) {
                  len_tot_sent += (uint32_t)len_sent;
                } else if (len_sent == 0) {
                  ret = ARM_SOCKET_EAGAIN;
                } else {
                  ret = ARM_SOCKET_ERROR;
                }
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            } else if (resp_code == -1) {
              socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
              ret = ARM_SOCKET_ECONNRESET;
            } else {
              ret = ARM_SOCKET_ERROR;
            }
          } else if (spi_ret == ARM_DRIVER_ERROR_TIMEOUT) {
            ret = ARM_SOCKET_ETIMEDOUT;
          } else {
            ret = ARM_SOCKET_ERROR;
          }
        } while ((ret == 0) && (len_tot_sent < len_to_send));
      }
      osMutexRelease(mutex_id_spi);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  if (len_tot_sent > 0) {
    ret = (int32_t)len_tot_sent;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Retrieve local IP address and port of a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where local address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where local port shall be returned (NULL for none)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetSockName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
        int32_t  ret, spi_ret;
  const uint8_t *ptr_resp_buf;
        uint16_t u16_val;
        uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (socket_arr[socket].bound == 0U) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    hw_socket = (uint8_t)socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }

    if (ip != NULL) {
      ip[0] = socket_arr[socket].bound_ip[0];
      ip[1] = socket_arr[socket].bound_ip[1];
      ip[2] = socket_arr[socket].bound_ip[2];
      ip[3] = socket_arr[socket].bound_ip[3];
      if (ip_len != NULL) {
        *ip_len = 4U;
      }
    }

    // Execute functionality on the module through SPI commands
    if (port != NULL) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        // Set communication socket number
        snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", socket); spi_recv_len = sizeof(spi_recv_buf);
        spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
        if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
          ret = ARM_SOCKET_ERROR;
        }

        if (ret == 0) {
          // Show Transport Settings
          memcpy((void *)spi_send_buf, "P?\r\n", 4U); spi_recv_len = sizeof(spi_recv_buf);
          spi_ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
          if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
            // Skip protocol and client IP (2 ',') from response
            ptr_resp_buf = SkipCommas(spi_recv_buf, 2U);
            if (ptr_resp_buf != NULL) {
              if (sscanf((const char *)ptr_resp_buf, "%hu", &u16_val) == 1) {
                *port = u16_val;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            } else {
              ret = ARM_SOCKET_ERROR;
            }
          } else {
            ret = ARM_SOCKET_ERROR;
          }
        }
      }
      osMutexRelease(mutex_id_spi);
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Retrieve remote IP address and port of a socket.
  \param[in]     socket   Socket identification number
  \param[out]    ip       Pointer to buffer where remote address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where remote port shall be returned (NULL for none)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument (pointer to buffer or length)
                   - ARM_SOCKET_ENOTCONN          : Socket is not connected
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port) {
        int32_t  ret, spi_ret;
  const uint8_t *ptr_resp_buf;
        uint16_t u16_val;
        uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (socket_arr[socket].state != SOCKET_STATE_CONNECTED) {
    return ARM_SOCKET_ENOTCONN;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    hw_socket = (uint8_t)socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }

    // Execute functionality on the module through SPI commands
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", socket); spi_recv_len = sizeof(spi_recv_buf);
      spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
        ret = ARM_SOCKET_ERROR;
      }

      if (ret == 0) {
        // Show Transport Settings
        memcpy((void *)spi_send_buf, "P?\r\n", 4U); spi_recv_len = sizeof(spi_recv_buf);
        ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
        if ((ret == ARM_DRIVER_OK) && (resp_code == 0)) {
          if (ip != NULL) {
            // Skip protocol (1 ',') from response
            ptr_resp_buf = SkipCommas(spi_recv_buf, 3U);
            if (ptr_resp_buf != NULL) {
              if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4) {
                *ip_len = 4U;
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Skip protocol, client IP, local port and host IP (4 ',') from response
          if (port != NULL) {
            ptr_resp_buf = SkipCommas(spi_recv_buf, 4U);
            if ((ptr_resp_buf != NULL) && (port != NULL)) {
              if (sscanf((const char *)ptr_resp_buf, "%hu", &u16_val) == 1) {
                if (port != NULL) {
                  *port = u16_val;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
             ret = ARM_DRIVER_ERROR;
            }
          }
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
      osMutexRelease(mutex_id_spi);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len)
  \brief         Get socket option.
  \param[in]     socket   Socket identification number
  \param[in]     opt_id   Option identifier
  \param[out]    opt_val  Pointer to the buffer that will receive the option value
  \param[in,out] opt_len  Pointer to length of the option value
                   - length of buffer on input
                   - length of data on output
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetOpt (int32_t socket, int32_t opt_id, void *opt_val, uint32_t *opt_len) {
  int32_t ret;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_val == NULL) || (opt_len == NULL) || (opt_len == NULL) || (*opt_len < 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;
    switch (opt_id) {
      case ARM_SOCKET_SO_RCVTIMEO:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].recv_timeout);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_SNDTIMEO:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].send_timeout);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_KEEPALIVE:
        __UNALIGNED_UINT32_WRITE(opt_val, socket_arr[socket].keepalive);
        *opt_len = 4U;
        break;
      case ARM_SOCKET_SO_TYPE:
        switch (socket_arr[socket].protocol) {
          case ARM_SOCKET_IPPROTO_TCP:
            __UNALIGNED_UINT32_WRITE(opt_val, ARM_SOCKET_SOCK_STREAM);
            *opt_len = 4U;
            break;
          case ARM_SOCKET_IPPROTO_UDP:
            __UNALIGNED_UINT32_WRITE(opt_val, ARM_SOCKET_SOCK_DGRAM);
            *opt_len = 4U;
            break;
          default:
            ret = ARM_SOCKET_EINVAL;
            break;
        }
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len)
  \brief         Set socket option.
  \param[in]     socket   Socket identification number
  \param[in]     opt_id   Option identifier
  \param[in]     opt_val  Pointer to the option value
  \param[in]     opt_len  Length of the option value in bytes
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketSetOpt (int32_t socket, int32_t opt_id, const void *opt_val, uint32_t opt_len) {
  int32_t  ret, spi_ret;
  uint32_t u32;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_val == NULL) || (opt_len == NULL) || (opt_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    u32 = __UNALIGNED_UINT32_READ(opt_val);
    switch (opt_id) {
      case ARM_SOCKET_IO_FIONBIO:
        socket_arr[socket].non_blocking = (uint8_t)u32;
        break;
      case ARM_SOCKET_SO_RCVTIMEO:
        socket_arr[socket].recv_timeout = u32;
        break;
      case ARM_SOCKET_SO_SNDTIMEO:
        socket_arr[socket].send_timeout = u32;
        break;
      case ARM_SOCKET_SO_KEEPALIVE:
        if (driver_initialized == 0U) {
          ret = ARM_DRIVER_ERROR;
        }
        if (ret == 0) {
          if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
            hw_socket = (uint8_t)socket;
            if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
              hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
            }
            snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "P0=%d\r\n", socket); spi_recv_len = sizeof(spi_recv_buf);
            spi_ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
            if ((spi_ret != ARM_DRIVER_OK) || (resp_code != 0)) {
              ret = ARM_SOCKET_ERROR;
            }
            if (ret == 0) {
              if (u32 == 0U) {
                memcpy((void *)spi_send_buf, (void *)"PK=0,0\r\n", 9);
              } else {
                snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "PK=1,%d\r", u32);
              }
              spi_recv_len = sizeof(spi_recv_buf);
              spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
              if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
                socket_arr[socket].keepalive = u32;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
            osMutexRelease(mutex_id_spi);
          } else {
            ret = ARM_SOCKET_ERROR;
          }
        }
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketClose (int32_t socket)
  \brief         Close and release a socket.
  \param[in]     socket   Socket identification number
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ESOCK             : Invalid socket
                   - ARM_SOCKET_EAGAIN            : Operation would block (may be called again)
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketClose (int32_t socket) {
  int32_t  ret;
  uint32_t wait_flags;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return ARM_SOCKET_ESOCK;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }
  if (socket < WIFI_ISM43362_SOCKETS_NUM) {
    if (socket_arr[socket + WIFI_ISM43362_SOCKETS_NUM].state != SOCKET_STATE_FREE) {
      // If request to close socket but socket corresponding to it (virtual) is not free
      return ARM_SOCKET_ERROR;
    }
  }

  hw_socket = (uint8_t)socket;
  if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
    hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
  }
  wait_flags = 0U;

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    if (event_flags_sockets_id[hw_socket] != NULL) {
      if (socket_arr[hw_socket].state == SOCKET_STATE_ACCEPTING) {
        // If socket is waiting for flag in accept, send abort flag to terminate accept
        osEventFlagsSet  (event_flags_sockets_id[hw_socket], EVENT_SOCK_ACCEPTING_CLOSE);
        wait_flags |= EVENT_SOCK_ACCEPTING_CLOSE;
      }
      if (socket_arr[socket].poll_recv != 0U) {       // If reception is being polled
        socket_arr[socket].poll_recv = 0U;            // stop polling reception
      }
      if (socket_arr[socket].recv_time_left != 0U) {
        // If socket is being read (waiting in SocketRecv), send abort flag to terminate receive
        osEventFlagsSet  (event_flags_sockets_id[hw_socket], EVENT_SOCK_RECV_CLOSE);
        wait_flags |= EVENT_SOCK_RECV_CLOSE_DONE;
      }
    }

    // Execute functionality on the module through SPI commands
    if ((socket_arr[socket].client == 1U) || (socket_arr[hw_socket].server == 1U)) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        if (socket_arr[socket].client == 1U) {
          ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, NULL, 0U, TRANSPORT_STOP, TRANSPORT_CLIENT);
          if (ret == 0) {
            socket_arr[socket].client = 0U;
          }
        }
        if (hw_socket == socket) {      // If this is listening socket stop the server if running
          if (socket_arr[socket].server == 1U) {
            ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, NULL, 0U, TRANSPORT_STOP, TRANSPORT_SERVER);
            if (ret == 0) {
              socket_arr[socket].server = 0U;
            }
          }
        } else {                        // If this is accepted socket then restart server if running on listening socket
          if (socket_arr[hw_socket].server == 1U) {
            ret = SPI_StartStopTransportServerClient (hw_socket, socket_arr[socket].protocol, 0U, NULL, 0U, TRANSPORT_RESTART, TRANSPORT_SERVER);
            if (ret == 0) {
              if (socket_arr[hw_socket].state == SOCKET_STATE_ACCEPTED) {
                socket_arr[hw_socket].state = SOCKET_STATE_LISTENING;
              }
            }
          }
        }
        osMutexRelease(mutex_id_spi);
      } else {
        ret = ARM_SOCKET_EAGAIN;
      }
    }

    WiFi_ISM43362_BufferFlush (hw_socket);
    if (hw_socket == socket) {
      // Uninitialize buffer for socket
      WiFi_ISM43362_BufferUninitialize (hw_socket);
    }

    // If socket close succeeded, clear all variables
    memset ((void *)&socket_arr[socket], 0 , sizeof(socket_t));

    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  if (ret == 0) {
    if (wait_flags != 0U) {           // If events were sent to terminate accept or recv, wait for them to finish
      osEventFlagsWait (event_flags_sockets_id[hw_socket], wait_flags, osFlagsWaitAll, osWaitForever);
    } else {
      osEventFlagsClear(event_flags_sockets_id[hw_socket], 0xFFU);
    }
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len)
  \brief         Retrieve host IP address from host name.
  \param[in]     name     Host name
  \param[in]     af       Address family
  \param[out]    ip       Pointer to buffer where resolved IP address shall be returned
  \param[in,out] ip_len   Pointer to length of 'ip'
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_EINVAL            : Invalid argument
                   - ARM_SOCKET_ENOTSUP           : Operation not supported
                   - ARM_SOCKET_ETIMEDOUT         : Operation timed out
                   - ARM_SOCKET_EHOSTNOTFOUND     : Host not found
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t WiFi_SocketGetHostByName (const char *name, int32_t af, uint8_t *ip, uint32_t *ip_len) {
  int32_t  ret, spi_ret;

  if ((name == NULL) || (af != ARM_SOCKET_AF_INET) || (ip == NULL) || (ip_len == NULL) || (*ip_len < 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (strlen(name) > 64U) {
    // ISM43362 Limitation: Domain name is limited to 64 characters
    return ARM_SOCKET_ERROR;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = 0;

    // Send command for DNS lookup
    snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "D0=%s\r", name); spi_recv_len = sizeof(spi_recv_buf);
    spi_ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((spi_ret == ARM_DRIVER_OK) && (resp_code == 0)) {
      // Parse IP Address
      if (sscanf((const char *)&spi_recv_buf[2], "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4) {
        if (ip_len != NULL) {
          *ip_len = 4U;
        }
        ret = 0;
      } else {
        ret = ARM_SOCKET_ERROR;
      }
      osDelay(100U);
    } else {
      // Check if "Host not found " string is present in response
      if (strstr((const char *)&spi_recv_buf[2], "Host not found ") != NULL) {
        ret = ARM_SOCKET_EHOSTNOTFOUND;
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_SOCKET_ETIMEDOUT;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len)
  \brief         Probe remote host with Ping command.
  \param[in]     ip       Pointer to remote host IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (NULL ip pointer or ip_len different than 4 or 16)
*/
static int32_t WiFi_Ping (const uint8_t *ip, uint32_t ip_len) {
  int32_t  ret;

  if (ip == NULL){
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ip_len == 16U) {
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  if (ip_len != 4U) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Set ping target address
    memcpy((void *)spi_send_buf, "T2=0\r\n", 6); spi_recv_len = sizeof(spi_recv_buf);
    ret = SPI_SendReceive(spi_send_buf, 6U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
    if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
      ret = ARM_DRIVER_ERROR;
    }

    if (ret == ARM_DRIVER_OK) {
      memcpy((void *)spi_send_buf, "T3=100\r\n", 8); spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, 8U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
        ret = ARM_DRIVER_ERROR;
      }
    }

    if (ret == ARM_DRIVER_OK) {
      snprintf((char *)spi_send_buf, sizeof(spi_send_buf), "T1=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, strlen((const char *)spi_send_buf), spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if ((ret == ARM_DRIVER_OK) && (resp_code != 0)) {
        ret = ARM_DRIVER_ERROR;
      }
    }

    if (ret == ARM_DRIVER_OK) {
      // Ping IP target address
      memcpy((void *)spi_send_buf, "T0\r\n", 4); spi_recv_len = sizeof(spi_recv_buf);
      ret = SPI_SendReceive(spi_send_buf, 4U, spi_recv_buf, &spi_recv_len, &resp_code, WIFI_ISM43362_CMD_TIMEOUT);
      if (resp_code != 0) {
        ret = ARM_DRIVER_ERROR;
      }
      if (ret == ARM_DRIVER_OK) {
        if (strstr((const char *)&spi_recv_buf[2], "Timeout") != NULL) {
          // If no response from the remote host
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    // If SPI interface is not accessible (locked by another thread)
    ret = ARM_DRIVER_ERROR;
  }

  return ret;
}


// Structure exported by driver Driver_WiFin (default: Driver_WiFi0)

extern
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_ISM43362_DRV_NUM);
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_ISM43362_DRV_NUM) = { 
  WiFi_GetVersion,
  WiFi_GetCapabilities,
  WiFi_Initialize,
  WiFi_Uninitialize,
  WiFi_PowerControl,
  WiFi_GetModuleInfo,
  WiFi_SetOption,
  WiFi_GetOption,
  WiFi_Scan,
  WiFi_Activate,
  WiFi_Deactivate,
  WiFi_IsConnected,
  WiFi_GetNetInfo,
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
};
