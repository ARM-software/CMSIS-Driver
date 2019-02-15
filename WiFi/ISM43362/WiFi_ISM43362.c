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
 * $Date:        29. January 2019
 * $Revision:    V1.0 (beta)
 *
 * Driver:       Driver_WiFin (n = WIFI_ISM43362_DRIVER_INDEX value)
 * Project:      WiFi Driver for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   WIFI_ISM43362_DRIVER_INDEX: defines index of driver structure variable
 *     - default value:    0
 *   WIFI_ISM43362_SPI_DRV_NUM: defines index of SPI driver used
 *     - default value:    3
 *   WIFI_ISM43362_SOCKETS_NUM: defines maximum number of sockets
 *     - default value:    4
 *     - maximum value:    4
 *   WIFI_ISM43362_SOCKET_DEF_TIMEOUT: defines initial timeout setting for 
 *                         socket send/receive
 *     - default value:    10000
 *   WIFI_ISM43362_SPI_TIMEOUT: defines maximum wait on SPI to become free
 *     - default value:    1000
 *   WIFI_ISM43362_CMD_TIMEOUT: defines maximum wait on SPI command to finish
 *     - default value:    30000
 *   WIFI_ISM43362_ASYNC_INTERVAL: defines polling interval for async messages
 *     - default value:    1000
 *   WIFI0_ISM43362_ASYNC_PRIORITY: asynchronous thread priority
 *     - default value:    osPriorityAboveNormal
 *   WIFI_ISM43362_SPI_RECEIVE_SIZE: defines maximum size of single 
 *                         SPI receive transfer
 *     - default value:    32
 *
 * Notes:
 * This driver uses SPI for communicating with ISM module, however there are 
 * 3 pins that are not handled by SPI peripheral, and they are:
 *  - SSN     = slave select (active low) (output)
 *  - DATARDY = data ready (active low)   (input)
 *  - RSTN    = reset (active low)        (output)
 *
 * To drive SSN and RSTN pins, and get state of DATARDY pin you need to 
 * implement following functions (available as template):
 *   - void WiFin_ISM43362_Pin_RSTN    (bool rstn)
 *   - void WiFin_ISM43362_Pin_SSN     (bool ssn)
 *   - bool WiFin_ISM43362_Pin_DATARDY (void)
 *
 * If DATARDY pin has interrupt capability for better performance of the driver 
 * upon interrupt of DATARDY pin state change to active state the IRQ routine 
 * should call the following function:
 *   - void WiFin_ISM43362_Pin_DATARDY_IRQ (void)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0 (beta)
 *    Initial beta version
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cmsis_os2.h"
#include "cmsis_compiler.h"
#ifdef   _RTE_
#include "RTE_Components.h"
#endif

#include "Driver_Common.h"
#include "Driver_WiFi.h"

#include "Driver_SPI.h"


// Externally overridable configuration definitions

#include "WiFi_ISM43362_Config.h"

// WiFi Driver Number (default: 0)
#ifndef WIFI_ISM43362_DRIVER_INDEX
#define WIFI_ISM43362_DRIVER_INDEX          0
#endif

// SPI Driver Number (default: 3)
#ifndef WIFI_ISM43362_SPI_DRV_NUM
#define WIFI_ISM43362_SPI_DRV_NUM           3
#endif

// Number of sockets supported by module (default: 4, maximum: 4)
#ifndef WIFI_ISM43362_SOCKETS_NUM
#define WIFI_ISM43362_SOCKETS_NUM          (4)
#else
#if    (WIFI_ISM43362_SOCKETS_NUM > 4)
#undef  WIFI_ISM43362_SOCKETS_NUM 
#define WIFI_ISM43362_SOCKETS_NUM          (4)
#warning ISM43362 module supports maximum 4 sockets, so define WIFI_ISM43362_SOCKETS_NUM is redefined to 4!
#endif
#endif

// Socket send/receive default timeout (default: 10000 ms)
#ifndef WIFI_ISM43362_SOCKET_DEF_TIMEOUT
#define WIFI_ISM43362_SOCKET_DEF_TIMEOUT   (10000)
#endif

// SPI mutex acquire timeout (default: 1000 ms)
#ifndef WIFI_ISM43362_SPI_TIMEOUT
#define WIFI_ISM43362_SPI_TIMEOUT          (1000)
#endif

// SPI command timeout (default: 30000 ms)
#ifndef WIFI_ISM43362_CMD_TIMEOUT
#define WIFI_ISM43362_CMD_TIMEOUT          (30000)
#endif

// Asynchronous thread polling time interval (default: 1000 ms)
#ifndef WIFI_ISM43362_ASYNC_INTERVAL
#define WIFI_ISM43362_ASYNC_INTERVAL       (1000)
#endif

// Asynchronous thread priority (default: osPriorityAboveNormal)
#ifndef WIFI0_ISM43362_ASYNC_PRIORITY
#define WIFI0_ISM43362_ASYNC_PRIORITY      (osPriorityAboveNormal)
#endif

// SPI Receive transfer size (for improved performance)
#ifndef WIFI_ISM43362_SPI_RECEIVE_SIZE
#define WIFI_ISM43362_SPI_RECEIVE_SIZE     (32)
#endif

// Hardware dependent functions --------

// Externally provided hardware dependent handling callback functions

extern void WiFi_ISM43362_Pin_RSTN    (bool rstn);
extern void WiFi_ISM43362_Pin_SSN     (bool ssn);
extern bool WiFi_ISM43362_Pin_DATARDY (void);

// Exported hardware dependent function called by user code

extern void WiFi_ISM43362_Pin_DATARDY_IRQ (void);

/**
  \fn          void WiFin_ISM43362_Pin_DATARDY_IRQ (void)
  \brief       Interrupt on DATARDY line state changed to active state.
  \detail      This callback function should be called by external user code 
               on interrupt when DATARDY line changes to active state.
               It's usage improves driver efficiency as it then uses 
               event for signaling DATARDY status change instead of polling.
  \return      none
*/
void WiFi_ISM43362_Pin_DATARDY_IRQ (void);


// WiFi Driver *****************************************************************

#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)        // Driver version

// Driver Version
static const ARM_DRIVER_VERSION driver_version = { ARM_WIFI_API_VERSION, ARM_WIFI_DRV_VERSION };

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = { 
  1U,                                   // Station: WiFi Protected Setup (WPS) supported
  1U,                                   // Access Point supported
  1U,                                   // Access Point: event generated on Station connect
  0U,                                   // Access Point: event not generated on Station disconnect
  1U,                                   // IP (UDP/TCP) (Socket interface) supported
  0U,                                   // IPv6 (Socket interface) not supported
  1U,                                   // Ping (ICMP) supported
  0U,                                   // Bypass or pass-through mode (Ethernet interface) not supported
  0U,                                   // Event not generated on Ethernet frame reception in bypass mode
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
  uint16_t local_port;                  // Local port number
  uint16_t reserved1;                   // Reserved
                                        // Module specific socket variables
  void    *data_to_recv;                // Pointer to where data should be received
  uint32_t len_to_recv;                 // Number of bytes to receive
  uint32_t len_recv;                    // Number of bytes received
  uint32_t recv_time_left;              // Receive Time left until Timeout
  uint8_t  client;                      // Socket client running
  uint8_t  server;                      // Socket server running
  uint16_t reserved2;                   // Reserved
} socket_t;

// Socket states
#define SOCKET_STATE_FREE              (0U)
#define SOCKET_STATE_CREATED           (1U)
#define SOCKET_STATE_BOUND             (2U)
#define SOCKET_STATE_LISTENING         (3U)
#define SOCKET_STATE_ACCEPTING         (4U)
#define SOCKET_STATE_ACCEPTED          (5U)
#define SOCKET_STATE_CONNECTING        (6U)
#define SOCKET_STATE_CONNECTED         (7U)
#define SOCKET_STATE_DISCONNECTED      (8U)

// Event flags
#define EVENT_SPI_XFER_DONE            (1U)
#define EVENT_SPI_READY                (1U)
#define EVENT_ASYNC_MSG_POLL           (1U)

// Local macros
#define SPI_Driver_(n)                  Driver_SPI##n
#define SPI_Driver(n)                   SPI_Driver_(n)
extern ARM_DRIVER_SPI                   SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM);
#define ptrSPI                        (&SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM))

// Mutex responsible for protecting SPI media access
const osMutexAttr_t mutex_spi_attr = {
  "Mutex_SPI",                                  // Mutex name
  osMutexRecursive | osMutexPrioInherit,        // attr_bits
  NULL,                                         // Memory for control block   
  0U                                            // Size for control block
};

// Mutex responsible for protecting socket local variables access
const osMutexAttr_t mutex_socket_attr = {
  "Mutex_Socket",                               // Mutex name
  osMutexRecursive | osMutexPrioInherit,        // attr_bits
  NULL,                                         // Memory for control block   
  0U                                            // Size for control block
};

// Thread for polling and processing asynchronous messages
#ifdef RTE_CMSIS_RTOS2_RTX5
#define ASYNC_MSG_STK_SZ (512U)
uint64_t thread_async_poll_stk[ASYNC_MSG_STK_SZ / 8];
#endif
const osThreadAttr_t thread_async_poll_attr = {
  .name       =       "Thread_Async_Poll",      // Thread name
#ifdef RTE_CMSIS_RTOS2_RTX5
  .stack_mem  =        thread_async_poll_stk,   // Thread stack
  .stack_size = sizeof(thread_async_poll_stk),  // Thread stack size
#endif
  .priority   =  WIFI0_ISM43362_ASYNC_PRIORITY
};


// Local variables and structures
static bool                             driver_initialized = false;
static ARM_WIFI_SignalEvent_t           signal_event_fn;

static bool                             spi_datardy_irq;
static osEventFlagsId_t                 event_id_spi_ready;
static osEventFlagsId_t                 event_id_spi_xfer_done;
static osEventFlagsId_t                 event_id_async_poll;
static osEventFlagsId_t                 event_id_accept;
static osEventFlagsId_t                 event_id_socket;
static osMutexId_t                      mutex_id_spi;
static osMutexId_t                      mutex_id_sockets;
static osThreadId_t                     thread_id_async_poll;

static bool                             sta_connected;
static bool                             sta_dhcp_enabled;
static bool                             ap_running;
static uint8_t                          ap_num_connected;
static uint32_t                         ap_dhcp_lease_time;

static uint8_t                          sta_local_ip  [4] __ALIGNED(4);
static uint8_t                          ap_local_ip   [4] __ALIGNED(4);
static ARM_WIFI_MAC_IP4_t               mac_ip4       [8] __ALIGNED(4);

static char                             cmd_buf [128  +1] __ALIGNED(4);
static uint8_t                          resp_buf[1210 +1] __ALIGNED(4);

static socket_t                         socket_arr[2 * WIFI_ISM43362_SOCKETS_NUM];

// Function prototypes
static int32_t WiFi_SocketRecvFrom (int32_t socket,       void *buf, uint32_t len,       uint8_t *ip, uint32_t *ip_len, uint16_t *port);
static int32_t WiFi_SocketSendTo   (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t  ip_len, uint16_t  port);
static int32_t WiFi_SocketClose    (int32_t socket);


// Helper Functions

/**
  \fn          void WiFi_ISM43362_Pin_DATARDY_IRQ (void)
  \brief       IRQ callback on DATARDY line state change from inactive to active.
  \detail      This function should be called from user IRQ routine when DATARDY pin state changes from inactive to active.
  \return      none
*/
void WiFi_ISM43362_Pin_DATARDY_IRQ (void) {
  osEventFlagsSet(event_id_spi_ready, EVENT_SPI_READY);
}

/**
  \fn          void SPI_SignalEvent (uint32_t event)
  \brief       SPI Signal Event callback, called by SPI driver when an SPI event occurs.
  \param[in]   event    Event signaled by SPI driver
  \return      none
*/
static void SPI_SignalEvent (uint32_t event) {
  if (event & ARM_SPI_EVENT_TRANSFER_COMPLETE) {
    osEventFlagsSet(event_id_spi_xfer_done, EVENT_SPI_XFER_DONE);
  }
}

/**
  \fn          void Wait_us (uint32_t us)
  \brief       Wait a specified number of microseconds (executing NOPs).
  \param[in]   us       Microseconds
  \return      none
*/
static void Wait_us (uint32_t us) {
  uint32_t start_cnt, us_cnt;

  us_cnt    = (osKernelGetSysTimerFreq() / 1000000U) * us;
  start_cnt =  osKernelGetSysTimerCount();

  while ((osKernelGetSysTimerCount() - start_cnt) < us_cnt) {
  }
  __NOP();
}

/**
  \fn          void SPI_WaitReady (uint32_t timeout)
  \brief       Wait for SPI ready (DATARDY pin active).
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      SPI ready state
                 - \b false SPI is not ready
                 - \b true  SPI is ready
*/
static bool SPI_WaitReady (uint32_t timeout) {
  bool ret;

  if (spi_datardy_irq) {                // If events are generated by WiFi_ISM43362_Pin_DATARDY_IRQ function
    ret = osEventFlagsWait(event_id_spi_ready, EVENT_SPI_READY, osFlagsWaitAny, timeout) == EVENT_SPI_READY;
  } else {
    do {
      ret = WiFi_ISM43362_Pin_DATARDY();
      if (!ret) {                       // If DATARDY is ready
        osDelay(1U);
        if (timeout > 0U) {
          timeout--;
        }
      }
    } while ((!ret) && (timeout != 0U));
  }

  return ret;
}

/**
  \fn          void SPI_WaitTransferDone (uint32_t timeout)
  \brief       Wait for SPI transfer to finish.
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      SPI transfer finished state
                 - \b false SPI transfer finished successfully
                 - \b true  SPI transfer did not finish successfully
*/
static bool SPI_WaitTransferDone (uint32_t timeout) {
  return (osEventFlagsWait(event_id_spi_xfer_done, EVENT_SPI_XFER_DONE, osFlagsWaitAny, timeout) == EVENT_SPI_XFER_DONE);
}

/**
  \fn          int32_t SPI_AT_SendCommandAndData (const char *cmd, const uint8_t *data, uint32_t data_len, uint32_t timeout)
  \brief       Send AT command and data.
  \param[in]   cmd      Pointer to command null-terminated string
  \param[in]   data     Pointer to data to be sent after command
  \param[in]   data_len Number of bytes of data to be sent
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      execution status
                 - ARM_DRIVER_OK                : Command and data sent successfully
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_AT_SendCommandAndData (const char *cmd, const uint8_t *data, uint32_t data_len, uint32_t timeout) {
        int32_t  ret;
  const uint8_t *ptr_u8_data;
        uint32_t cmd_len, data_len_to_send;
        uint8_t  tmp[2];

  ret = ARM_DRIVER_OK;

  if ((cmd == NULL) && (data == NULL)) {        // If cmd and data are null pointers
    ret = ARM_DRIVER_ERROR;
  }
  if (ret == ARM_DRIVER_OK) {
    cmd_len = strlen(cmd);
    if ((cmd_len == 0U) && (data_len == 0U)) {  // If length of command and data is 0
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    if (!SPI_WaitReady(timeout)) {              // If SPI is not ready
      ret = ARM_DRIVER_ERROR_TIMEOUT;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ptr_u8_data      = data;
    data_len_to_send = data_len;

    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us

    if ((cmd != NULL) && ((cmd_len / 2) > 0)) {
      // Send even number of bytes of cmd
      if (ptrSPI->Send(cmd, cmd_len / 2) == ARM_DRIVER_OK) {
        if (!SPI_WaitTransferDone(timeout)) {   // If SPI transfer has failed
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && (cmd != NULL) && ((cmd_len & 1U) != 0U)) {
      // If cmd contains odd number of bytes, append 1 byte of data, if 
      // there is data to be sent, otherwise append 1 byte of value 0x0A
      tmp[0] = (uint8_t)cmd[cmd_len - 1];
      if ((data != NULL) && (data_len_to_send != 0)) {
        tmp[1] = data[0];
        data_len_to_send--;
        ptr_u8_data++;
      } else {
        tmp[1] = '\n';
      }
      if (ptrSPI->Send(tmp, 1) == ARM_DRIVER_OK) {
        if (!SPI_WaitTransferDone(timeout)) {   // If SPI transfer has failed
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && (ptr_u8_data != NULL) && ((data_len_to_send / 2) > 0U)) {
      // Send even number of bytes of remaining data
      if (ptrSPI->Send(ptr_u8_data, data_len_to_send / 2) == ARM_DRIVER_OK) {
        if (!SPI_WaitTransferDone(timeout)) {   // If SPI transfer has failed
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && (ptr_u8_data != NULL) && ((data_len_to_send & 1U) != 0U)) {
      // If remaining data contains odd number of bytes, append 1 byte of value 0x0A
      tmp[0] = data[data_len - 1U];
      tmp[1] = 0x0AU;
      if (ptrSPI->Send(tmp, 1U) == ARM_DRIVER_OK) {
        if (!SPI_WaitTransferDone(timeout)) {   // If SPI transfer has failed
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if (ret != ARM_DRIVER_OK) {                 // If SPI transfer has failed
      ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
    }

    WiFi_ISM43362_Pin_SSN(false);               // Deactivate slave select line
  }

  return ret;
}

/**
  \fn          int32_t SPI_AT_ReceiveData (uint8_t *data, uint32_t data_len, uint32_t timeout)
  \brief       Receive data.
  \param[out]  data     Pointer to memory where data will be received
  \param[in]   data_len Maximum number of bytes of data that memory is prepared to receive
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      number of bytes received or error code
                 - > 0                          : Number of bytes received
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_AT_ReceiveData (uint8_t *data, uint32_t data_len, uint32_t timeout) {
  int32_t  ret;
  uint32_t len_to_rece, total_len_rece, len;
  int32_t  i;

  ret = ARM_DRIVER_OK;

  if ((data == NULL) || (data_len == 0U)) {     // If data is null pointer, or data len requested is 0
    ret = ARM_DRIVER_ERROR;
  }

  WiFi_ISM43362_Pin_SSN(false);                 // Deactivate slave select line
  Wait_us(3U);                                  // Wait 3 us

  if (ret == ARM_DRIVER_OK) {
    if (!SPI_WaitReady(timeout)) {              // If SPI is not ready
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    len_to_rece    = data_len;
    total_len_rece = 0U;

    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us

    do {
      len = len_to_rece;
      if (len > WIFI_ISM43362_SPI_RECEIVE_SIZE) {
        len = WIFI_ISM43362_SPI_RECEIVE_SIZE;
      }
      if (ptrSPI->Receive(data + total_len_rece, (len + 1U) / 2) == ARM_DRIVER_OK) {
        if (SPI_WaitTransferDone(timeout)) {
          total_len_rece += len;
          len_to_rece    -= len;
        } else {                                // SPI transfer did not finish in expected time
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      } else {                                  // SPI transfer failed
        ret = ARM_DRIVER_ERROR;
      }

      if (ret != ARM_DRIVER_OK) {               // If SPI transfer has failed
        ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
      }
    } while ((ret == ARM_DRIVER_OK) && (len_to_rece > 0U) && (WiFi_ISM43362_Pin_DATARDY()));
    if ((ret == ARM_DRIVER_OK) && (!WiFi_ISM43362_Pin_DATARDY())) {
      // If reception ended with DATARDY line inactive
      // correct number of received bytes to reduce the trailing 0x15 bytes
      for (i = total_len_rece; i > 0; i--) {
        if (data[i - 1U] != 0x15U) {  // If non 0x15 value from end was found
          break;
        }
      }
      total_len_rece = i;
    }

    WiFi_ISM43362_Pin_SSN(false);               // Deactivate slave select line
  }

  if (ret == ARM_DRIVER_OK) {
    ret = total_len_rece;
  }

  return ret;
}

/**
  \fn          int32_t SPI_AT_SendCommandReceiveResponse (const char *cmd, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief       Send AT command, receive response and check that response is OK.
  \param[in]   cmd      Pointer to command null-terminated string
  \param[out]  resp     Buffer where response will be received
  \param[in,
         out]  resp_len Pointer to a number
                 - input: number of bytes that can be received in response
                 - output: number of bytes actually received in response
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      execution status
                 - ARM_DRIVER_OK                : Command sent successfully
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_AT_SendCommandReceiveResponse (const char *cmd, uint8_t *resp, uint32_t *resp_len, uint32_t timeout) {
  int32_t ret, rece_num;

  ret = ARM_DRIVER_OK;
  if ((cmd == NULL) || (resp == NULL) || (resp_len == NULL)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = SPI_AT_SendCommandAndData(cmd, NULL, 0U, timeout);
    if (ret == ARM_DRIVER_OK) {
      rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
      if (rece_num < 0) {
        ret = rece_num;
      } else if ((rece_num < 8) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
        ret = ARM_DRIVER_ERROR;
      } else {
        resp[rece_num] = 0;     // Terminate response string
      }
      *resp_len = rece_num;
    }
  }

  return ret;
}

/**
  \fn          int32_t SPI_AT_SendCommandAndDataReceiveResponse (const char *cmd, const uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief       Send AT command and data, receive response and check that response is OK.
  \param[in]   cmd      Pointer to command null-terminated string
  \param[in]   data     Pointer to data to be sent after command
  \param[in]   data_len Number of bytes of data to be sent
  \param[out]  resp     Buffer where response will be received
  \param[in,
         out]  resp_len Pointer to a number
                 - input: number of bytes that can be received in response
                 - output: number of bytes actually received in response
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      number of data bytes sent or error code
                 - > 0                          : Number of data bytes sent
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_AT_SendCommandAndDataReceiveResponse (const char *cmd, const uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout) {
  int32_t ret, rece_num;

  ret = ARM_DRIVER_OK;
  if ((cmd == NULL) || (data == NULL) || (data_len == 0U) || (resp == NULL) || (resp_len == NULL)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = SPI_AT_SendCommandAndData(cmd, data, data_len, timeout);
    if (ret == ARM_DRIVER_OK) {
      rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
      if (rece_num < 0) {
        ret = rece_num;
      } else if ((rece_num < 8) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
        ret = ARM_DRIVER_ERROR;
      } else {
        resp[rece_num] = 0;     // Terminate response string
        // Parse how many data bytes were sent from response
        if (sscanf((const char *)(resp + 2), "%d", &ret) != 1) {
          ret = ARM_DRIVER_ERROR;
        }
      }
    }
  }

  return ret;
}

/**
  \fn          int32_t SPI_AT_SendCommandReceiveDataAndResponse (const char *cmd, uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief       Send AT command, receive data and response and check that response is OK.
  \param[in]   cmd      Pointer to command null-terminated string
  \param[in]   data     Pointer to memory where data will be received
  \param[in]   data_len Maximum number of bytes of data that memory is prepared to receive
  \param[out]  resp     Buffer where response will be received
  \param[in,
         out]  resp_len Pointer to a number
                 - input: number of bytes that can be received in response
                 - output: number of bytes actually received in response
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      number of bytes received or error code
                 - > 0                          : Number of bytes received
                 - ARM_DRIVER_ERROR             : Operation failed
                 - ARM_DRIVER_ERROR_TIMEOUT     : Operation timed out
*/
static int32_t SPI_AT_SendCommandReceiveDataAndResponse (const char *cmd, uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout) {
  int32_t ret, rece_num;

  ret = ARM_DRIVER_OK;
  if ((cmd == NULL) || (data == NULL) || (data_len == 0U) || (resp == NULL) || (resp_len == NULL)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = SPI_AT_SendCommandAndData(cmd, NULL, 0U, timeout);
    if (ret == ARM_DRIVER_OK) {
      rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
      if (rece_num < 0) {
        ret = rece_num;
      } else if ((rece_num < 10) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
        ret = ARM_DRIVER_ERROR;
      } else {
        resp[rece_num] = 0;     // Terminate response string
        // Data was read successfully
        if (data_len >= (rece_num - 10)) {
          memcpy(data, resp + 2, rece_num - 10);
          ret = rece_num - 10;
        } else {
          memcpy(data, resp + 2, data_len);
          ret = data_len;
        }
      }
    }
  }

  return ret;
}

/**
  \fn          void WiFi_AsyncMsgProcessThread (void)
  \brief       Thread that reads and processes asynchronous messages from WiFi module.
  \return      none
*/
__NO_RETURN static void WiFi_AsyncMsgProcessThread (void *arg) {
  uint8_t *ptr_u8_resp_buf;
  uint32_t event_accept, event_recv;
  uint32_t resp_len, len_req;
  int32_t  len_read;
  int      int_arr[6];
  uint8_t  mac_[6];
  bool     poll_async, poll_recv, event_signal;
  int8_t   i, hw_socket;

  for (;;) {
    if (osEventFlagsWait(event_id_async_poll, EVENT_ASYNC_MSG_POLL, osFlagsWaitAny, osWaitForever) == EVENT_ASYNC_MSG_POLL) {
      do {
        // Check if thread should poll for asynchronous messages or receive in long blocking
        poll_async = false;
        poll_recv  = false;
        for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
          if  (socket_arr[i].state == SOCKET_STATE_ACCEPTING) {
            poll_async = true;
          }
          if ((socket_arr[i].data_to_recv != NULL) && (socket_arr[i].recv_time_left != 0U)){
            poll_recv = true;
          }
        }
        if (ap_running) {
          poll_async = true;
        }

        if ((poll_async) || (poll_recv)) {
          event_signal = false;
          event_accept = 0U;
          event_recv   = 0U;

          // Lock access to SPI interface (acquire mutex)
          if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
            if (poll_async) {
              // Send command to read asynchronous message
              memcpy((void *)cmd_buf, (void *)"MR\r", 4); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                // If message is asynchronous Accept
                ptr_u8_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "Accepted ");
                if (ptr_u8_resp_buf != NULL) {
                  // If message contains "Accepted " string, parse it and extract ip and port
                  ptr_u8_resp_buf += 9U;
                  // Parse IP Address and port
                  if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d:%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[4]) == 5) {
                    // IP and port read from response correctly
                    // Find which socket is listening on accepted port
                    for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
                      if (socket_arr[i].local_port == (uint16_t)int_arr[4]) {
                        break;
                      }
                    }
                    if (i != WIFI_ISM43362_SOCKETS_NUM) {
                      // Socket 'i' has accepted connection
                      event_accept |= 1U << i;
                    }
                  }

                  // If message is asynchronous Assign
                  ptr_u8_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "Assigned ");
                  if (ptr_u8_resp_buf != NULL) {
                    // If message contains "Assigned " string, parse it and extract MAC
                    ptr_u8_resp_buf += 9U;
                    // Parse MAC Address
                    if (sscanf((const char *)ptr_u8_resp_buf, "%x:%x:%x:%x:%x:%x", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[4], &int_arr[5]) == 6) {
                      // MAC read from response correctly
                      mac_[0] = (uint8_t)int_arr[0];
                      mac_[1] = (uint8_t)int_arr[1];
                      mac_[2] = (uint8_t)int_arr[2];
                      mac_[3] = (uint8_t)int_arr[3];
                      mac_[4] = (uint8_t)int_arr[4];
                      mac_[5] = (uint8_t)int_arr[5];
                      // Check if MAC already exists in mac_ip4 array, if it does ignore it, otherwise add it to array
                      for (i = 0; i < 8; i++) {
                        if (memcmp((void *)mac_ip4[i].mac, (void *)mac_, 6) == 0) {
                          break;
                        }
                      }
                      if ((i < 8) && (ap_num_connected < 8)) {
                        memcpy((void *)mac_ip4[ap_num_connected].mac, (void *)mac_, 6);
                        ap_num_connected++;
                        event_signal = true;
                      }
                    }
                  }
                }
              }
            }

            if (poll_recv) {
              if (osMutexAcquire(mutex_id_sockets, WIFI_ISM43362_ASYNC_INTERVAL) == osOK) { // Lock socket variables
                // Loop through all sockets
                for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
                  hw_socket = i;
                  if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
                    hw_socket -= WIFI_ISM43362_SOCKETS_NUM;     // Actually used socket of module
                  }
                  if ((socket_arr[i].data_to_recv != NULL) && (socket_arr[i].len_to_recv != 0U) && (socket_arr[i].recv_time_left != 0U)) {
                    do {
                      // Send command to read data from remote client if socket is using receive in long blocking
                      sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
                      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                        len_req = socket_arr[i].len_to_recv - socket_arr[i].len_recv;
                        if (len_req > 1200U) {
                          len_req = 1200U;
                        }
                        // Set read data packet size
                        sprintf(cmd_buf, "R1=%d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
                        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                          // Receive data
                          memcpy((void *)cmd_buf, (void *)"R0\r", 4); resp_len = sizeof(resp_buf) - 1U;
                          len_read = SPI_AT_SendCommandReceiveDataAndResponse(cmd_buf, (uint8_t *)socket_arr[i].data_to_recv + socket_arr[i].len_recv, len_req, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
                          if (len_read > 0) {         // Some data was read
                            socket_arr[i].len_recv += len_read;
                            if (socket_arr[i].len_to_recv == socket_arr[i].len_recv) {
                              event_recv |= (1U << i);
                            }
                          } else if (len_read == 0) { // No data was read
                            if (socket_arr[i].recv_time_left >= WIFI_ISM43362_ASYNC_INTERVAL) {
                              socket_arr[i].recv_time_left -= WIFI_ISM43362_ASYNC_INTERVAL;
                            } else {
                              socket_arr[i].recv_time_left  = 0U;
                            }
                          } else {                    // If there was error during reception
                            if ((len_read == ARM_DRIVER_ERROR) && (memcmp(resp_buf + 2, "-1", 2) == 0)) {
                              // "-1" : Connection lost
                              event_recv |= ((1U << 8) << i);
                            } else if (len_read == ARM_DRIVER_ERROR_TIMEOUT) {
                              socket_arr[i].recv_time_left  = 0U;
                            }
                          }
                          if (socket_arr[i].recv_time_left == 0U) {       // If receive timeout has expired
                            event_recv |= (1U << i);
                          }
                        }
                      }
                    } while ((socket_arr[i].len_to_recv != socket_arr[i].len_recv) && (len_read > 0));
                  }
                }

                osMutexRelease(mutex_id_sockets);
              }
            }

            osMutexRelease(mutex_id_spi);
          }

          if ((event_signal) && (signal_event_fn != NULL)) {
            signal_event_fn(ARM_WIFI_EVENT_AP_CONNECT, mac_ip4[ap_num_connected-1].mac);
          }
          if (event_accept != 0U) {
            osEventFlagsSet(event_id_accept, event_accept);
          }
          if (event_recv != 0U) {
            osEventFlagsSet(event_id_socket, event_recv);
          }
        }

        // Wait async interval unless new event was signaled to process
        osEventFlagsWait(event_id_async_poll, EVENT_ASYNC_MSG_POLL, osFlagsWaitAny, WIFI_ISM43362_ASYNC_INTERVAL);
      } while ((poll_async) || (poll_recv));
    }
  }
}

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
  uint8_t *ptr_u8_resp_buf;
  uint32_t timeout, resp_len, fw_ver;
  int      int_arr[4];

  ret = ARM_DRIVER_OK;

  signal_event_fn = cb_event;

  if (!driver_initialized) {
    // Clear all local variables
    spi_datardy_irq        = false;
    sta_connected          = false;
    sta_dhcp_enabled       = true;
    ap_running             = false;

    event_id_spi_ready     = NULL;
    event_id_spi_xfer_done = NULL;
    event_id_async_poll    = NULL;
    event_id_accept        = NULL;
    event_id_socket        = NULL;
    mutex_id_spi           = NULL;
    mutex_id_sockets       = NULL;
    thread_id_async_poll   = NULL;

    ap_num_connected       = 0U;
    ap_dhcp_lease_time     = 30U * 60U;

    memset((void *)sta_local_ip, 0, sizeof(sta_local_ip));
    memset((void *)ap_local_ip,  0, sizeof(ap_local_ip));
    memset((void *)mac_ip4,      0, sizeof(mac_ip4));
    memset((void *)cmd_buf,      0, sizeof(cmd_buf));
    memset((void *)resp_buf,     0, sizeof(resp_buf));
    memset((void *)socket_arr,   0, sizeof(socket_arr));

    // Create resources required by this driver
    mutex_id_spi           = osMutexNew(&mutex_spi_attr);
    mutex_id_sockets       = osMutexNew(&mutex_socket_attr);
    event_id_spi_ready     = osEventFlagsNew(NULL);
    event_id_spi_xfer_done = osEventFlagsNew(NULL);
    event_id_async_poll    = osEventFlagsNew(NULL);
    event_id_accept        = osEventFlagsNew(NULL);
    event_id_socket        = osEventFlagsNew(NULL);
    if ((mutex_id_spi           == NULL) || 
        (mutex_id_sockets       == NULL) || 
        (event_id_spi_ready     == NULL) || 
        (event_id_spi_xfer_done == NULL) || 
        (event_id_async_poll    == NULL) || 
        (event_id_accept        == NULL) || 
        (event_id_socket        == NULL)) {
      // If any of mutex or flag creation failed
      ret = ARM_DRIVER_ERROR;
    }

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
      ret = ptrSPI->Control     (ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(16), 10000000U);
    }

    // Reset WiFi Module
    if (ret == ARM_DRIVER_OK) {
      WiFi_ISM43362_Pin_RSTN(true);
      osDelay(50U);
      WiFi_ISM43362_Pin_RSTN(false);
      osDelay(100U);
    }

    // Wait for initial SPI ready and detect if user IRQ is provided for SPI ready detection
    if (ret == ARM_DRIVER_OK) {
      // Lock access to SPI interface (acquire mutex)
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        // Initial SPI ready wait and determine if SPI ready is signaled by IRQ
        WiFi_ISM43362_Pin_SSN(false);
        Wait_us(3U);
        spi_datardy_irq = false;
        for (timeout = WIFI_ISM43362_SPI_TIMEOUT; timeout != 0U; timeout --) {
          if (WiFi_ISM43362_Pin_DATARDY()) {
            if (osEventFlagsWait(event_id_spi_ready, EVENT_SPI_READY, osFlagsWaitAny, 10U) == EVENT_SPI_READY) {
              spi_datardy_irq = true;
            }
            break;
          }
          osDelay(1U);
        }
        if (timeout == 0U) {            // If DATARDY pin did not signal ready in WIFI_ISM43362_SPI_TIMEOUT seconds
          ret = ARM_DRIVER_ERROR;
        }

        // Initial fetch cursor procedure, read (3 * 16 bits) 6 bytes
        if (ret == ARM_DRIVER_OK) {
          WiFi_ISM43362_Pin_SSN(true);
          Wait_us(15U);
          memset((void *)resp_buf, 0, sizeof(resp_buf));
          if (ptrSPI->Receive(resp_buf, 3U) == ARM_DRIVER_OK) {
            if (SPI_WaitTransferDone(1000U)) {
              if (memcmp(resp_buf, "\x15\x15\r\n> ", 6) != 0) {
                // If initial cursor is not as expected
                ret = ARM_DRIVER_ERROR;
              }
            } else {                    // If transfer timed out
              ret = ARM_DRIVER_ERROR;
            }
          }
        }

        WiFi_ISM43362_Pin_SSN(false);
        if (osMutexRelease(mutex_id_spi) != osOK) {     // If SPI mutex release has failed
          ret = ARM_DRIVER_ERROR;
        }
      } else {                          // If SPI interface is not accessible (locked by another thread)
        ret = ARM_DRIVER_ERROR;
      }
    }

    // Create asynchronous message processing thread
    if (ret == ARM_DRIVER_OK) {
      thread_id_async_poll = osThreadNew(WiFi_AsyncMsgProcessThread, NULL, &thread_async_poll_attr);
      if (thread_id_async_poll == NULL) {   // If thread creation failed
        ret = ARM_DRIVER_ERROR;
      }
    }

    // Clear sockets local variables array (socket_arr)
    if (ret == ARM_DRIVER_OK) {
      memset((void *)socket_arr, 0, sizeof(socket_arr));
    }

    if (ret == ARM_DRIVER_OK) {         // If initialization succeeded
      driver_initialized = true;
    }
  }

  // Check firmware revision
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Show applications information
      memcpy((void *)cmd_buf, (void *)"I?\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      if (ret == ARM_DRIVER_OK) {
        ptr_u8_resp_buf = resp_buf + 2U;
        // Position pointer 1 character after next ',' (skip ",C")
        while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
        if (*ptr_u8_resp_buf == ',') {
          ptr_u8_resp_buf += 2U;
        } else {
          ptr_u8_resp_buf  = NULL;
        }
        // Extract firmware version
        if (ptr_u8_resp_buf != NULL) {
          if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
            fw_ver = ((uint32_t)int_arr[0] * 1000U) + ((uint32_t)int_arr[1] * 100U) + ((uint32_t)int_arr[2] * 10U) + (uint32_t)int_arr[3];
            if (fw_ver < 3525U) {       // If firmware version is less than 3.5.2.5
              ret = ARM_DRIVER_ERROR;
            }
          }
        } else {
          ret = ARM_DRIVER_ERROR;       // If firmware version read has failed
        }
      }
      if (osMutexRelease(mutex_id_spi) != osOK) {       // If SPI mutex release has failed
        ret = ARM_DRIVER_ERROR;
      }
    } else {                            // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret != ARM_DRIVER_OK) {
    // If something failed during initialization clean-up (release all resources)
    if (thread_id_async_poll != NULL) {
      if (osThreadTerminate(thread_id_async_poll) == osOK) {
        thread_id_async_poll = NULL;
      }
    }
    if (event_id_socket != NULL) {
      if (osEventFlagsDelete(event_id_socket) == osOK) {
        event_id_socket = NULL;
      }
    }
    if (event_id_accept != NULL) {
      if (osEventFlagsDelete(event_id_accept) == osOK) {
        event_id_accept = NULL;
      }
    }
    if (event_id_async_poll != NULL) {
      if (osEventFlagsDelete(event_id_async_poll) == osOK) {
        event_id_async_poll = NULL;
      }
    }
    if (event_id_spi_xfer_done != NULL) {
      if (osEventFlagsDelete(event_id_spi_xfer_done) == osOK) {
        event_id_spi_xfer_done = NULL;
      }
    }
    if (event_id_spi_ready != NULL) {
      if (osEventFlagsDelete(event_id_spi_ready) == osOK) {
        event_id_spi_ready = NULL;
      }
    }
    if (mutex_id_sockets != NULL) {
      if (osMutexDelete(mutex_id_sockets) == osOK) {
        mutex_id_spi = NULL;
      }
    }
    if (mutex_id_spi != NULL) {
      if (osMutexDelete(mutex_id_spi) == osOK) {
        mutex_id_spi = NULL;
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
  uint8_t i;

  ret = ARM_DRIVER_OK;

  if (driver_initialized) {
    // Close all sockets
    for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
      if (WiFi_SocketClose(i + WIFI_ISM43362_SOCKETS_NUM) != 0) {
        ret = ARM_DRIVER_ERROR;
      }
      if (WiFi_SocketClose(i) != 0) {
        ret = ARM_DRIVER_ERROR;
      }
    }

    // Release all resources
    if (thread_id_async_poll != NULL) {
      if (osThreadTerminate(thread_id_async_poll) == osOK) {
        thread_id_async_poll = NULL;
      }
    }
    if (event_id_socket != NULL) {
      if (osEventFlagsDelete(event_id_socket) == osOK) {
        event_id_socket = NULL;
      }
    }
    if (event_id_accept != NULL) {
      if (osEventFlagsDelete(event_id_accept) == osOK) {
        event_id_accept = NULL;
      }
    }
    if (event_id_async_poll != NULL) {
      if (osEventFlagsDelete(event_id_async_poll) == osOK) {
        event_id_async_poll = NULL;
      }
    }
    if (event_id_spi_xfer_done != NULL) {
      if (osEventFlagsDelete(event_id_spi_xfer_done) == osOK) {
        event_id_spi_xfer_done = NULL;
      }
    }
    if (event_id_spi_ready != NULL) {
      if (osEventFlagsDelete(event_id_spi_ready) == osOK) {
        event_id_spi_ready = NULL;
      }
    }
    if (mutex_id_spi != NULL) {
      if (osMutexDelete(mutex_id_spi) == osOK) {
        mutex_id_spi = NULL;
      }
    }

    if ((ret == ARM_DRIVER_OK)           &&
       ((thread_id_async_poll   != NULL) ||
        (event_id_socket        != NULL) ||
        (event_id_async_poll    != NULL) ||
        (event_id_spi_xfer_done != NULL) ||
        (event_id_spi_ready     != NULL) ||
        (mutex_id_sockets       != NULL) ||
        (mutex_id_spi           != NULL))){
      ret = ARM_DRIVER_ERROR;
    }

    if (ret == ARM_DRIVER_OK) {
      // If uninitialization succeeded
      signal_event_fn    = NULL;
      driver_initialized = false;
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
  uint32_t resp_len;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((state != ARM_POWER_OFF) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    if (state != ARM_POWER_OFF) {
      // If requested state is different than ARM_POWER_OFF, for ARM_POWER_OFF state do nothing
      // Lock access to SPI interface (acquire mutex)
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        memcpy((void *)cmd_buf, (void *)"ZP=1, \r", 8);
        switch (state) {
          case ARM_POWER_LOW:
            cmd_buf[5] = '1';
            break;

          case ARM_POWER_FULL:
            cmd_buf[5] = '0';
            break;

          default:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        }
        // Execute command and receive response
        if (ret == ARM_DRIVER_OK) {
          resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }
        osMutexRelease(mutex_id_spi);
      } else {                          // If SPI interface is not accessible (locked by another thread)
        ret = ARM_DRIVER_ERROR;
      }
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
  const uint8_t *ptr_u8_data;
        uint32_t resp_len;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((data == NULL) || (len == 0U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (option) {
      case ARM_WIFI_TX_POWER:                           // Station Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_IP6_GLOBAL:                         // Station Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_LINK_LOCAL:                     // Station Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:              // Station Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_IP6_GATEWAY:                        // Station Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DNS1:                           // Station Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DNS2:                           // Station Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DHCP_MODE:                      // Station Set/Get IPv6 DHCPv6 client mode;         data = &mode,     len =  4, mode     (uint32_t): ARM_WIFI_IP6_DHCP_xxx
      case ARM_WIFI_AP_TX_POWER:                        // AP      Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_AP_SSID_HIDE:                       // AP      Set/Get SSID hide option;                data = &en,       len =  4, en       (uint32_t): 0 = disable (default), non-zero = enable
      case ARM_WIFI_AP_IP_DHCP_POOL_BEGIN:              // AP      Set/Get IPv4 DHCP pool begin address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DHCP_POOL_END:                // AP      Set/Get IPv4 DHCP pool end address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP6_GLOBAL:                      // AP      Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_LINK_LOCAL:                  // AP      Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_SUBNET_PREFIX_LEN:           // AP      Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_AP_IP6_GATEWAY:                     // AP      Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS1:                        // AP      Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS2:                        // AP      Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
      case ARM_WIFI_MAC:                                // Station Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if (len != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && ((!driver_initialized) || (sta_connected))) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_AP_MAC:                             // AP      Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if (len != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && ((!driver_initialized) || (ap_running))) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_IP:                                 // Station Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_SUBNET_MASK:                     // Station Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_IP_GATEWAY:                         // Station Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DNS1:                            // Station Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DNS2:                            // Station Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DHCP:                            // Station Set/Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
      case ARM_WIFI_AP_IP:                              // AP      Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_SUBNET_MASK:                  // AP      Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_AP_IP_GATEWAY:                      // AP      Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS1:                         // AP      Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS2:                         // AP      Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_AP_IP_DHCP:                         // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (*((uint32_t *)data) == 0U)) {
          // AP DHCP server cannot be disabled
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:              // AP      Set/Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t)
        if (len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if (ret == ARM_DRIVER_OK) {
          if (*((uint32_t *)data) < (30U * 60U)) {
            // Less then 30 minutes
            ret = ARM_DRIVER_ERROR_PARAMETER;
          } else if (*((uint32_t *)data) > (254U * 60U * 60U)) {
            // More then 254 hours
            ret = ARM_DRIVER_ERROR_PARAMETER;
          }
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  // Execute set option
  if (ret == ARM_DRIVER_OK) {
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) { // Lock access to SPI interface (acquire mutex)
      ptr_u8_data = (uint8_t *)data;
      resp_len    = sizeof(resp_buf) - 1U;
      switch (option) {
        case ARM_WIFI_MAC:                              // Station Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        case ARM_WIFI_AP_MAC:                           // AP      Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
          // Set MAC address
          sprintf(cmd_buf, "Z4=%02X:%02X:%02X:%02X:%02X:%02X\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3], ptr_u8_data[4], ptr_u8_data[5]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP:                               // Station Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
          // Set network IP address
          sprintf(cmd_buf, "C6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_AP_IP:                            // AP      Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
          // Set AP IP address
          sprintf(cmd_buf, "Z6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP_SUBNET_MASK:                   // Station Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
        case ARM_WIFI_AP_IP_SUBNET_MASK:                // AP      Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
          // Set network IP mask
          sprintf(cmd_buf, "C7=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP_GATEWAY:                       // Station Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_GATEWAY:                    // AP      Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
          // Set network gateway
          sprintf(cmd_buf, "C8=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP_DNS1:                          // Station Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_DNS1:                       // AP      Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
          // Set network primary DNS
          sprintf(cmd_buf, "C9=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP_DNS2:                          // Station Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_DNS2:                       // AP      Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
          // Set network secondary DNS
          sprintf(cmd_buf, "CA=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          break;
        case ARM_WIFI_IP_DHCP:                          // Station Set/Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
          // Set network DHCP
          memcpy((void *)cmd_buf, (void *)"C4= \r", 6);
          if (*((uint32_t *)data) != 0) {
            cmd_buf[3] = '1';
          } else {
            cmd_buf[3] = '0';
          }
          resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          if (ret == ARM_DRIVER_OK) {
            // Store set value to local variable
            if (*((uint32_t *)data) == 0) {
              sta_dhcp_enabled = false;
            } else {
              sta_dhcp_enabled = true;
            }
          }
          break;
        case ARM_WIFI_AP_IP_DHCP:                       // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en   (uint32_t): 0 = disable, non-zero = enable (default)
          // AP DHCP server is enabled by default and cannot be disabled
          break;
        case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:            // AP      Set/Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec  (uint32_t)
          // Set AP DHCP lease time
          sprintf(cmd_buf, "AL=%d\r", (*((uint32_t *)data))/(60U*60U));
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          if (ret == ARM_DRIVER_OK) {
            // Store set value to local variable
            ap_dhcp_lease_time = ((*((uint32_t *)data))/(60U*60U))*60U*60U;
          }
          break;
        default:
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
          break;
      }
      osMutexRelease(mutex_id_spi);
    } else {                            // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
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
  int32_t             ret;
  uint8_t            *ptr_u8_resp_buf;
  uint8_t            *ptr_u8_data;
  ARM_WIFI_MAC_IP4_t *ptr_ap_mac_ip4;
  uint32_t            resp_len;
  int                 int_arr[6];
  uint8_t             i, num;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((data == NULL) || (len == NULL)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (option) {
      case ARM_WIFI_TX_POWER:                           // Station Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_IP6_GLOBAL:                         // Station Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_LINK_LOCAL:                     // Station Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_SUBNET_PREFIX_LEN:              // Station Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_IP6_GATEWAY:                        // Station Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DNS1:                           // Station Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DNS2:                           // Station Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_IP6_DHCP_MODE:                      // Station Set/Get IPv6 DHCPv6 client mode;         data = &mode,     len =  4, mode     (uint32_t): ARM_WIFI_IP6_DHCP_xxx
      case ARM_WIFI_AP_TX_POWER:                        // AP      Set/Get transmit power;                  data = &dBm,      len =  4, dBm      (uint32_t): 0 .. 20
      case ARM_WIFI_AP_SSID_HIDE:                       // AP      Set/Get SSID hide option;                data = &en,       len =  4, en       (uint32_t): 0 = disable (default), non-zero = enable
      case ARM_WIFI_AP_IP_DHCP_POOL_BEGIN:              // AP      Set/Get IPv4 DHCP pool begin address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DHCP_POOL_END:                // AP      Set/Get IPv4 DHCP pool end address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP6_GLOBAL:                      // AP      Set/Get IPv6 global address;             data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_LINK_LOCAL:                  // AP      Set/Get IPv6 link local address;         data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_SUBNET_PREFIX_LEN:           // AP      Set/Get IPv6 subnet prefix length;       data = &len,      len =  4, len      (uint32_t): 1 .. 127
      case ARM_WIFI_AP_IP6_GATEWAY:                     // AP      Set/Get IPv6 gateway address;            data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS1:                        // AP      Set/Get IPv6 primary   DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
      case ARM_WIFI_AP_IP6_DNS2:                        // AP      Set/Get IPv6 secondary DNS address;      data = &ip6,      len = 16, ip6      (uint8_t[16])
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

      case ARM_WIFI_SSID:                               // Station     Get SSID of connected AP;            data = &ssid,     len<= 33, ssid     (char[32+1]), null-terminated string
      case ARM_WIFI_PASS:                               // Station     Get Password of connected AP;        data = &pass,     len<= 65, pass     (char[64+1]), null-terminated string
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_BSSID:                              // Station     Get BSSID of connected AP;           data = &bssid,    len =  6, bssid    (uint8_t[6])
        if ((*len) != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_SECURITY:                           // Station     Get Security Type of connected AP;   data = &security, len =  4, security (uint32_t): ARM_WIFI_SECURITY_xxx
      case ARM_WIFI_RSSI:                               // Station     Get RSSI of connected AP;            data = &rssi,     len =  4, rssi     (uint32_t)
        if ((*len) != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_MAC:                                // Station Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
      case ARM_WIFI_AP_MAC:                             // AP      Set/Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if ((*len) != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_IP:                                 // Station Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_SUBNET_MASK:                     // Station Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_IP_GATEWAY:                         // Station Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DNS1:                            // Station Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DNS2:                            // Station Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_IP_DHCP:                            // Station Set/Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
      case ARM_WIFI_AP_IP:                              // AP      Set/Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_SUBNET_MASK:                  // AP      Set/Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_AP_IP_GATEWAY:                      // AP      Set/Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS1:                         // AP      Set/Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS2:                         // AP      Set/Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if ((*len) != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      case ARM_WIFI_AP_IP_DHCP:                         // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
      case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:              // AP      Set/Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t)
        if ((*len) != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        break;
      case ARM_WIFI_AP_IP_DHCP_TABLE:                   // AP          Get IPv4 DHCP table;                 data = &mac_ip4[],len = sizeof(mac_ip4[]), mac_ip4 (array of ARM_WIFI_MAC_IP4_t structures)
        if ((((*len) / sizeof(ARM_WIFI_MAC_IP4_t)) == 0U) || (((*len) % sizeof(ARM_WIFI_MAC_IP4_t)) != 0U)) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
        if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
          ret = ARM_DRIVER_ERROR;
        }
        break;
      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  // Execute get option
  if (ret == ARM_DRIVER_OK) {
    if (option != ARM_WIFI_AP_IP_DHCP_LEASE_TIME) {
      // For all options except ARM_WIFI_AP_IP_DHCP_LEASE_TIME 
      // read data through SPI from module is necessary
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {    // Lock access to SPI interface (acquire mutex)
        if ((option == ARM_WIFI_MAC) || (option == ARM_WIFI_AP_MAC)) {
          memcpy((void *)cmd_buf, (void *)"Z5\r", 4);                           // Read MAC address from module
        } else if (option == ARM_WIFI_AP_IP) {
          memcpy((void *)cmd_buf, (void *)"A?\r", 4);                           // Read AP settings from module
        } else if (option == ARM_WIFI_AP_IP_DHCP_TABLE) {
          memcpy((void *)cmd_buf, (void *)"AA\r", 4);                           // Read AP DHCP cached addresses from module
        } else if (option == ARM_WIFI_RSSI) {
          memcpy((void *)cmd_buf, (void *)"CR\r", 4);                           // Read RSSI of associated Access Point
        } else {
          memcpy((void *)cmd_buf, (void *)"C?\r", 4);                           // Read network settings from module
        }
        resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        osMutexRelease(mutex_id_spi);
      } else {                                                                  // If SPI interface is not accessible (locked by another thread)
        ret = ARM_DRIVER_ERROR;
      }
    }
    if (ret == ARM_DRIVER_OK) {
      switch (option) {
        case ARM_WIFI_SSID:                             // Station     Get SSID of connected AP;            data = &ssid,     len<= 33, ssid     (char[32+1]), null-terminated string
          // Extract ssid from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          if (sscanf((const char *)ptr_u8_resp_buf, "%s", data) != 1) {
            ret = ARM_DRIVER_ERROR;
          }
          break;
        case ARM_WIFI_PASS:                             // Station     Get Password of connected AP;        data = &pass,     len<= 65, pass     (char[64+1]), null-terminated string
          // Skip ssid (1 ',') from response on "C?" command
          if (ret == ARM_DRIVER_OK) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract password from response on "C?" command
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%s", data) != 1) {
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_SECURITY:                         // Station     Get Security Type of connected AP;   data = &security, len =  4, security (uint32_t): ARM_WIFI_SECURITY_xxx
          // Skip ssid, password (2 ',') from response on "C?" command
          if (ret == ARM_DRIVER_OK) {
            for (i = 0U; i < 2U; i++) {
              if (ptr_u8_resp_buf != NULL) {
                while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
                if (*ptr_u8_resp_buf == ',') {
                  ptr_u8_resp_buf++;
                } else {
                  ptr_u8_resp_buf = NULL;
                  ret = ARM_DRIVER_ERROR;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            }
          }
          // Extract security from response on "C?" command
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d", &int_arr[0]) == 1) {
              switch (int_arr[0]) {
                case 0:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_OPEN);
                  break;
                case 1:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_WEP);
                  break;
                case 2:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_WPA);
                  break;
                case 3:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_WPA2);
                  break;
                case 4:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_WPA2);
                  break;
                default:
                  __UNALIGNED_UINT32_WRITE(data, ARM_WIFI_SECURITY_UNKNOWN);
                  break;
              }
            } else {                      // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }

        case ARM_WIFI_RSSI:                             // Station     Get RSSI of connected AP;            data = &rssi,     len =  4, rssi     (uint32_t)
          // Convert read RSSI in dB to positive value representing RSSI for the driver
          ptr_u8_resp_buf = resp_buf + 2U;
          if (sscanf((const char *)ptr_u8_resp_buf, "%d", &int_arr[0]) == 1) {
            __UNALIGNED_UINT32_WRITE(data, (uint32_t)(int_arr[0] + 255));
          } else {
            ret = ARM_DRIVER_ERROR;
          }
          break;
        case ARM_WIFI_MAC:                              // Station Set/Get MAC;                             data = &mac,     len = 6,  mac      (uint8_t[6])
        case ARM_WIFI_AP_MAC:                           // AP      Set/Get MAC;                             data = &mac,     len = 6,  mac      (uint8_t[6])
          // Extract MAC from response on "Z5" command
          ptr_u8_resp_buf = resp_buf + 2U;
          if (sscanf((const char *)ptr_u8_resp_buf, "%x:%x:%x:%x:%x:%x", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[4], &int_arr[5]) == 6) {
            ptr_u8_data    = (uint8_t *)data;
            ptr_u8_data[0] = (uint8_t)int_arr[0];
            ptr_u8_data[1] = (uint8_t)int_arr[1];
            ptr_u8_data[2] = (uint8_t)int_arr[2];
            ptr_u8_data[3] = (uint8_t)int_arr[3];
            ptr_u8_data[4] = (uint8_t)int_arr[4];
            ptr_u8_data[5] = (uint8_t)int_arr[5];
          } else {                      // If extraction of required information from response has failed
            ret = ARM_DRIVER_ERROR;
          }
          break;
        case ARM_WIFI_IP:                               // Station Set/Get IPv4 static/assigned address;    data = &ip,      len = 4,  ip       (uint8_t[4])
          // Skip ssid, password, security, DHCP, IP version (5 ',') from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 5U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract IP address
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_AP_IP:                            // AP      Set/Get IPv4 static/assigned address;    data = &ip,      len = 4,  ip       (uint8_t[4])
          // Skip ssid (1 ',') from response on "A?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          if (ptr_u8_resp_buf != NULL) {
            while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
            if (*ptr_u8_resp_buf == ',') {
              ptr_u8_resp_buf++;
            } else {
              ptr_u8_resp_buf = NULL;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
          // Extract IP address
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_IP_SUBNET_MASK:                   // Station Set/Get IPv4 subnet mask;                data = &msk,     len = 4,  msk      (uint8_t[4])
        case ARM_WIFI_AP_IP_SUBNET_MASK:                // AP      Set/Get IPv4 subnet mask;                data = &msk,     len = 4,  msk      (uint8_t[4])
          // Skip ssid, password, security, DHCP, IP version, IP address (6 ',') from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 6U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract subnet IP mask
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_IP_GATEWAY:                       // Station Set/Get IPv4 gateway address;            data = &ip,      len = 4,  ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_GATEWAY:                    // AP      Set/Get IPv4 gateway address;            data = &ip,      len = 4,  ip       (uint8_t[4])
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask (7 ',') from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 7U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract gateway IP
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_IP_DNS1:                          // Station Set/Get IPv4 primary   DNS address;      data = &ip,      len = 4,  ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_DNS1:                       // AP      Set/Get IPv4 primary   DNS address;      data = &ip,      len = 4,  ip       (uint8_t[4])
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP 
          // (8 ',') from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 8U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract primary DNS IP
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_IP_DNS2:                          // Station Set/Get IPv4 secondary DNS address;      data = &ip,      len = 4,  ip       (uint8_t[4])
        case ARM_WIFI_AP_IP_DNS2:                       // AP      Set/Get IPv4 secondary DNS address;      data = &ip,      len = 4,  ip       (uint8_t[4])
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP, primary DNS IP 
          // (9 ',') from response on "C?" command
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 9U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract secondary DNS IP
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              ptr_u8_data    = (uint8_t *)data;
              ptr_u8_data[0] = (uint8_t)int_arr[0];
              ptr_u8_data[1] = (uint8_t)int_arr[1];
              ptr_u8_data[2] = (uint8_t)int_arr[2];
              ptr_u8_data[3] = (uint8_t)int_arr[3];
            } else {                    // If extraction of required information from response has failed
              ret = ARM_DRIVER_ERROR;
            }
          }
          break;
        case ARM_WIFI_IP_DHCP:                          // Station Set/Get IPv4 DHCP client enable/disable; data = &en,      len = 4,  en       (uint32_t): 0 = disable, non-zero = enable (default)
          // Get DHCP mode
          if (sta_dhcp_enabled) {
            *((uint32_t *)data) = 1U;
          } else {
            *((uint32_t *)data) = 0U;
          }
          break;
        case ARM_WIFI_AP_IP_DHCP:                       // AP      Set/Get IPv4 DHCP server enable/disable; data = &en,      len = 4,  en       (uint32_t): 0 = disable, non-zero = enable (default)
          // AP DHCP server cannot be disabled
          *((uint32_t *)data) = 1U;
          break;
        case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:            // AP      Set/Get IPv4 DHCP lease time;            data = &sec,     len = 4,  sec      (uint32_t)
          // Get AP DHCP lease time
          *((uint32_t *)data) = ap_dhcp_lease_time;
          break;
        case ARM_WIFI_AP_IP_DHCP_TABLE:                 // AP          Get IPv4 DHCP table;                 data = &mac_ip4[],len = sizeof(mac_ip4[]), mac_ip4 (array of ARM_WIFI_MAC_IP4_t structures)
          ptr_u8_resp_buf = resp_buf + 2U;
          ptr_ap_mac_ip4  = (ARM_WIFI_MAC_IP4_t *)data;
          num             = 0U;
          for (i = 0U; i < ((*len) / sizeof(ARM_WIFI_MAC_IP4_t)); i++) {
            if (ptr_u8_resp_buf != NULL) {
              // Parse MAC Address
              if (sscanf((const char *)ptr_u8_resp_buf, "%x:%x:%x:%x:%x:%x", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[4], &int_arr[5]) == 6) {
                // If MAC read from response correctly, store it to response
                ptr_ap_mac_ip4[i].mac[0] = (uint8_t)int_arr[0];
                ptr_ap_mac_ip4[i].mac[1] = (uint8_t)int_arr[1];
                ptr_ap_mac_ip4[i].mac[2] = (uint8_t)int_arr[2];
                ptr_ap_mac_ip4[i].mac[3] = (uint8_t)int_arr[3];
                ptr_ap_mac_ip4[i].mac[4] = (uint8_t)int_arr[4];
                ptr_ap_mac_ip4[i].mac[5] = (uint8_t)int_arr[5];

                // Position pointer after next ','
                while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
                if (*ptr_u8_resp_buf == ',') {
                  ptr_u8_resp_buf++;
                } else {
                  ptr_u8_resp_buf = NULL;
                }
              } else {
                break;
              }
            } else {
              break;
            }
            if (ptr_u8_resp_buf != NULL) {
              // Parse IP Address
              if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
                // If IP read from response correctly, store it to response
                ptr_ap_mac_ip4[i].ip4[0] = (uint8_t)int_arr[0];
                ptr_ap_mac_ip4[i].ip4[1] = (uint8_t)int_arr[1];
                ptr_ap_mac_ip4[i].ip4[2] = (uint8_t)int_arr[2];
                ptr_ap_mac_ip4[i].ip4[3] = (uint8_t)int_arr[3];
                num++;

                // Position pointer after next ','
                while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
                if (*ptr_u8_resp_buf == ',') {
                  ptr_u8_resp_buf++;
                } else {
                  ptr_u8_resp_buf = NULL;
                }
              } else {
                break;
              }
            } else {
              break;
            }
            if (ptr_u8_resp_buf == NULL) {
              break;
            }
          }
          *len = num * sizeof(ARM_WIFI_MAC_IP4_t);
          break;
        default:
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
          break;
      }
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
  int32_t  ret;
  uint8_t *ptr_u8_resp_buf;
  uint32_t resp_len;
  int      i, int_arr[6];

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ap_info == NULL) || (max_num == 0U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Scan for network access points
      memcpy((void *)cmd_buf, (void *)"F0\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      // Extract scan data
      if (ret == ARM_DRIVER_OK) {
        ptr_u8_resp_buf = resp_buf + 2;
        while ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL) && (sscanf((const char *)ptr_u8_resp_buf, "#%d", &i) == 1) && (i > 0) && (i <= (int32_t)max_num)) {
          i--;
          ptr_u8_resp_buf++;
          // Position pointer 1 character after next ',' (skip ',' and '"')
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf += 2U;
          } else {
            ptr_u8_resp_buf  = NULL;
          }
          // Extract SSID
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%[^\"]", ap_info[i].ssid) == 1) {
              // Position pointer after next ','
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract BSSID
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%x:%x:%x:%x:%x:%x", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[4], &int_arr[5]) == 6) {
              ap_info[i].bssid[0] = (uint8_t)int_arr[0];
              ap_info[i].bssid[1] = (uint8_t)int_arr[1];
              ap_info[i].bssid[2] = (uint8_t)int_arr[2];
              ap_info[i].bssid[3] = (uint8_t)int_arr[3];
              ap_info[i].bssid[4] = (uint8_t)int_arr[4];
              ap_info[i].bssid[5] = (uint8_t)int_arr[5];

              // Position pointer after next ','
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Extract RSSI
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d", &int_arr[0]) == 1) {
              ap_info[i].rssi = (uint8_t)(-int_arr[0]);

              // Position pointer after next ','
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
          // Skip bitrate
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
            if (*ptr_u8_resp_buf == ',') {
              ptr_u8_resp_buf++;
            } else {
              ptr_u8_resp_buf = NULL;
            }
          }
          // Skip wireless mode
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
            if (*ptr_u8_resp_buf == ',') {
              ptr_u8_resp_buf++;
            } else {
              ptr_u8_resp_buf = NULL;
            }
          }
          // Extract security
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (ptr_u8_resp_buf != NULL) {
              if        (memcmp((const void *)ptr_u8_resp_buf, (const void *)"WPA2", 4) == 0) {
                ap_info[i].security = ARM_WIFI_SECURITY_WPA2;
              } else if (memcmp((const void *)ptr_u8_resp_buf, (const void *)"WPA",  3) == 0) {
                ap_info[i].security = ARM_WIFI_SECURITY_WPA;
              } else if (memcmp((const void *)ptr_u8_resp_buf, (const void *)"WEP",  3) == 0) {
                ap_info[i].security = ARM_WIFI_SECURITY_WEP;
              } else if (memcmp((const void *)ptr_u8_resp_buf, (const void *)"Open", 4) == 0) {
                ap_info[i].security = ARM_WIFI_SECURITY_OPEN;
              } else {
                ret = ARM_DRIVER_ERROR;
              }

              // Position pointer after next ','
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            }
          }
          // Skip frequency
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
            if (*ptr_u8_resp_buf == ',') {
              ptr_u8_resp_buf++;
            } else {
              ptr_u8_resp_buf = NULL;
            }
          }
          // Extract channel
          if ((ret == ARM_DRIVER_OK) && (ptr_u8_resp_buf != NULL)) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d", &int_arr[0]) == 1) {
              ap_info[i].ch = (uint8_t)(int_arr[0]);

              // Position pointer to next '#'
              while ((*ptr_u8_resp_buf != '#') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf != '#') {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ret = i + 1;
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
  int32_t  ret;
  uint8_t *ptr_u8_resp_buf;
  uint32_t resp_len;
  int      ip_[4];

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL))) {
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
  if ((ret == ARM_DRIVER_OK) && (ch > 13)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Set network SSID
      sprintf(cmd_buf, "C1=%s\r", ssid); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      // Set network passphrase
      if (ret == ARM_DRIVER_OK) {
        sprintf(cmd_buf, "C2=%s\r", pass); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Set network security mode
      if (ret == ARM_DRIVER_OK) {
        memcpy((void *)cmd_buf, (void *)"C3= \r", 6);
        switch (security) {
          case ARM_WIFI_SECURITY_OPEN:
            cmd_buf[3] = '0';
            break;
          case ARM_WIFI_SECURITY_WEP:
            cmd_buf[3] = '1';
            break;
          case ARM_WIFI_SECURITY_WPA:
            cmd_buf[3] = '2';
            break;
          case ARM_WIFI_SECURITY_WPA2:
            cmd_buf[3] = '3';
            break;
          default:
            ret = ARM_DRIVER_ERROR_PARAMETER;
            break;
        }
        if (ret == ARM_DRIVER_OK) {
          resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }
      }
      // Send command to join a network
      memcpy((void *)cmd_buf, (void *)"C0\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      if (ret == ARM_DRIVER_OK) {
        ptr_u8_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "JOIN ");
        if (ptr_u8_resp_buf != NULL) {
          // If message contains "JOIN " string, parse it and extract IP
          // Position pointer after next ','
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf++;
          } else {
            ptr_u8_resp_buf = NULL;
          }
          if (ptr_u8_resp_buf != NULL) {
            // Parse IP Address and port
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &ip_[0], &ip_[1], &ip_[2], &ip_[3]) == 4) {
              sta_local_ip[0] = (uint8_t)ip_[0];
              sta_local_ip[1] = (uint8_t)ip_[1];
              sta_local_ip[2] = (uint8_t)ip_[2];
              sta_local_ip[3] = (uint8_t)ip_[3];
            }
          }
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR_TIMEOUT;
    }
  }

  if (ret == 0) {
    sta_connected = true;
  } else {
    memset((void *)sta_local_ip, 0, 4);
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
  uint8_t *ptr_u8_resp_buf;
  uint32_t resp_len;
  int      int_arr[4];

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      if (*pin != NULL) {               // If pin connection requested
        // Set WPS pin
        if (ret == ARM_DRIVER_OK) {
          sprintf(cmd_buf, "Z7=%s\r", pin); resp_len = sizeof(resp_buf) -1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }
        // Activate WPS pin connection
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)cmd_buf, (void *)"CW=0\r", 6);
        }
      } else {                          // If push-button connection requested
        // Activate WPS push-button connection
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)cmd_buf, (void *)"CW=1\r", 6);
        }
      }
      // Execute command
      if (ret == ARM_DRIVER_OK) {
        resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, 120000U);
      }

      // Check if WPS connection has succeeded
      if (ret == ARM_DRIVER_OK) {
        if (resp_len > 0) {
          resp_buf[resp_len] = 0U;      // Terminate received string
        }
        ptr_u8_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "WPS ");
        if (ptr_u8_resp_buf != NULL) {
          // If message contains "WPS " string, parse it and extract ip
          // Position pointer after next ','
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf++;
          } else {
            ptr_u8_resp_buf = NULL;
          }
          if (ptr_u8_resp_buf != NULL) {
            // Parse IP Address and port
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
              sta_local_ip[0] = (uint8_t)int_arr[0];
              sta_local_ip[1] = (uint8_t)int_arr[1];
              sta_local_ip[2] = (uint8_t)int_arr[2];
              sta_local_ip[3] = (uint8_t)int_arr[3];
            }
          } else {
            memset((void *)sta_local_ip, 0, 4);
            ret = ARM_DRIVER_ERROR;
          }
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR_TIMEOUT;
    }
  }

  if (ret == 0) {
    sta_connected = true;
  } else {
    memset((void *)sta_local_ip, 0, 4);
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
  uint32_t resp_len;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Disconnect from network
      memcpy((void *)cmd_buf, (void *)"CD\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == 0) {
    memset((void *)sta_local_ip, 0, 4);
    sta_connected = false;
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
  int32_t  ret, con;
  uint32_t resp_len;

  ret = ARM_DRIVER_OK;
  con = 0;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Check connection status
      memcpy((void *)cmd_buf, (void *)"CS\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      if ((ret == ARM_DRIVER_OK) && (resp_len >= 3)) {
        if (resp_buf[2] == '1') {
          con = 1;
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ret = con;
    sta_connected = 1;
  } else {
    ret = 0;
    sta_connected = 0;
  }

  return ret;
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
  int32_t  ret;
  uint8_t *ptr_u8_resp_buf;
  uint32_t resp_len;
  int      int_arr[4];

  ret = ARM_DRIVER_OK;

  // Check conditions
  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL))) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if (ret == ARM_DRIVER_OK) {
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        break;
      case ARM_WIFI_SECURITY_WEP:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
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
  if ((ret == ARM_DRIVER_OK) && (ch > 13)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Set AP security mode
      memcpy((void *)cmd_buf, (void *)"A1= \r", 6);
      switch (security) {
        case ARM_WIFI_SECURITY_OPEN:
          cmd_buf[3] = '0';
          break;
        case ARM_WIFI_SECURITY_WPA:
          cmd_buf[3] = '2';
          break;
        case ARM_WIFI_SECURITY_WPA2:
          cmd_buf[3] = '3';
          break;
        default:
          ret = ARM_DRIVER_ERROR_PARAMETER;
          break;
      }
      if (ret == ARM_DRIVER_OK) {
        resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Set AP security key (password)
      if (ret == ARM_DRIVER_OK) {
        sprintf(cmd_buf, "A2=%s\r", pass); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Set AP channel (0 = autoselect)
      if (ret == ARM_DRIVER_OK) {
        sprintf(cmd_buf, "AC=%d\r", ch); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Set AP SSID
      if (ret == ARM_DRIVER_OK) {
        sprintf(cmd_buf, "AS=0,%s\r", ssid); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Set AP maximum number of clients to maximum which is 4
      if (ret == ARM_DRIVER_OK) {
        memcpy((void *)cmd_buf, (void *)"AT=4\r", 6); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Activate AP direct connect mode
      if (ret == ARM_DRIVER_OK) {
        memcpy((void *)cmd_buf, (void *)"AD\r", 4); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Check if AP has started and extract IP address from response
      if (ret == ARM_DRIVER_OK) {
        if (resp_len > 0) {
          resp_buf[resp_len] = 0U;      // Terminate received string
        }
        ptr_u8_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "[AP ");
        if (ptr_u8_resp_buf != NULL) {
          // Position pointer after next ','
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf++;
          } else {
            ptr_u8_resp_buf = NULL;
          }
        }
        if (ptr_u8_resp_buf != NULL) {
          // Position pointer after next ','
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf++;
          } else {
            ptr_u8_resp_buf = NULL;
          }
        }
        if (ptr_u8_resp_buf != NULL) {
          if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3]) == 4) {
            ap_local_ip[0] = (uint8_t)int_arr[0];
            ap_local_ip[1] = (uint8_t)int_arr[1];
            ap_local_ip[2] = (uint8_t)int_arr[2];
            ap_local_ip[3] = (uint8_t)int_arr[3];
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR_TIMEOUT;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ap_running = true;
    osEventFlagsSet(event_id_async_poll, EVENT_ASYNC_MSG_POLL);
  } else {
    memset((void *)ap_local_ip, 0, 4);
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
  int32_t  ret;
  uint32_t resp_len;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Exit AP direct connect mode
      memcpy((void *)cmd_buf, (void *)"AE\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ap_running = false;
    memset((void *)ap_local_ip, 0, 4);
  }

  return ret;
}

/**
  \fn          int32_t WiFi_AP_IsRunning (void)
  \brief       Check Access Point running status.
  \return      running status
                 - value != 0: running
                 - value = 0: not running
*/
static int32_t WiFi_AP_IsRunning (void) {
  int32_t  ret, run;
  uint8_t *ptr_u8_resp_buf;
  uint32_t resp_len;
  uint8_t  i;

  ret = ARM_DRIVER_OK;
  run = 0;

  // Check conditions
  if (!driver_initialized) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Check connection status
      memcpy((void *)cmd_buf, (void *)"A?\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      if (ret == ARM_DRIVER_OK) {
        // Skip ssid, IP address, channel, security, key, AP DHCP, Lease Time (7 ',') from response
        ptr_u8_resp_buf = resp_buf + 2U;
        for (i = 0U; i < 7U; i++) {
          if (ptr_u8_resp_buf != NULL) {
            while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
            if (*ptr_u8_resp_buf == ',') {
              ptr_u8_resp_buf++;
            } else {
              ptr_u8_resp_buf = NULL;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        if ((ptr_u8_resp_buf != NULL) && (*ptr_u8_resp_buf == '1')) {
          run = 1;
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    ret = run;
    ap_running = 1;
  } else {
    ret = 0;
    ap_running = 0;
  }

  return ret;
}

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
  return ARM_DRIVER_ERROR_UNSUPPORTED;
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
  return ARM_DRIVER_ERROR_UNSUPPORTED;
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
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          uint32_t WiFi_EthGetRxFrameSize (void)
  \brief       Get size of received Ethernet frame (in bypass mode only).
  \return      number of bytes in received frame
*/
static uint32_t WiFi_EthGetRxFrameSize (void) {
  return 0U;
}

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
  int32_t ret, i;

  ret = 0;

  // Check parameters
  if (ret == 0) {
    switch (af) {
      case ARM_SOCKET_AF_INET:
        break;
      case ARM_SOCKET_AF_INET6:
        ret = ARM_SOCKET_ENOTSUP;
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }
  }
  if (ret == 0) {
    switch (type) {
      case ARM_SOCKET_SOCK_DGRAM:
        if (protocol != ARM_SOCKET_IPPROTO_UDP) {
          ret = ARM_SOCKET_EINVAL;
        }
        break;
      case ARM_SOCKET_SOCK_STREAM:
        if (protocol != ARM_SOCKET_IPPROTO_TCP) {
          ret = ARM_SOCKET_EINVAL;
        }
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
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Find first free socket
      for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
        if (socket_arr[i].state == SOCKET_STATE_FREE) {
          break;
        }
      }
      if (i < WIFI_ISM43362_SOCKETS_NUM) {
        ret = i;                          // Index of found free socket
      } else {
        ret = ARM_SOCKET_ENOMEM;          // No free socket is available
      }

      if (ret >= 0) {
        // If socket creation succeeded
        memset((void *)&socket_arr[i], 0, sizeof(socket_t));
        socket_arr[i].protocol     = (uint8_t)protocol;
        socket_arr[i].recv_timeout = WIFI_ISM43362_SOCKET_DEF_TIMEOUT;
        socket_arr[i].send_timeout = WIFI_ISM43362_SOCKET_DEF_TIMEOUT;
        socket_arr[i].state        = SOCKET_STATE_CREATED;
      }

      osMutexRelease(mutex_id_sockets);
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
  int32_t  ret, i;
  uint32_t resp_len;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len != 4U))) {
    ret = ARM_SOCKET_EINVAL;
  }
  if ((ret == 0) && 
      (memcmp((void *)ip, "\x00\x00\x00\x00", 4U) != 0) &&      // If not 0.0.0.0 (all IP address accepted)
      (memcmp((void *)ip, sta_local_ip,       4U) != 0) &&      // If not sta_local_ip
      (memcmp((void *)ip, ap_local_ip,        4U) != 0)) {      // If not ap_local_ip
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].state != SOCKET_STATE_CREATED)) {
        ret = ARM_SOCKET_EINVAL;
      }
      if (ret == 0) {
        for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
          if (socket_arr[i].local_port == port) {
            // If local port is already used by another socket
            ret = ARM_SOCKET_EADDRINUSE;
            break;
          }
        }
      }

      // Execute functionality on the module through SPI commands
      if ((ret == 0) && (socket_arr[socket].protocol == ARM_SOCKET_SOCK_DGRAM) && (socket_arr[socket].server == 0U)) {
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          if (socket_arr[socket].server == 0U) {
            // Set communication socket number
            sprintf(cmd_buf, "P0=%d\r", socket); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            // Select transport protocol to UDP
            if (ret == 0) {
              memcpy((void *)cmd_buf, (void *)"P1=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Set transport local port number
            if (ret == 0) {
              sprintf(cmd_buf, "P2=%d\r", port); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Start UDP transport server
            if (ret == 0) {
              memcpy((void *)cmd_buf, (void *)"P5=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                socket_arr[socket].server = 1U;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
            osMutexRelease(mutex_id_spi);
          }
        } else {
          // If SPI interface is not accessible (locked by another thread)
          ret = ARM_SOCKET_ERROR;
        }
      }

      if (ret == 0) {
        // If socket bind succeeded
        socket_arr[socket].local_port  = port;
        socket_arr[socket].state       = SOCKET_STATE_BOUND;
      }

      osMutexRelease(mutex_id_sockets);
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
  int32_t  ret;
  uint32_t resp_len;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret ==0) && (backlog != 1)) {
    // Only backlog 1 is supported
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].protocol != ARM_SOCKET_SOCK_STREAM)) {
        ret = ARM_SOCKET_ENOTSUP;
      }
      if ((ret == 0) && (socket_arr[socket].state != SOCKET_STATE_BOUND)) {
        if (socket_arr[socket].state == SOCKET_STATE_CONNECTED) {
          ret = ARM_SOCKET_EISCONN;
        } else {
          ret = ARM_SOCKET_EINVAL;
        }
      }

      // Execute functionality on the module through SPI commands
      if ((ret == 0) && (socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].server == 0U)) {
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          if (socket_arr[socket].server == 0U) {
            // Set communication socket number
            sprintf(cmd_buf, "P0=%d\r", socket); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            // Select transport protocol to TCP
            if (ret == 0) {
              memcpy((void *)cmd_buf, (void *)"P1=0\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Set transport local port number
            if (ret == 0) {
              sprintf(cmd_buf, "P2=%d\r", socket_arr[socket].local_port); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Start TCP transport server
            if (ret == 0) {
              memcpy((void *)cmd_buf, (void *)"P5=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                socket_arr[socket].server = 1U;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
            osMutexRelease(mutex_id_spi);
          }
        } else {
          // If SPI interface is not accessible (locked by another thread)
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
  int32_t  ret;
  uint32_t flags;
  uint8_t  virtual_socket;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    ret = ARM_SOCKET_ESOCK;
  }

  virtual_socket = socket + WIFI_ISM43362_SOCKETS_NUM;
  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].protocol != ARM_SOCKET_SOCK_STREAM)) {
        ret = ARM_SOCKET_ENOTSUP;
      }
      if ((ret == 0) && (socket_arr[socket].state != SOCKET_STATE_LISTENING) && (socket_arr[socket].state != SOCKET_STATE_ACCEPTING)) {
        ret = ARM_SOCKET_EINVAL;
      }
      if ((ret == 0) && (socket_arr[virtual_socket].state != SOCKET_STATE_FREE)) {
        // ISM43362 module limitation: only one socket can be processed at a time
        ret = ARM_SOCKET_ERROR;
      }
      if ((ret == 0) && (!driver_initialized)) {
        ret = ARM_SOCKET_ERROR;
      }

      if (ret == 0) {
        if (socket_arr[socket].state != SOCKET_STATE_ACCEPTING) {
          socket_arr[socket].state = SOCKET_STATE_ACCEPTING;
          // Start polling for asynchronous messages signaling accept
          osEventFlagsSet(event_id_async_poll, EVENT_ASYNC_MSG_POLL);
        }
        if (socket_arr[socket].non_blocking != 0U) {          // If non-blocking mode
          if (socket_arr[socket].state == SOCKET_STATE_ACCEPTING) {
            flags = osEventFlagsWait(event_id_accept, (0x10001U << socket), osFlagsWaitAny, 1);
            if ((flags & 0x80000000UL) != 0U) {               // If error
              if (flags == osFlagsErrorTimeout) {
                ret = ARM_SOCKET_EAGAIN;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            } else if ((flags & (0x10000U << socket)) != 0U) {// If abort signaled from SocketClose
              ret = ARM_SOCKET_ERROR;
            }
            // If ret == 0, then accept was signaled from async thread
          }
        } else {                                              // If blocking mode
          flags = osEventFlagsWait(event_id_accept, (0x10001U << socket), osFlagsWaitAny, osWaitForever);
          if ((flags & 0x80000000UL) != 0U) {                 // If error
            ret = ARM_SOCKET_ERROR;
          } else if ((flags & (0x10000U << socket)) != 0U) {  // If abort signaled from SocketClose
            ret = ARM_SOCKET_ERROR;
          }
          // If ret == 0, then accept was signaled from async thread
        }
      }

      if (ret == 0) {
        // If socket accept succeeded, copy all variables to new (virtual) socket
        memcpy((void *)&socket_arr[virtual_socket], (void *)&socket_arr[socket], sizeof(socket_t));
        socket_arr[socket].state         = SOCKET_STATE_ACCEPTED;
        socket_arr[virtual_socket].state = SOCKET_STATE_CONNECTED;
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }


  // Because on ISM when accept succeeds same socket is used for communication, 
  // to comply with BSD we return different socket id (virtual) which is used for 
  // further communication, this number is considered virtual socket id
  if (ret == 0) {
    ret = virtual_socket;
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
  int32_t  ret;
  uint32_t resp_len;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len != 4U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].state == SOCKET_STATE_CONNECTED)) {
        ret = ARM_SOCKET_EISCONN;
      }
      if ((ret == 0) && ((socket_arr[socket].state != SOCKET_STATE_CREATED) && (socket_arr[socket].state != SOCKET_STATE_BOUND))) {
        ret = ARM_SOCKET_EINVAL;
      }

      if (ret == 0) {
        if (socket_arr[socket].state != SOCKET_STATE_CONNECTING) {      // If first call of connect
          if ((socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].client == 0U)) {
            if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
              if (socket_arr[socket].client == 0U) {
                // Set communication socket number
                sprintf(cmd_buf, "P0=%d\r", socket); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                  ret = ARM_SOCKET_ERROR;
                }
                // Select transport protocol to TCP
                if (ret == 0) {
                  memcpy((void *)cmd_buf, (void *)"P1=0\r", 6); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                    ret = ARM_SOCKET_ERROR;
                  }
                }
                // Set transport remote host IP address
                if (ret == 0) {
                  sprintf(cmd_buf, "P3=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                    ret = ARM_SOCKET_ERROR;
                  }
                }
                // Set transport remote port number
                if (ret == 0) {
                  sprintf(cmd_buf, "P4=%d\r", port); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                    ret = ARM_SOCKET_ERROR;
                  }
                }
                // Start TCP transport client
                if (ret == 0) {
                  memcpy((void *)cmd_buf, (void *)"P6=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                    socket_arr[socket].client = 1U;
                  } else {
                    ret = ARM_SOCKET_ERROR;
                  }
                }

                if (ret == 0) {
                  socket_arr[socket].state = SOCKET_STATE_CONNECTING;
                  if (socket_arr[socket].non_blocking != 0U) {          // If non-blocking mode
                    ret = ARM_SOCKET_EINPROGRESS;
                  }
                }

                osMutexRelease(mutex_id_spi);
              }
            } else {
              // If SPI interface is not accessible (locked by another thread)
              ret = ARM_SOCKET_ETIMEDOUT;
            }
          }
        }
        // Only for first call oc connect in non-blocking mode we return ARM_SOCKET_EINPROGRESS, 
        // otherwise as this module does not give any information on successful connect we 
        // consider that connect was successful and return 0
      }

      if (ret == 0) {
        // If socket connect succeeded
        socket_arr[socket].state = SOCKET_STATE_CONNECTED;
      }

      osMutexRelease(mutex_id_sockets);
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
  return WiFi_SocketRecvFrom(socket, buf, len, NULL, NULL, NULL);
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
  int32_t  ret;
  int32_t  len_read;
  uint32_t timeout, flags;
  uint32_t resp_len;
  uint32_t len_to_rece, len_tot_rece, len_req;
  uint8_t  hw_socket;
  bool     long_timeout, trigger_async_polling;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
        ret = ARM_SOCKET_ENOTCONN;
      }
      if ((ret == 0) && (!driver_initialized)) {
        ret = ARM_SOCKET_ERROR;
      }

      hw_socket = socket;
      if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
        hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
      }
      len_tot_rece = 0;

      if (ret == 0) {
        long_timeout = false;
        if (socket_arr[socket].non_blocking != 0U) {            // If non-blocking
          timeout = 1U;
        } else {                                                // If blocking
          if (socket_arr[socket].recv_timeout <= WIFI_ISM43362_ASYNC_INTERVAL) {
            // If short blocking, set timeout to requested
            timeout = socket_arr[socket].recv_timeout;
          } else {
            // If long blocking
            timeout = 1U;
            long_timeout = true;
          }
        }

        if ((ret == 0) && (socket_arr[socket].data_to_recv == NULL)) {
          // Setup receive parameters on socket of the module
          // Execute functionality on the module through SPI commands
          // Lock access to SPI interface (acquire mutex)
          if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
            // Set communication socket number
            sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            if ((ret == 0)                                              && 
                (socket_arr[socket].protocol == ARM_SOCKET_SOCK_DGRAM)  &&  // If UDP socket
                (socket_arr[socket].state    != SOCKET_STATE_BOUND)     &&  // Socket not bound
                (socket_arr[socket].client   == 0U)) {                      // UDP client not running on this socket
              // Select transport protocol to UDP
              memcpy((void *)cmd_buf, (void *)"P1=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
              // Set transport remote host IP address
              if (ret == 0) {
                sprintf(cmd_buf, "P3=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                  ret = ARM_SOCKET_ERROR;
                }
              }
              // Set transport remote port number
              if (ret == 0) {
                sprintf(cmd_buf, "P4=%d\r", socket_arr[socket].local_port); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                  ret = ARM_SOCKET_ERROR;
                }
              }
              // Start UDP transport client
              if (ret == 0) {
                memcpy((void *)cmd_buf, (void *)"P6=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                  socket_arr[socket].client = 1U;
                } else {
                  ret = ARM_SOCKET_ERROR;
                }
              }
            }
            // Set receive timeout (ms)
            if (ret == 0) {
              sprintf(cmd_buf, "R2=%d\r", timeout); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }

            osMutexRelease(mutex_id_spi);
          } else {
            // If SPI interface is not accessible (locked by another thread)
            ret = ARM_SOCKET_ERROR;
          }
        }

        if (ret == 0) {
          if (long_timeout) {                                                   // If long blocking receive request
            trigger_async_polling = false;
            if (socket_arr[socket].data_to_recv == NULL) {                      // If long blocking receive is not yet active
              // Store information for receiving to be done from async thread
              socket_arr[socket].data_to_recv   = buf;
              socket_arr[socket].len_to_recv    = len;
              socket_arr[socket].len_recv       = 0U;
              socket_arr[socket].recv_time_left = (socket_arr[socket].recv_timeout + 1U);
              trigger_async_polling             = true;
            }
          } else {                                                              // If short blocking receive or non-blocking receive request
            // Execute functionality on the module through SPI commands
            // Lock access to SPI interface (acquire mutex)
            if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
              len_to_rece  = len;
              len_tot_rece = 0U;

              while ((ret == 0) && (len_tot_rece < len_to_rece)) {
                len_req = len_to_rece - len_tot_rece;
                if (len_req > 1200U) {
                  len_req = 1200U;
                }
                // Set read data packet size
                sprintf(cmd_buf, "R1=%d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                  ret = ARM_SOCKET_ERROR;
                }
                // Receive data
                if (ret == 0) {
                  memcpy((void *)cmd_buf, (void *)"R0\r", 4); resp_len = sizeof(resp_buf) - 1U;
                  len_read = SPI_AT_SendCommandReceiveDataAndResponse(cmd_buf, (uint8_t *)buf + len_tot_rece, len_req, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
                  if (len_read > 0) {
                    len_tot_rece += len_read;
                  } else if (len_read == 0) {
                    ret = ARM_SOCKET_EAGAIN;
                  } else {
                    if ((len_read == ARM_DRIVER_ERROR) && (memcmp(resp_buf + 2, "-1", 2) == 0)) {
                      // "-1" : Connection lost
                      socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
                      ret = ARM_SOCKET_ECONNRESET;
                    } else if (len_read == ARM_DRIVER_ERROR_TIMEOUT) {
                      ret = ARM_SOCKET_ETIMEDOUT;
                    } else {
                      ret = ARM_SOCKET_ERROR;
                    }
                  }
                }
              }
              osMutexRelease(mutex_id_spi);
            } else {
              // If SPI interface is not accessible (locked by another thread)
              ret = ARM_SOCKET_ERROR;
            }
          }
        }
      }
      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  if ((ret == 0) && (long_timeout)) {
    if (trigger_async_polling) {
      // If requested activate asynchronous polling
      osEventFlagsSet(event_id_async_poll, EVENT_ASYNC_MSG_POLL);
    }
    // Wait for reception or timeout or disconnect signaled from aynshronous thread
    flags = osEventFlagsWait(event_id_socket, (0x00010101U << socket), osFlagsWaitAny, socket_arr[socket].recv_timeout);
    if (ret == 0) {
      if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
        if ((flags & 0x80000000UL) != 0U) {               // If error
          if (flags != osFlagsErrorTimeout) {
            ret = ARM_SOCKET_ERROR;
          }
        } else if ((flags & ((1U << 16) << socket)) != 0U) {// If reception was aborted locally by closing socket
          ret = ARM_SOCKET_ECONNABORTED;
        } else if ((flags & ((1U <<  8) << socket)) != 0U) {// If reception was aborted by remote host closing socket
          socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
          ret = ARM_SOCKET_ECONNRESET;
        } else if ((flags & ((1U      ) << socket)) != 0U) {// If reception has finished (by received all requested bytes or by timeout)
          len_tot_rece = socket_arr[socket].len_recv;
        } else {                                            // If still nothing was received
          ret = ARM_SOCKET_EAGAIN;
        }
        if (ret != ARM_SOCKET_EAGAIN) {
          // Clear information for receiving to be done from async thread, as it has finished for last receive operation
          socket_arr[socket].data_to_recv = NULL;
          socket_arr[socket].len_to_recv  = 0U;
          socket_arr[socket].len_recv     = 0U;
        }
        osMutexRelease(mutex_id_sockets);
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }
  }

  if (len_tot_rece > 0) {
    ret = len_tot_rece;
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
  return WiFi_SocketSendTo(socket, buf, len, NULL, 0U, 0U);
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
  int32_t  ret;
  int32_t  len_sent;
  uint32_t resp_len;
  uint32_t len_to_send, len_tot_sent, len_req;
  uint8_t  hw_socket;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
        ret = ARM_SOCKET_ENOTCONN;
      }
      if ((ret == 0) && (!driver_initialized)) {
        ret = ARM_SOCKET_ERROR;
      }

      hw_socket = socket;
      if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
        hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
      }
      len_tot_sent = 0;

      // Execute functionality on the module through SPI commands
      if (ret == 0) {
        // Lock access to SPI interface (acquire mutex)
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          // Set communication socket number
          sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
            ret = ARM_SOCKET_ERROR;
          }
          if ((ret == 0)                                              && 
              (socket_arr[socket].protocol == ARM_SOCKET_SOCK_DGRAM)  &&    // If UDP socket
              (socket_arr[socket].state    != SOCKET_STATE_BOUND)     &&    // Socket not bound
              (socket_arr[socket].client   == 0U)) {                        // UDP client not running on this socket
            // Select transport protocol to UDP
            memcpy((void *)cmd_buf, (void *)"P1=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            // Set transport remote host IP address
            if (ret == 0) {
              sprintf(cmd_buf, "P3=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Set transport remote port number
            if (ret == 0) {
              sprintf(cmd_buf, "P4=%d\r", socket_arr[socket].local_port); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
            // Start UDP transport client
            if (ret == 0) {
              memcpy((void *)cmd_buf, (void *)"P6=1\r", 6); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                socket_arr[socket].client = 1U;
              } else {
                ret = ARM_SOCKET_ERROR;
              }
            }
          }
          // Set transmit timeout (ms)
          if (ret == 0) {
            sprintf(cmd_buf, "S2=%d\r", socket_arr[socket].send_timeout); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
          }
          if (ret == 0) {
            len_to_send  = len;
            len_tot_sent = 0U;

            while ((ret == 0) && (len_tot_sent < len_to_send)) {
              len_req = len_to_send - len_tot_sent;
              if (len_req > 1200U) {
                len_req = 1200U;
              }
              // Send data
              sprintf(cmd_buf, "S3=%04d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
              len_sent = SPI_AT_SendCommandAndDataReceiveResponse(cmd_buf, (uint8_t *)buf + len_tot_sent, len_req, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
              if (len_sent > 0) {
                len_tot_sent += len_sent;
              } else if (len_sent == 0) {
                ret = ARM_SOCKET_EAGAIN;
              } else {
                if ((len_sent == ARM_DRIVER_ERROR) && (memcmp(resp_buf + 2, "-1", 2) == 0)) {
                  // "-1" : Connection lost
                  socket_arr[socket].state = SOCKET_STATE_DISCONNECTED;
                  ret = ARM_SOCKET_ECONNRESET;
                } else if (len_sent == ARM_DRIVER_ERROR_TIMEOUT) {
                  ret = ARM_SOCKET_ETIMEDOUT;
                } else {
                  ret = ARM_SOCKET_ERROR;
                }
              }
            }
          }
          osMutexRelease(mutex_id_spi);
        } else {
          // If SPI interface is not accessible (locked by another thread)
          ret = ARM_SOCKET_ERROR;
        }
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
  }

  if (len_tot_sent > 0) {
    ret = len_tot_sent;
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
  uint8_t *ptr_u8_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  int      int_arr[5];
  uint8_t  hw_socket;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U) || (port == NULL))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && ((socket_arr[socket].state == SOCKET_STATE_FREE) || (socket_arr[socket].state == SOCKET_STATE_CREATED))) {
        ret = ARM_SOCKET_EINVAL;
      }

      hw_socket = socket;
      if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
        hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
      }

      // Set communication socket number
      sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
        ret = ARM_SOCKET_ERROR;
      }

      if (ret == 0) {
        // Show Transport Settings
        memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
          // Skip Protocol from response
          ptr_u8_resp_buf = resp_buf + 2U;
          while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
          if (*ptr_u8_resp_buf == ',') {
            ptr_u8_resp_buf++;
          } else {
            ptr_u8_resp_buf = NULL;
          }

          // Parse Local IP Address and Port
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d,%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[5]) == 5) {
              ip[0] = (uint8_t )int_arr[0];
              ip[1] = (uint8_t )int_arr[1];
              ip[2] = (uint8_t )int_arr[2];
              ip[3] = (uint8_t )int_arr[3];
              *port = (uint16_t)int_arr[4];
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
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
  uint8_t *ptr_u8_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  int      int_arr[5];
  uint8_t  hw_socket, i;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U) || (port == NULL))) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if ((ret == 0) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
        ret = ARM_SOCKET_ENOTCONN;
      }

      hw_socket = socket;
      if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
        hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
      }

      // Set communication socket number
      sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
        ret = ARM_SOCKET_ERROR;
      }

      if (ret == 0) {
        // Show Transport Settings
        memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
          // Skip Protocol, Local IP, Local Port (3 ',') from response
          ptr_u8_resp_buf = resp_buf + 2U;
          for (i = 0U; i < 3U; i++) {
            if (ptr_u8_resp_buf != NULL) {
              while ((*ptr_u8_resp_buf != ',') && (*ptr_u8_resp_buf != 0U)) { ptr_u8_resp_buf++; }
              if (*ptr_u8_resp_buf == ',') {
                ptr_u8_resp_buf++;
              } else {
                ptr_u8_resp_buf = NULL;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }

          // Parse Remote IP Address and Port
          if (ptr_u8_resp_buf != NULL) {
            if (sscanf((const char *)ptr_u8_resp_buf, "%d.%d.%d.%d,%d", &int_arr[0], &int_arr[1], &int_arr[2], &int_arr[3], &int_arr[5]) == 5) {
              ip[0] = (uint8_t )int_arr[0];
              ip[1] = (uint8_t )int_arr[1];
              ip[2] = (uint8_t )int_arr[2];
              ip[3] = (uint8_t )int_arr[3];
              *port = (uint16_t)int_arr[4];
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR;
        }
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
    }
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
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((opt_len == NULL) || (*opt_len < 4U)) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      switch (opt_id) {
        case ARM_SOCKET_IO_FIONBIO:
          *((uint32_t *)opt_val) = (uint32_t)socket_arr[socket].non_blocking;
          *opt_len = 4U;
          break;
        case ARM_SOCKET_SO_RCVTIMEO:
          *((uint32_t *)opt_val) = socket_arr[socket].recv_timeout;
          *opt_len = 4U;
          break;
        case ARM_SOCKET_SO_SNDTIMEO:
          *((uint32_t *)opt_val) = socket_arr[socket].send_timeout;
          *opt_len = 4U;
          break;
        default:
          ret = ARM_SOCKET_ENOTSUP;
          break;
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
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
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }
  if ((opt_len == NULL) || (opt_len != 4U)) {
    ret = ARM_SOCKET_EINVAL;
  }

  if (ret == 0) {
    val = *((uint32_t *)opt_val);
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
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
        default:
          ret = ARM_SOCKET_ENOTSUP;
          break;
      }

      osMutexRelease(mutex_id_sockets);
    } else {
      ret = ARM_SOCKET_ERROR;
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
  int32_t  ret;
  uint32_t resp_len;
  uint8_t  hw_socket;

  ret = 0;

  // Check parameters and state
  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    ret = ARM_SOCKET_ESOCK;
  }

  if (ret == 0) {
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {      // Lock socket variables
      // Check state depending on socket variables
      if (socket_arr[socket].state == SOCKET_STATE_FREE) {
        ret = ARM_SOCKET_ESOCK;
      }
      hw_socket = socket;
      if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
        hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
      }
      if (socket != hw_socket) {
        if (socket_arr[socket].state != SOCKET_STATE_FREE) {
          // If request to close socket but socket corresponding to it (virtual) is not free
          ret = ARM_SOCKET_ERROR;
        }
      }

      // Execute functionality on the module through SPI commands
      if (ret == 0) {
        if ((socket_arr[socket].client == 1U) || (socket_arr[socket].server == 1U)) {
          if ((ret == 0) && (!driver_initialized)) {
            ret = ARM_SOCKET_ERROR;
          }
          if (ret == 0) {
            // Lock access to SPI interface (acquire mutex)
            if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
              // Set communication socket number
              sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
              // Select transport protocol
              if (ret == 0) {
                memcpy((void *)cmd_buf, (void *)"P1= \r", 6);
                if (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) {
                  cmd_buf[3] = '1';
                } else {
                  cmd_buf[3] = '0';
                }
                resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                  ret = ARM_SOCKET_ERROR;
                }
              }
              // Stop transport server
              if ((ret == 0) && (socket_arr[socket].server == 1U)) {
                if (ret == 0) {
                  memcpy((void *)cmd_buf, (void *)"P5=0\r", 6); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                    socket_arr[socket].server = 0U;
                  } else {
                    ret = ARM_SOCKET_ERROR;
                  }
                }
              }
              // Stop transport client
              if ((ret == 0) && (socket_arr[socket].client == 1U)) {
                if (ret == 0) {
                  memcpy((void *)cmd_buf, (void *)"P6=0\r", 6); resp_len = sizeof(resp_buf) - 1U;
                  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                    socket_arr[socket].client = 0U;
                  } else {
                    ret = ARM_SOCKET_ERROR;
                  }
                }
              }
              osMutexRelease(mutex_id_spi);
            } else {
              // If SPI interface is not accessible (locked by another thread)
              ret = ARM_SOCKET_EAGAIN;
            }
          }
        }

        if (ret == 0) {
          if ((event_id_accept != NULL) && (socket_arr[socket].state == SOCKET_STATE_ACCEPTING)) {
            // If socket is waiting for flag in accept, send flag to terminate accept
            osEventFlagsSet(event_id_accept, (0x10000U << socket));
          }
          if ((event_id_socket != NULL) && (socket_arr[socket].data_to_recv != NULL) && (socket_arr[socket].recv_time_left != 0U)) {
            // If socket is waiting for reception but has timed-out send event
            osEventFlagsSet(event_id_socket, ((1U << 16) << socket));
          }
        }
      }

      if (ret == 0) {
        // If socket close succeeded, clear all variables
        memset ((void *)&socket_arr[socket], 0 , sizeof(socket_t));
      }

      osMutexRelease(mutex_id_sockets);
    } else {
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
  int32_t  ret;
  uint32_t resp_len;
  int      ip_[4];

  ret = 0;

  // Check parameters and state
  if (af != ARM_SOCKET_AF_INET) {
    ret = ARM_SOCKET_ENOTSUP;
  }
  if ((ret == 0) && ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U))) {
    ret = ARM_SOCKET_EINVAL;
  }
  if ((ret == 0) && (strlen(name) > 64)) {
    // ISM43362 Limitation: Domain name is limited to 64 characters
    ret = ARM_SOCKET_ERROR;
  }
  if ((ret == 0) && (!driver_initialized)) {
    ret = ARM_SOCKET_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == 0) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Send command for DNS lookup
      sprintf(cmd_buf, "D0=%s\r", name); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
        if (resp_len > 0) {
          resp_buf[resp_len] = 0U;      // Terminate received string
          // Check that "Host not found " string is not present in response
          if (strstr((const char *)resp_buf, "Host not found ") == NULL) {
            // Parse IP Address
            if (sscanf((const char *)resp_buf + 2, "%d.%d.%d.%d", &ip_[0], &ip_[1], &ip_[2], &ip_[3]) == 4) {
              // IP and port read from response correctly
              ip[0] = (uint8_t)(ip_[0]);
              ip[1] = (uint8_t)(ip_[1]);
              ip[2] = (uint8_t)(ip_[2]);
              ip[3] = (uint8_t)(ip_[3]);
            } else {
              ret = ARM_SOCKET_ERROR;
            }
          } else {
            ret = ARM_SOCKET_EHOSTNOTFOUND;
          }
        } else {
          ret = ARM_SOCKET_ERROR;
        }
      } else {
        ret = ARM_SOCKET_ERROR;
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_SOCKET_ETIMEDOUT;
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
  uint32_t resp_len;

  ret = ARM_DRIVER_OK;

  // Check conditions
  if (ip == NULL){
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (ip_len == 16U)) {
    ret = ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  if ((ret == ARM_DRIVER_OK) && (ip_len != 4U)) {
    ret = ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((ret == ARM_DRIVER_OK) && (!driver_initialized)) {
    ret = ARM_DRIVER_ERROR;
  }

  // Execute functionality on the module through SPI commands
  if (ret == ARM_DRIVER_OK) {
    // Lock access to SPI interface (acquire mutex)
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Set ping target address
      sprintf(cmd_buf, "T1=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      // Ping IP target address
      if (ret == ARM_DRIVER_OK) {
        memcpy((void *)cmd_buf, (void *)"T0\r", 4); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        if ((ret == ARM_DRIVER_OK) && (strstr((const char *)resp_buf, "Timeout") != NULL)) {
          // If no response from the remote host
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
      osMutexRelease(mutex_id_spi);
    } else {
      // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
  }

  return ret;
}


// Structure exported by driver Driver_WiFin
extern
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_ISM43362_DRIVER_INDEX);
ARM_DRIVER_WIFI ARM_Driver_WiFi_(WIFI_ISM43362_DRIVER_INDEX) = { 
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
  WiFi_BypassControl,
  WiFi_EthSendFrame,
  WiFi_EthReadFrame,
  WiFi_EthGetRxFrameSize,
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
