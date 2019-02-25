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
 * $Revision:    V1.0 (beta)
 *
 * Driver:       Driver_WiFin (n = WIFI_ISM43362_DRV_NUM value)
 * Project:      WiFi Driver for 
 *               Inventek ISM43362-M3G-L44 WiFi Module (SPI variant)
 * --------------------------------------------------------------------------
 * Use the WiFi_ISM43362_Config.h file for compile time configuration 
 * of this driver.
 *
 * Notes:
 * This driver uses SPI for communicating with ISM module, however there are 
 * 3 pins that are not handled by SPI peripheral, and they are:
 *  - RSTN    = reset        (active low)  (output)
 *  - SSN     = slave select (active low)  (output)
 *  - DATARDY = data ready   (active high) (input)
 *
 * To drive SSN and RSTN pins, and get state of DATARDY pin you need to 
 * implement following functions (function template is available in 
 * WiFi_ISM43362_HW.c file and should be adapted according to hardware):
 *   - void    WiFi_ISM43362_Pin_RSTN    (uint8_t rstn)
 *   - void    WiFi_ISM43362_Pin_SSN     (uint8_t ssn)
 *   - uint8_t WiFi_ISM43362_Pin_DATARDY (void)
 *
 * For better performance of the driver DATARDY pin state change should be 
 * interrupt driven and following function should be called when line state 
 * changes from inactive to active (otherwise the driver will use polling 
 * with 1 ms interval to check DATARDY line state change):
 *   - void WiFi_ISM43362_Pin_DATARDY_IRQ (void)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0 (beta)
 *    Initial beta version
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "cmsis_compiler.h"

#include "Driver_WiFi.h"

#include "Driver_SPI.h"

#include "WiFi_ISM43362_Config.h"       // Driver configuration settings


// Hardware dependent functions --------

// Externally provided hardware dependent handling callback functions

extern void    WiFi_ISM43362_Pin_RSTN    (uint8_t rstn);
extern void    WiFi_ISM43362_Pin_SSN     (uint8_t ssn);
extern uint8_t WiFi_ISM43362_Pin_DATARDY (void);

// Exported hardware dependent function called by user code

extern void WiFi_ISM43362_Pin_DATARDY_IRQ (void);

/**
  \fn          void WiFi_ISM43362_Pin_DATARDY_IRQ (void)
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
  0U,                                   // Bypass or pass-through mode (Ethernet interface) not supported
  0U,                                   // Event not generated on Ethernet frame reception in bypass mode
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
  uint16_t local_port;                  // Local port number
  uint16_t remote_port;                 // Remote host port number
  uint8_t  remote_ip[4];                // Remote host IP
                                        // Module specific socket variables
  void    *data_to_recv;                // Pointer to where data should be received
  uint32_t len_to_recv;                 // Number of bytes to receive
  uint32_t len_recv;                    // Number of bytes received
  uint32_t recv_time_left;              // Receive Time left until Timeout
  uint8_t  client;                      // Socket client running
  uint8_t  server;                      // Socket server running
  uint16_t reserved1;                   // Reserved
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
#define EVENT_SPI_XFER_DONE            (1U     )
#define EVENT_SPI_READY                (1U << 1)
#define EVENT_ASYNC_POLL               (1U << 2)
#define EVENT_ACCEPT                   (1U << 3)
#define EVENT_SOCKET                   (1U << 4)

// Local macros
#define SPI_Driver_(n)                  Driver_SPI##n
#define SPI_Driver(n)                   SPI_Driver_(n)
extern ARM_DRIVER_SPI                   SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM);
#define ptrSPI                        (&SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM))

// Mutex responsible for protecting SPI media access
const osMutexAttr_t mutex_spi_attr = {
  "Mutex_SPI",                                  // Mutex name
  osMutexPrioInherit,                           // attr_bits
  NULL,                                         // Memory for control block   
  0U                                            // Size for control block
};

// Mutex responsible for protecting socket local variables access
const osMutexAttr_t mutex_socket_attr = {
  "Mutex_Socket",                               // Mutex name
  osMutexPrioInherit,                           // attr_bits
  NULL,                                         // Memory for control block   
  0U                                            // Size for control block
};

// Thread for polling and processing asynchronous messages
const osThreadAttr_t thread_async_poll_attr = {
  .name       =  "Thread_Async_Poll",           // Thread name
  .priority   =  WIFI_ISM43362_ASYNC_PRIORITY
};


// Local variables and structures
static uint8_t                          driver_initialized = 0U;
static ARM_WIFI_SignalEvent_t           signal_event_fn;

static uint8_t                          spi_datardy_irq;
static osEventFlagsId_t                 event_flags_id;
static osEventFlagsId_t                 event_flags_sockets_id;
static osMutexId_t                      mutex_id_spi;
static osMutexId_t                      mutex_id_sockets;
static osThreadId_t                     thread_id_async_poll;

static uint8_t                          sta_connected;
static uint8_t                          sta_dhcp_enabled;
static uint8_t                          ap_running;
static uint8_t                          ap_num_connected;
static uint32_t                         ap_dhcp_lease_time;

static uint8_t                          sta_local_ip  [4];
static uint8_t                          ap_local_ip   [4];
static ARM_WIFI_MAC_IP4_t               mac_ip4       [8];

static char                             cmd_buf [128  +1] __ALIGNED(4);
static uint8_t                          resp_buf[1210 +1] __ALIGNED(4);

static socket_t                         socket_arr[2 * WIFI_ISM43362_SOCKETS_NUM];

// Function prototypes
static int32_t WiFi_Uninitialize   (void);
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
  osEventFlagsSet(event_flags_id, EVENT_SPI_READY);
}

/**
  \fn          void SPI_SignalEvent (uint32_t event)
  \brief       SPI Signal Event callback, called by SPI driver when an SPI event occurs.
  \param[in]   event    Event signaled by SPI driver
  \return      none
*/
static void SPI_SignalEvent (uint32_t event) {
  if (event & ARM_SPI_EVENT_TRANSFER_COMPLETE) {
    osEventFlagsSet(event_flags_id, EVENT_SPI_XFER_DONE);
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
    __NOP();
  }
}

/**
  \fn          uint8_t *SkipCommas (uint8_t const *ptr, uint8_t num)
  \brief       Skip requested number of commas in character buffer.
  \param[in]   ptr      Pointer to character buffer
  \param[in]   num      Number of commas to skip
  \return      pointer to first character after requested number of commas, NULL in case of failure
*/
static uint8_t *SkipCommas (uint8_t const *ptr, uint8_t num) {
  char ch;

  if (ptr != NULL) {
    while (num > 0U) {
      do {
        ch = (char)*ptr++;
      } while ((ch != ',') && (ch != 0));
      if (ch == 0) {
        return NULL;
      }
      num--;
    }
  }

  return (uint8_t *)ptr;
}

/**
  \fn          void SPI_WaitReady (uint32_t timeout)
  \brief       Wait for SPI ready (DATARDY pin active).
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      SPI ready state
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
      if (ret == 0U) {                  // If DATARDY is ready
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
  \fn          void SPI_WaitTransferDone (uint32_t timeout)
  \brief       Wait for SPI transfer to finish.
  \param[in]   timeout  Timeout in milliseconds (0 = no timeout)
  \return      SPI transfer finished state
                 - 0: SPI transfer finished successfully
                 - 1: SPI transfer did not finish successfully
*/
static uint8_t SPI_WaitTransferDone (uint32_t timeout) {
  return (uint8_t)((osEventFlagsWait(event_flags_id, EVENT_SPI_XFER_DONE, osFlagsWaitAll, timeout) & EVENT_SPI_XFER_DONE) == EVENT_SPI_XFER_DONE);
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
  uint32_t cmd_len;
  uint8_t  tmp[2], wait_cmd;

  if (cmd == NULL) {                            // If cmd pointer is invalid
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  cmd_len = strlen(cmd);
  if (cmd_len == 0U) {                          // If length of command is 0
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  ret = ARM_DRIVER_OK;
  WiFi_ISM43362_Pin_SSN(false);                 // Deactivate slave select line
  Wait_us(4U);                                  // Wait 4 us

  if (SPI_WaitReady(timeout) != 0U) {           // If SPI is ready
    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us

    if (cmd_len > 2U) {
      // If command contains more than 2 bytes send even number of bytes first
      if (ptrSPI->Send(cmd, cmd_len / 2) != ARM_DRIVER_OK) {    // If SPI transfer failed
        ret = ARM_DRIVER_ERROR;
      } else {
        if (SPI_WaitTransferDone(timeout) == 0U) {              // If SPI transfer timed out
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && ((cmd_len & 1U) == 1U)) {
      // If command contains odd number of bytes, append 1 byte of data to the last byte
      // of command if there is data to be sent, otherwise append 1 byte of value 0x0A
      // and send those 2 bytes
      tmp[0] = (uint8_t)cmd[cmd_len - 1U];
      if ((data != NULL) && (data_len != 0U)) {
        tmp[1] = data[0];
        data_len--;
        data++;
      } else {
        tmp[1] = '\n';
      }
      if (ptrSPI->Send(tmp, 1U) != ARM_DRIVER_OK) {             // If SPI transfer failed
        ret = ARM_DRIVER_ERROR;
      } else {
        if (SPI_WaitTransferDone(timeout) == 0U) {              // If SPI transfer timed out
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && (data != NULL) && ((data_len / 2) > 0U)) {
      // Send even number of bytes of remaining data
      if (ptrSPI->Send(data, data_len / 2) != ARM_DRIVER_OK) {  // If SPI transfer failed
        ret = ARM_DRIVER_ERROR;
      } else {
        if (SPI_WaitTransferDone(timeout) == 0U) {              // If SPI transfer timed out
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }
    if ((ret == ARM_DRIVER_OK) && (data != NULL) && ((data_len & 1U) == 1U)) {
      // If remaining data contains odd number of bytes, append 1 byte of value 0x0A
      // and send those 2 bytes
      tmp[0] = data[data_len - 1U];
      tmp[1] = 0x0AU;
      if (ptrSPI->Send(tmp, 1U) != ARM_DRIVER_OK) {             // If SPI transfer failed
        ret = ARM_DRIVER_ERROR;
      } else {
        if (SPI_WaitTransferDone(timeout) == 0U) {              // If SPI transfer timed out
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      }
    }

    if (ret == ARM_DRIVER_ERROR_TIMEOUT) {                      // If SPI transfer has timed out
      ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);               // abort the SPI transfer
    }

    WiFi_ISM43362_Pin_SSN(false);               // Deactivate slave select line
    if (spi_datardy_irq != 0U) {                // If events are generated by WiFi_ISM43362_Pin_DATARDY_IRQ function
      // IRQ detects edge so it is not necessary to wait for DATARDY to go to inactive state
      Wait_us(3U);                              // Wait 3 us
    } else {
      // Wait max 1 ms for DATARDY line to go to inactive state signaling end of command
      wait_cmd = 100U;
      do {
        if (WiFi_ISM43362_Pin_DATARDY() == 0U) {
          break;
        }
        Wait_us(10U);                           // Wait 10 us
        wait_cmd--;
      } while (wait_cmd > 0U);
    }
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
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
  uint32_t total_len, len;
  int32_t  i;

  if ((data == NULL) && (data_len == 0U)) {     // If data pointer is invalid or data_len is 0
    return ARM_DRIVER_ERROR;
  }

  WiFi_ISM43362_Pin_SSN(false);                 // Deactivate slave select line
  Wait_us(3U);                                  // Wait 3 us

  if (SPI_WaitReady(timeout) != 0U) {           // If SPI is ready
    ret       = ARM_DRIVER_OK;
    total_len = 0U;

    WiFi_ISM43362_Pin_SSN(true);                // Activate slave select line
    Wait_us(15U);                               // Wait 15 us

    do {
      len = data_len;
      if (len > WIFI_ISM43362_SPI_RECEIVE_SIZE) {
        len = WIFI_ISM43362_SPI_RECEIVE_SIZE;
      }
      if (ptrSPI->Receive(data + total_len, (len + 1U) / 2) == ARM_DRIVER_OK) {
        if (SPI_WaitTransferDone(timeout) != 0U) {
          total_len += len;
          data_len  -= len;
        } else {                                // If SPI transfer timed out
          ret = ARM_DRIVER_ERROR_TIMEOUT;
        }
      } else {                                  // If SPI transfer failed
        ptrSPI->Control(ARM_SPI_ABORT_TRANSFER, 0);
        ret = ARM_DRIVER_ERROR;
      }
    } while ((ret == ARM_DRIVER_OK) && (data_len > 0U) && (WiFi_ISM43362_Pin_DATARDY()));

    if ((ret == ARM_DRIVER_OK) && (WiFi_ISM43362_Pin_DATARDY() == 0U)) {
      // If reception ended with DATARDY line inactive
      // correct number of received bytes to reduce the trailing 0x15 bytes
      for (i = total_len; i > 0; i--) {
        if (data[i - 1U] != 0x15U) {            // If non 0x15 value from end was found
          break;
        }
      }
      total_len = i;
    }

    WiFi_ISM43362_Pin_SSN(false);               // Deactivate slave select line
    Wait_us(3U);                                // Wait 3 us
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = total_len;
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

  ret = SPI_AT_SendCommandAndData(cmd, NULL, 0U, timeout);
  if (ret == ARM_DRIVER_OK) {
    rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
    if (rece_num < 0) {
      ret = rece_num;
    } else if ((rece_num < 8) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
      ret = ARM_DRIVER_ERROR;
    } else {
      resp[rece_num] = 0;               // Terminate response string
    }
    *resp_len = rece_num;
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

  ret = SPI_AT_SendCommandAndData(cmd, data, data_len, timeout);
  if (ret == ARM_DRIVER_OK) {
    rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
    if (rece_num < 0) {
      ret = rece_num;
    } else if ((rece_num < 8) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
      ret = ARM_DRIVER_ERROR;
    } else {
      resp[rece_num] = 0;               // Terminate response string
      // Parse how many data bytes were sent from response
      if (sscanf((const char *)(resp + 2), "%d", &ret) != 1) {
        ret = ARM_DRIVER_ERROR;
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

  ret = SPI_AT_SendCommandAndData(cmd, NULL, 0U, timeout);
  if (ret == ARM_DRIVER_OK) {
    rece_num = SPI_AT_ReceiveData(resp, *resp_len, timeout);
    if (rece_num < 0) {
      ret = rece_num;
    } else if ((rece_num < 10) || (memcmp((const void *)(resp + rece_num - 8), (const void *)"\r\nOK\r\n> ", 8) != 0)) {
      ret = ARM_DRIVER_ERROR;
    } else {
      resp[rece_num] = 0;               // Terminate response string
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

  return ret;
}

/**
  \fn          int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server)
  \brief       Start or stop transport server or client.
  \param [in]  socket       Socket identification number
  \param [in]  protocol     Protocol (ARM_SOCKET_SOCK_DGRAM or ARM_SOCKET_SOCK_STREAM)
  \param [in]  ip           Pointer to remote destination IP address
  \param [in]  local_port   Local port number
  \param [in]  remote_ip    Pointer to remote IP4 address
  \param [in]  remote_port  Remote port number
  \param [in]  start        Start/stop request (0 = stop, 1 = start)
  \param [in]  server       Server/client (0 = client, 1 = server)
  \return      status information
                 - 0                            : Operation successful
                 - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server) {
  uint32_t resp_len;

  // Set communication socket number
  sprintf(cmd_buf, "P0=%d\r", socket); resp_len = sizeof(resp_buf) - 1U;
  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
    return ARM_SOCKET_ERROR;
  }
  // Select transport protocol
  memcpy((void *)cmd_buf, (void *)"P1=1\r", 6);
  if (protocol == ARM_SOCKET_SOCK_STREAM) {
    cmd_buf[3] = '0';
  }
  resp_len = sizeof(resp_buf) - 1U;
  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
    return ARM_SOCKET_ERROR;
  }
  if (server != 0U) {
    // Set transport local port number
    sprintf(cmd_buf, "P2=%d\r", local_port); resp_len = sizeof(resp_buf) - 1U;
    if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
      return ARM_SOCKET_ERROR;
    }
  } else {
    // Set transport remote host IP address
    sprintf(cmd_buf, "P3=%d.%d.%d.%d\r", remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]); resp_len = sizeof(resp_buf) - 1U;
    if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
      return ARM_SOCKET_ERROR;
    }
    // Set transport remote port number
    sprintf(cmd_buf, "P4=%d\r", remote_port); resp_len = sizeof(resp_buf) - 1U;
    if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
      return ARM_SOCKET_ERROR;
    }
  }
  // Server/Client Start/Stop
  if (server != 0U) {
    memcpy((void *)cmd_buf, (void *)"P5=0\r", 6);
  } else {
    memcpy((void *)cmd_buf, (void *)"P6=0\r", 6);
  }
  if (start != 0U) {
    cmd_buf[3] = '1';
  }
  resp_len = sizeof(resp_buf) - 1U;
  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
    return ARM_SOCKET_ERROR;
  }

  return 0;
}

/**
  \fn          void WiFi_AsyncMsgProcessThread (void)
  \brief       Thread that reads and processes asynchronous messages from WiFi module.
  \return      none
*/
__NO_RETURN static void WiFi_AsyncMsgProcessThread (void *arg) {
  uint8_t *ptr_resp_buf;
  uint32_t event_accept, event_recv;
  uint32_t resp_len, len_req;
  int32_t  len_read;
  uint8_t  u8_arr[6];
  uint8_t  poll_async, poll_recv, repeat, event_signal;
  uint16_t u16_val;
  int8_t   i, hw_socket;

  for (;;) {
    if ((osEventFlagsWait(event_flags_id, EVENT_ASYNC_POLL, osFlagsWaitAny, osWaitForever) & EVENT_ASYNC_POLL) == EVENT_ASYNC_POLL) {
      do {
        if (driver_initialized == 0U) {
          break;
        }

        // Check if thread should poll for asynchronous messages or receive in long blocking
        poll_async = 0U;
        poll_recv  = 0U;
        for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
          if  (socket_arr[i].state == SOCKET_STATE_ACCEPTING) {
            poll_async = 1U;
          }
          if ((socket_arr[i].data_to_recv != NULL) && (socket_arr[i].recv_time_left != 0U)){
            poll_recv = 1U;
          }
        }
        if (ap_running != 0U) {
          poll_async = 1U;
        }

        if ((poll_async != 0U) || (poll_recv != 0U)) {
          event_signal = 0U;
          event_accept = 0U;
          event_recv   = 0U;

          if (osMutexAcquire(mutex_id_sockets, WIFI_ISM43362_ASYNC_INTERVAL) == osOK) { // Lock socket variables
            if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {      // Lock access to SPI interface (acquire mutex)
              if (poll_async != 0U) {
                // Send command to read asynchronous message
                memcpy((void *)cmd_buf, (void *)"MR\r", 4); resp_len = sizeof(resp_buf) - 1U;
                if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                  // If message is asynchronous Accept
                  ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "Accepted ");
                  if (ptr_resp_buf != NULL) {
                    // If message contains "Accepted " string, parse it and extract ip and port
                    ptr_resp_buf += 9U;
                    // Parse IP Address and port
                    if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu:%hu", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3], &u16_val) == 5) {
                      // IP and port read from response correctly
                      // Find which socket is listening on accepted port
                      for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
                        if (socket_arr[i].local_port == u16_val) {
                          socket_arr[i].remote_ip[0] = u8_arr[0];
                          socket_arr[i].remote_ip[1] = u8_arr[1];
                          socket_arr[i].remote_ip[2] = u8_arr[2];
                          socket_arr[i].remote_ip[3] = u8_arr[3];
                          break;
                        }
                      }
                      if (i != WIFI_ISM43362_SOCKETS_NUM) {
                        // Socket 'i' has accepted connection
                        event_accept |= 1U << i;
                      }
                    }

                    // If message is asynchronous Assign
                    ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "Assigned ");
                    if (ptr_resp_buf != NULL) {
                      // If message contains "Assigned " string, parse it and extract MAC
                      ptr_resp_buf += 9U;
                      // Parse MAC Address
                      if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3], &u8_arr[4], &u8_arr[5]) == 6) {
                        // Check if MAC already exists in mac_ip4 array, if it does ignore it, otherwise add it to array
                        for (i = 0; i < 8; i++) {
                          if (memcmp((void *)mac_ip4[i].mac, (void *)u8_arr, 6) == 0) {
                            break;
                          }
                        }
                        if ((i < 8) && (ap_num_connected < 8)) {
                          memcpy((void *)mac_ip4[ap_num_connected].mac, (void *)u8_arr, 6);
                          ap_num_connected++;
                          event_signal = 1U;
                        }
                      }
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
                            if (socket_arr[i].recv_time_left != 0xFFFFFFFFU) {
                              if (socket_arr[i].recv_time_left >= WIFI_ISM43362_ASYNC_INTERVAL) {
                                socket_arr[i].recv_time_left -= WIFI_ISM43362_ASYNC_INTERVAL;
                              } else {
                                socket_arr[i].recv_time_left  = 0U;
                              }
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
              }
              osMutexRelease(mutex_id_spi);
            }
            osMutexRelease(mutex_id_sockets);
          }

          if ((event_signal != 0U) && (signal_event_fn != NULL)) {
            signal_event_fn(ARM_WIFI_EVENT_AP_CONNECT, mac_ip4[ap_num_connected-1].mac);
          }
          if (event_accept != 0U) {
            osEventFlagsSet(event_flags_sockets_id, event_accept);
          }
          if (event_recv != 0U) {
            osEventFlagsSet(event_flags_sockets_id, event_recv);
          }
        }

        // Wait async interval unless new event was signaled to process
        repeat = 0U;
        if ((osEventFlagsWait(event_flags_id, EVENT_ASYNC_POLL, osFlagsWaitAny, WIFI_ISM43362_ASYNC_INTERVAL) & (0x80000000U | EVENT_ASYNC_POLL)) == EVENT_ASYNC_POLL) {
          repeat = 1U;
        }
      } while ((poll_async != 0U) || (poll_recv != 0U) || (repeat != 0U));
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
  uint8_t *ptr_resp_buf;
  uint32_t timeout, resp_len, flags, fw_ver;
  uint8_t  u8_arr[4];

  signal_event_fn = cb_event;

  if (driver_initialized != 0U) {
    return ARM_DRIVER_OK;
  }

  ret = ARM_DRIVER_OK;

  // Clear all local variables
  spi_datardy_irq        = 0U;
  sta_connected          = 0U;
  sta_dhcp_enabled       = 1U;
  ap_running             = 0U;

  event_flags_id         = NULL;
  event_flags_sockets_id = NULL;
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
  event_flags_id         = osEventFlagsNew(NULL);
  event_flags_sockets_id = osEventFlagsNew(NULL);
  if ((mutex_id_spi           == NULL) || 
      (mutex_id_sockets       == NULL) || 
      (event_flags_id         == NULL) || 
      (event_flags_sockets_id == NULL)) {
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

  if (ret == ARM_DRIVER_OK) {
    // Reset WiFi Module
    WiFi_ISM43362_Pin_RSTN(true);
    osDelay(50U);
    WiFi_ISM43362_Pin_RSTN(false);
    osDelay(100U);

    // Wait for initial SPI ready and detect if user IRQ is provided for SPI ready detection
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      // Initial SPI ready wait and determine if SPI ready is signaled by IRQ
      WiFi_ISM43362_Pin_SSN(false);
      Wait_us(4U);
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
      Wait_us(3U);

      if (ret == ARM_DRIVER_OK) {
        // Check firmware revision
        memcpy((void *)cmd_buf, (void *)"I?\r", 4); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        if (ret == ARM_DRIVER_OK) {
          // Position pointer 1 character after next ',' (skip ",C")
          ptr_resp_buf = SkipCommas(resp_buf, 1U);
          if (ptr_resp_buf != NULL) {
            ptr_resp_buf++;
            // Extract firmware version
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3]) == 4) {
              fw_ver = ((uint32_t)u8_arr[0] * 1000U) + ((uint32_t)u8_arr[1] * 100U) + ((uint32_t)u8_arr[2] * 10U) + (uint32_t)u8_arr[3];
              if (fw_ver < 3525U) {       // If firmware version is less than 3.5.2.5
//              ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }

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

  if (ret == ARM_DRIVER_OK) {         // If initialization succeeded
    driver_initialized = 1U;
  } else {
    // If something failed during initialization clean-up and release all resources
    WiFi_Uninitialize ();
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
  int32_t ret, close;
  uint8_t i;

  ret = ARM_DRIVER_OK;

  if (driver_initialized != 0U) {
    // Close all sockets
    for (i = 0; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
      close = WiFi_SocketClose(i + WIFI_ISM43362_SOCKETS_NUM);
      if ((close != 0) && (close != ARM_SOCKET_ESOCK)) {
        ret = ARM_DRIVER_ERROR;
      }
      close = WiFi_SocketClose(i);
      if ((close != 0) && (close != ARM_SOCKET_ESOCK)) {
        ret = ARM_DRIVER_ERROR;
      }
    }
  }

  // Release all resources
  if (thread_id_async_poll != NULL) {
    if (osThreadTerminate(thread_id_async_poll) == osOK) {
      thread_id_async_poll = NULL;
    }
  }
  if (event_flags_sockets_id != NULL) {
    if (osEventFlagsDelete(event_flags_sockets_id) == osOK) {
      event_flags_sockets_id = NULL;
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
      (event_flags_sockets_id != NULL) ||
      (event_flags_id         != NULL) ||
      (mutex_id_sockets       != NULL) ||
      (mutex_id_spi           != NULL))){
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    // If uninitialization succeeded
    signal_event_fn    = NULL;
    driver_initialized = 0U;
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

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  if (state != ARM_POWER_OFF) {
    // If requested state is different than ARM_POWER_OFF, for ARM_POWER_OFF state do nothing
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      memcpy((void *)cmd_buf, (void *)"ZP=1,0\r", 8);
      switch (state) {
        case ARM_POWER_LOW:
          cmd_buf[5] = '1';
          break;

        case ARM_POWER_FULL:
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
    } else {
      ret = ARM_DRIVER_ERROR;
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
        uint8_t  exec_cmd;

  if ((data == NULL) || (len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  exec_cmd = 1U;
  ptr_u8_data = (uint8_t *)data;

  switch (option) {
    case ARM_WIFI_MAC:                      // Station Set     MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
      if (len != 6U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else if (sta_connected == 0U) {
        ret = ARM_DRIVER_ERROR;
      } else {
        // Set MAC Address
        sprintf(cmd_buf, "Z4=%02X:%02X:%02X:%02X:%02X:%02X\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3], ptr_u8_data[4], ptr_u8_data[5]);
      }
      break;

    case ARM_WIFI_IP:                       // Station Set     IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network IP Address
        sprintf(cmd_buf, "C6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_SUBNET_MASK:           // Station Set     IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
    case ARM_WIFI_AP_IP_SUBNET_MASK:        // AP      Set     IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network IP Mask
        sprintf(cmd_buf, "C7=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_GATEWAY:               // Station Set     IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
    case ARM_WIFI_AP_IP_GATEWAY:            // AP      Set     IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network Gateway
        sprintf(cmd_buf, "C8=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_DNS1:                  // Station Set     IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
    case ARM_WIFI_AP_IP_DNS1:               // AP      Set     IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network Primary DNS
        sprintf(cmd_buf, "C9=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_DNS2:                  // Station Set     IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
    case ARM_WIFI_AP_IP_DNS2:               // AP      Set     IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network Secondary DNS
        sprintf(cmd_buf, "CA=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_DHCP:                  // Station Set     IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Network DHCP
        memcpy((void *)cmd_buf, (void *)"C4=0\r", 6);
        if (*((uint32_t *)data) != 0) {
          cmd_buf[3] += 1;
        }
        // Store set value to local variable
        sta_dhcp_enabled = __UNALIGNED_UINT32_READ(data);
      }
      break;

    case ARM_WIFI_AP_MAC:                   // AP      Set     MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
      if (len != 6U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else if (ap_running == 0U) {
        ret = ARM_DRIVER_ERROR;
      } else {
        // Set MAC Address
        sprintf(cmd_buf, "Z4=%02X:%02X:%02X:%02X:%02X:%02X\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3], ptr_u8_data[4], ptr_u8_data[5]);
      }
      break;

    case ARM_WIFI_AP_IP:                    // AP      Set     IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
      if (len != 4U) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        // Set Access Point IP Address
        sprintf(cmd_buf, "Z6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_AP_IP_DHCP:               // AP      Set     IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
      // AP DHCP server is enabled by default and cannot be disabled
      if ((len != 4U) || (__UNALIGNED_UINT32_READ(data) != 1U)) {
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        exec_cmd = 0U;
      }
      break;

    case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:    // AP      Set     IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t) [seconds]
      if ((len != 4U) || 
          (__UNALIGNED_UINT32_READ(data) < (30U * 60U)) ||            // If less then 30 minutes
          (__UNALIGNED_UINT32_READ(data) > (254U * 60U * 60U))) {     // If more then 254 hours
        ret = ARM_DRIVER_ERROR_PARAMETER;
      } else {
        sprintf(cmd_buf, "AL=%d\r", __UNALIGNED_UINT32_READ(data)/(60U*60U));
        // Store set value to local variable
        ap_dhcp_lease_time = (__UNALIGNED_UINT32_READ(data)/(60U*60U))*60U*60U;
      }
      break;

    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }
  if ((ret == ARM_DRIVER_OK) && (exec_cmd != 0U)) {   // If command should be sent through SPI
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      osMutexRelease(mutex_id_spi);
    } else {
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
  uint8_t            *ptr_resp_buf;
  uint8_t            *ptr_u8_data;
  ARM_WIFI_MAC_IP4_t *ptr_ap_mac_ip4;
  uint32_t            resp_len;
  int                 int_val;
  uint8_t             i, num, mutex_acquired;

  if ((data == NULL) || (len == NULL) || (*len < 4U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  ret = ARM_DRIVER_OK;
  mutex_acquired = 0U;

  if (option != ARM_WIFI_AP_IP_DHCP_LEASE_TIME) {
    // For all options except ARM_WIFI_AP_IP_DHCP_LEASE_TIME read data through SPI from module is necessary
    // For unsupported commands Network Settings will be read unnecessarily but as unsupported command 
    // is not critical that dummy read is irrelevant
    if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
      mutex_acquired = 1U;
      if ((option == ARM_WIFI_MAC) || (option == ARM_WIFI_AP_MAC)) {
        memcpy((void *)cmd_buf, (void *)"Z5\r", 4);                           // Get MAC Address
      } else if (option == ARM_WIFI_AP_IP) {
        memcpy((void *)cmd_buf, (void *)"A?\r", 4);                           // Show Access Point Settings
      } else if (option == ARM_WIFI_AP_IP_DHCP_TABLE) {
        memcpy((void *)cmd_buf, (void *)"AA\r", 4);                           // Get AP DHCP Cached Address(es)
      } else if (option == ARM_WIFI_RSSI) {
        memcpy((void *)cmd_buf, (void *)"CR\r", 4);                           // Get RSSI of Associated Access Point
      } else {
        memcpy((void *)cmd_buf, (void *)"C?\r", 4);                           // Show Network Settings
      }
      resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    } else {
      ret = ARM_DRIVER_ERROR;
    }
  }

  if (ret == ARM_DRIVER_OK) {
    switch (option) {
      case ARM_WIFI_SSID:                     // Station     Get SSID of connected AP;            data = &ssid,     len<= 33, ssid     (char[32+1]), null-terminated string
        // Extract ssid from response on "C?" command
        if (sscanf((const char *)resp_buf + 2U, "%33[^,]s", (char *)data) != 1) {
          ret = ARM_DRIVER_ERROR;
        }
        break;

      case ARM_WIFI_PASS:                     // Station     Get Password of connected AP;        data = &pass,     len<= 65, pass     (char[64+1]), null-terminated string
        // Skip ssid (1 ',') from response on "C?" command
        ptr_resp_buf = SkipCommas(resp_buf, 1U);
        if (ptr_resp_buf != NULL) {
          if (sscanf((const char *)ptr_resp_buf, "%65[^,]s", (char *)data) != 1) {
            ret = ARM_DRIVER_ERROR;
          }
        } else {
          ret = ARM_DRIVER_ERROR;
        }
        break;

      case ARM_WIFI_SECURITY:                 // Station     Get Security Type of connected AP;   data = &security, len =  4, security (uint32_t): ARM_WIFI_SECURITY_xxx
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password (2 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 2U);
          if (ptr_resp_buf != NULL) {
            if (sscanf((const char *)ptr_resp_buf, "%d", &int_val) == 1) {
              switch (int_val) {
                case 0:
                  int_val = ARM_WIFI_SECURITY_OPEN;
                  break;
                case 1:
                  int_val = ARM_WIFI_SECURITY_WEP;
                  break;
                case 2:
                  int_val = ARM_WIFI_SECURITY_WPA;
                  break;
                case 3:
                  int_val = ARM_WIFI_SECURITY_WPA2;
                  break;
                case 4:
                  int_val = ARM_WIFI_SECURITY_WPA2;
                  break;
                default:
                  int_val = ARM_WIFI_SECURITY_UNKNOWN;
                  break;
              }
              __UNALIGNED_UINT32_WRITE(data, int_val);
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_RSSI:                     // Station     Get RSSI of connected AP;            data = &rssi,     len =  4, rssi     (uint32_t)
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Convert read RSSI in dB to positive value representing RSSI for the driver
          if (sscanf((const char *)resp_buf + 2U, "%d", &int_val) == 1) {
            __UNALIGNED_UINT32_WRITE(data, (uint32_t)(int_val + 255));
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_MAC:                      // Station     Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if (*len != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Extract MAC from response on "Z5" command
          ptr_u8_data = (uint8_t *)data;
          if (sscanf((const char *)resp_buf + 2U, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3], &ptr_u8_data[4], &ptr_u8_data[5]) != 6) {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP:                       // Station     Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password, security, DHCP, IP version (5 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 5U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP_SUBNET_MASK:           // Station     Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
      case ARM_WIFI_AP_IP_SUBNET_MASK:        // AP          Get IPv4 subnet mask;                data = &msk,      len =  4, msk      (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password, security, DHCP, IP version, IP address (6 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 6U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP_GATEWAY:               // Station     Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_GATEWAY:            // AP          Get IPv4 gateway address;            data = &ip,       len =  4, ip       (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask (7 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 7U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP_DNS1:                  // Station     Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS1:               // AP          Get IPv4 primary   DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP 
          // (8 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 8U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP_DNS2:                  // Station     Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
      case ARM_WIFI_AP_IP_DNS2:               // AP          Get IPv4 secondary DNS address;      data = &ip,       len =  4, ip       (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid, password, security, DHCP, IP version, IP address, IP subnet mask, gateway IP, primary DNS IP 
          // (9 ',') from response on "C?" command
          ptr_resp_buf = SkipCommas(resp_buf, 9U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_IP_DHCP:                  // Station     Get IPv4 DHCP client enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Get DHCP mode
          __UNALIGNED_UINT32_WRITE(data, (uint32_t)sta_dhcp_enabled);
        }
        break;

      case ARM_WIFI_AP_MAC:                   // AP          Get MAC;                             data = &mac,      len =  6, mac      (uint8_t[6])
        if (*len != 6U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Extract MAC from response on "Z5" command
          ptr_u8_data = (uint8_t *)data;
          if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3], &ptr_u8_data[4], &ptr_u8_data[5]) != 6) {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_AP_IP:                    // AP          Get IPv4 static/assigned address;    data = &ip,       len =  4, ip       (uint8_t[4])
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Skip ssid (1 ',') from response on "A?" command
          ptr_resp_buf = SkipCommas(resp_buf, 1U);
          if (ptr_resp_buf != NULL) {
            ptr_u8_data = (uint8_t *)data;
            if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3]) != 4) {
              ret = ARM_DRIVER_ERROR;
            }
          } else {
            ret = ARM_DRIVER_ERROR;
          }
        }
        break;

      case ARM_WIFI_AP_IP_DHCP:               // AP          Get IPv4 DHCP server enable/disable; data = &en,       len =  4, en       (uint32_t): 0 = disable, non-zero = enable (default)
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // AP DHCP server cannot be disabled
          __UNALIGNED_UINT32_WRITE(data, 1U);
        }
        break;

      case ARM_WIFI_AP_IP_DHCP_LEASE_TIME:    // AP          Get IPv4 DHCP lease time;            data = &sec,      len =  4, sec      (uint32_t) [seconds]
        if (*len != 4U) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else {
          // Get AP DHCP lease time
          __UNALIGNED_UINT32_WRITE(data, ap_dhcp_lease_time);
        }
        break;

      case ARM_WIFI_AP_IP_DHCP_TABLE:         // AP          Get IPv4 DHCP table;                 data = &mac_ip4[],len = sizeof(mac_ip4[]), mac_ip4 (array of ARM_WIFI_MAC_IP4_t structures)
        if ((((*len) / sizeof(ARM_WIFI_MAC_IP4_t)) == 0U) || (((*len) % sizeof(ARM_WIFI_MAC_IP4_t)) != 0U)) {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        } else if (resp_len < 20U) {
          *len = 0U;
        } else {
          ptr_resp_buf = resp_buf + 2U;
          ptr_ap_mac_ip4  = (ARM_WIFI_MAC_IP4_t *)data;
          num             = 0U;
          for (i = 0U; i < ((*len) / sizeof(ARM_WIFI_MAC_IP4_t)); i++) {
            if (ptr_resp_buf != NULL) {
              // Parse MAC Address
              if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ptr_ap_mac_ip4[i].mac[0], &ptr_ap_mac_ip4[i].mac[1], &ptr_ap_mac_ip4[i].mac[2], &ptr_ap_mac_ip4[i].mac[3], &ptr_ap_mac_ip4[i].mac[4], &ptr_ap_mac_ip4[i].mac[5]) == 6) {
                // Position pointer after next ','
                ptr_resp_buf = SkipCommas(ptr_resp_buf, 1U);
                if (ptr_resp_buf != NULL) {
                  // Parse IP Address
                  if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &ptr_ap_mac_ip4[i].ip4[0], &ptr_ap_mac_ip4[i].ip4[1], &ptr_ap_mac_ip4[i].ip4[2], &ptr_ap_mac_ip4[i].ip4[3]) == 4) {
                    num++;
                    // Position pointer after next ','
                    ptr_resp_buf = SkipCommas(ptr_resp_buf, 1U);
                    if (ptr_resp_buf == NULL) {
                      // If no more entries are available
                      break;
                    }
                  } else {
                    ret = ARM_DRIVER_ERROR;
                  }
                } else {
                  ret = ARM_DRIVER_ERROR;
                }
              } else {
                ret = ARM_DRIVER_ERROR;
              }
            } else {
              ret = ARM_DRIVER_ERROR;
            }
            if (ret != ARM_DRIVER_OK) {
              break;
            }
          }
          if (ret == ARM_DRIVER_OK) {
            *len = num * sizeof(ARM_WIFI_MAC_IP4_t);
          }
        }
        break;

      default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
  }

  if (mutex_acquired != 0U) {
    osMutexRelease(mutex_id_spi);
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;
  int      i, int_val;

  if ((ap_info == NULL) || (max_num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Scan for network access points
    memcpy((void *)cmd_buf, (void *)"F0\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);

    // Extract scan data
    if (ret == ARM_DRIVER_OK) {
      ptr_resp_buf = resp_buf + 2;
      while ((ret == ARM_DRIVER_OK) && (ptr_resp_buf != NULL) && (sscanf((const char *)ptr_resp_buf, "#%d", &i) == 1) && (i > 0) && (i <= (int32_t)max_num)) {
        i--;
        // Position pointer 1 character after next ',' (skip ",C")
        // Parse SSID
        // Skip index (1 ',') and '"'
        ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
        if (ptr_resp_buf != NULL) {
          ptr_resp_buf++;          // Skip '"'
          // Extract SSID
          if (sscanf((const char *)ptr_resp_buf, "%[^\"]", ap_info[i].ssid) != 1) {
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
            if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ap_info[i].bssid[0], &ap_info[i].bssid[1], &ap_info[i].bssid[2], &ap_info[i].bssid[3], &ap_info[i].bssid[4], &ap_info[i].bssid[5]) != 6) {
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
              ap_info[i].rssi = (uint8_t)(int_val + 255);
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
            // Extract security
            if        (memcmp((const void *)ptr_resp_buf, (const void *)"WPA2", 4) == 0) {
              ap_info[i].security = ARM_WIFI_SECURITY_WPA2;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"WPA",  3) == 0) {
              ap_info[i].security = ARM_WIFI_SECURITY_WPA;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"WEP",  3) == 0) {
              ap_info[i].security = ARM_WIFI_SECURITY_WEP;
            } else if (memcmp((const void *)ptr_resp_buf, (const void *)"Open", 4) == 0) {
              ap_info[i].security = ARM_WIFI_SECURITY_OPEN;
            } else {
              ap_info[i].security = ARM_WIFI_SECURITY_UNKNOWN;
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
            if (sscanf((const char *)ptr_resp_buf, "%hhu", &ap_info[i].ch) != 1) {
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
    }
    osMutexRelease(mutex_id_spi);
  } else {
    return ARM_DRIVER_ERROR;
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;

  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL)) || (ch > 13)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

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
      ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "JOIN ");
      if (ptr_resp_buf != NULL) {
        // If message contains "JOIN " string, parse it and extract IP
        // Skip 1 comma
        ptr_resp_buf = SkipCommas((uint8_t const *)ptr_resp_buf, 1U);
        if (ptr_resp_buf != NULL) {
          // Extract IP Address
          if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu", &sta_local_ip[0], &sta_local_ip[1], &sta_local_ip[2], &sta_local_ip[3]) != 4) {
            ret = ARM_DRIVER_ERROR;
          }
        }
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_OK) {
    sta_connected = 1U;
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    if (*pin != NULL) {                 // If pin connection requested
      // Set WPS pin
      if (ret == ARM_DRIVER_OK) {
        sprintf(cmd_buf, "Z7=%s\r", pin); resp_len = sizeof(resp_buf) -1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
      }
      // Activate WPS pin connection
      if (ret == ARM_DRIVER_OK) {
        memcpy((void *)cmd_buf, (void *)"CW=0\r", 6);
      }
    } else {                            // If push-button connection requested
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
      ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "WPS ");
      if (ptr_resp_buf != NULL) {
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
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_OK) {
    sta_connected = 1U;
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

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    // Disconnect from network
    memcpy((void *)cmd_buf, (void *)"CD\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    memset((void *)sta_local_ip, 0, 4);
    sta_connected = 0U;
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

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;
    con = 0;

    // Check connection status
    memcpy((void *)cmd_buf, (void *)"CS\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    if ((ret == ARM_DRIVER_OK) && (resp_len >= 3)) {
      if (resp_buf[2] == '1') {
        con = 1;
      }
    } else {
      ret = ARM_DRIVER_ERROR;
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = con;
    if (con == 1) {
      sta_connected = 1U;
    } else {
      sta_connected = 0U;
    }
  } else {
    ret = 0;
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;

  if ((ssid == NULL) || ((security != ARM_WIFI_SECURITY_OPEN) && (pass == NULL)) || (ch > 13)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Set AP security mode
    memcpy((void *)cmd_buf, (void *)"A1=3\r", 6);
    switch (security) {
      case ARM_WIFI_SECURITY_OPEN:
        cmd_buf[3] = '0';
        break;
      case ARM_WIFI_SECURITY_WPA:
        cmd_buf[3] = '2';
        break;
      case ARM_WIFI_SECURITY_WPA2:
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
      ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "[AP ");
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
    }
    // Dummy read Access Point Settings as WEB Server can fail in which case 
    // module returns "[WEB SVR] Failed to listen on server socket"
    if (ret == ARM_DRIVER_OK) {
      osDelay(300U);    // Allow time for WEB SVR to start
      memcpy((void *)cmd_buf, (void *)"A?\r", 4); resp_len = sizeof(resp_buf) - 1U;
      SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_OK) {
    ap_running = 1U;
    osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);
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

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    // Exit AP direct connect mode
    memcpy((void *)cmd_buf, (void *)"AE\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ap_running = 0U;
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;
    run = 0;

    // Check connection status
    memcpy((void *)cmd_buf, (void *)"A?\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    if (ret == ARM_DRIVER_OK) {
      // Skip ssid, IP address, channel, security, key, AP DHCP, Lease Time (7 ',') from response
      ptr_resp_buf = SkipCommas((uint8_t const *)resp_buf, 7U);
      if (ptr_resp_buf != NULL) {
        if (*ptr_resp_buf == '1') {
          run = 1;
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    } else {
      ret = ARM_DRIVER_ERROR;
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = run;
    if (run == 1) {
      ap_running = 1U;
    } else {
      ap_running = 0U;
    }
  } else {
    ret = 0;
  }

  return ret;
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
      if (protocol != ARM_SOCKET_IPPROTO_UDP) {
        return ARM_SOCKET_EINVAL;
      }
      break;
    case ARM_SOCKET_SOCK_STREAM:
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
      socket_arr[i].protocol = (uint8_t)protocol;
      socket_arr[i].state    = SOCKET_STATE_CREATED;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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
  int32_t ret, i;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (                         (memcmp((void *)ip, "\x00\x00\x00\x00", 4U) != 0)  &&
     ((sta_connected != 0U) && (memcmp((void *)ip, sta_local_ip,       4U) != 0)) &&
     ((ap_running    != 0U) && (memcmp((void *)ip, ap_local_ip,        4U) != 0))) {
    // If IP is different then 0.0.0.0 (accept all) or 
    // if station is active and requested IP is different then station's IP or 
    // if AP is running and requested IP is different then AP's IP
    return ARM_SOCKET_EINVAL;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].state != SOCKET_STATE_CREATED) {
      ret = ARM_SOCKET_EINVAL;
    } else {
      for (i = 0; i < (2 * WIFI_ISM43362_SOCKETS_NUM); i++) {
        if ((                         (memcmp((void *)ip, "\x00\x00\x00\x00", 4U) == 0)   ||
            ((sta_connected != 0U) && (memcmp((void *)ip, sta_local_ip,       4U) == 0))  ||
            ((ap_running    != 0U) && (memcmp((void *)ip, ap_local_ip,        4U) == 0))) &&
             (socket_arr[i].local_port == port)) {
          // If IP and port is already used by another socket
          ret = ARM_SOCKET_EADDRINUSE;
          break;
        }
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

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (backlog != 1) {
    // Only backlog 1 is supported
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].protocol != ARM_SOCKET_SOCK_STREAM) {
      ret = ARM_SOCKET_ENOTSUP;
    } else if (socket_arr[socket].state != SOCKET_STATE_BOUND) {
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
          ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, socket_arr[socket].local_port, NULL, 0U, 1U, 1U);
          if (ret == 0) {
            socket_arr[socket].server = 1U;
          }
          osMutexRelease(mutex_id_spi);
        }
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
  uint32_t flags, timeout;
  uint8_t  virtual_socket, non_blocking, trigger_async_polling;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  virtual_socket = socket + WIFI_ISM43362_SOCKETS_NUM;
  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].protocol != ARM_SOCKET_SOCK_STREAM) {
      ret = ARM_SOCKET_ENOTSUP;
    } else if ((socket_arr[socket].state != SOCKET_STATE_LISTENING) && (socket_arr[socket].state != SOCKET_STATE_ACCEPTING)) {
      ret = ARM_SOCKET_EINVAL;
    } else if (socket_arr[virtual_socket].state != SOCKET_STATE_FREE) {
      // ISM43362 module limitation: only one socket can be processed at a time
      ret = ARM_SOCKET_ERROR;
    }

    if (ret == 0) {
      non_blocking = socket_arr[socket].non_blocking;
      trigger_async_polling = 0U;
      if (socket_arr[socket].state != SOCKET_STATE_ACCEPTING) {
        trigger_async_polling = 1U;
        socket_arr[socket].state = SOCKET_STATE_ACCEPTING;
      }
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
    flags = osEventFlagsWait(event_flags_sockets_id, (0x10001U << socket), osFlagsWaitAny, timeout);
    if ((flags & 0x80000000UL) != 0U) {                 // If error
      if (flags == osFlagsErrorTimeout) {
        ret = ARM_SOCKET_EAGAIN;
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    } else if ((flags & (0x10000U << socket)) != 0U) {  // If abort signaled from SocketClose
      ret = ARM_SOCKET_ERROR;
    }
    // If ret == 0, then accept was signaled from async thread

    if (ret == 0) {
      if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
        // If socket accept succeeded, copy all variables to new (virtual) socket
        memcpy((void *)&socket_arr[virtual_socket], (void *)&socket_arr[socket], sizeof(socket_t));
        socket_arr[socket].state         = SOCKET_STATE_ACCEPTED;
        socket_arr[virtual_socket].state = SOCKET_STATE_CONNECTED;
        osMutexRelease(mutex_id_sockets);
        if ((ip != NULL) && (ip_len != NULL) && (*ip_len >= 4U)) {
          ip[0] = socket_arr[virtual_socket].remote_ip[0];
          ip[1] = socket_arr[virtual_socket].remote_ip[1];
          ip[2] = socket_arr[virtual_socket].remote_ip[2];
          ip[3] = socket_arr[virtual_socket].remote_ip[3];
          *ip_len = 4U;
        }
      } else {
        ret = ARM_SOCKET_ERROR;
      }
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
  int32_t ret;

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }
  // This module does not support setting of local port it only supports 
  // 0 value and generates the local port randomly

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    switch (socket_arr[socket].state) {
      case SOCKET_STATE_CONNECTED:
        ret = ARM_SOCKET_EISCONN;
        break;
      case SOCKET_STATE_CREATED:
      case SOCKET_STATE_BOUND:
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }

    if (ret == 0) {
      if (socket_arr[socket].state != SOCKET_STATE_CONNECTING) {      // If first call of connect
        if ((socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].client == 0U)) {
          if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
            if (socket_arr[socket].client == 0U) {
              ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, ip, port, 1U, 0U);
              if (ret == 0) {
                socket_arr[socket].client = 1U;
                socket_arr[socket].state = SOCKET_STATE_CONNECTING;
                if (socket_arr[socket].non_blocking != 0U) {          // If non-blocking mode
                  ret = ARM_SOCKET_EINPROGRESS;
                }
              }
              osMutexRelease(mutex_id_spi);
            }
          } else {
            ret = ARM_SOCKET_ETIMEDOUT;
          }
        }
      }
      // Only for first call of connect in non-blocking mode we return ARM_SOCKET_EINPROGRESS, 
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
  uint8_t  long_timeout, trigger_async_polling;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].state == SOCKET_STATE_FREE) {
      ret = ARM_DRIVER_ERROR;
    } else if ((socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
      ret = ARM_SOCKET_ENOTCONN;
    }

    hw_socket = socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }
    len_tot_rece = 0;

    if (ret == 0) {
      long_timeout = 0U;
      if (socket_arr[socket].non_blocking != 0U) {            // If non-blocking
        timeout = 1U;
      } else {                                                // If blocking
        if ((socket_arr[socket].recv_timeout == 0U) ||        // If infinite or longer than async interval
            (socket_arr[socket].recv_timeout > WIFI_ISM43362_ASYNC_INTERVAL)) {
          timeout = 1U;
          long_timeout = 1U;
        } else if (socket_arr[socket].recv_timeout <= WIFI_ISM43362_ASYNC_INTERVAL) {
                                                              // If short blocking, set timeout to requested
          timeout = socket_arr[socket].recv_timeout;
        }
      }

      if ((ret == 0) && (socket_arr[socket].data_to_recv == NULL)) {
        // Setup receive parameters on socket of the module
        // Execute functionality on the module through SPI commands
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          if (ret == 0) {
            if (socket_arr[socket].protocol == ARM_SOCKET_SOCK_DGRAM) {     // If UDP socket
              if (socket_arr[socket].state == SOCKET_STATE_BOUND) {         // Socket bound
                if (socket_arr[socket].server == 0U) {                      // UDP server not running on this socket
                  ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, socket_arr[socket].local_port, NULL, 0U, 1U, 1U);
                  if (ret == 0) {
                    socket_arr[socket].server = 1U;
                  }
                }
              } else {                                                      // Socket not bound
                if (socket_arr[socket].client == 0U) {                      // UDP client not running on this socket
                  ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, (const uint8_t *)ip, *port, 1U, 0U);
                  if (ret == 0) {
                    socket_arr[socket].client = 1U;
                  }
                }
              }
            } else {
              // Set communication socket number
              sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
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
          ret = ARM_SOCKET_ERROR;
        }
      }

      if (ret == 0) {
        if (long_timeout != 0U) {                                             // If long blocking receive request
          trigger_async_polling = 0U;
          if (socket_arr[socket].data_to_recv == NULL) {                      // If long blocking receive is not yet active
            // Store information for receiving to be done from async thread
            socket_arr[socket].data_to_recv   = buf;
            socket_arr[socket].len_to_recv    = len;
            socket_arr[socket].len_recv       = 0U;
            if (socket_arr[socket].recv_timeout == 0U) {
              // 0 means infinite
              socket_arr[socket].recv_time_left = 0xFFFFFFFFU;
            } else {
              // other values increment for 1 ms to catch last async interval also
              socket_arr[socket].recv_time_left = socket_arr[socket].recv_timeout + 1U;
            }
            trigger_async_polling = 1U;
          }
        } else {                                                              // If short blocking receive or non-blocking receive request
          // Execute functionality on the module through SPI commands
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
                  break;
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
            ret = ARM_SOCKET_ERROR;
          }
        }
      }
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
  }

  if ((ret == 0) && (long_timeout != 0U)) {
    if (trigger_async_polling != 0U) {
      // If requested activate asynchronous polling
      osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);
    }
    // Wait for reception or timeout or disconnect signaled from asynchronous thread
    flags = osEventFlagsWait(event_flags_sockets_id, (0x00010101U << socket), osFlagsWaitAny, socket_arr[socket].recv_time_left);
    if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
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

  if (ret == 0) {
    if (len_tot_rece != 0) {
      ret = len_tot_rece;
    } else {
      // If timed out in blocking mode or no data available in non-blocking mode
      ret = ARM_SOCKET_EAGAIN;
    }
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

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ret == 0) && ((buf == NULL) || (len == 0U))) {
    ret = ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].state == SOCKET_STATE_FREE) {
      ret = ARM_SOCKET_EINVAL;
    } else if ((socket_arr[socket].protocol == ARM_SOCKET_SOCK_STREAM) && (socket_arr[socket].state != SOCKET_STATE_CONNECTED)) {
      ret = ARM_SOCKET_ENOTCONN;
    }

    hw_socket = socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }
    len_tot_sent = 0;

    // Execute functionality on the module through SPI commands
    if (ret == 0) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        if (socket_arr[socket].protocol == ARM_SOCKET_SOCK_DGRAM) {     // If UDP socket
          if (socket_arr[socket].state == SOCKET_STATE_BOUND) {         // Socket bound
            if (socket_arr[socket].server == 0U) {                      // UDP server not running on this socket
              ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, socket_arr[socket].local_port, NULL, 0U, 1U, 1U);
              if (ret == 0) {
                socket_arr[socket].server = 1U;
              }
            }
          } else {                                                      // Socket not bound
            if (socket_arr[socket].client == 0U) {                      // UDP client not running on this socket
              ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, (const uint8_t *)ip, port, 1U, 0U);
              if (ret == 0) {
                socket_arr[socket].client = 1U;
              }
            }
          }
        } else {
          // Set communication socket number
          sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
            ret = ARM_SOCKET_ERROR;
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
        ret = ARM_SOCKET_ERROR;
      }
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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
  uint8_t *ptr_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  uint16_t u16_val;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U) || (port == NULL)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    switch (socket_arr[socket].state) {
      case SOCKET_STATE_CONNECTED:
      case SOCKET_STATE_BOUND:
        break;
      default:
        ret = ARM_SOCKET_EINVAL;
        break;
    }

    hw_socket = socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }

    // Execute functionality on the module through SPI commands
    if (ret == 0) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        // Set communication socket number
        sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
          ret = ARM_SOCKET_ERROR;
        }

        if (ret == 0) {
          // Show Transport Settings
          memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
            // Skip protocol (1 ',') from response
            ptr_resp_buf = SkipCommas(resp_buf, 1U);
            if (ptr_resp_buf != NULL) {
              if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu,%hu", &ip[0], &ip[1], &ip[2], &ip[3], &u16_val) == 5) {
                *port = u16_val;
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
        osMutexRelease(mutex_id_spi);
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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
  uint8_t *ptr_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  uint16_t u16_val;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U) || (port == NULL)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].state != SOCKET_STATE_CONNECTED) {
      ret = ARM_SOCKET_ENOTCONN;
    }

    hw_socket = socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }

    // Execute functionality on the module through SPI commands
    if (ret == 0) {
      if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
        sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
          ret = ARM_SOCKET_ERROR;
        }

        if (ret == 0) {
          // Show Transport Settings
          memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
            // Skip protocol, local IP, local port (3 ',') from response
            ptr_resp_buf = SkipCommas(resp_buf, 3U);
            if (ptr_resp_buf != NULL) {
              if (sscanf((const char *)ptr_resp_buf, "%hhu.%hhu.%hhu.%hhu,%hu", &ip[0], &ip[1], &ip[2], &ip[3], &u16_val) == 5) {
                *port = u16_val;
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
        osMutexRelease(mutex_id_spi);
      } else {
        ret = ARM_SOCKET_ERROR;
      }
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_len == NULL) || (*opt_len < 4U)) {
    return ARM_SOCKET_EINVAL;
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
        ret = ARM_SOCKET_ENOTSUP;
        break;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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
  uint32_t val, resp_len;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_len == NULL) || (opt_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }

  val = __UNALIGNED_UINT32_READ(opt_val);
  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;
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
        if (driver_initialized == 0U) {
          ret = ARM_DRIVER_ERROR;
        }
        if (ret == 0) {
          if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
            hw_socket = socket;
            if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
              hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
            }
            sprintf(cmd_buf, "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            if (ret == 0) {
              if (val == 0U) {
                memcpy((void *)cmd_buf, (void *)"PK=0,0\r", 7); resp_len = sizeof(resp_buf) - 1U;
              } else {
                sprintf(cmd_buf, "PK=1,%d\r", val); resp_len = sizeof(resp_buf) - 1U;
              }
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                socket_arr[socket].keepalive = val;
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
        ret = ARM_SOCKET_ENOTSUP;
        break;
    }
    osMutexRelease(mutex_id_sockets);
  } else {
    ret = ARM_SOCKET_ERROR;
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
  uint8_t hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

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
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          if ((ret == 0) && (socket_arr[socket].client == 1U)) {
            ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, NULL, 0U, 0U, 0U);
            if (ret == 0) {
              socket_arr[socket].client = 0U;
            }
          }
          if ((ret == 0) && (socket_arr[socket].server == 1U)) {
            ret = SPI_StartStopTransportServerClient (socket, socket_arr[socket].protocol, 0U, NULL, 0U, 0U, 1U);
            if (ret == 0) {
              socket_arr[socket].server = 0U;
            }
          }
          osMutexRelease(mutex_id_spi);
        } else {
          ret = ARM_SOCKET_EAGAIN;
        }
      }

      if (ret == 0) {
        if ((event_flags_sockets_id != NULL) && (socket_arr[socket].state == SOCKET_STATE_ACCEPTING)) {
          // If socket is waiting for flag in accept, send flag to terminate accept
          osEventFlagsSet(event_flags_sockets_id, (0x10000U << socket));
        }
        if ((event_flags_sockets_id != NULL) && (socket_arr[socket].data_to_recv != NULL) && (socket_arr[socket].recv_time_left != 0U)) {
          // If socket is waiting for reception but has timed-out send event
          osEventFlagsSet(event_flags_sockets_id, ((1U << 16) << socket));
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

  if (af != ARM_SOCKET_AF_INET) {
    return ARM_SOCKET_ENOTSUP;
  }
  if ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (strlen(name) > 64) {
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
    sprintf(cmd_buf, "D0=%s\r", name); resp_len = sizeof(resp_buf) - 1U;
    if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
      if (resp_len > 0) {
        // Check that "Host not found " string is not present in response
        if (strstr((const char *)resp_buf, "Host not found ") == NULL) {
          // Parse IP Address
          if (sscanf((const char *)resp_buf + 2, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]) != 4) {
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
    ret = ARM_SOCKET_ETIMEDOUT;
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
