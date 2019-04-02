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
 * $Date:        1. April 2019
 * $Revision:    V1.0 (beta)
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

extern void    WiFi_ISM43362_Pin_Initialize   (void);
extern void    WiFi_ISM43362_Pin_Uninitialize (void);
extern void    WiFi_ISM43362_Pin_RSTN         (uint8_t rstn);
extern void    WiFi_ISM43362_Pin_SSN          (uint8_t ssn);
extern uint8_t WiFi_ISM43362_Pin_DATARDY      (void);

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

#define ARM_WIFI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)        // Driver version

// Driver Version
static const ARM_DRIVER_VERSION driver_version = { ARM_WIFI_API_VERSION, ARM_WIFI_DRV_VERSION };

// Driver Capabilities
static const ARM_WIFI_CAPABILITIES driver_capabilities = { 
  1U,                                   // Mode: Station supported
  1U,                                   // Mode: Access Point supported
  0U,                                   // Mode: Station and Access Point not supported
  0U,                                   // Mode: Ad-hoc not supported
  1U,                                   // WiFi Protected Setup (WPS) for Station supported
  0U,                                   // WiFi Protected Setup (WPS) for Access Point not supported
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
  uint16_t local_port;                  // Local       port number
  uint16_t remote_port;                 // Remote host port number
  uint8_t  remote_ip[4];                // Remote host IP
                                        // Module specific socket variables
  void    *data_to_recv;                // Pointer to where data should be received
  uint32_t len_to_recv;                 // Number of bytes to receive
  uint32_t len_recv;                    // Number of bytes received
  uint32_t recv_time_left;              // Receive Time left until Timeout
  uint8_t  client;                      // Socket client running
  uint8_t  server;                      // Socket server running
  uint8_t  bound;                       // Socket bound (server)
  uint8_t  reserved1;                   // Reserved
} socket_t;

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
#define EVENT_ACCEPT                    (1U << 3)
#define EVENT_SOCKET                    (1U << 4)

// Local macros
#define SPI_Driver_(n)                  Driver_SPI##n
#define SPI_Driver(n)                   SPI_Driver_(n)
extern ARM_DRIVER_SPI                   SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM);
#define ptrSPI                        (&SPI_Driver(WIFI_ISM43362_SPI_DRV_NUM))

// Mutex responsible for protecting SPI media access
const osMutexAttr_t mutex_spi_attr = {
  "Mutex_SPI",                          // Mutex name
  osMutexPrioInherit,                   // attr_bits
  NULL,                                 // Memory for control block
  0U                                    // Size for control block
};

// Mutex responsible for protecting socket local variables access
const osMutexAttr_t mutex_socket_attr = {
  "Mutex_Socket",                       // Mutex name
  osMutexPrioInherit,                   // attr_bits
  NULL,                                 // Memory for control block
  0U                                    // Size for control block
};

// Thread for polling and processing asynchronous messages
const osThreadAttr_t thread_async_poll_attr = {
  .name     = "Thread_Async_Poll",      // Thread name
  .priority = WIFI_ISM43362_ASYNC_PRIORITY
};


// Local variables and structures
static uint8_t                          driver_initialized = 0U;

static osEventFlagsId_t                 event_flags_id;
static osEventFlagsId_t                 event_flags_sockets_id;
static osMutexId_t                      mutex_id_spi;
static osMutexId_t                      mutex_id_sockets;
static osThreadId_t                     thread_id_async_poll;

static ARM_WIFI_SignalEvent_t           signal_event_fn;

static uint8_t                          spi_datardy_irq;

static uint8_t                          oper_mode;
static uint8_t                          sta_connected;
static uint8_t                          ap_running;

static char                             cmd_buf [64   + 1] __ALIGNED(4);
static uint8_t                          resp_buf[1210 + 1] __ALIGNED(4);

static uint32_t                         sta_lp_time;
static uint8_t                          sta_dhcp_client;
static uint8_t                          sta_config;
static uint8_t                          sta_config_wps_method;
static uint8_t                          ap_beacon_interval;
static uint8_t                          ap_config;
static uint8_t                          ap_num_connected;
static uint32_t                         ap_dhcp_lease_time;

static uint8_t                          sta_local_ip  [4];
static uint8_t                          ap_local_ip   [4];
static uint8_t                          ap_mac     [8][6];

static socket_t                         socket_arr[2 * WIFI_ISM43362_SOCKETS_NUM];

// Function prototypes
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

  sta_dhcp_client       = 1U;           // DHCP client is enabled by default

  signal_event_fn       = NULL;

  spi_datardy_irq       = 0U;

  oper_mode             = ARM_WIFI_MODE_NONE;
  sta_connected         = 0U;
  ap_running            = 0U;

  memset((void *)cmd_buf , 0, sizeof(cmd_buf));
  memset((void *)resp_buf, 0, sizeof(resp_buf));

  sta_lp_time           = 0U;
  sta_config            = 0U;
  sta_config_wps_method = 0U;
  ap_beacon_interval    = 0U;
  ap_config             = 0U;
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
  \fn            uint8_t *SkipCommas (uint8_t const *ptr, uint8_t num)
  \brief         Skip requested number of commas in character buffer.
  \param[in]     ptr      Pointer to character buffer
  \param[in]     num      Number of commas to skip
  \return        pointer to first character after requested number of commas, NULL in case of failure
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
  \fn            void SPI_WaitReady (uint32_t timeout)
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
  \fn            void SPI_WaitTransferDone (uint32_t timeout)
  \brief         Wait for SPI transfer to finish.
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        SPI transfer finished state
                   - 0: SPI transfer finished successfully
                   - 1: SPI transfer did not finish successfully
*/
static uint8_t SPI_WaitTransferDone (uint32_t timeout) {
  return (uint8_t)((osEventFlagsWait(event_flags_id, EVENT_SPI_XFER_DONE, osFlagsWaitAll, timeout) & EVENT_SPI_XFER_DONE) == EVENT_SPI_XFER_DONE);
}

/**
  \fn            int32_t SPI_AT_SendCommandAndData (const char *cmd, const uint8_t *data, uint32_t data_len, uint32_t timeout)
  \brief         Send AT command and data.
  \param[in]     cmd      Pointer to command null-terminated string
  \param[in]     data     Pointer to data to be sent after command
  \param[in]     data_len Number of bytes of data to be sent
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        execution status
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
  \fn            int32_t SPI_AT_ReceiveData (uint8_t *data, uint32_t data_len, uint32_t timeout)
  \brief         Receive data.
  \param[out]    data     Pointer to memory where data will be received
  \param[in]     data_len Maximum number of bytes of data that memory is prepared to receive
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        number of bytes received or error code
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
  \fn            int32_t SPI_AT_SendCommandReceiveResponse (const char *cmd, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief         Send AT command, receive response and check that response is OK.
  \param[in]     cmd      Pointer to command null-terminated string
  \param[out]    resp     Buffer where response will be received
  \param[in,out] resp_len Pointer to a number
                   - input: number of bytes that can be received in response
                   - output: number of bytes actually received in response
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        execution status
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
  \fn            int32_t SPI_AT_SendCommandAndDataReceiveResponse (const char *cmd, const uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief         Send AT command and data, receive response and check that response is OK.
  \param[in]     cmd      Pointer to command null-terminated string
  \param[in]     data     Pointer to data to be sent after command
  \param[in]     data_len Number of bytes of data to be sent
  \param[out]    resp     Buffer where response will be received
  \param[in,     
         out]    resp_len Pointer to a number
                   - input: number of bytes that can be received in response
                   - output: number of bytes actually received in response
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        number of data bytes sent or error code
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
  \fn            int32_t SPI_AT_SendCommandReceiveDataAndResponse (const char *cmd, uint8_t *data, uint32_t data_len, uint8_t *resp, uint32_t *resp_len, uint32_t timeout)
  \brief         Send AT command, receive data and response and check that response is OK.
  \param[in]     cmd      Pointer to command null-terminated string
  \param[in]     data     Pointer to memory where data will be received
  \param[in]     data_len Maximum number of bytes of data that memory is prepared to receive
  \param[out]    resp     Buffer where response will be received
  \param[in,     
         out]    resp_len Pointer to a number
                   - input: number of bytes that can be received in response
                   - output: number of bytes actually received in response
  \param[in]     timeout  Timeout in milliseconds (0 = no timeout)
  \return        number of bytes received or error code
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
  \fn            int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server)
  \brief         Start or stop transport server or client.
  \param[in]     socket       Socket identification number
  \param[in]     protocol     Protocol (ARM_SOCKET_SOCK_DGRAM or ARM_SOCKET_SOCK_STREAM)
  \param[in]     ip           Pointer to remote destination IP address
  \param[in]     local_port   Local port number
  \param[in]     remote_ip    Pointer to remote IP4 address
  \param[in]     remote_port  Remote port number
  \param[in]     start        Start/stop request (0 = stop, 1 = start)
  \param[in]     server       Server/client (0 = client, 1 = server)
  \return        status information
                   - 0                            : Operation successful
                   - ARM_SOCKET_ERROR             : Unspecified error
*/
static int32_t SPI_StartStopTransportServerClient (int32_t socket, uint8_t protocol, uint16_t local_port, const uint8_t *remote_ip, uint16_t remote_port, uint8_t start, uint8_t server) {
  int32_t  ret;
  uint32_t resp_len;

  ret = 0;

  // Set communication socket number
  snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", socket); resp_len = sizeof(resp_buf) - 1U;
  if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
    ret = ARM_SOCKET_ERROR;
  }

  // Select transport protocol
  if (ret == 0) {
    memcpy((void *)cmd_buf, (void *)"P1=1\r", 6);
    if (protocol == ARM_SOCKET_SOCK_STREAM) {
      cmd_buf[3] = '0';
    }
    resp_len = sizeof(resp_buf) - 1U;
    if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
      ret = ARM_SOCKET_ERROR;
    }
  }

  if (ret == 0) {
    if (server != 0U) {
      // Set transport local port number
      snprintf(cmd_buf, sizeof(cmd_buf), "P2=%d\r", local_port); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
        ret = ARM_SOCKET_ERROR;
      }
    } else {
      // Set transport remote host IP address
      snprintf(cmd_buf, sizeof(cmd_buf), "P3=%d.%d.%d.%d\r", remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
        ret = ARM_SOCKET_ERROR;
      }
      // Set transport remote port number
      snprintf(cmd_buf, sizeof(cmd_buf), "P4=%d\r", remote_port); resp_len = sizeof(resp_buf) - 1U;
      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
        ret = ARM_SOCKET_ERROR;
      }
    }
  }

  // Server/Client Start/Stop
  if (ret == 0) {
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

                          // Read remote port by P? command as it is not available in asynchronous response
                          // Select socket
                          snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", i); resp_len = sizeof(resp_buf) - 1U;
                          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                            // Show Transport Settings
                            memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
                            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                              // Skip protocol, client IP, local port and host IP (4 ',') from response
                              ptr_resp_buf = SkipCommas(resp_buf, 4U);
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
                        event_accept |= 1U << i;
                      }
                    }
                  }

                  // If message is asynchronous Assign
                  ptr_resp_buf = (uint8_t *)strstr((const char *)resp_buf, "Assigned ");
                  if (ptr_resp_buf != NULL) {
                    if (ptr_resp_buf != NULL) {
                      // If message contains "Assigned " string, parse it and extract MAC
                      ptr_resp_buf += 9U;
                      // Parse MAC Address
                      if (sscanf((const char *)ptr_resp_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &u8_arr[0], &u8_arr[1], &u8_arr[2], &u8_arr[3], &u8_arr[4], &u8_arr[5]) == 6) {
                        // Check if MAC already exists in mac_ip4 array, if it does ignore it, otherwise add it to array
                        for (i = 0; i < 8; i++) {
                          if (memcmp((void *)&ap_mac[i][0], (void *)u8_arr, 6) == 0) {
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
                      snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
                      if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
                        len_req = socket_arr[i].len_to_recv - socket_arr[i].len_recv;
                        if (len_req > 1200U) {
                          len_req = 1200U;
                        }
                        // Set read data packet size
                        snprintf(cmd_buf, sizeof(cmd_buf), "R1=%d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
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
            signal_event_fn(ARM_WIFI_EVENT_AP_CONNECT, (void *)&ap_mac[ap_num_connected-1][0]);
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
  int32_t  ret;
  uint32_t timeout, flags;

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
  event_flags_sockets_id = osEventFlagsNew(NULL);
  if ((mutex_id_spi           == NULL) || 
      (mutex_id_sockets       == NULL) || 
      (event_flags_id         == NULL) || 
      (event_flags_sockets_id == NULL)) {
    // If any of mutex or flag creation failed
    ret = ARM_DRIVER_ERROR;
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
      if (timeout == 0U) {              // If DATARDY pin did not signal ready in WIFI_ISM43362_SPI_TIMEOUT seconds
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
          } else {                      // If transfer timed out
            ret = ARM_DRIVER_ERROR;
          }
        }
      }
      WiFi_ISM43362_Pin_SSN(false);
      Wait_us(3U);

      if (osMutexRelease(mutex_id_spi) != osOK) {       // If SPI mutex release has failed
        ret = ARM_DRIVER_ERROR;
      }
    } else {                            // If SPI interface is not accessible (locked by another thread)
      ret = ARM_DRIVER_ERROR;
    }
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
  int32_t ret;
  uint8_t i;

  if (driver_initialized == 0U) {       // If driver is already uninitialized
    return ARM_DRIVER_OK;
  }

  ret = ARM_DRIVER_OK;

  // Close all open sockets
  for (i = 0U; i < WIFI_ISM43362_SOCKETS_NUM; i++) {
    if (WiFi_SocketClose(i + WIFI_ISM43362_SOCKETS_NUM) != 0) {
      ret = ARM_DRIVER_ERROR;
      break;
    }
    if (WiFi_SocketClose(i) != 0) {
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
  uint32_t resp_len;

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
        if ((sta_lp_time != 0U) && (oper_mode == ARM_WIFI_MODE_STATION)) {
          snprintf(cmd_buf, sizeof(cmd_buf), "ZP=6,%d\r", sta_lp_time);
        } else if (ap_beacon_interval != 0U) {
          snprintf(cmd_buf, sizeof(cmd_buf), "ZP=2,%d\r", ap_beacon_interval);
        } else {
          memcpy((void *)cmd_buf, (void *)"ZP=1,1\r", 8);
        }
        break;

      case ARM_POWER_FULL:
        memcpy((void *)cmd_buf, (void *)"ZP=1,0\r", 8);
        break;

      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
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
  uint32_t resp_len, copy_len;

  if ((module_info == NULL) || (max_len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  // Show Applications Information
  memcpy((void *)cmd_buf, (void *)"I?\r", 4); resp_len = sizeof(resp_buf) - 1U;
  ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
  if (ret == ARM_DRIVER_OK) {
    if (resp_len > 10U) {
      // Remove starting "\r\n" and trailing "\r\nOK\r\n> "
      resp_len -= 10U;
    }
    copy_len = resp_len + 1U;
    if (copy_len > max_len) {
      copy_len = max_len;
    }
    if (copy_len >= 1) {
      memcpy ((void *)module_info, (void *)(resp_buf + 2U), copy_len - 1U);
    }
    module_info[copy_len] = 0;
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
        uint32_t resp_len;
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
  ptr_u8_data = (uint8_t *)data;

  switch (option) {
    case ARM_WIFI_LP_TIMER:                 // Station    Set low-power deep-sleep time;              data = &time,     len =  4, uint32_t [seconds]: 0 = disable (default)
      u32 = *((uint32_t *)data);
      if (interface == 0U) {                // For Station interface
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
      u32 = *((uint32_t *)data);
      if (interface == 1U) {                // For AP interface
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
        snprintf(cmd_buf, sizeof(cmd_buf), "Z4=%02X:%02X:%02X:%02X:%02X:%02X\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3], ptr_u8_data[4], ptr_u8_data[5]);
      }
      break;

    case ARM_WIFI_IP:                       // Station/AP Set IPv4 static/assigned address;           data = &ip,       len =  4, uint8_t[4]
      if (interface == 0U) {                // For Station interface
        snprintf(cmd_buf, sizeof(cmd_buf), "C6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      } else {                              // For AP interface
        snprintf(cmd_buf, sizeof(cmd_buf), "Z6=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      }
      break;

    case ARM_WIFI_IP_SUBNET_MASK:           // Station/AP Set IPv4 subnet mask;                       data = &mask,     len =  4, uint8_t[4]
      snprintf(cmd_buf, sizeof(cmd_buf), "C7=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      break;

    case ARM_WIFI_IP_GATEWAY:               // Station/AP Set IPv4 gateway address;                   data = &ip,       len =  4, uint8_t[4]
      snprintf(cmd_buf, sizeof(cmd_buf), "C8=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      break;

    case ARM_WIFI_IP_DNS1:                  // Station/AP Set IPv4 primary   DNS address;             data = &ip,       len =  4, uint8_t[4]
      snprintf(cmd_buf, sizeof(cmd_buf), "C9=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      break;

    case ARM_WIFI_IP_DNS2:                  // Station/AP Set IPv4 secondary DNS address;             data = &ip,       len =  4, uint8_t[4]
      snprintf(cmd_buf, sizeof(cmd_buf), "CA=%d.%d.%d.%d\r", ptr_u8_data[0], ptr_u8_data[1], ptr_u8_data[2], ptr_u8_data[3]);
      break;

    case ARM_WIFI_IP_DHCP:                  // Station/AP Set IPv4 DHCP client/server enable/disable; data = &dhcp,     len =  4, uint32_t: 0 = disable, non-zero = enable (default)
      u32 = *((uint32_t *)data);
      if (interface == 0U) {                // For Station interface
        memcpy((void *)cmd_buf, (void *)"C4=0\r", 6);
        if (u32 != 0) {
          cmd_buf[3] += 1;
        }
        // Store set value to local variable
        sta_dhcp_client = u32;
      } else {                              // For AP interface
        if (u32 == 0U) {
          // DHCP server cannot be disabled
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        exec_cmd = 0U;  // Do not execute SPI command
      }
      break;

    case ARM_WIFI_IP_DHCP_LEASE_TIME:       //         AP Set IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
      u32 = *((uint32_t *)data);
      if (interface == 1U) {    // AP
        if ((u32 >= (30U * 60U)) &&         // If more then 30 minutes
            (u32 <= (254U * 60U * 60U))) {  // If less then 254 hours
          snprintf(cmd_buf, sizeof(cmd_buf), "AL=%d\r", u32/(60U*60U));
          // Store set value to local variable
          ap_dhcp_lease_time = (u32/(60U*60U))*60U*60U;
        } else {
          ret = ARM_DRIVER_ERROR_PARAMETER;
        }
      } else {
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    default:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  if ((ret == ARM_DRIVER_OK) && (exec_cmd != 0U)) {     // If command should be sent through SPI
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
  uint8_t  *ptr_resp_buf;
  uint8_t  *ptr_u8_data;
  uint32_t  resp_len;
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
        memcpy((void *)cmd_buf, (void *)"Z5\r", 4);     // Get MAC Address
      } else if ((interface == 1U) && (option == ARM_WIFI_IP)) {
        memcpy((void *)cmd_buf, (void *)"A?\r", 4);     // Show Access Point Settings
      } else {
        memcpy((void *)cmd_buf, (void *)"C?\r", 4);     // Show Network Settings
      }
      resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
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
      if (interface == 0U) {                // For Station interface
        *((uint32_t *)data) = sta_lp_time;
        *len = 4U;
      } else {
        // Not supported for AP interface
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_WIFI_BEACON:                   //         AP Get beacon interval;                        data = &interval, len =  4, uint32_t [ms]
      if (interface == 1U) {                // For AP interface
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
        if (sscanf((const char *)resp_buf + 2U, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ptr_u8_data[0], &ptr_u8_data[1], &ptr_u8_data[2], &ptr_u8_data[3], &ptr_u8_data[4], &ptr_u8_data[5]) == 6) {
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
        ptr_resp_buf = SkipCommas(resp_buf, 5U);
      } else {                              // For AP interface
        // Skip ssid (1 ',') from response on "A?" command
        ptr_resp_buf = SkipCommas(resp_buf, 1U);
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
      ptr_resp_buf = SkipCommas(resp_buf, 6U);
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
      ptr_resp_buf = SkipCommas(resp_buf, 7U);
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
      ptr_resp_buf = SkipCommas(resp_buf, 8U);
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
      ptr_resp_buf = SkipCommas(resp_buf, 9U);
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
      if (interface == 0U) {                // For Station interface
        *((uint32_t *)data) = sta_dhcp_client;
      } else {                              // For AP interface
        *((uint32_t *)data) = 1U;
      }
      *len = 4U;
      break;

    case ARM_WIFI_IP_DHCP_LEASE_TIME:       //         AP Get IPv4 DHCP lease time;                   data = &time,     len =  4, uint32_t [seconds]
      if (interface == 1U) {                // For AP interface
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
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;
  int      i, int_val;

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
    }
    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_OK) {
    ret = i + 1;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Configure (uint32_t interface, ARM_WIFI_CONFIG_t *config)
  \brief         Configure Network Parameters.
  \param[in]     interface Interface (0 = Station, 1 = Access Point)
  \param[in]     config    Pointer to ARM_WIFI_CONFIG_t structure where Configuration parameters are located
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported (security type, WPS or channel autodetect not supported)
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid interface, security type or NULL config_params pointer)
*/
int32_t WiFi_Configure (uint32_t interface, ARM_WIFI_CONFIG_t *config) {
  int32_t  ret;
  uint32_t resp_len;

  if ((interface > 1U) || (config == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  // SSID has to contain at least 1 non-null character
  if (config->ssid[0] == 0) {
    return ARM_DRIVER_ERROR;
  }

  switch (config->security) {
    case ARM_WIFI_SECURITY_OPEN:
      break;
    case ARM_WIFI_SECURITY_WEP:
    case ARM_WIFI_SECURITY_WPA:
    case ARM_WIFI_SECURITY_WPA2:
      // Password has to contain at least 1 non-null character
      if (config->pass[0] == 0) {
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_WIFI_SECURITY_UNKNOWN:
    default:
      return ARM_DRIVER_ERROR;
  }

  // Valid channel settings are 0 for auto and 1 to 13 for exact channel selection
  if (config->ch > 13U) {
    return ARM_DRIVER_ERROR;
  }

  switch (config->wps_method) {
    case ARM_WIFI_WPS_METHOD_NONE:
    case ARM_WIFI_WPS_METHOD_PBC:
      break;
    case ARM_WIFI_WPS_METHOD_PIN:
      // PIN has to contain at least 1 non-null character
      if (config->wps_pin[0] == 0) {
        return ARM_DRIVER_ERROR;
      }
      break;
    default:
      return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    if (interface == 0U) {              // Station
      if (config->wps_method != ARM_WIFI_WPS_METHOD_NONE) {     // If WPS is configured
        switch (config->wps_method) {
          case ARM_WIFI_WPS_METHOD_PBC:
            break;
          case ARM_WIFI_WPS_METHOD_PIN:
            snprintf(cmd_buf, sizeof(cmd_buf), "Z7=%s\r", config->wps_pin); resp_len = sizeof(resp_buf) - 1U;
            ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
            break;
          default:
            ret = ARM_DRIVER_ERROR_PARAMETER;
            break;
        }

        if (ret == ARM_DRIVER_OK) {
          sta_config_wps_method = config->wps_method;
        } else {
          sta_config_wps_method = 0U;
        }
      } else {                                                  // If WPS is not configured
        if (config->ch != 0U) {
          // Exact channel is not supported only auto
          ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        // Set network SSID
        if (ret == ARM_DRIVER_OK) {
          snprintf(cmd_buf, sizeof(cmd_buf), "C1=%s\r", config->ssid); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set network passphrase
        if (ret == ARM_DRIVER_OK) {
          snprintf(cmd_buf, sizeof(cmd_buf), "C2=%s\r", config->pass); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set network security mode
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)cmd_buf, (void *)"C3= \r", 6);
          switch (config->security) {
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
              ret = ARM_DRIVER_ERROR_UNSUPPORTED;
              break;
          }
          if (ret == ARM_DRIVER_OK) {
            resp_len = sizeof(resp_buf) - 1U;
            ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          }
        }

        if (ret == ARM_DRIVER_OK) {
          sta_config = 1U;
        } else {
          sta_config = 0U;
        }
      }
    } else {                            // AP
      if (config->wps_method != ARM_WIFI_WPS_METHOD_NONE) {     // If WPS is configured
        // WPS is not supported for Access Point
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      } else {                                                  // If WPS is not configured
        // Set AP security mode
        memcpy((void *)cmd_buf, (void *)"A1= \r", 6);
        switch (config->security) {
          case ARM_WIFI_SECURITY_OPEN:
            cmd_buf[3] = '0';
            break;
          case ARM_WIFI_SECURITY_WEP:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
          case ARM_WIFI_SECURITY_WPA:
            cmd_buf[3] = '2';
            break;
          case ARM_WIFI_SECURITY_WPA2:
            cmd_buf[3] = '3';
            break;
          default:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        }
        if (ret == ARM_DRIVER_OK) {
          resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set AP security key (password)
        if (ret == ARM_DRIVER_OK) {
          snprintf(cmd_buf, sizeof(cmd_buf), "A2=%s\r", config->pass); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set AP channel (0 = autoselect)
        if (ret == ARM_DRIVER_OK) {
          snprintf(cmd_buf, sizeof(cmd_buf), "AC=%d\r", config->ch); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set AP SSID
        if (ret == ARM_DRIVER_OK) {
          snprintf(cmd_buf, sizeof(cmd_buf), "AS=0,%s\r", config->ssid); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        // Set AP maximum number of clients to maximum which is 4
        if (ret == ARM_DRIVER_OK) {
          memcpy((void *)cmd_buf, (void *)"AT=4\r", 6); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        }

        if (ret == ARM_DRIVER_OK) {
          ap_config = 1U;
        } else {
          ap_config = 0U;
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
  \fn            int32_t WiFi_Activate (uint32_t mode)
  \brief         Activate selected mode of operation.
  \param[in]     mode     Mode of operation
                   - ARM_WIFI_MODE_STATION        : Station only
                   - ARM_WIFI_MODE_AP             : Access Point only
                   - ARM_WIFI_MODE_STATION_AP     : Station and Access Point
                   - ARM_WIFI_MODE_AD_HOC         : Ad-hoc
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
                   - ARM_DRIVER_ERROR_TIMEOUT     : Timeout occurred
                   - ARM_DRIVER_ERROR_UNSUPPORTED : Operation not supported
                   - ARM_DRIVER_ERROR_PARAMETER   : Parameter error (invalid mode)
*/
int32_t WiFi_Activate (uint32_t mode) {
  int32_t  ret;
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    ret = ARM_DRIVER_OK;

    switch (mode) {
      case ARM_WIFI_MODE_STATION:
        break;
      case ARM_WIFI_MODE_AP:
        break;
      case ARM_WIFI_MODE_NONE:
      case ARM_WIFI_MODE_STATION_AP:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
      default:
        ret = ARM_DRIVER_ERROR_PARAMETER;
        break;
    }

    if (mode == ARM_WIFI_MODE_STATION) {    // If station should be activated
      if (sta_config_wps_method != 0U) {
        memcpy((void *)cmd_buf, (void *)"CW= \r", 6);
        switch (sta_config_wps_method) {
          case ARM_WIFI_WPS_METHOD_PBC:
            // Prepare WPS push-button connection command
            cmd_buf[3] = '1';
            break;
          case ARM_WIFI_WPS_METHOD_PIN:
            // Prepare WPS PIN connection command
            cmd_buf[3] = '0';
            break;
          default:
            ret = ARM_DRIVER_ERROR;
            break;
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

        if (ret == ARM_DRIVER_OK) {
          sta_connected = 1U;
        } else {
          memset((void *)sta_local_ip, 0, 4);
        }
      } else if (sta_config != 0U) {
        // Send command to join a network
        memcpy((void *)cmd_buf, (void *)"C0\r", 4); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
        
        // Check if connection has succeeded
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

        if (ret == ARM_DRIVER_OK) {
          sta_connected = 1U;
        } else {
          memset((void *)sta_local_ip, 0, 4);
        }
      }
    } else {                                // If AP should be activated
      if (ap_config != 0U) {
        // Activate AP direct connect mode
        memcpy((void *)cmd_buf, (void *)"AD\r", 4); resp_len = sizeof(resp_buf) - 1U;
        ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);

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

        if (ret == ARM_DRIVER_OK) {
          ap_running = 1U;
          osEventFlagsSet(event_flags_id, EVENT_ASYNC_POLL);
        } else {
          memset((void *)ap_local_ip, 0, 4);
        }
      } else {
        ret = ARM_DRIVER_ERROR;
      }
    }

    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR_TIMEOUT;
  }

  if (ret == ARM_DRIVER_ERROR) {
    oper_mode = mode;
  }

  return ret;
}

/**
  \fn            int32_t WiFi_Deactivate (void)
  \brief         Deactivate current mode of operation.
  \return        execution status
                   - ARM_DRIVER_OK                : Operation successful
                   - ARM_DRIVER_ERROR             : Operation failed
*/
int32_t WiFi_Deactivate (void) {
  int32_t  ret;
  uint32_t resp_len;

  if (driver_initialized == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {

    if (sta_connected != 0U) {
      // Disconnect from network
      memcpy((void *)cmd_buf, (void *)"CD\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);

      if (ret == ARM_DRIVER_OK) {
        sta_connected = 0U;
        memset((void *)sta_local_ip, 0, 4);
      }
    }

    if (ap_running != 0U) {
      // Exit AP direct connect mode
      memcpy((void *)cmd_buf, (void *)"AE\r", 4); resp_len = sizeof(resp_buf) - 1U;
      ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);

      if (ret == ARM_DRIVER_OK) {
        ap_running = 0U;
        memset((void *)ap_local_ip, 0, 4);
      }
    }

    osMutexRelease(mutex_id_spi);
  } else {
    ret = ARM_DRIVER_ERROR;
  }

  if (ret == ARM_DRIVER_ERROR) {
    oper_mode = ARM_WIFI_MODE_NONE;
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
  uint32_t num;
  uint32_t resp_len;

  if (driver_initialized == 0U) {
    return 0U;
  }

  if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
    num    = 0U;
    status = ARM_DRIVER_OK;

    // Check station connection status
    memcpy((void *)cmd_buf, (void *)"CS\r", 4); resp_len = sizeof(resp_buf) - 1U;
    status = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    if ((status == ARM_DRIVER_OK) && (resp_len >= 3U)) {
      if (resp_buf[2] == '1') {
        num = 1U;
      }
    }

    if (num == 0U) {
      sta_connected = 0U;
      memset((void *)sta_local_ip, 0, 4);
    }

    osMutexRelease(mutex_id_spi);
  }

  return num;
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
int32_t WiFi_GetNetInfo (ARM_WIFI_NET_INFO_t *net_info) {
  int32_t  ret;
  uint8_t *ptr_resp_buf;
  uint32_t resp_len;
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
    memcpy((void *)cmd_buf, (void *)"C?\r", 4); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);

    ptr_resp_buf = SkipCommas(resp_buf, 14U);
    if (ptr_resp_buf != NULL) {
      if (*ptr_resp_buf == '1') {       // If station is connected
        // Channel is not available so clear it
        net_info->ch = 0U;

        // Extract ssid
        if (ret == ARM_DRIVER_OK) {
          if (sscanf((const char *)resp_buf + 2U, "%32[^,]s", net_info->ssid) != 1) {
            ret = ARM_DRIVER_ERROR;
          }
        }

        // Extract password
        // Skip ssid (1 ',')
        if (ret == ARM_DRIVER_OK) {
          ptr_resp_buf = SkipCommas(resp_buf, 1U);
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
          memcpy((void *)cmd_buf, (void *)"CR\r", 4); resp_len = sizeof(resp_buf) - 1U;
          ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
          if (ret == ARM_DRIVER_OK) {
            if (sscanf((const char *)resp_buf + 2U, "%d", &int_val) == 1) {
              net_info->rssi = (uint8_t)(int_val + 256);
            } else {
              ret = ARM_DRIVER_ERROR;
            }
          }
        }
      } else {                          // If station is not connected
        ret = ARM_DRIVER_ERROR;
      }
    } else {
      ret = ARM_DRIVER_ERROR;
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
  if ((ip == NULL) || (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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
  if (backlog != 1) {
    // Only backlog 1 is supported
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    // Check state depending on socket variables
    if (socket_arr[socket].protocol != ARM_SOCKET_SOCK_STREAM) {
      ret = ARM_SOCKET_ENOTSUP;
    } else if (socket_arr[socket].bound == 0U) {
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
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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

  if ((socket < 0) || (socket >= WIFI_ISM43362_SOCKETS_NUM)) {
    return ARM_SOCKET_ESOCK;
  }
  if ((ip == NULL) || (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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
      // Store remote host IP and port for UDP (datagram)
      memcpy((void *)socket_arr[socket].remote_ip, (void *)ip, 4);
      socket_arr[socket].remote_port = port;

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
  \fn            int32_t WiFi_SocketRecv (int32_t socket, void *buf, uint32_t len)
  \brief         Receive data on a connected socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes)
  \return        status information
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
  uint8_t  ip_udp[4];
  uint32_t ip_udp_len;
  uint16_t udp_port;

  if (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) {
    // For UDP reception SocketRecvFrom has to be used valid ip, ip_len and port pointers
    return (WiFi_SocketRecvFrom(socket, buf, len, ip_udp, &ip_udp_len, &udp_port));
  } else {
    return (WiFi_SocketRecvFrom(socket, buf, len, NULL, NULL, NULL));
  }
}

/**
  \fn            int32_t WiFi_SocketRecvFrom (int32_t socket, void *buf, uint32_t len, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Receive data on a socket.
  \param[in]     socket   Socket identification number
  \param[out]    buf      Pointer to buffer where data should be stored
  \param[in]     len      Length of buffer (in bytes)
  \param[out]    ip       Pointer to buffer where remote source address shall be returned (NULL for none)
  \param[in,out] ip_len   Pointer to length of 'ip' (or NULL if 'ip' is NULL)
                   - length of supplied 'ip' on input
                   - length of stored 'ip' on output
  \param[out]    port     Pointer to buffer where remote source port shall be returned (NULL for none)
  \return        status information
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
  if ((buf == NULL) || (len == 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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
              if (socket_arr[socket].bound != 0U) {                         // Socket bound (server)
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
              snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
                ret = ARM_SOCKET_ERROR;
              }
            }
          }
          // Set receive timeout (ms)
          if (ret == 0) {
            snprintf(cmd_buf, sizeof(cmd_buf), "R2=%d\r", timeout); resp_len = sizeof(resp_buf) - 1U;
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
              snprintf(cmd_buf, sizeof(cmd_buf), "R1=%d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
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
      if ((*ip != NULL) && (ip_len != NULL) && (*ip_len > 4U)) {
        ip[0] = socket_arr[socket].remote_ip[0];
        ip[1] = socket_arr[socket].remote_ip[1];
        ip[2] = socket_arr[socket].remote_ip[2];
        ip[3] = socket_arr[socket].remote_ip[3];
        *ip_len = 4U;
      }
      if (port != NULL) {
        *port = socket_arr[socket].remote_port;
      }
    } else {
      // If timed out in blocking mode or no data available in non-blocking mode
      ret = ARM_SOCKET_EAGAIN;
    }
  }

  return ret;
}

/**
  \fn            int32_t WiFi_SocketSend (int32_t socket, const void *buf, uint32_t len)
  \brief         Send data on a connected socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes)
  \return        status information
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
  uint8_t   ip_udp[4];
  uint32_t  ip_udp_len;
  uint16_t  udp_port;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((buf == NULL) || (len == 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (socket_arr[socket].protocol == ARM_SOCKET_IPPROTO_UDP) {
    // For UDP send SocketSendTo has to be used with ip, ip_len and port parameters 
    // that were provided on SocketConnect
    ip_udp_len = 4U;
    udp_port   = socket_arr[socket].remote_port;
    memcpy((void *)ip_udp, (void *)socket_arr[socket].remote_ip, ip_udp_len);

    return (WiFi_SocketSendTo(socket, buf, len, (const uint8_t *)ip_udp, ip_udp_len, udp_port));
  } else {
    return (WiFi_SocketSendTo(socket, buf, len, NULL, 0U, 0U));
  }
}

/**
  \fn            int32_t WiFi_SocketSendTo (int32_t socket, const void *buf, uint32_t len, const uint8_t *ip, uint32_t ip_len, uint16_t port)
  \brief         Send data on a socket.
  \param[in]     socket   Socket identification number
  \param[in]     buf      Pointer to buffer containing data to send
  \param[in]     len      Length of data (in bytes)
  \param[in]     ip       Pointer to remote destination IP address
  \param[in]     ip_len   Length of 'ip' address in bytes
  \param[in]     port     Remote destination port number
  \return        status information
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
  if ((buf == NULL) || (len == 0U)) {
    return ARM_SOCKET_EINVAL;
  }
  if ((ip != NULL) && (ip_len != 4U)) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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
          if (socket_arr[socket].bound != 0U) {                         // Socket bound (server)
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
          snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
            ret = ARM_SOCKET_ERROR;
          }
        }
        // Set transmit timeout (ms)
        if (ret == 0) {
          snprintf(cmd_buf, sizeof(cmd_buf), "S2=%d\r", socket_arr[socket].send_timeout); resp_len = sizeof(resp_buf) - 1U;
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
            snprintf(cmd_buf, sizeof(cmd_buf), "S3=%04d\r", len_req); resp_len = sizeof(resp_buf) - 1U;
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
  uint8_t *ptr_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  uint16_t u16_val;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

    hw_socket = socket;
    if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
      hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
    }

    if (ip != NULL) {
      if (sta_connected != 0U) {
        memcpy((void *)ip, (void *)sta_local_ip, 4);
      } else if (ap_running != 0U) {
        memcpy((void *)ip, (void *)ap_local_ip, 4);
      }
      if (ip_len != NULL) {
        *ip_len = 4U;
      }
    }

    // Execute functionality on the module through SPI commands
    if (port != NULL) {
      if (ret == 0) {
        if (osMutexAcquire(mutex_id_spi, WIFI_ISM43362_SPI_TIMEOUT) == osOK) {
          // Set communication socket number
          snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
            ret = ARM_SOCKET_ERROR;
          }

          if (ret == 0) {
            // Show Transport Settings
            memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
              // Skip protocol and client IP (2 ',') from response
              ptr_resp_buf = SkipCommas(resp_buf, 2U);
              if (ptr_resp_buf != NULL) {
                if (sscanf((const char *)ptr_resp_buf, "%hu", &u16_val) == 1) {
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
  \fn            int32_t WiFi_SocketGetPeerName (int32_t socket, uint8_t *ip, uint32_t *ip_len, uint16_t *port)
  \brief         Retrieve remote IP address and port of a socket
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
  uint8_t *ptr_resp_buf;
  int32_t  ret;
  uint32_t resp_len;
  uint16_t u16_val;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (((ip != NULL) && (ip_len == NULL)) || ((ip_len != NULL) && (*ip_len < 4U))) {
    return ARM_SOCKET_EINVAL;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
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
        snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
        if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
          ret = ARM_SOCKET_ERROR;
        }

        if (ret == 0) {
          // Show Transport Settings
          memcpy((void *)cmd_buf, (void *)"P?\r", 4); resp_len = sizeof(resp_buf) - 1U;
          if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
            if (ip != NULL) {
              // Skip protocol (1 ',') from response
              ptr_resp_buf = SkipCommas(resp_buf, 1U);
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
              ptr_resp_buf = SkipCommas(resp_buf, 4U);
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
  if ((opt_len == NULL) || (opt_len == NULL) || (*opt_len < 4U)) {
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
  int32_t  ret;
  uint32_t u32, resp_len;
  uint8_t  hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if ((opt_len == NULL) || (opt_len != 4U)) {
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
            hw_socket = socket;
            if (hw_socket >= WIFI_ISM43362_SOCKETS_NUM) {
              hw_socket -= WIFI_ISM43362_SOCKETS_NUM;
            }
            snprintf(cmd_buf, sizeof(cmd_buf), "P0=%d\r", hw_socket); resp_len = sizeof(resp_buf) - 1U;
            if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) != ARM_DRIVER_OK) {
              ret = ARM_SOCKET_ERROR;
            }
            if (ret == 0) {
              if (u32 == 0U) {
                memcpy((void *)cmd_buf, (void *)"PK=0,0\r", 7); resp_len = sizeof(resp_buf) - 1U;
              } else {
                snprintf(cmd_buf, sizeof(cmd_buf), "PK=1,%d\r", u32); resp_len = sizeof(resp_buf) - 1U;
              }
              if (SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT) == ARM_DRIVER_OK) {
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
  int32_t ret;
  uint8_t hw_socket;

  if ((socket < 0) || (socket >= (2 * WIFI_ISM43362_SOCKETS_NUM))) {
    return ARM_SOCKET_ESOCK;
  }
  if (socket_arr[socket].state == SOCKET_STATE_FREE) {
    return 0;
  }
  if (driver_initialized == 0U) {
    return ARM_SOCKET_ERROR;
  }

  if (osMutexAcquire(mutex_id_sockets, osWaitForever) == osOK) {
    ret = 0;

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
  int32_t  ret;
  uint32_t resp_len;

  if (af != ARM_SOCKET_AF_INET) {
    return ARM_SOCKET_ENOTSUP;
  }
  if ((ip == NULL) || (ip_len == NULL) || (*ip_len != 4U)) {
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
    snprintf(cmd_buf, sizeof(cmd_buf), "D0=%s\r", name); resp_len = sizeof(resp_buf) - 1U;
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
    snprintf(cmd_buf, sizeof(cmd_buf), "T1=%d.%d.%d.%d\r", ip[0], ip[1], ip[2], ip[3]); resp_len = sizeof(resp_buf) - 1U;
    ret = SPI_AT_SendCommandReceiveResponse(cmd_buf, resp_buf, &resp_len, WIFI_ISM43362_CMD_TIMEOUT);
    // Ping IP target address
    if (ret == ARM_DRIVER_OK) {
      memcpy((void *)cmd_buf, (void *)"T0\r", 3); resp_len = sizeof(resp_buf) - 1U;
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
  WiFi_GetModuleInfo,
  WiFi_SetOption,
  WiFi_GetOption,
  WiFi_Scan,
  WiFi_Configure,
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
