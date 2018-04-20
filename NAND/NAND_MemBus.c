/*
 * Copyright (c) 2013-2018 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------
 *
 * $Date:        20. April 2018
 * $Revision:    V1.1
 *
 * Driver:       Driver_NAND# (default: Driver_NAND0)
 * Project:      NAND Flash Device connected to Memory Bus Driver
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_NAND# = n (default: 0)
 * -------------------------------------------------------------------- */

#include "NAND_MemBus_Config.h"

#include "Driver_NAND.h"

#define ARM_NAND_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,1)   /* driver version */


#if   (NAND_DEV0 && NAND_DEV1 && NAND_DEV2 && NAND_DEV3)
#define NAND_NUM_DEVS 4
#elif (NAND_DEV0 && NAND_DEV1 && NAND_DEV2)
#define NAND_NUM_DEVS 3
#elif (NAND_DEV0 && NAND_DEV1)
#define NAND_NUM_DEVS 2
#elif (NAND_DEV0)
#define NAND_NUM_DEVS 1
#else
#error "Invalid NAND Device selection!"
#endif

#if (!(NAND_DEV0 && NAND_DEV0_RB_PIN && NAND_DEV0_RB_PIN_IRQ) && \
     !(NAND_DEV1 && NAND_DEV1_RB_PIN && NAND_DEV1_RB_PIN_IRQ) && \
     !(NAND_DEV2 && NAND_DEV2_RB_PIN && NAND_DEV2_RB_PIN_IRQ) && \
     !(NAND_DEV3 && NAND_DEV3_RB_PIN && NAND_DEV3_RB_PIN_IRQ))
#define NAND_DEVICE_EVENT  0
#else
#define NAND_DEVICE_EVENT  1
#endif

#if (!(NAND_DEV0 && NAND_DEV0_RB_PIN) && \
     !(NAND_DEV1 && NAND_DEV1_RB_PIN) && \
     !(NAND_DEV2 && NAND_DEV2_RB_PIN) && \
     !(NAND_DEV3 && NAND_DEV3_RB_PIN))
#define NAND_RB_MONITOR    0
#else
#define NAND_RB_MONITOR    1
#endif


#define MEM_8BIT( addr) (*(volatile uint8_t  *)(addr))
#define MEM_16BIT(addr) (*(volatile uint16_t *)(addr))

#define _Driver_NAND_(n)                    Driver_NAND##n
#define  Driver_NAND_(n)                   _Driver_NAND_(n)
#define _Driver_NAND_GetDeviceBusy_(n)      Driver_NAND##n##_##GetDeviceBusy
#define  Driver_NAND_GetDeviceBusy_(n)     _Driver_NAND_GetDeviceBusy_(n)
#define _Driver_NAND_Event_DeviceReady_(n)  Driver_NAND##n##_##Event_DeviceReady
#define  Driver_NAND_Event_DeviceReady_(n) _Driver_NAND_Event_DeviceReady_(n)


#if (NAND_RB_MONITOR)
/**
  \fn          int32_t Driver_NANDx_GetDeviceBusy (uint32_t dev_num)
  \brief       NAND Driver GetDeviceBusy callback.
               Needs to be implemented by user.
  \param[in]   dev_num   Device number
  \return      1=busy, 0=not busy, or error
*/
extern int32_t Driver_NAND_GetDeviceBusy_(NAND_DRIVER) (uint32_t dev_num);
#endif


#if (NAND_DEVICE_EVENT)
/**
  \fn          void Driver_NANDx_Event_DeviceReady (uint32_t dev_num)
  \brief       NAND Driver Event Device Ready callback.
               Needs to be called on Ready/Busy pin rising edge.
  \param[in]   dev_num   Device number
*/
extern void Driver_NAND_Event_DeviceReady_(NAND_DRIVER) (uint32_t dev_num);
#endif


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_NAND_API_VERSION,
  ARM_NAND_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_NAND_CAPABILITIES DriverCapabilities = {
  NAND_DEVICE_EVENT, /* event_device_ready  */
  1,                 /* reentrant_operation */
  0,                 /* sequence_operation  */
  0,                 /* vcc                 */
  0,                 /* vcc_1v8             */
  0,                 /* vccq                */
  0,                 /* vccq_1v8            */
  0,                 /* vpp                 */
  0,                 /* wp                  */
  NAND_NUM_DEVS - 1, /* ce_lines            */
  0,                 /* ce_manual           */
  NAND_RB_MONITOR,   /* rb_monitor          */
  1,                 /* data_width_16       */
  0,                 /* ddr                 */
  0,                 /* ddr2                */
  0,                 /* sdr_timing_mode     */
  0,                 /* ddr_timing_mode     */
  0,                 /* ddr2_timing_mode    */
  0,                 /* driver_strength_18  */
  0,                 /* driver_strength_25  */
  0,                 /* driver_strength_50  */
#if (ARM_NAND_API_VERSION > 0x201U)
  0
#endif
};


#if NAND_DEVICE_EVENT
static ARM_NAND_SignalEvent_t NAND_EventCallback;  /* Event Callback */
static uint32_t               NAND_DeviceEvent;    /* Device event enable/disable mask */
#endif


/* NAND Bus Information definition */
typedef struct {
  uint32_t addr_base;   /* Base Address */
  uint32_t addr_ale;    /* ALE Address  */
  uint32_t addr_cle;    /* CLE Address  */
  uint32_t data_width;  /* Data Bus Width: 0=>8-bit, 1=>16-bit */
} const NAND_BUS_INFO;


/* NAND Bus Information */
static NAND_BUS_INFO NAND_BusInfo[NAND_NUM_DEVS] = {
#if (NAND_NUM_DEVS >= 1)
#if (NAND_DEV0)
{ NAND_DEV0_ADDR_BASE,
  NAND_DEV0_ADDR_ALE,
  NAND_DEV0_ADDR_CLE,
 (NAND_DEV0_DATA_WIDTH == 16) ? 1 : 0
},
#else
 { 0 },
#endif
#endif
#if (NAND_NUM_DEVS >= 2)
#if (NAND_DEV1)
{ NAND_DEV1_ADDR_BASE,
  NAND_DEV1_ADDR_ALE,
  NAND_DEV1_ADDR_CLE,
 (NAND_DEV1_DATA_WIDTH == 16) ? 1 : 0
},
#else
 { 0 },
#endif
#endif
#if (NAND_NUM_DEVS >= 3)
#if (NAND_DEV2)
{ NAND_DEV2_ADDR_BASE,
  NAND_DEV2_ADDR_ALE,
  NAND_DEV2_ADDR_CLE,
 (NAND_DEV2_DATA_WIDTH == 16) ? 1 : 0
},
#else
 { 0 },
#endif
#endif
#if (NAND_NUM_DEVS >= 4)
#if (NAND_DEV3)
{ NAND_DEV3_ADDR_BASE,
  NAND_DEV3_ADDR_ALE,
  NAND_DEV3_ADDR_CLE,
 (NAND_DEV3_DATA_WIDTH == 16) ? 1 : 0
}
#else
 { 0 }
#endif
#endif
};


/* Exported functions */

/**
  \fn            ARM_DRIVER_VERSION GetVersion (void)
  \brief         Get driver version.
  \return        \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn            ARM_NAND_CAPABILITIES GetCapabilities (void)
  \brief         Get driver capabilities.
  \return        \ref ARM_NAND_CAPABILITIES
*/
static ARM_NAND_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}


/**
  \fn            int32_t Initialize (ARM_NAND_SignalEvent_t cb_event)
  \brief         Initialize the NAND Interface.
  \param[in]     cb_event  Pointer to \ref ARM_NAND_SignalEvent
  \return        \ref execution_status
*/
static int32_t Initialize (ARM_NAND_SignalEvent_t cb_event) {
#if (NAND_DEVICE_EVENT == 0)
  (void)cb_event;
#else
  NAND_EventCallback = cb_event;
#endif
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize the NAND Interface.
  \return        \ref execution_status
*/
static int32_t Uninitialize (void) {
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t PowerControl (ARM_POWER_STATE state)
  \brief         Control the NAND interface power.
  \param[in]     state  Power state
  \return        \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {

  switch ((int32_t)state) {
    case ARM_POWER_OFF:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t DevicePower (uint32_t voltage)
  \brief         Set device power supply voltage.
  \param[in]     voltage  NAND Device supply voltage
  \return        \ref execution_status
*/
static int32_t DevicePower (uint32_t voltage) {
  (void)voltage;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t WriteProtect (uint32_t dev_num, bool enable)
  \brief         Control WPn (Write Protect).
  \param[in]     dev_num  Device number
  \param[in]     enable
                - \b false Write Protect off
                - \b true  Write Protect on
  \return        \ref execution_status
*/
static int32_t WriteProtect (uint32_t dev_num, bool enable) {
  (void)dev_num; (void)enable;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ChipEnable (uint32_t dev_num, bool enable)
  \brief         Control CEn (Chip Enable).
  \param[in]     dev_num  Device number
  \param[in]     enable
                - \b false Chip Enable off
                - \b true  Chip Enable on
  \return        \ref execution_status
*/
static int32_t ChipEnable (uint32_t dev_num, bool enable) {
  (void)dev_num; (void)enable;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ARM_NAND_GetDeviceBusy (uint32_t dev_num)
  \brief         Get Device Busy pin state.
  \param[in]     dev_num  Device number
  \return        1=busy, 0=not busy, or error
*/
static int32_t GetDeviceBusy (uint32_t dev_num) {
#if NAND_RB_MONITOR
  if (dev_num >= NAND_NUM_DEVS) return ARM_DRIVER_ERROR_PARAMETER;

  return Driver_NAND_GetDeviceBusy_(NAND_DRIVER) (dev_num);
#else
  (void)dev_num;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
#endif
}


/**
  \fn            int32_t SendCommand (uint32_t dev_num, uint8_t cmd)
  \brief         Send command to NAND device.
  \param[in]     dev_num  Device number
  \param[in]     cmd      Command
  \return        \ref execution_status
*/
static int32_t SendCommand (uint32_t dev_num, uint8_t cmd) {
  if (dev_num >= NAND_NUM_DEVS) return ARM_DRIVER_ERROR_PARAMETER;

  MEM_8BIT (NAND_BusInfo[dev_num].addr_cle) = cmd;
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t SendAddress (uint32_t dev_num, uint8_t addr)
  \brief         Send address to NAND device.
  \param[in]     dev_num  Device number
  \param[in]     addr     Address
  \return        \ref execution_status
*/
static int32_t SendAddress (uint32_t dev_num, uint8_t addr) {
  if (dev_num >= NAND_NUM_DEVS) return ARM_DRIVER_ERROR_PARAMETER;

  MEM_8BIT (NAND_BusInfo[dev_num].addr_ale) = addr;
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t ReadData (uint32_t dev_num, void *data, uint32_t cnt, uint32_t mode)
  \brief         Read data from NAND device.
  \param[in]     dev_num  Device number
  \param[out]    data     Pointer to buffer for data to read from NAND device
  \param[in]     cnt      Number of data items to read
  \param[in]     mode     Operation mode
  \return        number of data items read or \ref execution_status
*/
static int32_t ReadData (uint32_t dev_num, void *data, uint32_t cnt, uint32_t mode) {
  uint32_t addr_base;
  uint32_t data_width;
  uint32_t n;

  (void)mode;

  if (dev_num >= NAND_NUM_DEVS) return ARM_DRIVER_ERROR_PARAMETER;
  if (data == NULL)             return ARM_DRIVER_ERROR_PARAMETER;

  addr_base  = NAND_BusInfo[dev_num].addr_base;
  data_width = NAND_BusInfo[dev_num].data_width;

  if (data_width != 0) {
    /* 16-bit bus read */
    for (n = 0; n < cnt; n++) {
      ((uint16_t *)data)[n] = MEM_16BIT (addr_base);
    }
  }
  else {
    /* 8-bit bus read */
    for (n = 0; n < cnt; n++) {
      ((uint8_t *)data)[n] = MEM_8BIT (addr_base);
    }
  }
  return (int32_t)cnt;
}


/**
  \fn            int32_t WriteData (uint32_t dev_num, const void *data, uint32_t cnt, uint32_t mode)
  \brief         Write data to NAND device.
  \param[in]     dev_num  Device number
  \param[out]    data     Pointer to buffer with data to write to NAND device
  \param[in]     cnt      Number of data items to write
  \param[in]     mode     Operation mode
  \return        number of data items written or \ref execution_status
*/
static int32_t WriteData (uint32_t dev_num, const void *data, uint32_t cnt, uint32_t mode) {
  uint32_t addr_base;
  uint32_t data_width;
  uint32_t n;

  (void)mode;

  if (dev_num >= NAND_NUM_DEVS) return ARM_DRIVER_ERROR_PARAMETER;
  if (data == NULL)             return ARM_DRIVER_ERROR_PARAMETER;

  addr_base  = NAND_BusInfo[dev_num].addr_base;
  data_width = NAND_BusInfo[dev_num].data_width;

  if (data_width != 0) {
    /* 16-bit bus write */
    for (n = 0; n < cnt; n++) {
      MEM_16BIT (addr_base) = ((const uint16_t *)data)[n];
    }
  }
  else {
    /* 8-bit bus write */
    for (n = 0; n < cnt; n++) {
      MEM_8BIT (addr_base) = ((const uint8_t *)data)[n];
    }
  }
  return (int32_t)cnt;
}


/**
  \fn            int32_t ExecuteSequence (uint32_t dev_num, uint32_t code, uint32_t cmd,
                                          uint32_t addr_col, uint32_t addr_row,
                                          void *data, uint32_t data_cnt,
                                          uint8_t *status, uint32_t *count)
  \brief         Execute sequence of operations.
  \param[in]     dev_num  Device number
  \param[in]     code     Sequence code
  \param[in]     cmd      Command(s)
  \param[in]     addr_col Column address
  \param[in]     addr_row Row address
  \param[in,out] data     Pointer to data to be written or read 
  \param[in]     data_cnt Number of data items in one iteration
  \param[out]    status   Pointer to status read
  \param[in,out] count    Number of iterations
  \return        \ref execution_status
*/
static int32_t ExecuteSequence (uint32_t dev_num, uint32_t code, uint32_t cmd,
                                uint32_t addr_col, uint32_t addr_row,
                                void *data, uint32_t data_cnt,
                                uint8_t *status, uint32_t *count) {
  (void)dev_num; (void)code; (void)cmd;
  (void)addr_col; (void)addr_row;
  (void)data; (void)data_cnt;
  (void)status; (void)count;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t AbortSequence (uint32_t dev_num)
  \brief         Abort sequence execution.
  \param[in]     dev_num  Device number
  \return        \ref execution_status
*/
static int32_t AbortSequence (uint32_t dev_num) {
  (void)dev_num;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t Control (uint32_t dev_num, uint32_t control, uint32_t arg)
  \brief         Control NAND Interface.
  \param[in]     dev_num  Device number
  \param[in]     control  operation
  \param[in]     arg      argument of operation 
  \return        \ref execution_status
*/
static int32_t Control (uint32_t dev_num, uint32_t control, uint32_t arg) {

  switch (control) {
    case ARM_NAND_BUS_MODE:
      switch (arg & ARM_NAND_BUS_INTERFACE_Msk) {
        case ARM_NAND_BUS_SDR:
          switch (arg & ARM_NAND_BUS_TIMING_MODE_Msk) {
            case ARM_NAND_BUS_TIMING_MODE_0:
              return ARM_DRIVER_OK;
            default:
              return ARM_DRIVER_ERROR_UNSUPPORTED;
          }
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

    case ARM_NAND_BUS_DATA_WIDTH:
      switch (arg) {
        case ARM_NAND_BUS_DATA_WIDTH_8:
          if (NAND_BusInfo[dev_num].data_width == 0) {
            return ARM_DRIVER_OK;
          }
          break;
        case ARM_NAND_BUS_DATA_WIDTH_16:
          if (NAND_BusInfo[dev_num].data_width == 1) {
            return ARM_DRIVER_OK;
          }
          break;
        default:
          break;
      }
      break;

    #if NAND_DEVICE_EVENT
    case ARM_NAND_DEVICE_READY_EVENT:
      if (arg != 0) {
        NAND_DeviceEvent |=  (1 << dev_num);
      } else {
        NAND_DeviceEvent &= ~(1 << dev_num);
      }
      return ARM_DRIVER_OK;
    #endif

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_ERROR;
}


/**
  \fn            ARM_NAND_STATUS GetStatus (uint32_t dev_num)
  \brief         Get NAND status.
  \param[in]     dev_num  Device number
  \return        NAND status \ref ARM_NAND_STATUS
*/
static ARM_NAND_STATUS GetStatus (uint32_t dev_num) {
  ARM_NAND_STATUS stat;
  (void)dev_num;

  stat.busy      = 0U;
  stat.ecc_error = 0U;

  return stat;
}


/**
  \fn            int32_t InquireECC (int32_t index, ARM_NAND_ECC_INFO *info)
  \brief         Inquire about available ECC.
  \param[in]     index   Device number
  \param[out]    info    Pointer to ECC information \ref ARM_NAND_ECC_INFO retrieved
  \return        \ref execution_status
*/
static int32_t InquireECC (int32_t index, ARM_NAND_ECC_INFO *info) {
  (void)index;
  (void)info;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn          void Driver_NANDx_Event_DeviceReady (uint32_t dev_num)
  \brief       NAND Driver Event Device Ready callback.
               Needs to be called on Ready/Busy pin rising edge.
  \param[in]   dev_num   Device number
*/
#if (NAND_DEVICE_EVENT)
void Driver_NAND_Event_DeviceReady_(NAND_DRIVER) (uint32_t dev_num) {
  if (NAND_EventCallback && (NAND_DeviceEvent & (1 << dev_num))) {
    NAND_EventCallback (dev_num, ARM_NAND_EVENT_DEVICE_READY);
  }
}
#endif


/* NAND Driver Control Block */
extern
ARM_DRIVER_NAND Driver_NAND_(NAND_DRIVER);
ARM_DRIVER_NAND Driver_NAND_(NAND_DRIVER) = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  DevicePower,
  WriteProtect,
  ChipEnable,
  GetDeviceBusy,
  SendCommand,
  SendAddress,
  ReadData,
  WriteData,
  ExecuteSequence,
  AbortSequence,
  Control,
  GetStatus,
  InquireECC
};
