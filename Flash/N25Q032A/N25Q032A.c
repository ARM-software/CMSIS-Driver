/*
 * Copyright (c) 2017-2018 Arm Limited. All rights reserved.
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
 * $Date:        19. April 2018
 * $Revision:    V1.1
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Driver for Micron N25Q032A (SPI)
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_Flash# = n (default: 0)
 * -------------------------------------------------------------------- */

/* Note:
    Use the following definitions to customize this driver:

    #define DRIVER_FLASH_NUM   #
    Replace symbol # with any integer to customize the number of exported
    driver (i.e. Driver_Flash#) Default setting is 0.
    
    #define DRIVER_SPI_NUM     #
    Replace symbol # with any integer to customize the number of SPI driver
    (i.e. Driver_SPI#) used to communicate with Flash device.
    Default setting is 0.
    
    #define DRIVER_SPI_BUS_SPEED #
    Replace symbol # with any integer to customize the SPI bus frequency.
    Default setting is 36000000 (36MHz).
*/

#include "Driver_Flash.h"
#include "Driver_SPI.h"
#include "N25Q032A.h"

#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,1) /* driver version */


#ifndef DRIVER_FLASH_NUM
#define DRIVER_FLASH_NUM        0         /* Default driver number */
#endif

#ifndef DRIVER_SPI_NUM
#define DRIVER_SPI_NUM          0         /* Default SPI driver number */
#endif

#ifndef DRIVER_SPI_BUS_SPEED
#define DRIVER_SPI_BUS_SPEED    36000000  /* Default SPI bus speed */
#endif

/* SPI Data Flash Commands */
#define CMD_READ_DATA           (0x03U)
#define CMD_READ_STATUS         (0x05U)
#define CMD_WRITE_ENABLE        (0x06U)
#define CMD_PAGE_PROGRAM        (0x02U)
#define CMD_READ_FLAG_STATUS    (0x70U)
#define CMD_SECTOR_ERASE        (0xD8U)
#define CMD_BULK_ERASE          (0xC7U)

/* Flash Driver Flags */
#define FLASH_INIT              (0x01U)
#define FLASH_POWER             (0x02U)


/* SPI Driver */
#define _SPI_Driver_(n)  Driver_SPI##n
#define  SPI_Driver_(n)  _SPI_Driver_(n)
extern ARM_DRIVER_SPI     SPI_Driver_(DRIVER_SPI_NUM);
#define ptrSPI          (&SPI_Driver_(DRIVER_SPI_NUM))

/* SPI Bus Speed */
#define SPI_BUS_SPEED   ((uint32_t)DRIVER_SPI_BUS_SPEED)

/* Flash Information */
static ARM_FLASH_INFO FlashInfo = {
  NULL,
  FLASH_SECTOR_COUNT,
  FLASH_SECTOR_SIZE,
  FLASH_PAGE_SIZE,
  FLASH_PROGRAM_UNIT,
  FLASH_ERASED_VALUE,
#if (ARM_FLASH_API_VERSION > 0x201U)
  { 0U, 0U, 0U }
#endif
};

/* Flash Status */
static ARM_FLASH_STATUS FlashStatus;
static uint8_t Flags;


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_FLASH_API_VERSION,
  ARM_FLASH_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_FLASH_CAPABILITIES DriverCapabilities = {
  0U,   /* event_ready */
  0U,   /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
  1U,   /* erase_chip */
#if (ARM_FLASH_API_VERSION > 0x200U)
  0U    /* reserved */
#endif
};


/* Read status or flag status register */
static int32_t ReadStatusReg (uint8_t cmd, uint8_t *stat) {
  int32_t status; /* driver execution status */
  uint8_t buf[4];

  /* Select Slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
    /* Set command */
    buf[0] = cmd;

    /* Send command and receive register value */
    status = ptrSPI->Transfer (&buf[0], &buf[2], 2U);

    if (status == ARM_DRIVER_OK) {
      /* Wait till transfer done */
      while (ptrSPI->GetDataCount() != 2U);

      *stat = buf[3];
    }
  }
  /* Deselect Slave */
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  
  return (status);
}

/* Set "Write enable latch" bit in status register */
static int32_t SetWriteEnable (void) {
  int32_t status;
  uint8_t val;
 
  /* Select slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
    val = CMD_WRITE_ENABLE;

    status = ptrSPI->Send(&val, 1U);

    if (status == ARM_DRIVER_OK) {
      while (ptrSPI->GetDataCount() != 1U);
    }
  }
  /* Deselect slave */
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

  if (status == ARM_DRIVER_OK) {
    /* Read status */
    val = 0U;

    status = ReadStatusReg(CMD_READ_STATUS, &val);

    if (status == ARM_DRIVER_OK) {
      /* Check if "Write enable latch" bit set */
      if ((val & 0x02U) == 0x00U) {
        status = ARM_DRIVER_ERROR;
      }
    }
  }

  return (status);
}


/**
  \fn          ARM_DRIVER_VERSION ARM_Flash_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          ARM_FLASH_CAPABILITIES ARM_Flash_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_FLASH_CAPABILITIES
*/
static ARM_FLASH_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t ARM_Flash_Initialize (ARM_Flash_SignalEvent_t cb_event)
  \brief       Initialize the Flash Interface.
  \param[in]   cb_event  Pointer to \ref ARM_Flash_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize (ARM_Flash_SignalEvent_t cb_event) {
  int32_t status;
  (void)cb_event;

  FlashStatus.busy  = 0U;
  FlashStatus.error = 0U;

  status = ptrSPI->Initialize(NULL);
  if (status != ARM_DRIVER_OK) { return ARM_DRIVER_ERROR; }

  Flags = FLASH_INIT;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_Uninitialize (void)
  \brief       De-initialize the Flash Interface.
  \return      \ref execution_status
*/
static int32_t Uninitialize (void) {

  Flags = 0U;
  return ptrSPI->Uninitialize();
} 

/**
  \fn          int32_t ARM_Flash_PowerControl (ARM_POWER_STATE state)
  \brief       Control the Flash interface power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  int32_t status;

  switch ((int32_t)state) {
    case ARM_POWER_OFF:
      Flags &= ~FLASH_POWER;

      FlashStatus.busy  = 0U;
      FlashStatus.error = 0U;

      return ptrSPI->PowerControl(ARM_POWER_FULL);

    case ARM_POWER_FULL:
      if ((Flags & FLASH_INIT) == 0U) {
        return ARM_DRIVER_ERROR;
      }

      if ((Flags & FLASH_POWER) == 0U) {
        status = ptrSPI->PowerControl(ARM_POWER_FULL);
        if (status != ARM_DRIVER_OK) {
          return ARM_DRIVER_ERROR;
        }
        status = ptrSPI->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0  |
                                                       ARM_SPI_DATA_BITS(8) |
                                                       ARM_SPI_MSB_LSB      |
                                                       ARM_SPI_SS_MASTER_SW,
                                                       SPI_BUS_SPEED);
        if (status != ARM_DRIVER_OK) {
          return ARM_DRIVER_ERROR;
        }

        FlashStatus.busy  = 0U;
        FlashStatus.error = 0U;

        Flags |= FLASH_POWER;
      }
      return ARM_DRIVER_OK;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    default:
      return ARM_DRIVER_ERROR;
  }
}

/**
  \fn          int32_t ARM_Flash_ReadData (uint32_t addr, void *data, uint32_t cnt)
  \brief       Read data from Flash.
  \param[in]   addr  Data address.
  \param[out]  data  Pointer to a buffer storing the data read from Flash.
  \param[in]   cnt   Number of data items to read.
  \return      number of data items read or \ref execution_status
*/
static int32_t ReadData (uint32_t addr, void *data, uint32_t cnt) {
  uint8_t  buf[8];
  int32_t  status;

  if ((addr > (FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)) || (data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /* Prepare Command with address */
  buf[0] = CMD_READ_DATA;
  buf[1] = (uint8_t)(addr >> 16);
  buf[2] = (uint8_t)(addr >>  8);
  buf[3] = (uint8_t)(addr >>  0);

  /* Select Slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
    /* Send Command with Address */
    status = ptrSPI->Send(buf, 4U);

    if (status == ARM_DRIVER_OK) {
      while (ptrSPI->GetDataCount() != 4U);

      status = ptrSPI->Receive(data, cnt);

      if (status == ARM_DRIVER_OK) {
        while (ptrSPI->GetDataCount() != cnt);
        
        /* Number of data items read */
        status = (int32_t)cnt;
      }
    }
  }
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

  return (status);
}

/**
  \fn          int32_t ARM_Flash_ProgramData (uint32_t addr, const void *data, uint32_t cnt)
  \brief       Program data to Flash.
  \param[in]   addr  Data address.
  \param[in]   data  Pointer to a buffer containing the data to be programmed to Flash.
  \param[in]   cnt   Number of data items to program.
  \return      number of data items programmed or \ref execution_status
*/
static int32_t ProgramData (uint32_t addr, const void *data, uint32_t cnt) {
  const uint8_t *buf;
        uint8_t  cmd[4];
        int32_t  status;
        uint32_t num, n;

  if ((addr > (FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)) || (data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  status = ARM_DRIVER_OK;

  num = 0U;
  buf = data;

  while (cnt) {
    /* Enable data write */
    status = SetWriteEnable();
    
    if (status == ARM_DRIVER_OK) {
      /* Select Slave */
      status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
      
      if (status == ARM_DRIVER_OK) {
        /* Prepare command with address */
        cmd[0] = CMD_PAGE_PROGRAM;
        cmd[1] = (uint8_t)(addr >> 16U);
        cmd[2] = (uint8_t)(addr >>  8U);
        cmd[3] = (uint8_t)(addr >>  0U);

        status = ptrSPI->Send(cmd, 4U);

        if (status == ARM_DRIVER_OK) {
          /* Wait until command and address are sent */
          while (ptrSPI->GetDataCount() != 4U);

          n = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);

          if (n > cnt) {
            n = cnt;
          }

          status = ptrSPI->Send(&buf[num], n);

          if (status == ARM_DRIVER_OK) {
            /* Wait until data sent */
            while (ptrSPI->GetDataCount() != n);

            addr += n;
            num  += n;
            cnt  -= n;
          }
        }
      }
      /* Deselect Slave */
      ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

      if (status == ARM_DRIVER_OK) {
        /* Read status until device ready */
        do {
          status = ReadStatusReg(CMD_READ_FLAG_STATUS, &cmd[0]);
          if (status != ARM_DRIVER_OK) {
            break;
          }

          /* Check Flags Status register value */
          if ((cmd[0] & 0x80U) != 0U) {
            FlashStatus.busy = 0U;
          }

          /* Check Program and Erase bits */
          if ((cmd[0] & 0x30) == 0U) {
            FlashStatus.error = 0U;
          } else {
            FlashStatus.error = 1U;
          }
        }
        while ((cmd[0] & 0x80U) == 0x00U);
      }
    }

    if (status != ARM_DRIVER_OK) {
      break;
    }
    /* Number of data items programmed */
    status = (int32_t)num;
  }

  return status;
}

/**
  \fn          int32_t ARM_Flash_EraseSector (uint32_t addr)
  \brief       Erase Flash Sector.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
static int32_t EraseSector (uint32_t addr) {
  uint8_t buf[4];
  int32_t status;

  status = SetWriteEnable();

  if (status == ARM_DRIVER_OK) {
    FlashStatus.busy  = 1U;
    FlashStatus.error = 0U;

    /* Select Slave */
    status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

    if (status == ARM_DRIVER_OK) {
      /* Prepare command with address */
      buf[0] = CMD_SECTOR_ERASE;
      buf[1] = (uint8_t)(addr >> 16U);
      buf[2] = (uint8_t)(addr >>  8U);
      buf[3] = (uint8_t)(addr >>  0U);

      status = ptrSPI->Send(buf, 4U);

      if (status == ARM_DRIVER_OK) {
        /* Wait until command and address are sent */
        while (ptrSPI->GetDataCount() != 4U);
      }
    }
    /* Deselect Slave */
    ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  }

  return status;
}

/**
  \fn          int32_t ARM_Flash_EraseChip (void)
  \brief       Erase complete Flash.
               Optional function for faster full chip erase.
  \return      \ref execution_status
*/
static int32_t EraseChip (void) {
  uint8_t cmd;
  int32_t status;

  FlashStatus.busy  = 1U;
  FlashStatus.error = 0U;

  status = SetWriteEnable();

  if (status == ARM_DRIVER_OK) {
    /* Select Slave */
    status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

    if (status == ARM_DRIVER_OK) {
      /* Prepare command */
      cmd = CMD_BULK_ERASE;

      status = ptrSPI->Send(&cmd, 1U);

      if (status == ARM_DRIVER_OK) {
        /* Wait until command is sent */
        while (ptrSPI->GetDataCount() != 1U);
      }
    }
    /* Deselect Slave */
    ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  }

  return status;
}

/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS GetStatus (void) {
  uint8_t val;

  if (FlashStatus.busy == 1U) {
    /* Read flag status register */
    if (ReadStatusReg (CMD_READ_FLAG_STATUS, &val) == ARM_DRIVER_OK) {
      /* Check "Program or erase controller" bit */
      if ((val & 0x80U) != 0U) {
        FlashStatus.busy = 0U;
      }

      /* Check Erase and Program bits */
      if ((val & 0x30) == 0U) {
        FlashStatus.error = 0U;
      } else {
        FlashStatus.error = 1U;
      }
    }
    else {
      FlashStatus.error = 1U;
    }
  }

  return FlashStatus;
}

/**
  \fn          ARM_FLASH_INFO * ARM_Flash_GetInfo (void)
  \brief       Get Flash information.
  \return      Pointer to Flash information \ref ARM_FLASH_INFO
*/
static ARM_FLASH_INFO * GetInfo (void) {
  return &FlashInfo;
}


/* Flash Driver Control Block */
extern
ARM_DRIVER_FLASH ARM_Driver_Flash_(DRIVER_FLASH_NUM);
ARM_DRIVER_FLASH ARM_Driver_Flash_(DRIVER_FLASH_NUM) = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  ReadData,
  ProgramData,
  EraseSector,
  EraseChip,
  GetStatus,
  GetInfo
};
