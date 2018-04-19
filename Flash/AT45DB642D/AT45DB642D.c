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
 * $Date:        19. April 2018
 * $Revision:    V1.3
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Driver for Atmel DataFlash AT45DB642D (SPI)
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
 */

#include "Driver_Flash.h"
#include "Driver_SPI.h"
#include "AT45DB642D.h"

#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,3) /* driver version */


#ifndef DRIVER_FLASH_NUM
#define DRIVER_FLASH_NUM        0         /* Default driver number */
#endif

#ifndef DRIVER_SPI_NUM
#define DRIVER_SPI_NUM          0         /* Default SPI driver number */
#endif


/* SPI Data Flash definitions */
#define BUS_SPEED       33000000          /* Bus speed   */
#define PAGE_SIZE       FLASH_PAGE_SIZE   /* Page size   */
#define BLOCK_SIZE      ( 8* PAGE_SIZE)   /* Block size  */
#define SECTOR_SIZE     (16*BLOCK_SIZE)   /* Sector size */

/* SPI Data Flash Commands */
#define CMD_READ_DATA           (0xE8U)
#define CMD_READ_STATUS         (0xD7U)
#define CMD_BLOCK_ERASE         (0x50U)
#define CMD_BUF_WRITE           (0x84U)
#define CMD_PAGE_PROGRAM        (0x83U)
#define CMD_PAGE_READ           (0x53U)

/* Flash Driver Flags */
#define FLASH_INIT              (0x01U)
#define FLASH_POWER             (0x02U)


/* SPI Driver */
#define _SPI_Driver_(n)  Driver_SPI##n
#define  SPI_Driver_(n)  _SPI_Driver_(n)
extern ARM_DRIVER_SPI     SPI_Driver_(DRIVER_SPI_NUM);
#define ptrSPI          (&SPI_Driver_(DRIVER_SPI_NUM))


/* Send command with optional data and wait until busy */
static bool SendCommand (uint8_t cmd, uint32_t addr, const uint8_t *data, uint32_t size) {
  uint32_t page_addr;
  uint32_t page_offs;
  uint8_t  buf[4];
  uint8_t  sr;
  int32_t  result;

  page_addr = addr / PAGE_SIZE;
  page_offs = addr % PAGE_SIZE;

  addr = (page_addr << 11) | page_offs;

  /* Prepare Command with address */
  buf[0] = cmd;
  buf[1] = (uint8_t)(addr >> 16);
  buf[2] = (uint8_t)(addr >>  8);
  buf[3] = (uint8_t)(addr >>  0);

  /* Select Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
  if (result != ARM_DRIVER_OK) return false;

  /* Send Command with address */
  result = ptrSPI->Send(buf, 4);
  if (result != ARM_DRIVER_OK) goto transfer_error;
  while (ptrSPI->GetDataCount() != 4);

  /* Send Data */
  if ((data != NULL) && (size != 0)) {
    result = ptrSPI->Send(data, size);
    if (result != ARM_DRIVER_OK) goto transfer_error;
    while (ptrSPI->GetDataCount() != size);
  }

  /* Deselect Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  if (result != ARM_DRIVER_OK) return false;

  /* Prepare Read Status Command */
  buf[0] = CMD_READ_STATUS;
  buf[1] = 0xFF;                /* Dummy byte */

  /* Select Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
  if (result != ARM_DRIVER_OK) return false;

  /* Send Command */
  result = ptrSPI->Send(buf, 2);
  if (result != ARM_DRIVER_OK) goto transfer_error;
  while (ptrSPI->GetDataCount() != 2);

  /* Check Status Register */
  do {
    result = ptrSPI->Receive(&sr, 1);
    if (result != ARM_DRIVER_OK) goto transfer_error;
    while (ptrSPI->GetDataCount() != 1);
  } while ((sr & 0x80) == 0);

  /* Deselect Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  if (result != ARM_DRIVER_OK) return false;

  return true;

transfer_error:
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  return false;
}


/* Sector Information */
#ifdef FLASH_SECTORS
static ARM_FLASH_SECTOR FLASH_SECTOR_INFO[FLASH_SECTOR_COUNT] = FLASH_SECTORS;
#else
#define FLASH_SECTOR_INFO NULL
#endif

/* Flash Information */
static ARM_FLASH_INFO FlashInfo = {
  FLASH_SECTOR_INFO,
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
  0U,   /* erase_chip */
#if (ARM_FLASH_API_VERSION > 0x200U)
  0U    /* reserved */
#endif
};


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
                                                       ARM_SPI_LSB_MSB      |
                                                       ARM_SPI_SS_MASTER_SW,
                                                       BUS_SPEED);
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
  uint32_t page_addr;
  uint32_t page_offs;
  uint8_t  buf[8];
  int32_t  result;

  if (data == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  page_addr = addr / PAGE_SIZE;
  page_offs = addr % PAGE_SIZE;

  addr = (page_addr << 11) | page_offs;

  /* Prepare Command with address */
  buf[0] = CMD_READ_DATA;
  buf[1] = (uint8_t)(addr >> 16);
  buf[2] = (uint8_t)(addr >>  8);
  buf[3] = (uint8_t)(addr >>  0);
  /* buf[4..7]: don't care */

  /* Select Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
  if (result != ARM_DRIVER_OK) { return ARM_DRIVER_ERROR; }

  /* Send Command with Address */
  result = ptrSPI->Send(buf, 8);
  if (result != ARM_DRIVER_OK) { goto transfer_error; }
  while (ptrSPI->GetDataCount() != 8);

  /* Receive Data */
  result = ptrSPI->Receive(data, cnt);
  if (result != ARM_DRIVER_OK) { goto transfer_error; }
  while (ptrSPI->GetDataCount() != cnt);

  /* Deselect Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  if (result != ARM_DRIVER_OK) { return result; }

  return (int32_t)cnt;

transfer_error:
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  return ARM_DRIVER_ERROR;
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
        uint32_t n;

  buf = data;
  while (cnt) {
    if (!SendCommand(CMD_PAGE_READ, addr, NULL, 0U)) {
      return ARM_DRIVER_ERROR;
    }

    n = PAGE_SIZE - (addr % PAGE_SIZE);
    if (n > cnt) { n = cnt; }
    
    if (!SendCommand(CMD_BUF_WRITE, addr, buf, n)) {
      return ARM_DRIVER_ERROR;
    }
    if (!SendCommand(CMD_PAGE_PROGRAM, addr, NULL, 0U)) {
      return ARM_DRIVER_ERROR;
    }

    addr += n;
    buf  += n;
    cnt  -= n;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_EraseSector (uint32_t addr)
  \brief       Erase Flash Sector.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
static int32_t EraseSector (uint32_t addr) {
  uint32_t n;

  for (n = 0U; n < (SECTOR_SIZE/BLOCK_SIZE); n++) {
    if (!SendCommand(CMD_BLOCK_ERASE, addr, NULL, 0U)) {
      return ARM_DRIVER_ERROR;
    }
    addr += BLOCK_SIZE;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_EraseChip (void)
  \brief       Erase complete Flash.
               Optional function for faster full chip erase.
  \return      \ref execution_status
*/
static int32_t EraseChip (void) {
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS GetStatus (void) {
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
