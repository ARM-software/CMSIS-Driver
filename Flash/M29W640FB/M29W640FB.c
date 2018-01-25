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
 * $Date:        29. June 2016
 * $Revision:    V1.3
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Driver for M29W640FB (16-bit Bus)
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
    
    #define FLASH_ADDR         #
    Replace symbol # with the base address of your Flash device. Check the
    external memory configuration for proper setting.
    Default setting is 0x80000000.
 */

#ifdef __clang__
  #pragma clang diagnostic ignored "-Wpadded"
  #pragma clang diagnostic ignored "-Wmissing-field-initializers"
#endif

#include "Driver_Flash.h"
#include "M29W640FB.h"

#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,3) /* driver version */


#ifndef DRIVER_FLASH_NUM
#define DRIVER_FLASH_NUM        0           /* Default driver number */
#endif

#ifndef FLASH_ADDR
#define FLASH_ADDR              0x80000000  /* Flash base address */
#endif


/* 16-bit Memory Bus Access Macro */
#define M16(addr)               (*((volatile uint16_t *) (addr)))

/* Flash Base Address */
#define BASE_ADDR               ((uint32_t)FLASH_ADDR)

/* Flash Commands */
#define CMD_RESET               ((uint16_t)0xF0)
#define CMD_ERASE               ((uint16_t)0x80)
#define CMD_ERASE_CHIP          ((uint16_t)0x10)
#define CMD_ERASE_SECTOR        ((uint16_t)0x30)
#define CMD_PROGRAM             ((uint16_t)0xA0)

/* Flash Status Register bits */
#define DQ6                     ((uint32_t)(1UL << 6))
#define DQ5                     ((uint32_t)(1UL << 5))

/* Flash Driver Flags */
#define FLASH_INIT              (0x01U)
#define FLASH_POWER             (0x02U)

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
  FLASH_ERASED_VALUE
};

/* Flash Status */
static ARM_FLASH_STATUS FlashStatus = {0};
static uint8_t Flags;


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_FLASH_API_VERSION,
  ARM_FLASH_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_FLASH_CAPABILITIES DriverCapabilities = {
  0U,   /* event_ready */
  1U,   /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
  1U    /* erase_chip */
};


/* Check if Program/Erase completed */
static bool DQ6_Polling (uint32_t addr) {
  uint32_t fsreg;
  uint32_t dqold;

  fsreg = M16(addr);
  do {
    dqold = fsreg & DQ6;
    fsreg = M16(addr);
    if ((fsreg & DQ6) == dqold) {
      return true;              /* Done */
    }
  } while ((fsreg & DQ5) != DQ5);
  fsreg = M16(addr);
  dqold = fsreg & DQ6;
  fsreg = M16(addr);
  if ((fsreg & DQ6) == dqold) {
    return true;                /* Done */
  }
  M16(addr) = CMD_RESET;        /* Reset Flash Device */
  return false;                 /* Error */
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
  (void)cb_event;
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
  return ARM_DRIVER_OK;
} 

/**
  \fn          int32_t ARM_Flash_PowerControl (ARM_POWER_STATE state)
  \brief       Control the Flash interface power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {

  switch ((int32_t)state) {
      case ARM_POWER_OFF:
      Flags &= ~FLASH_POWER;

      M16(BASE_ADDR) = CMD_RESET;

      FlashStatus.busy  = 0U;
      FlashStatus.error = 0U;
      break;

    case ARM_POWER_FULL:
      if ((Flags & FLASH_INIT) == 0U) {
        return ARM_DRIVER_ERROR;
      }

      if ((Flags & FLASH_POWER) == 0U) {
        M16(BASE_ADDR) = CMD_RESET;

        FlashStatus.busy  = 0U;
        FlashStatus.error = 0U;

        Flags |= FLASH_POWER;
      }
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    default:
      return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
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
  uint16_t *mem;
  uint32_t  n;

  if (((addr & 1U) != 0U) || (data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (FlashStatus.busy) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  FlashStatus.error = 0U;

  addr += BASE_ADDR;
  mem = data;
  for (n = cnt; n; n--) {
    *mem  = M16(addr);
    mem  += 1U;
    addr += 2U;
  }

  return (int32_t)cnt;
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
  const uint16_t *mem;
        uint32_t  n;

  if (((addr & 1U) != 0U) || (data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (FlashStatus.busy) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  if (cnt == 0U) { return 0; }

  FlashStatus.busy  = 1U;
  FlashStatus.error = 0U;

  addr += BASE_ADDR;
  mem = data;
  for (n = cnt; n; n--) {
    M16(BASE_ADDR + (0x555UL << 1)) = 0xAAU;
    M16(BASE_ADDR + (0x2AAUL << 1)) = 0x55U;
    M16(BASE_ADDR + (0x555UL << 1)) = CMD_PROGRAM;
    M16(addr) = *mem;
    mem  += 1U;
    addr += 2U;
    if (n > 1U) {
      if (!DQ6_Polling(BASE_ADDR)) {
        FlashStatus.busy  = 0U;
        FlashStatus.error = 1U;
        return ARM_DRIVER_ERROR;
      }
    }
  }

  return ((int32_t)cnt - 1);
}

/**
  \fn          int32_t ARM_Flash_EraseSector (uint32_t addr)
  \brief       Erase Flash Sector.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
static int32_t EraseSector (uint32_t addr) {

  if (FlashStatus.busy) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  FlashStatus.busy  = 1U;
  FlashStatus.error = 0U;

  M16(BASE_ADDR + (0x555UL << 1)) = 0xAAU;
  M16(BASE_ADDR + (0x2AAUL << 1)) = 0x55U;
  M16(BASE_ADDR + (0x555UL << 1)) = CMD_ERASE;
  M16(BASE_ADDR + (0x555UL << 1)) = 0xAAU;
  M16(BASE_ADDR + (0x2AAUL << 1)) = 0x55U;
  M16(BASE_ADDR +  addr)          = CMD_ERASE_SECTOR;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_EraseChip (void)
  \brief       Erase complete Flash.
               Optional function for faster full chip erase.
  \return      \ref execution_status
*/
static int32_t EraseChip (void) {

  if (FlashStatus.busy) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  FlashStatus.busy  = 1U;
  FlashStatus.error = 0U;

  M16(BASE_ADDR + (0x555UL << 1)) = 0xAAU;
  M16(BASE_ADDR + (0x2AAUL << 1)) = 0x55U;
  M16(BASE_ADDR + (0x555UL << 1)) = CMD_ERASE;
  M16(BASE_ADDR + (0x555UL << 1)) = 0xAAU;
  M16(BASE_ADDR + (0x2AAUL << 1)) = 0x55U;
  M16(BASE_ADDR + (0x555UL << 1)) = CMD_ERASE_CHIP;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS GetStatus (void) {
  uint32_t fsreg;
  uint32_t dqold;

  if (FlashStatus.busy) {
    fsreg = M16(BASE_ADDR);
    dqold = fsreg & DQ6;
    fsreg = M16(BASE_ADDR);
    if ((fsreg & DQ6) == dqold) {
      FlashStatus.busy = 0U;
    } else {
      if (fsreg & DQ5) {
        M16(BASE_ADDR) = CMD_RESET;
        FlashStatus.busy  = 0U;
        FlashStatus.error = 1U;
      }
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
