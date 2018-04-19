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
 * $Revision:    V1.4
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Driver for S29GL064Nx2 (32-bit Bus)
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

    #define FLASH_MANAGE_CACHE #
    Replace symbol # with 0 when data cache management is not needed (no cache
    or cache disabled) or with 1 to enable data cache management.
    Default settings are: 0 when data cache is not present and 1 when data cache
    is present (regardless of the MPU region configuration).

    MPU Region Configuration:

    Flash device memory space can be placed into cacheable region by configuring
    the MPU. Cacheable region parameters must be set as follows in order to
    configure "Write-Through, no write allocate" properties: TEX:000, C:1, B:0.
    Region base address should be set to defined FLASH_ADDR.
 */

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_Flash.h"
#include "S29GL064Nx2.h"

#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,4) /* driver version */


#ifndef DRIVER_FLASH_NUM
#define DRIVER_FLASH_NUM        0           /* Default driver number */
#endif

#ifndef FLASH_ADDR
#define FLASH_ADDR              0x80000000  /* Flash base address */
#endif

#ifndef FLASH_MANAGE_CACHE
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0U)
#define FLASH_MANAGE_CACHE      1           /* Cache management enabled (data cache present) */
#else
#define FLASH_MANAGE_CACHE      0           /* Cache management disabled (no data cache) */
#endif
#endif


/* Enable Data Cache management when present */
#if (FLASH_MANAGE_CACHE != 0)
#define DCACHE_INVALIDATE(addr) SCB_InvalidateDCache_by_Addr((uint32_t *)(addr),32);
#define DMEMORY_BARRIER()       __DMB()
#else
#define DCACHE_INVALIDATE(addr)
#define DMEMORY_BARRIER()
#endif

/* Memory Bus Access Macro */
#define RD_MEM(addr)            (*((volatile uint32_t *) (addr)))
#define WR_MEM(addr, value)     do {                                          \
                                  *((volatile uint32_t *)(addr)) = (value);   \
                                  DMEMORY_BARRIER();                          \
                                } while(0)

/* Flash Base Address */
#define BASE_ADDR               ((uint32_t)FLASH_ADDR)

/* Flash Commands */
#define CMD_RESET               ((uint32_t)0x00F000F0)
#define CMD_ERASE               ((uint32_t)0x00800080)
#define CMD_ERASE_CHIP          ((uint32_t)0x00100010)
#define CMD_ERASE_SECTOR        ((uint32_t)0x00300030)
#define CMD_PROGRAM             ((uint32_t)0x00A000A0)

/* Flash Status Register */
typedef union {
  struct b  {
    uint32_t q0l:1;
    uint32_t q1l:1;
    uint32_t q2l:1;
    uint32_t q3l:1;
    uint32_t q4l:1;
    uint32_t q5l:1;
    uint32_t q6l:1;
    uint32_t q7l:1;
    uint32_t rl:8;
    uint32_t q0h:1;
    uint32_t q1h:1;
    uint32_t q2h:1;
    uint32_t q3h:1;
    uint32_t q4h:1;
    uint32_t q5h:1;
    uint32_t q6h:1;
    uint32_t q7h:1;
    uint32_t rh:8;
  } b;
  uint32_t v;
} fsr_t;

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
  2U,   /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
  1U,   /* erase_chip */
#if (ARM_FLASH_API_VERSION > 0x200U)
  0U    /* reserved */
#endif
};

/* Read Flash Status register */
__STATIC_FORCEINLINE uint32_t RdStatus (uint32_t addr) {
  uint32_t fsreg;

  /* Invalidate data cache */
  DCACHE_INVALIDATE (addr);

  /* Read status register */
  fsreg = RD_MEM(addr);

  return (fsreg);
}

/* Check if Program/Erase completed */
static bool DQ6_Polling (uint32_t addr) {
  fsr_t    fsr;
  uint32_t q6l, q6h;

  fsr.v = RdStatus(addr);
  q6l = fsr.b.q6l;
  q6h = fsr.b.q6h;
  do {
    fsr.v = RdStatus(addr);
    if ((fsr.b.q6l == q6l) && (fsr.b.q6h == q6h)) {
      return true;              /* Done */
    }
    q6l = fsr.b.q6l;
    q6h = fsr.b.q6h;
  } while ((fsr.b.q5l == 0U) || (fsr.b.q5h == 0U));
  fsr.v = RdStatus(addr);
  q6l = fsr.b.q6l;
  q6h = fsr.b.q6h;
  fsr.v = RdStatus(addr);
  if ((fsr.b.q6l == q6l) && (fsr.b.q6h == q6h)) {
    return true;                /* Done */
  }
  WR_MEM(addr, CMD_RESET);      /* Reset Flash Device */
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

  FlashStatus.busy  = 0U;
  FlashStatus.error = 0U;

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

      WR_MEM(BASE_ADDR, CMD_RESET);

      FlashStatus.busy  = 0U;
      FlashStatus.error = 0U;
      break;

    case ARM_POWER_FULL:
      if ((Flags & FLASH_INIT) == 0U) {
        return ARM_DRIVER_ERROR;
      }

      if ((Flags & FLASH_POWER) == 0U) {
        WR_MEM(BASE_ADDR, CMD_RESET);

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
  uint32_t *mem;
  uint32_t  n;

  if (((addr & 3U) != 0U) || (data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (FlashStatus.busy) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  FlashStatus.error = 0U;

  addr += BASE_ADDR;
  mem = data;
  for (n = cnt; n; n--) {
    *mem  = RD_MEM(addr);
    mem  += 1U;
    addr += 4U;
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
  const uint32_t *mem;
        uint32_t  n;

  if (((addr & 3U) != 0U) || (data == NULL)) {
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
    WR_MEM(BASE_ADDR + (0x555UL << 2), 0x00AA00AAU);
    WR_MEM(BASE_ADDR + (0x2AAUL << 2), 0x00550055U);
    WR_MEM(BASE_ADDR + (0x555UL << 2), CMD_PROGRAM);
    WR_MEM(addr, *mem);
    mem  += 1U;
    addr += 4U;
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

  WR_MEM(BASE_ADDR + (0x555UL << 2), 0x00AA00AAU);
  WR_MEM(BASE_ADDR + (0x2AAUL << 2), 0x00550055U);
  WR_MEM(BASE_ADDR + (0x555UL << 2), CMD_ERASE);
  WR_MEM(BASE_ADDR + (0x555UL << 2), 0x00AA00AAU);
  WR_MEM(BASE_ADDR + (0x2AAUL << 2), 0x00550055U);
  WR_MEM(BASE_ADDR +  addr, CMD_ERASE_SECTOR);

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

  WR_MEM(BASE_ADDR + (0x555UL << 2), 0x00AA00AAU);
  WR_MEM(BASE_ADDR + (0x2AAUL << 2), 0x00550055U);
  WR_MEM(BASE_ADDR + (0x555UL << 2), CMD_ERASE);
  WR_MEM(BASE_ADDR + (0x555UL << 2), 0x00AA00AAU);
  WR_MEM(BASE_ADDR + (0x2AAUL << 2), 0x00550055U);
  WR_MEM(BASE_ADDR + (0x555UL << 2), CMD_ERASE_CHIP);

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS GetStatus (void) {
  fsr_t    fsr;
  uint32_t q6l, q6h;
  uint32_t rl, rh;

  rl = 0U;
  rh = 0U;

  if (FlashStatus.busy) {
    fsr.v = RdStatus(BASE_ADDR);
    q6l = fsr.b.q6l;
    q6h = fsr.b.q6h;
    fsr.v = RdStatus(BASE_ADDR);
    if (fsr.b.q6l == q6l) {
      rl = 1U;
    }
    if (fsr.b.q6h == q6h) {
      rh = 1U;
    }
    if (rl && rh) {
      FlashStatus.busy = 0U;
    }
    else {
      if ((!rl && fsr.b.q5l) || (!rh && fsr.b.q5h)) {
        WR_MEM(BASE_ADDR, CMD_RESET);
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
