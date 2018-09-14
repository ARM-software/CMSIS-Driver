/*
 * Copyright (c) 2018 Arm Limited. All rights reserved.
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
 * $Date:        20. February 2018
 * $Revision:    V1.0.1
 *
 * Driver:       Driver_SPI# (default: Driver_SPI0)
 * Project:      SPI Master to Multi-Slave Wrapper
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_SPI# = n (default: 0)
 * -------------------------------------------------------------------- */

#include <string.h>
#include "cmsis_compiler.h"
#include "cmsis_os2.h"
#include "SPI_MultiSlave.h"
#include "SPI_MultiSlave_Config.h"

/* Determine if load/store exclusive is available */
#ifndef SPI_EXCLUSIVE_ACCESS
#if    ((defined(__ARM_ARCH_7M__)      && (__ARM_ARCH_7M__      != 0)) || \
        (defined(__ARM_ARCH_7EM__)     && (__ARM_ARCH_7EM__     != 0)) || \
        (defined(__ARM_ARCH_8M_BASE__) && (__ARM_ARCH_8M_BASE__ != 0)) || \
        (defined(__ARM_ARCH_8M_MAIN__) && (__ARM_ARCH_8M_MAIN__ != 0)))
#define SPI_EXCLUSIVE_ACCESS    (1)
#else
#define SPI_EXCLUSIVE_ACCESS    (0)
#endif
#endif

/* Multislave implementation enable */
#define SPI_MULTISLAVE_EN      (SPI_ENABLE_SLAVE_0 + SPI_ENABLE_SLAVE_1 + \
                                SPI_ENABLE_SLAVE_2 + SPI_ENABLE_SLAVE_3)
/* State flags */
#define SPI_INIT               ((uint8_t)0x01)
#define SPI_POWER              ((uint8_t)0x02)

/* Expand Driver_SPI_(x) to Driver_SPIx (where x is the #define) */
#define  Driver_SPIx_(n)  Driver_SPI##n
#define  Driver_SPI_(n)   Driver_SPIx_(n)

/* SPI Transfer (Run-Time) */
typedef struct {
  void    *data_out;                    /* Output (tx) data buffer    */
  void    *data_in;                     /* Input (rx) data buffer     */
  uint32_t num;                         /* Number of data to transfer */
  uint32_t cnt;                         /* Number of data transferred */
} SPI_XFER;

/* SPI Control (Run-Time) */
typedef struct {
  uint32_t mode;                        /* Configured mode             */
  uint32_t bus_speed;                   /* Configured bus speed        */
  uint16_t tx_value;                    /* Configured default tx value */
  uint16_t ss;                          /* Slave select state          */
} SPI_CONTROL;

/* SPI Status (Run-Time) */
typedef struct {
  uint8_t          state;               /* State flags                */
  uint8_t volatile pend;                /* Transfer is pending        */
  uint8_t volatile busy;                /* Transfer in progress       */
  uint8_t          rsvd;
} SPI_STATUS;

/* SPI Info (Run-Time) */
typedef struct {
  ARM_SPI_SignalEvent_t cb_event;       /* Event callback function */
  SPI_XFER              xfer;           /* Transfer info           */
  SPI_CONTROL           control;        /* Control info            */
  SPI_STATUS            status;         /* Status info             */
} SPI_INFO;

/* SPI Resources */
typedef const struct {
  SPI_INFO *info;                       /* Run-time information */
  uint32_t  instance;                   /* Slave instance       */
} SPI_RESOURCES;

/* SPI Wrapper State */
typedef struct SPI_State {
  SPI_RESOURCES   *Active;              /* Currently active instance     */
  SPI_RESOURCES   *Queue[4];            /* Transfer queue                */
  uint8_t          Qin;                 /* Transfer queue in index       */
  uint8_t          Qout;                /* Transfer queue out index      */
  uint8_t          InitState;           /* State of Init/Uninit calls    */
  uint8_t          PowerState;          /* State of Power Full/Off calls */
  uint8_t          XferLock;            /* Transfer lock state           */
  uint8_t          CallLock;            /* Func call sequence lock state */
  uint16_t         TxValue;             /* Configured default tx value   */
  uint32_t         BusSpeed;            /* Configured bus speed          */
  uint32_t         Mode;                /* Configured mode               */
} SPI_STATE;

/* Exported drivers */
#if (SPI_ENABLE_SLAVE_0 != 0)
extern ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_0);
#endif
#if (SPI_ENABLE_SLAVE_1 != 0)
extern ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_1);
#endif
#if (SPI_ENABLE_SLAVE_2 != 0)
extern ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_2);
#endif
#if (SPI_ENABLE_SLAVE_3 != 0)
extern ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_3);
#endif

/* Reference to underlying CMSIS SPI driver */
extern ARM_DRIVER_SPI                   Driver_SPI_(SPI_DRIVER);
#define Driver_SPI                    (&Driver_SPI_(SPI_DRIVER))

/* Static function prototypes */
#if (SPI_MULTISLAVE_EN != 0)
static uint32_t       LockSet      (uint8_t *lock);
static void           LockClr      (uint8_t *lock);
static uint32_t       LockEnter    (SPI_RESOURCES *spi);
static void           LockExit     (void);
static void           XferEnqueue  (SPI_RESOURCES *spi);
static SPI_RESOURCES *XferDequeue  (void);
static int32_t        ReloadConfig (SPI_RESOURCES *spi);

static ARM_DRIVER_VERSION   SPI_GetVersion      (void);
static ARM_SPI_CAPABILITIES SPI_GetCapabilities (void);
static int32_t              SPI_Initialize      (ARM_SPI_SignalEvent_t cb_event, const SPI_RESOURCES *spi);
static int32_t              SPI_Uninitialize    (const SPI_RESOURCES *spi);
static int32_t              SPI_PowerControl    (ARM_POWER_STATE state, const SPI_RESOURCES *spi);
static int32_t              SPI_Send            (const void *data, uint32_t num, const SPI_RESOURCES *spi);
static int32_t              SPI_Receive         (void *data, uint32_t num, const SPI_RESOURCES *spi);
static int32_t              SPI_Transfer        (const void *data_out, void *data_in, uint32_t num, const SPI_RESOURCES *spi);
static uint32_t             SPI_GetDataCount    (const SPI_RESOURCES *spi);
static int32_t              SPI_Control         (uint32_t control, uint32_t arg, const SPI_RESOURCES *spi);
static ARM_SPI_STATUS       SPI_GetStatus       (const SPI_RESOURCES *spi);
#endif

/* Slave 0 Resources */
#if (SPI_ENABLE_SLAVE_0 != 0)
static SPI_INFO SPI0_Info = {
  0U, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
static const SPI_RESOURCES SPI0_Resources = {
  &SPI0_Info,
  0 /* Slave device number */
};
#endif

/* Slave 1 Resources */
#if (SPI_ENABLE_SLAVE_1 != 0)
static SPI_INFO SPI1_Info = {
  0U, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
static const SPI_RESOURCES SPI1_Resources = {
  &SPI1_Info,
  1 /* Slave device number */
};
#endif

/* Slave 2 Resources */
#if (SPI_ENABLE_SLAVE_2 != 0)
static SPI_INFO SPI2_Info = {
  0U, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
static const SPI_RESOURCES SPI2_Resources = {
  &SPI2_Info,
  2 /* Slave device number */
};
#endif

/* Slave 3 Resources */
#if (SPI_ENABLE_SLAVE_3 != 0)
static SPI_INFO SPI3_Info = {
  0U, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
static const SPI_RESOURCES SPI3_Resources = {
  &SPI3_Info,
  3 /* Slave device number */
};
#endif

#if (SPI_MULTISLAVE_EN != 0)

/* Wrapper State */
static SPI_STATE SPI = {
  0U, { 0U, 0U, 0U, 0U }, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};


#if (SPI_EXCLUSIVE_ACCESS != 0)
#if defined(__CC_ARM)
static __asm uint32_t LockSet_Ex (uint8_t *lock) {
  mov    r2,#1
1
  ldrexb r1,[r0]
  cbz    r1,%F2
  clrex
  movs   r0,#0
  bx     lr
2
  strexb r1,r2,[r0]
  cmp    r1,#0
  bne    %B1
  movs   r0,#1
  bx     lr
}

static __asm void LockClr_Ex (uint8_t *lock) {
  mov    r2,#0
1
  ldrexb r1,[r0]
  strexb r1,r2,[r0]
  cbz    r1,%F2
  b      %B1
2
  bx     lr
}

static __asm uint8_t Qincrement_Ex (uint8_t *addr) {
  mov    r2,r0
1
  ldrexb r0,[r2]
  adds   r1,r0,#1
  and    r1,r1,#3
  strexb r3,r1,[r2]
  cbz    r3,%F2
  b      %B1
2
  bx     lr
}
#else
static uint32_t LockSet_Ex (uint8_t *lock) {
  uint32_t ex;
  uint32_t val;

  val = 1U;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n"
#endif
  "1:\n\t"
    "ldrexb %[ex], [%[lock]]\n\t"
    "cbz    %[ex], 2f\n\t"
    "clrex\n\t"
    "movs   %[ex], #0\n\t"
    "b      3f\n"
  "2:\n\t"
    "strexb %[ex], %[val], [%[lock]]\n\t"
    "cmp    %[ex], #0\n"
    "bne    1b\n\t"
    "movs   %[ex], #1\n"
  "3:"
  : [ex] "=&l" (ex)
  : [lock] "l" (lock), [val] "l" (val)
  : "memory"
  );

  return (ex);
}

static void LockClr_Ex (uint8_t *lock) {
  uint32_t ex;
  uint32_t val;

  val = 0U;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n"
#endif
  "1:\n\t"
    "ldrexb %[ex],[%[lock]]\n\t"
    "strexb %[ex],%[val],[%[lock]]\n\t"
    "cbz    %[ex],2f\n\t"
    "b       1b\n"
  "2:"
  : [ex] "=&l" (ex)
  : [lock] "l" (lock), [val] "l" (val)
  : "memory"
  );
}

static uint8_t Qincrement_Ex (uint8_t *addr) {
  uint32_t ex, idx;
  uint8_t in;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n"
#endif
  "1:\n\t"
    "ldrexb %[in],[%[addr]]\n\t"
    "add    %[idx],%[in],#1\n\t"
    "and    %[idx],%[idx],#3\n\t"
    "strexb %[ex],%[idx],[%[addr]]\n\t"
    "cmp    %[ex],#0\n\t"
    "bne    1b"
  : [in]  "=&l" (in),
    [idx] "=&l" (idx),
    [ex]  "=&l" (ex)
  : [addr]  "l" (addr)
  : "memory"
  );

  return (in);
}
#endif
#endif /* (SPI_EXCLUSIVE_ACCESS != 0) */


/**
  Set lock flag to 1 if its current value is 0.
  
  \param[in]  *lock      Pointer to lock variable
  \return      lock state (0:not locked, 1:locked)
*/
__INLINE static uint32_t LockSet (uint8_t *lock) {
#if (SPI_EXCLUSIVE_ACCESS == 0)
  uint32_t rval;
  uint32_t primask = __get_PRIMASK();

  rval = 1U;

  __disable_irq();

  if (*lock == 1U) {
    rval = 0U;
  } else {
    *lock = 1U;
  }

  if (primask == 0U) {
    __enable_irq();
  }
  return (rval);
#else
/*
  do {
    if (__LDREXB (lock) != 0U) {
      __CLREX ();
      return 0U;
    }
  } while (__STREXB (1U, lock));
  return 1U;
*/
  return LockSet_Ex(lock);
#endif
}


/**
  Clear lock flag to 0.

  \param[in]  *lock      Pointer to lock variable
*/
__INLINE static void LockClr (uint8_t *lock) {
#if (SPI_EXCLUSIVE_ACCESS == 0)
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  *lock = 0U;

  if (primask == 0U) {
    __enable_irq();
  }
#else
/*
  do {__LDREXB (lock);
  } while (__STREXB (0U, lock));
*/
  LockClr_Ex(lock);
#endif
}


/**
  Lock transfer access by setting lock flag and active instance.

  \param[in]   spi       Pointer to SPI resources
  \return      lock state (0:not locked, 1:locked)
*/
static uint32_t LockEnter (SPI_RESOURCES *spi) {
  uint32_t lock;

  lock = LockSet(&SPI.XferLock);

  if (lock == 0U) {
    /* Already locked */
    if (spi == SPI.Active) {
      /* Same instance as active */
      lock = 1U;
    }
  } else {
    /* Acquired lock */
    SPI.Active = spi;
  }

  return (lock);
}


/**
  Unlock transfer access by clearing lock flag and active instance.
*/
static void LockExit (void) {
  /* Clear active instance */
  SPI.Active = NULL;

  /* Clear transfer lock */
  LockClr(&SPI.XferLock);
}


/**
  Put transfer into the queue

  \param[in]   spi       Pointer to SPI resources
*/
static void XferEnqueue (SPI_RESOURCES *spi) {
  uint8_t in;
#if (SPI_EXCLUSIVE_ACCESS == 0)
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  in = SPI.Qin;

  SPI.Qin += 1U;
  SPI.Qin &= 3U;
  if (primask == 0U) {
    __enable_irq();
  }
#else
  /* Atomic Qin increment:
    do {
      in = __LDREXB(&SPI.Qin);
    } while (__STREXB ((in + 1U) & 3U, &SPI.Qin));
  */
  in = Qincrement_Ex (&SPI.Qin);
#endif

  SPI.Queue[in] = spi;
}


/**
  Get transfer from the queue and set it as active.

  \return      pointer to active SPI instance
*/
static SPI_RESOURCES *XferDequeue (void) {
  SPI_RESOURCES *spi;

  if (SPI.Qin != SPI.Qout) {
    spi = SPI.Queue[SPI.Qout];

    SPI.Qout += 1U;
    SPI.Qout &= 3U;
  }
  else {
    spi = NULL;
  }

  SPI.Active = spi;

  return (spi);
}


/**
  Reload bus configuration.

  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t ReloadConfig (SPI_RESOURCES *spi) {
  SPI_CONTROL *c;
  uint32_t control;
  int32_t  status;

  status = ARM_DRIVER_OK;

  c = &spi->info->control;  /* Current instance */

  /* Check for configuration differences */
  if (c->mode != SPI.Mode) {
    /* Configure mode */
    SPI.Mode     = c->mode;
    SPI.BusSpeed = c->bus_speed;

    control  = c->mode;
    control &= ~ARM_SPI_SS_MASTER_MODE_Msk;
    control |=  ARM_SPI_SS_MASTER_UNUSED;

    status = Driver_SPI->Control (control, c->bus_speed);

    if (status != ARM_DRIVER_OK) {
      SPI.Mode     = 0U;
      SPI.BusSpeed = 0U;
    }
  }
  else {
    if (c->bus_speed != SPI.BusSpeed) {
      /* Configure the SPI bus speed */
      SPI.BusSpeed = c->bus_speed;

      status = Driver_SPI->Control (ARM_SPI_SET_BUS_SPEED, c->bus_speed);

      if (status != ARM_DRIVER_OK) {
        SPI.BusSpeed = 0U;
      }
    }
  }

  if (status == ARM_DRIVER_OK) {
    if (c->tx_value != SPI.TxValue) {
      /* Set default TX value */
      SPI.TxValue = c->tx_value;

      status = Driver_SPI->Control (ARM_SPI_SET_DEFAULT_TX_VALUE, c->tx_value);

      if (status != ARM_DRIVER_OK) {
        SPI.TxValue = 0U;
      }
    }
  }

  return (status);
}


/**
  Common event callback function.

  \param[in]   event \ref SPI_events notification mask
*/
static void SPI_MultiSlaveEvent (uint32_t event) {
  int32_t        status;
  SPI_XFER      *xfer;
  SPI_RESOURCES *spi = SPI.Active;

  spi->info->status.busy = 0U;
  spi->info->status.pend = 0U;

  /* Call slave specific callback */
  if (spi->info->cb_event) {
    spi->info->cb_event (event);
  }

  if (spi->info->status.busy == 0U) {
    /* Update number of transfered data */
    spi->info->xfer.cnt = Driver_SPI->GetDataCount();

    /* New transfer not set, check SS state */
    if (spi->info->control.ss == ARM_SPI_SS_INACTIVE) {
      /* Slave select is inactive, check for pending transfer */
      spi = XferDequeue();

      if (spi != NULL) {
        xfer = &spi->info->xfer;

        /* Start enqueued transfer */
        if ((xfer->data_out != NULL) && (xfer->data_in != NULL)) {
          status = SPI_Transfer (xfer->data_out, xfer->data_in, xfer->num, spi);
        }
        else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
          status = SPI_Receive (xfer->data_in, xfer->num, spi);
        }
        else {
          status = SPI_Send (xfer->data_out, xfer->num, spi);
        }

        if (status != ARM_DRIVER_OK) {
          /* Put transfer back into the queue */
          XferEnqueue(spi);
        }
      }
      else {
        /* No pending transfer */
        LockExit();
      }
    }
  }
}


/**
  Get driver version.

  \return \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION SPI_GetVersion (void) {
  return (Driver_SPI->GetVersion());
}


/**
  Get driver capabilities.

  \return \ref ARM_SPI_CAPABILITIES
*/
static ARM_SPI_CAPABILITIES SPI_GetCapabilities (void) {
  return (Driver_SPI->GetCapabilities());
}


/**
  Initialize SPI Interface.

  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Initialize (ARM_SPI_SignalEvent_t cb_event, const SPI_RESOURCES *spi) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  if ((spi->info->status.state & SPI_INIT) == 0U) {
    /* Clear run-time resources */
    memset (spi->info, 0, sizeof(SPI_INFO));

    /* Register application callback */
    spi->info->cb_event = cb_event;

    do {
      lock = LockSet(&SPI.CallLock);

      if (lock == 1U) {
        /* Increment the number of initializations */
        SPI.InitState += 1U;

        if (SPI.InitState == 1U) {
          /* Initialize resources */
          SPI.Active     = NULL;
          SPI.Qin        = 0U;
          SPI.Qout       = 0U;
          SPI.PowerState = 0U;
          SPI.TxValue    = 0U;
          SPI.BusSpeed   = 0U;
          SPI.Mode       = 0U;

          /* Call the underlying driver */
          status = Driver_SPI->Initialize (SPI_MultiSlaveEvent);
        }

        if (status == ARM_DRIVER_OK) {
          spi->info->status.state = SPI_INIT;
        } else {
          SPI.InitState = 0U;
        }

        LockClr (&SPI.CallLock);
      }
      else {
        osThreadYield();
      }
    } while (lock == 0U);
  }

  return (status);
}


/**
  De-initialize SPI Interface.

  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Uninitialize (const SPI_RESOURCES *spi) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  do {
    lock = LockSet (&SPI.CallLock);

    if (lock == 1U) {
      if (SPI.InitState > 0U) {
        /* Decrement the number of initializations */
        SPI.InitState -= 1U;

        if (SPI.InitState == 0U) {
          /* Call the underlying driver */
          status = Driver_SPI->Uninitialize();
        }
      }
      LockClr (&SPI.CallLock);
    }
    else {
      osThreadYield();
    }
  } while (lock == 0U);

  spi->info->status.state = 0U;

  return (status);
}


/**
  Control SPI Interface Power.

  \param[in]   state     Power state
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_PowerControl (ARM_POWER_STATE state, const SPI_RESOURCES *spi) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  switch (state) {
    case ARM_POWER_OFF:
      spi->info->status.state &= ~SPI_POWER;

      do {
        lock = LockSet (&SPI.CallLock);

        if (lock == 1U) {
          if (SPI.PowerState > 0U) {
            SPI.PowerState -= 1U;

            if (SPI.PowerState == 0U) {
              status = Driver_SPI->PowerControl (state);
            }
          }
          LockClr (&SPI.CallLock);
        }
        else {
          osThreadYield();
        }
      } while (lock == 0U);
      break;

    case ARM_POWER_FULL:
      if ((spi->info->status.state & SPI_INIT) == 0U) {
        status = ARM_DRIVER_ERROR;
      }
      else {
        do {
          lock = LockSet (&SPI.CallLock);

          if (lock == 1U) {
            SPI.PowerState += 1U;

            if (SPI.PowerState == 1U) {
              status = Driver_SPI->PowerControl (state);
            }
            LockClr (&SPI.CallLock);
          }
          else {
            osThreadYield();
          }
        } while (lock == 0U);

        /* Deselect slave */
        SPI_Control_SlaveSelect (spi->instance, ARM_SPI_SS_INACTIVE);

        if (status == ARM_DRIVER_OK) {
          spi->info->status.state |= SPI_POWER;
        }
      }
      break;

    case ARM_POWER_LOW:
      status = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }

  return (status);
}


/**
  Start sending data to SPI transmitter.

  \param[in]   data      Pointer to buffer with data to send to SPI transmitter
  \param[in]   num       Number of data items to send
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Send (const void *data, uint32_t num, const SPI_RESOURCES *spi) {
  SPI_XFER *xfer;
  int32_t   status;

  if ((data == NULL) || (num == 0U)) {
    status = ARM_DRIVER_ERROR_PARAMETER;
  }
  else {
    if (LockEnter(spi) == 0U) {
      /* Unable to lock */
      if (spi->info->status.pend != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        status = ARM_DRIVER_OK;

        spi->info->status.pend = 1U;

        /* Enqueue transfer */
        spi->info->xfer.data_out = (void *)(uint32_t)data;
        spi->info->xfer.data_in  = NULL;
        spi->info->xfer.num      = num;
        spi->info->xfer.cnt      = 0U;

        XferEnqueue (spi);

        /* Check if we can transfer immediately */
        if (LockEnter(spi) == 1U) {
          spi = XferDequeue();

          if (spi != NULL) {
            xfer = &spi->info->xfer;

            /* Start enqueued transfer */
            if ((xfer->data_out != NULL) && (xfer->data_in != NULL)) {
              status = SPI_Transfer (xfer->data_out, xfer->data_in, xfer->num, spi);
            }
            else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
              status = SPI_Receive (xfer->data_in, xfer->num, spi);
            }
            else {
              status = SPI_Send (xfer->data_out, xfer->num, spi);
            }
          }
        }
      }
    }
    else {
      if (spi->info->status.busy != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        /* Reload configuration */
        status = ReloadConfig (spi);

        spi->info->status.busy = 1U;
        spi->info->status.pend = 0U;

        if (status == ARM_DRIVER_OK) {
          SPI_Control_SlaveSelect (spi->instance, spi->info->control.ss);

          status = Driver_SPI->Send (data, num);
        }

        if (status != ARM_DRIVER_OK) {
          /* Reset busy flag */
          spi->info->status.busy = 0U;

          /* Unlock access */
          LockExit();
        }
      }
    }
  }

  return (status);
}


/**
  Start receiving data from SPI receiver.

  \param[out]  data      Pointer to buffer for data to receive from SPI receiver
  \param[in]   num       Number of data items to receive
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Receive (void *data, uint32_t num, const SPI_RESOURCES *spi) {
  SPI_XFER *xfer;
  int32_t   status;

  if ((data == NULL) || (num == 0U)) {
    status = ARM_DRIVER_ERROR_PARAMETER;
  }
  else {
    if (LockEnter(spi) == 0U) {
      /* Unable to lock */
      if (spi->info->status.pend != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        status = ARM_DRIVER_OK;

        spi->info->status.pend = 1U;

        /* Enqueue transfer */
        spi->info->xfer.data_out = NULL;
        spi->info->xfer.data_in  = data;
        spi->info->xfer.num      = num;
        spi->info->xfer.cnt      = 0U;

        XferEnqueue (spi);

        /* Check if we can transfer immediately */
        if (LockEnter(spi) == 1U) {
          spi = XferDequeue();

          if (spi != NULL) {
            xfer = &spi->info->xfer;

            /* Start enqueued transfer */
            if ((xfer->data_out != NULL) && (xfer->data_in != NULL)) {
              status = SPI_Transfer (xfer->data_out, xfer->data_in, xfer->num, spi);
            }
            else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
              status = SPI_Receive (xfer->data_in, xfer->num, spi);
            }
            else {
              status = SPI_Send (xfer->data_out, xfer->num, spi);
            }
          }
        }
      }
    }
    else {
      if (spi->info->status.busy != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        /* Reload configuration */
        status = ReloadConfig (spi);

        spi->info->status.busy = 1U;
        spi->info->status.pend = 0U;

        if (status == ARM_DRIVER_OK) {
          SPI_Control_SlaveSelect (spi->instance, spi->info->control.ss);

          status = Driver_SPI->Receive (data, num);
        }

        if (status != ARM_DRIVER_OK) {
          /* Reset busy flag */
          spi->info->status.busy = 0U;

          /* Unlock access */
          LockExit();
        }
      }
    }
  }

  return (status);
}


/**
  Start sending/receiving data to/from SPI transmitter/receiver.

  \param[in]   data_out  Pointer to buffer with data to send to SPI transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SPI receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Transfer (const void *data_out, void *data_in, uint32_t num, const SPI_RESOURCES *spi) {
  SPI_XFER *xfer;
  int32_t   status;

  if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) {
    status = ARM_DRIVER_ERROR_PARAMETER;
  }
  else {
    if (LockEnter(spi) == 0U) {
      /* Unable to lock */
      if (spi->info->status.pend != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        status = ARM_DRIVER_OK;

        spi->info->status.pend = 1U;

        /* Enqueue transfer */
        spi->info->xfer.data_out = (void *)(uint32_t)data_out;
        spi->info->xfer.data_in  = (void *)(uint32_t)data_in;
        spi->info->xfer.num      = num;
        spi->info->xfer.cnt      = 0U;

        XferEnqueue (spi);

        /* Check if we can transfer immediately */
        if (LockEnter(spi) == 1U) {
          spi = XferDequeue();

          if (spi != NULL) {
            xfer = &spi->info->xfer;

            /* Start enqueued transfer */
            if ((xfer->data_out != NULL) && (xfer->data_in != NULL)) {
              status = SPI_Transfer (xfer->data_out, xfer->data_in, xfer->num, spi);
            }
            else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
              status = SPI_Receive (xfer->data_in, xfer->num, spi);
            }
            else {
              status = SPI_Send (xfer->data_out, xfer->num, spi);
            }
          }
        }
      }
    }
    else {
      if (spi->info->status.busy != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        /* Reload configuration */
        status = ReloadConfig (spi);

        spi->info->status.busy = 1U;
        spi->info->status.pend = 0U;

        if (status == ARM_DRIVER_OK) {
          SPI_Control_SlaveSelect (spi->instance, spi->info->control.ss);

          status = Driver_SPI->Transfer (data_out, data_in, num);
        }

        if (status != ARM_DRIVER_OK) {
          /* Reset busy flag */
          spi->info->status.busy = 0U;

          /* Unlock access */
          LockExit();
        }
      }
    }
  }

  return (status);
}


/**
  Get transferred data count.

  \param[in]   spi       Pointer to SPI resources
  \return      number of data items transferred
*/
static uint32_t SPI_GetDataCount (const SPI_RESOURCES *spi) {

  if (spi == SPI.Active) {
    spi->info->xfer.cnt = Driver_SPI->GetDataCount();
  }

  return (spi->info->xfer.cnt);
}


/**
  Control SPI Interface.

  \param[in]   control   operation
  \param[in]   arg       argument of operation (optional)
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Control (uint32_t control, uint32_t arg, const SPI_RESOURCES *spi) {
  SPI_XFER *xfer;
  int32_t  status;
  uint32_t lock, ctrl, val;
  uint32_t pend; /* Pending transfer check flag */

  if ((spi->info->status.state & SPI_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  lock = LockEnter(spi);

  pend = 0U;
  ctrl = control & ARM_SPI_CONTROL_Msk;

  if (ctrl == ARM_SPI_ABORT_TRANSFER) {
    /* Abort transfer */
    if (lock != 0U) {
      status = Driver_SPI->Control (control, arg);
    } else {
      status = ARM_DRIVER_OK;
    }

    /* Clear number of transfered data */
    spi->info->xfer.cnt    = 0U;
    spi->info->status.busy = 0U;
    spi->info->status.pend = 0U;
  }
  else if ((spi->info->status.busy == 1U) || (spi->info->status.pend == 1U)) {
    /* Transfer is in progress, return busy */
    status = ARM_DRIVER_ERROR_BUSY;
  }
  else if (ctrl == ARM_SPI_CONTROL_SS) {
    /* Control slave select pin */
    val = spi->info->control.mode & ARM_SPI_CONTROL_Msk;

    if (val == ARM_SPI_MODE_MASTER) {
      /* Master mode */
      val = spi->info->control.mode & ARM_SPI_SS_MASTER_MODE_Msk;

      if (val == ARM_SPI_SS_MASTER_SW) {
        status = ARM_DRIVER_OK;

        /* Save current SS state */
        spi->info->control.ss = (uint16_t)arg;

        if (arg == ARM_SPI_SS_INACTIVE) {
          if (lock != 0U) {
            /* Deactivate active slave */
            SPI_Control_SlaveSelect (spi->instance, ARM_SPI_SS_INACTIVE);

            /* Check for pending transfers */
            pend = 1U;
          }
        }
      }
      else {
        /* Not software controlled SS */
        status = ARM_DRIVER_ERROR;
      }
    }
    else {
      /* Slave mode */
      status = ARM_DRIVER_ERROR;
    }
  }
  else if (ctrl == ARM_SPI_SET_DEFAULT_TX_VALUE) {
    /* Set default TX value */
    spi->info->control.tx_value = (uint16_t)(arg & 0xFFFFU);

    if (lock == 0U) {
      status = ARM_DRIVER_OK;
    }
    else {
      status = Driver_SPI->Control (control, arg);

      if (status == ARM_DRIVER_OK) {
        SPI.TxValue = (uint16_t)(arg & 0xFFFFU);
      } else {
        SPI.TxValue = 0U;
      }
    }
  }
  else if (ctrl == ARM_SPI_SET_BUS_SPEED) {
    /* Configure the SPI bus speed */
    spi->info->control.bus_speed = arg;

    if (lock == 0U) {
      status = ARM_DRIVER_OK;
    }
    else {
      status = Driver_SPI->Control (control, arg);

      if (status == ARM_DRIVER_OK) {
        SPI.BusSpeed = arg;
      } else {
        SPI.BusSpeed = 0U;
      }
    }
  }
  else if (ctrl == ARM_SPI_GET_BUS_SPEED) {
    /* Return the SPI bus speed */
    if (lock != 0U) {
      spi->info->control.bus_speed = (uint32_t)Driver_SPI->Control (ARM_SPI_GET_BUS_SPEED, 0U);
    }
    status = (int32_t)spi->info->control.bus_speed;
  }
  else {
    /* Configure mode */
    spi->info->control.mode      = control;
    spi->info->control.bus_speed = arg;

    if (lock == 0U) {
      status = ARM_DRIVER_OK;
    }
    else {
      control &= ~ARM_SPI_SS_MASTER_MODE_Msk;
      control |=  ARM_SPI_SS_MASTER_UNUSED;

      status = Driver_SPI->Control (control, arg);

      if (status == ARM_DRIVER_OK) {
        SPI.Mode     = control;
        SPI.BusSpeed = arg;
      } else {
        SPI.Mode     = 0U;
        SPI.BusSpeed = 0U;
      }
    }
  }

  if (lock != 0U) {
    if (pend == 0U) {
      LockExit();
    }
    else {
      /* Check for enqueued transfer */
      spi = XferDequeue();

      if (spi != NULL) {
        xfer = &spi->info->xfer;

        /* Start enqueued transfer */
        if ((xfer->data_out != NULL) && (xfer->data_in != NULL)) {
          status = SPI_Transfer (xfer->data_out, xfer->data_in, xfer->num, spi);
        }
        else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
          status = SPI_Receive (xfer->data_in, xfer->num, spi);
        }
        else {
          status = SPI_Send (xfer->data_out, xfer->num, spi);
        }
      }
      else {
        LockExit();
      }
    }
  }

  return (status);
}


/**
  Get SPI status.

  \param[in]   spi       Pointer to SPI resources
  \return      SPI status \ref ARM_SPI_STATUS
*/
static ARM_SPI_STATUS SPI_GetStatus (const SPI_RESOURCES *spi) {
  ARM_SPI_STATUS status;

  status.busy       = spi->info->status.busy | spi->info->status.pend;
  status.data_lost  = 0U;
  status.mode_fault = 0U;

  return (status);
}


/* SPI Driver Slave 0 Instance Definition */
#if (SPI_ENABLE_SLAVE_0 != 0)
static int32_t SPI0_Initialize (ARM_SPI_SignalEvent_t cb_event) {
  return SPI_Initialize (cb_event, &SPI0_Resources);
}
static int32_t SPI0_Uninitialize (void) {
  return SPI_Uninitialize (&SPI0_Resources);
}
static int32_t SPI0_PowerControl (ARM_POWER_STATE state) {
  return SPI_PowerControl (state, &SPI0_Resources);
}
static int32_t SPI0_Send (const void *data, uint32_t num) {
  return SPI_Send (data, num, &SPI0_Resources);
}
static int32_t SPI0_Receive (void *data, uint32_t num) {
return SPI_Receive (data, num, &SPI0_Resources); }

static int32_t SPI0_Transfer (const void *data_out, void *data_in, uint32_t num) {
  return SPI_Transfer (data_out, data_in, num, &SPI0_Resources);
}
static uint32_t SPI0_GetDataCount (void) {
  return SPI_GetDataCount (&SPI0_Resources);
}
static int32_t SPI0_Control (uint32_t control, uint32_t arg) {
  return SPI_Control (control, arg, &SPI0_Resources);
}
static ARM_SPI_STATUS SPI0_GetStatus (void) {
  return SPI_GetStatus (&SPI0_Resources);
}

ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_0) = {
  SPI_GetVersion,
  SPI_GetCapabilities,
  SPI0_Initialize,
  SPI0_Uninitialize,
  SPI0_PowerControl,
  SPI0_Send,
  SPI0_Receive,
  SPI0_Transfer,
  SPI0_GetDataCount,
  SPI0_Control,
  SPI0_GetStatus
};
#endif


/* SPI Driver Slave 1 Instance Definition */
#if (SPI_ENABLE_SLAVE_1 != 0)
static int32_t SPI1_Initialize (ARM_SPI_SignalEvent_t cb_event) {
  return SPI_Initialize (cb_event, &SPI1_Resources);
}
static int32_t SPI1_Uninitialize (void) {
  return SPI_Uninitialize (&SPI1_Resources);
}
static int32_t SPI1_PowerControl (ARM_POWER_STATE state) {
  return SPI_PowerControl (state, &SPI1_Resources);
}
static int32_t SPI1_Send (const void *data, uint32_t num) {
  return SPI_Send (data, num, &SPI1_Resources);
}
static int32_t SPI1_Receive (void *data, uint32_t num) {
return SPI_Receive (data, num, &SPI1_Resources); }

static int32_t SPI1_Transfer (const void *data_out, void *data_in, uint32_t num) {
  return SPI_Transfer (data_out, data_in, num, &SPI1_Resources);
}
static uint32_t SPI1_GetDataCount (void) {
  return SPI_GetDataCount (&SPI1_Resources);
}
static int32_t SPI1_Control (uint32_t control, uint32_t arg) {
  return SPI_Control (control, arg, &SPI1_Resources);
}
static ARM_SPI_STATUS SPI1_GetStatus (void) {
  return SPI_GetStatus (&SPI1_Resources);
}

ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_1) = {
  SPI_GetVersion,
  SPI_GetCapabilities,
  SPI1_Initialize,
  SPI1_Uninitialize,
  SPI1_PowerControl,
  SPI1_Send,
  SPI1_Receive,
  SPI1_Transfer,
  SPI1_GetDataCount,
  SPI1_Control,
  SPI1_GetStatus
};
#endif


/* SPI Driver Slave 2 Instance Definition */
#if (SPI_ENABLE_SLAVE_2 != 0)
static int32_t SPI2_Initialize (ARM_SPI_SignalEvent_t cb_event) {
  return SPI_Initialize (cb_event, &SPI2_Resources);
}
static int32_t SPI2_Uninitialize (void) {
  return SPI_Uninitialize (&SPI2_Resources);
}
static int32_t SPI2_PowerControl (ARM_POWER_STATE state) {
  return SPI_PowerControl (state, &SPI2_Resources);
}
static int32_t SPI2_Send (const void *data, uint32_t num) {
  return SPI_Send (data, num, &SPI2_Resources);
}
static int32_t SPI2_Receive (void *data, uint32_t num) {
return SPI_Receive (data, num, &SPI2_Resources); }

static int32_t SPI2_Transfer (const void *data_out, void *data_in, uint32_t num) {
  return SPI_Transfer (data_out, data_in, num, &SPI2_Resources);
}
static uint32_t SPI2_GetDataCount (void) {
  return SPI_GetDataCount (&SPI2_Resources);
}
static int32_t SPI2_Control (uint32_t control, uint32_t arg) {
  return SPI_Control (control, arg, &SPI2_Resources);
}
static ARM_SPI_STATUS SPI2_GetStatus (void) {
  return SPI_GetStatus (&SPI2_Resources);
}

ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_2) = {
  SPI_GetVersion,
  SPI_GetCapabilities,
  SPI2_Initialize,
  SPI2_Uninitialize,
  SPI2_PowerControl,
  SPI2_Send,
  SPI2_Receive,
  SPI2_Transfer,
  SPI2_GetDataCount,
  SPI2_Control,
  SPI2_GetStatus
};
#endif


/* SPI Driver Slave 3 Instance Definition */
#if (SPI_ENABLE_SLAVE_3 != 0)
static int32_t SPI3_Initialize (ARM_SPI_SignalEvent_t cb_event) {
  return SPI_Initialize (cb_event, &SPI3_Resources);
}
static int32_t SPI3_Uninitialize (void) {
  return SPI_Uninitialize (&SPI3_Resources);
}
static int32_t SPI3_PowerControl (ARM_POWER_STATE state) {
  return SPI_PowerControl (state, &SPI3_Resources);
}
static int32_t SPI3_Send (const void *data, uint32_t num) {
  return SPI_Send (data, num, &SPI3_Resources);
}
static int32_t SPI3_Receive (void *data, uint32_t num) {
return SPI_Receive (data, num, &SPI3_Resources); }

static int32_t SPI3_Transfer (const void *data_out, void *data_in, uint32_t num) {
  return SPI_Transfer (data_out, data_in, num, &SPI3_Resources);
}
static uint32_t SPI3_GetDataCount (void) {
  return SPI_GetDataCount (&SPI3_Resources);
}
static int32_t SPI3_Control (uint32_t control, uint32_t arg) {
  return SPI_Control (control, arg, &SPI3_Resources);
}
static ARM_SPI_STATUS SPI3_GetStatus (void) {
  return SPI_GetStatus (&SPI3_Resources);
}

ARM_DRIVER_SPI Driver_SPI_(SPI_DRIVER_SLAVE_3) = {
  SPI_GetVersion,
  SPI_GetCapabilities,
  SPI3_Initialize,
  SPI3_Uninitialize,
  SPI3_PowerControl,
  SPI3_Send,
  SPI3_Receive,
  SPI3_Transfer,
  SPI3_GetDataCount,
  SPI3_Control,
  SPI3_GetStatus
};
#endif
#endif /* SPI_MULTISLAVE_EN */
