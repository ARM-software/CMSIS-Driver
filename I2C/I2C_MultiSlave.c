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
 * Driver:       Driver_I2C# (default: Driver_I2C0)
 * Project:      I2C Master to Multi-Slave Wrapper
 * -----------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value
 *   ---------------------                   -----
 *   Connect to hardware via Driver_I2C# = n (default: 0)
 * -------------------------------------------------------------------- */

#include <string.h>
#include "cmsis_compiler.h"
#include "cmsis_os2.h"
#include "Driver_I2C.h"
#include "I2C_MultiSlave_Config.h"

/* Determine if load/store exclusive is available */
#ifndef I2C_EXCLUSIVE_ACCESS
#if    ((defined(__ARM_ARCH_7M__)      && (__ARM_ARCH_7M__      != 0)) || \
        (defined(__ARM_ARCH_7EM__)     && (__ARM_ARCH_7EM__     != 0)) || \
        (defined(__ARM_ARCH_8M_BASE__) && (__ARM_ARCH_8M_BASE__ != 0)) || \
        (defined(__ARM_ARCH_8M_MAIN__) && (__ARM_ARCH_8M_MAIN__ != 0)))
#define I2C_EXCLUSIVE_ACCESS    (1)
#else
#define I2C_EXCLUSIVE_ACCESS    (0)
#endif
#endif

/* Multislave implementation enable */
#define I2C_MULTISLAVE_EN      (I2C_ENABLE_SLAVE_0 + I2C_ENABLE_SLAVE_1 + \
                                I2C_ENABLE_SLAVE_2 + I2C_ENABLE_SLAVE_3 + \
                                I2C_ENABLE_SLAVE_4 + I2C_ENABLE_SLAVE_5 + \
                                I2C_ENABLE_SLAVE_6 + I2C_ENABLE_SLAVE_7)
/* State flags */
#define I2C_INIT               ((uint8_t)0x01)
#define I2C_POWER              ((uint8_t)0x02)

/* Expand Driver_I2C_(x) to Driver_I2Cx (where x is the #define) */
#define  Driver_I2Cx_(n)  Driver_I2C##n
#define  Driver_I2C_(n)   Driver_I2Cx_(n)

/* I2C Transfer (Run-Time) */
typedef struct {
  uint32_t addr;                        /* Device address             */
  const
  uint8_t *data_out;                    /* Transmit data buffer       */
  uint8_t *data_in;                     /* Receive data buffer        */
  uint32_t num;                         /* Number of data to transfer */
  int32_t  cnt;                         /* Number of data transferred */
  uint8_t  pending;                     /* Transfer pending flag      */
  uint8_t  rsvd;                        /* Reserved                   */
  uint16_t bus_speed;                   /* Configured bus speed       */
} I2C_XFER;

/* I2C Status (Run-Time) */
typedef struct {
  uint8_t          state;               /* State flags                    */
  uint8_t volatile pend;                /* Transfer is pending            */
  uint8_t volatile busy;                /* Transfer in progress           */
  uint8_t          dir;                 /* Transfer direction (0:Tx,1:Rx) */
} I2C_STATUS;

/* I2C Resources (Run-Time) */
typedef struct {
  ARM_I2C_SignalEvent_t cb_event;       /* Event callback function */
  I2C_XFER              xfer;           /* Transfer info           */
  I2C_STATUS            status;         /* Status info             */
} I2C_RESOURCES;

/* I2C Wrapper State */
typedef struct I2C_State {
  I2C_RESOURCES   *Active;              /* Currently active instance     */
  I2C_RESOURCES   *Queue[8];            /* Transfer queue                */
  uint8_t          Qin;                 /* Transfer queue in index       */
  uint8_t          Qout;                /* Transfer queue out index      */
  uint8_t          InitState;           /* State of Init/Uninit calls    */
  uint8_t          PowerState;          /* State of Power Full/Off calls */
  uint8_t          XferLock;            /* Transfer lock state           */
  uint8_t          CallLock;            /* Func call sequence lock state */
  uint16_t         BusSpeed;            /* Configured bus speed          */
} I2C_STATE;

/* Exported drivers */
#if (I2C_ENABLE_SLAVE_0 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_0);
#endif
#if (I2C_ENABLE_SLAVE_1 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_1);
#endif
#if (I2C_ENABLE_SLAVE_2 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_2);
#endif
#if (I2C_ENABLE_SLAVE_3 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_3);
#endif
#if (I2C_ENABLE_SLAVE_4 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_4);
#endif
#if (I2C_ENABLE_SLAVE_5 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_5);
#endif
#if (I2C_ENABLE_SLAVE_6 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_6);
#endif
#if (I2C_ENABLE_SLAVE_7 != 0)
extern ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_7);
#endif

/* Reference to underlying CMSIS I2C driver */
extern ARM_DRIVER_I2C                   Driver_I2C_(I2C_DRIVER);
#define Driver_I2C                    (&Driver_I2C_(I2C_DRIVER))

/* Static function prototypes */
#if (I2C_MULTISLAVE_EN != 0)
static uint32_t       LockSet      (uint8_t *lock);
static void           LockClr      (uint8_t *lock);
static uint32_t       LockEnter    (I2C_RESOURCES *i2c);
static void           LockExit     (void);
static void           XferEnqueue  (I2C_RESOURCES *i2c);
static I2C_RESOURCES *XferDequeue  (void);
static int32_t        ReloadConfig (I2C_RESOURCES *i2c);

static ARM_DRIVER_VERSION   I2C_GetVersion      (void);
static ARM_I2C_CAPABILITIES I2C_GetCapabilities (void);
static int32_t              I2C_Initialize      (ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c);
static int32_t              I2C_Uninitialize    (I2C_RESOURCES *i2c);
static int32_t              I2C_PowerControl    (ARM_POWER_STATE state, I2C_RESOURCES *i2c);
static int32_t              I2C_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending, I2C_RESOURCES *i2c);
static int32_t              I2C_MasterReceive   (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending, I2C_RESOURCES *i2c);
static int32_t              I2C_SlaveTransmit   (const uint8_t *data, uint32_t num, I2C_RESOURCES *i2c);
static int32_t              I2C_SlaveReceive    (      uint8_t *data, uint32_t num, I2C_RESOURCES *i2c);
static int32_t              I2C_GetDataCount    (I2C_RESOURCES *i2c);
static int32_t              I2C_Control         (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c);
static ARM_I2C_STATUS       I2C_GetStatus       (I2C_RESOURCES *i2c);
#endif

/* Slave 0 Resources */
#if (I2C_ENABLE_SLAVE_0 != 0)
static I2C_RESOURCES I2C0_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 1 Resources */
#if (I2C_ENABLE_SLAVE_1 != 0)
static I2C_RESOURCES I2C1_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 2 Resources */
#if (I2C_ENABLE_SLAVE_2 != 0)
static I2C_RESOURCES I2C2_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 3 Resources */
#if (I2C_ENABLE_SLAVE_3 != 0)
static I2C_RESOURCES I2C3_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 4 Resources */
#if (I2C_ENABLE_SLAVE_4 != 0)
static I2C_RESOURCES I2C4_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 5 Resources */
#if (I2C_ENABLE_SLAVE_5 != 0)
static I2C_RESOURCES I2C5_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 6 Resources */
#if (I2C_ENABLE_SLAVE_6 != 0)
static I2C_RESOURCES I2C6_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

/* Slave 7 Resources */
#if (I2C_ENABLE_SLAVE_7 != 0)
static I2C_RESOURCES I2C7_Resources = {
  0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, { 0U, 0U, 0U, 0U }
};
#endif

#if (I2C_MULTISLAVE_EN != 0)

/* Wrapper State */
static I2C_STATE I2C = {
  0U, {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};


#if (I2C_EXCLUSIVE_ACCESS != 0)
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
  and    r1,r1,#7
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
    "and    %[idx],%[idx],#7\n\t"
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
#endif /* (I2C_EXCLUSIVE_ACCESS != 0) */


/**
  Set lock flag to 1 if its current value is 0.
  
  \param[in]  *lock      Pointer to lock variable
  \return      lock state (0:not locked, 1:locked)
*/
__INLINE static uint32_t LockSet (uint8_t *lock) {
#if (I2C_EXCLUSIVE_ACCESS == 0)
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
#if (I2C_EXCLUSIVE_ACCESS == 0)
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

  \param[in]   i2c       Pointer to I2C resources
  \return      lock state (0:not locked, 1:locked)
*/
static uint32_t LockEnter (I2C_RESOURCES *i2c) {
  uint32_t lock;

  lock = LockSet(&I2C.XferLock);

  if (lock == 0U) {
    /* Already locked */
    if (i2c == I2C.Active) {
      /* Same instance as active */
      lock = 1U;
    }
  } else {
    /* Acquired lock */
    I2C.Active = i2c;
  }

  return (lock);
}


/**
  Unlock transfer access by clearing lock flag and active instance.
*/
static void LockExit (void) {
  /* Clear active instance */
  I2C.Active = NULL;

  /* Clear transfer lock */
  LockClr(&I2C.XferLock);
}


/**
  Put transfer into the queue

  \param[in]   i2c       Pointer to I2C resources
*/
static void XferEnqueue (I2C_RESOURCES *i2c) {
  uint8_t in;
#if (I2C_EXCLUSIVE_ACCESS == 0)
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  in = I2C.Qin;

  I2C.Qin += 1U;
  I2C.Qin &= 3U;
  if (primask == 0U) {
    __enable_irq();
  }
#else
  /* Atomic Qin increment:
    do {
      in = __LDREXB(&I2C.Qin);
    } while (__STREXB ((in + 1U) & 7U, &I2C.Qin));
  */
  in = Qincrement_Ex (&I2C.Qin);
#endif

  I2C.Queue[in] = i2c;
}


/**
  Get transfer from the queue and set it as active.

  \return      pointer to active I2C instance
*/
static I2C_RESOURCES *XferDequeue (void) {
  I2C_RESOURCES *i2c;

  if (I2C.Qin != I2C.Qout) {
    i2c = I2C.Queue[I2C.Qout];

    I2C.Qout += 1U;
    I2C.Qout &= 7U;
  }
  else {
    i2c = NULL;
  }

  I2C.Active = i2c;

  return (i2c);
}


/**
  Reload bus configuration.

  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t ReloadConfig (I2C_RESOURCES *i2c) {
  int32_t status;

  status = ARM_DRIVER_OK;

  if (i2c->xfer.bus_speed != I2C.BusSpeed) {
    status = Driver_I2C->Control (ARM_I2C_BUS_SPEED, i2c->xfer.bus_speed);

    if (status == ARM_DRIVER_OK) {
      I2C.BusSpeed = i2c->xfer.bus_speed;
    } else {
      I2C.BusSpeed = 0U;
    }
  }

  return (status);
}


/**
  Common event callback function.

  \param[in]   event \ref I2C_events notification mask
*/
static void I2C_MultiSlaveEvent (uint32_t event) {
  int32_t        status;
  I2C_XFER      *xfer;
  I2C_RESOURCES *i2c = I2C.Active;

  i2c->status.busy = 0U;
  i2c->status.pend = 0U;

  /* Call slave specific callback */
  if (i2c->cb_event) {
    i2c->cb_event (event);
  }

  if (i2c->status.busy == 0U) {
    /* Update number of transfered data */
    i2c->xfer.cnt = Driver_I2C->GetDataCount();

    /* New transfer not set, check if any transfer pending */
    i2c = XferDequeue();

    if (i2c != NULL) {
      xfer = &i2c->xfer;

      /* Start enqueued transfer */
      if ((xfer->data_out != NULL) && (xfer->data_in == NULL)) {
        status = I2C_MasterTransmit (xfer->addr, xfer->data_out, xfer->num, xfer->pending, i2c);
      }
      else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
        status = I2C_MasterReceive (xfer->addr, xfer->data_in, xfer->num, xfer->pending, i2c);
      }
      else {
        status = ARM_DRIVER_ERROR;
      }

      if (status != ARM_DRIVER_OK) {
        /* Put transfer back into the queue */
        XferEnqueue(i2c);
      }
    }
    else {
      /* No pending transfer */
      LockExit();
    }
  }
}


/**
  Get driver version.

  \return \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION I2C_GetVersion (void) {
  return (Driver_I2C->GetVersion());
}


/**
  Get driver capabilities.

  \return \ref ARM_I2C_CAPABILITIES
*/
static ARM_I2C_CAPABILITIES I2C_GetCapabilities (void) {
  return (Driver_I2C->GetCapabilities());
}


/**
  Initialize I2C Interface.

  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2C_Initialize (ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  if ((i2c->status.state & I2C_INIT) == 0U) {
    /* Clear run-time resources */
    memset ((void *)i2c, 0, sizeof(I2C_RESOURCES));

    /* Register application callback */
    i2c->cb_event = cb_event;

    do {
      lock = LockSet(&I2C.CallLock);

      if (lock == 1U) {
        /* Increment the number of initializations */
        I2C.InitState += 1U;

        if (I2C.InitState == 1U) {
          /* Initialize resources */
          I2C.Active     = NULL;
          I2C.Qin        = 0U;
          I2C.Qout       = 0U;
          I2C.PowerState = 0U;
          I2C.BusSpeed   = 0U;

          /* Call the underlying driver */
          status = Driver_I2C->Initialize (I2C_MultiSlaveEvent);
        }

        if (status == ARM_DRIVER_OK) {
          i2c->status.state = I2C_INIT;
        } else {
          I2C.InitState = 0U;
        }

        LockClr (&I2C.CallLock);
      }
      else {
        osThreadYield();
      }
    } while (lock == 0U);
  }

  return (status);
}


/**
  De-initialize I2C Interface.

  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2C_Uninitialize (I2C_RESOURCES *i2c) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  do {
    lock = LockSet (&I2C.CallLock);

    if (lock == 1U) {
      if (I2C.InitState > 0U) {
        /* Decrement the number of initializations */
        I2C.InitState -= 1U;

        if (I2C.InitState == 0U) {
          /* Call the underlying driver */
          status = Driver_I2C->Uninitialize();
        }
      }
      LockClr (&I2C.CallLock);
    }
    else {
      osThreadYield();
    }
  } while (lock == 0U);

  i2c->status.state = 0U;

  return (status);
}


/**
  Control I2C Interface Power.

  \param[in]   state     Power state
  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2C_PowerControl (ARM_POWER_STATE state, I2C_RESOURCES *i2c) {
  int32_t status;
  uint32_t lock;

  status = ARM_DRIVER_OK;

  switch (state) {
    case ARM_POWER_OFF:
      i2c->status.state &= ~I2C_POWER;

      do {
        lock = LockSet (&I2C.CallLock);

        if (lock == 1U) {
          if (I2C.PowerState > 0U) {
            I2C.PowerState -= 1U;

            if (I2C.PowerState == 0U) {
              status = Driver_I2C->PowerControl (state);
            }
          }
          LockClr (&I2C.CallLock);
        }
        else {
          osThreadYield();
        }
      } while (lock == 0U);
      break;

    case ARM_POWER_FULL:
      if ((i2c->status.state & I2C_INIT) == 0U) {
        status = ARM_DRIVER_ERROR;
      }
      else {
        do {
          lock = LockSet (&I2C.CallLock);

          if (lock == 1U) {
            I2C.PowerState += 1U;

            if (I2C.PowerState == 1U) {
              status = Driver_I2C->PowerControl (state);
            }
            LockClr (&I2C.CallLock);
          }
          else {
            osThreadYield();
          }
        } while (lock == 0U);

        if (status == ARM_DRIVER_OK) {
          i2c->status.state |= I2C_POWER;
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
  Start transmitting data as I2C Master.

  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[in]   data          Pointer to buffer with data to transmit to I2C Slave
  \param[in]   num           Number of data bytes to transmit
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status
*/
static int32_t I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending, I2C_RESOURCES *i2c) {
  I2C_XFER *xfer;
  int32_t   status;

  if ((data == NULL) || (num == 0U)) {
    status = ARM_DRIVER_ERROR_PARAMETER;
  }
  else {
    if (LockEnter(i2c) == 0U) {
      /* Unable to lock */
      if (i2c->status.pend != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        status = ARM_DRIVER_OK;

        /* Set status */
        i2c->status.pend = 1U;
        i2c->status.dir  = 0U;

        /* Enqueue transfer */
        i2c->xfer.addr     = addr;
        i2c->xfer.data_out = data;
        i2c->xfer.data_in  = NULL;
        i2c->xfer.num      = num;
        i2c->xfer.cnt      = 0U;
        i2c->xfer.pending  = xfer_pending;

        XferEnqueue (i2c);

        /* Check if we can transfer immediately */
        if (LockEnter(i2c) == 1U) {
          i2c = XferDequeue();

          if (i2c != NULL) {
            xfer = &i2c->xfer;

            /* Start enqueued transfer */
            if ((xfer->data_out != NULL) && (xfer->data_in == NULL)) {
              status = I2C_MasterTransmit (xfer->addr, xfer->data_out, xfer->num, xfer->pending, i2c);
            }
            else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
              status = I2C_MasterReceive (xfer->addr, xfer->data_in, xfer->num, xfer->pending, i2c);
            }
            else {
              status = ARM_DRIVER_ERROR;
            }

            if (status != ARM_DRIVER_OK) {
              /* Put transfer back into the queue */
              XferEnqueue(i2c);
            }
          }
          else {
            /* No pending transfer */
            LockExit();
          }
        }
      }
    }
    else {
      if (i2c->status.busy != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        /* Reload configuration */
        status = ReloadConfig (i2c);

        i2c->status.busy = 1U;
        i2c->status.pend = 0U;

        if (status == ARM_DRIVER_OK) {
          status = Driver_I2C->MasterTransmit (addr, data, num, xfer_pending);
        }

        if (status != ARM_DRIVER_OK) {
          /* Reset busy flag */
          i2c->status.busy = 0U;

          /* Unlock access */
          LockExit();
        }
      }
    }
  }

  return (status);
}


/**
  Start receiving data as I2C Master.

  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from I2C Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status
*/
static int32_t I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending, I2C_RESOURCES *i2c) {
  I2C_XFER *xfer;
  int32_t   status;

  if ((data == NULL) || (num == 0U)) {
    status = ARM_DRIVER_ERROR_PARAMETER;
  }
  else {
    if (LockEnter(i2c) == 0U) {
      /* Unable to lock */
      if (i2c->status.pend != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        status = ARM_DRIVER_OK;

        i2c->status.pend = 1U;
        i2c->status.dir  = 1U;

        /* Enqueue transfer */
        i2c->xfer.addr     = addr;
        i2c->xfer.data_out = data;
        i2c->xfer.data_in  = NULL;
        i2c->xfer.num      = num;
        i2c->xfer.cnt      = 0U;
        i2c->xfer.pending  = xfer_pending;

        XferEnqueue (i2c);

        /* Check if we can transfer immediately */
        if (LockEnter(i2c) == 1U) {
          i2c = XferDequeue();

          if (i2c != NULL) {
            xfer = &i2c->xfer;

            /* Start enqueued transfer */
            if ((xfer->data_out != NULL) && (xfer->data_in == NULL)) {
              status = I2C_MasterTransmit (xfer->addr, xfer->data_out, xfer->num, xfer->pending, i2c);
            }
            else if ((xfer->data_out == NULL) && (xfer->data_in != NULL)) {
              status = I2C_MasterReceive (xfer->addr, xfer->data_in, xfer->num, xfer->pending, i2c);
            }
            else {
              status = ARM_DRIVER_ERROR;
            }

            if (status != ARM_DRIVER_OK) {
              /* Put transfer back into the queue */
              XferEnqueue(i2c);
            }
          }
          else {
            /* No pending transfer */
            LockExit();
          }
        }
      }
    }
    else {
      if (i2c->status.busy != 0U) {
        status = ARM_DRIVER_ERROR_BUSY;
      }
      else {
        /* Reload configuration */
        status = ReloadConfig (i2c);

        i2c->status.busy = 1U;
        i2c->status.pend = 0U;

        if (status == ARM_DRIVER_OK) {
          status = Driver_I2C->MasterReceive (addr, data, num, xfer_pending);
        }

        if (status != ARM_DRIVER_OK) {
          /* Reset busy flag */
          i2c->status.busy = 0U;

          /* Unlock access */
          LockExit();
        }
      }
    }
  }

  return (status);
}


/**
  Start transmitting data as I2C Slave.

  \param[in]   data  Pointer to buffer with data to transmit to I2C Master
  \param[in]   num   Number of data bytes to transmit
  \return      \ref execution_status
*/
static int32_t I2C_SlaveTransmit (const uint8_t *data, uint32_t num, I2C_RESOURCES *i2c) {
  (void)data;
  (void)num;
  (void)i2c;

  return ARM_DRIVER_ERROR;
}


/**
  Start receiving data as I2C Slave.

  \param[out]  data  Pointer to buffer for data to receive from I2C Master
  \param[in]   num   Number of data bytes to receive
  \return      \ref execution_status
*/
static int32_t I2C_SlaveReceive (uint8_t *data, uint32_t num, I2C_RESOURCES *i2c) {
  (void)data;
  (void)num;
  (void)i2c;

  return ARM_DRIVER_ERROR;
}


/**
  Get transferred data count.

  \param[in]   i2c       Pointer to I2C resources
  \return      number of data items transferred
*/
static int32_t I2C_GetDataCount (I2C_RESOURCES *i2c) {

  if (i2c == I2C.Active) {
    i2c->xfer.cnt = Driver_I2C->GetDataCount();
  }

  return (i2c->xfer.cnt);
}


/**
  Control I2C Interface.

  \param[in]   control   operation
  \param[in]   arg       argument of operation (optional)
  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2C_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c) {
  int32_t  status;
  uint32_t lock;

  if ((i2c->status.state & I2C_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  lock = LockEnter(i2c);

  if (control == ARM_I2C_ABORT_TRANSFER) {
    /* Abort transfer */
    if (lock != 0U) {
      status = Driver_I2C->Control (control, arg);
    } else {
      status = ARM_DRIVER_OK;
    }

    /* Clear number of transfered data */
    i2c->xfer.cnt    = 0U;
    i2c->status.busy = 0U;
    i2c->status.pend = 0U;
  }
  else if ((i2c->status.busy == 1U) || (i2c->status.pend == 1U)) {
    /* Transfer is in progress, return busy */
    status = ARM_DRIVER_ERROR_BUSY;
  }
  else if (control == ARM_I2C_BUS_CLEAR) {
    if (lock != 0U) {
      status = Driver_I2C->Control (control, arg);
    } else {
      status = ARM_DRIVER_OK;
    }
  }
  else if (control == ARM_I2C_BUS_SPEED) {
    /* Set bus speed */
    i2c->xfer.bus_speed = (uint16_t)arg;

    status = ARM_DRIVER_OK;

    if (lock != 0U) {
      if ((arg < I2C.BusSpeed) || (I2C.BusSpeed == 0U)) {
        status = Driver_I2C->Control (control, arg);

        if (status == ARM_DRIVER_OK) {
          I2C.BusSpeed = (uint16_t)arg;
        }
      }
    }
  }
  else {
    status = ARM_DRIVER_ERROR;
  }

  if (lock != 0U) {
    LockExit();
  }

  return (status);
}


/**
  Get I2C status.

  \param[in]   i2c       Pointer to I2C resources
  \return      I2C status \ref ARM_I2C_STATUS
*/
static ARM_I2C_STATUS I2C_GetStatus (I2C_RESOURCES *i2c) {
  ARM_I2C_STATUS out, in;

  if (i2c == I2C.Active) {
    in = Driver_I2C->GetStatus();

    out.arbitration_lost = in.arbitration_lost;
    out.bus_error        = in.bus_error;
  } else {
    out.arbitration_lost = 0U;
    out.bus_error        = 0U;
  }

  out.busy             = i2c->status.busy | i2c->status.pend;
  out.mode             = 1;/*Master*/
  out.direction        = i2c->status.dir;

  return (out);
}


/* I2C Driver Slave 0 Instance Definition */
#if (I2C_ENABLE_SLAVE_0 != 0)
static int32_t I2C0_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C0_Resources);
}
static int32_t I2C0_Uninitialize (void) {
  return I2C_Uninitialize (&I2C0_Resources);
}
static int32_t I2C0_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C0_Resources);
}
static int32_t I2C0_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C0_Resources);
}
static int32_t I2C0_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C0_Resources);
}
static int32_t I2C0_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C0_Resources);
}
static int32_t I2C0_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C0_Resources);
}
static int32_t I2C0_GetDataCount (void) {
  return I2C_GetDataCount (&I2C0_Resources);
}
static int32_t I2C0_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C0_Resources);
}
static ARM_I2C_STATUS I2C0_GetStatus (void) {
  return I2C_GetStatus (&I2C0_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_0) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C0_Initialize,
  I2C0_Uninitialize,
  I2C0_PowerControl,
  I2C0_MasterTransmit,
  I2C0_MasterReceive,
  I2C0_SlaveTransmit,
  I2C0_SlaveReceive,
  I2C0_GetDataCount,
  I2C0_Control,
  I2C0_GetStatus
};
#endif


/* I2C Driver Slave 1 Instance Definition */
#if (I2C_ENABLE_SLAVE_1 != 0)
static int32_t I2C1_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C1_Resources);
}
static int32_t I2C1_Uninitialize (void) {
  return I2C_Uninitialize (&I2C1_Resources);
}
static int32_t I2C1_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C1_Resources);
}
static int32_t I2C1_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C1_Resources);
}
static int32_t I2C1_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C1_Resources);
}
static int32_t I2C1_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C1_Resources);
}
static int32_t I2C1_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C1_Resources);
}
static int32_t I2C1_GetDataCount (void) {
  return I2C_GetDataCount (&I2C1_Resources);
}
static int32_t I2C1_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C1_Resources);
}
static ARM_I2C_STATUS I2C1_GetStatus (void) {
  return I2C_GetStatus (&I2C1_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_1) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_MasterTransmit,
  I2C1_MasterReceive,
  I2C1_SlaveTransmit,
  I2C1_SlaveReceive,
  I2C1_GetDataCount,
  I2C1_Control,
  I2C1_GetStatus
};
#endif


/* I2C Driver Slave 2 Instance Definition */
#if (I2C_ENABLE_SLAVE_2 != 0)
static int32_t I2C2_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C2_Resources);
}
static int32_t I2C2_Uninitialize (void) {
  return I2C_Uninitialize (&I2C2_Resources);
}
static int32_t I2C2_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C2_Resources);
}
static int32_t I2C2_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C2_Resources);
}
static int32_t I2C2_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C2_Resources);
}
static int32_t I2C2_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C2_Resources);
}
static int32_t I2C2_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C2_Resources);
}
static int32_t I2C2_GetDataCount (void) {
  return I2C_GetDataCount (&I2C2_Resources);
}
static int32_t I2C2_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C2_Resources);
}
static ARM_I2C_STATUS I2C2_GetStatus (void) {
  return I2C_GetStatus (&I2C2_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_2) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C2_Initialize,
  I2C2_Uninitialize,
  I2C2_PowerControl,
  I2C2_MasterTransmit,
  I2C2_MasterReceive,
  I2C2_SlaveTransmit,
  I2C2_SlaveReceive,
  I2C2_GetDataCount,
  I2C2_Control,
  I2C2_GetStatus
};
#endif


/* I2C Driver Slave 3 Instance Definition */
#if (I2C_ENABLE_SLAVE_3 != 0)
static int32_t I2C3_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C3_Resources);
}
static int32_t I2C3_Uninitialize (void) {
  return I2C_Uninitialize (&I2C3_Resources);
}
static int32_t I2C3_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C3_Resources);
}
static int32_t I2C3_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C3_Resources);
}
static int32_t I2C3_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C3_Resources);
}
static int32_t I2C3_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C3_Resources);
}
static int32_t I2C3_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C3_Resources);
}
static int32_t I2C3_GetDataCount (void) {
  return I2C_GetDataCount (&I2C3_Resources);
}
static int32_t I2C3_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C3_Resources);
}
static ARM_I2C_STATUS I2C3_GetStatus (void) {
  return I2C_GetStatus (&I2C3_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_3) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C3_Initialize,
  I2C3_Uninitialize,
  I2C3_PowerControl,
  I2C3_MasterTransmit,
  I2C3_MasterReceive,
  I2C3_SlaveTransmit,
  I2C3_SlaveReceive,
  I2C3_GetDataCount,
  I2C3_Control,
  I2C3_GetStatus
};
#endif


/* I2C Driver Slave 4 Instance Definition */
#if (I2C_ENABLE_SLAVE_4 != 0)
static int32_t I2C4_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C4_Resources);
}
static int32_t I2C4_Uninitialize (void) {
  return I2C_Uninitialize (&I2C4_Resources);
}
static int32_t I2C4_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C4_Resources);
}
static int32_t I2C4_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C4_Resources);
}
static int32_t I2C4_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C4_Resources);
}
static int32_t I2C4_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C4_Resources);
}
static int32_t I2C4_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C4_Resources);
}
static int32_t I2C4_GetDataCount (void) {
  return I2C_GetDataCount (&I2C4_Resources);
}
static int32_t I2C4_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C4_Resources);
}
static ARM_I2C_STATUS I2C4_GetStatus (void) {
  return I2C_GetStatus (&I2C4_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_4) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C4_Initialize,
  I2C4_Uninitialize,
  I2C4_PowerControl,
  I2C4_MasterTransmit,
  I2C4_MasterReceive,
  I2C4_SlaveTransmit,
  I2C4_SlaveReceive,
  I2C4_GetDataCount,
  I2C4_Control,
  I2C4_GetStatus
};
#endif


/* I2C Driver Slave 45nstance Definition */
#if (I2C_ENABLE_SLAVE_5 != 0)
static int32_t I2C5_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C5_Resources);
}
static int32_t I2C5_Uninitialize (void) {
  return I2C_Uninitialize (&I2C5_Resources);
}
static int32_t I2C5_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C5_Resources);
}
static int32_t I2C5_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C5_Resources);
}
static int32_t I2C5_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C5_Resources);
}
static int32_t I2C5_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C5_Resources);
}
static int32_t I2C5_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C5_Resources);
}
static int32_t I2C5_GetDataCount (void) {
  return I2C_GetDataCount (&I2C5_Resources);
}
static int32_t I2C5_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C5_Resources);
}
static ARM_I2C_STATUS I2C5_GetStatus (void) {
  return I2C_GetStatus (&I2C5_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_5) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C5_Initialize,
  I2C5_Uninitialize,
  I2C5_PowerControl,
  I2C5_MasterTransmit,
  I2C5_MasterReceive,
  I2C5_SlaveTransmit,
  I2C5_SlaveReceive,
  I2C5_GetDataCount,
  I2C5_Control,
  I2C5_GetStatus
};
#endif


/* I2C Driver Slave 6 Instance Definition */
#if (I2C_ENABLE_SLAVE_6 != 0)
static int32_t I2C6_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C6_Resources);
}
static int32_t I2C6_Uninitialize (void) {
  return I2C_Uninitialize (&I2C6_Resources);
}
static int32_t I2C6_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C6_Resources);
}
static int32_t I2C6_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C6_Resources);
}
static int32_t I2C6_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C6_Resources);
}
static int32_t I2C6_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C6_Resources);
}
static int32_t I2C6_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C6_Resources);
}
static int32_t I2C6_GetDataCount (void) {
  return I2C_GetDataCount (&I2C6_Resources);
}
static int32_t I2C6_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C6_Resources);
}
static ARM_I2C_STATUS I2C6_GetStatus (void) {
  return I2C_GetStatus (&I2C6_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_6) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C6_Initialize,
  I2C6_Uninitialize,
  I2C6_PowerControl,
  I2C6_MasterTransmit,
  I2C6_MasterReceive,
  I2C6_SlaveTransmit,
  I2C6_SlaveReceive,
  I2C6_GetDataCount,
  I2C6_Control,
  I2C6_GetStatus
};
#endif


/* I2C Driver Slave 7 Instance Definition */
#if (I2C_ENABLE_SLAVE_7 != 0)
static int32_t I2C7_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize (cb_event, &I2C7_Resources);
}
static int32_t I2C7_Uninitialize (void) {
  return I2C_Uninitialize (&I2C7_Resources);
}
static int32_t I2C7_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl (state, &I2C7_Resources);
}
static int32_t I2C7_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C7_Resources);
}
static int32_t I2C7_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive (addr, data, num, xfer_pending, &I2C7_Resources);
}
static int32_t I2C7_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit (data, num, &I2C7_Resources);
}
static int32_t I2C7_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive (data, num, &I2C7_Resources);
}
static int32_t I2C7_GetDataCount (void) {
  return I2C_GetDataCount (&I2C7_Resources);
}
static int32_t I2C7_Control (uint32_t control, uint32_t arg) {
  return I2C_Control (control, arg, &I2C7_Resources);
}
static ARM_I2C_STATUS I2C7_GetStatus (void) {
  return I2C_GetStatus (&I2C7_Resources);
}

ARM_DRIVER_I2C Driver_I2C_(I2C_DRIVER_SLAVE_7) = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C7_Initialize,
  I2C7_Uninitialize,
  I2C7_PowerControl,
  I2C7_MasterTransmit,
  I2C7_MasterReceive,
  I2C7_SlaveTransmit,
  I2C7_SlaveReceive,
  I2C7_GetDataCount,
  I2C7_Control,
  I2C7_GetStatus
};
#endif
#endif /* I2C_MULTISLAVE_EN */
