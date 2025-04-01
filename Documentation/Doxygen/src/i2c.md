# I2C {#page_driver_i2c}

The I2C MultiSlave wrapper [I2C_MultiSlave.c](https://github.com/ARM-software/CMSIS-Driver/blob/main/I2C/I2C_MultiSlave.c) resides on top of an arbitrary I2C CMSIS-Driver and exports maximum of eight I2C CMSIS-Drivers (with I2C Master functionality only). Slave functionalities are disabled and calling slave functions will return with an error. An I2C slave device connects to one of the exported driver and uses it as any other CMSIS I2C driver (in master mode only). The wrapper provides multi-thread protection.

Each slave can use its own bus speed configuration, but the MultiSlave wrapper will limit the bus speed to the lowest requested frequency (assuming that three slaves are present and if two slaves configure bus speed `ARM_I2C_BUS_SPEED_FAST` (400 KHz) and one slave `ARM_I2C_BUS_SPEED_STANDARD` (100 kHz), then the actual bus speed will be `ARM_I2C_BUS_SPEED_STANDARD`).

The wrapper is configured using the [I2C_MultiSlave_Config.h](https://github.com/ARM-software/CMSIS-Driver/blob/main/Config/I2C_MultiSlave_Config.h) file, which contains the following options:

- `#define I2C_DRIVER` specifies the underlying I2C CMSIS-Driver, which controls the I2C peripheral and accesses the bus. The wrapper connects to that driver.
- `#define I2C_ENABLE_SLAVE_x` enables each connected slave on the I2C bus. This basically means that the driver control block `Driver_I2Cn` will be exported by the wrapper for each particular slave.
- `#define I2C_DRIVER_SLAVE_x` sets the exported control block number n, for example `Driver_I2Cn`. The user application connects to this driver.

**Code example**

This is a demo application which demonstrates the usage of the I2C MultiSlave driver wrapper. It consists of two threads that periodically access two I2C slave devices.

```c
#include <string.h>
#include "cmsis_os2.h"

#include "RTE_Components.h"
#include  CMSIS_device_header

#include "Driver_I2C.h"                 // ::CMSIS Driver:I2C

/* Thread prototypes */
static void Thread_A (void *argument);
static void Thread_B (void *argument);
static void app_main (void *argument);

/* A and B Thread IDs */
static osThreadId_t ThreadId_A;
static osThreadId_t ThreadId_B;

/* I2C A Driver, controls Slave Device 0, uses underlying Driver_I2C1 (see I2C_MultiSlave_Config.h) */
extern ARM_DRIVER_I2C         Driver_I2C10;
#define I2C_A               (&Driver_I2C10)

/* I2C B Driver, controls Slave Device 1, uses underlying Driver_I2C1 (see I2C_MultiSlave_Config.h) */
extern ARM_DRIVER_I2C         Driver_I2C11;
#define I2C_B               (&Driver_I2C11)

/*----------------------------------------------------------------------------
 * I2C Thread A
 *---------------------------------------------------------------------------*/
__NO_RETURN static void Thread_A (void *argument) {
  uint8_t addr;
  uint8_t reg;
  uint8_t val;
  (void)argument;

  /* Initialize and configure I2C */
  I2C_A->Initialize  (NULL);
  I2C_A->PowerControl(ARM_POWER_FULL);
  I2C_A->Control     (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2C_A->Control     (ARM_I2C_BUS_CLEAR, 0);

  /* Periodically read device register at address 0x0F */
  addr = 0x68;
  reg  = 0x0F;

  while(1) {
    I2C_A->MasterTransmit(addr, &reg, 1, true);
    while (I2C_A->GetStatus().busy);

    I2C_A->MasterReceive (addr, &val, 1, false);
    while (I2C_B->GetStatus().busy);

    osDelay(10);
  }
}

/*----------------------------------------------------------------------------
 * I2C Thread B
 *---------------------------------------------------------------------------*/
__NO_RETURN static void Thread_B (void *argument) {
  uint8_t addr;
  uint8_t reg;
  uint8_t val;
  (void)argument;

  /* Initialize and configure I2C */
  I2C_B->Initialize  (NULL);
  I2C_B->PowerControl(ARM_POWER_FULL);
  I2C_B->Control     (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
  I2C_B->Control     (ARM_I2C_BUS_CLEAR, 0);

  /* Periodically write device register at address 0x02 */
  addr = 0x44;
  reg  = 0x02;
  val  = 0xA5;

  while(1) {
    I2C_A->MasterTransmit(addr, &reg, 1, true);
    while (I2C_A->GetStatus().busy);

    I2C_A->MasterTransmit(addr, &val, 1, false);
    while (I2C_B->GetStatus().busy);

    osDelay(10);
  }
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN static void app_main (void *argument) {
  (void)argument;

  /* Create SPI threads */
  ThreadId_A = osThreadNew(Thread_A, NULL, NULL);
  ThreadId_B = osThreadNew(Thread_B, NULL, NULL);

  osDelay(osWaitForever);

  for (;;) {}
}

int main (void) {

  // System Initialization
  SystemCoreClockUpdate();

  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
```
