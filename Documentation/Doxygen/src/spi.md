# SPI {#page_driver_spi}

The SPI MultiSlave wrapper [SPI_MultiSlave.c](https://github.com/ARM-software/CMSIS-Driver/blob/main/SPI/SPI_MultiSlave.c) resides on top of an arbitrary SPI CMSIS-Driver and exports a maximum of four SPI CMSIS-Drivers with SPI Master functionality only. Slave functionalities are disabled and calling the slave functions will return with an error. An SPI slave device connects to one of the exported drivers and uses it as any other SPI CMSIS-Driver (in master mode only). The wrapper provides multi thread protection.

Each slave can use a different bus configuration. The MultiSlave wrapper will detect which slave device is addressed by a particular function call and reconfigure the SPI bus accordingly. When using the SPI MultiSlave wrapper, the slave select mode must always be configured as `ARM_SPI_SS_MASTER_SW`. Since the underlying bus controlling SPI driver can only control one slave select line at the time, the slave select line for each particular slave device is instead controlled by the MultiSlave wrapper using the function SPI_Control_SlaveSelect that must be implemented in the user application. A function prototype can be found in the [SPI_Multislave.h](https://github.com/ARM-software/CMSIS-Driver/blob/main/SPI/SPI_MultiSlave.h) header file and must be included in the project.

When called from different threads, the MultiSlave wrapper can be busy (if any data transfer is in progress). In such a case, transfer operation will be queued and executed immediately after the busy slave is deselected. The transfer queue operates as a FIFO, so transfers will be executed in the same call order as expected by the application.

The wrapper is configured using the [SPI_MultiSlave_Config.h](https://github.com/ARM-software/CMSIS-Driver/blob/main/Config/SPI_MultiSlave_Config.h) file, which contains the following options:

- `#define SPI_DRIVER` specifies the underlying SPI CMSIS-Driver, which actually controls the SPI peripheral and the accesses the bus. The wrapper will connect to that driver.
- `#define SPI_ENABLE_SLAVE_x` enables each SPI bus connected slave. This basically means that the driver control block `Driver_SPIn` will be exported by the wrapper for each particular slave.
- `#define SPI_DRIVER_SLAVE_x` sets the exported control block number n, for example `Driver_SPIn`. The application connects to this driver.

**Code example**

This is a demo application which demonstrates the usage of the SPI MultiSlave driver wrapper. It consists of two threads that periodically access two SPI slave devices.

```c
#include <string.h>
#include "cmsis_os2.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "stm32f2xx_hal.h"

#include "SPI_MultiSlave.h"             // Keil::CMSIS Driver:SPI:Multi-Slave

/* Thread prototypes */
__NO_RETURN static void Thread_A (void *argument);
__NO_RETURN static void Thread_B (void *argument);
__NO_RETURN static void app_main (void *argument);

/* A and B Thread IDs */
static osThreadId_t ThreadId_A;
static osThreadId_t ThreadId_B;

/* SPI A Driver, controls Slave Device 0, uses underlying Driver_SPI1 (see SPI_MultiSlaveConfig.h) */
extern ARM_DRIVER_SPI         Driver_SPI10;
#define SPI_A               (&Driver_SPI10)

/* SPI B Driver, controls Slave Device 1, uses underlying Driver_SPI1 (see SPI_MultiSlaveConfig.h) */
extern ARM_DRIVER_SPI         Driver_SPI11;
#define SPI_B               (&Driver_SPI11)

/*
  Slave select pin control function.

  \param[in]    driver    SPI Driver Instance
  \param[in]    ss_state  Slave Select signal state (ARM_SPI_SS_INACTIVE | ARM_SPI_SS_ACTIVE)
*/
void SPI_Control_SlaveSelect (uint32_t device, uint32_t ss_state) {
  GPIO_TypeDef* GPIOx;
  uint16_t pin;

  if (device == 0) {
    /* Select Device 0 SS pin (SPI_A) */
    GPIOx = GPIOE;
    pin   = GPIO_PIN_0;
  }
  else {
    /* Select Device 1 SS pin (SPI_B) */
    GPIOx = GPIOE;
    pin   = GPIO_PIN_15;
  }

  if (ss_state == ARM_SPI_SS_INACTIVE) {
    /* Set GPIO pin high */
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
  } else {
    /* Set GPIO pin low */
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
  }
}

/*----------------------------------------------------------------------------
 * SPI Thread A
 *---------------------------------------------------------------------------*/
static void Thread_A (void *argument) {
  char *p = "Sending data to Slave Device 0";
  (void)argument;
  
  /* Initialize and configure SPI A */
  SPI_A->Initialize(NULL);
  SPI_A->PowerControl(ARM_POWER_FULL);
  SPI_A->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0  \
                                     | ARM_SPI_DATA_BITS(8) \
                                     | ARM_SPI_MSB_LSB      \
                                     | ARM_SPI_SS_MASTER_SW,
                                     10000000);

  SPI_A->Control(ARM_SPI_SET_DEFAULT_TX_VALUE, 0xFF);

  while(1) {
    /* Send to Slave Device 0 */
    SPI_A->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    SPI_A->Send(p, strlen(p));

    /* Wait until SPI A busy (or alternatively, wait for SPI event) */
    while (SPI_A->GetStatus().busy);
    SPI_A->Control (ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    osDelay(10);
  }
}

/*----------------------------------------------------------------------------
 * SPI Thread B
 *---------------------------------------------------------------------------*/
static void Thread_B (void *argument) {
  char *p = "Sending data to Slave Device 1";
  (void)argument;

  /* Initialize and configure SPI B */
  SPI_B->Initialize(NULL);
  SPI_B->PowerControl(ARM_POWER_FULL);
  SPI_B->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1  \
                                     | ARM_SPI_DATA_BITS(8) \
                                     | ARM_SPI_MSB_LSB      \
                                     | ARM_SPI_SS_MASTER_SW,
                                     15000000);

  SPI_B->Control(ARM_SPI_SET_DEFAULT_TX_VALUE, 0xFF);

  while(1) {
    /* Send to Slave Device 1 */
    SPI_B->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    SPI_B->Send(p, strlen(p));

    /* Wait until SPI B busy (or alternatively, wait for SPI event) */
    while (SPI_B->GetStatus().busy);
    SPI_B->Control (ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    osDelay(10);
  }
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
static void app_main (void *argument) {
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
