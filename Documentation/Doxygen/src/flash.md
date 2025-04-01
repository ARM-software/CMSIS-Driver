# Flash {#page_driver_flash}

## Driver Implementations

The \ref pack_content contains implementations of **[CMSIS-Flash drivers](https://arm-software.github.io/CMSIS_6/latest/Driver/group__flash__interface__gr.html)** for the following devices:

| Driver         | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| **AM29x800BB** | Flash interface for Parallel NOR Flash **AM29x800BB**. This product is discontinued. |
| **AT45DB641E** | Flash interface for SPI Serial DataFlash **[AT45DB641E](https://www.renesas.com/br/en/document/dst/at45db641e-datasheet)**. |
| **AT45DB642D** | Flash interface for SPI Serial DataFlash **AT45DB642D**.                    |
| **M29EW28F128**| Flash interface for Parallel NOR Flash **M29EW28F128**.                     |
| **M29W640FB**  | Flash interface for Parallel NOR Flash **M29W640FB**.                       |
| **N25Q032A**   | Flash interface for Serial NOR Flash **N25Q032A**.                          |
| **S29GL064Nx2**| Flash interface for Parallel NOR Flash **[S29GL064N](http://www.cypress.com/documentation/datasheets/s29gl064n-s29gl032n-64-mbit-32-mbit-3-v-page-mode-mirrorbit-flash)**. |

## Multiple Driver Instances

CMSIS-Driver API supports multiple driver instances. The Flash drivers are implemented within a single C module and several driver instances of the same type can be used in a project as follows:

1. Add the first driver instance to the project. In IDEs with CMSIS-pack management support this can be done from the Run-Time Environment (RTE).
2. Create a copy of the driver's .c file with a different file name and add it to the project. This will be the second driver instance. For example, copy `AT45DB641E.c` file as `AT45DB641E_2.c`.
3. Copy the driver's .h file to the project or add the driver's folder to the compiler include search path.
4. Specify the driver parameters for the second instance based on the hardware design. For example, in `AT45DB641E_2.c` the values for the flash driver number and SPI driver number need to be configured as shown below:
    ```c
    #define DRIVER_FLASH_NUM        1
    #define DRIVER_SPI_NUM          1
    ```
5. Now both Flash instances can be accessed from the application. For example:
    ```c
    #include "Driver_Flash.h"

    /* Flash driver instances */
    extern ARM_DRIVER_FLASH Driver_Flash0;
    extern ARM_DRIVER_FLASH Driver_Flash1;

    #define flash0  (&Driver_Flash0)
    #define flash1  (&Driver_Flash1)
    ```
