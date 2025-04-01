# Ethernet {#page_driver_eth}

## Driver Implementations

The \ref pack_content provides implementations of **[CMSIS-Ethernet drivers](https://arm-software.github.io/CMSIS_6/latest/Driver/group__eth__interface__gr.html)** for the following devices:

| Driver         | Description                                                                                         |
|----------------|-----------------------------------------------------------------------------------------------------|
| **KSZ8851SNL** | Ethernet PHY and MAC interfaces for the Microchip **[KSZ8851](https://www.microchip.com/KSZ8851)**. |
| **LAN9220**    | Ethernet PHY and MAC interfaces for the Microchip **[LAN9220](https://www.microchip.com/LAN9220)**. |
| **DP83848C**   | Ethernet PHY interface for the Texas Instruments **[DP83848C](http://www.ti.com/product/DP83848C)**.|
| **KSZ8061RNB** | Ethernet PHY interface for the Microchip **[KSZ8061](https://www.microchip.com/KSZ8061)**.          |
| **KSZ8081RNA** | Ethernet PHY interface for the Microchip **[KSZ8081](https://www.microchip.com/KSZ8081)**.          |
| **LAN8710A**   | Ethernet PHY interface for the Microchip **[LAN8710A](https://www.microchip.com/LAN8710A)**.        |
| **LAN8720**    | Ethernet PHY interface for the Microchip **[LAN8720A](https://www.microchip.com/LAN8720A)**.        |
| **LAN8740A**   | Ethernet PHY interface for the Microchip **[LAN8740A](https://www.microchip.com/LAN8740A)**.        |
| **LAN8742A**   | Ethernet PHY interface for the Microchip **[LAN8742A](https://www.microchip.com/LAN8742A)**.        |
| **ST802RT1**   | Ethernet PHY interface for the STMicroelectronics **[ST802RT1](http://www.st.com/content/ccc/resource/technical/document/data_brief/37/8e/14/c7/84/39/4d/61/CD00263138.pdf/files/CD00263138.pdf/jcr:content/translations/en.CD00263138.pdf)**. |

## Multiple Driver Instances

CMSIS-Driver API supports multiple driver instances. The Ethernet drivers are implemented within a single C module and several driver instances of the same type can be used in a project as follows:

1. Add the first driver instance to the project. In IDEs with CMSIS-pack management support this can be done from the Run-Time Environment (RTE).
2. Create a copy of the driver's .c file with a different file name and add it to the project. This will be the second driver instance. For example, copy `ETH_LAN9220.c` file as `ETH2_LAN9220.c`.
3. Copy the driver's .h file to the project or add the driver's folder to the compiler include search path.
4. Specify the driver parameters for the second instance. For example, in `ETH2_LAN9220.c` new values to the following parameters are needed instead of default ones:
    ```c
    #define ETH_MAC_NUM             1
    #define ETH_PHY_NUM             1
    #define LAN9220_BASE            (0x53000000UL)
    ```
5. Now both Ethernet instances can be accessed from the application. For example:
    ```c
    #include "Driver_ETH_MAC.h"
    #include "Driver_ETH_PHY.h"

    extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;
    extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC1;

    extern ARM_DRIVER_ETH_PHY Driver_ETH_PHY0;
    extern ARM_DRIVER_ETH_PHY Driver_ETH_PHY1;

    #define eth0    (&Driver_ETH_MAC0)
    #define eth1    (&Driver_ETH_MAC1)

    #define phy0    (&Driver_ETH_PHY0)
    #define phy1    (&Driver_ETH_PHY1)
    ```
