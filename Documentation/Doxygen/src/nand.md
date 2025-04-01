# NAND {#page_driver_nand}

This is a CMSIS compliant driver for NAND Flash devices that are connected to the microcontroller's memory bus. It implements a [CMSIS-Driver NAND Interface](https://arm-software.github.io/CMSIS_6/latest/Driver/group__nand__interface__gr.html) with up to four memory mapped NAND Flash devices.

The driver is configured using the NAND_MemBus_Config.h configuration file, which contains the following options:

- `#define NAND_DRIVER` exports the NAND CMSIS-Driver instance, which can be used to access the memory mapped NAND Flash devices.
- `#define NAND_DEVx` enables the memory mapped NAND Flash device.
- `#define NAND_DEVx_ADDR_BASE` specifies the base address of the Flash device, which is used to control the data/address lines and nCE, nRE, nWE lines.
- `#define NAND_DEVx_ADDR_ALE` specifies the ALE address of the Flash device, which is used to control the ALE line.
- `#define NAND_DEVx_ADDR_CLE` specifies the CLE address of the Flash device, which is used to control the CLE line.
- `#define NAND_DEVx_DATA_WIDTH` specifies the data bus width of the Flash device.
- `#define NAND_DEVx_RB_PIN` specifies if Ready/Busy line of the Flash device is available (used for Ready/Busy monitoring).
- `#define NAND_DEVx_RB_PIN_IRQ` specifies if Ready/Busy line of the Flash device can trigger an interrupt.
