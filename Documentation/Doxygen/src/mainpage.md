# Overview {#mainpage}

The **[CMSIS-Driver specification](http://arm-software.github.io/CMSIS_6/latest/Driver/index.html)** defines a uniform software API for peripheral driver interfaces that can be used by middleware stacks and user applications.

This documentation covers a set of reference CMSIS-Driver implementations for external peripheral devices.

The implementations are maintained in a public **[GitHub repository](https://github.com/arm-software/CMSIS-Driver)**. Their releases in **[CMSIS Pack format](https://www.open-cmsis-pack.org/)** are also available on **[CMSIS Packs page](https://developer.arm.com/tools-and-software/embedded/cmsis/cmsis-packs)** under *Arm* - *CMSIS Drivers for external devices* category and can be used in environments supporting the CMSIS-Pack concept.

Interested parties are welcome to contribute their drivers to the referenced repository.

## Pack Content {#pack_content}

The **ARM::CMSIS-Driver** Pack contains the following items:

| File/Directory              | Content                                                         |
|-----------------------------|-----------------------------------------------------------------|
|📄 **ARM.CMSIS-Driver.pdsc** | Package description file in CMSIS-Pack format.                  |
|📄 **LICENSE**               | CMSIS license agreement (Apache 2.0).                           |
|📂 **Config/**               | Configuration files for I2C, NAND, and SPI bus implementations. |
|📂 **Documentation/**        | This documentation.                                             |
|📂 **Ethernet/**             | [Ethernet](#page_driver_eth) driver implementations.            |
|📂 **Ethernet_PHY/**         | [Ethernet](#page_driver_eth) PHY driver implementations.        |
|📂 **Flash/**                | [Flash](#page_driver_flash) memory driver implementations.      |
|📂 **I2C/**                  | [I2C](#page_driver_i2c) driver implementations.                 |
|📂 **NAND/**                 | [NAND](#page_driver_nand) driver implementations.               |
|📂 **Shield/**               | [Shield layer](#page_shield_layer) implementations.             |
|📂 **SPI/**                  | [SPI](#page_driver_spi) driver implementations.                 |
|📂 **USB/**                  | [USB](#page_driver_usb) driver implementations.                 |
|📂 **WiFi/**                 | [WiFi](#page_driver_wifi) driver implementations.               |

## License

The CMSIS Driver example implementations are provided free of charge under the Apache 2.0 license.  
See the [Apache 2.0 License](https://raw.githubusercontent.com/ARM-software/CMSIS-Driver/main/LICENSE)
