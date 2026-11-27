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
|📂 **Ethernet/**             | [Ethernet](#driver_ETH) driver implementations.            |
|📂 **Ethernet_PHY/**         | [Ethernet](#driver_ETH) PHY driver implementations.        |
|📂 **Flash/**                | [Flash](#driver_Flash) memory driver implementations.      |
|📂 **I2C/**                  | [I2C](#driver_I2C) driver implementations.                 |
|📂 **NAND/**                 | [NAND](#driver_NAND) driver implementations.               |
|📂 **Shield/**               | [Shield layer](#shield_layer) implementations.             |
|📂 **SPI/**                  | [SPI](#driver_SPI) driver implementations.                 |
|📂 **USB/**                  | [USB](#driver_USB) driver implementations.                 |
|📂 **WiFi/**                 | [WiFi](#driver_WiFi) driver implementations.               |

## License

The CMSIS Driver example implementations are provided free of charge under the Apache 2.0 license.  
See the [Apache 2.0 License](https://raw.githubusercontent.com/ARM-software/CMSIS-Driver/main/LICENSE)
