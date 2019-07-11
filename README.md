# CMSIS-Driver

This repository contains MCU independent device driver implementations and template files. 
The drivers are implemented using the [CMSIS-Driver API specification](http://arm-software.github.io/CMSIS_5/Driver/html/index.html).
All drivers refer the API Interface header file defintions published in the CMSIS pack (https://github.com/ARM-software/CMSIS_5).

## ETH - Ehternet MAC & PHY or Ethernet PHY drivers
- ETH_KSZ8851SNL
- ETH_LAN9220
- PHY_DP83848
- PHY_KSZ8061RNB
- PHY_KSZ8081RNA
- PHY_LAN8710A
- PHY_LAN8720
- PHY_LAN8742A
- PHY_ST802RT1

## Flash - Flash device drivers
- AM29x800BB
- AT45DB641E
- AT45DB642D
- M29EW28F128
- M29W640FB
- N25Q32A
- S29GL064Nx2

## I2C - MultiSlave wrapper for I2C CMSIS-Driver implementations
- I2C_MultiSlave

## NAND - NAND Flash drivers
- NAND_MemBus

## SPI - MultiSlave wrapper for SPI CMSIS-Driver implementations
- SPI_MultiSlave

## WiFi - WiFi device drivers (compliant with CMSIS-Driver WiFi specification 1.0.0)
- ISM43362

## License
Licensed under Apache 2.0 License.
