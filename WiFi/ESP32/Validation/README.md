# Validation of the Espressif ESP32 WiFi CMSIS-Driver

## The [Test Report](./TestReport.txt) file was produced with the following configuration:
 - Hardware:  **NXP MIMXRT1064-EVK** board with **ESP32-WROOM-32D** on **SparkFun ESP32 Thing Plus**
 - Firmware:  **AT Command Set 2.1.0.0**
 - Interface: **UART at 115200 bps**

## Packs used for validation:
 - **ARM::CMSIS-Driver v2.6.1**
 - ARM::CMSIS-Driver_Validation v2.0.0
 - ARM::CMSIS v5.7.0
 - Keil::ARM_Compiler v1.6.3
 - NXP MIMXRT1064_DFP v12.2.0
 - NXP EVK-MIMXRT1064_BSP v12.2.0

### AT Version Information (AT+GMR):
 - AT version:2.1.0.0(883f7f2 - Jul 24 2020 11:50:07)
 - SDK version:v4.0.1-193-ge7ac221
 - compile time(0ad6331):Jul 28 2020 02:47:21
 - Bin version:2.1.0(WROOM-32)
