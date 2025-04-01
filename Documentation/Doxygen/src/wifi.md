# WiFi {#page_driver_wifi}

## Driver Implementations

The \ref pack_content provides implementations of **[CMSIS-WiFi drivers](https://arm-software.github.io/CMSIS_6/latest/Driver/group__wifi__interface__gr.html)** for the following devices:

| Driver                         | Description                            |
|--------------------------------|----------------------------------------|
| **[DA16200](#wifi_da16200)**   | WiFi Driver for the Renesas DA16200.   |
| **[ESP32](#wifi_esp32)**       | WiFi Driver for the Espressif ESP32.   |
| **[ESP8266](#wifi_esp8266)**   | WiFi Driver for the Espressif ESP8266. |
| **[ISM43362](#wifi_ism43362)** | WiFi Driver for the Inventek ISM43362. |
| **[WizFi360](#wifi_wizfi360)** | WiFi Driver for the WizNet WizFi360.   |

### DA16200 {#wifi_da16200}

The documentation for the Renesas DA16200 can be found here:

- https://www.renesas.com/eu/en/products/interface-connectivity/wireless-communications/wi-fi/da16200-ultra-low-power-wi-fi-soc-battery-powered-iot-devices

**Required Firmware** : **DA16200 FreeRTOS SDK Firmware Image v3.2.3.0 (or newer)**

Firmware image and programming instructions **DA16200 DA16600 FreeRTOS Getting Started Guide** are also available on the above web site.

### ESP32 {#wifi_esp32}

The documentation for the Espressif ESP32 can be found here:

- https://www.espressif.com/en/products/hardware/esp-wroom-32/overview

### ESP8266 {#wifi_esp8266}

The documentation for the Espressif ESP8266 can be found here:

- https://www.espressif.com/en/products/hardware/esp8266ex/overview/

### ISM43362 {#wifi_ism43362}

The documentation for the Inventek ISM43362 can be found here:

- https://www.inventeksys.com/ism4336-m3g-l44-e-embedded-serial-to-wifi-module/

**Required Firmware** : **SPI v6.2.1.7**

For firmware update procedure see [Inventek ISMART43362-E Shield Firmware](#inventek_ismart43362-e_firmware) section.

### WizFi360 {#wifi_wizfi360}

The documentation for the WIZnet WizFi360 can be found here:

- https://docs.wiznet.io/Product/Wi-Fi-Module/WizFi360
