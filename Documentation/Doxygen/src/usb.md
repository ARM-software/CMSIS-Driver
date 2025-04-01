# USB {#page_driver_usb}

## Driver Implementations

The \ref pack_content provides implementations of **[CMSIS-USB drivers](https://arm-software.github.io/CMSIS_6/latest/Driver/group__usb__interface__gr.html)** for the following controllers:

| Driver         | Description                                                                    |
|----------------|--------------------------------------------------------------------------------|
| **EHCI**       | USB Host Driver for the EHCI with Transaction Translator (TT) host controller. |
| **OHCI**       | USB Host Driver for the OHCI host controller.                                  |

### EHCI

Enhanced Host Controller Interface (EHCI) with TT is EHCI controller with integrated Transaction Translator that supports high/full/low-speed devices.

It is usually used in embedded devices to remove the requirement of having additional host controller (OHCI) for handling full/low-speed devices separately.

This driver exports up to 2 driver instances thus it can support 2 EHCI with TT host controllers.

It requires hardware-specific functions implementation that are available in the template module **USBH_EHCI_HW.c**.

It is configured via define values in the **USBH_EHCI_Config.h** configuration file.

**Configuration**

- **USB Host Controller 0**:
  - **Export control block Driver_USBH#**: Specifies the exported driver control block number.
  - **EHCI Registers base address**: Specifies the absolute address at which EHCI controller registers are located.
  - **Locate EHCI Communication Area**: Specifies if the communication area is located in a specific memory (via the linker script):
    - **Section name**: Specifies the section name for the EHCI communication area (for positioning via the linker script).

- **USB Host Controller 1** (can be enabled/disabled): 
  - **Export control block Driver_USBH#**: Specifies the exported driver control block number.
  - **EHCI Registers base address**: Specifies the absolute address at which EHCI controller registers are located.
  - **Locate EHCI Communication Area**: Specifies if the communication area is located in a specific memory (via the linker script):
    - **Section name**: Specifies the section name for the EHCI communication area (for positioning via the linker script).

- **Maximum number of Pipes (per controller)**: Specifies the maximum number of pipes that the driver will support (per controller).

### OHCI

Open Host Controller Interface (OHCI) is a host controller interface that supports full/low-speed devices.

This driver exports up to 2 driver instances thus it can support 2 OHCI host controllers.

It requires hardware-specific functions implementation that are available in the template module **USBH_OHCI_HW.c**.

It is configured via define values in the **USBH_OHCI_Config.h** configuration file.

**Configuration**

- **USB Host Controller 0**:
  - **Export control block Driver_USBH#**: Specifies the exported driver control block number.
  - **OHCI Registers base address**: Specifies the absolute address at which OHCI controller registers are located.
  - **Locate OHCI Communication Area (HCCA)**: Specifies if the communication area is located in a specific memory (via the linker script):
    - **Section name**: Specifies the section name for the OHCI communication area (for positioning via the linker script).

- **USB Host Controller 1** (can be enabled/disabled): 
  - **Export control block Driver_USBH#**: Specifies the exported driver control block number.
  - **OHCI Registers base address**: Specifies the absolute address at which OHCI controller registers are located.
  - **Locate OHCI Communication Area (HCCA)**: Specifies if the communication area is located in a specific memory (via the linker script):
    - **Section name**: Specifies the section name for the OHCI communication area (for positioning via the linker script).

- **Maximum number of Pipes (per controller)**: Specifies the maximum number of pipes that the driver will support (per controller).
