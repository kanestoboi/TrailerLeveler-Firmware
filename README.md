# Trailer Leveler Firmware

## Developer Dependencies
* [Segger Embedded Studio for ARM (V6.40)](https://www.segger.com/downloads/embedded-studio/)
* [nRF5 SDK (Version 17.1)](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk)
* [Segger RTT Viewer (V7.84) ](https://www.segger.com/products/debug-probes/j-link/tools/rtt-viewer/)
* [nRF Util](https://www.nordicsemi.com/Products/Development-tools/nrf-util)

## Folder Structure
The reposository should be placed at the same level as the nRF5 SDK. The nRF5 SDK folder should be renamed as *nRF5_SDK_Current*

### Example Folder Structure
```
nRF Workspace
└───TrailerLeveler-Firmware
└───nRF5_SDK_Current
    └───components
    └───config
    └───documentation
    └───examples
    ...
```

## Flashing Firmware to Device

To flash firmmware to device, use the batch `GenerateFirmwareImage.bat` script located in dfu_images. Merge the firmware, bootloader, soft device and settings into a .hex file and program the connected device.

NOTE: the nrfutil.exe needs to be located in the dfu_images folder for this script to work.
