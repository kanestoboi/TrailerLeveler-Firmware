


@Echo Off
echo Creating Trailer Leveler Firmware Image

set build-configuration=Debug
set application-hex=../Output/%build-configuration%/Exe/TrailerLeveler.hex
set soft-device-hex=../../nRF5_SDK_Current/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex
set bootloader-hex=../../TrailerLeveler-Bootloader/Output/Release/Exe/TrailerLevelerSecureBootloader_s140_pca10056.hex

echo Generating DFU Settings Page
nrfutil.exe settings generate --family NRF52840 --application %application-hex% --application-version 1 --bootloader-version 1 --bl-settings-version 2 bl_settings.hex

echo Merging Bootloader, Soft Device, Settings, and Application
mergehex --merge bl_settings.hex %bootloader-hex% %soft-device-hex% %application-hex% --output bl_sd_settings_app.hex

echo Programming Firmware Image to Device
nrfjprog --recover
nrfjprog -f nrf52 --program bl_sd_settings_app.hex --verify
nrfjprog --reset
