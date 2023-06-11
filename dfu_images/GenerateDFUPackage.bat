
@Echo Off
set build-configuration=Release
set /A firmwareVersion=2
set /A bootloaderVersion=1

set application-hex=../Output/%build-configuration%/Exe/TrailerLeveler.hex
set soft-device-hex=../../nRF5_SDK_Current/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex
set bootloader-hex=../../TrailerLeveler-Bootloader/Output/Release/Exe/TrailerLevelerSecureBootloader_s140_pca10056.hex

echo Generating DFU Settings Page
nrfutil.exe settings generate --family NRF52840 --application %application-hex% --application-version %firmwareVersion% --bootloader-version %bootloaderVersion% --bl-settings-version 1 bl_settings.hex

echo Merging Bootloader, Soft Device, Settings, and Application
mergehex --merge bl_settings.hex %bootloader-hex% %soft-device-hex% %application-hex% --output bl_sd_settings_app.hex

nrfutil.exe pkg generate --hw-version 52 --application-version %firmwareVersion% --application %application-hex% --sd-req 0x0100 --sd-id 0x0100 --key-file keys/private.key trailer_leveler_application_v%firmwareVersion%_s140.zip