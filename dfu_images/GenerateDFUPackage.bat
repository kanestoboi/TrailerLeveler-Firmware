
@Echo Off
set build-configuration=Release
set /A version=2


set application-hex=../Output/%build-configuration%/Exe/TrailerLeveler.hex

nrfutil.exe pkg generate --hw-version 52 --application-version %version% --application %application-hex% --sd-req 0x0100 --sd-id 0x0100 --key-file keys/private.key trailer_leveler_application_v%version%_s140.zip