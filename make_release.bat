@SET VERSION=V1.37

del .pioenvs\esp12e\firmware.bin
del .pioenvs\esp12e\spiffs.bin
del firmware.bin
del spiffs.bin

@REM set us up for a 4MB build:
copy platformio.ini.4m platformio.ini

@REM build firmare.bin binary:
platformio run
@REM build spiffs.bin binary:
platformio run -t buildfs

@REM copy the target binaries to a releasable name:
copy .pioenvs\esp12e\spiffs.bin .
rename spiffs.bin RFDTxMod-%VERSION%.4m.spiffs.bin
copy .pioenvs\esp12e\firmware.bin .
rename firmware.bin RFDTxMod-%VERSION%.4m.bin

echo ------------------------------------------------------------------
@REM show user:
@dir RFDTx*.bin
