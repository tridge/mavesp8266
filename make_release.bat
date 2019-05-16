@REM !/bin/bash

@REM if you want to build and rename binaries that will go to files.rfdesign.com.au, and can be uploaded through the TXMOD webpage run this:

@REM edit these #defines in src/mavesp8266.h to match this .sh before running it. ( ie inc@REMent both if you inc@REMent either )
@REM  #define MAVESP8266_VERSION_MAJOR    1
@REM  #define MAVESP8266_VERSION_MINOR    3
@REM  #define MAVESP8266_VERSION_BUILD    1

@REM ------------------------------------------------------------------

@REM set us up for a 2MB build:
@REM copy platformio.ini.2m platformio.ini

@REM del .pioenvs\esp12e\firmware.bin
@REM del .pioenvs\esp12e\spiffs.bin
@REM del firmware.bin
@REM del spiffs.bin

@REM build firmare.bin binary:
@REM platformio run
@REM build spiffs.bin binary:
@REM platformio run -t buildfs

@REM copy the target binaries to a releasable name:
@SET VERSION=V1.35
@REM copy .pioenvs\esp12e\spiffs.bin .
@REM rename spiffs.bin RFDTxMod-%VERSION%.2m.spiffs.bin
@REM copy .pioenvs\esp12e\firmware.bin .
@REM rename firmware.bin RFDTxMod-%VERSION%.2m.bin

del .pioenvs\esp12e\firmware.bin
del .pioenvs\esp12e\spiffs.bin
del firmware.bin
del spiffs.bin

@REM------------------------------------------------------------------

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

@REM------------------------------------------------------------------
echo ------------------------------------------------------------------
@REM show user:
@dir RFDTx*.bin
