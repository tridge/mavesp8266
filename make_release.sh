#!/bin/bash

# if you want to build and rename binaries that will go to files.rfdesign.com.au, and can be uploaded through the TXMOD webpage run this:

#edit these #defines in src/mavesp8266.h to match this .sh before running it. ( ie increment both if you increment either )
#  #define MAVESP8266_VERSION_MAJOR    1
#  #define MAVESP8266_VERSION_MINOR    3
#  #define MAVESP8266_VERSION_BUILD    1

#------------------------------------------------------------------

# set us up for a 2MB build:
cp platformio.ini.2m platformio.ini

# build firmare.bin binary:
platformio run
# build spiffs.bin binary:
platformio run -t buildfs

# copy the target binaries to a releasable name:
VERSION=V1.32
echo cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.2m.spiffs.bin
cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.2m.spiffs.bin
echo cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.2m.bin
cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.2m.bin

#------------------------------------------------------------------

# set us up for a 4MB build:
cp platformio.ini.4m platformio.ini

# build firmare.bin binary:
platformio run
# build spiffs.bin binary:
platformio run -t buildfs

# copy the target binaries to a releasable name:
VERSION=V1.32
echo cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.4m.spiffs.bin
cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.4m.spiffs.bin
echo cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.4m.bin
cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.4m.bin

#------------------------------------------------------------------
echo ------------------------------------------------------------------
# show user:
ls -l RFDTx*.bin
