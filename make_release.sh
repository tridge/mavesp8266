#!/bin/bash

# if you want to build and rename binaries that will go to files.rfdesign.com.au, and can be uploaded through the TXMOD webpage run this:

#edit these #defines in src/mavesp8266.h to match this .sh before running it. ( ie increment both if you increment either )
#  #define MAVESP8266_VERSION_MAJOR    1
#  #define MAVESP8266_VERSION_MINOR    3
#  #define MAVESP8266_VERSION_BUILD    1

# build firmare.bin binary:
platformio run
# build spiffs.bin binary:
platformio run -t buildfs
# copy the target binaries to a releasable name:
VERSION=V1.31
echo cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.spiffs.bin
cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.spiffs.bin
echo cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.bin
cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.bin
