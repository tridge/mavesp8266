#!/bin/bash

# if you want to build and rename binaries that will go to files.rfdesign.com.au, and can be uploaded through the TXMOD webpage run this:

# build firmare.bin binary:
platformio run
# build spiffs.bin binary:
platformio run -t buildfs
# copy the target binaries to a releasable name:
VERSION=V1.30
echo cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.spiffs.bin
cp .pioenvs/esp12e/spiffs.bin RFDTxMod-$VERSION.spiffs.bin
echo cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.bin
cp .pioenvs/esp12e/firmware.bin RFDTxMod-$VERSION.bin
