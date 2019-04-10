@echo off
set /p com=Enter COM port number: 
echo COM%com% selected
:start
esptool.py --port COM%com% --baud 921600 erase_flash
echo Cycle module power by removing and pluging in FTDI cable
pause
esptool.py --port COM%com% --baud 921600 write_flash -fm dio -fs 4MB -ff 40m 0x00000 firmware.bin 0x00300000 spiffs.bin
set choice=
set /p choice="Flash completed. Do you want to restart? Press 'y' and enter for Yes: "
if not '%choice%'=='' set choice=%choice:~0,1%
if '%choice%'=='y' goto start