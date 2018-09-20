/*
 * zmodem.h
 * zmodem constants
 * (C) Mattheij Computer Service 1994
 */

/* $Id: zmodem.h,v 1.53 2012/10/24 19:03:14 deuce Exp $ */

#ifndef _ZMODEM_H
#define _ZMODEM_H

#include <Arduino.h>
#include <FS.h>  

/*
 * ascii constants
 */

#ifndef SOH
#define SOH     0x01
#define STX     0x02
#define EOT     0x04
#define ENQ     0x05
#define ACK     0x06
#define LF      0x0a
#define CR      0x0d
#define DLE     0x10
#define XON     0x11
#define XOFF    0x13
#define NAK     0x15
#define CAN     0x18
#endif

#define ModeXModem  0
#define ModeYModem  1

class XModem
{
  public:
    XModem(Stream *port, char mode);
    int sendFile(File dataFile, char *fileName);
  private:
    Stream *port;
    unsigned char packetNo, checksumBuf;
    long filepos;
    unsigned int packetLen;
    int crcBuf;
    unsigned char mode;
    unsigned char oldChecksum;

    char sync(void);
    void outputByte(unsigned char inChar);
    char waitACK(void);
};

#endif
