/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_httpd.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"

#include <ESP8266WebServer.h>
#include <FS.h> // spiffs support

#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include <SoftwareSerial.h>
extern SoftwareSerial swSer;

const char PROGMEM kTEXTPLAIN[]  = "text/plain";
const char PROGMEM kTEXTHTML[]   = "text/html";
const char PROGMEM kACCESSCTL[]  = "Access-Control-Allow-Origin";

const char PROGMEM kUPLOADFORM[] = "";
const char PROGMEM kHEADER[]     = "<!doctype html><html><head><title>RFDesign TXMOD</title></head><body><h1><a href='/'>RFDesign TXMOD</a></h1>";
const char PROGMEM kBADARG[]     = "BAD ARGS";
const char PROGMEM kAPPJSON[]    = "application/json";

const char* kBAUD       = "baud";
const char* kPLAIN       = "plain";
const char* kPWD        = "pwd";
const char* kSSID       = "ssid";
const char* kPWDSTA     = "pwdsta";
const char* kSSIDSTA    = "ssidsta";
const char* kIPSTA      = "ipsta";
const char* kGATESTA    = "gatewaysta";
const char* kSUBSTA     = "subnetsta";
const char* kCPORT      = "cport";
const char* kHPORT      = "hport";
const char* kCHANNEL    = "channel";
const char* kDEBUG      = "debug";
const char* kREBOOT     = "reboot";
const char* kPOSITION   = "position";
const char* kMODE       = "mode";

const char* kFlashMaps[7] = {
    "512KB (256/256)",
    "256KB",
    "1MB (512/512)",
    "2MB (512/512)",
    "4MB (512/512)",
    "2MB (1024/1024)",
    "4MB (1024/1024)"
};

// convert git version and build date to string
#define str(s) #s
#define xstr(s) str(s)
#define GIT_VERSION_STRING xstr(PIO_SRC_REV)
#define BUILD_DATE_STRING xstr(PIO_BUILD_DATE)
#define BUILD_TIME_STRING xstr(PIO_BUILD_TIME)

static uint32_t flash = 0;
static char paramCRC[12] = {""};

ESP8266WebServer    webServer(80);
MavESP8266Update*   updateCB    = NULL;
bool                started     = false;


ESP8266HTTPUpdateServer httpUpdater;


//holds the current upload, if there is one, except OTA binary uploads, done elsewhere.
File fsUploadFile;

//---------------------------------------------------------------------------------
void setNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

//---------------------------------------------------------------------------------
void returnFail(String msg) {
    webServer.send(500, FPSTR(kTEXTPLAIN), msg + "\r\n");
}

//---------------------------------------------------------------------------------
void respondOK() {
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
}

//---------------------------------------------------------------------------------
void handle_update() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTHTML), FPSTR(kUPLOADFORM));
}

//---------------------------------------------------------------------------------
void handle_upload() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTPLAIN), (Update.hasError()) ? "FAIL" : "OK");
    if(updateCB) {
        updateCB->updateCompleted();
    }
    //ESP.restart(); lets not do this just here, lets tell the user before we drop out on them... ( we'll let an ajax call to /reboot do it )
}

//---------------------------------------------------------------------------------

// push some notification/s during the firmware update through the soft-serial port...
#define DEBUG_SERIAL swSer

void handle_upload_status() {
    bool success  = true;
    if(!started) {
        started = true;
        if(updateCB) {
            updateCB->updateStarted();
        }
    }
    HTTPUpload& upload = webServer.upload();
    if(upload.status == UPLOAD_FILE_START) {
        #ifdef DEBUG_SERIAL
            //DEBUG_SERIAL.setDebugOutput(true);
        #endif
        WiFiUDP::stopAll();
        Serial.end();
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("Update: %s\n", upload.filename.c_str());
        #endif
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.printf("Update Success: %u\nReboot Needed.\n", upload.totalSize);
            #endif
              webServer.sendHeader("Location","/success.htm");      // Redirect the client to the success page
              webServer.send(303);
              //delay(2000); // so client can request /success just before we reboot.
        } else {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
            webServer.send(500, "text/plain", "500: couldn't create file");
        }
        #ifdef DEBUG_SERIAL
            //DEBUG_SERIAL.setDebugOutput(false);
        #endif


    }
    yield();
    if(!success) {
        if(updateCB) {
            updateCB->updateError();
        }
    }
}

//---------------------------------------------------------------------------------
void handle_getParameters()
{
    String message = FPSTR(kHEADER);
    message += "<p>TXMOD Parameters</p><table><tr><td width=\"240\">Name</td><td>Value</td></tr>";
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        message += "<tr><td>";
        message += getWorld()->getParameters()->getAt(i)->id;
        message += "</td>";
        unsigned long val = 0;
        if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT32)
            val = (unsigned long)*((uint32_t*)getWorld()->getParameters()->getAt(i)->value);
        else if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT16)
            val = (unsigned long)*((uint16_t*)getWorld()->getParameters()->getAt(i)->value);
        else
            val = (unsigned long)*((int8_t*)getWorld()->getParameters()->getAt(i)->value);
        message += "<td>";
        message += val;
        message += "</td></tr>";
    }
    message += "</table>";
    message += "</body>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_root()
{
    String message = FPSTR(kHEADER);
    message += "Version: ";
    char vstr[30];
    snprintf(vstr, sizeof(vstr), "%u.%u.%u", MAVESP8266_VERSION_MAJOR, MAVESP8266_VERSION_MINOR, MAVESP8266_VERSION_BUILD);
    message += vstr;
    message += "<br>\n";
    message += "Git Version: ";
    message += GIT_VERSION_STRING;
    message += "<br>\n";
   /* message += "Build Date: ";
    message += BUILD_DATE_STRING;
    message += " ";
    message += BUILD_TIME_STRING; */
    message += "<p>\n";


    // if we have an index.html in spiffs, use that, otherwise use a basic version that's included below.
    File f = SPIFFS.open("/index.htm", "r");
    if ( f ) { 
        //while ( f.available() ) { 
            message += f.readString();
        //}
    } else { 
    
    message += "<ul>\n";
    message += "<li><a href='/getstatus'>Get Status ( UDP/Mavlink mode )</a>\n";
    message += "<li><a href='/getstatus_tcp'>Get Status (TCP/passthrough mode )</a>\n";
    message += "<li><a href='/setup'>View/Edit WiFi/Network Setup</a>\n";
//    message += "<li><a href='/getparameters'>Get TXMOD Parameters</a>\n";
//    message += "<li><a href='/r900x_params.txt'>Get 900x Radio Parameters</a>\n";
    message += "<li><a href='/plist'>View/Edit 900x Radio Parameters</a>\n";
    message += "<li><a href='/updatepage'>Update Firmware</a>\n";
    message += "<li><a href='/reboot'>Reboot</a>\n";
    message += "<li><a href='/edit'>Advanced Mode -  Review and Edit (some) files in the SPIFFS filesystem.</a>";
    message += "</ul>\n";
    message += "<hr>\n";
    message += "<h2>Documentation</h2>\n";
    message += "(requires internet access)<br>\n";
    message += "<ul>\n";
    message += "<li><a href='http://ardupilot.org'>ArduPilot Website</a>\n";
    message += "<li><a href='http://ardupilot.org/copter/docs/common-esp8266-telemetry.html'>ESP8266 WiFi Documentation</a>\n";
    message += "<li><a href='https://github.com/RFDesign/mavesp8266'>RFDesign ESP8266 Source Code</a>\n";
    message += "<li><a href='http://files.rfdesign.com.au/firmware/'>RFDesign TXMOD Firmware Updates</a>\n";

    message += "</ul>\n";
    message += "</body></html>";

    }
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_update_html()
{
    String message = "oops, sory, try rebooting.";
    // if we have an index.html in spiffs, use that, otherwise use a basic version that's included below.
    File f = SPIFFS.open("/update.htm", "r");
    if ( f ) { 
            message = f.readString();
    } else {  // in the event that update.htm is missing, give enough so we can upload a spiffs.bin and make one:

        message =  FPSTR(kHEADER);
        message += "Please upload a spiffs.bin to continue: \n";
        message += "<form method='POST' action='/update' enctype='multipart/form-data'>\n";
        message += "Spiffs:<br>\n";
        message += "<input type='file' name='spiffs'>\n";
        message += "<input type='submit' value='Update SPIFFS'>\n";
        message += "</form>\n";

    }
    //setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_setup()
{
    String message = FPSTR(kHEADER);
    message += "<h1>Wifi Setup</h1>\n";
    message += "<form action='/setparameters' method='post'>\n";

    message += "WiFi Mode:&nbsp;";
    message += "<input type='radio' name='mode' value='0'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
        message += " checked";
    }
    message += ">AccessPoint\n";
    message += "<input type='radio' name='mode' value='1'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA) {
        message += " checked";
    }
    message += ">Station<br>\n";
    
    message += "AP SSID:&nbsp;";
    message += "<input type='text' name='ssid' value='";
    message += getWorld()->getParameters()->getWifiSsid();
    message += "'><br>";

    message += "AP Password (min len 8):&nbsp;";
    message += "<input type='text' name='pwd' value='";
    message += getWorld()->getParameters()->getWifiPassword();
    message += "'><br>";

    message += "WiFi Channel:&nbsp;";
    message += "<input type='text' name='channel' value='";
    message += getWorld()->getParameters()->getWifiChannel();
    message += "'><br>";

    message += "Station SSID:&nbsp;";
    message += "<input type='text' name='ssidsta' value='";
    message += getWorld()->getParameters()->getWifiStaSsid();
    message += "'><br>";

    message += "Station Password:&nbsp;";
    message += "<input type='text' name='pwdsta' value='";
    message += getWorld()->getParameters()->getWifiStaPassword();
    message += "'><br>";

    IPAddress IP;    
    message += "Station IP:&nbsp;";
    message += "<input type='text' name='ipsta' value='";
    IP = getWorld()->getParameters()->getWifiStaIP();
    message += IP.toString();
    message += "'><br>";

    message += "Station Gateway:&nbsp;";
    message += "<input type='text' name='gatewaysta' value='";
    IP = getWorld()->getParameters()->getWifiStaGateway();
    message += IP.toString();
    message += "'><br>";

    message += "Station Subnet:&nbsp;";
    message += "<input type='text' name='subnetsta' value='";
    IP = getWorld()->getParameters()->getWifiStaSubnet();
    message += IP.toString();
    message += "'><br>";

    message += "Host Port:&nbsp;";
    message += "<input type='text' name='hport' value='";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "'><br>";

    message += "Client Port:&nbsp;";
    message += "<input type='text' name='cport' value='";
    message += getWorld()->getParameters()->getWifiUdpCport();
    message += "'><br>";
    
    message += "Baudrate:&nbsp;";
    message += "<input type='text' name='baud' value='";
    message += getWorld()->getParameters()->getUartBaudRate();
    message += "'><br>";
    
    message += "<input type='submit' value='Save'>";
    message += "</form>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}


//---------------------------------------------------------------------------------
static void handle_getStatus()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    String message = FPSTR(kHEADER);
    message += "<p>Comm Status</p><table><tr><td width=\"240\">Packets Received from GCS</td><td>";
    message += gcsStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to GCS</td><td>";
    message += gcsStatus->packets_sent;
    message += "</td></tr><tr><td>GCS Packets Lost</td><td>";
    message += gcsStatus->packets_lost;
    message += "</td></tr><tr><td>GCS Parse Errors</td><td>";
    message += gcsStatus->parse_errors;
    message += "</td></tr><tr><td>Packets Received from Vehicle</td><td>";
    message += vehicleStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to Vehicle</td><td>";
    message += vehicleStatus->packets_sent;
    message += "</td></tr><tr><td>Vehicle Packets Lost</td><td>";
    message += vehicleStatus->packets_lost;
    message += "</td></tr><tr><td>Vehicle Parse Errors</td><td>";
    message += vehicleStatus->parse_errors;
    message += "</td></tr><tr><td>Radio Messages</td><td>";
    message += gcsStatus->radio_status_sent;
    message += "</td></tr></table>";
    message += "<p>System Status</p><table>\n";
    message += "<tr><td width=\"240\">Flash Size</td><td>";
    message += ESP.getFlashChipRealSize();
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";
    message += "<tr><td>Parameters CRC</td><td>";
    message += paramCRC;
    message += "</td></tr>\n";
    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_getStatusTcp()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();

    String message = FPSTR(kHEADER);

// kinda hacky to use extern global vars here, but it'll do for now.
extern long int stats_serial_in;
extern long int stats_tcp_in;
extern long int stats_serial_pkts;
extern long int stats_tcp_pkts;
//extern long int largest_serial_packet;
//extern long int largest_tcp_packet;
extern bool tcp_passthrumode;

    message += "<p>TCP Comms Status - last 1 second of TCP throughput.</p><br>";

    if ( tcp_passthrumode == true ) { 
    message += "<font color=green>Currently In TCP pass-through mode right now.</font><br>\n";
    } else { 
    message += "<font color=red>NOT In TCP pass-through mode right now</font><br>\n";
    }

    message += "<font color=green>To connect your GCS in TCP mode, please connect as a TCP client to IP: 192.168.4.1, with port number 23.</font><br>\n";

    message += "<table><tr><td width=\"240\">TCP Bytes Received from GCS</td><td>";
    message += stats_tcp_in;
    message += "</td></tr>";

    message += "<tr><td>TCP Packets Sent to GCS</td><td>";
    message += stats_tcp_pkts;
    message += "</td></tr>";

    message += "<tr><td>Serial Bytes Received from Vehicle</td><td>";
    message += stats_serial_in;
    message += "</td></tr>";

    message += "<tr><td>Serial Packets Sent to Vehicle</td><td>";
    message += stats_serial_pkts;
    message += "</td></tr></table>";

    message += "<p>System Status</p><table>\n";
    message += "<tr><td width=\"240\">Flash Size</td><td>";
    message += ESP.getFlashChipRealSize();
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";

    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_getJLog()
{
    uint32_t position = 0, len;
    if(webServer.hasArg(kPOSITION)) {
        position = webServer.arg(kPOSITION).toInt();
    }
    String logText = getWorld()->getLogger()->getLog(&position, &len);
    char jStart[128];
    snprintf(jStart, 128, "{\"len\":%d, \"start\":%d, \"text\": \"", len, position);
    String payLoad = jStart;
    payLoad += logText;
    payLoad += "\"}";
    webServer.send(200, FPSTR(kAPPJSON), payLoad);
}

//---------------------------------------------------------------------------------
void handle_getJSysInfo()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    uint32_t fid = spi_flash_get_id();
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"size\": \"%s\", "
        "\"id\": \"0x%02lX 0x%04lX\", "
        "\"flashfree\": \"%u\", "
        "\"heapfree\": \"%u\", "
        "\"logsize\": \"%u\", "
        "\"paramcrc\": \"%s\""
        " }",
        kFlashMaps[system_get_flash_size_map()],
        (long unsigned int)(fid & 0xff), (long unsigned int)((fid & 0xff00) | ((fid >> 16) & 0xff)),
        flash,
        ESP.getFreeHeap(),
        getWorld()->getLogger()->getPosition(),
        paramCRC
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_getJSysStatus()
{
    bool reset = false;
    if(webServer.hasArg("r")) {
        reset = webServer.arg("r").toInt() != 0;
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(reset) {
        memset(gcsStatus,     0, sizeof(linkStatus));
        memset(vehicleStatus, 0, sizeof(linkStatus));
    }
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"gpackets\": \"%u\", "
        "\"gsent\": \"%u\", "
        "\"glost\": \"%u\", "
        "\"vpackets\": \"%u\", "
        "\"vsent\": \"%u\", "
        "\"vlost\": \"%u\", "
        "\"radio\": \"%u\", "
        "\"buffer\": \"%u\""
        " }",
        gcsStatus->packets_received,
        gcsStatus->packets_sent,
        gcsStatus->packets_lost,
        vehicleStatus->packets_received,
        vehicleStatus->packets_sent,
        vehicleStatus->packets_lost,
        gcsStatus->radio_status_sent,
        vehicleStatus->queue_status
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_setParameters()
{
    if(webServer.args() == 0) {
        returnFail(kBADARG);
        return;
    }
    bool ok = false;
    bool reboot = false;
    if(webServer.hasArg(kBAUD)) {
        ok = true;
        getWorld()->getParameters()->setUartBaudRate(webServer.arg(kBAUD).toInt());
    }
    if(webServer.hasArg(kPWD)) {
        ok = true;
        getWorld()->getParameters()->setWifiPassword(webServer.arg(kPWD).c_str());
    }
    if(webServer.hasArg(kSSID)) {
        ok = true;
        getWorld()->getParameters()->setWifiSsid(webServer.arg(kSSID).c_str());
    }
    if(webServer.hasArg(kPWDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaPassword(webServer.arg(kPWDSTA).c_str());
    }
    if(webServer.hasArg(kSSIDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaSsid(webServer.arg(kSSIDSTA).c_str());
    }
    if(webServer.hasArg(kIPSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kIPSTA).c_str());
        getWorld()->getParameters()->setWifiStaIP(ip);
    }
    if(webServer.hasArg(kGATESTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kGATESTA).c_str());
        getWorld()->getParameters()->setWifiStaGateway(ip);
    }
    if(webServer.hasArg(kSUBSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kSUBSTA).c_str());
        getWorld()->getParameters()->setWifiStaSubnet(ip);
    }
    if(webServer.hasArg(kCPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpCport(webServer.arg(kCPORT).toInt());
    }
    if(webServer.hasArg(kHPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpHport(webServer.arg(kHPORT).toInt());
    }
    if(webServer.hasArg(kCHANNEL)) {
        ok = true;
        getWorld()->getParameters()->setWifiChannel(webServer.arg(kCHANNEL).toInt());
    }
    if(webServer.hasArg(kDEBUG)) {
        ok = true;
        getWorld()->getParameters()->setDebugEnabled(webServer.arg(kDEBUG).toInt());
    }
    if(webServer.hasArg(kMODE)) {
        ok = true;
        getWorld()->getParameters()->setWifiMode(webServer.arg(kMODE).toInt());
    }
    if(webServer.hasArg(kREBOOT)) {
        ok = true;
        reboot = webServer.arg(kREBOOT) == "1";
    }
    if(ok) {
        getWorld()->getParameters()->saveAllToEeprom();
        //-- Send new parameters back
        handle_getParameters();
        if(reboot) {
            delay(100);
            ESP.restart();
        }
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
static void handle_reboot()
{
    String message = FPSTR(kHEADER);
    message += "rebooting ...</body>\n";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
    delay(500);
    ESP.restart();    
}

//---------------------------------------------------------------------------------
//-- 404
/* static void handle_notFound(){
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += webServer.uri();
    message += "\nMethod: ";
    message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += webServer.args();
    message += "\n";
    for (uint8_t i = 0; i < webServer.args(); i++){
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
    }
    webServer.send(404, FPSTR(kTEXTPLAIN), message);
} */

#define DBG_OUTPUT_PORT swSer

String getContentType(String filename) {
  if (webServer.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".txt")) {
    return "text/plain";
  } else if (filename.endsWith(".json")) {
    return "application/json";
  }
  return "text/plain";
}


extern bool r900x_saveparams(String file); // its in main.cpp

/*
void save900xparams() { 

    String message = FPSTR(kHEADER);
    if (r900x_saveparams("/r900x_params.txt") ) { 
        message += "900x radio params saved to modem OK. ";
    } else { 
        message += "900x radio params save FAILED. Does file /r900x_params.txt exist? ";
    }
    message =+ "<a href=/>Click here to continue.</a></body></html>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
} 
*/

// by storing all the big files ( especially javascript ) in SPIFFS as .gz, we can save a bunch of space easily.
// but this can present either form to the user
bool handleFileRead(String path) {
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz)) {
      path += ".gz";
    }
    File file = SPIFFS.open(path, "r");
    webServer.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload() {
    //DBG_OUTPUT_PORT.println("handleFileUpload 0: "); 
  if (webServer.uri() != "/edit") {
    return;
  }

  HTTPUpload& upload = webServer.upload();
  DBG_OUTPUT_PORT.print("status:"); DBG_OUTPUT_PORT.println(upload.status);
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if (fsUploadFile) {
      fsUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
      DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
      webServer.sendHeader("Location","/success.htm");      // Redirect the client to the success page
      webServer.send(303);
    } else { 
      webServer.send(500, "text/plain", "500: couldn't create file");
    }
    //DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
  //webServer.send(200, "text/plain", "");
}

// /plist
void handle900xParamList() { 

    File html = SPIFFS.open("/r900x_params.htm", "r");
 
    html.setTimeout(50); // don't wait long as it's a file object, not a serial port.
 
    if ( ! html ) {  // did we open this file ok, ie does it exist? 
        swSer.println("no /r900x_params.htm exists, can't display modem settings properly.\n");
        webServer.send(200, "text/html", "ERR,couldn't do it - /r900x_params.htm is missing from spiffs."); return;
    }


    String contentType = getContentType("/r900x_params.htm");
    webServer.streamFile(html, contentType);
    html.close();

    return;
}

// do AT and RT commands to get 
extern int r900x_getparams(String filename,bool factory_reset_first); // see main.cpp
extern void r900x_setup(bool refresh);

// /prefresh
void handle900xParamRefresh() { 

    String type = webServer.arg("type");

    String factory = webServer.arg("factory");
    bool f = false;
    // a factory-default of the 900 radios is actually a r900x_getparams() request with the 2nd "factory_reset_first" parameter true
    if ( factory == "yes" ) {  f = true; }

    int num_read = 0;
    if ( type == "local" ) { 
    num_read = r900x_getparams("/r900x_params.txt",f);
    }
    if ( type == "remote" ) { 
    num_read = r900x_getparams("/r900x_params_remote.txt",f);
    }
    if ( type == "setup" ) { 
    r900x_setup(false);
    }
    if ( type == "reflash" ) { 
    r900x_setup(true);
    }

    //todo would this be better sent as json ? 
    //var mydata = JSON.parse(data);
    type = "handle900xParamRefresh() executed. type:"+type+" num_read:"+num_read;


    webServer.send(200, "text/plain", type );

} 


// spiffs files  goto url:  http://192.168.4.1/list?dir=/
void handleFileList() {
  if (!webServer.hasArg("dir")) {
    webServer.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = webServer.arg("dir");
  DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") {
      output += ',';
    }
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  webServer.send(200, "text/json", output);
}

void handleFileDelete() {
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  String path = webServer.arg(0);
  DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if (path == "/") {
    return webServer.send(500, "text/plain", "BAD PATH");
  }
  if (!SPIFFS.exists(path)) {
    return webServer.send(404, "text/plain", "FileNotFound");
  }
  SPIFFS.remove(path);
  webServer.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  String path = webServer.arg(0);
  DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if (path == "/") {
    return webServer.send(500, "text/plain", "BAD PATH");
  }
  if (SPIFFS.exists(path)) {
    return webServer.send(500, "text/plain", "FILE EXISTS");
  }
  File file = SPIFFS.open(path, "w");
  if (file) {
    file.close();
  } else {
    return webServer.send(500, "text/plain", "CREATE FAILED");
  }
  webServer.send(200, "text/plain", "");
  path = String();
}

void handle900xParamSave() { 
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  //String p = webServer.arg('params');
   // DBG_OUTPUT_PORT.println("handle900xParamSave: " + p);


  String message = "";
  for (uint8_t i = 0; i < webServer.args(); i++) {
    message += " " + webServer.argName(i) + " -> " + webServer.arg(i) + "\n";
  }

  int retval = 404; // if its good, then some sort of 200 message, depending on what happens.

  File f;
  String filename = "/r900x_params.txt"; // default name
  if (webServer.hasArg("f")) { 
       filename = webServer.arg("f");
       f = SPIFFS.open(filename, "w");
  } else { 
      f = SPIFFS.open(filename, "w");
  }

  if(webServer.hasArg(kPLAIN)) { // ie PUT/POST content was given....
        // write params to spiffs,  TODO need better checks here, as it comes from the client, but the worse they can do is pretty minor
       //File f = SPIFFS.open("/r900x_params.txt", "w");
       f.print(webServer.arg(kPLAIN));
       f.close();
       message += "\nFile written:";
       message += filename;
       message += "\n";

        if (r900x_saveparams(filename) ) { 
            message += "and 900x radio params activated to modem OK. ";
            retval = 201;
        } else { 
            message += "and 900x radio params activate FAILED. Does file /r900x_params.txt exist? ";
            retval = 202;
        }
    
  }

  webServer.send(retval, "text/plain", message);

  DBG_OUTPUT_PORT.println("handle900xParamSave: " + message);




// optionally run  save900xparams() or r900x_saveparams() 

}

//---------------------------------------------------------------------------------
MavESP8266Httpd::MavESP8266Httpd()
{

}

//---------------------------------------------------------------------------------
//-- Initialize


void
MavESP8266Httpd::begin(MavESP8266Update* updateCB_)
{
    updateCB = updateCB_;
    webServer.on("/",               handle_root);
    webServer.on("/getparameters",  handle_getParameters);
    webServer.on("/setparameters",  handle_setParameters);
    webServer.on("/getstatus",      handle_getStatus);
    webServer.on("/getstatus_tcp",  handle_getStatusTcp);
    webServer.on("/reboot",         handle_reboot);
    webServer.on("/setup",          handle_setup);
    webServer.on("/info.json",      handle_getJSysInfo);
    webServer.on("/status.json",    handle_getJSysStatus);
    webServer.on("/log.json",       handle_getJLog);

    webServer.on("/updatepage",       handle_update_html); // presents a webpage that then might upload a binary to the /upload endpoint via POST


//    webServer.on("/save900xparams",         save900xparams); // see /psave

    
    webServer.on("/upload",         HTTP_POST, handle_upload, handle_upload_status); // this save/s an uploaded file


    //list directory, return details as json.
    webServer.on("/list", HTTP_GET, handleFileList);

    //load editor
    webServer.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) {
      webServer.send(404, "text/plain", "FileNotFound");
    }
    });

    //create file
    webServer.on("/edit", HTTP_PUT, handleFileCreate);

    //delete file
    webServer.on("/edit", HTTP_DELETE, handleFileDelete);


   //TIP:  /edit endpoint uses ace.js and friends from https://github.com/ajaxorg/ace-builds/tree/master/src

    //first callback is called after the request has ended with all parsed arguments
    //second callback handles file uploads at that location
    //webServer.on("/edit", HTTP_POST, handleFileUpload);

    //webServer.send(200, "text/plain", "");
    //}, handleFileUpload);

//    webServer.on("/edit", HTTP_POST, [](){ webServer.send(200, "text/plain", "yup."); }, handleFileUpload);

    webServer.on("/edit", HTTP_POST, [](){ webServer.sendHeader("Location","/success.htm"); webServer.send(303); }, handleFileUpload);

    //      // Redirect the client to the success page
    //webServer.send(303);

    webServer.on("/plist", HTTP_GET, handle900xParamList);

    webServer.on("/prefresh", HTTP_GET, handle900xParamRefresh);

    webServer.on("/psave", HTTP_PUT, handle900xParamSave);

    //called when the url is not defined here
    //use it to load content from SPIFFS
    webServer.onNotFound([]() {
    if (!handleFileRead(webServer.uri())) {
      webServer.send(404, "text/plain", "FileNotFound");
    }
    });

    //const char* webupdatehost = "esp8266-webupdate";

    //MDNS.begin(webupdatehost);

    httpUpdater.setup(&webServer);

    webServer.begin();

    //MDNS.addService("http", "tcp", 80);
    //DBG_OUTPUT_PORT.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", webupdatehost);
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::checkUpdates()
{
    webServer.handleClient();
}
