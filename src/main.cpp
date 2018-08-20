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
 * @file main.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */


#include <SoftwareSerial.h>

SoftwareSerial swSer(14, 16, false, 256);


#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_component.h"
#include "FS.h" // for SPIFFS acccess

#include <XModem.h> // for firmware updates

#include <ESP8266mDNS.h>

#define GPIO02  2

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update {
public:
    MavESP8266UpdateImp ()
        : _isUpdating(false)
    {

    }
    void updateStarted  ()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError    ()
    {
        //-- TODO
    }
    bool isUpdating     () { return _isUpdating; }
private:
    bool _isUpdating;
};



//-- Singletons
IPAddress               localIP;
MavESP8266Component     Component;
MavESP8266Parameters    Parameters;
MavESP8266GCS           GCS;
MavESP8266Vehicle       Vehicle;
MavESP8266Httpd         updateServer;
MavESP8266UpdateImp     updateStatus;
MavESP8266Log           Logger;
MavESP8266Log           SoftLogger;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World {
public:
    MavESP8266Parameters*   getParameters   () { return &Parameters;    }
    MavESP8266Component*    getComponent    () { return &Component;     }
    MavESP8266Vehicle*      getVehicle      () { return &Vehicle;       }
    MavESP8266GCS*          getGCS          () { return &GCS;           }
    MavESP8266Log*          getLogger       () { return &Logger;        }
    MavESP8266Log*          getSoftLogger       () { return &SoftLogger;        }
};

MavESP8266WorldImp      World;

MavESP8266World* getWorld()
{
    return &World;
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    DEBUG_LOG("Waiting for a client...\n");
#ifdef ENABLE_DEBUG
    int wcount = 0;
#endif
    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count) {
#ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
#endif
        delay(1000);
        client_count = wifi_softap_get_station_num();
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
    ESP.reset();
}


XModem xmodem(&Serial, ModeXModem);


//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Parameters.begin();

#ifdef ENABLE_SOFTDEBUG
    // software serial on unrelated pin/s for usb/serial/debug
    swSer.begin(115200);
    swSer.println("swSer output for SOFTDEBUG");
#endif

   Serial.begin(115200);

#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(115200);
    swSer.println("Serial1 output for DEBUG");
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif

    Logger.begin(2048);

    SoftLogger.begin(2048);

    while (Serial.available()){
        swSer.print(Serial.read());
    }
    

    swSer.println("looking at spiffs\n");
    
    if ( SPIFFS.begin() ) { 

        swSer.println("spiffs started\n");

        Dir dir = SPIFFS.openDir(""); //read all files
        while (dir.next()) {
            swSer.print(dir.fileName()); swSer.print(" -> ");
            File f = dir.openFile("r");
            swSer.println(f.size());
        }

        //byte binary_firmware_data[120000];  // currently about 102k, so 120k is enough.
        int bytecount = 0;
        bool trying = true;
        File f = SPIFFS.open("/RFDSiK900x.bin", "r");
        if (!f) {
            swSer.println("firmware file open from SPIFFS failed: /RFDSiK900x.bin\n");
            trying = false;
        } else { 

            swSer.println("firmware file open from SPIFFS !: /RFDSiK900x.bin\n");

            // This sucks entire firmware data into ram, it's only ~100k, so we can get away with this.
            swSer.println("file size clecking....\n");
            while (f.available()){
                int t = f.read();
                bytecount++;
            }

            swSer.println("file size cleck complete\n");
            if (bytecount < 50000 ) { trying = false;} 
            //f.close();
            swSer.println("blah\n");

            // lets make at most 3 attempts to enter AT command mode...
            for (int r=0; r < 3; r++){

                swSer.print("+++ Attempt Number: "); swSer.println(r);

                // try to drop 900 radio into bootloader mode
                Serial.write("\r");
                delay(1000); 
                Serial.write("+++");
                delay(1000); 
                Serial.write("AT\r\n");
                String response = "";
                // look for response, for max 2 seconds
                unsigned long now = millis(); 
                swSer.print("waiting for +++ and AT results"); swSer.println(r);
//                while (Serial.available() && (now+50000 > millis() ) )     {
                while ( (now+50000 > millis() ) )     {

                    char c = Serial.read();
                    response += c;
                    swSer.print(c);
                }
                swSer.print("got +++ and AT results"); swSer.println(r);
                if (trying && (response.indexOf("OK") > 0)) {
                    swSer.println("GOT OK Response from radio.\n");
                } else { 
                    trying = false;
                    swSer.println("FALED-TO-GET OK Response from radio.\n");
                } 

                
                // we try the ATI style commands upto 5 times to get a 'sync' if needed.
                for (int i=0; i < 5; i++){

                    swSer.print("ATI Attempt Number: "); swSer.println(i);

                    // now check if the ATI command says it's a SiK modem of some sort
                    now = millis(); 
                    response = "";
                    if ( trying ) { 
                        Serial.write("ATI\r");
                        while (Serial.available() && (now+2000 > millis() ) )     {
                        char c = Serial.read();
                            response += c;
                        }
                    } 
                    swSer.print("ATI reslts.. "); swSer.println(i);
                    if (trying && (response.indexOf("SiK") > 0)) {
                        swSer.println("GOT SiK Response from radio.\n");
                    } else { 
                        trying = false;
                        swSer.println("FAILED-TO-GET SiK Response from radio.\n");
                    } 

                    // now try to put it into bootloader mode
                    if (trying) { 
                        Serial.write("\r\n");
                        delay(200); 
                        Serial.flush();
                        Serial.write("AT&UPDATE\r\n");
                        delay(700); 
                        Serial.flush();
                    }

                    // upload!
                    if (trying) { 
                    
                        //print("Uploading %s" % fw)
                        //self.fw_size = os.path.getsize(fw)

                        Serial.write("UPLOAD\r");
                        swSer.println("UPLOAD\r\n");

                        //self.expect("Ready", 2)
                        now = millis(); 
                        response = "";
                        Serial.write("UPLOAD\r");
                        while (Serial.available() && (now+2000 > millis() ) )     {
                        char c = Serial.read();
                            response += c;
                        }
                        if ((response.indexOf("Ready") > 0)) {
                            swSer.println("GOT Ready Response from radio.\n");
                        } else { 
                            trying = false;
                            swSer.println("FAILED-TO-GET Ready Response from radio.\n");
                       } 

                        //self.expect("\r\n", 1) //TODO do we need to wait for this newline too? 


                        xmodem.sendFile(f, "unused");

                        //f = open(fw, 'rb')
                        //xm = xmodem.XMODEM(self.getc, self.putc)
                        //xm.send(f, callback=self.callback)

                        f.close();
                        //print("Booting new firmware")
                        Serial.write("BOOTNEW\r");
                        swSer.println("BOOTNEW\r\n");

                    } 

                }
            }

        }
    }


    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        //-- Connect to an existing network
        WiFi.mode(WIFI_STA);
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());

        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            #ifdef ENABLE_DEBUG
            Serial.print(".");
            #endif
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
        }
    }

    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        //-- Start AP
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        wait_for_client();
    }

    //-- Boost power to Max
    WiFi.setOutputPower(20.5);
    //-- MDNS
    char mdsnName[256];
    sprintf(mdsnName, "MavEsp8266-%d",localIP[3]);
    MDNS.begin(mdsnName);
    MDNS.addService("http", "tcp", 80);
    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());

    Parameters.setLocalIPAddress(localIP);
    IPAddress gcs_ip(localIP);
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    gcs_ip[3] = 255;
    GCS.begin(&Vehicle, gcs_ip);
    Vehicle.begin(&GCS);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        if (Component.inRawMode()) {
            GCS.readMessageRaw();
            delay(0);
            Vehicle.readMessageRaw();

        } else {
            GCS.readMessage();
            delay(0);
            Vehicle.readMessage();
        }
    }
    updateServer.checkUpdates();
}
