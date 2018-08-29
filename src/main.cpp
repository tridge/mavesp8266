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

#include "SmartSerial.h"

// LED is on GPIO2
#define GPIO02  2
#define LED 2


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

uint8 client_count = 0;


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


HardwareSerial Serial9x(1); //  attached 900x device is on 'Serial' and instantiated as a Serial9x object. 
MySerial *SmartSerial = new MySerial(&Serial9x); 
File f; // global handle use for the Xmodem upload


bool r900x_sync() { 
      // __sync
      
      //
      Serial.write("U"); // autobauder
      Serial.flush(); 
      
      //Serial.write("\rCHIPID\r");
      //Serial.flush(); // output buffer flush

     // chipid info etc
     // swSer.println("\t\tCHIPID\r\n");
 
      // __getSync

      bool ok = SmartSerial->expect("ChipID:",500); 
      if ( ok ) { 
          swSer.println("\t\tGOT ChipID Response from radio.\n");
          //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
      } else { 

          swSer.println("\t\tFAILED-TO-GET ChipID Response from radio.\n");
      }  
      return ok;
      
}


bool r990x_getparams() { 

        Serial.write("\r");
        delay(1000); 
        Serial.write("+++");
        Serial.flush(); // output buffer flush

        bool ok = SmartSerial->expect("OK",2000); 
        if ( ok ) { 
            swSer.println("GOT OK Response from radio.\n");
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 
            //trying = false; HACK
            swSer.println("FALED-TO-GET OK Response from radio.\n");
            return false;
        } 

        //  read params from radio , and write to a file in spiffs.
        Serial.write("ATI5\r");
        Serial.flush(); // output buffer flush
        swSer.print("----------------------------------------------");
        String data = SmartSerial->expect_s("SIS_RSSI=50\r\n",3000); 
        while (Serial.available() ) { char t = Serial.read();  data += t; } // flush read buffer upto this point.
        swSer.print(data);
        swSer.print("----------------------------------------------");
        
        // also write params to spiffs, for user record:
        f = SPIFFS.open("/r990x_params.txt", "w");
        f.print(data);
        f.close();

        Serial.write("AT&Z\r"); // reboot radio to restore non-command mode.
        Serial.flush(); // output buffer flush
        delay(200); 

     return true;
} 

// TODO 
bool r990x_saveparams() { 

// iterate over the params found in r990x_params.txt and save those that aren't already set correctly


    // put it into command mode first....
    Serial.write("\r");
    delay(1000); 
    Serial.write("+++");
    Serial.flush(); // output buffer flush

    bool ok = SmartSerial->expect("OK",2000); 
    if ( ok ) { 
        swSer.println("GOT OK Response from radio.\n");
        while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
    } else { 
        //trying = false; HACK
        swSer.println("FALED-TO-GET OK Response from radio, might already be in command mode....\n");
        //return false;
    } 


    File f = SPIFFS.open("/r990x_params.txt", "r");

    f.setTimeout(200); // don't wait long as it's a file object, not a serial port.

    if ( ! f ) {  // did we open this file ok, ie does it exist? 
        swSer.println("no /r990x_params.txt exists, can't update modem settings, skipping.\n");
        return false;
    }

    int done = 0; 
    while ( done < 30 ) { 

        String line = f.readStringUntil('\n'); // read a line, wait max 100ms.

        if (line.substring(0,3) == "ATI" ) { continue; } // ignore this first line.
        //if (line == "\r\n" ) { continue; } // ignore blank line/s
        
        int colon_offset = line.indexOf(":"); // it should have a colon, and an = sign
        int equals_offset = line.indexOf("="); // it should have a colon, and an = sign
        int eol_offset = line.indexOf("\r"); // line ends with \r\n. this finds the first of these

        // if all of them is -1, it's the end of the file.
        if (( colon_offset == -1 ) && ( equals_offset == -1 ) && ( eol_offset == -1 ) ) {    
            //return true;
            goto save;
        } 

        //  if any of these is -1, it failed, just skip that line
        if (( colon_offset == -1 ) || ( equals_offset == -1 ) || ( eol_offset == -1 ) ) {    
            swSer.println("skipping line as it didn't parse well.");
            continue;
        } 


        String ParamID = line.substring(0,colon_offset);
        String ParamNAME = line.substring(colon_offset+1,equals_offset);
        String ParamVAL = line.substring(equals_offset+1,eol_offset); 


        String ParamCMD= "AT"+ParamID+"="+ParamVAL+"\r\n";
        swSer.println(ParamCMD); // debug only.
        Serial.write(ParamCMD.c_str());
        Serial.flush(); // output buffer flush
        bool ok = SmartSerial->expect("OK",1000); 
        if ( ok ) { 
            swSer.println("GOT OK from radio.\n");
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 
            //trying = false; HACK
            swSer.println("FALED-TO-GET OK Response from radio.\n");
        } 
        
        done++;
    }   

    

    // jump here when done writing parms:
   save:

   // save params AND take out of command mode via a quick reboot
    Serial.write("AT&W\r\nAT&Z\r\n");
    Serial.flush(); // output buffer flush


    ok = SmartSerial->expect("OK",2000); 
    if ( ok ) { 
        swSer.println("GOT OK Response from radio.\n");
        while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
    } else { 
        //trying = false; HACK
        swSer.println("FALED-TO-GET OK Response from radio, no biggie, reboot expected.\n");
        //return false;
    } 
    return true;  
} 




bool r900x_autosync() { 

    swSer.println("Trying autosync....\n");
    
        Serial.write("\r");
        delay(1000); 
        Serial.write("+++");
        Serial.flush(); // output buffer flush

        bool ok = SmartSerial->expect("OK",2000); 
        if ( ok ) { 
            swSer.println("GOT OK Response from radio.\n");
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 
            //trying = false; HACK
            swSer.println("FALED-TO-GET OK Response from radio.\n");
        } 

        // we try the ATI style commands upto 5 times to get a 'sync' if needed.
        for (int i=0; i < 5; i++){

           // int i=0;


            swSer.print("\tATI Attempt Number: "); swSer.println(i);
            swSer.write("\tSending ATI to radio.\r");
            //
            Serial.write("ATI\r");
            Serial.flush(); // output buffer flush

            ok = SmartSerial->expect("SiK",2000); 
            if ( ok ) { 
                swSer.println("\tGOT SiK Response from radio.\n");
            } else { 
                swSer.println("\tFAILED-TO-GET SiK Response from radio.\n");
                continue;
            } 

            //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point, displaying it for posterity ( it has version info )


            swSer.println("\t\tSending AT&UPDATE to radio...\n");
            //
            Serial.write("\r\n");
            delay(200); 
            Serial.flush();
            Serial.write("AT&UPDATE\r"); // must be \r only, do NOT include \n here
            delay(700); 
            Serial.flush();

            swSer.println("Sent update command");

            return true;
        }
  return false;
}

bool r900x_check() { 
  for (int r=0; r < 3; r++){

        bool ok = r900x_sync();
        if ( ok ) { swSer.println("got sync");  return true; }

        ok = r900x_autosync();

        if ( ok) return true;
        
  }
  return false;
}

bool r900x_upload () { 

    swSer.print("--#");
    while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.
    swSer.println("#--");

    Serial.write("U"); // 57600 AUTO BAUD RATE CODE EXPECTS THIS As the first byte sent to the bootloader, not even \r or \n should be sent earlier
    Serial.flush();

    bool ok = SmartSerial->expect("ChipID:",2000);  // response to 'U' is the long string including chipid

    delay(200);

    while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.

    // UPLOAD COMMAND EXPECTS 'Ready' string
    swSer.println("\t\tUPLOAD\r\n");
    //
    Serial.write("UPLOAD\r");
    Serial.flush(); // output buffer flush

    ok = SmartSerial->expect("Ready",3000); 
    ok = SmartSerial->expect("\r\n",1000); 

    if ( ok ) { 
        swSer.println("\t\tGOT UPLOAD/Ready Response from radio.\n");
        while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.                        
    } else { 

        swSer.println("\t\tFAILED-TO-GET UPLOAD/Ready Response from radio.\n");
        return false;
    } 

    // TODO xmodem stuff...
    swSer.println("bootloader handshake done");

    swSer.println("\t\tXMODEM SENDFILE BEGAN\n");
    xmodem.sendFile(f, "unused");
    swSer.println("\t\tXMODEM SENDFILE ENDED\n");

    //f.close();

    swSer.println("Booting new firmware");


    Serial.write("BOOTNEW\r");
    swSer.println("\t\tBOOTNEW\r\n");

    return true;
}


void r900x_setup() { 

    //delay(3000);  // allow time for 900x radio to complete its startup.? 

    if ( SPIFFS.begin() ) { 
      swSer.println("spiffs started\n");
    } else { 
      swSer.println("spiffs FAILED to start\n");

      //# TODO create/format SPIFFS filesystem and leave it empty. ?
    }

    /* 
    Dir dir = SPIFFS.openDir(""); //read all files
    while (dir.next()) {
      swSer.print(dir.fileName()); swSer.print(" -> ");
      File f = dir.openFile("r");
      swSer.println(f.size());
      f.close();
    }*/


#define BOOTLOADERNAME "/RFDSiK900x.bin"
#define BOOTLOADERCOMPLETE "/RFDSiK900x.bin.ok"

//#define BOOTLOADERNAME "/RFDSiK900x.bin.ok"
//#define BOOTLOADERCOMPLETE "/RFDSiK900x.bin"

    f = SPIFFS.open(BOOTLOADERNAME, "r");

    if ( ! f ) {  // did we open this file ok, ie does it exist? 
        swSer.println("no firmware to program, skipping reflash.\n");
        return;
    }


    if ( !r900x_check() ) { 
       swSer.println("Failed to contact bootloader");
    } else { 
        
        if (r900x_upload() ) { 
            swSer.println("renamed firmware file after successful flash.");
            SPIFFS.rename(BOOTLOADERNAME, BOOTLOADERCOMPLETE); // after a successful upload to the 900x radio, rename it out of the way.
        } else { 
            swSer.println("Failed to upload to 900x, will retry on next boot.");
        }
    }
    
    r990x_getparams();  

    f.close();


} 

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Parameters.begin();

#ifdef ENABLE_SOFTDEBUG
    // software serial on unrelated pin/s for usb/serial/debug
    swSer.begin(57600);
    swSer.println("swSer output for SOFTDEBUG");
#endif

   Serial.begin(57600); // needed to force baud rate of hardware to right mode for 900x bootloader reflash code.

   SmartSerial->begin();

#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(57600);
    swSer.println("Serial1 output for DEBUG");
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif

    Logger.begin(2048);

    SoftLogger.begin(2048);

    r900x_setup(); // probe for 900x and if a new firware update is needed , do it.


    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        DEBUG_LOG("\nEntering station mode...\n");
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
        DEBUG_LOG("\nEntering AP mode...\n");
        //-- Start AP
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        //wait_for_client();
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


void client_check() { 

    uint8 x = wifi_softap_get_station_num();
    if ( client_count != x ) { 
        Serial.println("new client/s connected"); 
        client_count = x;
        DEBUG_LOG("Got %d client(s)\n", client_count); 
    } 
} 
//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {

    client_check(); 

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
