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
 * original @author Gus Grubba <mavlink@grubba.com>
 * TCP support by davidbuzz@gmail.com
 * txmod/900x bootloader and flashing support by davidbuzz@gmail.com
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

// LED is on GPIO2, see also XModem.cpp, line 125
//#define GPIO02  2
#define LEDGPIO 2
// txmod reset button 
#define RESETGPIO 12

// platformio doesn't seem to have F(), but has FPSTR and PSTR
#define F(string_literal) (FPSTR(PSTR(string_literal)))


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
MavESP8266Parameters    Parameters; // esp params

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

// global for LED state.
bool LEDState = 0;

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    LEDState = 1;
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
        LEDState = !LEDState;
        digitalWrite(LEDGPIO,LEDState);
        client_count = wifi_softap_get_station_num();
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
    LEDState = 0;
    digitalWrite(LEDGPIO,LEDState);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    swSer.println(F("FACTORY RESET BUTTON PRESSED - wifi params defaulted!\n"));
    swSer.flush();
    LEDState = 1;
    digitalWrite(LEDGPIO,LEDState);
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
    ESP.reset();
}

// count the number of user presses, and when it exceeds 5, reset to defaults.
volatile byte interruptCounter = 0;
volatile long first_press = 0;
void count_interrupts() { 
    interruptCounter++;
    if ( first_press == 0 ) { first_press = millis(); } 
    if ( millis() - first_press > 5000) { first_press = 0; interruptCounter = 0; } 
    if ( interruptCounter >= 5 ) { reset_interrupt(); }// reboot after 5 presses
} 

XModem xmodem(&Serial, ModeXModem);


HardwareSerial Serial9x(1); //  attached 900x device is on 'Serial' and instantiated as a Serial9x object. 
MySerial *SmartSerial = new MySerial(&Serial9x); 
File f; // global handle use for the Xmodem upload


int r900x_booloader_mode_sync() { 

swSer.println(F("r900x_booloader_mode_sync()\n"));
      Serial.write("U"); // autobauder
      Serial.flush(); 

      int ok = SmartSerial->expect_multi3("ChipID:", "UPLOAD", "XXXXXXXXXXX",500); // we're really looking for a ChipID, but UPLOAD will do too, and XXX is an unused parameter
      if ( ok > 0 ) { 
          swSer.println(F("\t\tGOT ChipID/UPLOAD Response from radio.\n"));
          while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
          return ok;
      } else { 
          swSer.println(F("\t\tFAILED-TO-GET ChipID/UPLOAD Response from radio.\n"));
          return -1;
      }       
}


bool have_we_chatted_to_modem_yet = false;

// returns -1 on major error, positive number of params fetched on 'success'
int r900x_getparams(String filename, bool factory_reset_first) { 


   swSer.println(F("r900x_getparams()- START\n"));
   if (have_we_chatted_to_modem_yet == false ) { 
    have_we_chatted_to_modem_yet = true;
    }

   int chances = 0;
   retry:
        Serial.write("\r");
        Serial.flush(); // output buffer flush
        delay(1500); 
        Serial.write("+++");
        Serial.flush(); // output buffer flush
        LEDState = !LEDState;
        digitalWrite(LEDGPIO,LEDState);
        //bool ok = SmartSerial->expect("OK",1000); 
        int ok2 = SmartSerial->expect_multi3("OK","+++","XXXXX",500); // look for 'ok' or '+++'

        if ( ok2 == 1 ) { 
            swSer.println(F("GOT OK Response from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        }
        else if ( ok2 == 2 ) { 
            // if modem echo's back the +++ we sent it, we are already in command mode.
            goto already_in_cmd_mode;
        } else { 
            //trying = false; HACK
            swSer.print(F("FALED-TO-GET OK Response from radio. chance:"));
            swSer.println(chances);
            // lets give it two chances, then give up, not just one. 
            chances++;
            if (chances < 2 ) goto retry;
            //return false; // might already be in command mode, let it try anyway...
            
        } 

        already_in_cmd_mode:

        delay(1500); // give a just booted radio tim to be ready.

        Serial.write("\r"); // after +++ we need to clear the line before we set AT commands
        Serial.flush(); // output buffer flush
        delay(500); // give a just booted radio tim to be ready.

        while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this 

        //  read AT params from radio , and write to a file in spiffs.
        String prefix = "AT";
        if (filename == "/r900x_params_remote.txt" ) {prefix = "RT";}

        // untested- implement factory_reset_first boolean
        if ( factory_reset_first ) { 
            //String cmd = prefix+"&F\r"+
            String factorycmd = prefix+"&W\r\n"+prefix+"&F\r\n"; 
            Serial.write(factorycmd.c_str());
            Serial.flush(); // output buffer flush
            //swSer.print(F("----------------------------------------------"));
            bool b = SmartSerial->expect("OK",3000); 
            while (Serial.available() ) { char t = Serial.read();  } // flush read buffer upto this point and discard
            swSer.print(F("attempted factory reset."));
        }

        String cmd = prefix+"I5\r";
        Serial.write(cmd.c_str());
        Serial.flush(); // output buffer flush
        swSer.print(F("----------------------------------------------"));
        String data = SmartSerial->expect_s("SIS_RSSI=50\r\n",3000); 
        while (Serial.available() ) { char t = Serial.read();  data += t; } // flush read buffer upto this point.
        swSer.print(data);
        swSer.print(F("----------------------------------------------"));

        // count number of lines in output, and return it as result.
        int linecount = 0;
        for ( int c = 0 ; c < data.length(); c++ ) { 
            if ( data.charAt(c) == '\n' ) { linecount++; }
        }

        // as user experience uses the SPIFFS .txt to render the html page, we cleanup an old one if we've been asked
        // to get fresh params, even if we cant replace it, as the *absense* of it mean sthe remote radio is no longer connected.
        SPIFFS.remove(filename); 

        
        // also write params to spiffs, for user record:
        if ( data.length() > 300 ) { // typical file length is around 400chars 
            f = SPIFFS.open(filename, "w");
            f.print(data);
            f.close();
        } else { 
            swSer.println(F("didn't write param file, as it contained insufficient data"));
        }

        delay(1000); // time for local and remote to sync and RT values to populate.

        cmd = prefix+"&Z\r";
        Serial.write(cmd.c_str()); // reboot radio to restore non-command mode.
        Serial.flush(); // output buffer flush
        delay(200); 

        swSer.println(F("r900x_getparams()- END\n"));

     return linecount;
} 


bool r900x_saveparams(String filename) { 
swSer.println(F("r900x_saveparams()\n"));
// iterate over the params found in r900x_params.txt and save those that aren't already set correctly


    // put it into command mode first....
    Serial.write("\r");
    delay(1000); 
    Serial.write("+++");
    Serial.flush(); // output buffer flush

    bool ok = SmartSerial->expect("OK",2000); 
    if ( ok ) { 
        swSer.println(F("GOT OK Response from radio.\n"));
        while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
    } else { 
        //trying = false; HACK
        swSer.println(F("FALED-TO-GET OK Response from radio, might already be in command mode....\n"));
        //return false;
    } 


    String localfilename = "/r900x_params.txt";
    String remotefilename = "/r900x_params_remote.txt";

    File f = SPIFFS.open(filename, "r");    

    String prefix = "AT";
    if ( filename == remotefilename ) { prefix = "RT"; } 

    f.setTimeout(200); // don't wait long as it's a file object, not a serial port.

    if ( ! f ) {  // did we open this file ok, ie does it exist? 
        swSer.println(F("no txt file exists, can't update modem settings, skipping:"));
        swSer.println(filename);
        return false;
    }

    int done = 0; 
    int failurecount = 0;
    while ( done < 30 ) { 

        LEDState = !LEDState;
        digitalWrite(LEDGPIO,LEDState);

        String line = f.readStringUntil('\n'); // read a line, wait max 100ms.

        if (line.substring(0,3) == "ATI" ) { continue; } // ignore this first line.
        if (line.substring(0,3) == "RTI" ) { continue; } // ignore this first line.

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
            swSer.println(F("skipping line as it didn't parse well."));
            continue;
        } 


        String ParamID = line.substring(0,colon_offset);
        String ParamNAME = line.substring(colon_offset+1,equals_offset);
        String ParamVAL = line.substring(equals_offset+1,eol_offset); 



        
        String ParamCMD= prefix+ParamID+"="+ParamVAL+"\r\n";

        swSer.println(ParamCMD); // debug only.
        Serial.write(ParamCMD.c_str());
        Serial.flush(); // output buffer flush
        bool ok = SmartSerial->expect("OK",1000); 
        if ( ok ) { 
            swSer.println(F("GOT OK from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 
            //trying = false; HACK
            swSer.println(F("FALED-TO-GET OK Response from radio.\n"));
            failurecount++;
            if ( failurecount > 2 ) return false; // in the event that we don't have modem, this fails it faster
        } 
        
        done++;
    }   

    
    // jump here when done writing parms:
   save:

   // save params AND take out of command mode via a quick reboot
    String rebootcmd = prefix+"&W\r\n"+prefix+"&Z\r\n"; // "AT&W\r\nAT&Z\r\n"
    Serial.write(rebootcmd.c_str());
    Serial.flush(); // output buffer flush

    LEDState = 0;
    digitalWrite(LEDGPIO,LEDState);

    ok = SmartSerial->expect("OK",2000); 
    if ( ok ) { 
        swSer.println(F("GOT OK Response from radio.\n"));
        while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
    } else { 
        //trying = false; HACK
        swSer.println(F("FALED-TO-GET OK Response from radio, no biggie, reboot expected.\n"));
        //return false;
    } 
    return true;  
} 


bool got_hex = false;
bool r900x_command_mode_sync() { 
        swSer.println(F("r900x_command_mode_sync()\n"));
        swSer.println(F("Trying command-mode sync....\n"));
    
        Serial.write("\r");
        delay(1000); 
        Serial.write("+++");
        Serial.flush(); // output buffer flush

        bool ok = SmartSerial->expect("OK",2000); 
        if ( ok ) { 
            swSer.println(F("GOT OK Response from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 
            //trying = false; HACK
            swSer.println(F("FALED-TO-GET OK Response from radio.\n"));
        } 

        // we try the ATI style commands up to 2 times to get a 'sync' if needed. ( was 5, but 2 is faster )
        for (int i=0; i < 2; i++) {

            LEDState = !LEDState;
            digitalWrite(LEDGPIO,LEDState);

            swSer.print(F("\tATI Attempt Number: ")); swSer.println(i);
            swSer.write("\tSending ATI to radio.\r");
            //
            Serial.write("ATI\r");
            Serial.flush(); // output buffer flush

            int ok2 = SmartSerial->expect_multi3("SiK","\xC1\xE4\xE3\xF8","\xC1\xE4\xE7\xF8",2000);  // we really want to see 'Sik' here, but if we see the hex string/s we cna short-curcuit 

            if ( ok2 == 1 ) { 
                swSer.println(F("\tGOT SiK Response from radio.\n"));
            } 
            if ( ok2 == 2 ) { 
                swSer.println(F("\tGOT bootloader HEX (C1 E4 E3 F8 ) Response from radio.\n"));
                got_hex = true;
                return false;
            } 
            if ( ok2 == 3 ) { 
                swSer.println(F("\tGOT bootloader HEX (C1 E4 E7 F8 ) Response from radio.\n"));
                got_hex = true;
                return false;
            } 

            if ( ok2 == -1 ) { 
                swSer.println(F("\tFAILED-TO-GET SiK Response from radio.\n"));
                continue;
            } 

            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point, displaying it for posterity ( it has version info )


            swSer.println(F("\t\tSending AT&UPDATE to radio...\n"));
            //
            Serial.write("\r\n");
            delay(200); 
            Serial.flush();
            Serial.write("AT&UPDATE\r"); // must be \r only, do NOT include \n here
            delay(700); 
            Serial.flush();

            swSer.println(F("Sent update command"));

            return true;
        }
  return false;
}


void r900x_setup(bool reflash) { // if true. it will attempt to reflash, if false it will go through the motions.

    swSer.println(F("r900x_setup(...)\n"));
    delay(1000);  // allow time for 900x radio to complete its startup.? 


    LEDState = 0;
    digitalWrite(LEDGPIO,LEDState);

    if ( SPIFFS.begin() ) { 
      swSer.println(F("spiffs started\n"));
    } else { 
      swSer.println(F("spiffs FAILED to start\n"));

    }

    //Format File System if it doesn't at least have an index.htm file on it.
    if (!SPIFFS.exists("/index.htm")) {
        swSer.println(F("SPIFFS File System Format started...."));
        SPIFFS.format();
        swSer.println(F("...SPIFFS File System Format Done."));
    }
    else
    {
        swSer.println(F("SPIFFS File System left as-is."));
    }

    swSer.flush();

    // debug only, list files and their size-on-disk
    swSer.println(F("List of files in SPIFFs:"));
    Dir dir = SPIFFS.openDir(""); //read all files
    while (dir.next()) {
      swSer.print(dir.fileName()); swSer.print(F(" -> "));
      File f = dir.openFile("r");
      swSer.println(f.size());
      f.close();
    }

    // in the event we aren't reflashing, but have no parameters cached from the modem do that now
    File p = SPIFFS.open("/r900x_params.txt", "r"); 

    
    if ( !p ) { 
        swSer.println(F("can't see local parameters files, re-getting in 2 secs..."));
        delay(2000);
        r900x_getparams("/r900x_params.txt",false); 
    } 

    File p2 = SPIFFS.open("/r900x_params_remote.txt", "r");

    if ( !p2 ) {
        swSer.println(F("can't see remote parameters files, re-getting in 2 secs...."));
        delay(2000);
        r900x_getparams("/r900x_params_remote.txt",false); 
    } 


#define BOOTLOADERNAME "/RFDSiK900x.bin"
#define BOOTLOADERCOMPLETE "/RFDSiK900x.bin.ok"


    f = SPIFFS.open(BOOTLOADERNAME, "r");

    // if we've got anything available to try a reflash, must be a .bin
    if ( reflash) { 
        if ( ! f ) { 
            f.close();
            //swSer.println("flashing with .ok firrmware....\n");
            //f = SPIFFS.open(BOOTLOADERCOMPLETE, "r");
            f = SPIFFS.open(BOOTLOADERNAME, "r"); // retry opening .bin file, just in case it was intermittent.
        }
    }

    if ( ! f ) {  // did we open this file ok, ie does it exist? 
        swSer.println(F("no firmware .bin available to program, skipping reflash.\n"));
        return;
    }


    if (( f.size() > 0) and (f.size() < 90000 )) { 
        swSer.println(F("incomplete or too-small firmware for 900x to program ( < 90k bytes ), deleting corrupted file and skipping reflash.\n"));
        SPIFFS.remove(BOOTLOADERNAME); 
        return;   
    } 

   int xmodem_retries = 0;
retrypoint:
    xmodem_retries++;

    // first try at the default baud rate...
    int result = -1; // returns 1,2,3,4 or 5 on some sort of success

    for (int r=0; r < 3; r++){

        LEDState = !LEDState;
        digitalWrite(LEDGPIO,LEDState);

        swSer.println("Serial.begin(57600);");
        Serial.begin(57600);
        // first try to communicate with the bootloader, if possible....
        int ok = r900x_booloader_mode_sync();
        if ( ok == 1 ) { swSer.println(F("got boot-loader sync(ChipID)"));  result= 1; break; }
        if ( ok == 2 ) { swSer.println(F("got boot-loader sync(UPLOAD)"));  result= 2; break;}

        // also try to communicate with the bootloader, if possible....
        swSer.println("Serial.begin(74880);");
        Serial.begin(74880);
        ok = r900x_booloader_mode_sync();
        if ( ok == 1 ) { swSer.println(F("got boot-loader sync(ChipID) at 74880"));  result= 3; break; }
        if ( ok == 2 ) { swSer.println(F("got boot-loader sync(UPLOAD) at 74880"));  result= 4; break;}   

        if ( got_hex == false ) { // hack buzz temp disable, r-enable me.
            swSer.println(F("Serial.begin(57600);"));
            Serial.begin(57600);
            // if that doesn't work, try to communicate with the radio firmware, and put it into AT mode...
            ok = r900x_command_mode_sync();
            if ( ok) { swSer.println(F("got command-mode sync"));  result= 9; break; }
            if ( got_hex == true ) { result = 3; break;  } // grappy way to throwing hands in air and trying bootloader
        }

    } 

           
    // result= -1 might mean our modem was maybe  already stuck in bootloader mode to start with.., we can try to recover that too... 

    swSer.print("result:"); swSer.println(result);
    
        
    if ( true ) { 
        
        if (( result == 1 ) or ( result == 2 ) ){Serial.begin(57600); swSer.print(F("r900x_upload attempt.... AT 57600\n--#")); }
        if (( result == 3 ) or ( result == 4 ) ) { Serial.begin(74880); swSer.print(F("r900x_upload attempt.... AT 74880\n--#")); }
        if (( result == 5 ) or ( result == 6 ) ) { Serial.begin(115200); swSer.print(F("r900x_upload attempt.... AT 115200\n--#")); }
        if (( result == 7 ) or ( result == 8 ) ) { Serial.begin(19200); swSer.print(F("r900x_upload attempt.... AT 19200\n--#")); }

        delay(200);

        //swSer.print("r900x_upload anyway attempt....\n--#");
        while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.
        swSer.println("#--");


        // if we already saw ChipID or UPLOAD output, don't try to do 
        if ( result == 9 ) {
            Serial.write("U"); // 57600 AUTO BAUD RATE CODE EXPECTS THIS As the first byte sent to the bootloader, not even \r or \n should be sent earlier
            Serial.flush();

            bool ok = SmartSerial->expect("ChipID:",2000);  // response to 'U' is the long string including chipid
            // todo handle this return value. 
            //for now just to stop compilter warning:
            ok = !ok;

            delay(200);

            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.

        }

        // UPLOAD COMMAND EXPECTS 'Ready' string
        swSer.println(F("\t\tsending 'UPLOAD'\r\n"));
        //
        Serial.write("UPLOAD\r");
        Serial.flush(); // output buffer flush

        bool ok = SmartSerial->expect("Ready",3000);  // we really MUST see Ready here

        ok = SmartSerial->expect("\r\n",1000);  // and then some \r\n precisely.

        int xok = -1; // xmodem programming return status later on decides if we really, finally, succeeded in flashing
        if ( ok ) { 
            swSer.println(F("\t\tGOT UPLOAD/Ready Response from radio.\n"));
            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.   

            //  xmodem stuff...
            swSer.println(F("bootloader handshake done"));

            if ( reflash ) { 
            swSer.println(F("\t\tXMODEM SENDFILE BEGAN\n"));
            xok = xmodem.sendFile(f, (char *)"unused");
            swSer.println(F("\t\tXMODEM SENDFILE ENDED\n"));

            } else { 
                swSer.println(F("\t\tXSKIPPING XMODEM REFLASH BECAUSE OF 'reflash' false.\n"));

            }

            //f.close();

            swSer.println(F("Booting new firmware"));

            Serial.write("BOOTNEW\r");
            Serial.flush();
            swSer.println(F("\t\tBOOTNEW\r\n"));

            if (reflash && (xok == -1) ) {  // sendfile failed, after a BOOTNEW causes the modm to power cycle, lets retry at 57600
                
                //Serial.begin(57600); or Serial.begin(74880); ? 
                if ( xmodem_retries < 3 ) { 
                    goto retrypoint; // sorry. maybe a corrupted modem .bin file in spiffs?
                }
               swSer.println(F("ERROR! gave up on xmodem-reflash, sorry."));
                                
            } 
            if ( xok == 1 ) { // flash succeeded, really, truly.

               swSer.println(F("renamed firmware file after successful flash ( file ends in .ok now)"));

               // after flashing successfully from the .bin file, rename it
               SPIFFS.remove(BOOTLOADERCOMPLETE); // cleanup incase an old one is still there. 
               SPIFFS.rename(BOOTLOADERNAME, BOOTLOADERCOMPLETE); // after a successful upload to the 900x radio, rename it out of the 
            
            }
                     
        } else { 

            swSer.println(F("\t\tFAILED-TO-GET UPLOAD/Ready Response from radio.\n"));
            //return false;
            //result = 6; 

            Serial.write("BOOTNEW\r");
            Serial.flush();
            delay(200);
            swSer.println(F("\t\tBOOTNEW to powercycle and rety.\r\n"));
            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point.   

            goto retrypoint; // sorry.

        } 

    }


    swSer.println(F("Serial.begin(57600);"));
    Serial.begin(57600); // // get params from modem with command-mode, without talking ot the bootloader, at stock firmware baud rate.
    while (Serial.available() ) { char t = Serial.read();  swSer.print(t); } // flush read buffer upto this point, displaying it for posterity.
    Serial.flush();

   //we should put a AT&F here to factory-reset the modem after the reflash and before we get params from it
    
    if (reflash ) { 
    r900x_getparams("/r900x_params_remote.txt",true);  
    r900x_getparams("/r900x_params.txt",true);  
    } else { 
    r900x_getparams("/r900x_params_remote.txt",false);  
    r900x_getparams("/r900x_params.txt",false);  
    }

    f.close();

} 

#define PROTOCOL_TCP

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer tcpserver(23);
WiFiClient tcpclient;

#define bufferSize 512  
#define packTimeout 1
#define max_tcp_size 500 // never SEND any tcp packet over this size
#define max_serial_size 128 // never SEND any serial chunk over this size

// raw serial<->tcp passthrough buffers, if used.
uint8_t buf1[bufferSize];
uint8_t i1=0;

uint8_t buf2[bufferSize];
uint8_t i2=0;

// variables for tcp-serial passthrough stats
long unsigned int msecs_counter = 0;
long int secs = 0;
long int stats_serial_in = 0;
long int stats_tcp_in = 0;
long int stats_serial_pkts = 0;
long int stats_tcp_pkts = 0;
long int largest_serial_packet = 0;
long int largest_tcp_packet = 0;
#endif
bool tcp_passthrumode = false;

//#define DEBUG_LOG swSer.println



//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
//    bool LEDState;
    delay(1000);
    Parameters.begin();

#ifdef ENABLE_SOFTDEBUG
    // software serial on unrelated pin/s for usb/serial/debug
    swSer.begin(57600);
    swSer.println(F("swSer output for SOFTDEBUG"));
#endif

   Serial.begin(57600); // needed to force baud rate of hardware to right mode for 900x bootloader reflash code.

   SmartSerial->begin();

    swSer.println(F("doing setup()")); swSer.flush();

#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(57600);
    swSer.println(F("Serial1 output for DEBUG"));
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    //pinMode(GPIO02, INPUT_PULLUP);
    //attachInterrupt(GPIO02, count_interrupts, FALLING);

    pinMode(LEDGPIO, OUTPUT); 
    LEDState = 1; 
    digitalWrite(LEDGPIO,LEDState); 
    //-- Initialized RESETGPIO (Used for "Reset To Factory") 
    pinMode(RESETGPIO, INPUT_PULLUP); 
    attachInterrupt(RESETGPIO, count_interrupts, FALLING); 

#endif

    Logger.begin(2048);
    SoftLogger.begin(2048);

    // make sure programmed with correct spiffs settings.
    String realSize = String(ESP.getFlashChipRealSize());
    String ideSize = String(ESP.getFlashChipSize());
    bool flashCorrectlyConfigured = realSize.equals(ideSize);
    if(!flashCorrectlyConfigured)  swSer.println("flash incorrectly configured,  cannot start, IDE size: " + ideSize + ", real size: " + realSize);




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
            LEDState = !LEDState;
            digitalWrite(LEDGPIO,LEDState);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            LEDState = 0;
            digitalWrite(LEDGPIO,LEDState);
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            LEDState = 1;
            digitalWrite(LEDGPIO,LEDState);
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


    #ifdef PROTOCOL_TCP
    swSer.println(F("Starting TCP Server on port 23"));
    tcpserver.begin(); // start TCP server 
    #endif

    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());

    Parameters.setLocalIPAddress(localIP);

  //Setup Websocket debug logger on http port 81.
  //webSocket.begin();
  //webSocket.onEvent(webSocketEvent);

    //-- Initialize Update Server
    updateServer.begin(&updateStatus); 

    //try at current/stock baud rate, 57600, first.
    r900x_setup(true); // probe for 900x and if a new firware update is needed , do it.



swSer.println(F("setup() complete"));
}


void client_check() { 

    uint8 x = wifi_softap_get_station_num();
    if ( client_count != x ) { 
        Serial.println("new client/s connected"); 
        client_count = x;
        DEBUG_LOG("Got %d client(s)\n", client_count); 

        if (client_count != 0 ) { 

           //Parameters.setLocalIPAddress(localIP);
            IPAddress gcs_ip(localIP);
            //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
            gcs_ip[3] = 255;
            swSer.println(F("setting UDP client IP!")); 

            // twiddle LEDs here so user sees they are connected
            for ( int l = 0 ; l < 10; l++ ) { 
                LEDState = !LEDState;
                digitalWrite(LEDGPIO,LEDState);
                delay(100);
            }
            
            GCS.begin(&Vehicle, gcs_ip);
            swSer.println(F("GCS.begin finished")); 
            Vehicle.begin(&GCS);
            swSer.println(F("Vehicle.begin finished")); 
    
        } 

    } 
} 

bool tcp_check() { 
#ifdef PROTOCOL_TCP

    if (!tcpclient.connected()) { 
        tcpclient = tcpserver.available();
        if ( tcpclient ) { 
            return true;
        } else { 
            return false;
        }
    } else { 
        return true;
    }
#endif
    return false;  // if compiled without tcp, always return false.
} 


void handle_tcp_and_serial_passthrough() {

    #ifdef PROTOCOL_TCP

      if ( millis() > msecs_counter +1000 ) { 
        msecs_counter = millis(); 
        secs++;

        swSer.println(F("stats:"));
        swSer.print(F("serial in:")); swSer.println(stats_serial_in);
        swSer.print(F("tcp in   :")); swSer.println(stats_tcp_in);
        swSer.print(F("ser pkts :")); swSer.println(stats_serial_pkts);
        swSer.print(F("tcp pkts :")); swSer.println(stats_tcp_pkts);    

        swSer.print(F("largest serial pkt:")); swSer.println(largest_serial_packet);    
        swSer.print(F("largest tcp pkt:")); swSer.println(largest_tcp_packet);    


        swSer.println(F("----------------------------------------------"));
        stats_serial_in=0;
        stats_tcp_in=0;
        stats_serial_pkts=0;
        stats_tcp_pkts=0;
      }

      if(tcpclient.available()) {
        while(tcpclient.available()) {
          buf1[i1] = (uint8_t)tcpclient.read(); // read char from client 
          stats_tcp_in++;
          if(i1<bufferSize-1) i1++;
          if ( i1 >= max_tcp_size ) { // don't exceed max tcp size, even if incoming data is continuous.
            Serial.write(buf1, i1); stats_serial_pkts++; if ( i1 > largest_tcp_packet ) largest_tcp_packet = i1;
            i1 = 0;
            swSer.println(F("max tcp break"));
            break;
          }
        }
        // now send to UART:
        Serial.write(buf1, i1);  stats_serial_pkts++; if ( i1 > largest_tcp_packet ) largest_tcp_packet = i1;
        i1 = 0;
      }

      if(Serial.available()) {

        // read the data until pause or max size reached
        
        while(1) {
          if(Serial.available()) {
            buf2[i2] = (char)Serial.read(); // read char from UART
            stats_serial_in++;
            if(i2<bufferSize-1) i2++;
            if ( i2 >= max_tcp_size ) { // don't exceed max serial size, even if incoming data is continuous.
                tcpclient.write((char*)buf2, i2); stats_tcp_pkts++; if ( i2 > largest_serial_packet ) largest_serial_packet = i2;
                i2 = 0;
                swSer.println(F("max serial break"));
                break;
            }
          } else {
            //delayMicroseconds(packTimeoutMicros);
            delay(packTimeout);
            if(!Serial.available()) {
              break;
            }
          }
        }
        // now send to WiFi:
        tcpclient.write((char*)buf2, i2);  stats_tcp_pkts++; if ( i2 > largest_serial_packet ) largest_serial_packet = i2;
        i2 = 0;
      }
    #endif

}
//---------------------------------------------------------------------------------
//-- Main Loop

void loop() {

    client_check(); 

    if (tcp_check() ) {  // if a client connects to the tcp server, stop doing everything else and handle that
        if ( tcp_passthrumode == false ) {tcp_passthrumode = true;
        swSer.println(F("entered tcp-serial passthrough mode")); }
        handle_tcp_and_serial_passthrough();

    } else { // do udp & mavlink comms by default and when no tcp clients are available

        if ( tcp_passthrumode == true ) { tcp_passthrumode = false; swSer.println(F("exited tcp-serial passthrough mode")); }

        if(!updateStatus.isUpdating()) {
            if (Component.inRawMode()) {
                GCS.readMessageRaw();
                delay(0);
                Vehicle.readMessageRaw();

            } else {
                GCS.readMessage();
                delay(0);
                Vehicle.readMessage();
                LEDState = !LEDState;
                digitalWrite(LEDGPIO,LEDState);
            }
        }

    }
    updateServer.checkUpdates();
}
