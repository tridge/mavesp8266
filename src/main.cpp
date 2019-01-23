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

//-------------------------------------------
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

//-------------------------------------------

// this improved version is used consistently across the code now, as it has a collection of optimisations
// that include a quick ATI check first ( with very short timeout ) before doing a long +++ timeout if needed.
// and it toggles LEDS, and it also terminates the +++ command afterwards, with \r\n as needed.
bool enter_command_mode() { 

    // to make it potentially faster for the majority of cases where the radio is *still* in 
    // command-mode, we could quickly try an 'ATI' and if we get an echo back along with the 
    // version info, were already in command mode and don't have to wait a whone second to 
    // find out.
    //ATI
    //RFD SiK 2.65 on RFD900X R1.3
    Serial.write("\r\nATI\r\n");
    Serial.flush(); // output buffer flush
    delay(100);

    int already_commmand_mode_test = SmartSerial->expect_multi3("ATI","RFD SiK","RFD900X",250); // look for any of these

    while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

    if (  already_commmand_mode_test <= 0 ) {  // no match on the above 3 strings means ...

        // put it into command mode first....
        Serial.write("\r");
        Serial.flush();
        delay(1010); 
        Serial.write("+++");
        Serial.flush(); // output buffer flush

        // toggle LED to be more interesting.
        LEDState = !LEDState;
        digitalWrite(LEDGPIO,LEDState);

        int ok2 = SmartSerial->expect_multi3("OK","+++","XXXXX",1100); // look for +++ takes 50ms, OK might take 1000 ?
        if ( ok2 == 1 ) { 
            swSer.println(F("GOT OK Response from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
            return true;
        }
        else if ( ok2 == 2 ) { 
            // if modem echo's back the +++ we sent it, we are already in command mode.
            swSer.println(F("radio is already_in_cmd_mode2, continuing... sending RN.\n"));
            Serial.write("\r\n"); // terminate the +++ command.
            Serial.flush(); // output buffer flush
            //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
            //goto already_in_cmd_mode2;
            return true;
        } else { 
            swSer.println(F("FALED-TO-GET 'OK' or '+++' Response from radio."));
            swSer.println(F("RETURN:no cmd-mode modem coms")); 
            return false;  
        } 

    }

    // if already_commmand_mode_test > 0 
    return true;
    

}

//-------------------------------------------
// warning execution time on this can be as much as ~4 seconds if the device is repeatly refusing to enter command mode with +++
bool enter_command_mode_with_retries() { 

    bool result = enter_command_mode(); // try 1

    swSer.println(result?"true":"false");

    if ( result ) return true;

    int chances = 0;

    while (( ! result) && (chances < 3 )) {  // chances 0,1,2

        result = enter_command_mode(); // try 2,3,4

        swSer.println(result?"re-true":"re-false");

        if ( result ) return true;

        chances++;
    }
    return false;
}
//-------------------------------------------
// returns -1 on major error, positive number of params fetched on 'success'
int r900x_getparams(String filename, bool factory_reset_first) { 

    swSer.println(F("r900x_getparams()- START\n"));


    swSer.println(F("b4\n"));
    if ( ! enter_command_mode_with_retries() ) { 
    swSer.println(F("failed\n"));
    return -1; 
}
    swSer.println(F("after\n"));

    //TODO - do we need these timeouts? 

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
        swSer.print(F("attempting factory reset.... &F and &W ... \n"));

        String factorycmd = prefix+"&F\r\n";
        Ser9x.write(factorycmd.c_str());
        Ser9x.flush(); // output buffer flush
        bool b = SmartSerial->expect("OK",100); 
        while (Ser9x.available() ) { Ser9x.read();  } // flush read buffer upto this point and discard

        String factorycmd2 = prefix+"&W\r\n"; 
        Ser9x.write(factorycmd2.c_str());
        Ser9x.flush(); // output buffer flush
        bool b2 = SmartSerial->expect("OK",100); 
        while (Ser9x.available() ) { Ser9x.read();  } // flush read buffer upto this point and discard

        swSer.print(F("...attempted factory reset.\n"));
    }

    if (filename == "/r900x_params.txt" ) { 

        String led_on_cmd2 = "ATS19=1\r\n"; 
        Ser9x.write(led_on_cmd2.c_str());
        Ser9x.flush(); // output buffer flush
        bool b3 = SmartSerial->expect("OK",100);
        while (Ser9x.available() ) { Ser9x.read();  } // flush read buffer upto this point and discard
        
    }

    // TODO get version info here from local and/or remote modem with an AT command

    // now get params list ATI5 or RTI5 as needed 
    String cmd = prefix+"I5\r";
    Serial.write(cmd.c_str());
    Serial.flush(); // output buffer flush
    swSer.print(F("----------------------------------------------"));
    String data = SmartSerial->expect_s("SIS_RSSI=50\r\n",500); 
    while (Serial.available() ) { char t = Serial.read();  data += t; } // flush read buffer upto this point.
    swSer.print(data);
    swSer.print(F("----------------------------------------------"));

    // count number of lines in output, and return it as result.
    unsigned int linecount = 0;
    for ( unsigned int c = 0 ; c < data.length(); c++ ) { 
        if ( data.charAt(c) == '\n' ) { linecount++; }
    }

    // as user experience uses the SPIFFS .txt to render the html page, we cleanup an old one if we've been asked
    // to get fresh params, even if we cant replace it, as the *absense* of it means the remote radio is 
	// no longer connected.
    SPIFFS.remove(filename); 

    
    // now write params to spiffs, for user record:
    if ( data.length() > 300 ) { // typical file length is around 400chars 
        f = SPIFFS.open(filename, "w");
        f.print(data);  //actual parm data.

        // tack encryption key onto the end of the param file, if it exists, instead of read from remote radio
		// as we can't do that right now. - TODO.
        File e = SPIFFS.open("/key.txt", "r");
        String estr = "&E:ENCRYPTION_KEY="+e.readString();// entire file, includes /r/n on end.
        
        if (estr.length() > 30 && e) { // basic check, file should exist and have at least 30 bytes in it to be plausible
          f.print(estr);
        }
        e.close(); 

        f.close();
    } else { 
        swSer.println(F("didn't write param file, as it contained insufficient data"));
    }


	bool got_vers = false; // assume this to start with 
	if ( 1 ) { 

		String vercmd = prefix+"I\r\n";  //ATI or RTI
		swSer.print(vercmd.c_str()); // for debug only
		Serial.write(vercmd.c_str());
		Serial.flush(); // output buffer flush

		String vers = "RFD SiK"; //starts with this...
		bool ok = SmartSerial->expect_s(vers,1500);  // we really want to see 'RFD SiK' here, 

		if ( ok ) { 
		    swSer.println(F("\tGOT SiK Response from radio.\n"));

			// this line *may* have started with 'RFD SiK' ( we matched on the SiK above), and end with '2.65 on RFD900X R1.3' 
			while (Serial.available() ) { char t = Serial.read();  swSer.print(t); vers += t; } // flush read buffer upto this point, displaying it for posterity ( it has version info )

			if (vers.indexOf(" on ") > 10  ) { //"RFD SiK Q.QQ on RFDXXXX RZ.Z"
				swSer.print(F("VERSION STRING FOUND:"));
				swSer.println(vers);

				// save version string to a file for later use by the webserver to present to the user.
				String vf = "/r900x_version.txt";
				if (filename == "/r900x_params_remote.txt" ) {vf = "/r900x_version_remote.txt";}
				File v = SPIFFS.open(vf, "w"); 
				v.print(vers);
				v.close();
				got_vers = true;
			}

		} else {
		    swSer.println(F("\tFAILED-TO-GET SiK Response from radio.\n"));
		} 
	}
	
    if ( factory_reset_first ) { 

        delay(1000); // time for local and remote to sync and RT values to populate.

        swSer.println(F("ATZ/RTZ rebooting 900x due to factory_reset_first "));
        cmd = prefix+"Z\r";
        Serial.write(cmd.c_str()); // reboot radio to restore non-command mode.
        Serial.flush(); // output buffer flush
        delay(200); 
    }

    swSer.println(F("r900x_getparams()- END\n"));

     return linecount;
} 

// prerequisite, u should have called some form of enter_command_mode() before this one:
int r900x_readsingle_param_impl( String prefix, String ParamID ) { 
    swSer.println(F("r900x_readsingle_param_impl"));

    //  read individial param first  ( eg ATS4? ) to see if it even needs changing
    String cmd = prefix+ParamID+"?\r\n"; // first \r\n is to end the +++ command we did above, if any.


    // shortcurcuit for &E etc
    if ( ParamID == "&E" ) return -4; // not readable, for now. 
    if ( ParamID == "&R" ) return -4; // not readable, for now. 
    if ( ParamID == "&F" ) return -4; // not readable, for now. 

    swSer.println(cmd); // debug only

    // here we neter a "retries" loop for approx 100ms each iteration and maxloops 5
    // because it's proven in testing that the RTS3?   and similar style commands don't 
    // work first-go , perhaps 2nd go? 
    // also, this doesn't work if the remote radio isn't there and it's a RT command
    String data = "";
    String data3 = "";
    int max = 0;
    // TODO does not handle RT&E? params. as it returns a long string with 32 hex chars.
    while (( data.length() < 10 ) && (max < 5 ) && ( data3.length() <= 2 ) )  { 
        Serial.write(cmd.c_str());
        Serial.flush(); // output buffer flush
        data = SmartSerial->expect_s(cmd,55);  // this should capture the echo of the RTS3?
        data3 = SmartSerial->expect_s("\r\n",80);  // this should capture the line after and its contents
        swSer.println(F("^^^^^^^^^^^^^^^^^^^"));
        swSer.println(max);
        max++;
    }

    if (data3.length() <= 2 ) { swSer.println(F("RETURN:existing val cant be read")); return -4;  } 
    // if we didn't get a number and \r and \n, then give up.    

    // TODO, what if we hit retry limit ? 
    //if ( max == 5 ) {  ... } 

    //here  - decide if param valus is already set right, and if so, return OK status as it's verified! 
    // data3 contains the stored value for the ParamID in the prefix-type radio., with newlines and as a string.
    int value = data3.toInt(); 

    swSer.println(data);
    swSer.println(value);
   
    return value; 
}

//-------------------------------------------
// no prerequisite version of the above function. with more retries. 
int r900x_readsingle_param(String prefix, String ParamID) { 

    swSer.println(F("r900x_readsingle_param"));

    if ( ! enter_command_mode_with_retries() ) { return -1 ; } 

    swSer.println(F("Z-------flush---------------"));
    while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

    // might return negative number on error.... so add one ghetto retry here as it already has some retrues...
    int retval =  r900x_readsingle_param_impl( prefix, ParamID );

    if ( retval >= 0 ) return retval;

    if ( retval < 0 ) { 
    
        delay(500);

        // erp, should not be needed, but aparently can be...
        String exitcmd= "ATO\r\n";
        swSer.println(exitcmd); // debug only.
        Serial.write(exitcmd.c_str());
        Serial.flush(); // output buffer flush

        swSer.println(F("r900x_readsingle_param-RETRY"));

        if ( ! enter_command_mode_with_retries() ) { return retval ; } 

        swSer.println(F("Q-------flush---------------"));
        //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

        // last try, might return negative number on error, just return it.. 
        return r900x_readsingle_param_impl( prefix, ParamID );
    }

}

//-------------------------------------------
// returns negative numbers on error, and positive number on success
//        String prefix = "AT";  or RT
//        String ParamID = line.substring(0,colon_offset); // S4  or E?
//        String ParamNAME = line.substring(colon_offset+1,equals_offset); // TXPOWER
//        String ParamVAL = line.substring(equals_offset+1,eol_offset);  // 30

int r900x_savesingle_param_and_verify_more(String prefix, String ParamID, String ParamVAL, bool save_and_reboot) { 

    swSer.println(F("r900x_savesingle_param_and_verify"));

    // sometimes we skip parts of these see below
    bool actually_write = true;


    if ( ! enter_command_mode_with_retries() ) { return -1 ; } 

    swSer.println(F("X-------flush---------------"));
    while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

    // might return negative number on error.... ( such as &E being empty, or &R not being standard )
    int value = r900x_readsingle_param_impl( prefix, ParamID );
    
    // success, it's already set at that target val  
    // this  honors the save_and_reboot parameter by jumping to the save-and-reboot section, which it should.  
    if (value == ParamVAL.toInt() ) { 
        if ( save_and_reboot ) {
            swSer.println(F("val already set, jumping to save-and-reboot anyway"));
            actually_write = false;   //continues below
        } else {
            swSer.println(F("RETURN:val already set, no save/reboot requested"));
            return 1;   // shortcurcuit and return immediately
        }
    } 

    // unreadable error... for now we try to write it anyway.
    //if ( value < 0 ) {     }

    if (  actually_write ) { 
        int param_write_counter = 0;
        param_write_retries:

        // set param to new vaue
        String ParamCMD= prefix+ParamID+"="+ParamVAL+"\r\n";

            // overwride for special case/s ( very rare ) 
            if (( prefix == "RT" ) && ( ParamID == "&R" )) ParamCMD = prefix+ParamID+"\r\n";
            if ( ParamID == "&F" ) ParamCMD = prefix+ParamID+"\r\n";

        swSer.println(ParamCMD); // debug only.
        Serial.write(ParamCMD.c_str());
        Serial.flush(); // output buffer flush
        bool ok = SmartSerial->expect("OK",500);  // needs to be bigger than 200.
        swSer.println(F("7#############################################################"));
        if ( ok ) { 
            swSer.println(F("GOT OK from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 

            // HACK. we don't get an OK reply from RT&E=XXXXXXXXXXXXX, just an echo 
            //of the command and a blat of suddenly unreadable bytes and the 250ms timeout.
            //if (( ParamID == "&E" ) && ( param_write_counter > 8 ) ) { 
            //   return 3; // positve number = success.
            //}

            swSer.println(F("FALED-TO-GET OK Response from radio. - retrying:\n"));
            param_write_counter++;
            if ( param_write_counter < 5 ) { 
                goto param_write_retries;
            }

            // if 5 retries exceeded, return error.
            swSer.println(F("RETURN:no response to param set request - 5 retries exceeded")); 
            return -2; // neg number/s mean error
        } 


        if ( ! save_and_reboot ) {
            swSer.println(F("Skipping save and reboot\nRETURN:PERFECTO"));
            return 3;
        } 
    }

    if ( save_and_reboot )  { 
        int param_save_counter = 0;
        param_save_retries:

        // save params AND take out of command mode via a quick reboot
        String savecmd = prefix+"&W\r\n"; // "AT&W\r\n
        swSer.println(savecmd); // debug only.
        Serial.write(savecmd.c_str());
        Serial.flush(); // output buffer flush

        // LEDState = 0;
        // digitalWrite(LEDGPIO,LEDState);
        bool ok = SmartSerial->expect("OK",500); 
        if ( ok ) { 
            swSer.println(F("GOT OK Response from radio.\n"));
            //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

            // save params AND take out of command mode via a quick reboot
            String rebootcmd = prefix+"Z\r\n"; // "ATZ\r\n"
            swSer.println(rebootcmd); // debug only.
            Serial.write(rebootcmd.c_str());
            Serial.flush(); // output buffer flush

            delay(150);
            // TODO ? ok = SmartSerial->expect("RTZ",200);
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point. - ie 'RTZ' response

        } else { 

            swSer.println(F("FALED-TO-GET OK Response from radio. - retrying:\n"));
            delay(100); // might help by delaying the retry a bit.
            param_save_counter++;
            if ( param_save_counter < 5 ) { 
                goto param_save_retries;
            }

            // if 5 retries exceeded, return error.
            swSer.println(F("RETURN:no response to param save request - 5 retries exceeded")); 
            return -3;
        } 
    }

    swSer.println(F("RETURN:PERFECT"));
    return 2;
}
//-------------------------------------------
int r900x_savesingle_param_and_verify(String prefix, String ParamID, String ParamVAL) { 
    return r900x_savesingle_param_and_verify_more( prefix,  ParamID,  ParamVAL, true);
}

//-------------------------------------------

bool r900x_saveparams(String filename) { 
swSer.println(F("r900x_saveparams()\n"));
// iterate over the params found in r900x_params.txt and save them to the modem as best as we can.


    if ( ! enter_command_mode_with_retries() ) { return -1; } 
 
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

        // if its an ATS2 or RTS0 command, skipp it, as we don't allows writes to S0
        if ( ParamID == "S0" ) { 
            swSer.println(F("skipping S0, as we don't write it."));
            continue; 
        }

        // basic retry counter for 
        int param_write_counter2 = 0;
        param_write_retries2:
        
        String ParamCMD= prefix+ParamID+"="+ParamVAL+"\r\n";

        // we implement retries on ths for if we didn't get a response to the command in-time..,
        // and becasue AT and RT commands are notoriously non-guaranteed.

        swSer.println(ParamCMD); // debug only.
        Serial.write(ParamCMD.c_str());
        Serial.flush(); // output buffer flush
        bool ok = SmartSerial->expect("OK",200); // typically we get a remote response in under 150ms, local response under 30
        if ( ok ) { 
            swSer.println(F("GOT OK from radio.\n"));
            while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.
        } else { 

            swSer.println(F("FALED-TO-GET OK Response from radio. - retrying:\n"));
            param_write_counter2++;
            if ( param_write_counter2 < 5 ) { 
                goto param_write_retries2;
            }
            // if ~5 retries exceeded, return error.
            swSer.println(F("RETURN:no response to param set request - 5 retries exceeded")); 
            return -2; // neg number/s mean error

        } 
        
        done++;
    }   

    
    // jump here when done writing parms:
   save:

   // save params AND maybe take out of command mode via a quick reboot
    String savecmd = prefix+"&W\r\n"; // "AT&W\r\n
    swSer.println(savecmd); // debug only.
    Serial.write(savecmd.c_str());
    Serial.flush(); // output buffer flush

    LEDState = 0;
    digitalWrite(LEDGPIO,LEDState);

    bool ok = SmartSerial->expect("OK",2000); 
    if ( ok ) { 
        swSer.println(F("GOT OK Response from radio.\n"));
        //while (Serial.available() ) { Serial.read(); } // flush read buffer upto this point.

          // take out of command mode via a quick reboot
            String rebootcmd = prefix+"Z\r\n"; // "ATZ\r\n"
            swSer.println(rebootcmd); // debug only.
            Serial.write(rebootcmd.c_str());
            Serial.flush(); // output buffer flush

            LEDState = 0;
            digitalWrite(LEDGPIO,LEDState);

            delay(1000); // hack to allow the radio to come back and sync after a ATZ or RTZ
 
            //ok = SmartSerial->expect("OK",2000);  we don't expect a response from ATZ or RTZ
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
    
        // try to put radio into command mode with +++, but if no response, carry on anyway, as it's probably in bootloader mode already.
        enter_command_mode() ; // don't giveup if this returns false

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

/*            // this line *may* have started with 'RFD SiK' ( we matched on the SiK above), and end with '2.65 on RFD900X R1.3' 
            String vers = "RFD SiK";
            while (Serial.available() ) { char t = Serial.read();  swSer.print(t); vers += t; } // flush read buffer upto this point, displaying it for posterity ( it has version info )

            if (vers.indexOf(" on ") > 10  ) { //"RFD SiK Q.QQ on RFDXXXX RZ.Z"
                swSer.print(F("VERSION STRING FOUND:"));
                swSer.println(vers);

                // save version string to a file for later use by the webserver to present to the user.
                File v = SPIFFS.open("/r900x_version.txt", "w"); 
                v.print(vers);
                v.close();
            }
*/


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


void r900x_setup(bool reflash) { // if true. it will attempt to reflash ( and factory defaults), if false it will go through the motions.

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

        //TODO - can we do Hardware flow control? 
        // https://www.esp8266.com/viewtopic.php?f=27&t=8560
        // maybe not with the arduino IDE, but maybe yes with the espressif SDK...? 

        // possible impl:
        // const int CTSPin = 13; // GPIO13 for CTS input
        // pinMode(CTSPin, FUNCTION_4); // make pin U0CTS
        // U0C0 |= UCTXHFE; //add this sentense to add a tx flow control via MTCK( CTS )

        // other notes:
        // SET_PERI_REG_MASK( UART_CONF0(uart_no),UART_TX_FLOW_EN); //add this sentense to add a tx flow control via MTCK( CTS )
        // or U0C0 |= UCTXHFE; //add this sentense to add a tx flow control via MTCK( CTS )
        // https://github.com/esp8266/Arduino/blob/master/cores/esp8266/esp8266_peri.h#L189
        //
        //As for the GPIOs, when flow control is enabled:
        //GPIO13	= U0CTS
        //GPIO15	= U0RTS
        // this last link uses the RTOS SDK, not the arduino one, so isn't 100% accurate...
        // http://forgetfullbrain.blogspot.com/2015/08/uart-sending-and-receiving-data-using.html



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

    //we put a AT&F here to factory-reset the modem after the reflash and before we get params from it
    
    if (reflash ) { 
    r900x_getparams("/r900x_params_remote.txt",true);  // true = reset to factory defaults before reading params
    r900x_getparams("/r900x_params.txt",true);         // true = reset to factory defaults before reading params
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


String mac2String(byte ar[]){
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);
    s += buf;
    if (i < 5) s += '-'; // traditionally a :, but we want a - in this case
  }
  return s;
}

String half_mac2String(byte ar[]){
  String s;
  for (byte i = 3; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);
    s += buf;
    if (i < 5) s += '-'; // traditionally a :, but we want a - in this case
  }
  return s;
}

// globals
String mac_s;
String mac_ap_s;
String realSize;

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
    realSize = String(ESP.getFlashChipRealSize());
    String ideSize = String(ESP.getFlashChipSize());
    bool flashCorrectlyConfigured = realSize.equals(ideSize);
    if(!flashCorrectlyConfigured)  swSer.println("ERROR!!! flash incorrectly configured,  cannot start.");
    swSer.println("Flash IDE size: " + ideSize + ", real size: " + realSize);




    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);


    // get MAC address of adaptor as used in STA mode
    byte mac[6]; 
    WiFi.macAddress(mac);
    // as a string as well as its easier.
    String mac_half_s = half_mac2String(mac);
    mac_s = mac2String(mac); // set this as a global, as we use it 'extern' in  http server to show to the user.

    // get MAC address of adaptor as used in AP mode
    byte mac_ap[6]; 
    WiFi.softAPmacAddress(mac_ap);
    mac_ap_s = mac2String(mac_ap); // set this as a global, as we use it 'extern' in  http server to show to the user.

    //-- MDNS
    char mdnsName[256];
    sprintf(mdnsName, "TXMOD-%s",mac_half_s.c_str());
    //sprintf(mdsnName, "TXMOD123");

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        DEBUG_LOG("\nEntering station mode...\n");
        //-- Connect to an existing network
        WiFi.mode(WIFI_STA);
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());

        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            #ifdef ENABLE_DEBUG
            //Serial.print(".");
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

        // look at the out-of-box SSID which is compiled-in as 'TXMOD' and revise it to TXMOD-xx-xx-xx-xx-xx-xx if needed.
        char *_tmp_wifi_ssid;
        _tmp_wifi_ssid = Parameters.getWifiSsid();
        int compare= strcmp(_tmp_wifi_ssid, "TXMOD");
        if ( compare == 0 ) {  // equal zero means it's still set as compiled-in 'TXMOD'
          Parameters.setWifiSsid(mdnsName);
        }

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




    int retries = 5;
    while ( retries > 0) { 
       bool success = MDNS.begin(mdnsName);
       if ( success ) { retries = 0; } 
       else { 
        swSer.println("Error setting up MDNS responder!");
        delay(1000);
        retries --;
       }
    }
    MDNS.addService("_http", "_tcp", 80);    
    //MDNS.addService("tcp", "tcp", 23);


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

    // TODO , the r900x_setup() can take some time to run, is is possible that we could get the webserver responding to requests during this period?

    //try at current/stock baud rate, 57600, first.
    r900x_setup(true); // probe for 900x and if a new firware update is needed , do it.  CAUTION may hang in retries if 900x modem is NOT attached


swSer.println(F("setup() complete"));
}


void client_check() { 

    
    //swSer.print("."); //debug only

    uint8 x = wifi_softap_get_station_num();
    if ( client_count != x ) { 
        swSer.println("new client/s connected"); 
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

// periodic client check..
int period = 200;
unsigned long time_next = 0;
 
void loop() {

    if(millis() > time_next){
        time_next = millis()+period;
        client_check();
    }


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
    updateServer.checkUpdates(); // aka webserver.handleClient()


  MDNS.update();
}
