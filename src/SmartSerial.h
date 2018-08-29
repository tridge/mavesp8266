#pragma once

#include <SoftwareSerial.h> 
extern SoftwareSerial swSer;

class MySerial { 
public:
  HardwareSerial * _Serial = NULL;
  bool _serial_connected = false;

  uint8_t rawserialbytes[255];

  unsigned long last_ms; 

  MySerial(HardwareSerial *serial) {
   _Serial = serial;
  }
  void begin() { 
      //Serial->begin(4800, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
      // delay(1000); // time to start up.

      // put_into_at_mode();

      swSer.println("SmartSerial begin() done.");
        
      _serial_connected = true;
  }

 // similar to expect() except returns the complete raw string-data parsesd/found on success, and empty string on fail.
  String expect_s( String lookfor, uint32_t wait_ms ) { 

    String return_val = "";
    last_ms = millis();
    //bool found = false;
    // look for response, for max X milli seconds
    //swSer.println("waiting for string:"+ lookfor);
    unsigned int offset = 0;
    bool finding = false; // are we part-way through a string match? 

   // swSer.print("-->");
    while ((last_ms+wait_ms) > millis() )     {
        if ( Serial.available() ) {
            char c = Serial.read();
            //swSer.print(c); // debug only
            return_val += c;
            if ( c == lookfor.charAt(offset) ) {  // next char we got is the next char we expected, so move forward a char and keep looking.
                offset++;
                finding = true;
            } else {  // we got a char that isn't the next one we wanted.
                finding = false;
                offset = 0;
            }
            if (( finding==true ) && ( offset >= lookfor.length() )) { // we found the entire string, return immediately, dont wait for any more serial data or timeout
                //swSer.println("<--");
                //swSer.print("\nfound string:"+ lookfor); 
                //swSer.print(" with length:"); swSer.println(offset);
                return return_val;
            } 
        }
    }
    //swSer.println("<--");
    return return_val;
  }

  bool expect( String lookfor, uint32_t wait_ms ) { 

    last_ms = millis();
    //bool found = false;
    // look for response, for max X milli seconds
    swSer.println("waiting for string:"+ lookfor);
    unsigned int offset = 0;
    bool finding = false; // are we part-way through a string match? 

    swSer.print("-->");
    while ((last_ms+wait_ms) > millis() )     {
        if ( Serial.available() ) {
            char c = Serial.read();
            swSer.print(c); // debug only
            if ( c == lookfor.charAt(offset) ) {  // next char we got is the next char we expected, so move forward a char and keep looking.
                offset++;
                finding = true;
            } else {  // we got a char that isn't the next one we wanted.
                finding = false;
                offset = 0;
            }
            if (( finding==true ) && ( offset >= lookfor.length() )) { // we found the entire string, return immediately, dont wait for any more serial data or timeout
                swSer.println("<--");
                swSer.print("\nfound string:"+ lookfor); 
                swSer.print(" with length:"); swSer.println(offset);
                return true;
            } 
        }
    }
    swSer.println("<--");
    return false;
  }
};


