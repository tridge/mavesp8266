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


  // return negative vals for error, positive vals to say which match was successful
  // this is essentially the same code as expect() function, but can handle looking for upto *three* different strings concurrently.
  int expect_multi3( String lookfor, String lookfor2, String lookfor3, uint32_t wait_ms ) { 

    last_ms = millis();
    //bool found = false;
    // look for response, for max X milli seconds
    swSer.println("waiting for string:"+ lookfor);
    swSer.println("waiting for string2:"+ lookfor2);
    swSer.println("waiting for string3:"+ lookfor3);
    unsigned int offset = 0;
    unsigned int offset2 = 0;
    unsigned int offset3 = 0;
    bool finding = false; // are we part-way through a string match? 
    bool finding2 = false; // are we part-way through a string match? 
    bool finding3 = false; // are we part-way through a string match? 

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

            if ( c == lookfor2.charAt(offset2) ) {  // next char we got is the next char we expected, so move forward a char and keep looking.
                offset2++;
                finding2 = true;
            } else {  // we got a char that isn't the next one we wanted.
                finding2 = false;
                offset2 = 0;
            }

            if ( c == lookfor3.charAt(offset3) ) {  // next char we got is the next char we expected, so move forward a char and keep looking.
                offset3++;
                finding3 = true;
            } else {  // we got a char that isn't the next one we wanted.
                finding3 = false;
                offset3 = 0;
            }

            if (( finding==true ) && ( offset >= lookfor.length() )) { // we found the entire string, return immediately, dont wait for any more serial data or timeout
                swSer.println("<1--");
                swSer.print("\nfound string:"+ lookfor); 
                swSer.print(" with length:"); swSer.println(offset);
                return 1;
            } 

            if (( finding2==true ) && ( offset2 >= lookfor2.length() )) { // we found the entire string, return immediately, dont wait for any more serial data or timeout
                swSer.println("<2--");
                swSer.print("\nfound string2:"+ lookfor2); 
                swSer.print(" with length2:"); swSer.println(offset);
                return 2;
            } 

            if (( finding3==true ) && ( offset3 >= lookfor3.length() )) { // we found the entire string, return immediately, dont wait for any more serial data or timeout
                swSer.println("<3--");
                swSer.print("\nfound string3:"+ lookfor3); 
                swSer.print(" with length3:"); swSer.println(offset);
                return 3;
            } 
        }
    }
    swSer.println("<--");
    return -1;
  }

};


