#ifndef TEENSYDUINO  
#error this sketch will only work on teensyduino. However, there is an eeprom WITHIN THE DW1000 that can be used for the same purpose (and which is maybe already populated from the factory)
#ifndef

#include <SPI.h>
#include <DecaDuino.h>
#include <EEPROM.h>

#define ANTENNA_DELAY_VALUE_TO_SET_IN_EEPROM DWM1000_DEFAULT_ANTENNA_DELAY_VALUE

DecaDuino decaduino;

void setup() {

  uint8_t buf[2];

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) {
      digitalWrite(13, HIGH); 
      delay(50);    
      digitalWrite(13, LOW); 
      delay(50);    
    }
  }

  // Stores antenna delay at the end of EEPROM. The two last bytes are used for DecaWiNo label, 
  // so use n-2 and n-3 to store the antenna delay (16bit value)
  decaduino.encodeUint16(ANTENNA_DELAY_VALUE_TO_SET_IN_EEPROM, buf);
  EEPROM.write(EEPROM.length()-4, buf[0]);
  EEPROM.write(EEPROM.length()-3, buf[1]);
}

void loop() {

}


