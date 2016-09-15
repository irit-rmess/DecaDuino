#include <SPI.h>
#include <DecaDuino.h>
#include <EEPROM.h>

#define ANTENNA_DELAY_VALUE_TO_SET_IN_EEPROM DWM1000_DEFAULT_ANTENNA_DELAY_VALUE

DecaDuino decaduino;

void setup() {

  uint8_t buf[2];
  uint16_t antennaDelay;

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

  // Gets antenna delay from the end of EEPROM. The two last bytes are used for DecaWiNo label, 
  // so use n-2 and n-3 to store the antenna delay (16bit value)
  buf[0] = EEPROM.read(EEPROM.length()-4);
  buf[1] = EEPROM.read(EEPROM.length()-3);
  antennaDelay = decaduino.decodeUint16(buf);

  delay(1000); // 

  if ( antennaDelay == 0xffff ) {
    Serial.println("Unvalid antenna delay value found in EEPROM. Using default value.");
  } else decaduino.setAntennaDelay(antennaDelay);
}

void loop() {

}


