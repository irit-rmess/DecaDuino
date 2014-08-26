#include <spi4teensy3.h>
#include <DecaDuino.h>

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
#define FRAME_LEN 64


void setup() {

  pinMode(13, OUTPUT);
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
   
    while(1);
  }
  
  //Serial.println(decaduino.getEuid, HEX);
}

void loop() {
  
  int i;

  digitalWrite(13, HIGH);
  for (i=0; i<FRAME_LEN; i++)
    txData[i] = i;
  decaduino.plmeDataRequest(txData, FRAME_LEN);
  delay(1);
  digitalWrite(13, LOW);
  delay(1000);
}


