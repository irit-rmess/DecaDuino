#include <SPI.h>
#include <DecaDuino.h>

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
#define FRAME_LEN 64
int rxFrames;


void setup() {

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    digitalWrite(13, HIGH);   
    while(1);
  }
  
  rxFrames = 0;
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
}

void loop() {
  
  int i;

  if ( decaduino.rxFrameAvailable() ) {
    digitalWrite(13, HIGH);
    Serial.print("["); Serial.print(++rxFrames); Serial.print("] ");
    Serial.print(rxLen);
    Serial.print("bytes received: ");
    for (i=0; i<rxLen; i++) {
      Serial.print(rxData[i], HEX);
      Serial.print("|");
    }
    Serial.println();
    decaduino.plmeRxEnableRequest();
    digitalWrite(13, LOW);
  }
}


