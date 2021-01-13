#include <SPI.h>
#include <DecaDuino.h>

int i;
int rxFrames;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
#define FRAME_LEN 64

#define SENDER
// ou
//#define RECEIVER

void setup() {

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    digitalWrite(13, HIGH);  
    while(1);
  }
  
  #ifdef RECEIVER
  rxFrames = 0;
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  #endif
}

void loop() {

  decaduino.engine();
  
  int i;

  #ifdef RECEIVER
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
  #endif

  #ifdef SENDER
  digitalWrite(13, HIGH);
  for (i=0; i<FRAME_LEN; i++)
    txData[i] = i;
  decaduino.pdDataRequest(txData, FRAME_LEN);
  delay(1);
  digitalWrite(13, LOW);
  delay(1000);
  #endif
  
  // End of the cycle
}


