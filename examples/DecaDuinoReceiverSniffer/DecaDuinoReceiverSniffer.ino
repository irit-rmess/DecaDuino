// DecaDuinoReceiver
// This sketch shows how to use the DecaDuino library to receive messages over the UWB radio.
// The sketch prints the received bytes in HEX; it can be used as a frame sniffer.
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// Licensing: see DecaDuino licence

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
DecaDuino decaduino;
int rxFrames;


void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(115200);
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
  
  rxFrames = 0;
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
}


void loop()
{  
  // If a message has been received, print it and re-enable receiver
  if ( decaduino.rxFrameAvailable() ) {
    digitalWrite(13, HIGH);
    Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
    Serial.print(rxLen);
    Serial.print("bytes received: |");
    for (int i=0; i<rxLen; i++) {
      Serial.print(rxData[i], HEX);
      Serial.print("|");
    }
    Serial.println();
    decaduino.plmeRxEnableRequest();
    digitalWrite(13, LOW);
  }
}


