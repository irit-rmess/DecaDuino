// DecaDuinoReceiver
// This sketch shows how to use the DecaDuino library to receive messages over the UWB radio.
// The sketch prints the received bytes in HEX; it can be used as a frame sniffer.
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
DecaDuino decaduino;
int rxFrames;


void setup()
{
  pinMode(13, OUTPUT); // Internal LED (pin 13 on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(13, HIGH); delay(50); digitalWrite(13, LOW); delay(50); }
  }
  
  // Set RX buffer and enable RX
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  rxFrames = 0;
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
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(13, LOW);
  }
}


