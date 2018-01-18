// DecaDuinoSender
// This sketch shows how to use the DecaDuino library to send messages over the UWB radio 
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t txData[MAX_FRAME_LEN];
uint16_t txLen;
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
}


void loop()
{
  // make dummy data, send it and wait the end of the transmission.
  digitalWrite(13, HIGH);
  for (int i=0; i<MAX_FRAME_LEN; i++) {
    txData[i] = i;
  }
  decaduino.pdDataRequest(txData, MAX_FRAME_LEN);
  while ( !decaduino.hasTxSucceeded() );
  digitalWrite(13, LOW);
  
  // wait 1 second 
  delay(1000);
}


