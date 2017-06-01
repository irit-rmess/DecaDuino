// DecaDuinoSender
// This sketch shows how to use the DecaDuino library to send messages over the UWB radio 
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// Licensing: see DecaDuino licence

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t txData[MAX_FRAME_LEN];
uint16_t txLen;
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
}


void loop()
{
  int i;

  // make dummy data, send it and wait the end of the transmission.
  digitalWrite(13, HIGH);
  for (i=0; i<MAX_FRAME_LEN; i++) {
    txData[i] = i;
  }
  decaduino.pdDataRequest(txData, MAX_FRAME_LEN);
  while ( !decaduino.hasTxSucceeded() );
  digitalWrite(13, LOW);
  
  // wait 1 second 
  delay(1000);
}


