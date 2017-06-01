// DecaDuinoChat
// This sketch shows how to use the DecaDuino library to send and receive ascii messages via the Serial port over the UWB radio
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// Licensing: see DecaDuino licence

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t txData[MAX_FRAME_LEN];
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
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

  rxFrames = 0;
  txLen = 0;
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  Serial.println("DecaDuino chat ready!");
}


void loop()
{
  int i;
  int let_s_send = false;

  // This is the sender part
  // Get chars on Serial and prepare buffer
  while ( ( Serial.available() > 0 ) && ( txLen < MAX_FRAME_LEN ) ) {
    char c = Serial.read();
    txData[txLen++] = c;
    if ( (c=='\n')||(c=='\r')||(txLen==MAX_FRAME_LEN) ) {
      // Send if NL or CR char is  detected, or if buffer is full
      let_s_send = true;
    }
  }
  // Send the buffered chars
  if ( let_s_send == true ) {
    // LED ON, disable RX, send, wait sent complete, re-enable RX and LED OFF
    digitalWrite(13, HIGH);
    decaduino.plmeRxDisableRequest();
    decaduino.pdDataRequest(txData, txLen);
    txData[txLen]= 0;
    Serial.print("Sending ["); Serial.print((char*)txData); Serial.print("]... ");
    while ( !decaduino.hasTxSucceeded() );
    Serial.println("done!");
    txLen = 0;
    decaduino.plmeRxEnableRequest();
    digitalWrite(13, LOW);
  }

  // This is the receiver part
  if ( decaduino.rxFrameAvailable() ) {
    // LED ON, print received chars, re-enable RX and LED OFF
    digitalWrite(13, HIGH);
    Serial.print("["); Serial.print(++rxFrames); Serial.print("] ");
    Serial.print(rxLen);
    Serial.print("bytes received: ");
    rxData[rxLen] = 0;
    Serial.println((char*)rxData);
    decaduino.plmeRxEnableRequest();
    digitalWrite(13, LOW);
  }
}


