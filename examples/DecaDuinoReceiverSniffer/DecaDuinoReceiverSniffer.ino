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
int rxFrames;
#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#define LED_ON LOW
#define LED_OFF HIGH
#else
DecaDuino decaduino;
#define LED_ON HIGH
#define LED_OFF LOW
#endif


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED (pin LED_BUILTIN on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
  #ifndef ARDUINO_DWM1001_DEV
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)
  #endif

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(LED_BUILTIN, LED_ON); delay(50); digitalWrite(LED_BUILTIN, LED_OFF); delay(50); }
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
    digitalWrite(LED_BUILTIN, LED_ON);
    Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
    Serial.print(rxLen);
    Serial.print("bytes received: |");
    for (int i=0; i<rxLen; i++) {
      if ( rxData[i]<16 ) Serial.print("0");
      Serial.print(rxData[i], HEX);
      Serial.print("|");
    }
    Serial.println();
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
}


