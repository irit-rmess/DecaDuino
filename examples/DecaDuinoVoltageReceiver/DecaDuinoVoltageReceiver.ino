// DecaDuinoVoltageReceiver
// This sketch shows how to use the DecaDuino library to receive messages over the UWB radio from the DecaDuinoVoltageSender code
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
int rxFrames;
#ifdef UWB_MODULE_DWM1001
DecaDuino decaduino(SS1, DW_IRQ);
#ifdef ARDUINO_DWM1001_DEV
#define LED_ON LOW
#define LED_OFF HIGH
#else
#define LED_ON HIGH
#define LED_OFF LOW
#endif
#else
DecaDuino decaduino;
#define LED_ON HIGH
#define LED_OFF LOW
#endif


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED (pin LED_BUILTIN on DecaWiNo board)
  pinMode(LED_GREEN, OUTPUT); // Internal LED (pin LED_BUILTIN on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
  #ifndef UWB_MODULE_DWM1001
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
  decaduino.engine();
  // If a message has been received, print it and re-enable receiver
  if ( decaduino.rxFrameAvailable() )
  {
    char str[16];
    str[15] = 0;
    digitalWrite(LED_GREEN, LED_ON);

    if ( (rxData[0]=='B') && (rxData[1]=='A') && (rxData[2]=='T') )
    {
      char str[256];
      int inCharge;
      float voltage = decaduino.decodeFloat(&rxData[6]);
      float temperature = decaduino.decodeFloat(&rxData[10]);
      uint16_t nodeId = decaduino.decodeUint16(&rxData[3]);
      
      if ( rxData[5] ) inCharge = 0;
      else inCharge = 1;
      sprintf(str,"{\"id\":%d,\"volt\":%.2f,\"temp\":%.1f,\"ichrg\":%d}", nodeId, voltage, temperature, inCharge);
      Serial.print(str);
      Serial.println("");

    }
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_GREEN, LED_OFF);
  }
}
