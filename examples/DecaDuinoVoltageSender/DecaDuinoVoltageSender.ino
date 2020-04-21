// DecaDuinoVoltageSender
// This sketch shows how to use the DecaDuino library to send messages containing voltage and temperature over the UWB radio 
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>
#include <DWM100x_id.h>

#define MAX_FRAME_LEN 120
uint8_t txData[MAX_FRAME_LEN];
uint16_t txLen;
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
uint16_t nodeId;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  Serial.begin(115200); // Init Serial port
  #ifndef UWB_MODULE_DWM1001
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)
  #endif

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(LED_BUILTIN, LED_ON); delay(50); digitalWrite(LED_BUILTIN, LED_OFF); delay(50); }
  }

  pinMode(1, INPUT);
  nodeId = toNodeID(getDwMacAddrUint64());
}


void loop()
{
  digitalWrite(LED_RED, LED_ON);
  float voltage = decaduino.getVoltage();
  float temperature = decaduino.getTemperature();
  txData[0] = 'B'; txData[1] = 'A'; txData[2] = 'T';
  decaduino.encodeUint16(nodeId, &txData[3]);
  txData[5] = digitalRead(1);
  decaduino.encodeFloat(voltage, &txData[6]);
  decaduino.encodeFloat(temperature, &txData[10]);
  decaduino.pdDataRequest(txData, 14);
  while ( !decaduino.hasTxSucceeded() );
  digitalWrite(LED_RED, LED_OFF);
  // wait 2 second 
  delay(2000);
}
