// DecaDuinoVoltageSenderWithOptionnalRxPeriod
// This sketch shows how to use the DecaDuino library to send messages containing voltage and temperature over the UWB radio 
// with an optionnal short RX window after the message sending
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>
#include <DWM100x_id.h>

#define RX_ENABLE
#define RX_ENABLE_DURATION 100
#define SENDING_PERIOD 2000

#define MAX_FRAME_LEN 120
uint8_t txData[MAX_FRAME_LEN];
uint16_t txLen;
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
uint16_t nodeId;
uint32_t timeToSend = 0;
uint32_t timeToStopRx = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
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

  pinMode(1, INPUT);
  nodeId = toNodeID(getDwMacAddrUint64());

#ifdef RX_ENABLE
  // Set RX buffer and enable RX
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  rxFrames = 0;
#endif

}


void loop()
{
  decaduino.engine();
  if ( millis() >= timeToSend ) 
  {
    digitalWrite(LED_RED, LED_ON);
    float voltage = decaduino.getVoltage();
    float temperature = decaduino.getTemperature();
    txData[0] = 'B'; txData[1] = 'A'; txData[2] = 'T';
    decaduino.encodeUint16(nodeId, &txData[3]);
    txData[5] = digitalRead(1);
    decaduino.encodeFloat(voltage, &txData[6]);
    decaduino.encodeFloat(temperature, &txData[10]);
#ifdef RX_ENABLE
    decaduino.plmeRxDisableRequest();
#ifdef RX_ENABLE_DURATION
    timeToStopRx = timeToSend+RX_ENABLE_DURATION;
#endif
#endif
    decaduino.pdDataRequest(txData, 14);
    while ( !decaduino.hasTxSucceeded() ) { decaduino.engine(); };
#ifdef RX_ENABLE
    decaduino.plmeRxEnableRequest();
#endif
    digitalWrite(LED_RED, LED_OFF);
    timeToSend += SENDING_PERIOD;
  }

#ifdef RX_ENABLE_DURATION
  if ( millis() >= timeToStopRx ) 
  {
    decaduino.plmeRxDisableRequest();
    timeToStopRx += SENDING_PERIOD;
  }
#endif

#ifdef RX_ENABLE
  // If a message has been received, drop it and re-enable receiver
  if ( decaduino.rxFrameAvailable() )
  {
    digitalWrite(LED_GREEN, LED_ON);
    delay(20);
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_GREEN, LED_OFF);
  }
#endif
}
