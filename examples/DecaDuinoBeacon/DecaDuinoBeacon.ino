// DecaDuinoBeacon
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>
#include <DWM100x_id.h>

#define BEACON_PERIOD 2000
#define MAX_FRAME_LEN 15
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
uint8_t txData[MAX_FRAME_LEN];
uint16_t txLen;
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

uint32_t beacon_send_time = BEACON_PERIOD;
uint64_t dwMacAddr;
uint16_t myAddress = 0;
uint16_t myNseq = 0;


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

  dwMacAddr = getDwMacAddrUint64();

  decaduino.printUint64(getDwMacAddrUint64()); Serial.println(); Serial.flush();

  myAddress = toNodeID(getDwMacAddrUint64());
  if (myAddress == 0){
      Serial.print("NRF chip ID not registered ind DWM100x_id.cpp : ");
      decaduino.printUint64(getDwMacAddrUint64()); Serial.println(); Serial.flush();
      while(1) { digitalWrite(LED_BUILTIN, LED_ON); delay(200); digitalWrite(LED_BUILTIN, LED_OFF); delay(200); }
  }
  randomSeed(dwMacAddr);
}

#define SERIAL_BUFFER_LENGTH 2048
char serialBuffer[SERIAL_BUFFER_LENGTH];
int serialBufferWriteIndex = 0;
int serialBufferReadIndex = 0;


void pushSerialBuffer(char* str)
{
    int len = strlen(str);

    for (int i=0; i<len; i++)
    {
        serialBuffer[serialBufferWriteIndex] = str[i];
        if ( ++serialBufferWriteIndex == SERIAL_BUFFER_LENGTH ) serialBufferWriteIndex = 0;
    }
}


int pullSerialBuffer(char* c)
{
    if ( serialBufferReadIndex != serialBufferWriteIndex )
    {
      *c = serialBuffer[serialBufferReadIndex];
      if ( ++serialBufferReadIndex == SERIAL_BUFFER_LENGTH ) serialBufferReadIndex = 0;
      return true;
    }
    return false;
}


void pushFloatInSerialBuffer(float f)
{
    char str[16];
    int int_part = floor(f);
    int dec_part = f*1000 - int_part*1000;

    sprintf(str, "%d.%d", int_part, dec_part);
    pushSerialBuffer(str);
}


void loop()
{
  decaduino.engine();

  char c;

  // Pull one char in Serial Buffer per Arduino loop
  if ( pullSerialBuffer(&c) )
  {
    Serial.print(c);
  }

  // If it is time to send a beacon
  if ( millis() > beacon_send_time )
  {
    beacon_send_time += random(BEACON_PERIOD/2, BEACON_PERIOD*1.5);
    digitalWrite(LED_BUILTIN, LED_ON);
    decaduino.plmeRxDisableRequest(); // Always disable RX before sending a frame
    // make dummy data, send it and wait the end of the transmission.
    for (int i=0; i<MAX_FRAME_LEN; i++) txData[i] = i;
    decaduino.encodeUint16(myAddress, &txData[0]);
    decaduino.encodeUint16(++myNseq, &txData[2]);
    decaduino.pdDataRequest(txData, MAX_FRAME_LEN);

    while ( !decaduino.hasTxSucceeded() ) decaduino.engine();

    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_BUILTIN, LED_OFF);
  }

  // If a message has been received, print it and re-enable receiver
  if ( decaduino.rxFrameAvailable() ) {
    char str[1024];
    digitalWrite(LED_BUILTIN, LED_ON);
    ++rxFrames;
    uint16_t addr = decaduino.decodeUint16(&rxData[0]);
    uint16_t nseq = decaduino.decodeUint16(&rxData[2]);

    /*
    // Do not use direct Serial Printing
    int debut = micros();
    Serial.print("{");
    Serial.print("\"message_type\":\"rx_frame\"");
    Serial.print(",\"rx_addr\":"); Serial.print(addr);
    Serial.print(",\"nseq\":"); Serial.print(nseq);
    Serial.print(",\"nlos\":"); Serial.print(decaduino.getNLOSIndication());
    Serial.print(",\"skew\":"); Serial.print(decaduino.getLastRxSkew());
    Serial.print(",\"rssi\":"); Serial.print(decaduino.getRSSI());
    Serial.print(",\"snr\":"); Serial.print(decaduino.getSNR());
    Serial.print(",\"serial_time\":"); Serial.print(micros()-debut);
    Serial.println("}");
    */

    // Use Serial Buffer instead
    sprintf(str,"{"); pushSerialBuffer(str);
    sprintf(str,"\"message_type\":\"rx_frame\""); pushSerialBuffer(str);
    sprintf(str,",\"rx_addr\":%d", addr); pushSerialBuffer(str);
    sprintf(str,",\"nseq\":%d", nseq); pushSerialBuffer(str);
    sprintf(str,",\"nlos\":"); pushSerialBuffer(str); pushFloatInSerialBuffer(decaduino.getNLOSIndication());
    sprintf(str,",\"skew\":"); pushSerialBuffer(str); pushFloatInSerialBuffer(decaduino.getLastRxSkew());
    sprintf(str,",\"rssi\":"); pushSerialBuffer(str); pushFloatInSerialBuffer(decaduino.getRSSI());
    sprintf(str,",\"snr\":"); pushSerialBuffer(str); pushFloatInSerialBuffer(decaduino.getSNR());
    sprintf(str,"}\n"); pushSerialBuffer(str);

    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
}
