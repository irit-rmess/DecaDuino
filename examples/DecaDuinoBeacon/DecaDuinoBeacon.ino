// DecaDuinoBeacon
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

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


uint64_t getDwMacAddrUint64()
{
  return (NRF_FICR->DEVICEADDR[1] * 0x100000000) | NRF_FICR->DEVICEADDR[0];
}


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

  switch ( dwMacAddr )
  {
    case 0xfc55bd52582b4543: myAddress = 100; break; // 100
    case 0x43145fe1013ae4c3: myAddress = 101; break; // 101
    case 0x99398ce627393b5c: myAddress = 102; break; // 102
    case 0x103514eac0ab0d9d: myAddress = 103; break; // 103
    case 0x2b8f349bbcb0aabd: myAddress = 104; break; // 104
    case 0x7bd99bc4bae559fc: myAddress = 105; break; // 105
    case 0x847d584201108686: myAddress = 106; break; // 106
    case 0x9eaea872f9d6b996: myAddress = 107; break; // 107
    case 0x5ef583659124cdd6: myAddress = 108; break; // 108
    case 0x6e27f809be495edd: myAddress = 109; break; // 109
    case 0x5f20a93050af01b5: myAddress = 110; break; // 110
    case 0xde32acf71c29add6: myAddress = 111; break; // 111
    case 0xcefb49379bde4411: myAddress = 115; break; // 115
    //case 0x: myAddress = 116; break; // 116
    case 0x5cd8c0b8a1af863a: myAddress = 117; break; // 117
    case 0x9313b1f763fac473: myAddress = 118; break; // 118
    case 0x6b6c03e429789c24: myAddress = 119; break; // 119
    case 0x9984c78adfe8c818: myAddress = 142; break; // 142
    case 0xbe1a2bcb17af5e71: myAddress = 143; break; // 143
    case 0xde9b67ae2c99f3d5: myAddress = 144; break; // 144
    case 0xa06723f28aa6aa4e: myAddress = 145; break; // 145
    case 0x34df5388fb866553: myAddress = 146; break; // 146
    case 0x139e9dbae75ee553: myAddress = 147; break; // 147
    case 0xa12ed9bcb2237c12: myAddress = 148; break; // 148
    case 0x7c3dce7f589baeeb: myAddress = 149; break; // 149
    case 0xedb0988610994ea3: myAddress = 150; break; // 150
    case 0x53fd46cf48102b52: myAddress = 151; break; // 151
    case 0xf0a46ff55426ba69: myAddress = 152; break; // 152
    case 0x7368bfea8acf8003: myAddress = 153; break; // 153

    default:
      delay(3000);
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

    while ( !decaduino.hasTxSucceeded() );

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
