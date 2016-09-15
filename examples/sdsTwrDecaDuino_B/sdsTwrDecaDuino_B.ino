#define ENABLE_CALIBRATION_FROM_EEPROM

#include <SPI.h>
#include <DecaDuino.h>
#ifdef ENABLE_CALIBRATION_FROM_EEPROM
#include <EEPROM.h>
uint16_t antennaDelay;
#endif

#define TIMEOUT 10

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_RX_ON 2
#define TWR_ENGINE_STATE_WAIT_START 3
#define TWR_ENGINE_STATE_MEMORISE_T2 4
#define TWR_ENGINE_STATE_SEND_ACK_REQ 5
#define TWR_ENGINE_STATE_WAIT_SENT 6
#define TWR_ENGINE_STATE_MEMORISE_T3 7
#define TWR_ENGINE_STATE_WATCHDOG_FOR_ACK 8
#define TWR_ENGINE_STATE_RX_ON_FOR_ACK 9
#define TWR_ENGINE_STATE_WAIT_ACK 10
#define TWR_ENGINE_STATE_MEMORISE_T6 11
#define TWR_ENGINE_STATE_SEND_DATA_REPLY 12

#define TWR_MSG_TYPE_UNKNOW 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK_REQ 2
#define TWR_MSG_TYPE_ACK 3
#define TWR_MSG_TYPE_DATA_REPLY 4

int i;
int rxFrames;

uint64_t t2, t3, t6;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
int timeout;

void setup() {

  uint8_t buf[2];

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if ( !decaduino.init() ) {
    Serial.print("decaduino init failed");
    while(1) {
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
    }
  }

#ifdef ENABLE_CALIBRATION_FROM_EEPROM

  // Gets antenna delay from the end of EEPROM. The two last bytes are used for DecaWiNo label, 
  // so use n-2 and n-3 to store the antenna delay (16bit value)
  buf[0] = EEPROM.read(EEPROM.length()-4);
  buf[1] = EEPROM.read(EEPROM.length()-3);
  antennaDelay = decaduino.decodeUint16(buf);

  if ( antennaDelay == 0xffff ) {
    Serial.println("Unvalid antenna delay value found in EEPROM. Using default value.");
  } else decaduino.setAntennaDelay(antennaDelay);

#endif

  decaduino.setRxBuffer(rxData, &rxLen);
  state=TWR_ENGINE_STATE_INIT;
}

void loop() {
  
  switch (state) {
  
    case TWR_ENGINE_STATE_INIT :
       state = TWR_ENGINE_STATE_RX_ON;
       break;
       
    case TWR_ENGINE_STATE_RX_ON :
       decaduino.plmeRxEnableRequest();
       state = TWR_ENGINE_STATE_WAIT_START;
       break;
       
    case TWR_ENGINE_STATE_WAIT_START :
       if (decaduino.rxFrameAvailable()){
         if ( rxData[0] == TWR_MSG_TYPE_START){
            state = TWR_ENGINE_STATE_MEMORISE_T2;
         } else state = TWR_ENGINE_STATE_RX_ON; 
       }
       break;
       
    case TWR_ENGINE_STATE_MEMORISE_T2 :
       t2 = decaduino.lastRxTimestamp;
       state = TWR_ENGINE_STATE_SEND_ACK_REQ;
       break;
         
    case TWR_ENGINE_STATE_SEND_ACK_REQ : 
       txData[0]= TWR_MSG_TYPE_ACK_REQ;
       decaduino.pdDataRequest(txData, 1);
       state = TWR_ENGINE_STATE_WAIT_SENT;
       break;
       
    case TWR_ENGINE_STATE_WAIT_SENT:
       if (decaduino.hasTxSucceeded()){
      state = TWR_ENGINE_STATE_MEMORISE_T3; }
       break;
       
    case TWR_ENGINE_STATE_MEMORISE_T3 :
       t3 = decaduino.lastTxTimestamp;
       state = TWR_ENGINE_STATE_WATCHDOG_FOR_ACK;
       break;
       
    case TWR_ENGINE_STATE_WATCHDOG_FOR_ACK :
       timeout = millis() + TIMEOUT;
       state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
       break;
       
    case TWR_ENGINE_STATE_RX_ON_FOR_ACK :
       decaduino.plmeRxEnableRequest();
       state = TWR_ENGINE_STATE_WAIT_ACK;
       break;
       
    case TWR_ENGINE_STATE_WAIT_ACK :
       if ( millis() > timeout) {
       state = TWR_ENGINE_STATE_INIT;}
       else { 
         if (decaduino.rxFrameAvailable()){
           if (rxData[0] == TWR_MSG_TYPE_ACK) {
           state = TWR_ENGINE_STATE_MEMORISE_T6;
           } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
         }
       }
       break;
       
    case TWR_ENGINE_STATE_MEMORISE_T6 :
       t6 = decaduino.lastRxTimestamp;
       state = TWR_ENGINE_STATE_SEND_DATA_REPLY; 
       break;
       
    case TWR_ENGINE_STATE_SEND_DATA_REPLY :
       txData[0] = TWR_MSG_TYPE_DATA_REPLY;
       decaduino.encodeUint64(t2, &txData[1]);
       decaduino.encodeUint64(t3, &txData[9]);
       decaduino.encodeUint64(t6, &txData[17]);
       decaduino.pdDataRequest(txData, 25);
       state = TWR_ENGINE_STATE_INIT;
       break;
       
    default :
      state = TWR_ENGINE_STATE_INIT;
      break;  
  }
}
