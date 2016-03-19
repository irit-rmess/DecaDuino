#include <SPI.h>
#include <DecaDuino.h>

#define AIR_SPEED_OF_LIGHT 229702547.0
#define DW1000_TIMEBASE 15.65E-12
#define COEFF AIR_SPEED_OF_LIGHT*DW1000_TIMEBASE

#define TIMEOUT 10

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_WAIT_NEW_CYCLE 2
#define TWR_ENGINE_STATE_SEND_START 3
#define TWR_ENGINE_STATE_WAIT_START_SENT 4
#define TWR_ENGINE_STATE_MEMORISE_T1 5
#define TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ 6
#define TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ 7
#define TWR_ENGINE_STATE_WAIT_ACK_REQ 8
#define TWR_ENGINE_STATE_MEMORISE_T4 9
#define TWR_ENGINE_STATE_SEND_ACK 10 
#define TWR_ENGINE_STATE_WAIT_ACK_SENT 11
#define TWR_ENGINE_STATE_MEMORISE_T5 12
#define TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY 13
#define TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY 14
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 15
#define TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 16

#define TWR_MSG_TYPE_UNKNOW 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK_REQ 2
#define TWR_MSG_TYPE_ACK 3
#define TWR_MSG_TYPE_DATA_REPLY 4

int rxFrames;

uint64_t t1, t2, t3, t4, t5, t6;
int32_t tof;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
int timeout;

void setup() {

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if (!decaduino.init()){
    Serial.print("decaduino init failled");
    while(1){
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
    }
  }
  decaduino.setRxBuffer(rxData, &rxLen);
  state=TWR_ENGINE_STATE_INIT;
}

void loop() {
  
  switch (state) {
  
    case TWR_ENGINE_STATE_INIT :
      decaduino.plmeRxDisableRequest();
      state = TWR_ENGINE_STATE_WAIT_NEW_CYCLE;
      break;
    
    case TWR_ENGINE_STATE_WAIT_NEW_CYCLE :
      delay(1000);
      Serial.println("New SDS_TWR");
      state = TWR_ENGINE_STATE_SEND_START;
      break;
    
    case TWR_ENGINE_STATE_SEND_START :
      txData[0] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 1);
      state = TWR_ENGINE_STATE_WAIT_START_SENT;
      break;
       
    case TWR_ENGINE_STATE_WAIT_START_SENT :
      if (decaduino.hasTxSucceeded())
      state = TWR_ENGINE_STATE_MEMORISE_T1;
      break;
       
    case TWR_ENGINE_STATE_MEMORISE_T1 :
      t1 = decaduino.lastTxTimestamp;
      state = TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ :
      timeout = millis() + TIMEOUT;
      state = TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ :
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_WAIT_ACK_REQ :
      if ( millis() > timeout) {
      state = TWR_ENGINE_STATE_INIT;}
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == TWR_MSG_TYPE_ACK_REQ) {
          state = TWR_ENGINE_STATE_MEMORISE_T4;
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
        }
      }
      break;
       
    case TWR_ENGINE_STATE_MEMORISE_T4 :
      t4 = decaduino.lastRxTimestamp;
      state = TWR_ENGINE_STATE_SEND_ACK;
      break;
       
    case TWR_ENGINE_STATE_SEND_ACK :
      txData[0]= TWR_MSG_TYPE_ACK;
      decaduino.pdDataRequest(txData, 1);
      state = TWR_ENGINE_STATE_WAIT_ACK_SENT;
      break;
       
    case TWR_ENGINE_STATE_WAIT_ACK_SENT :
      if (decaduino.hasTxSucceeded()){
      state = TWR_ENGINE_STATE_MEMORISE_T5;}
      break;
      
    case TWR_ENGINE_STATE_MEMORISE_T5 :
      t5 = decaduino.lastTxTimestamp;
      state = TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY :
      timeout = millis() + TIMEOUT;
      state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY :
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_WAIT_DATA_REPLY :
      if ( millis() > timeout) {
      state = TWR_ENGINE_STATE_INIT;}
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == TWR_MSG_TYPE_DATA_REPLY) {
          state = TWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
        }
      }
      break;
       
    case TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 :
      t2 = decaduino.decodeUint64(&rxData[1]);
      t3 = decaduino.decodeUint64(&rxData[9]);
      t6 = decaduino.decodeUint64(&rxData[17]);
      tof = (2*t4 - t1 - 2*t3 + t2 + t6 - t5)/4;
      Serial.print("ToF=");
      Serial.print(tof);
      Serial.print(" d=");
      Serial.print(tof*COEFF);
      Serial.println();
      state = TWR_ENGINE_STATE_INIT;
      break;
       
    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}
