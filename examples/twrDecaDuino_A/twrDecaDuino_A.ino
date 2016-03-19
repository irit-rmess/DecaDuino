#include <SPI.h>
#include <DecaDuino.h>

#define AIR_SPEED_OF_LIGHT 229702547.0
#define DW1000_TIMEBASE 15.65E-12
#define COEFF AIR_SPEED_OF_LIGHT*DW1000_TIMEBASE

#define X_CORRECTION 1.0000000
//#define Y_CORRECTION 0.230000000
#define Y_CORRECTION 0.000000000

#define TIMEOUT 20

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_WAIT_NEW_CYCLE 2
#define TWR_ENGINE_STATE_SEND_START 3
#define TWR_ENGINE_STATE_WAIT_SEND_START 4
#define TWR_ENGINE_STATE_MEMORISE_T1 5
#define TWR_ENGINE_STATE_WATCHDOG_FOR_ACK 6
#define TWR_ENGINE_STATE_RX_ON_FOR_ACK 7
#define TWR_ENGINE_STATE_WAIT_ACK 8
#define TWR_ENGINE_STATE_MEMORISE_T4 9
#define TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY 10
#define TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY 11
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 12
#define TWR_ENGINE_STATE_EXTRACT_T2_T3 13

#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK 2
#define TWR_MSG_TYPE_DATA_REPLY 3

int i;
int rxFrames;

uint64_t t1, t2, t3, t4;
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
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) {
      digitalWrite(13, HIGH); 
      delay(50);    
      digitalWrite(13, LOW); 
      delay(50);    
    }
  }
  
  // Set RX buffer
  decaduino.setRxBuffer(rxData, &rxLen);
  state = TWR_ENGINE_STATE_INIT;
}

void loop() {

  float distance;
  
  switch (state) {
   
    case TWR_ENGINE_STATE_INIT:
      decaduino.plmeRxDisableRequest();
      state = TWR_ENGINE_STATE_WAIT_NEW_CYCLE;
      break;
      
    case TWR_ENGINE_STATE_WAIT_NEW_CYCLE:
      delay(1000);
      //delay(100);
      Serial.println("New TWR");
      state = TWR_ENGINE_STATE_SEND_START;
      break;

    case TWR_ENGINE_STATE_SEND_START:
      txData[0] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 1);
      state = TWR_ENGINE_STATE_WAIT_SEND_START;
      break;

    case TWR_ENGINE_STATE_WAIT_SEND_START:
      if ( decaduino.hasTxSucceeded() )
        state = TWR_ENGINE_STATE_MEMORISE_T1;
      break;

    case TWR_ENGINE_STATE_MEMORISE_T1:
      t1 = decaduino.lastTxTimestamp;
      state = TWR_ENGINE_STATE_WATCHDOG_FOR_ACK;
      break;

    case TWR_ENGINE_STATE_WATCHDOG_FOR_ACK:
      timeout = millis() + TIMEOUT;
      state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
      break;     

    case TWR_ENGINE_STATE_RX_ON_FOR_ACK:
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_ACK;
      break;

    case TWR_ENGINE_STATE_WAIT_ACK:
      if ( millis() > timeout ) {
        state = TWR_ENGINE_STATE_INIT;
      } else {
        if ( decaduino.rxFrameAvailable() ) {
          if ( rxData[0] == TWR_MSG_TYPE_ACK ) {
            state = TWR_ENGINE_STATE_MEMORISE_T4;
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
        }
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T4:
      t4 = decaduino.lastRxTimestamp;
      state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
      break;

    case TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY:
      timeout = millis() + TIMEOUT;
      state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
      break;     

    case TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY:
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;

    case TWR_ENGINE_STATE_WAIT_DATA_REPLY:
      if ( millis() > timeout ) {
        state = TWR_ENGINE_STATE_INIT;
      } else {
        if ( decaduino.rxFrameAvailable() ) {
          if ( rxData[0] == TWR_MSG_TYPE_DATA_REPLY ) {
            state = TWR_ENGINE_STATE_EXTRACT_T2_T3;
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
        }
      }
      break;

    case TWR_ENGINE_STATE_EXTRACT_T2_T3:
      t2 = decaduino.decodeUint64(&rxData[1]);
      t3 = decaduino.decodeUint64(&rxData[9]);
      tof = (t4 - t1 - (t3 - t2))/2;
      distance = tof*COEFF*X_CORRECTION + Y_CORRECTION;
      Serial.print("ToF=");
      Serial.print(tof, HEX);
      Serial.print(" d=");
      Serial.print(distance);
      Serial.println();
      state = TWR_ENGINE_STATE_INIT;
      break;

    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}


