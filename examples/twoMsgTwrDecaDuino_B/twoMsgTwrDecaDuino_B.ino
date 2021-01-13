#include <SPI.h>
#include <DecaDuino.h>

#define FRAME_LEN 64

#define TRANSMITION_DELAY 1000000000000/15.65*0.1*5E-3 // 0.1ms

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_RX_ON 2
#define TWR_ENGINE_STATE_WAIT_START 3
#define TWR_ENGINE_STATE_MEMORISE_T2 4
#define TWR_ENGINE_STATE_SEND_ACK 5
#define TWR_ENGINE_STATE_WAIT_SENT 6
#define TWR_ENGINE_STATE_MEMORISE_T3 7
#define TWR_ENGINE_STATE_WAIT_BEFORE_SEND_DATA_REPLY 8
#define TWR_ENGINE_STATE_SEND_DATA_REPLY 9 

#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK 2
#define TWR_MSG_TYPE_DATA_REPLY 3

int i;
int rxFrames;

uint64_t t2, t3, t3_predicted;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;


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
  decaduino.engine();

  switch (state) {
   
    case TWR_ENGINE_STATE_INIT:
      //decaduino.plmeRxDisableRequest();
      state = TWR_ENGINE_STATE_RX_ON;
      break;
      
    case TWR_ENGINE_STATE_RX_ON:
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_START;
      break;

    case TWR_ENGINE_STATE_WAIT_START:
      if ( decaduino.rxFrameAvailable() ) {
        if ( rxData[0] == TWR_MSG_TYPE_START ) {
          state = TWR_ENGINE_STATE_MEMORISE_T2;
        } else state = TWR_ENGINE_STATE_RX_ON;
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T2:
      t2 = decaduino.lastRxTimestamp;
      state = TWR_ENGINE_STATE_SEND_ACK;
      break;

    case TWR_ENGINE_STATE_SEND_ACK:
      txData[0] = TWR_MSG_TYPE_ACK;
      t3_predicted = decaduino.alignDelayedTransmission(TRANSMITION_DELAY);
      decaduino.encodeUint64(t2, &txData[1]);
      decaduino.encodeUint64(t3_predicted, &txData[9]);
      decaduino.pdDataRequest(txData, 17, true, t3_predicted);
      state = TWR_ENGINE_STATE_WAIT_SENT;
      break;

    case TWR_ENGINE_STATE_WAIT_SENT:
      if ( decaduino.hasTxSucceeded() ) {
        t3 = decaduino.lastTxTimestamp;
        Serial.print("t3-t3_predicted=");
        Serial.print((int)(t3-t3_predicted));
        Serial.println();
        state = TWR_ENGINE_STATE_INIT;
      }
      break;
 
    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}


