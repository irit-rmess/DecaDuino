#include <spi4teensy3.h>
#include <DecaDuino.h>

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
#define TWR_ENGINE_STATE_ON_FOR_DATA_REQUEST 13
#define TWR_ENGINE_WAIT_DATAREQ 14

#define TWR_MSG_TYPE_UNKNOW 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK_REQ 2
#define TWR_MSG_TYPE_ACK 3
#define TWR_MSG_TYPE_DATA_REPLY 4
#define TWR_MSG_TYPE_DATAREQ 5

int i;
int rxFrames;
int seqnum;

uint64_t t2, t3, t6;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
int timeout;

//uint32_t addrPANID=0xDECA0022;
//uint32_t addrPANID=0xDECA0023;
uint32_t addrPANID=0xDECA0024;
int dtim;

void setup() {

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if (!decaduino.init(addrPANID)){
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
  seqnum=0;
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
         if ( rxData[9] == TWR_MSG_TYPE_START){
            state = TWR_ENGINE_STATE_MEMORISE_T2;
            //selon position dans liste, attendre
            //
            switch((uint16_t)addrPANID){
            case 0x23: dtim=10;
            break;
            case 0x22: dtim=0;
            break;
            case 0x24: dtim=20;
            break;
            
            }
            delay(dtim);
            Serial.println("received START");
         } else state = TWR_ENGINE_STATE_RX_ON; 
       }
       break;
       
    case TWR_ENGINE_STATE_MEMORISE_T2 :
       t2 = decaduino.lastRxTimestamp;
       state = TWR_ENGINE_STATE_SEND_ACK_REQ;
       break;
         
    case TWR_ENGINE_STATE_SEND_ACK_REQ : 
        txData[0] =0x41;
        txData[1] =0x88;
        txData[2] =seqnum;      
        txData[3] =0xCA;
        txData[4] =0xDE;
        txData[5] =0x25;
        txData[6] =0x00;
        txData[7] =(uint16_t)addrPANID;//0x23;
        txData[8] =0x00;
        txData[9]= TWR_MSG_TYPE_ACK_REQ;
       decaduino.pdDataRequest(txData, 10);
       state = TWR_ENGINE_STATE_WAIT_SENT;
             seqnum++;
       break;
       
    case TWR_ENGINE_STATE_WAIT_SENT:
       if (decaduino.hasTxSucceeded()){
      state = TWR_ENGINE_STATE_MEMORISE_T3; 
        Serial.println("_________________got T3!");
      }
       break;
       
    case TWR_ENGINE_STATE_MEMORISE_T3 :
       t3 = decaduino.lastTxTimestamp;
       //state = TWR_ENGINE_STATE_WATCHDOG_FOR_ACK;
       state = TWR_ENGINE_STATE_ON_FOR_DATA_REQUEST;
       break;
    case TWR_ENGINE_STATE_ON_FOR_DATA_REQUEST :
       decaduino.plmeRxEnableRequest();
       state=TWR_ENGINE_WAIT_DATAREQ;
       break;
    case TWR_ENGINE_WAIT_DATAREQ :
          if (decaduino.rxFrameAvailable()){
           if (rxData[9] == TWR_MSG_TYPE_DATAREQ) {
             t6 = decaduino.lastRxTimestamp;
             Serial.println("Received DATAREQ____sent datareply");
             //time to reply!
             switch((uint16_t)addrPANID){
            case 0x23: dtim=20;
            break;
            case 0x22: dtim=10;
            break;
            case 0x24: dtim=30;
            break;
            
            }
            delay(dtim);
             txData[0] =0x41;
            txData[1] =0x88;
            txData[2] =seqnum;      
            txData[3] =0xCA;
            txData[4] =0xDE;
            txData[5] =0x25;
            txData[6] =0x00;
            txData[7] =(uint16_t)addrPANID;//0x23;
            txData[8] =0x00;
             txData[9] = TWR_MSG_TYPE_DATA_REPLY;
             decaduino.encodeUint64(t2, &txData[10]);
             decaduino.encodeUint64(t3, &txData[19]);
             decaduino.encodeUint64(t6, &txData[27]);
             decaduino.pdDataRequest(txData, 35);
             state = TWR_ENGINE_STATE_INIT;  
          seqnum++;         
           } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
         }
       break;
//    case TWR_ENGINE_STATE_WATCHDOG_FOR_ACK :
//       timeout = millis() + TIMEOUT;
//       state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
//       break;
//       
//    case TWR_ENGINE_STATE_RX_ON_FOR_ACK :
//       decaduino.plmeRxEnableRequest();
//       state = TWR_ENGINE_STATE_WAIT_ACK;
//       break;
//       
//    case TWR_ENGINE_STATE_WAIT_ACK :
//       if ( millis() > timeout) {
//       state = TWR_ENGINE_STATE_INIT;}
//       else { 
//         if (decaduino.rxFrameAvailable()){
//           if (rxData[0] == TWR_MSG_TYPE_ACK) {
//           state = TWR_ENGINE_STATE_MEMORISE_T6;
//           } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK;
//         }
//       }
//       break;
//       
//    case TWR_ENGINE_STATE_MEMORISE_T6 :
//       t6 = decaduino.lastRxTimestamp;
//       state = TWR_ENGINE_STATE_SEND_DATA_REPLY; 
//       break;
//       
//    case TWR_ENGINE_STATE_SEND_DATA_REPLY :
//       txData[0] = TWR_MSG_TYPE_DATA_REPLY;
//       decaduino.encodeUint64(t2, &txData[1]);
//       decaduino.encodeUint64(t3, &txData[9]);
//       decaduino.encodeUint64(t6, &txData[17]);
//       decaduino.pdDataRequest(txData, 25);
//       state = TWR_ENGINE_STATE_INIT;
//       break;
       
    default :
      state = TWR_ENGINE_STATE_INIT;
      break;  
  }
}
