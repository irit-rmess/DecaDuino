// DecaDuinoSDSTWR_server
// A simple implementation of the TWR protocol, server side
// Contributors: Adrien van den Bossche, Réjane Dalcé, Ibrahim Fofana, Robert Try, Thierry Val
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

// Timeout parameters
#define TIMEOUT_WAIT_ACK_REQ_SENT 5 //ms
#define TIMEOUT_WAIT_ACK 10 //ms
#define TIMEOUT_WAIT_DATA_REPLY_SENT 5 //ms

// SDS-TWR server states state machine enumeration: see state diagram on documentation for more details
enum { SDSTWR_ENGINE_STATE_INIT, SDSTWR_ENGINE_STATE_WAIT_START, SDSTWR_ENGINE_STATE_MEMORISE_T2, 
SDSTWR_ENGINE_STATE_SEND_ACK_REQ, SDSTWR_ENGINE_STATE_WAIT_ACK_REQ_SENT, SDSTWR_ENGINE_STATE_MEMORISE_T3, 
SDSTWR_ENGINE_STATE_WAIT_ACK, SDSTWR_ENGINE_STATE_MEMORISE_T6, SDSTWR_ENGINE_STATE_SEND_DATA_REPLY,
SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT};

// Message types of the SDS-TWR protocol
#define SDSTWR_MSG_TYPE_UNKNOWN 10
#define SDSTWR_MSG_TYPE_START 11
#define SDSTWR_MSG_TYPE_ACK_REQ 12
#define SDSTWR_MSG_TYPE_ACK 13
#define SDSTWR_MSG_TYPE_DATA_REPLY 14

uint64_t t2, t3, t6;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
uint32_t timeout;


void setup()
{
  pinMode(13, OUTPUT); // Internal LED (pin 13 on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(13, HIGH); delay(50); digitalWrite(13, LOW); delay(50); }
  }

  // Set RX buffer
  decaduino.setRxBuffer(rxData, &rxLen);
  state = SDSTWR_ENGINE_STATE_INIT;
}


void loop()
{
  switch (state) {
  
    case SDSTWR_ENGINE_STATE_INIT :
			decaduino.plmeRxEnableRequest();
      state = SDSTWR_ENGINE_STATE_WAIT_START;
      break;

    case SDSTWR_ENGINE_STATE_WAIT_START :
      if (decaduino.rxFrameAvailable()){
        if ( rxData[0] == SDSTWR_MSG_TYPE_START){
         state = SDSTWR_ENGINE_STATE_MEMORISE_T2;
       } else {
          decaduino.plmeRxEnableRequest();
          state = SDSTWR_ENGINE_STATE_WAIT_START; 
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_MEMORISE_T2 :
      t2 = decaduino.getLastRxTimestamp();
      state = SDSTWR_ENGINE_STATE_SEND_ACK_REQ;
      break;
         
    case SDSTWR_ENGINE_STATE_SEND_ACK_REQ : 
      txData[0]= SDSTWR_MSG_TYPE_ACK_REQ;
      decaduino.pdDataRequest(txData, 1);
      timeout = millis() + TIMEOUT_WAIT_ACK_REQ_SENT;
      state = SDSTWR_ENGINE_STATE_WAIT_ACK_REQ_SENT;
      break;
       
    case SDSTWR_ENGINE_STATE_WAIT_ACK_REQ_SENT:
      if ( millis() > timeout ) {
        state = SDSTWR_ENGINE_STATE_INIT;
      } else {
        if ( decaduino.hasTxSucceeded() ) {
          state = SDSTWR_ENGINE_STATE_MEMORISE_T3; 
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_MEMORISE_T3 :
      t3 = decaduino.getLastTxTimestamp();
      timeout = millis() + TIMEOUT_WAIT_ACK;
      decaduino.plmeRxEnableRequest();
      state = SDSTWR_ENGINE_STATE_WAIT_ACK;
      break;
 
    case SDSTWR_ENGINE_STATE_WAIT_ACK :
      if ( millis() > timeout) {
      	 state = SDSTWR_ENGINE_STATE_INIT;
      }
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == SDSTWR_MSG_TYPE_ACK) {
         	  state = SDSTWR_ENGINE_STATE_MEMORISE_T6;
          } else {
          	decaduino.plmeRxEnableRequest();
           	state = SDSTWR_ENGINE_STATE_WAIT_ACK;
          }
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_MEMORISE_T6 :
      t6 = decaduino.getLastRxTimestamp();
      state = SDSTWR_ENGINE_STATE_SEND_DATA_REPLY; 
      break;
      
    case SDSTWR_ENGINE_STATE_SEND_DATA_REPLY :
      txData[0] = SDSTWR_MSG_TYPE_DATA_REPLY;
      decaduino.encodeUint40(t2, &txData[1]);
      decaduino.encodeUint40(t3, &txData[6]);
      decaduino.encodeUint40(t6, &txData[11]);
      decaduino.pdDataRequest(txData, 16);
      timeout = millis() + TIMEOUT_WAIT_DATA_REPLY_SENT;
      state = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT;
      break;

    case SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT:
      if ( (millis()>timeout) || (decaduino.hasTxSucceeded()) ) {
        state = SDSTWR_ENGINE_STATE_INIT;
      }
      break;

    default :
      state = SDSTWR_ENGINE_STATE_INIT;
      break;  
  }
}
