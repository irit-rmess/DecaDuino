// DecaDuinoSDSTWR_client
// A simple implementation of the SDS-TWR protocol, client side
// Contributors: Adrien van den Bossche, Réjane Dalcé, Ibrahim Fofana, Robert Try, Thierry Val
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details
// https://arxiv.org/pdf/1603.06736.pdf

#include <SPI.h>
#include <DecaDuino.h>

// Timeout parameters
#define TIMEOUT_WAIT_START_SENT 5 //ms
#define TIMEOUT_WAIT_ACK_REQ 10 //ms
#define TIMEOUT_WAIT_ACK_SENT 5 //ms
#define TIMEOUT_WAIT_DATA_REPLY 10 //ms

// Ranging period parameter
#define RANGING_PERIOD 500 //ms

// SDS-TWR client states state machine enumeration
enum { SDSTWR_ENGINE_STATE_INIT, SDSTWR_ENGINE_STATE_WAIT_START_SENT, SDSTWR_ENGINE_STATE_MEMORISE_T1, 
SDSTWR_ENGINE_STATE_WAIT_ACK_REQ, SDSTWR_ENGINE_STATE_MEMORISE_T4, SDSTWR_ENGINE_STATE_SEND_ACK, 
SDSTWR_ENGINE_STATE_WAIT_ACK_SENT, SDSTWR_ENGINE_STATE_MEMORISE_T5, SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY, 
SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6 };

// Message types of the SDS-TWR protocol
#define SDSTWR_MSG_TYPE_UNKNOWN 10
#define SDSTWR_MSG_TYPE_START 11
#define SDSTWR_MSG_TYPE_ACK_REQ 12
#define SDSTWR_MSG_TYPE_ACK 13
#define SDSTWR_MSG_TYPE_DATA_REPLY 14

uint64_t t1, t2, t3, t4, t5, t6;
uint64_t mask = 0xFFFFFFFFFF;
int32_t tof;

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

  // Print top table colomns
  Serial.println("ToF\td");
}


void loop()
{
  float distance;
  
  switch (state) {
  
    case SDSTWR_ENGINE_STATE_INIT:
      delay(RANGING_PERIOD); // Wait to avoid medium flooding between two rangings or if a ranging fails
      decaduino.plmeRxDisableRequest();
      Serial.println("New SDS-TWR");
      txData[0] = SDSTWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 1);
      timeout = millis() + TIMEOUT_WAIT_START_SENT;
      state = SDSTWR_ENGINE_STATE_WAIT_START_SENT;
      break;
      
    case SDSTWR_ENGINE_STATE_WAIT_START_SENT:
      if ( millis() > timeout ) {
        state = SDSTWR_ENGINE_STATE_INIT;
      } else {
        if (decaduino.hasTxSucceeded()) {
        	state = SDSTWR_ENGINE_STATE_MEMORISE_T1;
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_MEMORISE_T1 :
      t1 = decaduino.getLastTxTimestamp();
      timeout = millis() + TIMEOUT_WAIT_ACK_REQ;
      decaduino.plmeRxEnableRequest();
      state = SDSTWR_ENGINE_STATE_WAIT_ACK_REQ;
      break;
       
    case SDSTWR_ENGINE_STATE_WAIT_ACK_REQ :
      if ( millis() > timeout) {
      	state = SDSTWR_ENGINE_STATE_INIT;
      }
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == SDSTWR_MSG_TYPE_ACK_REQ) {
          	state = SDSTWR_ENGINE_STATE_MEMORISE_T4;
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = SDSTWR_ENGINE_STATE_WAIT_ACK_REQ;
          }
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_MEMORISE_T4 :
      t4 = decaduino.getLastRxTimestamp();
      state = SDSTWR_ENGINE_STATE_SEND_ACK;
      break;
       
    case SDSTWR_ENGINE_STATE_SEND_ACK :
      txData[0]= SDSTWR_MSG_TYPE_ACK;
      decaduino.pdDataRequest(txData, 1);
      timeout = millis() + TIMEOUT_WAIT_ACK_SENT;
      state = SDSTWR_ENGINE_STATE_WAIT_ACK_SENT;
      break;
       
    case SDSTWR_ENGINE_STATE_WAIT_ACK_SENT :
      if ( millis() > timeout ) {
        state = SDSTWR_ENGINE_STATE_INIT;
      } else {
        if (decaduino.hasTxSucceeded()){
        	state = SDSTWR_ENGINE_STATE_MEMORISE_T5;
        }
      }
      break;
      
    case SDSTWR_ENGINE_STATE_MEMORISE_T5 :
      t5 = decaduino.getLastTxTimestamp();
      timeout = millis() + TIMEOUT_WAIT_DATA_REPLY;
      decaduino.plmeRxEnableRequest();
      state = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;
       
    case SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY :
      if ( millis() > timeout) {
      	state = SDSTWR_ENGINE_STATE_INIT;
      }
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == SDSTWR_MSG_TYPE_DATA_REPLY) {
          state = SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY;
          	}
        }
      }
      break;
       
    case SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6 :
      t2 = decaduino.decodeUint40(&rxData[1]);
      t3 = decaduino.decodeUint40(&rxData[6]);
      t6 = decaduino.decodeUint40(&rxData[11]);
      tof = (((((t4 - t1) & mask) - ((t3 - t2) & mask)) & mask) + ((((t6 - t3) & mask) - ((t5 - t4) & mask)) & mask))/4;
      distance = tof*RANGING_UNIT;
      Serial.print(tof);
      Serial.print("\t");
      Serial.println(distance);
      state = SDSTWR_ENGINE_STATE_INIT;
      break;
       
    default:
      state = SDSTWR_ENGINE_STATE_INIT;
      break;
  }
}
