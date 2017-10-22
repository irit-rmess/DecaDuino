// DecaDuinoTWR_server
// A simple implementation of the TWR protocol, server side
// Contributors: Adrien van den Bossche, Réjane Dalcé, Ibrahim Fofana, Robert Try, Thierry Val
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

// Timeout parameters
#define TIMEOUT_WAIT_ACK_SENT 5 //ms
#define TIMEOUT_WAIT_DATA_REPLY_SENT 5 //ms
#define ACK_DATA_REPLY_INTERFRAME 10 //ms

// TWR server states state machine enumeration: see state diagram on documentation for more details
enum { TWR_ENGINE_STATE_INIT, TWR_ENGINE_STATE_WAIT_START, TWR_ENGINE_STATE_MEMORISE_T2,
TWR_ENGINE_STATE_SEND_ACK, TWR_ENGINE_STATE_WAIT_ACK_SENT, TWR_ENGINE_STATE_MEMORISE_T3,
TWR_ENGINE_STATE_SEND_DATA_REPLY, TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT };

// Message types of the TWR protocol
#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK 2
#define TWR_MSG_TYPE_DATA_REPLY 3

uint64_t t2, t3;

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
  state = TWR_ENGINE_STATE_INIT;
}


void loop()
{
  switch (state) {
   
    case TWR_ENGINE_STATE_INIT:
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_START;
      break;

    case TWR_ENGINE_STATE_WAIT_START:
      if ( decaduino.rxFrameAvailable() ) {
        if ( rxData[0] == TWR_MSG_TYPE_START ) {
          state = TWR_ENGINE_STATE_MEMORISE_T2;
        } else {
					state = TWR_ENGINE_STATE_INIT;
				}
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T2:
      t2 = decaduino.getLastRxTimestamp();
      state = TWR_ENGINE_STATE_SEND_ACK;
      break;

    case TWR_ENGINE_STATE_SEND_ACK:
      txData[0] = TWR_MSG_TYPE_ACK;
      decaduino.pdDataRequest(txData, 1);
      timeout = millis() + TIMEOUT_WAIT_ACK_SENT;
      state = TWR_ENGINE_STATE_WAIT_ACK_SENT;
      break;

    case TWR_ENGINE_STATE_WAIT_ACK_SENT:
      if ( millis() > timeout ) {
        state = TWR_ENGINE_STATE_INIT;
      } else {
        if ( decaduino.hasTxSucceeded() ) {
          state = TWR_ENGINE_STATE_MEMORISE_T3;  
        }
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T3:
      t3 = decaduino.getLastTxTimestamp();
      state = TWR_ENGINE_STATE_SEND_DATA_REPLY;
      break;

    case TWR_ENGINE_STATE_SEND_DATA_REPLY:
    	delay(ACK_DATA_REPLY_INTERFRAME);
      txData[0] = TWR_MSG_TYPE_DATA_REPLY;
      decaduino.encodeUint40(t2, &txData[1]);
      decaduino.encodeUint40(t3, &txData[6]);
      decaduino.pdDataRequest(txData, 11);
      timeout = millis() + TIMEOUT_WAIT_DATA_REPLY_SENT;
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT;
      break;
 
 		case TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT:
      if ( (millis()>timeout) || (decaduino.hasTxSucceeded()) ) {
        state = TWR_ENGINE_STATE_INIT;
      }
      break;
 		
    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}


