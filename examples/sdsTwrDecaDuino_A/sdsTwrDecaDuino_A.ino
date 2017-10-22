#define ENABLE_CALIBRATION_FROM_EEPROM

#include <SPI.h>
#include <DecaDuino.h>
#ifdef ENABLE_CALIBRATION_FROM_EEPROM
#include <EEPROM.h>
uint16_t antennaDelay;
#endif

#define COEFF RANGING_UNIT

#define TIMEOUT 10

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_WAIT_START_SENT 2
#define TWR_ENGINE_STATE_MEMORISE_T1 3
#define TWR_ENGINE_STATE_WAIT_ACK_REQ 4
#define TWR_ENGINE_STATE_MEMORISE_T4 5
#define TWR_ENGINE_STATE_SEND_ACK 6
#define TWR_ENGINE_STATE_WAIT_ACK_SENT 7
#define TWR_ENGINE_STATE_MEMORISE_T5 8
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 9
#define TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 10

#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK_REQ 2
#define TWR_MSG_TYPE_ACK 3
#define TWR_MSG_TYPE_DATA_REPLY 4

int rxFrames;

uint64_t t1, t2, t3, t4, t5, t6;
uint64_t mask = 0xFFFFFFFFFF;
int32_t tof;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
uint32_t timeout;

void setup() {

  uint8_t buf[2];

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

  // Set RX buffer
  decaduino.setRxBuffer(rxData, &rxLen);
  state = TWR_ENGINE_STATE_INIT;
}

void loop() {
  
  switch (state) {
  
    case TWR_ENGINE_STATE_INIT :
      decaduino.plmeRxDisableRequest();
			delay(1000);
      Serial.println("New SDS_TWR");
      txData[0] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 1);
      state = TWR_ENGINE_STATE_WAIT_START_SENT;
      break;
      
    case TWR_ENGINE_STATE_WAIT_START_SENT :
      if (decaduino.hasTxSucceeded()) {
      	state = TWR_ENGINE_STATE_MEMORISE_T1;
      }
      break;
       
    case TWR_ENGINE_STATE_MEMORISE_T1 :
      t1 = decaduino.lastTxTimestamp;
      timeout = millis() + TIMEOUT;
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_WAIT_ACK_REQ :
      if ( millis() > timeout) {
      	state = TWR_ENGINE_STATE_INIT;
      }
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == TWR_MSG_TYPE_ACK_REQ) {
          	state = TWR_ENGINE_STATE_MEMORISE_T4;
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = TWR_ENGINE_STATE_WAIT_ACK_REQ;
          }
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
      	state = TWR_ENGINE_STATE_MEMORISE_T5;
      }
      break;
      
    case TWR_ENGINE_STATE_MEMORISE_T5 :
      t5 = decaduino.lastTxTimestamp;
      timeout = millis() + TIMEOUT;
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_WAIT_DATA_REPLY :
      if ( millis() > timeout) {
      	state = TWR_ENGINE_STATE_INIT;
      }
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[0] == TWR_MSG_TYPE_DATA_REPLY) {
          state = TWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
          	}
        }
      }
      break;
       
    case TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 :
      t2 = decaduino.decodeUint64(&rxData[1]);
      t3 = decaduino.decodeUint64(&rxData[9]);
      t6 = decaduino.decodeUint64(&rxData[17]);
      tof = (((((t4 - t1) & mask) - ((t3 - t2) & mask)) & mask) + ((((t6 - t3) & mask) - ((t5 - t4) & mask)) & mask))/4;
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
