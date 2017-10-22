#define ENABLE_CALIBRATION_FROM_EEPROM

#include <SPI.h>
#include <DecaDuino.h>
#ifdef ENABLE_CALIBRATION_FROM_EEPROM
#include <EEPROM.h>
uint16_t antennaDelay;
#endif

#define COEFF RANGING_UNIT

#define TIMEOUT_WAIT_ACK 20
#define TIMEOUT_WAIT_DATA_REPLY 20

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_WAIT_START_SENT 2
#define TWR_ENGINE_STATE_MEMORISE_T1 3
#define TWR_ENGINE_STATE_WAIT_ACK 4
#define TWR_ENGINE_STATE_MEMORISE_T4 5
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 6
#define TWR_ENGINE_STATE_EXTRACT_T2_T3 7

#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK 2
#define TWR_MSG_TYPE_DATA_REPLY 3

int i;
int rxFrames;

uint64_t t1, t2, t3, t4;
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

  float distance;
  
  switch (state) {
   
    case TWR_ENGINE_STATE_INIT:
      decaduino.plmeRxDisableRequest();
      delay(1000);
      //delay(100);
      Serial.println("New TWR");
      txData[0] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 1);
      state = TWR_ENGINE_STATE_WAIT_START_SENT;
      break;
      
    case TWR_ENGINE_STATE_WAIT_START_SENT:
      if ( decaduino.hasTxSucceeded() ) {
        state = TWR_ENGINE_STATE_MEMORISE_T1;
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T1:
      t1 = decaduino.lastTxTimestamp;
      timeout = millis() + TIMEOUT_WAIT_ACK;
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
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = TWR_ENGINE_STATE_WAIT_ACK;
          }
        }
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T4:
      t4 = decaduino.lastRxTimestamp;
      timeout = millis() + TIMEOUT_WAIT_DATA_REPLY;
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
          } else {
          	decaduino.plmeRxEnableRequest();
          	state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
          }
        }
      }
      break;

    case TWR_ENGINE_STATE_EXTRACT_T2_T3:
      t2 = decaduino.decodeUint64(&rxData[1]);
      t3 = decaduino.decodeUint64(&rxData[9]);
      tof = (((t4 - t1) & mask) - (1+1.0E-6*decaduino.getLastRxSkew())*((t3 - t2) & mask))/2;
      distance = tof*COEFF;
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


