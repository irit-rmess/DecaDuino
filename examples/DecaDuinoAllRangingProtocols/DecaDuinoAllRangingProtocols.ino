//#define ENABLE_CALIBRATION_FROM_EEPROM
//#define ENABLE_LABEL_FROM_EEPROM

#include <SPI.h>
#include <DecaDuino.h>
#if defined(ENABLE_CALIBRATION_FROM_EEPROM) || defined(ENABLE_LABEL_FROM_EEPROM)
#include <EEPROM.h>
uint16_t antennaDelay;
#endif 

uint16_t label, label2;
int role, protocol;
int inter_cycle_delay;
int rxA_enable_between_cycles;

#define EXPERIMENTATION_DURATION 8 // seconds

#define COEFF RANGING_UNIT

#define TIMEOUT 10

#define RGB_RED_PIN 5
#define RGB_GREEN_PIN 4
#define RGB_BLUE_PIN 3


// TWR_ENGINE_STATE for node 'A'
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

// TWR_ENGINE_STATE for node 'B'
#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_RX_ON 2
#define TWR_ENGINE_STATE_WAIT_START 3
#define TWR_ENGINE_STATE_MEMORISE_T2 4
#define TWR_ENGINE_STATE_SEND_ACK 5
#define TWR_ENGINE_STATE_WAIT_SENT 6
#define TWR_ENGINE_STATE_MEMORISE_T3 7
#define TWR_ENGINE_STATE_WAIT_BEFORE_SEND_DATA_REPLY 8
#define TWR_ENGINE_STATE_SEND_DATA_REPLY 9 

// SDSTWR_ENGINE_STATE for node 'A'
#define SDSTWR_ENGINE_STATE_INIT 1
#define SDSTWR_ENGINE_STATE_WAIT_NEW_CYCLE 2
#define SDSTWR_ENGINE_STATE_SEND_START 3
#define SDSTWR_ENGINE_STATE_WAIT_START_SENT 4
#define SDSTWR_ENGINE_STATE_MEMORISE_T1 5
#define SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ 6
#define SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ 7
#define SDSTWR_ENGINE_STATE_WAIT_ACK_REQ 8
#define SDSTWR_ENGINE_STATE_MEMORISE_T4 9
#define SDSTWR_ENGINE_STATE_SEND_ACK 10 
#define SDSTWR_ENGINE_STATE_WAIT_ACK_SENT 11
#define SDSTWR_ENGINE_STATE_MEMORISE_T5 12
#define SDSTWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY 13
#define SDSTWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY 14
#define SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY 15
#define SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6 16

// SDSTWR_ENGINE_STATE for node 'B'
#define SDSTWR_ENGINE_STATE_INIT 1
#define SDSTWR_ENGINE_STATE_RX_ON 2
#define SDSTWR_ENGINE_STATE_WAIT_START 3
#define SDSTWR_ENGINE_STATE_MEMORISE_T2 4
#define SDSTWR_ENGINE_STATE_SEND_ACK_REQ 5
#define SDSTWR_ENGINE_STATE_WAIT_SENT 6
#define SDSTWR_ENGINE_STATE_MEMORISE_T3 7
#define SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK 8
#define SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK 9
#define SDSTWR_ENGINE_STATE_WAIT_ACK 10
#define SDSTWR_ENGINE_STATE_MEMORISE_T6 11
#define SDSTWR_ENGINE_STATE_SEND_DATA_REPLY 12

// TWR messages
#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 52
#define TWR_MSG_TYPE_ACK 53
#define TWR_MSG_TYPE_DATA_REPLY 54

// SDSTWR messages
#define SDSTWR_MSG_TYPE_UNKNOW 0
#define SDSTWR_MSG_TYPE_START 55
#define SDSTWR_MSG_TYPE_ACK_REQ 56
#define SDSTWR_MSG_TYPE_ACK 57
#define SDSTWR_MSG_TYPE_DATA_REPLY 58

int i;
int rxFrames;

uint64_t t1, t2, t3, t4, t5, t6;
int64_t tof;
int64_t tof_skew;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
uint32_t timeout;

void setup() {

  uint8_t buf[2];

  pinMode(13, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  digitalWrite(RGB_RED_PIN, HIGH);
  digitalWrite(RGB_GREEN_PIN, LOW);
  digitalWrite(RGB_BLUE_PIN, HIGH);

  while(Serial.available()==0);
  Serial.setTimeout(86400);
  label=Serial.parseInt();
  label2=Serial.parseInt();
  protocol=Serial.parseInt();
  role=Serial.parseInt();
  inter_cycle_delay=Serial.parseInt();
  rxA_enable_between_cycles=Serial.parseInt();

  Serial.printf("My label is 0x%04X, my role is %d and I range with 0x%04X using protocol %d, with inter_cycle_delay=%d and rxA_enable_between_cycles=%d\n", label, role, label2, protocol, inter_cycle_delay, rxA_enable_between_cycles);

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

#ifdef ENABLE_LABEL_FROM_EEPROM

  // Gets DecaWiNo label from the end of EEPROM (the two last bytes)
  buf[0] = EEPROM.read(EEPROM.length()-2);
  buf[1] = EEPROM.read(EEPROM.length()-1);
  label = decaduino.decodeUint16(buf);

  if ( label == 0xffff ) {
    Serial.println("Unvalid label value found in EEPROM. Using default value (0x0000).");
    label = 0;
  } else Serial.printf("My label is 0x%04X\n", label);

#endif

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

  switch (protocol) {
    case 1: state=TWR_ENGINE_STATE_INIT; break;
    case 2: state=SDSTWR_ENGINE_STATE_INIT; break;
    default: break;
  }

  Serial.println("RUN");
}


void loop() {

  decaduino.engine();

  if ( millis() > EXPERIMENTATION_DURATION*1000 ) {
    Serial.println("END");
    digitalWrite(RGB_RED_PIN, HIGH);
    digitalWrite(RGB_GREEN_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, HIGH);
    while (1);
  }

  switch (protocol) {
    case 1: twrEngine(); break;
    case 2: sdstwrEngine(); break;
    default: break;
  }
}


void twrEngine ( void ) {

  switch (role) {
    
    case 1:
      switch (state) {
       
        case TWR_ENGINE_STATE_INIT:
          digitalWrite(RGB_RED_PIN, LOW);
          digitalWrite(RGB_GREEN_PIN, HIGH);
          digitalWrite(RGB_BLUE_PIN, HIGH);
          decaduino.plmeRxDisableRequest();
          state = TWR_ENGINE_STATE_WAIT_NEW_CYCLE;
          break;
          
        case TWR_ENGINE_STATE_WAIT_NEW_CYCLE:
          if ( rxA_enable_between_cycles == 1 ) {
            decaduino.plmeRxEnableRequest();
          }
          delay(inter_cycle_delay);
          decaduino.plmeRxDisableRequest();
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
            Serial.println("ACK not received");
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
            Serial.println("DATA_REPLY not received");
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
          tof_skew = (t4 - t1 - (1+1.0E-6*decaduino.getLastRxSkew())*(t3 - t2))/2;
          //Serial.printf("New timestamps,0x%04X,0x%04X,%d,%d,%f,%lld,%lld,%lld,%lld\n",label, label2, protocol, role, decaduino.getLastRxSkew(), t1, t2, t3, t4);
          Serial.printf("New timestamps_hex,0x%04X,0x%04X,%d,%d,%f,",label, label2, protocol, role, decaduino.getLastRxSkew());
          decaduino.printUint64(t1);
          Serial.print(",");
          decaduino.printUint64(t2);
          Serial.print(",");
          decaduino.printUint64(t3);
          Serial.print(",");
          decaduino.printUint64(t4);
          Serial.printf(",%d,%d,%f,%d",rxA_enable_between_cycles, inter_cycle_delay, decaduino.getNLOSIndication(),decaduino.getTemperatureRaw());
          Serial.println();
          Serial.printf("New ranging,0x%04X,0x%04X,%f,-1,%d,%d,-1,%f,%f,%d\n",label, label2, tof*COEFF, /*tof, */protocol, role, /*tof_skew, */decaduino.getLastRxSkew(), decaduino.getNLOSIndication(),decaduino.getTemperatureRaw());
          state = TWR_ENGINE_STATE_INIT;
          break;
    
        default:
          state = TWR_ENGINE_STATE_INIT;
          break;
      }
      break;

    case 2:
      switch (state) {
       
        case TWR_ENGINE_STATE_INIT:
          digitalWrite(RGB_RED_PIN, HIGH);
          digitalWrite(RGB_GREEN_PIN, HIGH);
          digitalWrite(RGB_BLUE_PIN, LOW);
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
          decaduino.pdDataRequest(txData, 1);
          state = TWR_ENGINE_STATE_WAIT_SENT;
          break;
    
        case TWR_ENGINE_STATE_WAIT_SENT:
          if ( decaduino.hasTxSucceeded() )
            state = TWR_ENGINE_STATE_MEMORISE_T3;  
          break;
    
        case TWR_ENGINE_STATE_MEMORISE_T3:
          t3 = decaduino.lastTxTimestamp;
          state = TWR_ENGINE_STATE_WAIT_BEFORE_SEND_DATA_REPLY;
          break;
    
        case TWR_ENGINE_STATE_WAIT_BEFORE_SEND_DATA_REPLY:
          delay(1);
          state = TWR_ENGINE_STATE_SEND_DATA_REPLY;
          break;
    
        case TWR_ENGINE_STATE_SEND_DATA_REPLY:
          txData[0] = TWR_MSG_TYPE_DATA_REPLY;
          decaduino.encodeUint64(t2, &txData[1]);
          decaduino.encodeUint64(t3, &txData[9]);
          decaduino.pdDataRequest(txData, 17);
          state = TWR_ENGINE_STATE_INIT;
          break;
     
        default:
          state = TWR_ENGINE_STATE_INIT;
          break;
      }
      break;
  }
}


void sdstwrEngine ( void ) {

  switch (role) {
    
    case 1:
      switch (state) {
      
        case SDSTWR_ENGINE_STATE_INIT :
          digitalWrite(RGB_RED_PIN, LOW);
          digitalWrite(RGB_GREEN_PIN, HIGH);
          digitalWrite(RGB_BLUE_PIN, HIGH);
          decaduino.plmeRxDisableRequest();
          state = SDSTWR_ENGINE_STATE_WAIT_NEW_CYCLE;
          break;
        
        case SDSTWR_ENGINE_STATE_WAIT_NEW_CYCLE :
          if ( rxA_enable_between_cycles == 1 ) {
            decaduino.plmeRxEnableRequest();
          }
          delay(inter_cycle_delay);
          decaduino.plmeRxDisableRequest();
          Serial.println("New SDS_TWR");
          state = SDSTWR_ENGINE_STATE_SEND_START;
          break;
        
        case SDSTWR_ENGINE_STATE_SEND_START :
          txData[0] = SDSTWR_MSG_TYPE_START;
          decaduino.pdDataRequest(txData, 1);
          state = SDSTWR_ENGINE_STATE_WAIT_START_SENT;
          break;
           
        case SDSTWR_ENGINE_STATE_WAIT_START_SENT :
          if (decaduino.hasTxSucceeded())
          state = SDSTWR_ENGINE_STATE_MEMORISE_T1;
          break;
           
        case SDSTWR_ENGINE_STATE_MEMORISE_T1 :
          t1 = decaduino.lastTxTimestamp;
          state = SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ;
          break;
           
        case SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ :
          timeout = millis() + TIMEOUT;
          state = SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
          break;
           
        case SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ :
          decaduino.plmeRxEnableRequest();
          state = SDSTWR_ENGINE_STATE_WAIT_ACK_REQ;
          break;
           
        case SDSTWR_ENGINE_STATE_WAIT_ACK_REQ :
          if ( millis() > timeout) {
            Serial.println("ACK not received");
            state = SDSTWR_ENGINE_STATE_INIT;
          }
          else { 
            if (decaduino.rxFrameAvailable()){
              if (rxData[0] == SDSTWR_MSG_TYPE_ACK_REQ) {
              state = SDSTWR_ENGINE_STATE_MEMORISE_T4;
              } else state = SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
            }
          }
          break;
           
        case SDSTWR_ENGINE_STATE_MEMORISE_T4 :
          t4 = decaduino.lastRxTimestamp;
          state = SDSTWR_ENGINE_STATE_SEND_ACK;
          break;
           
        case SDSTWR_ENGINE_STATE_SEND_ACK :
          txData[0]= SDSTWR_MSG_TYPE_ACK;
          decaduino.pdDataRequest(txData, 1);
          state = SDSTWR_ENGINE_STATE_WAIT_ACK_SENT;
          break;
           
        case SDSTWR_ENGINE_STATE_WAIT_ACK_SENT :
          if (decaduino.hasTxSucceeded()){
          state = SDSTWR_ENGINE_STATE_MEMORISE_T5;}
          break;
          
        case SDSTWR_ENGINE_STATE_MEMORISE_T5 :
          t5 = decaduino.lastTxTimestamp;
          state = SDSTWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY;
          break;
           
        case SDSTWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY :
          timeout = millis() + TIMEOUT;
          state = SDSTWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
          break;
           
        case SDSTWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY :
          decaduino.plmeRxEnableRequest();
          state = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY;
          break;
           
        case SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY :
          if ( millis() > timeout) {
            state = SDSTWR_ENGINE_STATE_INIT;
            Serial.println("DATA_REPLY not received");
          }
          else { 
            if (decaduino.rxFrameAvailable()){
              if (rxData[0] == SDSTWR_MSG_TYPE_DATA_REPLY) {
              state = SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
              } else state = SDSTWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
            }
          }
          break;
           
        case SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6 :
          t2 = decaduino.decodeUint64(&rxData[1]);
          t3 = decaduino.decodeUint64(&rxData[9]);
          t6 = decaduino.decodeUint64(&rxData[17]);
          tof = (2*t4 - t1 - 2*t3 + t2 + t6 - t5)/4;
          tof_skew = ( (t4-t1) - (t5-t4) - (1+1.0E-6*decaduino.getLastRxSkew())*((t3-t2)+(t6-t3)) )/4;
          //tof = (2*t4 - t1 - 2*t3 + t2 + t6 - t5)/4;
          //tof_skew = (t4 - t1 - (1+1.0E-6*decaduino.getLastRxSkew())*(t3 - t2))/2;
          // Serial.printf("New ranging,0x%04X,0x%04X,%f,%ld,%d,%d\n",label, label2, tof*COEFF, (int32_t)tof, protocol, role, (int32_t)tof_skew, decaduino.getLastRxSkew());
          //Serial.printf("New timestamps,0x%04X,0x%04X,%d,%d,%f,%lld,%lld,%lld,%lld,%lld,%lld\n",label, label2, protocol, role, decaduino.getLastRxSkew(), t1, t2, t3, t4, t5, t6);
          Serial.printf("New timestamps_hex,0x%04X,0x%04X,%d,%d,%f,",label, label2, protocol, role, decaduino.getLastRxSkew());
          decaduino.printUint64(t1);
          Serial.print(",");
          decaduino.printUint64(t2);
          Serial.print(",");
          decaduino.printUint64(t3);
          Serial.print(",");
          decaduino.printUint64(t4);
          Serial.print(",");
          decaduino.printUint64(t5);
          Serial.print(",");
          decaduino.printUint64(t6);
          Serial.printf(",%d,%d,%f,%d",rxA_enable_between_cycles, inter_cycle_delay,decaduino.getNLOSIndication(),decaduino.getTemperatureRaw());
          Serial.println();
          Serial.printf("New ranging,0x%04X,0x%04X,%f,-1,%d,%d,-1,%f,%f,%d\n",label, label2, tof*COEFF, /*tof, */protocol, role, /*tof_skew, */decaduino.getLastRxSkew(), decaduino.getNLOSIndication(),decaduino.getTemperatureRaw());
          //Serial.printf("New ranging,0x%04X,0x%04X,%f,%ld,%d,%d\n",label, label2, tof*COEFF, (int32_t)tof, protocol, role, tof*COEFF);
          state = SDSTWR_ENGINE_STATE_INIT;
          break;
           
        default:
          state = SDSTWR_ENGINE_STATE_INIT;
          break;
      }
      break;

    case 2:
      switch (state) {
      
        case SDSTWR_ENGINE_STATE_INIT :
           digitalWrite(RGB_RED_PIN, HIGH);
           digitalWrite(RGB_GREEN_PIN, HIGH);
           digitalWrite(RGB_BLUE_PIN, LOW);
           state = SDSTWR_ENGINE_STATE_RX_ON;
           break;
           
        case SDSTWR_ENGINE_STATE_RX_ON :
           decaduino.plmeRxEnableRequest();
           state = SDSTWR_ENGINE_STATE_WAIT_START;
           break;
           
        case SDSTWR_ENGINE_STATE_WAIT_START :
           if (decaduino.rxFrameAvailable()){
             if ( rxData[0] == SDSTWR_MSG_TYPE_START){
                state = SDSTWR_ENGINE_STATE_MEMORISE_T2;
             } else state = SDSTWR_ENGINE_STATE_RX_ON; 
           }
           break;
           
        case SDSTWR_ENGINE_STATE_MEMORISE_T2 :
           t2 = decaduino.lastRxTimestamp;
           state = SDSTWR_ENGINE_STATE_SEND_ACK_REQ;
           break;
             
        case SDSTWR_ENGINE_STATE_SEND_ACK_REQ : 
           txData[0]= SDSTWR_MSG_TYPE_ACK_REQ;
           decaduino.pdDataRequest(txData, 1);
           state = SDSTWR_ENGINE_STATE_WAIT_SENT;
           break;
           
        case SDSTWR_ENGINE_STATE_WAIT_SENT:
           if (decaduino.hasTxSucceeded()){
          state = SDSTWR_ENGINE_STATE_MEMORISE_T3; }
           break;
           
        case SDSTWR_ENGINE_STATE_MEMORISE_T3 :
           t3 = decaduino.lastTxTimestamp;
           state = SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK;
           break;
           
        case SDSTWR_ENGINE_STATE_WATCHDOG_FOR_ACK :
           timeout = millis() + TIMEOUT;
           state = SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK;
           break;
           
        case SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK :
           decaduino.plmeRxEnableRequest();
           state = SDSTWR_ENGINE_STATE_WAIT_ACK;
           break;
           
        case SDSTWR_ENGINE_STATE_WAIT_ACK :
           if ( millis() > timeout) {
             state = SDSTWR_ENGINE_STATE_INIT;
             Serial.println("ACK not received");
           }
           else { 
             if (decaduino.rxFrameAvailable()){
               if (rxData[0] == SDSTWR_MSG_TYPE_ACK) {
               state = SDSTWR_ENGINE_STATE_MEMORISE_T6;
               } else state = SDSTWR_ENGINE_STATE_RX_ON_FOR_ACK;
             }
           }
           break;
           
        case SDSTWR_ENGINE_STATE_MEMORISE_T6 :
           t6 = decaduino.lastRxTimestamp;
           state = SDSTWR_ENGINE_STATE_SEND_DATA_REPLY; 
           break;
           
        case SDSTWR_ENGINE_STATE_SEND_DATA_REPLY :
           txData[0] = SDSTWR_MSG_TYPE_DATA_REPLY;
           decaduino.encodeUint64(t2, &txData[1]);
           decaduino.encodeUint64(t3, &txData[9]);
           decaduino.encodeUint64(t6, &txData[17]);
           decaduino.pdDataRequest(txData, 25);
           state = SDSTWR_ENGINE_STATE_INIT;
           break;
           
        default :
          state = SDSTWR_ENGINE_STATE_INIT;
          break;  
      }
      break;

    default:
      break;      
  }
}

