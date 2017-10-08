//#define ENABLE_CALIBRATION_FROM_EEPROM
//#define ENABLE_LABEL_FROM_EEPROM

#include <math.h>
#include <SPI.h>
#include <DecaDuino.h>
#if defined(ENABLE_CALIBRATION_FROM_EEPROM) || defined(ENABLE_LABEL_FROM_EEPROM)
#include <EEPROM.h>
uint16_t antennaDelay;
#endif 

#define RANGING_DEBUG 1   // 1 to print debugging messages else 0

bool rangingDebug = RANGING_DEBUG;

uint16_t label, label2;
uint16_t broadcast = 0xFFFF;

uint8_t protocol;
int inter_cycle_delay;
int rxA_enable_between_cycles;

uint8_t sqn = 0;
uint8_t sqn_ack;
bool isAvailable = false;

uint8_t src[2];
uint8_t dst[2];

// Node specifications
uint8_t node_status;
float node_x;
float node_y;
float node_z;
float node_eAR;
uint8_t node_protocols_available;

typedef struct Neighb Neighb;

struct Neighb {
  uint16_t neighb_label;
  uint8_t Status;
  float x;
  float y;
  float z;
  float eAR;
  uint8_t protocols_available;
};

uint32_t printNeighbTimeout;
uint32_t rangeNeighbTimeout;

#define PRINT_NEIGHB_PERIOD 3 // seconds
#define RANGE_NEIGHB_PERIOD 1 // seconds

#define MAX_NEIGHB 100

Neighb neighb_table[MAX_NEIGHB];

#define EXPERIMENTATION_DURATION 500 // seconds

#define COEFF RANGING_UNIT

#define TIMEOUT 50
#define TIMEOUT2 40

#define RGB_RED_PIN 5
#define RGB_GREEN_PIN 4
#define RGB_BLUE_PIN 3

#define NEIGHB_POS 0
#define RANGING_MSG 1

#define TWR_PROTOCOL 0
#define SDSTWR_PROTOCOL 1

#define RANGING_ENGINE_STATE_IDLE 1
#define TWR_ENGINE_STATE_IDLE 1
#define SDSTWR_ENGINE_STATE_IDLE 1

// RANGING_ENGINE_STATE
#define RANGING_ENGINE_STATE_SEND_RANGING_REQUEST 2
#define RANGING_ENGINE_STATE_WAIT_RANGING_REQUEST_SENT 3

// TWR_ENGINE_STATE for node 'CLIENT'
#define TWR_ENGINE_STATE_SEND_START 2
#define TWR_ENGINE_STATE_MEMORISE_T1 3
#define TWR_ENGINE_STATE_WAIT_ACK 4
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 5
#define TWR_ENGINE_STATE_EXTRACT_T2_T3 6
#define TWR_ENGINE_STATE_SEND_ACK_CLIENT 7
#define TWR_ENGINE_STATE_WAIT_ACK_SENT_CLIENT 8

// TWR_ENGINE_STATE for node 'SERVER'
#define TWR_ENGINE_STATE_INIT_SERVER 9
#define TWR_ENGINE_STATE_WAIT_START 10
#define TWR_ENGINE_STATE_SEND_ACK_SERVER 11
#define TWR_ENGINE_STATE_WAIT_ACK_SENT_SERVER 12
#define TWR_ENGINE_STATE_MEMORISE_T3 13
#define TWR_ENGINE_STATE_SEND_DATA_REPLY 14
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT 15
#define TWR_ENGINE_STATE_WAIT_ACK_SERVER 16

// SDSTWR_ENGINE_STATE for node 'CLIENT'
#define SDSTWR_ENGINE_STATE_SEND_START 2
#define SDSTWR_ENGINE_STATE_MEMORISE_T1 3
#define SDSTWR_ENGINE_STATE_WAIT_ACK_CLIENT 4
#define SDSTWR_ENGINE_STATE_SEND_ACK_CLIENT 5
#define SDSTWR_ENGINE_STATE_MEMORISE_T5 6
#define SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY 7
#define SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6 8

// SDSTWR_ENGINE_STATE for node 'SERVER'
#define SDSTWR_ENGINE_STATE_INIT_SERVER 9
#define SDSTWR_ENGINE_STATE_WAIT_START 10
#define SDSTWR_ENGINE_STATE_SEND_ACK_SERVER 11
#define SDSTWR_ENGINE_STATE_MEMORISE_T3 12
#define SDSTWR_ENGINE_STATE_WAIT_ACK_SERVER 13
#define SDSTWR_ENGINE_STATE_SEND_DATA_REPLY 14
#define SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT 15

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

#define RX_ON 1           // Waiting for a specific frame
#define RX_DONE 0         // Correct frame received
#define TX_ON  1          // A frame needs to be sent
#define TX_SENDING 0      // pdDataRequest
#define TX_END_ON 1       // Waiting for the frame to be sent
#define TX_SUCCEEDED 0    // Frame has been sent

int tx = 0;
int txEnd = 0;
int rx = RX_ON;

int i;
int rxFrames;

uint64_t t1, t2, t3, t4, t5, t6;
uint64_t mask = 0xFFFFFFFFFF;
int32_t tof;
int32_t tof_skew;

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;

int state_ranging;
int state_twr;
int state_sdstwr;

#define NEIGHB_POS_PERIOD 10 // seconds

uint32_t timeout;
uint32_t neighb_timeout;

int nbData;
int msg_type;
bool ranging = false;
bool rangingReq = false;


void macEngine() {

  if ( tx == TX_ON ) {
    // A frame need to be sent
    rx = RX_DONE;
    decaduino.plmeRxDisableRequest();
    decaduino.pdDataRequest(txData, nbData);
    tx = TX_SENDING;
    txEnd = TX_END_ON;
  }

  if ( txEnd == TX_END_ON ) {
    // Checks if the frame has been sent
    if ( decaduino.hasTxSucceeded()) {
      txEnd = TX_SUCCEEDED;
      if ( txData[5] == NEIGHB_POS ) {
        rx = RX_ON;
      }
      decaduino.plmeRxEnableRequest();
    }
  }

  if ( rx == RX_ON ) {
    if ( decaduino.rxFrameAvailable()) {
      dst[0] = rxData[2];
      dst[1] = rxData[3];
      if ( decaduino.decodeUint16(dst) == label ) {
        src[0] = rxData[0];
        src[1] = rxData[1];
        msg_type = rxData[7];
        if (( !rangingReq ) && ( !ranging ) && ( rxData[5] == RANGING_MSG )) {
          protocol = rxData[6];
          rangingResponse();
          rx = RX_DONE;
        } else if ( ranging ) {
          if ( decaduino.decodeUint16(src) == label2 ) {
            rx = RX_DONE;
          }
        }
      } else if (( !rangingReq ) && ( !ranging ) && ( rxData[5] == NEIGHB_POS )) {
        src[0] = rxData[0];
        src[1] = rxData[1];
        neighbSave();
      }
    decaduino.plmeRxEnableRequest();
    }
  }
}


void rangingEngine() {
  if ( rangingReq) {
    switch (state_ranging) {

      case RANGING_ENGINE_STATE_IDLE:
        break;
  
      case RANGING_ENGINE_STATE_SEND_RANGING_REQUEST:
        decaduino.encodeUint16(label, &txData[0]);
        decaduino.encodeUint16(label2, &txData[2]);
        txData[4] = sqn;
        txData[5] = RANGING_MSG;
        txData[6] = protocol;
        nbData = 7;
        tx = TX_ON;
        state_ranging = RANGING_ENGINE_STATE_WAIT_RANGING_REQUEST_SENT;
        break;
  
      case RANGING_ENGINE_STATE_WAIT_RANGING_REQUEST_SENT:
        if (( txEnd == TX_SUCCEEDED )) {
          if ( rangingDebug ) {
            Serial.println("RANGING REQUEST SENT");
          }
          rangingReq = false;
          state_ranging = RANGING_ENGINE_STATE_IDLE;
          if ( protocol == TWR_PROTOCOL ) {
            ranging = true;
            state_twr = TWR_ENGINE_STATE_SEND_START;
          } else if ( protocol == SDSTWR_PROTOCOL ) {
            ranging = true;
            state_sdstwr = SDSTWR_ENGINE_STATE_SEND_START;
          }
        }
        break;  
  
      default:
        state_ranging = RANGING_ENGINE_STATE_IDLE;
        break;
    }
  }
}


void twrEngine() {

  switch (state_twr) {
    
    case TWR_ENGINE_STATE_IDLE:
      break;
      
    case TWR_ENGINE_STATE_SEND_START:
      if ( rangingDebug ) {
        Serial.println("SEND START");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn;
      txData[5] = RANGING_MSG;
      txData[6] = TWR_PROTOCOL;
      txData[7] = TWR_MSG_TYPE_START;
      nbData = 8;
      tx = TX_ON;
      state_twr = TWR_ENGINE_STATE_MEMORISE_T1;
      break;

    case TWR_ENGINE_STATE_MEMORISE_T1:
      if (( txEnd == TX_SUCCEEDED )) {
        if ( rangingDebug ) {
          Serial.println("START SENT AND MEMO T1");
        }
        t1 = decaduino.lastTxTimestamp;
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        state_twr = TWR_ENGINE_STATE_WAIT_ACK;
      }
      break;

    case TWR_ENGINE_STATE_WAIT_ACK:
      if ( millis() > timeout ) {
        state_twr = TWR_ENGINE_STATE_SEND_START;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == TWR_MSG_TYPE_ACK )) {
          if ( rangingDebug ) {
            Serial.println("ACK RECEIVED & MEMO T4");
          }
          sqn++;
          t4 = decaduino.lastRxTimestamp;
          timeout = millis() + TIMEOUT2;
          rx = RX_ON;
          state_twr = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
        }
      }
      break; 

    case TWR_ENGINE_STATE_WAIT_DATA_REPLY:
      if ( millis() > timeout ) {
        state_twr = TWR_ENGINE_STATE_SEND_START;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == TWR_MSG_TYPE_DATA_REPLY )) {
          if ( rangingDebug ) {
            Serial.println("DATA REPLY RECEIVED");
          }
          sqn_ack = rxData[4] + 1;
          state_twr = TWR_ENGINE_STATE_EXTRACT_T2_T3;
        }
      }
      break;

    case TWR_ENGINE_STATE_EXTRACT_T2_T3:
      if ( rangingDebug ) {
        Serial.println("EXTRACT T2 T3");
      }
      t2 = decaduino.decodeUint64(&rxData[8]);
      t3 = decaduino.decodeUint64(&rxData[16]);
      tof_skew = ((( t4 - t1 ) & mask ) - ( 1 + 1.0E-6*decaduino.getLastRxSkew()) * (( t3 - t2 ) & mask ))/2;
      Serial.print("ToF=");
      Serial.print(tof_skew, HEX);
      Serial.print(" d=");
      Serial.print(tof_skew*COEFF);
      Serial.println();
      if (tof_skew*COEFF < 0) {
        Serial.println("ERROR NEGATIVE VALUE FOR DISTANCE");
      }
      state_twr = TWR_ENGINE_STATE_SEND_ACK_CLIENT;
      break; 
      
    case TWR_ENGINE_STATE_SEND_ACK_CLIENT: 
      if ( rangingDebug ) { 
        Serial.println("SEND ACK TO SERVER");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn_ack;
      txData[5] = RANGING_MSG;
      txData[6] = TWR_PROTOCOL;
      txData[7] = TWR_MSG_TYPE_ACK;
      tx = TX_ON;
      state_twr = TWR_ENGINE_STATE_WAIT_ACK_SENT_CLIENT;
      break;

    case TWR_ENGINE_STATE_WAIT_ACK_SENT_CLIENT:
      if ( txEnd == TX_SUCCEEDED ) {
        if ( rangingDebug ) {
          Serial.println("ACK SENT");
        }
        ranging = false;
        rx = RX_ON;
        state_twr = TWR_ENGINE_STATE_IDLE;
      }
      break;

    
    case TWR_ENGINE_STATE_INIT_SERVER:
      if ( rangingDebug ) {
        Serial.println("TWR SERVER");
      }
      rx = RX_ON;
      state_twr = TWR_ENGINE_STATE_WAIT_START;
      break;

    case TWR_ENGINE_STATE_WAIT_START:
      if (( rx == RX_DONE ) && ( msg_type == TWR_MSG_TYPE_START )) {
        if ( rangingDebug ) {
          Serial.println("START RECEIVED & MEMO T2");
        }
        sqn_ack = rxData[4] + 1;
        t2 = decaduino.lastRxTimestamp;
        state_twr = TWR_ENGINE_STATE_SEND_ACK_SERVER;
      }
      break;
      
    case TWR_ENGINE_STATE_SEND_ACK_SERVER:
      if ( rangingDebug ) {
        Serial.println("SEND ACK TO CLIENT");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn_ack;
      txData[5] = RANGING_MSG;
      txData[6] = TWR_PROTOCOL;
      txData[7] = TWR_MSG_TYPE_ACK;
      nbData = 8;
      tx = TX_ON;
      state_twr = TWR_ENGINE_STATE_MEMORISE_T3;
      break;

    case TWR_ENGINE_STATE_MEMORISE_T3:
      if ( txEnd == TX_SUCCEEDED) {
        if ( rangingDebug ) {
          Serial.println("ACK SENT AND MEMO T3");
        }
        t3 = decaduino.lastTxTimestamp;
        state_twr = TWR_ENGINE_STATE_SEND_DATA_REPLY;
      }
      break;

    case TWR_ENGINE_STATE_SEND_DATA_REPLY:
      if ( rangingDebug ) {
        Serial.println("SEND DATA REPLY");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn;
      txData[5] = RANGING_MSG;
      txData[6] = TWR_PROTOCOL;
      txData[7] = TWR_MSG_TYPE_DATA_REPLY;
      decaduino.encodeUint64(t2, &txData[8]);
      decaduino.encodeUint64(t3, &txData[16]);
      nbData = 24;
      tx = TX_ON;
      state_twr = TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT;
      break;

    case TWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT:
      if (( txEnd == TX_SUCCEEDED )) {
        if ( rangingDebug ) {
          Serial.println("DATA REPLY SENT");
        }
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        state_twr = TWR_ENGINE_STATE_WAIT_ACK_SERVER;
      }
      break;

    case TWR_ENGINE_STATE_WAIT_ACK_SERVER:
      if ( millis() > timeout ) {
        state_twr = TWR_ENGINE_STATE_INIT_SERVER;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == TWR_MSG_TYPE_ACK )) {
          if ( rangingDebug ) {
            Serial.println("ACK RECEIVED FROM CLIENT");
          }
          sqn++;
          ranging = false;
          rx = RX_ON;
          state_twr = TWR_ENGINE_STATE_IDLE;
        }
      }
      break;
  
    default:
      state_twr = TWR_ENGINE_STATE_IDLE;
      break;
  }
}


void sdstwrEngine() {

  switch (state_sdstwr) {

    case SDSTWR_ENGINE_STATE_IDLE:
      break;

    case SDSTWR_ENGINE_STATE_SEND_START:
      if ( rangingDebug ) {
        Serial.println("SEND START");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn;
      txData[5] = RANGING_MSG;
      txData[6] = SDSTWR_PROTOCOL;
      txData[7] = SDSTWR_MSG_TYPE_START;
      nbData = 8;
      tx = TX_ON;
      state_sdstwr = TWR_ENGINE_STATE_MEMORISE_T1;
      break;
    
    case SDSTWR_ENGINE_STATE_MEMORISE_T1:
      if (( txEnd == TX_SUCCEEDED )) {
        if ( rangingDebug ) {
          Serial.println("START SENT AND MEMO T1");
        }
        t1 = decaduino.lastTxTimestamp;
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        state_sdstwr = SDSTWR_ENGINE_STATE_WAIT_ACK_CLIENT;
      }   
      break;

    case SDSTWR_ENGINE_STATE_WAIT_ACK_CLIENT:
      if ( millis() > timeout ) {
        state_sdstwr = SDSTWR_ENGINE_STATE_SEND_START;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == SDSTWR_MSG_TYPE_ACK )) {
          if ( rangingDebug ) {
            Serial.println("ACK RECEIVED & MEMO T4");
          }
          sqn++;
          t4 = decaduino.lastRxTimestamp;
          state_sdstwr = SDSTWR_ENGINE_STATE_SEND_ACK_CLIENT;
        }
      }
      break;

    case SDSTWR_ENGINE_STATE_SEND_ACK_CLIENT:
      if ( rangingDebug ) {
        Serial.println("SEND ACK TO SERVER");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn_ack;
      txData[5] = RANGING_MSG;
      txData[6] = SDSTWR_PROTOCOL;
      txData[7] = SDSTWR_MSG_TYPE_ACK;
      nbData = 8;
      tx = TX_ON;
      state_sdstwr = SDSTWR_ENGINE_STATE_MEMORISE_T5;
      break;

    case SDSTWR_ENGINE_STATE_MEMORISE_T5:
      if (( txEnd == TX_SUCCEEDED )) {
        if ( rangingDebug ) {
          Serial.println("ACK SENT AND MEMO T5");
        }
        t5 = decaduino.lastTxTimestamp;
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        state_sdstwr = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY;
      }   
      break;

    case SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY:
      if ( millis() > timeout ) {
        state_sdstwr = SDSTWR_ENGINE_STATE_SEND_START;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == SDSTWR_MSG_TYPE_DATA_REPLY )) {
          if ( rangingDebug ) {
            Serial.println("DATA REPLY RECEIVED");
          }
          sqn_ack = rxData[4] + 1;
          state_sdstwr = SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
        }
      }
      break;

    case SDSTWR_ENGINE_STATE_EXTRACT_T2_T3_T6:
      if ( rangingDebug ) {
        Serial.println("EXTRACT T2 T3 T6");
      }
      t2 = decaduino.decodeUint64(&rxData[8]);
      t3 = decaduino.decodeUint64(&rxData[16]);
      t6 = decaduino.decodeUint64(&rxData[24]);
      tof = ((((( t4 - t1 ) & mask ) - (( t3 - t2 ) & mask )) & mask ) + (((( t6 - t3 ) & mask ) - (( t5 - t4 ) & mask )) & mask ))/4;
      Serial.print("ToF=");
      Serial.print(tof, HEX);
      Serial.print(" d=");
      Serial.print(tof * COEFF);
      Serial.println();
      if ( tof * COEFF < 0 ) {
        Serial.println("ERROR NEGATIVE VALUE FOR DISTANCE");
      }
      ranging = false;
      rx = RX_ON;
      state_sdstwr = SDSTWR_ENGINE_STATE_IDLE;
      break;


    case SDSTWR_ENGINE_STATE_INIT_SERVER:
      if ( rangingDebug ) {
        Serial.println("SDSTWR SERVER");
      }
      rx = RX_ON;
      state_sdstwr = SDSTWR_ENGINE_STATE_WAIT_START;
    break;

    case SDSTWR_ENGINE_STATE_WAIT_START:
      if (( rx == RX_DONE ) && ( msg_type == SDSTWR_MSG_TYPE_START )) {
        if ( rangingDebug ) {
          Serial.println("START RECEIVED & MEMO T2");
        }
        sqn_ack = rxData[4] + 1;
        t2 = decaduino.lastRxTimestamp;
        state_sdstwr = SDSTWR_ENGINE_STATE_SEND_ACK_SERVER;
      }
      break;

    case SDSTWR_ENGINE_STATE_SEND_ACK_SERVER:
      if ( rangingDebug ) {
        Serial.println("SEND ACK TO CLIENT");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn_ack;
      txData[5] = RANGING_MSG;
      txData[6] = SDSTWR_PROTOCOL;
      txData[7] = SDSTWR_MSG_TYPE_ACK;
      nbData = 8;
      tx = TX_ON;
      state_sdstwr = SDSTWR_ENGINE_STATE_MEMORISE_T3;
      break;

    case SDSTWR_ENGINE_STATE_MEMORISE_T3:
      if ( txEnd == TX_SUCCEEDED ) {
        if ( rangingDebug ) {
          Serial.println("ACK SENT AND MEMO T3");
        }
        t3 = decaduino.lastTxTimestamp;
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        state_sdstwr = TWR_ENGINE_STATE_WAIT_ACK_SERVER;
      }
      break;

    case TWR_ENGINE_STATE_WAIT_ACK_SERVER:
      if ( millis() > timeout ) {
        state_sdstwr = SDSTWR_ENGINE_STATE_INIT_SERVER;
      } else {
        if (( rx == RX_DONE ) && ( msg_type == SDSTWR_MSG_TYPE_ACK )) {
          if ( rangingDebug ) {
            Serial.println("ACK RECEIVED & MEMO T6");
          }
          t6 = decaduino.lastRxTimestamp;
          state_sdstwr = SDSTWR_ENGINE_STATE_SEND_DATA_REPLY;
        }
      }
      break;

    case SDSTWR_ENGINE_STATE_SEND_DATA_REPLY:
      if ( rangingDebug ) {
        Serial.println("SEND DATA REPLY");
      }
      decaduino.encodeUint16(label, &txData[0]);
      decaduino.encodeUint16(label2, &txData[2]);
      txData[4] = sqn;
      txData[5] = RANGING_MSG;
      txData[6] = SDSTWR_PROTOCOL;
      txData[7] = SDSTWR_MSG_TYPE_DATA_REPLY;
      decaduino.encodeUint64(t2, &txData[8]);
      decaduino.encodeUint64(t3, &txData[16]);
      decaduino.encodeUint64(t6, &txData[24]);
      nbData = 32;
      tx = TX_ON;
      state_sdstwr = SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT;
      break;

    case SDSTWR_ENGINE_STATE_WAIT_DATA_REPLY_SENT:
      if (( txEnd == TX_SUCCEEDED )) {
        if ( rangingDebug ) {
          Serial.println("DATA REPLY SENT");
        }
        timeout = millis() + TIMEOUT;
        rx = RX_ON;
        sqn++;
        ranging = false;
        state_sdstwr = SDSTWR_ENGINE_STATE_IDLE;
      }
      break;

    default:
      state_sdstwr = SDSTWR_ENGINE_STATE_IDLE;
      break;
  }
}


void rangingRequest(int index) {
  if ( neighb_table[index].Status%2 == 1 ) {
    if ( protocol == TWR_PROTOCOL ) {
      if (( neighb_table[index].protocols_available%2 == 1 ) && ( node_protocols_available%2 == 1 )) {
        Serial.println("TWR RANGING REQUEST");
        rangingReq = true;
        state_ranging = RANGING_ENGINE_STATE_SEND_RANGING_REQUEST;
      } else {
        Serial.println("PROTOCOL IS NOT AVAILABLE");
      }
    } else if ( protocol == SDSTWR_PROTOCOL ) {
      if ((( neighb_table[index].protocols_available%4 == 2 ) || ( neighb_table[index].protocols_available%4 == 3 )) && (( node_protocols_available%4 == 2 ) || ( node_protocols_available%4 == 3 ))) {
        Serial.println("SDSTWR RANGING REQUEST");
        rangingReq = true;
        state_ranging = RANGING_ENGINE_STATE_SEND_RANGING_REQUEST;
      } else {
        Serial.println("PROTOCOL IS NOT AVAILABLE");
      }
    }
  } else {
    Serial.println("NODE IS NOT AVAILABLE");
  }
}


void rangingResponse() {
  
  if ( rangingDebug ) {
    Serial.println("RANGING RESPONSE");
  }
  label2 = decaduino.decodeUint16(src);
  if ( protocol == TWR_PROTOCOL ) {
    state_twr = TWR_ENGINE_STATE_INIT_SERVER;
  } else if ( protocol == SDSTWR_PROTOCOL ) {
    state_sdstwr = TWR_ENGINE_STATE_INIT_SERVER;
  }
  ranging = true;
}


void sendNeighbPosition() {
  
  decaduino.encodeUint16(label, &txData[0]);
  decaduino.encodeUint16(broadcast, &txData[2]);
  txData[4] = sqn;
  txData[5] = NEIGHB_POS;
  txData[6] = node_status;
  if (( node_status%8 == 4 ) || ( node_status%8 == 5 )) {
    // 3D Position
    decaduino.encodeUint32(node_x, &txData[7]);
    decaduino.encodeUint32(node_y, &txData[11]);
    decaduino.encodeUint32(node_z, &txData[15]);
    if (( node_status%32 >= 8 ) && ( node_status%32 <= 23 )) {
      // eAR announced
      decaduino.encodeUint32(node_eAR, &txData[19]);
      txData[23] = node_protocols_available;
      nbData = 24;
    } else {
      txData[19] = node_protocols_available;
      nbData = 20;
    }
  } else if (( node_status%8 == 2 ) || ( node_status%8 == 3 )) {
    // 2D Position
    decaduino.encodeUint32(node_x, &txData[7]);
    decaduino.encodeUint32(node_y, &txData[11]);
    if (( node_status%32 >= 8 ) && ( node_status%32 <= 23 )) {
      // eAR announced
      decaduino.encodeUint32(node_eAR, &txData[15]);
      txData[19] = node_protocols_available;
      nbData = 20;
    } else {
      txData[15] = node_protocols_available;
      nbData = 16;
    }
  } else {
    // No position announced
    if (( node_status%32 >= 8 ) && ( node_status%32 <= 23 )) {
      // eAR announced
      decaduino.encodeUint32(node_eAR, &txData[7]);
      txData[11] = node_protocols_available;
      nbData = 12;
    } else {
      txData[7] = node_protocols_available;
      nbData = 8;
    }
  }
  if ( rangingDebug ) {
    Serial.println("SEND NEIGHBOUR POSITION");
    Serial.printf("Source : 0x%04X", label);
    Serial.println();
    Serial.printf("Destination : 0x%04X", broadcast);
    Serial.println();
    Serial.print("sqn : ");
    Serial.println(sqn);
    Serial.print("Node status : 0b");
    Serial.println(node_status, BIN);
    if (( node_status%8 == 4 ) || ( node_status%8 == 5 )) {
      Serial.print("X : ");
      Serial.println(node_x);
      Serial.print("Y : ");
      Serial.println(node_y);
      Serial.print("Z : ");
      Serial.println(node_z);
    } else if (( node_status%8 == 2 ) || ( node_status%8 == 3 )) {
      Serial.print("X : ");
      Serial.println(node_x);
      Serial.print("Y : ");
      Serial.println(node_y);
    }
    if (( node_status%32 >= 8 ) && ( node_status%32 <= 23 )) {
      Serial.print("eAR value : ");
      Serial.println(node_eAR);
    }
    Serial.print("Available protocols : 0b");
    Serial.println(node_protocols_available, BIN);
    Serial.println();
  }
  tx = TX_ON;
}


void initNeighbTable() {
  
  int i;
  for (i = 0; i<MAX_NEIGHB; i++) {
    neighb_table[i].neighb_label = 0xFFFF;
  }
}


//check if the node is already registered in the table and returns the index of the node in the table
int nodeInNeighbTable(uint16_t addr) {
  
  int i;
  for (i = 0; i<MAX_NEIGHB; i++) {
    if ( neighb_table[i].neighb_label == addr ) {
      return i;
    }
  }
  return -1;
}


void printNeighbTable(void) {

  Serial.printf("index\t@\tstatus\tx\ty\tz\teAR\tprotocs\n");
  for (int i=0; i<MAX_NEIGHB; i++) {
    if (neighb_table[i].neighb_label != 0xFFFF) {
      Serial.printf("%d\t0x%04x\t0x%02x\t%.4f\t%.4f\t%.4f\t%.4f\t0x%02x\n", i, neighb_table[i].neighb_label, neighb_table[i].Status, neighb_table[i].x, neighb_table[i].y, neighb_table[i].z, neighb_table[i].eAR, neighb_table[i].protocols_available);
    }
  }
}


//check if the node if available
bool nodeAvailable(int index) {
  
  // Available == 1
  return (neighb_table[index].Status > 3);
}


//return index of the last node in table
int indexLastNode() {
  
  int i;
  for (i = 0; i<MAX_NEIGHB; i++) {
    if ( neighb_table[i].neighb_label  == 0xFFFF ) {
      return i;
    }
  }
  return -1;
}


void neighbUpdate(int index) {
  
  neighb_table[index].neighb_label = decaduino.decodeUint16(src);
  neighb_table[index].Status = rxData[6];
  if (( neighb_table[index].Status%8 == 4 ) || ( neighb_table[index].Status%8 == 5 )) {
    // 3D Position
    neighb_table[index].x = decaduino.decodeUint32(&rxData[7]);
    neighb_table[index].y = decaduino.decodeUint32(&rxData[11]);
    neighb_table[index].z = decaduino.decodeUint32(&rxData[15]);
    if (( neighb_table[index].Status%32 >= 8 ) && ( neighb_table[index].Status%32 <= 23 )) {
      // eAR announced
      neighb_table[index].eAR = decaduino.decodeUint32(&rxData[19]);
      neighb_table[index].protocols_available = rxData[23];
    } else {
      neighb_table[index].protocols_available = rxData[19];
    }
  } else if (( neighb_table[index].Status%8 == 2 ) || ( neighb_table[index].Status%8 == 3 )) {
    // 2D Position
    neighb_table[index].x = decaduino.decodeUint32(&rxData[7]);
    neighb_table[index].y = decaduino.decodeUint32(&rxData[11]);
    if (( neighb_table[index].Status%32 >= 8 ) && ( neighb_table[index].Status%32 <= 23 )) {
      // eAR announced
      neighb_table[index].eAR = decaduino.decodeUint32(&rxData[15]);
      neighb_table[index].protocols_available = rxData[19];
    } else {
      neighb_table[index].protocols_available = rxData[15];
    }
  } else {
    // No position
    if (( neighb_table[index].Status%32 >= 8 ) && ( neighb_table[index].Status%32 <= 23 )) {
      // eAR announced
      neighb_table[index].eAR = decaduino.decodeUint32(&rxData[7]);
      neighb_table[index].protocols_available = rxData[11];
    } else {
      neighb_table[index].protocols_available = rxData[7];
    }
  }
  if ( rangingDebug ) {
    Serial.print("NEIGHB AT INDEX ");
    Serial.println(index);
    Serial.printf("label : 0x%04X", neighb_table[index].neighb_label);
    Serial.println();
    Serial.print("status : 0b");
    Serial.println(neighb_table[index].Status, BIN);
    if (( neighb_table[index].Status%8 == 4 ) || ( neighb_table[index].Status%8 == 5 )) {
      // 3D Position
      Serial.print("X : ");
      Serial.println(neighb_table[index].x);
      Serial.print("Y : ");
      Serial.println(neighb_table[index].y);
      Serial.print("Z : ");
      Serial.println(neighb_table[index].z);
    } else if (( neighb_table[index].Status%8 == 2 ) || ( neighb_table[index].Status%8 == 3 )) {
      // 2D Position
      Serial.print("X : ");
      Serial.println(neighb_table[index].x);
      Serial.print("Y : ");
      Serial.println(neighb_table[index].y);
    }
    if (( neighb_table[index].Status%32 >= 8 ) && ( neighb_table[index].Status%32 <= 23 )) {
        // eAR announced
      Serial.print("eAR value : ");
      Serial.println(neighb_table[index].eAR);
    }
    Serial.print("Available protocols : 0b");
    Serial.println(neighb_table[index].protocols_available, BIN);
    Serial.println();
  }
}


void neighbSave() {
  
  if ( rangingDebug ) {
    Serial.println("SAVE NEIGHB");
  }
  int index = nodeInNeighbTable(decaduino.decodeUint16(src));
  // node in table
  if ( index != -1) {
    // update
    if ( rangingDebug ) {
      Serial.print("UPDATE ");
    }
    neighbUpdate(index);
  }
  else {
    // add
    if ( rangingDebug ) {
      Serial.print("ADD ");
    }
    neighbUpdate(indexLastNode());
  }
}


bool nodePositionIsKnown() {
  
  return (( node_status%8 >= 2 ) && ( node_status%8 <= 5 ));
}


bool neighbPositionIsKnown(int index) {
  return (( neighb_table[index].Status%8 >= 2 ) && ( neighb_table[index].Status%8 <= 5 ));
}


float calcPosition(int index) {
  
  if ((( node_status%8 == 4 ) || ( node_status%8 == 5 )) && (( neighb_table[index].Status%8 == 4 ) || ( neighb_table[index].Status%8 == 5 ))) {
    // calc 3D Position
    if ( rangingDebug ) {
      Serial.println("3D distance");
    }
    return calc3DPosition(index);
  } else {
    // calc 2D Position
    if ( rangingDebug ) {
      Serial.println("2D distance");
    }
    return calc2DPosition(index);
  }
}


float square(float a) {
  return (a*a); 
}


float calc3DPosition(int index) {
  return ( sqrt(square(node_x - neighb_table[index].x) + square(node_y - neighb_table[index].y) + square(node_z - neighb_table[index].z)));
}


float calc2DPosition(int index) {
  return ( sqrt(square(node_x - neighb_table[index].x) + square(node_y - neighb_table[index].y)));
}


void checkSerialAvailable() {
  
  if (isAvailable) {
    
    while (Serial.available() > 1) {
      // if the positions of both nodes are known, only the label is needed
      label2 = Serial.parseInt();
      if ( rangingDebug ) {
        Serial.print("Destination label = ");
        Serial.println(label2);
      }
      int index = nodeInNeighbTable(label2);
      if ( index != -1 ) {
        if ( nodePositionIsKnown() && neighbPositionIsKnown(index) ) {
          float distance = calcPosition(index);
          Serial.print("Calculated distance = ");
          Serial.println(distance);
        } else {
          // else protocol number is also needed
          Serial.println("Specify the protocol number : 0 for TWR and 1 for SDSTWR");
          protocol = Serial.parseInt();
          if ( rangingDebug ) {
            if ( protocol == 0 ) {
              Serial.println("Protocol is TWR");
            } else if ( protocol == 1 ) {
              Serial.println("Protocol is SDSTWR");
            }
          }
          if (( protocol == TWR_PROTOCOL ) || ( protocol == SDSTWR_PROTOCOL )) {
            rangingRequest(index);
          } else {
            Serial.println("Specified protocol number is wrong, choose between 0 for TWR and 1 for SDSTWR");
          }
        } 
      } else {
        Serial.println("NODE IS NOT IN TABLE");
      }
      if ( rangingDebug ) {
        Serial.println();
      }
    }
  }
}


void neighbPosition() {
  
  if (( !rangingReq ) && ( !ranging ) && ( millis() > neighb_timeout )) {
    // NEIGHB_POSITION can't be send during a ranging phase
    sendNeighbPosition();
    sqn++;
    neighb_timeout = millis()+NEIGHB_POS_PERIOD*1000+random(-2000, 2000);
  }
}



void setup() {

  uint8_t buf[2];

  pinMode(13, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  digitalWrite(RGB_RED_PIN, HIGH);
  digitalWrite(RGB_GREEN_PIN, LOW);
  digitalWrite(RGB_BLUE_PIN, HIGH);

  randomSeed(analogRead(A20));

  while(Serial.available()==0);
  Serial.setTimeout(86400);
  Serial.println("<label> <status> <2D or 3D position> <eAR> <protocols>");
  label=Serial.parseInt();
  Serial.printf("label : 0x%04X", label);
  Serial.println();
  node_status=Serial.parseInt();
  Serial.print("Node status : 0b");
  Serial.println(node_status, BIN);
  if ( node_status%2 == 1 ) {
    isAvailable = true;
    Serial.println("Node is available for ranging");
  }
  if (( node_status%8 == 4 ) || ( node_status%8 == 5 )) {
    // 3D Position
    node_x = Serial.parseFloat();
    Serial.print("X : ");
    Serial.println(node_x);
    node_y = Serial.parseFloat();
    Serial.print("Y : ");
    Serial.println(node_y);
    node_z = Serial.parseFloat();
    Serial.print("Z : ");
    Serial.println(node_z);
  } else if (( node_status%8 == 2 ) || ( node_status%8 == 3 )) {
    // 2D Position
    node_x = Serial.parseFloat();
    Serial.print("X : ");
    Serial.println(node_x);
    node_y = Serial.parseFloat();
    Serial.print("Y : ");
    Serial.println(node_y);
  }
  if (( node_status%32 >= 8 ) && ( node_status%32 <= 23 )) {
    // eAR announced
    node_eAR = Serial.parseFloat();
    Serial.print("eAR : ");
    Serial.println(node_eAR);
  }
  node_protocols_available = Serial.parseInt();
  Serial.print("Protocols available : 0b");
  Serial.println(node_protocols_available, BIN);
  /*inter_cycle_delay=Serial.parseInt();
  rxA_enable_between_cycles=Serial.parseInt();*/
  
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

  initNeighbTable();
  neighb_timeout = millis()+NEIGHB_POS_PERIOD*1000+random(-2000, 2000);
  rangeNeighbTimeout = millis() + 5000; //RANGE_NEIGHB_PERIOD*1000;
  decaduino.plmeRxEnableRequest();
  Serial.println("RUN");
  Serial.println();
}


void loop() { 

/*
  if ( millis() > EXPERIMENTATION_DURATION*1000 ) {
    Serial.println("END");
    digitalWrite(RGB_RED_PIN, HIGH);
    digitalWrite(RGB_GREEN_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, HIGH);
    decaduino.plmeRxDisableRequest();
    while (1);
  }
*/

  rangingEngine();
  twrEngine();
  sdstwrEngine();
  macEngine();
  checkSerialAvailable();
  neighbPosition();

  if ( millis() > printNeighbTimeout ) {
    printNeighbTimeout = millis() + PRINT_NEIGHB_PERIOD*1000;
    printNeighbTable();
  }
}
