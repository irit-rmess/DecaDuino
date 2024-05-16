#include <SPI.h>
#include <DecaDuino.h>

#define AIR_SPEED_OF_LIGHT 299792458
#define DW1000_TIMEBASE 15.65E-12
#define COEFF AIR_SPEED_OF_LIGHT*DW1000_TIMEBASE

//#define VERBOSE 1

#define TIMEOUT 40

#define TWR_ENGINE_STATE_INIT 1
#define TWR_ENGINE_STATE_WAIT_NEW_CYCLE 2
#define TWR_ENGINE_STATE_SEND_START 3
#define TWR_ENGINE_STATE_WAIT_START_SENT 4
#define TWR_ENGINE_STATE_MEMORISE_T1 5
#define TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ 6
#define TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ 7
#define TWR_ENGINE_STATE_WAIT_ACK_REQ 8
#define TWR_ENGINE_STATE_MEMORISE_T4 9
#define TWR_ENGINE_STATE_SEND_ACK 10 
#define TWR_ENGINE_STATE_WAIT_ACK_SENT 11
#define TWR_ENGINE_STATE_MEMORISE_T5 12
#define TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY 13
#define TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY 14
#define TWR_ENGINE_STATE_WAIT_DATA_REPLY 15
#define TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 16

#define TWR_ENGINE_STATE_ON_FOR_DATA_REQUEST 17
#define TWR_ENGINE_WAIT_DATAREQ 18

#define TWR_MSG_TYPE_UNKNOW 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_ACK_REQ 2
#define TWR_MSG_TYPE_ACK 3
#define TWR_MSG_TYPE_DATA_REPLY 4

#define TWR_MSG_TYPE_DATAREQ 5

#define COMPUTE 50

int seqnum;

int rxFrames;

uint64_t t1, t2, t3, t4, t5, t6;
uint64_t tof;

uint64_t _Delai;

#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#elif defined(TEENSYDUINO)
DecaDuino decaduino;
#endif
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
int timeout;

int ons;

/*storage for timestamps*/
struct Ranging{
uint16_t anchor;
uint64_t t1;
uint64_t t2;
uint64_t t3;
uint64_t t4;
uint64_t t5;
uint64_t t6;
double distance;
int valid;
double offsetPPM_t4;
double offsetPPM_drep;
};

uint16_t hi16=0;
double tmp=0;

uint64_t test=0x0000000F24586219;

Ranging results[3];
uint32_t addrPANID=0xDECA0025;
int mindex;

void setup() {

  pinMode(13, OUTPUT);
  
#ifdef TEENSYDUINO    
  SPI.setSCK(14);
#endif
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
  ons=0;
  results[0].anchor=0x0022;
  results[1].anchor=0x0023;
  results[2].anchor=0x0024;
  results[0].valid=0;
    results[1].valid=0;
      results[2].valid=0;
  seqnum=0;
  //decaduino.print(test);
  mindex=0;
}

void loop() {

  decaduino.engine();
  
  switch (state) {
  
    case TWR_ENGINE_STATE_INIT :
      decaduino.plmeRxDisableRequest();
      state = TWR_ENGINE_STATE_WAIT_NEW_CYCLE;
      break;
    
    case TWR_ENGINE_STATE_WAIT_NEW_CYCLE :
      delay(100);
      //Serial.println("New PDS_TWR");
      
      state = TWR_ENGINE_STATE_SEND_START;
        results[0].valid=0;
    results[1].valid=0;
      results[2].valid=0;
      break;
    
    case TWR_ENGINE_STATE_SEND_START :
      txData[0] =0x41;
      txData[1] =0x88;
      txData[2] =seqnum;      
      txData[3] =0xCA;
      txData[4] =0xDE;
      txData[5] =0xFF;
      txData[6] =0xFF;
      txData[7] =0x25;
      txData[8] =0x00;
      txData[9] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 10);
      state = TWR_ENGINE_STATE_WAIT_START_SENT;
      seqnum++;
      break;
       
    case TWR_ENGINE_STATE_WAIT_START_SENT :
      if (decaduino.hasTxSucceeded())
      state = TWR_ENGINE_STATE_MEMORISE_T1;
      break;
       
    case TWR_ENGINE_STATE_MEMORISE_T1 :
      t1 = decaduino.getLastTxTimestamp();
      state = TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_WATCHDOG_FOR_ACK_REQ :
      timeout = millis() + TIMEOUT*2;
      state = TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ :
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_ACK_REQ;
      break;
       
    case TWR_ENGINE_STATE_WAIT_ACK_REQ :
      if (( millis() > timeout)||(ons>2)) {//too late
      //Serial.println("_______________TIMEOUT!!!!!!");
        if(ons==0){//je n'ai aucune réponse
            state = TWR_ENGINE_STATE_INIT;
        } else { state=TWR_ENGINE_STATE_SEND_ACK;}
      }
      else { //encore du temps
        if (decaduino.rxFrameAvailable()){
          if (rxData[9] == TWR_MSG_TYPE_ACK_REQ) {
          state = TWR_ENGINE_STATE_MEMORISE_T4;
          
          
          ons++;
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
        }
      }
      break;
       
    case TWR_ENGINE_STATE_MEMORISE_T4 :
      t4 = decaduino.getLastRxTimestamp();
      switch(rxData[7]){
            case 0x23: results[1].t4=t4;
            results[1].offsetPPM_t4=decaduino.getLastRxSkew();
           // Serial.println("rx ACK+REQ: 0x23 ");
            break;
            case 0x22: results[0].t4=t4;
            results[0].offsetPPM_t4=decaduino.getLastRxSkew();
           //Serial.println("rx ACK+REQ: 0x22 ");
            break;
            case 0x24: results[2].t4=t4;
            results[2].offsetPPM_t4=decaduino.getLastRxSkew();
            //Serial.println("rx ACK+REQ: 0x24 ");
            break;
            
            }
      if ( millis() > timeout){
        state = TWR_ENGINE_STATE_SEND_ACK;
      }else{//still got time
        state=TWR_ENGINE_STATE_RX_ON_FOR_ACK_REQ;
      }
      break;
       
    case TWR_ENGINE_STATE_SEND_ACK :
    decaduino.plmeRxDisableRequest();
    txData[0] =0x41;
      txData[1] =0x88;
      txData[2] =seqnum;      
      txData[3] =0xCA;
      txData[4] =0xDE;
      txData[5] =0xFF;
      txData[6] =0xFF;
      txData[7] =0x25;
      txData[8] =0x00;
      txData[9]= TWR_MSG_TYPE_DATAREQ;
      decaduino.pdDataRequest(txData, 10);
      state = TWR_ENGINE_STATE_WAIT_ACK_SENT;
      //Serial.println("ready to send datareq");
            seqnum++;
      break;
       
    case TWR_ENGINE_STATE_WAIT_ACK_SENT :
      if (decaduino.hasTxSucceeded()){
      state = TWR_ENGINE_STATE_MEMORISE_T5;
               //Serial.println("_________________got t5!"); 
              ons=0; 
  }//else Serial.println("tx issue");
      break;
      
    case TWR_ENGINE_STATE_MEMORISE_T5 :
      t5 = decaduino.getLastTxTimestamp();
      state = TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_WATCHDOG_FOR_DATA_REPLY :
      timeout = millis() + TIMEOUT;
      state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY :
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;
       
    case TWR_ENGINE_STATE_WAIT_DATA_REPLY :
      if (( millis() > timeout)||(ons>2)) {
        if(ons!=0)
          state=COMPUTE;
        else
      state = TWR_ENGINE_STATE_INIT;}
      else { 
        if (decaduino.rxFrameAvailable()){
          if (rxData[9] == TWR_MSG_TYPE_DATA_REPLY) {
          state = TWR_ENGINE_STATE_EXTRACT_T2_T3_T6;
          ons++;
          //Serial.println("received DATA_REPLY");
          } else state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
        }
      }
      break;
       
    case TWR_ENGINE_STATE_EXTRACT_T2_T3_T6 :
      t2 = decaduino.decodeUint64(&rxData[10]);
      t3 = decaduino.decodeUint64(&rxData[19]);
      t6 = decaduino.decodeUint64(&rxData[27]);
      
      switch(rxData[7]){
        case 0x23: results[1].t2=t2;
                   results[1].t3=t3;
                   results[1].t6=t6;
                   results[1].valid=1;
                   results[1].offsetPPM_drep=decaduino.getLastRxSkew();
                   
        break;
        case 0x22: results[0].t2=t2;
                   results[0].t3=t3;
                   results[0].t6=t6;
                   results[0].valid=1;
                   results[0].offsetPPM_drep=decaduino.getLastRxSkew();
                   break;
        case 0x24: results[2].t2=t2;
                   results[2].t3=t3;
                   results[2].t6=t6;
                   results[2].valid=1;
                   results[2].offsetPPM_drep=decaduino.getLastRxSkew();
                   break;
        
        }
     
      if ( millis() > timeout) {
        if(ons!=0){
          state=COMPUTE;
        }else state = TWR_ENGINE_STATE_INIT;}
      else { 
        state = TWR_ENGINE_STATE_RX_ON_FOR_DATA_REPLY;
      }
      break;
    
    case COMPUTE:
    Serial.print("index=");
    Serial.println(mindex);
      if(results[0].valid==1){
        Serial.print("set 0|");  
        #ifdef VERBOSE
        Serial.print("t1=");
        decaduino.print(t1);   
        
        Serial.print("t2=");
        decaduino.print(results[0].t2);
       
        Serial.print("t3=");
        decaduino.print(results[0].t3);
        
        Serial.print("t4=");
        decaduino.print(results[0].t4);
        
        Serial.print("t5=");
        decaduino.print(t5);
        
        Serial.print("t6=");
        decaduino.print(results[0].t6);
        Serial.println();
#endif
        tof = (2*results[0].t4 - t1 - 2*results[0].t3 + results[0].t2 + results[0].t6 - t5)/4;
        Serial.print("ToF=");
        printf("%"PRIu64"",tof);
         hi16=(uint16_t)(tof >> 32);
        if(hi16 & 0x80!=0){
         // Serial.println("TOF is negative");
          tof |=0xFFFFFF0000000000;
          tof=~tof;
          tof++;
          tmp=-1*tof;
          Serial.print("|d=");
          Serial.print(-1*(double)(tof*COEFF));
          Serial.print("|");
          //Serial.println(tmp);
        }else{
         //Serial.println("TOF is positive");        
         Serial.print("|d=");
         Serial.print((double)(tof*COEFF));
         Serial.print("|");
        }
        Serial.print("clkOffset(ppm)_t4=");
        Serial.print(results[0].offsetPPM_t4);
        Serial.print("|");
         Serial.print("clkOffset(ppm)_drep=");
        Serial.print(results[0].offsetPPM_drep);
        Serial.print("|");
        _Delai=results[1].t4-results[0].t4;
      Serial.print("D=");
        printf("%"PRIu64"",_Delai);
        Serial.println();
      }
      
      if(results[1].valid==1){
        Serial.print("set 1|");
        #ifdef VERBOSE
        Serial.print("t1=");
        decaduino.print(t1);   
        
        Serial.print("t2=");
        decaduino.print(results[1].t2);
        
        Serial.print("t3=");
        decaduino.print(results[1].t3);
        
        Serial.print("t4=");
        decaduino.print(results[1].t4);
       
        Serial.print("t5=");
        decaduino.print(t5);
        
        Serial.print("t6=");
        decaduino.print(results[1].t6);
        Serial.println();
   #endif     
        tof = (2*results[1].t4 - t1 - 2*results[1].t3 + results[1].t2 + results[1].t6 - t5)/4;
        Serial.print("ToF=");        
        printf("%"PRIu64"",tof);
        
        hi16=(uint16_t)(tof >> 32);
        if(hi16 & 0x80!=0){
          //Serial.println("TOF is negative");
          //erial.println("TOF is negative");
          tof |=0xFFFFFF0000000000;
          tof=~tof;
          tof++;
          Serial.print("|d=");
          Serial.print(-1*(double)(tof*COEFF));
          Serial.print("|");
          //Serial.println(tmp);
        }else{
        // Serial.println("TOF is positive");
         
        Serial.print("|d=");
        Serial.print((double)(tof*COEFF));
        Serial.print("|");
        }
        Serial.print("clkOffset(ppm)=");
        Serial.print(results[1].offsetPPM_t4);
        Serial.print("|");
         Serial.print("clkOffset(ppm)_drep=");
        Serial.print(results[1].offsetPPM_drep);
        Serial.print("|");
        _Delai=results[1].t4-results[0].t4;
      Serial.print("D=");
        printf("%"PRIu64"",_Delai);
        Serial.println();
      }
      
      if(results[2].valid==1){
        Serial.print("set 2|");
        #ifdef VERBOSE
        Serial.print("t1=");
        decaduino.print(t1);   
        
        Serial.print("t2=");
        decaduino.print(results[2].t2);
        
        Serial.print("t3=");
        decaduino.print(results[2].t3);
        
        Serial.print("t4=");
        decaduino.print(results[2].t4);
        
        Serial.print("t5=");
        decaduino.print(t5);
        
        Serial.print("t6=");
        decaduino.print(results[2].t6);
        Serial.println();
      #endif  
        tof = (2*results[2].t4 - t1 - 2*results[2].t3 + results[2].t2 + results[2].t6 - t5)/4;
        Serial.print("ToF=");
        printf("%"PRIu64"",tof);
        
        hi16=(uint16_t)(tof >> 32);
        if(hi16 & 0x80!=0){
          //Serial.println("TOF is negative");
          //Serial.println("TOF is negative");
          tof |=0xFFFFFF0000000000;
          tof=~tof;
          tof++;
          Serial.print("|d=");
          Serial.print(-1*(double)(tof*COEFF));
          Serial.print("|");
          //Serial.println(tmp);
        }else{
         //Serial.println("TOF is positive");
         
          Serial.print("|d=");
          Serial.print((double)(tof*COEFF));
          Serial.print("|");
        }
        Serial.print("clkOffset(ppm)=");
        Serial.print(results[2].offsetPPM_t4);
        Serial.print("|");
         Serial.print("clkOffset(ppm)_drep=");
        Serial.print(results[2].offsetPPM_drep);
        Serial.print("|");
        _Delai=results[1].t4-results[0].t4;
      Serial.print("D=");
        printf("%"PRIu64"",_Delai);
        Serial.println();
      }
      
      mindex=mindex+1;
      state = TWR_ENGINE_STATE_INIT;
    break;
       
    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}
