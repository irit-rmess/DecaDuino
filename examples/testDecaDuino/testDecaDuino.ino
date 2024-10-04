#include <SPI.h>
#include <DecaDuino.h>

int i;
int rxFrames;
#ifdef UWB_MODULE_DWM1001
DecaDuino decaduino(SS1, DW_IRQ);
#elif defined(TEENSYDUINO)
DecaDuino decaduino;
#endif
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
#define FRAME_LEN 64

#define SENDER
// ou
//#define RECEIVER

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
#ifdef TEENSYDUINO    
  SPI.setSCK(14);
  delay(250); // delay for wino serial setup
#endif  
  if ( !decaduino.init() ) {
    while(1){
      Serial.println("decaduino init failed");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  
  #ifdef RECEIVER
  rxFrames = 0;
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  
  #endif
}

void loop() {
  int i;
  decaduino.engine(); // it is required to periodically call this function

  #ifdef RECEIVER
  if ( decaduino.rxFrameAvailable() ) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("["); Serial.print(++rxFrames); Serial.print("] ");
    Serial.print(rxLen);
    Serial.print("bytes received: ");
    for (i=0; i<rxLen; i++) {
      Serial.print(rxData[i], HEX);
      Serial.print("|");
    }
    Serial.println();
    decaduino.plmeRxEnableRequest();
    digitalWrite(LED_BUILTIN, LOW);
  }
  #endif

  #ifdef SENDER
  digitalWrite(LED_BUILTIN, HIGH);
  for (i=0; i<FRAME_LEN; i++)
    txData[i] = i;
  decaduino.pdDataRequest(txData, FRAME_LEN);
  while (!decaduino.hasTxSucceeded()) { decaduino.engine();}  
  Serial.println("Frame sent");
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  #endif
  
  // End of the cycle
}


