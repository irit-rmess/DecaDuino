#include <spi4teensy3.h>
#include <DecaDuino.h>

int i;
DecaDuino decaduino;
uint8_t data[1024];
#define FRAME_LEN 64

#define SENDER
// ou
//#define RECEIVER

void send() {
  
  int i;
  
  for (i=0; i<FRAME_LEN; i++)
    data[i] = i;

  decaduino.plmeDataRequest(data, FRAME_LEN);
}

void setup() {

  pinMode(13, OUTPUT);
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
   
    while(1);
  }
}

void loop() {

  digitalWrite(13, HIGH);

  #ifdef RECEIVER
  decaduino.plmeRxEnableRequest();
  delay(5000);
  #endif

  #ifdef SENDER
  send();
  #endif

  delay(1);
  digitalWrite(13, LOW);
  Serial.print("+");

  #ifdef RECEIVER
  decaduino.plmeRxDisableRequest();
  delay(5000);
  #endif

  #ifdef SENDER
  delay(1000);
  #endif
  
  //Serial.println(decaduino.getEuid, HEX);
  
  // End of the cycle
}


