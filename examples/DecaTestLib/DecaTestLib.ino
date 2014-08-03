#include <spi4teensy3.h>
#include <DecaDuino.h>

int i;
DecaDuino decaduino;
uint8_t data[128];
#define FRAME_LEN 30

//#define SENDER

void send() {
  
  int i;
  
  for (i=0; i<FRAME_LEN; i++)
    data[i] = i;

  decaduino.plmeDataRequest(data, FRAME_LEN);  
}

void setup() {

  pinMode(13, OUTPUT);
  if ( !decaduino.init() ) {
    while(1) Serial.println("decaduino init failed");
  }

  //decaduino.plmeRxEnableRequest();
/*  for (i=9; i!=0; i++) {
    delay(1000);
    Serial.print("+");
  }*/
}

void loop() {

  digitalWrite(13, HIGH);
  //decaduino.dummy();
  #ifdef SENDER
  send();
  #endif
  delay(2);
  //decaduino._my_handleInterrupt();
  //delay(100);
  digitalWrite(13, LOW);
  Serial.print("+");
  delay(200);
  // End of the cycle
}


