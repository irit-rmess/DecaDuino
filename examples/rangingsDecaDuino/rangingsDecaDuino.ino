#include <SPI.h>
#include <DecaDuino.h>

DecaDuino decaduino;
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;

void setup() {

  pinMode(13, OUTPUT);
  SPI.setSCK(14);
  if (!decaduino.init()){
    Serial.print("decaduino init failled");
    while(1){
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
    }
  }
  decaduino.setRxBuffer(rxData, &rxLen);
  
  delay(2000);
  printMenu();
}

void loop() {
  
  decaduino.rangingEngine();
  
  while ( Serial.available() > 0 ) {
    
    int choice = Serial.parseInt();
    
    if (Serial.read() == '\n') {
    
      switch ( choice ) {
        
        case 1:
          Serial.println("Active RX and waiting for any ranging requests");
          break;
        
        case 2:
          Serial.println("Sending periodic TWR ranging requests");
          break;
        
        case 3:
          Serial.println("Sending periodic SDS-TWR ranging requests");
          break;
        
        default:
          Serial.println("Unknown choice");
          break;
      }
      
      printMenu();
    }
  }
}

void printMenu() {

  Serial.println("--------------------");
  Serial.println("1. RX and wait for any ranging request (default)");
  Serial.println("2. Send periodic TWR ranging request");
  Serial.println("3. Send periodic SDS-TWR ranging request");
  Serial.println("--------------------");
  Serial.print("Enter choice: ");
}
