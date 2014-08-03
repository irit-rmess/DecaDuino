#include <spi4teensy3.h>
#include <Arduino.h>

/*make sure this is a power of two. */
#define max 1024
static uint8_t buf[max]; /* buffer */
int i;

void setup() {

  pinMode(13, OUTPUT);
  pinMode(0, INPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  for (i=0; i<max; i++)
    buf[i] = 0;

  spi4teensy3::init(1,0,0);
  // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1);
  // and then reassign pin 14 to SCK
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); 
}

void loop() {

  digitalWrite(13, HIGH);

  // Read device ID
  buf[0] = 0x40;
  buf[1] = 0x02;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,2); //buf, max);
  spi4teensy3::receive(buf, 2);
  digitalWrite(10, HIGH);
  for (i=0; i<2; i++) {
    Serial.print ("|0x");
    Serial.print (buf[i], HEX);
  }
  Serial.println("|");
  delay(500);

  // Read short address and PANid
  buf[0] = 0x03;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,1);
  spi4teensy3::receive(buf, 4);
  digitalWrite(10, HIGH);
  for (i=0; i<4; i++) {
    Serial.print ("|0x");
    Serial.print (buf[i], HEX);
  }
  Serial.println("|");
  delay(500);
/*
  // Write short address and PANid
  buf[0] = 0xC3;
  buf[1] = 0x00;
  buf[2] = 0xEF;
  buf[3] = 0xBE;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,4);
  digitalWrite(10, HIGH);
  Serial.println("writing short address and panid");
  delay(500);
*/
  // Write short address and PANid
  buf[0] = 0x83;
  buf[1] = 0x34;
  buf[2] = 0x12;
  buf[3] = 0x78;
  buf[4] = 0x56;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,5);
  digitalWrite(10, HIGH);
  Serial.println("writing short address and panid");
  delay(500);

  // Read short address and PANid
  buf[0] = 0x03;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,1);
  spi4teensy3::receive(buf, 4);
  digitalWrite(10, HIGH);
  for (i=0; i<4; i++) {
    Serial.print ("|0x");
    Serial.print (buf[i], HEX);
  }
  Serial.println("|");
  delay(500);

  // Read TX params
  buf[0] = 0x08;
  digitalWrite(10, LOW);
  spi4teensy3::send(buf,1);
  spi4teensy3::receive(buf, 5);
  digitalWrite(10, HIGH);
  for (i=0; i<5; i++) {
    Serial.print ("|0x");
    Serial.print (buf[i], HEX);
  }
  Serial.println("|");

  // End of the cycle
  digitalWrite(13, LOW);
  delay(100);

}


