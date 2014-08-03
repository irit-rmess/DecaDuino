// DecaDuino.cpp
//
// DecaWave DW1000 driver for Arduino
// See the README file in this directory for documentation

#include "DecaDuino.h"

DecaDuino* DecaDuino::_DecaDuinoInterrupt[MAX_NB_DW1000_FOR_INTERRUPTS] = {0, 0, 0};

DecaDuino::DecaDuino(uint8_t slaveSelectPin, uint8_t interruptPin) {

  _slaveSelectPin = slaveSelectPin;
  _interruptPin = interruptPin;
}


boolean DecaDuino::init() {

  uint16_t i;
  uint32_t u32tmp;

  // Wait for DW1000 POR (up to 5msec)
  delay(5);

  // Initialise the IRQ and Slave Select pin
  pinMode(_interruptPin, INPUT);
  pinMode(_slaveSelectPin, OUTPUT);
  digitalWrite(_slaveSelectPin, HIGH);

  // Initialise the SPI port
  spi4teensy3::init(1,0,0);
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK

  // Reset the DW1000 now
  resetDW1000();

  // Check the device type
  readSpi(DW1000_REGISTER_DEV_ID, buf, 4);
  if ( decodeUint32(buf) != 0xdeca0130 ) return false;

  // Attach interrupt handler
  if (_interruptPin == DW1000_IRQ0_PIN) {
    _DecaDuinoInterrupt[DW1000_IRQ0_PIN] = this;
    attachInterrupt(_interruptPin, DecaDuino::isr0, HIGH);
  } else if (_interruptPin == DW1000_IRQ1_PIN) {
    _DecaDuinoInterrupt[DW1000_IRQ1_PIN] = this;
    attachInterrupt(_interruptPin, DecaDuino::isr1, HIGH);
  } else if (_interruptPin == DW1000_IRQ2_PIN) {
    _DecaDuinoInterrupt[DW1000_IRQ2_PIN] = this;
    attachInterrupt(_interruptPin, DecaDuino::isr2, HIGH);
  } else return false;

  // Initialise buffer
  for (i=0; i<BUFFER_MAX_LEN; i++)
    buf[i] = 0;

  // Configure DW1000
  // System Event Mask Register
  //readSpiSubAddress(0x0E, 1, buf, 1);
  //buf[0] |= 0x20; // MRXPRD
  //buf[0] |= 0x02; // MRXSFDD
  //writeSpiSubAddress(0x0E, 1, buf, 1);

  /*
  readSpi(0x0E, buf, 4);
  for (i=0; i<4; i++) {
    Serial.print ("|");
    Serial.print (buf[i], HEX);
  }
  Serial.println("|");
  */

  /*
  setPanId(0xCAFE);
  Serial.print("PANid: ");
  Serial.println (getPanId(), HEX);
  setShortAddress(0x1234);
  Serial.print("Short address: ");
  Serial.println (getShortAddress(), HEX);
  */

  // Return true if everything OK
  return true;
}


void DecaDuino::resetDW1000() {

/*
  uint32_t u32tmp;

  readSpi(DW1000_REGISTER_PMSC_CTRL0, buf, 4);
  u32tmp = decodeUint32(buf);

  // Set SYSCLKS bits to 01 and clear SOFTRESET bits
  u32tmp = 1 | ( u32tmp & 0x0FFFFFFC );
  encodeUint32( u32tmp, buf);
  writeSpi(DW1000_REGISTER_PMSC_CTRL0, buf, 4);

  // Clear SOFTRESET bits
  //encodeUint32( u32tmp & 0x0FFFFFFF, buf );
  //writeSpi(DW1000_REGISTER_PMSC_CTRL0, buf, 4);

  delay(100);

  // Set SOFTRESET bits
  u32tmp = 0xF0000000 | (u32tmp & 0xFFFFFFFC);
  encodeUint32( u32tmp, buf);
  writeSpi(DW1000_REGISTER_PMSC_CTRL0, buf, 4);
  delay(100);
*/
}


void DecaDuino::isr0() {

  Serial.println("isr0");
  if (_DecaDuinoInterrupt[DW1000_IRQ0_PIN]) _DecaDuinoInterrupt[DW1000_IRQ0_PIN]->handleInterrupt();
}


void DecaDuino::isr1() {

  Serial.println("isr1");
  if (_DecaDuinoInterrupt[DW1000_IRQ1_PIN]) _DecaDuinoInterrupt[DW1000_IRQ1_PIN]->handleInterrupt();
}


void DecaDuino::isr2() {

  Serial.println("isr2");
  if (_DecaDuinoInterrupt[DW1000_IRQ2_PIN]) _DecaDuinoInterrupt[DW1000_IRQ2_PIN]->handleInterrupt();
}

void DecaDuino::handleInterrupt() {

//}
//void DecaDuino::_my_handleInterrupt() {

  uint32_t statusReg;
  uint16_t frameLen;
  uint16_t i;

  readSpi(DW1000_REGISTER_SYS_STATUS, buf, 4);
  statusReg = decodeUint32(buf);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"SYS_STATUS=%08x\n", statusReg);
  Serial.println((char*)buf);
#endif

  //if ( statusReg & DW1000_REGISTER_SYS_STATUS_RXDFR_MASK ) { // RXDFR

    //if ( statusReg & DW1000_REGISTER_SYS_STATUS_RXFCG_MASK ) { // RXFCG

  //}
}


void DecaDuino::plmeDataRequest(uint8_t* buf, uint16_t len) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"I will send %dbyte(s)\n", len);
  Serial.println((char*)buf);
#endif

  // copy PSDU in tx buffer
  writeSpi(DW1000_REGISTER_TX_BUFFER, buf, len);

  // read tx frame control register
  readSpi(DW1000_REGISTER_TX_FCTRL, buf, 4);
  ui32t = decodeUint32(buf);
 
#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"TX_FCTRL=%08x\n", ui32t);
  Serial.println((char*)buf);
#endif

  // set frame length
  encodeUint32((ui32t & ~DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK) | len, buf);
  writeSpi(DW1000_REGISTER_TX_FCTRL, buf, 4);

  // set tx start bit
  readSpi(DW1000_REGISTER_SYS_CTRL, buf, 4);
  ui32t = decodeUint32(buf) | DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK;
  encodeUint32(ui32t, buf);
  writeSpi(DW1000_REGISTER_SYS_CTRL, buf, 4);

#ifdef DECADUINO_DEBUG 
  readSpi(DW1000_REGISTER_TX_FCTRL, buf, 4);
  ui32t = decodeUint32(buf);
  sprintf((char*)buf,"TX_FCTRL=%08x\n", ui32t);
  Serial.println((char*)buf);
#endif
}


void DecaDuino::readSpi(uint8_t address, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) ; // Mask register address (6bits) and preserve MSb at low (Read) and no subaddress

  digitalWrite(_slaveSelectPin, LOW);
  spi4teensy3::send(addr);
  spi4teensy3::receive(buf,len);
  digitalWrite(_slaveSelectPin, HIGH);
}


void DecaDuino::readSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x40; // Mask register address (6bits), preserve MSb at low (Read) and set subaddress present bit (0x40)
  uint8_t sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

  digitalWrite(_slaveSelectPin, LOW);
  spi4teensy3::send(addr);
  spi4teensy3::send(sub_addr);
  spi4teensy3::receive(buf,len);
  digitalWrite(_slaveSelectPin, HIGH);
}


void DecaDuino::writeSpi(uint8_t address, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x80; // Mask register address (6bits) and set MSb (Write) and no subaddress

  digitalWrite(_slaveSelectPin, LOW);
  spi4teensy3::send(addr);
  spi4teensy3::send(buf,len);
  digitalWrite(_slaveSelectPin, HIGH);
}


void DecaDuino::writeSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x80 | 0x40; // Mask register address (6bits), set MSb (Write) and set subaddress present bit (0x40)
  uint8_t sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

  digitalWrite(_slaveSelectPin, LOW);
  spi4teensy3::send(addr);
  spi4teensy3::send(sub_addr);
  spi4teensy3::send(buf,len);
  digitalWrite(_slaveSelectPin, HIGH);
}


uint16_t DecaDuino::decodeUint16 ( uint8_t *data ) {

  return 0 | (data[1] << 8) | data[0];
}


void DecaDuino::encodeUint16 ( uint16_t from, uint8_t *to ) {

  to[1] = (from & 0xFF00) >> 8;
  to[0] = from & 0xFF;
}


uint32_t DecaDuino::decodeUint32 ( uint8_t *data ) {

  return 0 | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}


void DecaDuino::encodeUint32 ( uint32_t from, uint8_t *to ) {

  to[3] = (from & 0xFF000000) >> 24;
  to[2] = (from & 0xFF0000) >> 16;
  to[1] = (from & 0xFF00) >> 8;
  to[0] = from & 0xFF;
}


uint16_t DecaDuino::getPanId() {

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
  return decodeUint16(buf);
}


void DecaDuino::setPanId(uint16_t panId) {

  encodeUint16(panId, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
}


uint16_t DecaDuino::getShortAddress() {

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
  return decodeUint16(buf);
}


void DecaDuino::setShortAddress(uint16_t shortAddress) {

  encodeUint16(shortAddress, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
}

