// DecaDuino.cpp
//
// DecaWave DW1000 driver for Arduino
// See the README file in this directory for documentation

#include "DecaDuino.h"
#include <util/atomic.h>

DecaDuino* DecaDuino::_DecaDuinoInterrupt[MAX_NB_DW1000_FOR_INTERRUPTS] = {0, 0, 0};

DecaDuino::DecaDuino(uint8_t slaveSelectPin, uint8_t interruptPin) {

  _slaveSelectPin = slaveSelectPin;
  _interruptPin = interruptPin;
}


boolean DecaDuino::init() {

  uint16_t ui16t;
  uint32_t ui32t;

  // Initialise the IRQ and Slave Select pin
  pinMode(_interruptPin, INPUT);
  pinMode(_slaveSelectPin, OUTPUT);
  digitalWrite(_slaveSelectPin, HIGH);

  // Initialise buffer
  for (ui16t=0; ui16t<BUFFER_MAX_LEN; ui16t++)
    buf[ui16t] = 0;

  // Wait for DW1000 POR (up to 5msec)
  delay(5);

  // Reset the DW1000 now
  resetDW1000();

  // Check the device type
  if ( readSpiUint32(DW1000_REGISTER_DEV_ID) != 0xdeca0130 ) return false;

  // Load Extended Unique Identifier – the 64-bit IEEE device address - in memory
  euid = getEuid();

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


  // --- Configure DW1000 -----------------------------------------------------------------------------------------

  // System Configuration Register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_CFG);
  ui32t |= DW1000_REGISTER_SYS_CFG_RXAUTR_MASK; // RXAUTR: Receiver Auto-Re-enable after a RX failure
  writeSpiUint32(DW1000_REGISTER_SYS_CFG,ui32t);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"SYS_CFG=%08x", ui32t);
  Serial.println((char*)buf);
#endif

  // System Event Mask Register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  ui32t |= DW1000_REGISTER_SYS_MASK_MRXFCG_MASK; // MRXFCG: interrupt when good frame (FCS OK) received
  writeSpiUint32(DW1000_REGISTER_SYS_MASK, ui32t);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"SYS_MASK=%08x", ui32t);
  Serial.println((char*)buf);
#endif

  // --- End of DW1000 configuration ------------------------------------------------------------------------------

  // Return true if everything OK
  return true;
}


void DecaDuino::resetDW1000() {

  uint32_t ui32t;

  // Initialise the SPI port
  spi4teensy3::init(5,0,0); // Low speed SPICLK for performing DW1000 reset
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK

  // Getting PMSC_CTRL0 register
  ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)buf);
#endif

  // Set SYSCLKS bits to 01
  ui32t = ( ui32t & 0xFFFFFFFC ) | 1;
  writeSpiUint32(DW1000_REGISTER_PMSC_CTRL0, ui32t);
  delay(1);

  // Clear SOFTRESET bits
  ui32t &= 0x0FFFFFFF;
  writeSpiUint32(DW1000_REGISTER_PMSC_CTRL0, ui32t);
  delay(1);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)buf);
#endif

  // Set SOFTRESET bits
  ui32t |= 0xF0000000;
  ui32t &= 0xFFFFFFFC;
  writeSpiUint32(DW1000_REGISTER_PMSC_CTRL0, ui32t);
  delay(1);

  // Initialise the SPI port
  spi4teensy3::init(1,0,0); // Normal speed SPICLK for performing DW1000 reset
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK
  delay(1);

#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);
  sprintf((char*)buf,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)buf);
#endif
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

  uint8_t buf[BUFFER_MAX_LEN];
  uint32_t statusReg, ack;
  uint16_t frameLen;
  uint16_t i;

  ack = 0;

  // Read System Event Status Register
  statusReg = readSpiUint32(DW1000_REGISTER_SYS_STATUS);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"SYS_STATUS=%08x ", statusReg);
  Serial.print((char*)buf);
#endif

  // Checking RX frame interrupt
  if ( statusReg & DW1000_REGISTER_SYS_STATUS_RXDFR_MASK ) { // RXDFR

#ifdef DECADUINO_DEBUG 
    Serial.print("RXDFR ");
#endif

    // Good frame
    if ( statusReg & DW1000_REGISTER_SYS_STATUS_RXFCG_MASK ) { // RXFCG

#ifdef DECADUINO_DEBUG 
      Serial.print("RXFCG ");
#endif
      // get frame length
      readSpi(DW1000_REGISTER_RX_FINFO, buf, 2);
      frameLen = decodeUint16(buf) & DW1000_REGISTER_RX_FINFO_RXFLEN_MASK;
#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"length=%dbytes |", frameLen);
  Serial.print((char*)buf);
#endif

      // get frame data
      readSpi(DW1000_REGISTER_RX_BUFFER, buf, frameLen);
#ifdef DECADUINO_DEBUG
      for (i=0; i<frameLen; i++) { 
        sprintf((char*)buf,"%02x|", buf[i]);
        Serial.print((char*)buf);
      }
#endif

      // Clearing the RXFCG bit (it clears the interrupt if enabled)
      ack |= DW1000_REGISTER_SYS_STATUS_RXFCG_MASK;
    }

    // Bad frame (FCS error)
    if ( statusReg & DW1000_REGISTER_SYS_STATUS_RXFCE_MASK ) { // RXFCE

#ifdef DECADUINO_DEBUG 
      Serial.println("RXFCG (FCS error)");
#endif
      // Clearing the RXFCG bit (it clears the interrupt if enabled)
      ack |= DW1000_REGISTER_SYS_STATUS_RXFCE_MASK;
    }

    // Clearing the RXDFR bit (it clears the interrupt if enabled)
    ack |= DW1000_REGISTER_SYS_STATUS_RXDFR_MASK;
  }

  // Acknoledge by writing '1' in all set bits in the System Event Status Register
  //writeSpiUint32(DW1000_REGISTER_SYS_STATUS, statusReg);
  writeSpiUint32(DW1000_REGISTER_SYS_STATUS, ack);
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
  ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
 
#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"TX_FCTRL=%08x\n", ui32t);
  Serial.println((char*)buf);
#endif

  // set frame length
  ui32t = (ui32t & ~DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK) | len;
  writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

  // set tx start bit
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_CTRL);
  ui32t |= DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK;
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, ui32t);

#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
  ui32t = decodeUint32(buf);
  sprintf((char*)buf,"TX_FCTRL=%08x\n", ui32t);
  Serial.println((char*)buf);
#endif
}


void DecaDuino::plmeRxEnableRequest(void) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"RX enable request");
  Serial.println((char*)buf);
#endif

  // set rx enable bit in system control register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_CTRL);
  ui32t |= DW1000_REGISTER_SYS_CTRL_RXENAB_MASK;
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, ui32t);
}


void DecaDuino::plmeRxDisableRequest(void) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)buf,"RX disable request");
  Serial.println((char*)buf);
#endif

  // set transceiver off bit in system control register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_CTRL);
  ui32t |= DW1000_REGISTER_SYS_CTRL_TRXOFF_MASK;
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, ui32t);
}


void DecaDuino::readSpi(uint8_t address, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) ; // Mask register address (6bits) and preserve MSb at low (Read) and no subaddress

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(_slaveSelectPin, LOW);
    spi4teensy3::send(addr);
    spi4teensy3::receive(buf,len);
    digitalWrite(_slaveSelectPin, HIGH);
  }
}


void DecaDuino::readSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x40; // Mask register address (6bits), preserve MSb at low (Read) and set subaddress present bit (0x40)
  uint8_t sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(_slaveSelectPin, LOW);
    spi4teensy3::send(addr);
    spi4teensy3::send(sub_addr);
    spi4teensy3::receive(buf,len);
    digitalWrite(_slaveSelectPin, HIGH); 
  }
}


uint32_t DecaDuino::readSpiUint32(uint8_t address) {

  uint8_t buf[4];

  readSpi(address, buf, 4);
  return decodeUint32(buf);
}


void DecaDuino::writeSpi(uint8_t address, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x80; // Mask register address (6bits) and set MSb (Write) and no subaddress

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(_slaveSelectPin, LOW);
    spi4teensy3::send(addr);
    spi4teensy3::send(buf,len);
    digitalWrite(_slaveSelectPin, HIGH);
  }
}


void DecaDuino::writeSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr = 0 | (address & 0x3F) | 0x80 | 0x40; // Mask register address (6bits), set MSb (Write) and set subaddress present bit (0x40)
  uint8_t sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(_slaveSelectPin, LOW);
    spi4teensy3::send(addr);
    spi4teensy3::send(sub_addr);
    spi4teensy3::send(buf,len);
    digitalWrite(_slaveSelectPin, HIGH);
  }
}


void DecaDuino::writeSpiUint32(uint8_t address, uint32_t ui32t) {

  uint8_t buf[4];

  encodeUint32(ui32t, buf);
  writeSpi(address, buf, 4);
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


uint64_t DecaDuino::decodeUint64 ( uint8_t *data ) {

  return 0 | (data[7] << 56) | (data[6] << 48) | (data[5] << 40) | (data[4] << 32) | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}


void DecaDuino::encodeUint64 ( uint64_t from, uint8_t *to ) {

  to[7] = (from & 0xFF00000000000000) >> 56;
  to[6] = (from & 0xFF000000000000) >> 48;
  to[5] = (from & 0xFF0000000000) >> 40;
  to[4] = (from & 0xFF00000000) >> 32;
  to[3] = (from & 0xFF000000) >> 24;
  to[2] = (from & 0xFF0000) >> 16;
  to[1] = (from & 0xFF00) >> 8;
  to[0] = from & 0xFF;
}


uint16_t DecaDuino::getPanId() {

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
  return decodeUint16(buf);
}


uint16_t DecaDuino::getShortAddress() {

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
  return decodeUint16(buf);
}


uint64_t DecaDuino::getEuid() {

  readSpi(DW1000_REGISTER_EUI, buf, 8);
  return decodeUint64(buf);
}


void DecaDuino::setPanId(uint16_t panId) {

  encodeUint16(panId, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
}


void DecaDuino::setShortAddress(uint16_t shortAddress) {

  encodeUint16(shortAddress, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
}

