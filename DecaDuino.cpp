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

  uint8_t buf[8];
  uint16_t ui16t;
  uint32_t ui32t;

  // Initialise the IRQ and Slave Select pin
  pinMode(_interruptPin, INPUT);
  pinMode(_slaveSelectPin, OUTPUT);
  digitalWrite(_slaveSelectPin, HIGH);

  // Initialise the RX pointers
  rxDataAvailable = false;
  rxData = NULL;
  rxDataLen = NULL;

  // Wait for DW1000 POR (up to 5msec)
  delay(5);

#ifdef DECADUINO_DEBUG 
  delay(3000); // delay to see next messages on console for debug
#endif

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
  sprintf((char*)debugStr,"SYS_CFG=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  // System Event Mask Register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  ui32t |= DW1000_REGISTER_SYS_MASK_MRXFCG_MASK; // MRXFCG: interrupt when good frame (FCS OK) received
  writeSpiUint32(DW1000_REGISTER_SYS_MASK, ui32t);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"SYS_MASK=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)
  // Load the LDE algorithm microcode into LDE RAM (procedure p.22 DW1000 User Manual + comment p.21)
  // /!\ ceci utilise la mémoire OTP donc potentiellement destructeur. Implémenté mais
  // non testé : je préfère "disable LDE execution" plus bas
  //encodeUint16(0x0301, buf);
  //writeSpiSubAddress(0x36, 0, buf, 2);
  //encodeUint16(0x8000, buf);
  //writeSpiSubAddress(0x2D, 0x06, buf, 2);
  //delay(1);
  //encodeUint16(0x0200, buf);
  //writeSpiSubAddress(0x36, 0, buf, 2);

  // Disable LDE execution (clear LDERUNE)
  readSpiSubAddress(0x36, 4, buf, 4);
  ui32t = decodeUint32(buf);
  ui32t &= 0xFFFDFFFF; // clear bit17 (LDERUNE)
  encodeUint32(ui32t, buf);
  writeSpiSubAddress(0x36, 4, buf, 4);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"PMSC_CTRL1=%08x", ui32t);
  Serial.println((char*)debugStr);
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
  sprintf((char*)debugStr,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)debugStr);
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
  sprintf((char*)debugStr,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  // Set SOFTRESET bits
  ui32t |= 0xF0000000;
  ui32t &= 0xFFFFFFFC;
  writeSpiUint32(DW1000_REGISTER_PMSC_CTRL0, ui32t);
  delay(1);

  // Initialise the SPI port
  spi4teensy3::init(1,0,0); // Normal speed SPICLK after performing DW1000 reset
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK
  delay(1);

#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);
  sprintf((char*)debugStr,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif
}


void DecaDuino::isr0() {

#ifdef DECADUINO_DEBUG 
  //Serial.println("\n###isr0###");
#endif
  if (_DecaDuinoInterrupt[DW1000_IRQ0_PIN]) _DecaDuinoInterrupt[DW1000_IRQ0_PIN]->handleInterrupt();
}


void DecaDuino::isr1() {

#ifdef DECADUINO_DEBUG 
  //Serial.println("\n###isr1###");
#endif
  if (_DecaDuinoInterrupt[DW1000_IRQ1_PIN]) _DecaDuinoInterrupt[DW1000_IRQ1_PIN]->handleInterrupt();
}


void DecaDuino::isr2() {

#ifdef DECADUINO_DEBUG 
  //Serial.println("\n###isr2###");
#endif
  if (_DecaDuinoInterrupt[DW1000_IRQ2_PIN]) _DecaDuinoInterrupt[DW1000_IRQ2_PIN]->handleInterrupt();
}


void DecaDuino::handleInterrupt() {

  uint32_t sysStatusReg, ack, ui32t;
  uint16_t i;

  ack = 0;

  // Read System Event Status Register
  sysStatusReg = readSpiUint32(DW1000_REGISTER_SYS_STATUS);

  // If IRQS is cleared, no enabled interrupt (SYS_MASK) have assert the IRQ pin: exit
  if ( ! ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_IRQS_MASK ) )
    return;

#ifdef DECADUINO_DEBUG 
  Serial.print("\n###isr### ");
  //ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  //sprintf((char*)debugStr,"SYS_MASK  =%08x", ui32t);
  //Serial.println((char*)debugStr);
  sprintf((char*)debugStr,"SYS_STATUS=%08x ", sysStatusReg);
  Serial.print((char*)debugStr);
#endif

  // Checking RX frame interrupt
  if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_RXDFR_MASK ) { // RXDFR

#ifdef DECADUINO_DEBUG 
    Serial.print("RXDFR ");
#endif

    // Good frame
    if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_RXFCG_MASK ) { // RXFCG

#ifdef DECADUINO_DEBUG 
      Serial.print("RXFCG ");
#endif

      if ( rxData == NULL ) {

#ifdef DECADUINO_DEBUG 
      Serial.print("Error: no RX buffer set");
#endif

      } else {

        // get frame length
        ui32t = (readSpiUint32(DW1000_REGISTER_RX_FINFO) & DW1000_REGISTER_RX_FINFO_RXFLEN_MASK) - 2; // FCS is 2-bytes long. Avoid it in the len.
        *rxDataLen = (uint16_t)ui32t;
#ifdef DECADUINO_DEBUG 
        sprintf((char*)debugStr,"length=%dbytes ", *rxDataLen);
        Serial.print((char*)debugStr);
#endif
        // get frame data
        readSpi(DW1000_REGISTER_RX_BUFFER, rxData, *rxDataLen);
        rxDataAvailable = true;
      }
      // Clearing the RXFCG bit (it clears the interrupt if enabled)
      ack |= DW1000_REGISTER_SYS_STATUS_RXFCG_MASK;
    }

    // Bad frame (FCS error)
    if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_RXFCE_MASK ) { // RXFCE

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
  writeSpiUint32(DW1000_REGISTER_SYS_STATUS, ack);

#ifdef DECADUINO_DEBUG 
      Serial.println();
#endif
}


uint8_t DecaDuino::plmeDataRequest(uint8_t* buf, uint16_t len) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"Request to send %dbyte(s) ", len);
  Serial.print((char*)debugStr);
#endif

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // copy PSDU in tx buffer
    writeSpi(DW1000_REGISTER_TX_BUFFER, buf, len);

    // read tx frame control register
    ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);

    // set frame length
    ui32t = (ui32t & ~DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK) | len+2; // FCS is 2-bytes long
    writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

    // set tx start bit
    writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK);
  }

#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
  sprintf((char*)debugStr,"TX_FCTRL=%08x\n", ui32t);
  Serial.print((char*)debugStr);
#endif

  return true;
}


uint8_t DecaDuino::send(uint8_t* buf, uint16_t len) {

  plmeDataRequest(buf, len);
}


void DecaDuino::setRxBuffer(uint8_t* buf, uint16_t *len) {

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"Setting RX buffer address to 0x%08x", buf);
  Serial.println((char*)debugStr);
#endif

  rxData = buf;
  rxDataLen = len;
}


void DecaDuino::plmeRxEnableRequest(void) {

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"RX enable request");
  Serial.println((char*)debugStr);
#endif

  // set rx enable bit in system control register
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_RXENAB_MASK);
}


void DecaDuino::plmeRxEnableRequest(uint8_t* buf, uint16_t *len) {

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"RX enable request with address buffer");
  Serial.println((char*)debugStr);
#endif

  setRxBuffer(buf, len);
  plmeRxEnableRequest(); 
}


void DecaDuino::plmeRxDisableRequest(void) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"RX disable request");
  Serial.println((char*)debugStr);
#endif

  // set transceiver off bit in system control register
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_TRXOFF_MASK);
}


uint8_t DecaDuino::rxFrameAvailable(void) {

  if ( rxDataAvailable ) {
    rxDataAvailable = false;
    return true;
  }
  else return false;
}


uint8_t DecaDuino::rxFrameAvailable(uint8_t* buf, uint16_t *len) {

  uint16_t i;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if ( rxDataAvailable ) {
      for (i=0; i<*rxDataLen; i++)
        buf[i] = rxData[i];
      len = rxDataLen;
      rxDataAvailable = false;
      return true;
    }
  }
  return false;
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


void DecaDuino::readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr, sub_addr, sub_sub_addr;

  addr = 0 | (address & 0x3F) | 0x40; // Mask register address (6bits), preserve MSb at low (Read) and set subaddress present bit (0x40)

  if ( subAddress < 128 ) {

    // This is a 2-bytes header SPI transaction

    sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      digitalWrite(_slaveSelectPin, LOW);
      spi4teensy3::send(addr);
      spi4teensy3::send(sub_addr);
      spi4teensy3::receive(buf,len);
      digitalWrite(_slaveSelectPin, HIGH); 
    }

  } else {

    // This is a 3-bytes header SPI transaction
    /** @todo implement readSpiSubAddress in case of a 3-bytes header SPI transaction */
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


void DecaDuino::writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

  uint8_t addr, sub_addr, sub_sub_addr;

  addr = 0 | (address & 0x3F) | 0x80 | 0x40; // Mask register address (6bits), set MSb (Write) and set subaddress present bit (0x40)

  if ( subAddress < 128 ) {

    // This is a 2-bytes header SPI transaction

    sub_addr = 0 | (subAddress & 0x3F); // Mask register address (6bits)

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      digitalWrite(_slaveSelectPin, LOW);
      spi4teensy3::send(addr);
      spi4teensy3::send(sub_addr);
      spi4teensy3::send(buf,len);
      digitalWrite(_slaveSelectPin, HIGH);
    }

  } else {

    // This is a 3-bytes header SPI transaction
    /** @todo implement writeSpiSubAddress in case of a 3-bytes header SPI transaction */
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

  uint8_t buf[2];

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
  return decodeUint16(buf);
}


uint16_t DecaDuino::getShortAddress() {

  uint8_t buf[2];

  readSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
  return decodeUint16(buf);
}


uint64_t DecaDuino::getEuid() {

  uint8_t buf[8];

  readSpi(DW1000_REGISTER_EUI, buf, 8);
  return decodeUint64(buf);
}


void DecaDuino::setPanId(uint16_t panId) {

  uint8_t buf[2];

  encodeUint16(panId, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_PANID_OFFSET, buf, 2);
}


void DecaDuino::setShortAddress(uint16_t shortAddress) {

  uint8_t buf[2];

  encodeUint16(shortAddress, buf);
  writeSpiSubAddress(DW1000_REGISTER_PANADR, DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET, buf, 2);
}

