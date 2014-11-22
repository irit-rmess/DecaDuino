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


boolean DecaDuino::init(uint32_t shorAddrPANID) {

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
ui32t |= 0x0000003D;//frame filtering
  writeSpiUint32(DW1000_REGISTER_SYS_CFG,ui32t);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"SYS_CFG=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  // System Event Mask Register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  ui32t |= DW1000_REGISTER_SYS_MASK_MRXFCG_MASK; // MRXFCG: interrupt when good frame (FCS OK) received
  ui32t |= DW1000_REGISTER_SYS_MASK_MTXFRS_MASK;
  writeSpiUint32(DW1000_REGISTER_SYS_MASK, ui32t);

//set short addr and PANID

 writeSpiUint32(0x03, shorAddrPANID);
shorAddrPANID = readSpiUint32(0x03);
 sprintf((char*)debugStr,"PANID=%08x", shorAddrPANID);
  Serial.println((char*)debugStr);

//uint8_t val[2]={0x7C,0xCD};
//writeSpi(0x18,val,2);


#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"SYS_MASK=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

//val[0]=0;
//val[1]=0;


//readSpi(0x18,val,2);
//sprintf((char*)debugStr,"TX_ANTD=%04x", val);
//  Serial.println((char*)debugStr);

  //ui16t = 33000;
  ui16t = 32870;
  encodeUint16(ui16t, buf);
  writeSpi(0x18, buf, 2);


  // --- End of DW1000 configuration ------------------------------------------------------------------------------

  lastTxOK = false;

  // Return true if everything OK
  return true;

}//end of init()


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
//ui32t |= 0x0000003D;//frame filtering
  writeSpiUint32(DW1000_REGISTER_SYS_CFG,ui32t);

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"SYS_CFG=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  // System Event Mask Register
  ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  ui32t |= DW1000_REGISTER_SYS_MASK_MRXFCG_MASK; // MRXFCG: interrupt when good frame (FCS OK) received
  ui32t |= DW1000_REGISTER_SYS_MASK_MTXFRS_MASK;
  writeSpiUint32(DW1000_REGISTER_SYS_MASK, ui32t);



//uint8_t val[2]={0x7C,0xCD};
//writeSpi(0x18,val,2);


#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"SYS_MASK=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

//val[0]=0;
//val[1]=0;


//readSpi(0x18,val,2);
//sprintf((char*)debugStr,"TX_ANTD=%04x", val);
//  Serial.println((char*)debugStr);

  //ui16t = 33000;
  ui16t = 32870;
  encodeUint16(ui16t, buf);
  writeSpi(0x18, buf, 2);


  // --- End of DW1000 configuration ------------------------------------------------------------------------------

  lastTxOK = false;

  // Return true if everything OK
  return true;

}//end of init()


void DecaDuino::resetDW1000() {

  uint8_t buf[8];
  uint32_t ui32t;

  // Initialise the SPI port
  spi4teensy3::init(4,0,0); // Low speed SPICLK for performing DW1000 reset
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK

  // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)
  // Load the LDE algorithm microcode into LDE RAM (procedure p.22 DW1000 User Manual + comment p.21)
/*
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xF6;
    buf[1] = 0x00;
    buf[2] = 0x01;
    buf[3] = 0x03;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xED;
    buf[1] = 0x06;
    buf[2] = 0x00;
    buf[3] = 0x80;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);

    delayMicroseconds(160);

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xF6;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x02;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);
  }
*/
  delay(100);

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

  delay(5);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xF6;
    buf[1] = 0x00;
    buf[2] = 0x01;
    buf[3] = 0x03;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xED;
    buf[1] = 0x06;
    buf[2] = 0x00;
    buf[3] = 0x80;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);

    delayMicroseconds(160);

    digitalWrite(_slaveSelectPin, LOW);
    buf[0] = 0xF6;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x02;
    spi4teensy3::send(buf,4);
    digitalWrite(_slaveSelectPin, HIGH);
  }

  // Initialise the SPI port
  //spi4teensy3::init(1,0,0); // Normal speed SPICLK after performing DW1000 reset
  spi4teensy3::init(2,0,0); // Normal speed SPICLK after performing DW1000 reset
  CORE_PIN13_CONFIG = PORT_PCR_MUX(1); // First reassign pin 13 to Alt1 so that it is not SCK but the LED still works
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // and then reassign pin 14 to SCK
  delay(1);

#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);
  sprintf((char*)debugStr,"PMSC_CTRL0=%08x", ui32t);
  Serial.println((char*)debugStr);
#endif

  trxStatus = DW1000_TRX_STATUS_IDLE;
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

  uint8_t buf[8];
  uint16_t i;
  uint32_t sysStatusReg, ack, ui32t;
  uint64_t ui64t;

  ack = 0;

  // Read System Event Status Register
  sysStatusReg = readSpiUint32(DW1000_REGISTER_SYS_STATUS);

  // If IRQS is cleared, no enabled interrupt (SYS_MASK) have assert the IRQ pin: exit
  if ( ! ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_IRQS_MASK ) )
    return;

#ifdef DECADUINO_DEBUG 
 // Serial.print("\n###isr### ");
  //ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
  //sprintf((char*)debugStr,"SYS_MASK  =%08x", ui32t);
  //Serial.println((char*)debugStr);
 //sprintf((char*)debugStr,"SYS_STATUS=%08x ", sysStatusReg);
//  Serial.print((char*)debugStr);
#endif

  // Checking RX frame interrupt
  if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_RXDFR_MASK ) { // RXDFR

    trxStatus = DW1000_TRX_STATUS_IDLE;

#ifdef DECADUINO_DEBUG 
   // Serial.print("RXDFR ");
#endif

    // Good frame
    if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_RXFCG_MASK ) { // RXFCG

#ifdef DECADUINO_DEBUG 
      //Serial.print("RXFCG ");
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
        //sprintf((char*)debugStr,"length=%dbytes ", *rxDataLen);
       // Serial.print((char*)debugStr);
#endif
        // get frame data
        if ( rxDataLenMax != 0 ) {
          // Put frame data at the end of the buffer
          readSpi(DW1000_REGISTER_RX_BUFFER, rxData+rxDataLenMax-*rxDataLen, *rxDataLen);
        } else {
          // Put frame data at the begin of the buffer
          readSpi(DW1000_REGISTER_RX_BUFFER, rxData, *rxDataLen);
        }
       // rxDataAvailable = true;

        if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_LDEDONE_MASK ) {
          encodeUint64(0, buf); // init buffer
          readSpi(0x15, buf, 5);
          lastRxTimestamp = decodeUint64(buf);
#ifdef DECADUINO_DEBUG 
          sprintf((char*)debugStr, "\nRX Frame timestamp %08x %08x\n", decodeUint32(&buf[4]), decodeUint32(buf));
          Serial.println((char*)debugStr);
#endif
          readSpi(0x14, buf, 3);
          ui32t = decodeUint32(buf) & 0x0007FFFF;
          if ( ui32t & 0x00080000 ) ui32t |= 0xFFF80000;		  

	//***************** Clock offset
	// Serial.print("RXTOFS=0x");
			  //Serial.println(ui32t, HEX);
			  ui32t = 0x01F00000/ui32t;
	//RD032014
	double rxtofs, rxttcki;
	rxttcki=32505856;

	//turn rxtofs to a signed double value
	if(buf[2] & 0x04){//rxtofs is negative
	//Serial.println("rxtofs is negative");
		buf[2] |=0xF8;
		buf[2]=~buf[2];
		buf[1]=~buf[1];
		buf[0]=~buf[0];

	rxtofs=buf[2]*256*256+buf[1]*256+buf[0];
	
		rxtofs=rxtofs+1;
		rxtofs=rxtofs*-1;
	}else{
		//Serial.println("rxtofs is positive");
	rxtofs=buf[2]*256*256+buf[1]*256+buf[0];
	}

	double offseti=rxtofs*1000000/rxttcki;
	clkOffset=offseti;
  rxDataAvailable = true;
	//  Serial.print("clock offset=");
	 // Serial.println(ui32t, HEX);		  
	//Serial.println(offseti);

		 // Serial.print("RXTOFS=0x");
		//  Serial.println(ui32t, HEX);
		 // ui32t = 0x01F00000/ui32t;
		//  Serial.print("clock offset=0x");
		//  Serial.println(ui32t, HEX);	
		  

        }
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

  // Manage TX completion interrupt
  if ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_TXFRS_MASK ) { // TXFRS

    trxStatus = DW1000_TRX_STATUS_IDLE;

    // lastTxOK = true; moved after reading TX timestamp

#ifdef DECADUINO_DEBUG
    //Serial.println("LastTx OK");
#endif

    // Read TX timestamp
    encodeUint64(0, buf); // init buffer
    readSpi(0x17, buf, 5);
    lastTxTimestamp = decodeUint64(buf);
//Serial.print("\nLAST TX ");
//print(lastTxTimestamp);

    lastTxOK = true;
 
#ifdef DECADUINO_DEBUG 
    sprintf((char*)debugStr, "\nTX Frame timestamp %08x %08x\n", decodeUint32(&buf[4]), decodeUint32(buf));
    Serial.print((char*)debugStr);
#endif

    ack |= DW1000_REGISTER_SYS_STATUS_TXFRS_MASK;
  }

  // Acknoledge by writing '1' in all set bits in the System Event Status Register
  writeSpiUint32(DW1000_REGISTER_SYS_STATUS, ack);

#ifdef DECADUINO_DEBUG 
  Serial.println();
#endif
}


bool DecaDuino::hasTxSucceeded() {

  return lastTxOK;
}


float DecaDuino::rangeNode(uint64_t destination, uint8_t protocol=DEFAULT_RANGING_PROTOCOL) {

  switch(protocol) {

    case RANGING_PROTOCOL_TWR:
      twrRequest(destination);
      break;

    case RANGING_PROTOCOL_SDS_TWR:
      sdsTwrRequest(destination);
      break;

    default:
#ifdef DECADUINO_DEBUG 
      Serial.println("rangeNode(): unknown ranging protocol");
#endif
      break;
  }
}


uint8_t DecaDuino::twrRequest(uint64_t destination) {

  // ToDo
}


uint8_t DecaDuino::sdsTwrRequest(uint64_t destination) {

  // ToDo
}


void DecaDuino::rangingEngine(void) {

  // ToDo
}

uint8_t DecaDuino::plmeDataRequest(uint8_t* buf, uint16_t len) {

  plmeDataRequest(buf, len, false, 0);
}

uint8_t DecaDuino::plmeDataRequest(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time) {

  uint32_t ui32t;
  uint64_t ui64t;
  uint8_t tempbuf[8];

#ifdef DECADUINO_DEBUG 
 /* sprintf((char*)debugStr,"Request to send %dbyte(s)\n ", len);
  Serial.print((char*)debugStr);*/

for(int q=0;q<len;q++){
  sprintf((char*)debugStr," %02X ", buf[q]);
  Serial.print((char*)debugStr);

}
#endif

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    trxStatus = DW1000_TRX_STATUS_TX;

    // copy PSDU in tx buffer
    writeSpi(DW1000_REGISTER_TX_BUFFER, buf, len);

    // read tx frame control register
    ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);

    // set frame length
    ui32t = (ui32t & ~DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK) | len+2; // FCS is 2-bytes long
    writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

    if ( delayed ) { // if delayed transmission
      // send time
      encodeUint64 ( time & 0x000000FFFFFFFE00, tempbuf); // time is 5-bytes long, 9 lsb=0
      writeSpi(DW1000_REGISTER_DX_TIME, tempbuf, 5);
      
      // set tx start bit and Transmitter Delayed Sendind bit
      writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK | DW1000_REGISTER_SYS_CTRL_TXDLYS_MASK);

    } else {
      // set tx start bit
      writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK);
    }

    lastTxOK = false;
  }

/*
#ifdef DECADUINO_DEBUG 
  ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
  sprintf((char*)debugStr,"TX_FCTRL=%08x\n", ui32t);
  Serial.print((char*)debugStr);
#endif
*/

  return true;
}


uint8_t DecaDuino::send(uint8_t* buf, uint16_t len) {

  plmeDataRequest(buf, len);
}


uint8_t DecaDuino::send(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time) {

  plmeDataRequest(buf, len, delayed, time);
}


uint64_t DecaDuino::predictT5(){

	uint64_t t5;
	uint8_t buf[8];

	encodeUint64(0, buf); // init buffer
	readSpi(0x06, buf, 5);
	t5 = decodeUint64(buf);

	return t5;
}





void DecaDuino::setRxBuffer(uint8_t* buf, uint16_t *len) {

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"Setting RX buffer address to 0x%08x", buf);
  Serial.println((char*)debugStr);
#endif

  rxData = buf;
  rxDataLen = len;
  rxDataLenMax = 0;
}


void DecaDuino::setRxBuffer(uint8_t* buf, uint16_t *len, uint16_t max) {

  setRxBuffer(buf, len);
  rxDataLenMax = max;
}


void DecaDuino::plmeRxEnableRequest(void) {

#ifdef DECADUINO_DEBUG 
 sprintf((char*)debugStr,"RX enable request");
  Serial.println((char*)debugStr);
#endif

  // set rx enable bit in system control register
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_RXENAB_MASK);

  trxStatus = DW1000_TRX_STATUS_RX;
}


void DecaDuino::plmeRxEnableRequest(uint16_t max) {

  plmeRxEnableRequest();
  rxDataLenMax = max;
}


void DecaDuino::plmeRxEnableRequest(uint8_t* buf, uint16_t *len) {

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"RX enable request with address buffer");
  Serial.println((char*)debugStr);
#endif

  setRxBuffer(buf, len);
  plmeRxEnableRequest(); 
}


void DecaDuino::plmeRxEnableRequest(uint8_t* buf, uint16_t *len, uint16_t max) {

  plmeRxEnableRequest(buf, len);
  rxDataLenMax = max;
}


void DecaDuino::plmeRxDisableRequest(void) {

  uint32_t ui32t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"RX disable request");
  Serial.println((char*)debugStr);
#endif

  // set transceiver off bit in system control register (go to idle mode)
  writeSpiUint32(DW1000_REGISTER_SYS_CTRL, DW1000_REGISTER_SYS_CTRL_TRXOFF_MASK);

  trxStatus = DW1000_TRX_STATUS_IDLE;
}


uint8_t DecaDuino::rxFrameAvailable(void) {

  if ( rxDataAvailable ) {
    rxDataAvailable = false;
    return true;
  }
  else return false;
}


uint8_t DecaDuino::rxFrameAvailable(uint8_t* buf, uint16_t *len) {

  return rxFrameAvailable(buf, len, 0);
}


uint8_t DecaDuino::rxFrameAvailable(uint8_t* buf, uint16_t *len, uint16_t max) {

  uint16_t i;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if ( rxDataAvailable ) {
      if ( max == 0 ) {
        for (i=0; i<*rxDataLen; i++)
          buf[i] = rxData[i];
      } else {
        for (i=0; i<*rxDataLen; i++)
          buf[i+max-*rxDataLen] = rxData[i];
      }
      *len = *rxDataLen;
      rxDataAvailable = false;
      return true;
    }
  }
  return false;
}


uint8_t DecaDuino::getTrxStatus(void) {

  return trxStatus;
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

  if ( subAddress < 0x80 ) {

    // This is a 2-bytes header SPI transaction

    sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

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

  if ( subAddress < 0x80 ) {

    // This is a 2-bytes header SPI transaction

    sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

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

	uint64_t tmp=0;
	tmp=decodeUint32(&data[4]);// | decodeUint32(data);
	tmp=tmp<<32;
	tmp=tmp | decodeUint32(data);
	
	return tmp;
	//  return 0 | (data[7] << 56) | (data[6] << 48) | (data[5] << 40) | (data[4] << 32) | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}

uint64_t DecaDuino::decodeUint64_2 ( uint8_t *data ) {

  return 0 |  (data[4] << 32) | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
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


void DecaDuino::printUint64 ( uint64_t ui64 ) {

  uint8_t buf[8];
  uint8_t debugStr[18];

  encodeUint64(ui64, buf);
  
  sprintf((char*)debugStr, "%08x%08x", decodeUint32(&buf[4]), decodeUint32(buf));
  Serial.print((char*)debugStr);
}


void DecaDuino::getSystemTimeCounter ( uint64_t *p ) {

  uint8_t buf[8];

  encodeUint64(0, buf); // init buffer
  readSpi(DW1000_REGISTER_SYS_TIME, buf, 5);
  *p = decodeUint64(buf);
}


uint64_t DecaDuino::getSystemTimeCounter ( void ) {

  uint64_t p;

  getSystemTimeCounter(&p);

  return p;
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


uint8_t DecaDuino::getTemperatureRaw() {

  uint8_t u8t;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    u8t = 0x80; writeSpiSubAddress(0x28, 0x11, &u8t, 1); // 1. Write Sub-Register 28:11 1byte 0x80
    u8t = 0x0A; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 2. Write Sub-Register 28:12 1byte 0x0A
    u8t = 0x0F; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 3. Write Sub-Register 28:12 1byte 0x0F
    u8t = 0x01; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 4. Write Register 2A:00 1byte 0x01
    u8t = 0x00; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 5. Write Register 2A:00 1byte 0x00
    readSpiSubAddress(0x2A, 0x04, &u8t, 1); // 6. Read Register 2A:04 1byte 8 bit Temperature reading
  }

  return u8t;
}


uint8_t DecaDuino::getVoltageRaw() {

  uint8_t u8t;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    
    u8t = 0x80; writeSpiSubAddress(0x28, 0x11, &u8t, 1); // 1. Write Sub-Register 28:11 1byte 0x80
    u8t = 0x0A; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 2. Write Sub-Register 28:12 1byte 0x0A
    u8t = 0x0F; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 3. Write Sub-Register 28:12 1byte 0x0F
    u8t = 0x01; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 4. Write Register 2A:00 1byte 0x01
    u8t = 0x00; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 5. Write Register 2A:00 1byte 0x00
    readSpiSubAddress(0x2A, 0x03, &u8t, 1); // 6. Read Register 2A:03 1byte 8 bit Voltage reading
  }

  return u8t;
}


float DecaDuino::getTemperature(void) {

  // Temperature (°C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23°C )) x 1.14) + 23
  // Todo: what is OTP_READ(Vtemp @ 23°C ) ?

  return 0;
}


float DecaDuino::getVoltage(void) {

  // Voltage (volts) = (SAR_LVBAT- (OTP_READ(Vmeas @ 3.3 V )) /173) + 3.3
  // Todo: what is OTP_READ(Vmeas @ 3.3 V ) ?

  return 0;
}


void DecaDuino::sleepRequest(void) {

  uint8_t ui8t;

#ifdef DECADUINO_DEBUG 
  sprintf((char*)debugStr,"sleep request");
  Serial.println((char*)debugStr);
#endif

  readSpiSubAddress(DW1000_REGISTER_AON_CFG0, DW1000_REGISTER_OFFSET_AON_CFG0, &ui8t, 1);
  ui8t |= DW1000_REGISTER_AON_CFG0_SLEEP_EN_MASK;
  writeSpiSubAddress(DW1000_REGISTER_AON_CFG0, DW1000_REGISTER_OFFSET_AON_CFG0, &ui8t, 1);

  readSpiSubAddress(DW1000_REGISTER_AON_CTRL, DW1000_REGISTER_OFFSET_AON_CTRL, &ui8t, 1);
  ui8t |= DW1000_REGISTER_AON_CTRL_UPL_CFG_MASK;
  writeSpiSubAddress(DW1000_REGISTER_AON_CTRL, DW1000_REGISTER_OFFSET_AON_CTRL, &ui8t, 1);
  delay(1);

  // The DWM1000 is now sleepping

  trxStatus = DW1000_TRX_STATUS_SLEEP;
}


void DecaDuino::deepsleepRequest(void) {

}


void DecaDuino::wakeRequest(void) {

}


void DecaDuino::test(void) {

}

void DecaDuino::print(uint64_t val) {
uint16_t hi16=(uint16_t)(val >> 32);


sprintf((char*)debugStr, "%02X",hi16);
  Serial.print((char*)debugStr);

sprintf((char*)debugStr, "%08x",val);
  Serial.println((char*)debugStr);
}


