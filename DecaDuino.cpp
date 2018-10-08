// DecaDuino.cpp
//
// Another DecaWave DW1000 driver for Arduino
// See the README file in this directory for documentation

#include <SPI.h>
#include "DecaDuino.h"
#include <util/atomic.h>

DecaDuino* DecaDuino::_DecaDuinoInterrupt[MAX_NB_DW1000_FOR_INTERRUPTS] = {0, 0, 0};

DecaDuino::DecaDuino(uint8_t slaveSelectPin, uint8_t interruptPin) {

	_slaveSelectPin = slaveSelectPin;
	_interruptPin = interruptPin;
}


boolean DecaDuino::init() {

	// Call init with 0xFFFF for both Short Address and PanId (no address/panid identification: Promiscuous mode)
	return init(0xFFFFFFFF);
}

boolean DecaDuino::init ( uint32_t shortAddressAndPanId ) {

	uint8_t buf[8];
	uint16_t ui16t;
	uint32_t ui32t;

	// Initialise the IRQ and Slave Select pin
	pinMode(_interruptPin, INPUT);
	pinMode(_slaveSelectPin, OUTPUT);
	digitalWrite(_slaveSelectPin, HIGH);
	SPI.begin();

	// Initialise the RX pointers
	rxDataAvailable = false;
	rxData = NULL;
	rxDataLen = NULL;

	// Wait for DW1000 POR (up to 5msec)
	delay(5);

#ifdef DECADUINO_DEBUG 
	delay(3000); // delay to see next messages on console for debug
	Serial.println("DecaDuino Debug is active!");
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
	sprintf((char*)debugStr,"SYS_CFG=%08lx", ui32t);
	Serial.println((char*)debugStr);
#endif

	// System Event Mask Register
	ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
	ui32t |= DW1000_REGISTER_SYS_MASK_MRXFCG_MASK; // MRXFCG: interrupt when good frame (FCS OK) received
	ui32t |= DW1000_REGISTER_SYS_MASK_MTXFRS_MASK;
	writeSpiUint32(DW1000_REGISTER_SYS_MASK, ui32t);

#ifdef DECADUINO_DEBUG 
	sprintf((char*)debugStr,"SYS_MASK=%08lx", ui32t);
	Serial.println((char*)debugStr);
#endif

	// Enable frame filtering on addressing fields if init() is called with a shortAddressAndPanId != 0xFFFFFFFF
	if ( shortAddressAndPanId != 0xFFFFFFFF ) {
		ui32t = readSpiUint32(DW1000_REGISTER_SYS_CFG);
		ui32t |= 0x0000003D;
		writeSpiUint32(DW1000_REGISTER_SYS_CFG,ui32t);
		setShortAddressAndPanId(shortAddressAndPanId);
	}

	// Set default antenna delay value
	setAntennaDelay(DWM1000_DEFAULT_ANTENNA_DELAY_VALUE);

	// --- End of DW1000 configuration ------------------------------------------------------------------------------

	lastTxOK = false;

	// Return true if everything OK
	return true;

} // End of init()


void DecaDuino::resetDW1000() {

	uint8_t buf[8];
	uint32_t ui32t;

	// Initialise the SPI port
	currentSPISettings = SPISettings(500000, MSBFIRST, SPI_MODE0);

	delay(100);

	// Getting PMSC_CTRL0 register
	ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);

#ifdef DECADUINO_DEBUG 
	sprintf((char*)debugStr,"PMSC_CTRL0=%08lx", ui32t);
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
	sprintf((char*)debugStr,"PMSC_CTRL0=%08lx", ui32t);
	Serial.println((char*)debugStr);
#endif

	// Set SOFTRESET bits
	ui32t |= 0xF0000000;
	ui32t &= 0xFFFFFFFC;
	writeSpiUint32(DW1000_REGISTER_PMSC_CTRL0, ui32t);

	delay(5);

		// Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)
		// Load the LDE algorithm microcode into LDE RAM (procedure p.22 DW1000 User Manual + comment p.21)

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		buf[0] = 0xF6;
		buf[1] = 0x00;
		buf[2] = 0x01;
		buf[3] = 0x03;
		spi_send(buf,4);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();

		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		buf[0] = 0xED;
		buf[1] = 0x06;
		buf[2] = 0x00;
		buf[3] = 0x80;
		spi_send(buf,4);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();

		delayMicroseconds(160);

		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		buf[0] = 0xF6;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x02;
		spi_send(buf,4);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction(); 
	}

	// Initialise the SPI port
	currentSPISettings = SPISettings(6000000, MSBFIRST, SPI_MODE0);
	delay(1);

#ifdef DECADUINO_DEBUG 
	ui32t = readSpiUint32(DW1000_REGISTER_PMSC_CTRL0);
	sprintf((char*)debugStr,"PMSC_CTRL0=%08lx", ui32t);
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
	uint32_t sysStatusReg, ack, ui32t;
	double rxtofs, rxttcki;
	ack = 0;

	// Read System Event Status Register
	sysStatusReg = readSpiUint32(DW1000_REGISTER_SYS_STATUS);

	// If IRQS is cleared, no enabled interrupt (SYS_MASK) have assert the IRQ pin: exit
	if ( ! ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_IRQS_MASK ) )
		return;

#ifdef DECADUINO_DEBUG 
	// Serial.print("\n###isr### ");
	//ui32t = readSpiUint32(DW1000_REGISTER_SYS_MASK);
	//sprintf((char*)debugStr,"SYS_MASK	=%08x", ui32t);
	//Serial.println((char*)debugStr);
	//sprintf((char*)debugStr,"SYS_STATUS=%08x ", sysStatusReg);
	//Serial.print((char*)debugStr);
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

					// Get RX timestamp
					encodeUint64(0, buf); // init buffer the 64-bit buffer 
					readSpi(DW1000_REGISTER_RX_TIME, buf, 5);
					lastRxTimestamp = decodeUint64(buf);

					// Get transmitter-receiver skew (clock offset or crystal offset between the local receiver and the remote end transmitter device)
					readSpi(DW1000_REGISTER_RX_TTCKO, buf, 3);

/* Commented by Adrien on 20150906
					ui32t = decodeUint32(buf) & 0x0007FFFF; // Clock offset is a 19-bit signed integer
					if ( ui32t & 0x00080000 )
						ui32t |= 0xFFF80000; // negative value
					ui32t = 0x01F00000/ui32t;
*/
					// Drien 20150906: should we read rxttcki value in DW1000_REGISTER_RX_TTCKI?
					rxttcki = 32505856;

					// Turn rxtofs to a signed double value (RD032014)
					if (buf[2] & 0x04 ) { // rxtofs is negative

						buf[2] |=  0xF8;
						buf[2] =  ~buf[2];
						buf[1] =  ~buf[1];
						buf[0] =  ~buf[0];
						rxtofs =   buf[2] * 256*256 + buf[1] * 256 + buf[0];
						rxtofs =   rxtofs+1;
						rxtofs =   rxtofs*-1;

					} else {

						rxtofs = buf[2] * 256*256 + buf[1] * 256 + buf[0];
					}

					clkOffset = rxtofs * 1000000 / rxttcki;
					rxDataAvailable = true;

					// Serial.print("clock offset=");
					// Serial.println(ui32t, HEX);			
					// Serial.println(offseti);

					// Serial.print("RXTOFS=0x");
					// Serial.println(ui32t, HEX);
					// ui32t = 0x01F00000/ui32t;
					// Serial.print("clock offset=0x");
					// Serial.println(ui32t, HEX);	
#ifdef DECADUINO_DEBUG 
					Serial.print("RX Frame timestamp=");
					printUint64(lastRxTimestamp);
					Serial.print(", skew=");
					Serial.println(clkOffset);
#endif

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

		// Read TX timestamp
		encodeUint64(0, buf); // // init buffer the 64-bit buffer
		readSpi(DW1000_REGISTER_TX_TIME, buf, 5);
		lastTxTimestamp = decodeUint64(buf);

		lastTxOK = true;
 
#ifdef DECADUINO_DEBUG
		Serial.print("TX Frame OK. Tx timestamp=");
		printUint64(lastTxTimestamp);
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


uint64_t DecaDuino::alignDelayedTransmission ( uint64_t wantedDelay ) {

	return ((getSystemTimeCounter() + wantedDelay) & 0xFFFFFFFE00) + getAntennaDelay();
}


uint8_t DecaDuino::pdDataRequest(uint8_t* buf, uint16_t len) {

	return pdDataRequest(buf, len, false, 0);
}

uint8_t DecaDuino::pdDataRequest(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time) {

	uint32_t ui32t;
	uint8_t tempbuf[8];

#ifdef DECADUINO_DEBUG 
	sprintf((char*)debugStr,"Request to send %dbyte(s): |", len);
	Serial.print((char*)debugStr);
	for (int i=0;i<len;i++) {
		sprintf((char*)debugStr,"%02x|", buf[i]);
		Serial.print((char*)debugStr);
	}
	Serial.println();
#endif

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

		trxStatus = DW1000_TRX_STATUS_TX;

		// copy PSDU in tx buffer
		writeSpi(DW1000_REGISTER_TX_BUFFER, buf, len);

		// read tx frame control register
		ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);

		// set frame length
		ui32t = (ui32t & ~DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK) | (len+2); // FCS is 2-bytes long
		writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

		if ( delayed ) { // if delayed transmission
			// send time
			encodeUint64 ( (time - getAntennaDelay() ) & 0x000000FFFFFFFE00, tempbuf); // time is 5-bytes long, 9 lsb=0
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

	return pdDataRequest(buf, len);
}


uint8_t DecaDuino::send(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time) {

	return pdDataRequest(buf, len, delayed, time);
}


void DecaDuino::setRxBuffer(uint8_t* buf, uint16_t *len) {

#ifdef DECADUINO_DEBUG 
	sprintf((char*)debugStr,"Setting RX buffer address to 0x%08lx", (uint32_t)buf);
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


void DecaDuino::spi_send ( uint8_t u8 ) {

	SPI.transfer(u8);
}


void DecaDuino::spi_send ( uint16_t u16 ) {

	SPI.transfer16(u16);
}


void DecaDuino::spi_send ( uint8_t* buf, uint16_t len ) {

	int i;

	for (i=0; i<len; i++)
		SPI.transfer(buf[i]);
}


void DecaDuino::spi_receive ( uint8_t* buf, uint16_t len ) {

	int i;

	for (i=0; i<len; i++)
		buf[i] = SPI.transfer(0);
}


void DecaDuino::readSpi(uint8_t address, uint8_t* buf, uint16_t len) {

	uint8_t addr = 0 | (address & 0x3F) ; // Mask register address (6bits) and preserve MSb at low (Read) and no subaddress

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		spi_send(addr);
		spi_receive(buf,len);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
}


void DecaDuino::readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

	uint8_t addr, sub_addr;

	addr = 0 | (address & 0x3F) | 0x40; // Mask register address (6bits), preserve MSb at low (Read) and set subaddress present bit (0x40)

	if ( subAddress < 0x80 ) {

		// This is a 2-bytes header SPI transaction

		sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			SPI.beginTransaction(currentSPISettings);
			digitalWrite(_slaveSelectPin, LOW);
			spi_send(addr);
			spi_send(sub_addr);
			spi_receive(buf,len);
			digitalWrite(_slaveSelectPin, HIGH); 
			SPI.endTransaction();
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
		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		spi_send(addr);
		spi_send(buf,len);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
}


void DecaDuino::writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

	uint8_t addr, sub_addr;

	addr = 0 | (address & 0x3F) | 0x80 | 0x40; // Mask register address (6bits), set MSb (Write) and set subaddress present bit (0x40)

	if ( subAddress < 0x80 ) {

		// This is a 2-bytes header SPI transaction

		sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			SPI.beginTransaction(currentSPISettings);
			digitalWrite(_slaveSelectPin, LOW);
			spi_send(addr);
			spi_send(sub_addr);
			spi_send(buf,len);
			digitalWrite(_slaveSelectPin, HIGH);
			SPI.endTransaction();
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


uint64_t DecaDuino::decodeUint40 ( uint8_t *data )
{
	uint64_t tmp = 0;
	tmp = data[4];
	tmp = tmp << 32;
	tmp = tmp | decodeUint32(data);
	
	return tmp;
}


void DecaDuino::encodeUint40 ( uint64_t from, uint8_t *to )
{
	to[4] = (from & 0xFF00000000) >> 32;
	to[3] = (from & 0xFF000000) >> 24;
	to[2] = (from & 0xFF0000) >> 16;
	to[1] = (from & 0xFF00) >> 8;
	to[0] = from & 0xFF;
}


uint64_t DecaDuino::decodeUint64 ( uint8_t *data ) {

	uint64_t tmp = 0;
	tmp = decodeUint32(&data[4]); // | decodeUint32(data);
	tmp = tmp << 32;
	tmp = tmp | decodeUint32(data);
	
	return tmp;
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


float DecaDuino::decodeFloat ( uint8_t *data ) {

	typedef union _data {
		float f;
		char  s[4];
	} myData;

	myData q;

	q.s[0] = data[0];
	q.s[1] = data[1];
	q.s[2] = data[2];
	q.s[3] = data[3];

	return q.f;
}


void DecaDuino::encodeFloat ( float from, uint8_t *to ) {

	typedef union _data {
		float f;
		char  s[4];
	} myData;

	myData q;
	q.f = from;

	to[0] = q.s[0];
	to[1] = q.s[1];
	to[2] = q.s[2];
	to[3] = q.s[3];
}


void DecaDuino::printUint64 ( uint64_t ui64 ) {

	uint8_t buf[8];

	encodeUint64(ui64, buf);

	sprintf((char*)debugStr, "%08lx%08lx", decodeUint32(&buf[4]), decodeUint32(buf));
	Serial.print((char*)debugStr);
}


void DecaDuino::getSystemTimeCounter ( uint64_t *p ) {

	uint8_t buf[8];

	encodeUint64(0, buf); // init buffer the 64-bit buffer
	readSpi(DW1000_REGISTER_SYS_TIME, buf, 5);
	*p = decodeUint64(buf);
}


void DecaDuino::setPHRMode(uint8_t mode) {
	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_SYS_CFG);
	ui32t = ui32t & (~DW1000_REGISTER_SYS_CFG_PHR_MODE_MASK);
	ui32t |= mode << DW1000_REGISTER_SYS_CFG_PHR_MODE_SHIFT;
	writeSpiUint32(DW1000_REGISTER_SYS_CFG,ui32t);
}

uint8_t DecaDuino::getPHRMode(void) {
	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_SYS_CFG);
	ui32t = (ui32t & DW1000_REGISTER_SYS_CFG_PHR_MODE_MASK) >> DW1000_REGISTER_SYS_CFG_PHR_MODE_SHIFT;
		return (uint8_t)ui32t;
}

uint64_t DecaDuino::getSystemTimeCounter ( void ) {

	uint64_t p;

	getSystemTimeCounter(&p);

	return p;
}

uint64_t DecaDuino::getLastTxTimestamp() {

	return lastTxTimestamp;
}


uint64_t DecaDuino::getLastRxTimestamp() {

	return lastRxTimestamp;
}


double DecaDuino::getLastRxSkew() {

	return clkOffset;
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


void DecaDuino::setShortAddressAndPanId(uint16_t shortAddress, uint16_t panId) {

	setPanId(panId);
	setShortAddress(shortAddress);
}


int DecaDuino::setShortAddressAndPanId(uint32_t shortAddressPanId) {

	uint32_t ret;	

	writeSpiUint32(0x03, shortAddressPanId);
	ret = readSpiUint32(0x03);
	if ( ret != shortAddressPanId ) {
#ifdef DECADUINO_DEBUG
		Serial.println("Setting Short Address and PanId OK\n");
#endif
		return false;
	} else {
#ifdef DECADUINO_DEBUG
		Serial.println("Error while Setting Short Address and PanId\n");
#endif
		return true;
	}
}


uint8_t DecaDuino::getChannelRaw(void) {

	uint8_t buf;
 
 	readSpiSubAddress(DW1000_REGISTER_CHAN_CTRL, 0, &buf, 1);
 	return buf;
}

 
uint8_t DecaDuino::getChannel(void) {

	return getChannelRaw() & 0x0F;
}


uint8_t DecaDuino::getRxPrf(void) {

 	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
	ui32t = ( ui32t & DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK) >> 18;
	return (uint8_t)ui32t;
}

uint8_t DecaDuino::getFpAmpl1(void) {
	
	
	uint8_t u8t;
	
	readSpiSubAddress(DW1000_REGISTER_RX_TIME, 0x07, &u8t, 1);
	
	
	
	
	return u8t;
}




uint16_t DecaDuino::getFpAmpl2(void) {
	
	uint32_t ui32t;
	ui32t = readSpiUint32(DW1000_REGISTER_RX_RFQUAL);
	ui32t = ( ui32t & DW1000_REGISTER_RX_RFQUAL_FPAMPL2_MASK ) >> 16;
	
	return (uint16_t) ui32t;
}
	

	

uint16_t DecaDuino::getFpAmpl3(void) {
	uint8_t buffer[2];
	uint16_t ui16t;
	readSpiSubAddress(DW1000_REGISTER_RX_RFQUAL, 0x06, buffer, 2);
	ui16t =   *((uint16_t *)buffer) ;
	
	
	return ui16t;
}

uint16_t DecaDuino::getRxPacc(void) {

 	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_RX_FINFO);
	ui32t = ( ui32t & DW1000_REGISTER_RX_FINFO_RXPACC_MASK) >> 20;
	return (uint16_t)ui32t;
}


double DecaDuino::getFpPower(void) {
	
	double fppow;
	float F1 = (float) getFpAmpl1();
	float F2 = (float) getFpAmpl2();
	float F3 = (float) getFpAmpl3();
	float N = (float) getRxPacc();
	uint8_t prf = getRxPrf();
	float A;
	
	if (prf == 1) {
		// prf set to 16 MHz
		A = 113.77;
	}
	else {
		// prf set to 64 MHz
		A = 121.74;
	}
		
	fppow = 10 * ( log10( ( (F1 * F1) + (F2 * F2) + (F3 * F3) ) / (N * N) ) ) - A;
	
	return(fppow);
}
	
	
uint16_t DecaDuino::getCirp(void) {
	uint8_t buffer[2];
	uint16_t ui16t;
	readSpiSubAddress(DW1000_REGISTER_RX_RFQUAL, 0x04, buffer, 2);
	ui16t = *((uint16_t *)buffer);
	return ui16t;
	
}

float DecaDuino::getSNR(void) {
	float ratio,cire,ampl2;
	cire = (float) getCire();
	ampl2 = (float) getFpAmpl2();
	
	ratio = ampl2 / cire;
	
	return(ratio);
}

uint16_t DecaDuino::getCire(void) {
	uint32_t ui32t;
	ui32t = readSpiUint32(DW1000_REGISTER_RX_RFQUAL);
	ui32t = ( ui32t & DW1000_REGISTER_RX_RFQUAL_CIRE_MASK );
	
	return (uint16_t) ui32t;
	
}
	
	
double DecaDuino::getRSSI(void) {
	
	double rss;
	float C = (float) getCire();
	float N = (float) getRxPacc();
	uint8_t prf = getRxPrf();
	float A;
	
	if (prf == 1) {
		// prf set to 16 MHz
		A = 113.77;
	}
	else {
		// prf set to 64 MHz
		A = 121.74;
	}
	
	rss = 10 * ( log10( (C * pow(2,17)) / (N * N) ) )  - A;
	
	return(rss);
}
	
	
	


uint8_t DecaDuino::getTxPcode(void) {

 	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
	ui32t = ( ui32t & DW1000_REGISTER_CHAN_CTRL_TX_PCODE_MASK) >> 22;
	return (uint8_t)ui32t;
}


uint8_t DecaDuino::getRxPcode(void) {

 	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
	ui32t = ( ui32t & DW1000_REGISTER_CHAN_CTRL_RX_PCODE_MASK) >> 27;
	return (uint8_t)ui32t;
}


bool DecaDuino::setChannel(uint8_t channel) {

 	uint32_t ui32t;

 	if ( ( channel != 6 ) && ( channel <= 7 ) && ( channel >= 1 ) ) {

 		channel =  channel + (channel << 4);
 		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
 		ui32t = ui32t & 0xFFFFFF00;
 		ui32t |= channel; // set rx and tx channel 
 		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);
 		if ( getChannelRaw() == channel )
 			return true;
 	}

 	return false;
}


bool DecaDuino::setRxPrf(uint8_t prf) {

	uint32_t ui32t;

	if ( ( prf == 1 ) || ( prf == 2 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK);
		ui32t |= prf << 18; 
		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);
		return true;

	} else return false;
}


bool DecaDuino::setTxPcode(uint8_t pcode) {

	uint32_t ui32t;

 	if ( ( pcode > 0 ) && ( pcode <= 20 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_TX_PCODE_MASK);
		ui32t |= pcode << 22; 
		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);
		return true;

	} else return false;
}


bool DecaDuino::setRxPcode(uint8_t pcode) {

	uint32_t ui32t;

 	if ( ( pcode > 0 ) && ( pcode <= 20 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_RX_PCODE_MASK);
		ui32t |= pcode << 27; 
		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);
		return true;

	} else return false;
}


int DecaDuino::getPreambleLength(void) {

	uint32_t ui32t;
	uint32_t mask;
	int plength;
	mask = 0x003C0000; // bits 21, 20, 19 and 18 from DW1000_REGISTER_TX_FCTRL
	ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
	ui32t = ui32t & mask;
	ui32t = ui32t >> 18;
	// Preamble length selection (Table 15, Page 73 from DW1000 User Manual)
	switch(ui32t){
		case 0x00000001:
			plength = 64;
			break;
		case 0x00000005:
			plength = 128;
			break;
		case 0x00000009:
			plength = 256;
			break;
		case 0x0000000D:
			plength = 512;
			break;
		case 0x00000002:
			plength = 1024;
			break;
		case 0x00000006:
			plength = 1536;
			break;
		case 0x0000000A:
			plength = 2048;
			break;
		case 0x00000003:
			plength = 4096;
			break;
		default:
			plength = 128;
			break;
	}

#ifdef DECADUINO_DEBUG
	sprintf((char*)debugStr,"TX_FCTRL=%08x\n", ui32t);
	Serial.print((char*)debugStr);
#endif

	return plength;
}


bool DecaDuino::setPreambleLength (int plength) {

	uint32_t ui32t;
	uint32_t mask;
	switch(plength){
		case 64:
			mask = 0x00040000;
			break;
		case 128:
			mask = 0x00140000;
			break;
		case 256:
			mask = 0x00240000;
			break;
		case 512:
			mask = 0x00340000;
			break;
		case 1024:
			mask = 0x00080000;
			break;
		case 1536:
			mask = 0x00180000;
			break;
		case 2048:
			mask = 0x00280000;
			break;
		case 4096:
			mask = 0x000C0000;
			break;
		default:
			return false;			
	}
	ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
	ui32t = ui32t & 0xFFC3FFFF; // bits 21, 20, 19, 18 to zero
	ui32t |= mask;
	writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);
	return true;		
}


uint16_t DecaDuino::getAntennaDelay() {

	return antennaDelay;
}


void DecaDuino::setAntennaDelay(uint16_t newAntennaDelay) {

	setAntennaDelayReg(newAntennaDelay);
	antennaDelay = newAntennaDelay;
}


uint16_t DecaDuino::getAntennaDelayReg(){

	uint8_t buf[2];

	readSpi(DW1000_REGISTER_TX_ANTD, buf, 2);
	return decodeUint16(buf);
}


void DecaDuino::setAntennaDelayReg(uint16_t newAntennaDelay) {

	uint8_t buf[2];

	encodeUint16(newAntennaDelay, buf);
	writeSpi(DW1000_REGISTER_TX_ANTD, buf, 2);
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
	
	
	uint8_t u8t;
	uint8_t buf_16[2];
	uint8_t buf_32[4];
	float temp,diff;
	uint8_t t23,raw_temp;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		
		buf_16[0] = 0x09;buf_16[1] = 0x00; writeSpiSubAddress(0x2D, 0x04, buf_16, 2);
		u8t= 0x03; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		u8t= 0x00; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		readSpiSubAddress(0x2D,0x0A,buf_32,4);
		
		
	}
	raw_temp = getTemperatureRaw();
	t23 =   buf_32[0];
	diff = (float) (raw_temp - t23);
	temp =   diff * 1.14 + 23.0; 
		
		

	// Temperature (°C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23°C )) x 1.14) + 23
	// Todo: what is OTP_READ(Vtemp @ 23°C ) ?

	return temp;
}


float DecaDuino::getVoltage(void) {

	// Voltage (volts) = (SAR_LVBAT- (OTP_READ(Vmeas @ 3.3 V )) /173) + 3.3
	// Todo: what is OTP_READ(Vmeas @ 3.3 V ) ?
	
	uint8_t u8t;
	uint8_t buf_16[2];
	uint8_t buf_32[4];
	float raw_v;
	float v33,v;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		
		buf_16[0] = 0x08;buf_16[1] = 0x00; writeSpiSubAddress(0x2D, 0x04, buf_16, 2);
		u8t= 0x03; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		u8t= 0x00; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		readSpiSubAddress(0x2D,0x0A,buf_32,4);
		
		
	}
	raw_v = (float)getVoltageRaw();
	v33 =  (float) buf_32[0];
	v =  ( ( raw_v - v33 ) / 173) + 3.3; 
		
		

	// Temperature (°C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23°C )) x 1.14) + 23
	// Todo: what is OTP_READ(Vtemp @ 23°C ) ?

	return v;

	
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

