// DecaDuino.cpp
//
// Another DecaWave DW1000 driver for Arduino
// See the README file in this directory for documentation

#include <SPI.h>
#include "DecaDuino.h"
#include "printfToSerial.h"
#include "base64.hpp"
#include <machine/endian.h>

#ifdef UWB_MODULE_DWM1001

#define SPI SPI1
static inline uint32_t begin_atomic()
{
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    return prim;
}

static inline void end_atomic(uint32_t prim)
{
    if (!prim) {
        __enable_irq();
    }
}

#else
#include <util/atomic.h>
#endif


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
	if ( getDevID()  != 0xdeca0130 ) return false;

	// Load Extended Unique Identifier – the 64-bit IEEE device address - in memory
	euid = getEuid();

    #ifdef UWB_MODULE_DWM1001
	_DecaDuinoInterrupt[0] = this;
    attachInterrupt(_interruptPin, DecaDuino::isr0, RISING);
    #else
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
    #endif
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
	useCalibratedAntennaDelay();

	// --- End of DW1000 configuration ------------------------------------------------------------------------------

	lastTxOK = false;

	// dummy channel config set-up : we set up these elements with the same values as the defaults, so that the
	// setXXX methods will do the fine-tuning that is proposed in DW1000 user manual section 2.5.5
	setDefaultChannelConfig();

	// other fine tuning for default config
	setNTM(0x0D);
	encodeUint32(0X2502A907,buf);
	writeSpiSubAddress(DW1000_REGISTER_AGC_CTRL, DW1000_REGISTER_OFFSET_AGC_TUNE2, buf, 4);
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

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif

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

    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif

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
    #ifdef UWB_MODULE_DWM1001
    _DecaDuinoInterrupt[0]->handleInterrupt();
    #else
	if (_DecaDuinoInterrupt[DW1000_IRQ0_PIN]) _DecaDuinoInterrupt[DW1000_IRQ0_PIN]->handleInterrupt();
    #endif
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
    _interrupReceived = true;

}

void DecaDuino::engine() {
    // Do nothing if no interrupt were received
    if (!_interrupReceived) {
        return;
    }

	uint8_t buf[8];
	uint32_t sysStatusReg, ack, ui32t;
	double rxtofs, rxttcki;
	ack = 0;

	// Read System Event Status Register
	sysStatusReg = readSpiUint32(DW1000_REGISTER_SYS_STATUS);

	// If IRQS is cleared, no enabled interrupt (SYS_MASK) have assert the IRQ pin: exit
	if ( ! ( sysStatusReg & DW1000_REGISTER_SYS_STATUS_IRQS_MASK ) ){
		return;
	}


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
			    uint32_t RX_FINFO = readSpiUint32(DW1000_REGISTER_RX_FINFO);
				*rxDataLen = (uint16_t)((RX_FINFO & DW1000_REGISTER_RX_FINFO_RXFLEN_MASK) - 2); // FCS is 2-bytes long. Avoid it in the len.
				uint8_t RXPSR = (RX_FINFO & DW1000_REGISTER_RX_FINFO_RXPSR_MASK) >> DW1000_REGISTER_RX_FINFO_RXPSR_SHIFT;
				uint8_t RXNSPL = (RX_FINFO & DW1000_REGISTER_RX_FINFO_RXFNSPL_MASK) >> DW1000_REGISTER_RX_FINFO_RXFNSPL_SHIFT;
				uint16_t pLength = RXPSR | (RXNSPL << 2);
				switch (pLength) {  // according to DWM1000 user manual
                    case 0x4: pLength = 64; break;
                    case 0x5: pLength = 128; break;
                    case 0x6: pLength = 256; break;
                    case 0x7: pLength = 512; break;
                    case 0x8: pLength = 1024; break;
                    case 0x9: pLength = 1536; break;
                    case 0xA: pLength = 2048; break;
                    case 0xC: pLength = 4096; break;
                    default: pLength = -1;    break;
                }
				_lastRxDuration = computeRxDuration(pLength, (*rxDataLen) + 2); // adds the 2-bytes from the FCS
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
					rxttcki =  _rxPrf == 16 ? 0x01F00000 : 0x01FC0000; // we are not reading the value from the registers since they are only depending on the PRF.

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
	    //Serial.println("TX frame interrupt");
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

	_interrupReceived = false;

#ifdef DECADUINO_DEBUG
	Serial.println();
	Serial.print("End if interrupt");
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

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif

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

    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif
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

	if (trxStatus == DW1000_TRX_STATUS_RX) return ;

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

	if ( trxStatus == DW1000_TRX_STATUS_IDLE) return;

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

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif

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
    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif
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

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif
		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		spi_send(addr);
		spi_receive(buf,len);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif
}


void DecaDuino::readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

	uint8_t addr, sub_addr;

	addr = 0 | (address & 0x3F) | 0x40; // Mask register address (6bits), preserve MSb at low (Read) and set subaddress present bit (0x40)

	if ( subAddress < 0x80 ) {

		// This is a 2-bytes header SPI transaction

		sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

        #ifdef UWB_MODULE_DWM1001
        uint32_t prim = begin_atomic();
        {
        #else
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        #endif
			SPI.beginTransaction(currentSPISettings);
			digitalWrite(_slaveSelectPin, LOW);
			spi_send(addr);
			spi_send(sub_addr);
			spi_receive(buf,len);
			digitalWrite(_slaveSelectPin, HIGH);
			SPI.endTransaction();
		}
        #ifdef UWB_MODULE_DWM1001
        end_atomic(prim);
        #endif

	} else {

		// This is a 3-bytes header SPI transaction

		uint8_t sub_addrL, sub_addrH;

		sub_addrL = 0x80 | (subAddress & 0x7F); // Extension Address Indicator (0x80) + low-order 7 bits of sub address
		sub_addrH = 0 | ((subAddress>>7) & 0xFF); // high-order 8 bits of sub address

        #ifdef UWB_MODULE_DWM1001
        uint32_t prim = begin_atomic();
        {
        #else
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        #endif
			SPI.beginTransaction(currentSPISettings);
			digitalWrite(_slaveSelectPin, LOW);
			spi_send(addr);
			spi_send(sub_addrL);
			spi_send(sub_addrH);
			spi_receive(buf,len);
			digitalWrite(_slaveSelectPin, HIGH);
			SPI.endTransaction();
		}
        #ifdef UWB_MODULE_DWM1001
        end_atomic(prim);
        #endif
	}
}


uint32_t DecaDuino::readSpiUint32(uint8_t address) {

	uint8_t buf[4];

	readSpi(address, buf, 4);
	return decodeUint32(buf);
}


void DecaDuino::writeSpi(uint8_t address, uint8_t* buf, uint16_t len) {

	uint8_t addr = 0 | (address & 0x3F) | 0x80; // Mask register address (6bits) and set MSb (Write) and no subaddress

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif
		SPI.beginTransaction(currentSPISettings);
		digitalWrite(_slaveSelectPin, LOW);
		spi_send(addr);
		spi_send(buf,len);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif
}


void DecaDuino::writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len) {

	uint8_t addr, sub_addr;

	addr = 0 | (address & 0x3F) | 0x80 | 0x40; // Mask register address (6bits), set MSb (Write) and set subaddress present bit (0x40)

	if ( subAddress < 0x80 ) {

		// This is a 2-bytes header SPI transaction

		sub_addr = 0 | (subAddress & 0x7F); // Mask register address (6bits)

        #ifdef UWB_MODULE_DWM1001
        uint32_t prim = begin_atomic();
        {
        #else
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        #endif
			SPI.beginTransaction(currentSPISettings);
			digitalWrite(_slaveSelectPin, LOW);
			spi_send(addr);
			spi_send(sub_addr);
			spi_send(buf,len);
			digitalWrite(_slaveSelectPin, HIGH);
			SPI.endTransaction();
		}
        #ifdef UWB_MODULE_DWM1001
        end_atomic(prim);
        #endif

	} else {
	    // This is a 3-bytes header SPI transaction

        uint8_t sub_addrL, sub_addrH;

        sub_addrL = 0x80 | (subAddress & 0x7F); // Extension Address Indicator (0x80) + low-order 7 bits of sub address
        sub_addrH = 0 | ((subAddress>>7) & 0xFF); // high-order 8 bits of sub address

        #ifdef UWB_MODULE_DWM1001
        uint32_t prim = begin_atomic();
        {
        #else
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        #endif
            SPI.beginTransaction(currentSPISettings);
            digitalWrite(_slaveSelectPin, LOW);
            spi_send(addr);
            spi_send(sub_addrL);
            spi_send(sub_addrH);
            spi_send(buf,len);
            digitalWrite(_slaveSelectPin, HIGH);
            SPI.endTransaction();
        }
        #ifdef UWB_MODULE_DWM1001
        end_atomic(prim);
        #endif
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

uint8_t DecaDuino::powerSettingsToRegisterValue(COARSE_POWER_SETTING coarse, uint8_t fine){
    fine = fine < 31 ? fine : 31;   // coerce fine to the interval [ 0, 31 ]
    return (uint8_t)coarse << 5 | fine;
}

void DecaDuino::setSmartTxPower(bool trackChanges){
    // get SYS_CFG register
    uint32_t u32;
    u32 = readSpiUint32(DW1000_REGISTER_SYS_CFG);

    // set DIS_SXTP to 0
    u32 &= ~DW1000_REGISTER_SYS_CFG_DIS_SXTP_MASK;

    // write value back
    writeSpiUint32(DW1000_REGISTER_SYS_CFG,u32);

    writeSmartTxPowerConf();
    _txPowerTracksChanges = trackChanges;
}

void DecaDuino::writeSmartTxPowerConf(){
    uint32_t buf;
    encodeUint32(smartTxPowerConf[ TX_POWER_CHANNEL[getChannel()] ][ getTxPrf() >> 6 ], (uint8_t *)&buf);
    writeSpiUint32(DW1000_REGISTER_TX_POWER, buf);
}

bool DecaDuino::isTxPowerSmart(){
    // get SYS_CFG register
    uint32_t u32;
    u32 = readSpiUint32(DW1000_REGISTER_SYS_CFG);
    return ! (u32 & DW1000_REGISTER_SYS_CFG_DIS_SXTP_MASK);
}

void DecaDuino::setManualTxPower(COARSE_POWER_SETTING coarse, unsigned int fine){

    // compute the power value according to the specs
    uint8_t power = powerSettingsToRegisterValue(coarse, fine);

    setManualTxPowerRaw(power);
}


void DecaDuino::setManualTxPowerRawFullRegister(uint32_t registerValue){
    // get SYS_CFG register
    uint32_t u32;
    u32 = readSpiUint32(DW1000_REGISTER_SYS_CFG);

    // set DIS_SXTP to 1
    u32 |= DW1000_REGISTER_SYS_CFG_DIS_SXTP_MASK;

    // write value back
    writeSpiUint32(DW1000_REGISTER_SYS_CFG,u32);

    // set the power values to the registers
    writeSpiUint32(DW1000_REGISTER_TX_POWER,registerValue);
}

void DecaDuino::setManualTxPowerRaw(uint8_t registerValue){
    setManualTxPowerRawFullRegister(registerValue | registerValue << 8 | registerValue << 16 | registerValue << 24);
}

void DecaDuino::setRecommendedFixedTxPower(bool trackChanges){
    writeRecommendedFixedTxPowerConf();
    _txPowerTracksChanges = trackChanges;
}

void DecaDuino::writeRecommendedFixedTxPowerConf(){
    uint32_t buf;
    encodeUint32(recommendedManualTxPowerConf[ TX_POWER_CHANNEL[getChannel()] ][ getTxPrf() >> 6 ], (uint8_t *)&buf);
    setManualTxPowerRawFullRegister(buf);
}

bool DecaDuino::isTxPowerManual(){
    return !isTxPowerSmart();
}


uint32_t DecaDuino::getTX_POWER(){
    uint32_t u32;
    u32 = readSpiUint32(DW1000_REGISTER_TX_POWER);
    return u32;
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


double DecaDuino::getLastRxDuration(){
    return _lastRxDuration;
}


double DecaDuino::computeRxDuration(int preambleLength, unsigned int payloadSize){
    double duration = 0;

    double symbolDuration = 0;
    if (_rxPrf == 16) symbolDuration  = 0.00000099359;
    else symbolDuration = 0.00000101763;

    // preamble
    duration += ((double)preambleLength) * symbolDuration;

    // SFD
    if (_DWSFD){
        switch (_datarate) {
            case DW1000_DATARATE_110KBPS: duration += 64.0 * symbolDuration; break;
            case DW1000_DATARATE_850KBPS: duration += 16.0 * symbolDuration; break;
            case DW1000_DATARATE_6_8MBPS: duration += 8.0 * symbolDuration; break;
            default: duration += 1; break;
        }
    }
    else {
        duration += 8.0*symbolDuration;
    }

    // PHR
    if (_datarate == DW1000_DATARATE_110KBPS) duration += 19. / 110000.;
    else  duration += 19. / 850000.;

    // payload
    switch (_datarate) {
        case DW1000_DATARATE_110KBPS: duration += ((double)payloadSize) * 8.0 / 110000.; break;
        case DW1000_DATARATE_850KBPS: duration += ((double)payloadSize) * 8.0 / 850000.; break;
        case DW1000_DATARATE_6_8MBPS: duration += ((double)payloadSize) * 8.0 / 6800000.; break;
        default: duration += 1.;  break;
    }

    return duration;
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

uint32_t DecaDuino::getChanControl(void) {

	return readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
}


uint8_t DecaDuino::getRxPrf(void) {

 	uint32_t ui32t;

	ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
	ui32t = ( ui32t & DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK) >> 18;
	switch ((uint8_t)ui32t) {
		case 1: return 16;
		case 2: return 64;
	}
	return 0;
}


uint8_t DecaDuino::getTxPrf(void){
    uint32_t ui32t;

    ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
    ui32t = ( ui32t & DW1000_REGISTER_TX_FCTRL_TX_PRF_MASK) >> DW1000_REGISTER_TX_FCTRL_TX_PRF_SHIFT;
    switch ((uint8_t)ui32t) {
        case 1: return 16;
        case 2: return 64;
    }
    return 0;
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
	uint16_t ui16t;
	readSpiSubAddress(DW1000_REGISTER_RX_RFQUAL, 0x04,
            (uint8_t *) &ui16t, sizeof(ui16t));
	return ui16t;
}

uint16_t DecaDuino::getRxPacc(void) {

 	uint32_t RXPACC ;

	RXPACC = readSpiUint32(DW1000_REGISTER_RX_FINFO);
	RXPACC = ( RXPACC & DW1000_REGISTER_RX_FINFO_RXPACC_MASK) >> DW1000_REGISTER_RX_FINFO_RXPACC_SHIFT;

 	uint16_t RXPACC_NOSAT;
 	uint8_t buf[2];
	readSpiSubAddress(DW1000_REGISTER_DRX_CONF, DW1000_REGISTER_OFFSET_RXPACC_NOSAT, buf, 2);
	RXPACC_NOSAT = decodeUint16(buf);

	channelCTRL_t channel = getChannelControlRegister();

	if (RXPACC != RXPACC_NOSAT) {
            return RXPACC;
	}
    else {
        if ( channel.DWSFD == 0 and channel.RNSSFD == 1){
            // Custom SFD used for reception, not implemented yet. See See DW1000 user manual for help on how to implement this.
            return -1;
        }
        else if ( channel.RNSSFD == 0 ) {
            // standard IEEE 802.15.4 SFD adjustment
            if ( getRXM110K() == 1 ) {
                // 110 kb/s : long SFD
                return RXPACC - 64;
            }
            else {
                // other bitrates : short SFD
                return RXPACC - 5;
            }
        }
        else {
            // proprietary decawave SFD
            if ( getRXM110K() == 1 ) {
                // 110 kb/s : long SFD
                return RXPACC - 82;
            }
            else {
                // other bitrates
                uint8_t SFD_LENGTH = getSFD_LENGTH();
                if ( SFD_LENGTH == 8 ){
                    return RXPACC - 10;
                }
                else if ( SFD_LENGTH == 16 ) {
                    return RXPACC - 18;
                }
                else {
                    return -1; // Invalid SFD_LENGTH for this config
                }
            }
        }
    }


}


double DecaDuino::getFpPower(void) {

	double fppow;
	float F1 = (float) getFpAmpl1();
	float F2 = (float) getFpAmpl2();
	float F3 = (float) getFpAmpl3();
	float N = (float) getRxPacc();
	uint8_t prf = getRxPrf();
	float A;

	if (prf == 16) {
		// prf set to 16 MHz
		A = 113.77;
	}
	else if (prf == 64) {
		// prf set to 64 MHz
		A = 121.74;
	}
	else return 0;

	fppow = 10 * ( log10( ( (F1 * F1) + (F2 * F2) + (F3 * F3) ) / (N * N) ) ) - A;

	return(fppow);
}

double DecaDuino::getPeakPower(void){
    double fppow;

    uint8_t buf[2];
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_PPAMPL_OFFSET,buf,2);
    float PPA = (float) decodeUint16(buf);
    float N = (float) getRxPacc();
    uint8_t prf = getRxPrf();
    float A;

    if (prf == 16) {
        // prf set to 16 MHz
        A = 113.77;
    }
    else if (prf == 64) {
        // prf set to 64 MHz
        A = 121.74;
    }
    else return 0;

    fppow = 10 * ( log10( ( PPA * PPA) / (N * N) ) ) - A;

    return(fppow);
}


uint16_t DecaDuino::getCirp(void) {
	uint16_t ui16t;
	readSpiSubAddress(DW1000_REGISTER_RX_RFQUAL, 0x06,
            (uint8_t *)&ui16t, sizeof(ui16t));
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
	float C = (float) getCirp();
	float N = (float) getRxPacc();
	uint8_t prf = getRxPrf();
	float A;

	if (prf == 16) {
		// prf set to 16 MHz
		A = 113.77;
	}
	else if (prf == 64) {
		// prf set to 64 MHz
		A = 121.74;
	}
	else return 0;

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

/* Channel configuration
 * Inspired from Decawave DW1000 API
 */

// Translation table giving the indexes for channel settings tables
static const int CHANNEL[] = {
    -1,
    0,
    1,
    2,
    3,
    4,
    -1,
    5
};

#define FS_CTRL_ID              0x2B            /* Frequency synthesiser control block */
/* offset from FS_CTRL_ID in bytes */
#define FS_PLLCFG_OFFSET        0x07            /* Frequency synthesiser  PLL configuration */
#define FS_PLLCFG_LEN           (4)
static const uint32_t FS_PLLCFG_CH[] = {
    0x09000407UL,                               /* Operating Channel 1 */
    0x08400508UL,                               /* Operating Channel 2 */
    0x08401009UL,                               /* Operating Channel 3 */
    0x08400508UL,                               /* Operating Channel 4 (same as 2) */
    0x0800041DUL,                               /* Operating Channel 5 */
    0x0800041DUL                                /* Operating Channel 7 (same as 5) */
};

/* offset from FS_CTRL_ID in bytes */
#define FS_PLLTUNE_OFFSET       0x0B            /* Frequency synthesiser  PLL Tuning */
#define FS_PLLTUNE_LEN          (1)
static const uint8_t FS_PLLTUNE_CH[] = {
    0x1E,                                       /* Operating Channel 1 */
    0x26,                                       /* Operating Channel 2 */
    0x56,                                       /* Operating Channel 3 */
    0x26,                                       /* Operating Channel 4 (same as 2) */
    0xBE,                                       /* Operating Channel 5 */
    0xBE                                        /* Operating Channel 7 (same as 5) */
};

#define RF_CONF_ID              0x28            /* Analog RF Configuration */
#define RF_CONF_LEN             (58)
/* offset from TX_CAL_ID in bytes */
#define RF_RXCTRLH_OFFSET       0x0B            /* Analog RX Control Register */
#define RF_RXCTRLH_LEN          (1)
#define RF_RXCTRLH_NBW          0xD8            /* RXCTRLH value for narrow bandwidth channels */
#define RF_RXCTRLH_WBW          0xBC            /* RXCTRLH value for wide bandwidth channels */
/* offset from TX_CAL_ID in bytes */
#define RF_TXCTRL_OFFSET        0x0C            /* Analog TX Control Register */
#define RF_TXCTRL_LEN           (4)
#define RF_TXCTRL_TXMTUNE_MASK  0x000001E0UL    /* Transmit mixer tuning register */
#define RF_TXCTRL_TXTXMQ_MASK   0x00000E00UL    /* Transmit mixer Q-factor tuning register */
static const uint32_t RF_TXCTRL[] = {
    0x00005C40UL,                               /* Operating Channel 1 */
    0x00045CA0UL,                               /* Operating Channel 2 */
    0x00086CC0UL,                               /* Operating Channel 3 */
    0x00045C80UL,                               /* Operating Channel 4 */
    0x001E3FE0UL,                               /* Operating Channel 5 */
    0x001E7DE0UL                                /* Operating Channel 7 */
};

#define CHAN_CTRL_TX_CHAN_MASK  0x0000000FUL    /* Supported channels are 1, 2, 3, 4, 5, and 7.*/
#define CHAN_CTRL_TX_CHAN_SHIFT (0)             /* Bits 0..3        TX channel number 0-15 selection */

#define CHAN_CTRL_RX_CHAN_MASK  0x000000F0UL
#define CHAN_CTRL_RX_CHAN_SHIFT (4)             /* Bits 4..7        RX channel number 0-15 selection */

#define CHAN_CTRL_RXFPRF_MASK   0x000C0000UL    /* Bits 18..19      Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_SHIFT  (18)
/* Specific RXFPRF configuration */
#define CHAN_CTRL_RXFPRF_4      0x00000000UL    /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_16     0x00040000UL    /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_64     0x00080000UL    /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_TX_PCOD_MASK  0x07C00000UL    /* Bits 22..26      TX Preamble Code selection, 1 to 24. */
#define CHAN_CTRL_TX_PCOD_SHIFT (22)
#define CHAN_CTRL_RX_PCOD_MASK  0xF8000000UL    /* Bits 27..31      RX Preamble Code selection, 1 to 24. */
#define CHAN_CTRL_RX_PCOD_SHIFT (27)
/*offset 16 */
#define CHAN_CTRL_DWSFD         0x00020000UL    /* Bit 17 This bit enables a non-standard DecaWave proprietary SFD sequence. */
#define CHAN_CTRL_DWSFD_SHIFT   (17)
#define CHAN_CTRL_TNSSFD        0x00100000UL    /* Bit 20 Non-standard SFD in the transmitter */
#define CHAN_CTRL_TNSSFD_SHIFT  (20)
#define CHAN_CTRL_RNSSFD        0x00200000UL    /* Bit 21 Non-standard SFD in the receiver */
#define CHAN_CTRL_RNSSFD_SHIFT  (21)

#define LDE_IF_ID               0x2E            /* Leading edge detection control block */
/* offset from LDE_IF_ID in bytes */
#define LDE_REPC_OFFSET         0x2804  /* 16-bit configuration register for setting the replica avoidance coefficient */
#define LDE_REPC_LEN            (2)
// LDE replica coefficient configuration base on rx pcode
static const uint16_t LDE_REPC[] = {
    0,
    0x5998,
    0x5998,
    0x51EA,
    0x428E,
    0x451E,
    0x2E14,
    0x8000,
    0x51EA,
    0x28F4,
    0x3332,
    0x3AE0,
    0x3D70,
    0x3AE0,
    0x35C2,
    0x2B84,
    0x35C2,
    0x3332,
    0x35C2,
    0x35C2,
    0x47AE,
    0x3AE0,
    0x3850,
    0x30A2,
    0x3850
};

// Transmitter Calibration - Pulse Generator Delay
static const uint8_t TC_PGDELAY[] = {
    0xC9,
    0xC2,
    0xC5,
    0x95,
    0xC0,
    0x93
};

bool DecaDuino::setChannel(uint8_t channel) {
    if ( ( channel != 6 ) && ( channel <= 7 ) && ( channel >= 1 ) ) {
        // PLL configuration
        writeSpiSubAddress(FS_CTRL_ID, FS_PLLCFG_OFFSET,
                (uint8_t*)&FS_PLLCFG_CH[CHANNEL[channel]], FS_PLLCFG_LEN);
        writeSpiSubAddress(FS_CTRL_ID, FS_PLLTUNE_OFFSET,
                (uint8_t*)&FS_PLLTUNE_CH[CHANNEL[channel]], FS_PLLTUNE_LEN);

        // Narrow/Wide Band configuration
        uint8_t rxctrlh;
        if (channel == 4 || channel == 7)
        {
            rxctrlh = RF_RXCTRLH_WBW;
        }
        else
        {
            rxctrlh = RF_RXCTRLH_NBW;
        }
        writeSpiSubAddress(RF_CONF_ID, RF_RXCTRLH_OFFSET,
                &rxctrlh, RF_RXCTRLH_LEN);

        // Analog TX control configuration
        writeSpiSubAddress(RF_CONF_ID, RF_TXCTRL_OFFSET,
                (uint8_t*)&RF_TXCTRL[CHANNEL[channel]], RF_TXCTRL_LEN);

        // Transmitter Calibration - Pulse Generator Delay
        writeSpiSubAddress(DW1000_REGISTER_TX_CAL, DW1000_REGISTER_OFFSET_TC_PGDELAY,
                        (uint8_t*)&TC_PGDELAY[CHANNEL[channel]], sizeof(TC_PGDELAY[0]));

		// select corresponding preamble code among valid values for selected (channel, PRF) pair; at least two values are available for each selection
        uint8_t currentPcode = getTxPcode();
        uint8_t pcode = 0;
		switch(channel){
			case 1:
				if(getRxPrf()==16) pcode = 2;
				else if(getRxPrf()==64) pcode = 9;
				break;
			case 2:
				if(getRxPrf()==16) pcode = 3;
				else if(getRxPrf()==64) pcode = 10;
				break;
			case 3:
				if(getRxPrf()==16) pcode = 6;
				else if(getRxPrf()==64) pcode = 11;
				break;
			case 4:
				if(getRxPrf()==16) pcode = 7;
				else if(getRxPrf()==64) pcode = 17;
				break;
			case 5:
				if(getRxPrf()==16) pcode = 4;
				else if(getRxPrf()==64) pcode = 12;
				break;
			case 7:
				if(getRxPrf()==16) pcode = 8;
				else if(getRxPrf()==64) pcode = 18;
				break;
		}

        uint16_t lde_repc = LDE_REPC[pcode];
        if (getDataRate() == DW1000_DATARATE_110KBPS)
        {
            lde_repc >>= 3;
        }
        writeSpiSubAddress(LDE_IF_ID, LDE_REPC_OFFSET,
                (uint8_t*)&lde_repc, LDE_REPC_LEN);

        uint32_t chan_ctrl = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
        chan_ctrl &= ~(CHAN_CTRL_TX_CHAN_MASK | CHAN_CTRL_RX_CHAN_MASK
            | CHAN_CTRL_TX_PCOD_MASK | CHAN_CTRL_RX_PCOD_MASK);

        chan_ctrl |= (CHAN_CTRL_TX_CHAN_MASK & (channel << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
              (CHAN_CTRL_RX_CHAN_MASK & (channel << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
              (CHAN_CTRL_TX_PCOD_MASK & (pcode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
              (CHAN_CTRL_RX_PCOD_MASK & (pcode << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

        writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, chan_ctrl);

        uint32_t actual_chan_ctrl = getChanControl();
    #ifdef DECADUINO_DEBUG
        sprintf((char*)debugStr,"Expected CHAN_CTRL=0x%08x, Actual CHAN_CTRL=0x%08x", chan_ctrl, actual_chan_ctrl);
        Serial.println((char*)debugStr);
    #endif
        if ( actual_chan_ctrl == chan_ctrl ){
            if (_antennaDelayTracksChanges){
                setCalibratedAntennaDelay();
            }
            if (_txPowerTracksChanges){
                if (isTxPowerSmart()) writeSmartTxPowerConf();
                else if (isTxPowerManual())  writeRecommendedFixedTxPowerConf();
            }
            return true;
        }
    }
    return false;
}

bool DecaDuino::setPrf(uint8_t prf) {
    bool ret = setTxPrf(prf) && setRxPrf(prf);
    if (_antennaDelayTracksChanges){
        setCalibratedAntennaDelay();
    }
    if (_txPowerTracksChanges){
        if (isTxPowerSmart()) writeSmartTxPowerConf();
        else if (isTxPowerManual())  writeRecommendedFixedTxPowerConf();
    }
    return ret;
}

bool DecaDuino::setRxPrf(uint8_t prf) {

	uint32_t ui32t;
	_rxPrf = prf;
	if ( ( prf == 16 ) || ( prf == 64 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK);
		prf = (prf == 16 ? 1 : 2);
		ui32t |= prf << DW1000_REGISTER_CHAN_CTRL_RXPRF_SHIFT;
		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);

        // other tuning related to PRF
		uint8_t drx_tun1a[2];
		uint8_t drx_tun2[4];
        uint8_t lde_cfg2[2];
        uint8_t pac = recommendedPACSize(getPreambleLength());
        if (prf == 1){  //16MHz
            // DRX_TUNE1A
            encodeUint16(0x0087, drx_tun1a);
            // DRX_TUNE2
            switch (pac) {
            case 8: encodeUint32(0x311A002D, drx_tun2); break;
            case 16: encodeUint32(0x331A0052, drx_tun2); break;
            case 32: encodeUint32(0x351A009A, drx_tun2); break;
            case 64:
            default: encodeUint32(0x371A011D, drx_tun2); break;
            }
            // LDE_CFG2
            encodeUint16(_NLOSOptims ? 0x0003 : 0x1607,lde_cfg2);
        }
        else { //64MHz
            // DRX_TUNE1A
            encodeUint16(0x008D, drx_tun1a);
            // DRX_TUNE2
            switch (pac) {
            case 8: encodeUint32(0x313B006B, drx_tun2); break;
            case 16: encodeUint32(0x333B00BE, drx_tun2); break;
            case 32: encodeUint32(0x353B015E, drx_tun2); break;
            case 64:
            default: encodeUint32(0x373B0296, drx_tun2); break;
            }
            // LDE_CFG2
            encodeUint16(0x0607,lde_cfg2);
        }
        writeSpiSubAddress(DW1000_REGISTER_DRX_CONF, DW1000_REGISTER_OFFSET_DRX_TUNE1A, drx_tun1a, 2);
        writeSpiSubAddress(DW1000_REGISTER_DRX_CONF, DW1000_REGISTER_OFFSET_DRX_TUNE2, drx_tun2, 4);
        writeSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE, DW1000_REGISTER_LDE_INTERFACE_LDE_CFG2_OFFSET, lde_cfg2, 2);

        return true;

	} else return false;
}

bool DecaDuino::setTxPrf(uint8_t prf){
    uint32_t ui32t;

    if ( ( prf == 16 ) || ( prf == 64 ) ) {

        // sets PRF
        ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
        ui32t = ui32t & (~DW1000_REGISTER_TX_FCTRL_TX_PRF_MASK);
        prf = prf == 16 ? 1 : 2;    // converts PRF value to the value required in the register
        ui32t |= prf << DW1000_REGISTER_TX_FCTRL_TX_PRF_SHIFT;
        writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

        uint8_t agc_tun1[2];
        // other tuning related to PRF
        if (prf == 1){  //16MHz
            // AGC_TUNE1
            encodeUint16(0x8870, agc_tun1);
        }
        else {
            // AGC_TUNE1
            encodeUint16(0x889B, agc_tun1);
        }
        writeSpiSubAddress(DW1000_REGISTER_AGC_CTRL, DW1000_REGISTER_OFFSET_AGC_TUNE1, agc_tun1, 2);
        return true;

    }
    else return false;
}

uint8_t DecaDuino::recommendedPACSize(uint16_t preamble_len){
    if (preamble_len <= 128) return 8;
    if (preamble_len <= 512) return 16;
    if (preamble_len <= 1024) return 32;
    else return 64;
}

bool DecaDuino::setPcode(uint8_t pcode){
    return setTxPcode(pcode) && setRxPcode(pcode);
}

bool DecaDuino::setTxPcode(uint8_t pcode) {

	uint32_t ui32t;

 	if ( ( pcode > 0 ) && ( pcode <= 20 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_TX_PCODE_MASK);
		ui32t |= (pcode << DW1000_REGISTER_CHAN_CTRL_TX_PCODE_SHIFT);
		writeSpiUint32(DW1000_REGISTER_CHAN_CTRL, ui32t);
		return true;

	} else return false;
}


bool DecaDuino::setRxPcode(uint8_t pcode) {

	uint32_t ui32t;

 	if ( ( pcode > 0 ) && ( pcode <= 20 ) ) {

		ui32t = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
		ui32t = ui32t & (~DW1000_REGISTER_CHAN_CTRL_RX_PCODE_MASK);
		ui32t |= pcode << DW1000_REGISTER_CHAN_CTRL_RX_PCODE_SHIFT;
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

#define DRX_CONF_ID             0x27            /* Digital Receiver configuration */
#define DRX_CONF_LEN            (44)

/* offset from DRX_CONF_ID in bytes */
#define DRX_TUNE2_OFFSET        0x08    /* 7.2.40.5 Sub-Register 0x27:08  DRX_TUNE2 */
#define DRX_TUNE2_LEN           (4)
uint32_t DRX_TUNE2[][2] = {             /* Row = PAC, Col = PRF */
    { 0x311A002DUL, 0x313B006BUL},      /* PAC 8 */
    { 0x331A0052UL, 0x333B00BEUL},      /* PAC 16 */
    { 0x351A009AUL, 0x353B015EUL},      /* PAC 32 */
    { 0x371A011DUL, 0x373B0296UL}       /* PAC 64 */
};

/* offset from DRX_CONF_ID in bytes */
#define DRX_TUNE4H_OFFSET       0x26    /* 7.2.40.10 Sub-Register 0x27:26  DRX_TUNE4H */
#define DRX_TUNE4H_LEN          (2)
#define DRX_TUNE4H_MASK         0xFFFF
#define DRX_TUNE4H_PRE64        0x0010
#define DRX_TUNE4H_PRE128PLUS   0x0028

bool DecaDuino::setPreambleLength (int plength) {

	uint32_t ui32t;
	uint32_t mask;
    int recommended_pac_size_index;
	switch(plength){
		case 64:
			mask = 0x00040000;
            recommended_pac_size_index = 0;
			break;
		case 128:
			mask = 0x00140000;
            recommended_pac_size_index = 0;
			break;
		case 256:
			mask = 0x00240000;
            recommended_pac_size_index = 1;
			break;
		case 512:
			mask = 0x00340000;
            recommended_pac_size_index = 1;
			break;
		case 1024:
			mask = 0x00080000;
            recommended_pac_size_index = 2;
			break;
		case 1536:
			mask = 0x00180000;
            recommended_pac_size_index = 3;
			break;
		case 2048:
			mask = 0x00280000;
            recommended_pac_size_index = 3;
			break;
		case 4096:
			mask = 0x000C0000;
            recommended_pac_size_index = 3;
			break;
		default:
			return false;
	}

	ui32t = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
	ui32t = ui32t & 0xFFC3FFFF; // bits 21, 20, 19, 18 to zero
	ui32t |= mask;
	writeSpiUint32(DW1000_REGISTER_TX_FCTRL, ui32t);

    int prf_index = getRxPrf() == 16 ? 0 : 1;
    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE2_OFFSET,
            (uint8_t*)&DRX_TUNE2[recommended_pac_size_index][prf_index], DRX_TUNE2_LEN);

    dw1000_datarate_t datarate = getDataRate();
    uint8_t drx_tun1b[2];
    if (datarate == DW1000_DATARATE_110KBPS){
        encodeUint16(0x064, drx_tun1b);
    }
    else {
        if (plength == 64){
            encodeUint16(0x0010, drx_tun1b);
        }
        else {
            encodeUint16(0x0020, drx_tun1b);
        }
    }
    writeSpiSubAddress(DW1000_REGISTER_DRX_CONF, DW1000_REGISTER_OFFSET_DRX_TUNE1B,
                drx_tun1b, 2);

    uint16_t tune4h;
    if (plength > 64)
    {
        tune4h = DRX_TUNE4H_PRE128PLUS;
    }
    else
    {
        tune4h = DRX_TUNE4H_PRE64;
    }
    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE4H_OFFSET,
            (uint8_t*)&tune4h, DRX_TUNE4H_LEN);
	return true;
}


uint16_t DecaDuino::getAntennaDelay() {

	return antennaDelay;
}


void DecaDuino::setAntennaDelay(uint16_t newAntennaDelay, bool trackChanges) {

    uint32_t tempVar;   // var to prevent overflow during computations

    tempVar = newAntennaDelay*44; // TX antenna delay is 44% of the total delay (see DW doc on antenna delay calibration)
    tempVar /= 100;
	setTXAntennaDelayReg(tempVar);

	tempVar = newAntennaDelay*56; // RX antenna delay is 56% of the total delay (see DW doc on antenna delay calibration)
    tempVar /= 100;
	setRXAntennaDelayReg(tempVar);

	antennaDelay = newAntennaDelay;
	_antennaDelayTracksChanges = trackChanges;
}

void DecaDuino::useCalibratedAntennaDelay(){
    _antennaDelayTracksChanges = true;
    setCalibratedAntennaDelay();
}

void DecaDuino::setCalibratedAntennaDelay(){
    int channelIndex = getChannel() -1 ;
    uint8_t prfIndex = _rxPrf >> 6;
    setAntennaDelay(calibratedAntennaDelay[channelIndex][prfIndex], true);
}


uint16_t DecaDuino::getTXAntennaDelayReg(){

	uint8_t buf[2];

	readSpi(DW1000_REGISTER_TX_ANTD, buf, 2);
	return decodeUint16(buf);
}


void DecaDuino::setTXAntennaDelayReg(uint16_t newAntennaDelay) {

	uint8_t buf[2];
	encodeUint16(newAntennaDelay, buf);
	writeSpi(DW1000_REGISTER_TX_ANTD, buf, 2);
}


uint16_t DecaDuino::getRXAntennaDelayReg(){

    uint8_t buf[2];

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE, DW1000_REGISTER_LDE_INTERFACE_LDE_RXANTD_OFFSET, buf, 2);
    return decodeUint16(buf);
}

void DecaDuino::setRXAntennaDelayReg(uint16_t newAntennaDelay) {

    uint8_t buf[2];
    encodeUint16(newAntennaDelay, buf);
    writeSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE, DW1000_REGISTER_LDE_INTERFACE_LDE_RXANTD_OFFSET, buf, 2);
}


uint8_t DecaDuino::getTemperatureRaw() {

	uint8_t u8t;

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif

		u8t = 0x80; writeSpiSubAddress(0x28, 0x11, &u8t, 1); // 1. Write Sub-Register 28:11 1byte 0x80
		u8t = 0x0A; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 2. Write Sub-Register 28:12 1byte 0x0A
		u8t = 0x0F; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 3. Write Sub-Register 28:12 1byte 0x0F
		u8t = 0x01; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 4. Write Register 2A:00 1byte 0x01
		u8t = 0x00; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 5. Write Register 2A:00 1byte 0x00
		readSpiSubAddress(0x2A, 0x04, &u8t, 1); // 6. Read Register 2A:04 1byte 8 bit Temperature reading
	}
    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif

	return u8t;
}


uint8_t DecaDuino::getVoltageRaw() {

	uint8_t u8t;

    #ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif

		u8t = 0x80; writeSpiSubAddress(0x28, 0x11, &u8t, 1); // 1. Write Sub-Register 28:11 1byte 0x80
		u8t = 0x0A; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 2. Write Sub-Register 28:12 1byte 0x0A
		u8t = 0x0F; writeSpiSubAddress(0x28, 0x12, &u8t, 1); // 3. Write Sub-Register 28:12 1byte 0x0F
		u8t = 0x01; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 4. Write Register 2A:00 1byte 0x01
		u8t = 0x00; writeSpiSubAddress(0x2A, 0x00, &u8t, 1); // 5. Write Register 2A:00 1byte 0x00
		readSpiSubAddress(0x2A, 0x03, &u8t, 1); // 6. Read Register 2A:03 1byte 8 bit Voltage reading
	}
    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif

	return u8t;
}


float DecaDuino::getTemperature(void) {
	uint8_t u8t;
	uint8_t buf_16[2];
	uint8_t buf_32[4];
	float temp,diff;
	uint8_t t23,raw_temp;

	#ifdef UWB_MODULE_DWM1001
	uint32_t prim = begin_atomic();
	{
	#else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	#endif

		buf_16[0] = 0x09;buf_16[1] = 0x00; writeSpiSubAddress(0x2D, 0x04, buf_16, 2);
		u8t= 0x03; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		u8t= 0x00; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		readSpiSubAddress(0x2D,0x0A,buf_32,4);
	}

    #ifdef UWB_MODULE_DWM1001
    end_atomic(prim);
    #endif

	raw_temp = getTemperatureRaw();
	t23 =   buf_32[0];
	diff = (float) (raw_temp - t23);
	temp =   diff * 1.14 + 23.0;
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

	#ifdef UWB_MODULE_DWM1001
    uint32_t prim = begin_atomic();
    {
    #else
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    #endif
		buf_16[0] = 0x08;buf_16[1] = 0x00; writeSpiSubAddress(0x2D, 0x04, buf_16, 2);
		u8t= 0x03; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		u8t= 0x00; writeSpiSubAddress(0x2D, 0x06, &u8t, 1);
		readSpiSubAddress(0x2D,0x0A,buf_32,4);

	}
	#ifdef UWB_MODULE_DWM1001
	end_atomic(prim);
	#endif
	raw_v = (float)getVoltageRaw();
	v33 =  (float) buf_32[0];
	v =  ( ( raw_v - v33 ) / 173) + 3.3;

	return v;


}


RXTime_t DecaDuino::getRxTimeRegister(){
    uint8_t buf[14] = {0} ;
    readSpi(DW1000_REGISTER_RX_TIME, buf, 14);

    RXTime_t data;
    data.RX_STAMP = decodeUint40(&(buf[0]));
    data.FP_INDEX = decodeUint16(&(buf[5]));
    data.FP_AMPL1 = decodeUint16(&(buf[7]));
    data.RX_RAWST = decodeUint40(&(buf[9]));
    return data;
}

int DecaDuino::getRxTimeRegisterAsJSon(const RXTime_t &data, char *buf, int maxlen){
    char stamp[20];
    char coarse[20];
    return snprintf(buf, maxlen,"{\"RX_STAMP\": %s,\"FP_INDEX\": %u, \"FP_AMPL1\": %u, \"RX_RAWST\": %s}",
            ulltoa(data.RX_STAMP,stamp,sizeof(stamp)),
            data.FP_INDEX,
            data.FP_AMPL1,
            ulltoa(data.RX_RAWST,coarse,sizeof(coarse)));
}
int DecaDuino::getRxTimeRegisterAsJSon(char *buf, int maxlen){
    RXTime_t data = getRxTimeRegister();
    return getRxTimeRegisterAsJSon(data, buf, maxlen);
}

RXFQual_t DecaDuino::getRxQualityRegister(){
    uint8_t buf[8] = {0} ;
    readSpi(DW1000_REGISTER_RX_RFQUAL, buf, 8);
    RXFQual_t data;
    data.STD_NOISE = decodeUint16(&(buf[0]));
    data.FP_AMPL2 = decodeUint16(&(buf[2]));
    data.FP_AMPL3= decodeUint16(&(buf[4]));
    data.CIR_PWR= decodeUint16(&(buf[6]));
    return data;
}

int DecaDuino::getRxQualityRegisterAsJSon(const RXFQual_t &data, char *buf, int maxlen){
    return snprintf(buf, maxlen,"{\"STD_NOISE\": %lu,\"FP_AMPL2\": %lu, \"FP_AMPL3\": %lu, \"CIR_PWR\": %lu}",
                    data.STD_NOISE,
                    data.FP_AMPL2,
                    data.FP_AMPL3,
                    data.CIR_PWR);
}

int DecaDuino::getRxQualityRegisterAsJSon(char *buf, int maxlen){
    RXFQual_t data = getRxQualityRegister();
    return getRxQualityRegisterAsJSon(data, buf, maxlen);
}


RXFInfo_t DecaDuino::getRxFrameInfoRegister(){
    uint32_t buf = readSpiUint32(DW1000_REGISTER_RX_FINFO);
    RXFInfo_t data;
    data.RXFLEN = (buf & DW1000_REGISTER_RX_FINFO_RXFLEN_MASK) >> DW1000_REGISTER_RX_FINFO_RXFLEN_SHIFT;
    data.RXNSPL = (buf & DW1000_REGISTER_RX_FINFO_RXFNSPL_MASK) >> DW1000_REGISTER_RX_FINFO_RXFNSPL_SHIFT;
    data.RXBR   = (buf & DW1000_REGISTER_RX_FINFO_RXBR_MASK) >> DW1000_REGISTER_RX_FINFO_RXBR_SHIFT;
    data.RNG    = (buf & DW1000_REGISTER_RX_FINFO_RNG_MASK) >> DW1000_REGISTER_RX_FINFO_RNG_SHIFT;
    data.RXPRFR = (buf & DW1000_REGISTER_RX_FINFO_RXPRFR_MASK) >> DW1000_REGISTER_RX_FINFO_RXPRFR_SHIFT;
    data.RXPSR  = (buf & DW1000_REGISTER_RX_FINFO_RXPSR_MASK) >> DW1000_REGISTER_RX_FINFO_RXPSR_SHIFT;
    data.RXPACC = (buf & DW1000_REGISTER_RX_FINFO_RXPACC_MASK) >> DW1000_REGISTER_RX_FINFO_RXPACC_SHIFT;
    return data;
}

int DecaDuino::getRxFrameInfoRegisterAsJSon(const RXFInfo_t &data,char *buf, int maxlen){
    return snprintf(buf, maxlen,"{\"RXFLEN\": %" PRIu16 ",\"RXNSPL\": %u, \"RXBR\": %u, \"RNG\": %u,\"RXPRFR\":%u, \"RXPSR\": %u, \"RXPACC\": %" PRIu16 "}",
                data.RXFLEN,
                data.RXNSPL,
                data.RXBR,
                data.RNG,
                data.RXPRFR,
                data.RXPSR,
                data.RXPACC);
}
int DecaDuino::getRxFrameInfoRegisterAsJSon(char *buf, int maxlen){
    RXFInfo_t data = getRxFrameInfoRegister();
    return getRxFrameInfoRegisterAsJSon(data, buf, maxlen);
}

channelCTRL_t DecaDuino::getChannelControlRegister(){
    uint32_t buf = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
    channelCTRL_t data;
    data.TX_CHAN = (buf & DW1000_REGISTER_CHAN_CTRL_TX_CHAN_MASK) >> DW1000_REGISTER_CHAN_CTRL_TX_CHAN_SHIFT ;
    data.RX_CHAN = (buf & DW1000_REGISTER_CHAN_CTRL_RX_CHAN_MASK) >> DW1000_REGISTER_CHAN_CTRL_RX_CHAN_SHIFT ;
    data.DWSFD = (buf & DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK) >> DW1000_REGISTER_CHAN_CTRL_DWSFD_SHIFT ;
    data.RXPRF = (buf & DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK) >> DW1000_REGISTER_CHAN_CTRL_RXPRF_SHIFT ;
    data.TNSSFD = (buf & DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK) >> DW1000_REGISTER_CHAN_CTRL_TNSSFD_SHIFT ;
    data.RNSSFD = (buf & DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK) >> DW1000_REGISTER_CHAN_CTRL_RNSSFD_SHIFT ;
    data.TX_PCODE = (buf & DW1000_REGISTER_CHAN_CTRL_TX_PCODE_MASK) >> DW1000_REGISTER_CHAN_CTRL_TX_PCODE_SHIFT ;
    data.RX_PCODE = (buf & DW1000_REGISTER_CHAN_CTRL_RX_PCODE_MASK) >> DW1000_REGISTER_CHAN_CTRL_RX_PCODE_SHIFT ;
    return data;
}

int DecaDuino::getChannelControlRegisterAsJSon(const channelCTRL_t &data, char *buf, int maxlen){
    return snprintf(buf, maxlen,"{\"TX_CHAN\": %u,\"RX_CHAN\": %u, \"DWSFD\": %u, \"RXPRF\": %u,\"TNSSFD\":%u, \"RNSSFD\": %u, \"TX_PCODE\": %u, \"RX_PCODE\": %u}",
                    data.TX_CHAN,
                    data.RX_CHAN,
                    data.DWSFD,
                    data.RXPRF,
                    data.TNSSFD,
                    data.RNSSFD,
                    data.TX_PCODE,
                    data.RX_PCODE);
}
int DecaDuino::getChannelControlRegisterAsJSon(char *buf, int maxlen){
    channelCTRL_t data = getChannelControlRegister();
    return getChannelControlRegisterAsJSon(data, buf, maxlen);
}

LDEInterface_t DecaDuino::getLDEInterfaceRegister(){
    uint8_t buf[2];
    LDEInterface_t data;
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_THRESH_OFFSET,buf,2);
    data.LDE_THRESH = decodeUint16(buf);

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    data.NTM = (buf[0] & DW1000_REGISTER_LDE_INTERFACE_NTM_MASK) >> DW1000_REGISTER_LDE_INTERFACE_NTM_SHIFT;
    data.PMULT= (buf[0] & DW1000_REGISTER_LDE_INTERFACE_PMULT_MASK) >> DW1000_REGISTER_LDE_INTERFACE_PMULT_SHIFT;

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_PPINDX_OFFSET,buf,2);
    data.LDE_PPINDX= decodeUint16(buf);

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_PPAMPL_OFFSET,buf,2);
    data.LDE_PPAMPL = decodeUint16(buf);

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_RXANTD_OFFSET,buf,2);
    data.LDE_RXANTD = decodeUint16(buf);

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG2_OFFSET,buf,2);
    data.LDE_CFG2 = decodeUint16(buf);

    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_REPC_OFFSET,buf,2);
    data.LDE_REPC = decodeUint16(buf);

    return data;
}

int DecaDuino::getChannelLDEInterfaceAsJSon(const LDEInterface_t &data, char *buf, int maxlen){
    return snprintf(buf, maxlen,"{\"LDE_THRESH\": %" PRIu16 ",\"NTM\": %u, \"PMULT\": %" PRIu16 ",\"LDE_PPINDX\":%" PRIu16 ", \"LDE_PPAMPL\": %" PRIu16 ", \"LDE_RXANTD\": %" PRIu16 ", \"LDE_CFG2\": %" PRIu16 ", \"LDE_REPC\": %" PRIu16 "}",
                    data.LDE_THRESH,
                    data.NTM,
                    data.PMULT,
                    data.LDE_PPINDX,
                    data.LDE_PPAMPL,
                    data.LDE_RXANTD,
                    data.LDE_CFG2,
                    data.LDE_REPC);
}
int DecaDuino::getChannelLDEInterfaceAsJSon(char *buf, int maxlen){
    LDEInterface_t data = getLDEInterfaceRegister();
    return getChannelLDEInterfaceAsJSon(data, buf, maxlen);
}

uint8_t DecaDuino::getRXM110K(){
    uint32_t buf = readSpiUint32(DW1000_REGISTER_SYS_CFG);
    return (buf & DW1000_REGISTER_SYS_CFG_RXM110K_MASK) >> DW1000_REGISTER_SYS_CFG_RXM110K_SHIFT;
}


uint8_t DecaDuino::getSFD_LENGTH(){
    uint8_t buf;
    readSpiSubAddress(DW1000_REGISTER_USR_SFD,DW1000_REGISTER_USR_SFD_LENGTH_OFFSET,&buf,1);
    return buf;
}

void DecaDuino::setSFD_LENGTH(uint8_t SFD_LENGTH){
    uint8_t buf  = SFD_LENGTH;
    writeSpiSubAddress(DW1000_REGISTER_USR_SFD,DW1000_REGISTER_USR_SFD_LENGTH_OFFSET,&buf,1);
}

void DecaDuino::setSFDTimeout(uint16_t timeout){
    uint8_t buf[2];
    encodeUint16(timeout,buf);
    writeSpiSubAddress(DW1000_REGISTER_DRX_CONF,DW1000_REGISTER_OFFSET_DRX_SFDTOC,buf,2);
}

uint16_t DecaDuino::getRXPACC_NOSAT(){
    uint8_t buf[2];
    readSpiSubAddress(DW1000_REGISTER_DRX_CONF,DW1000_REGISTER_OFFSET_RXPACC_NOSAT,buf,2);
    return decodeUint16(buf);
}

void DecaDuino::enableCIRAccumulatorRead(bool enable){
    uint8_t pmscctrl0[4];
    readSpi(DW1000_REGISTER_PMSC_CTRL0,pmscctrl0,4);
    if (enable){
        // set FACE bit and RXCLKS to values required to read CIR accumulator
        pmscctrl0[0] &= 0xB3;
        pmscctrl0[0] |= 0x48;
        // set ACME bit
        pmscctrl0[1] |= 0x80;
    }
    else {
        // unset FACE bit, an set RXCLKS to auto
        pmscctrl0[0] &= 0xB3;
        // unset ACME bit
        pmscctrl0[1] &= (~0x80);
    }
    writeSpi(DW1000_REGISTER_PMSC_CTRL0,pmscctrl0,4);
}

int DecaDuino::getCIRAccumulator(CIRSample_t *buffer, size_t arrayLength, unsigned int startIndex, int readLength){
    unsigned int numSamples = getRxPrf() == 16 ? 992 : 1016;                    // CIR contains 992 samples if PRF == 16 MHz, 1016 if PRF == 64, so we start with this number of samples to read
    numSamples -= startIndex;                                                   // If we do not start at sample 0, there are less samples to read
    if (readLength >= 0 && readLength < numSamples) numSamples = readLength;    // The user has requested less samples to be read, so be it.
    unsigned int bulkBonus;
    if ( numSamples >= arrayLength ){                            // make sure that we will not write too much to the buffer
        numSamples = arrayLength;
        bulkBonus = 0;
    }
    else {
        // there is some space left in the buffer : read everything in one single pass
        bulkBonus = 1;
    }

    // enable CIR accumulator read
    enableCIRAccumulatorRead(true);

    // extreme bulk reading : read everything in (almost) one call, use destination buffer as temporary buffer.
    int i;
    uint8_t *buff = (uint8_t*)buffer;   // in place reading
    readSpiSubAddress(0x25, startIndex*4, buff, (startIndex + numSamples)*4  + bulkBonus);  // read everything directly into  buf
    uint8_t lastOne[2];
    if (!bulkBonus) {
        readSpiSubAddress(0x25, (startIndex + numSamples)*4 - 1 , lastOne, 2);  // read last byte (due to first byte to be dropped
    }
    // discard first byte, and handle byte order (in place)
    for (i = 0; i < numSamples - 1 + bulkBonus; i++){
        buffer[i].r = buff[ i*4 + 1] | buff[ i*4 + 2] << 8;
        buffer[i].i = buff[ i*4 + 3] | buff[ i*4 + 4] << 8;
    }
    if (!bulkBonus){
        // hand-processing of last one if we had to read it separately
        buffer[i].r = buff[ i*4 + 1] | buff[ i*4 + 2] << 8;
        buffer[i].i = buff[ i*4 + 3] | lastOne[1] << 8;
        i++;
    }

    // reset CIR accumulator read
    enableCIRAccumulatorRead(false);

    return i;
}

int DecaDuino::CIRAccumulatorToJSon(CIRSample_t *samples, uint16_t numSamples, char* buf, uint16_t maxlen){
    unsigned int c=0;
    buf[c++] = '[';
    unsigned int i = 0;
    for (; i < numSamples && c < maxlen ; i++){
        c += snprintf(&(buf[c]),maxlen-c,"{\"r\": %" PRId16 ", \"i\": %" PRId16 "}, ",samples[i].r,samples[i].i);
    }
    if (c >= (maxlen - 2)){
        strncpy(buf,"buf too small to hold whole representation",maxlen);
        buf[maxlen-1] = '\0';
        return c;
    }
    buf[c-2] = ']';
    buf[c-1] = '\0';
    return c;
}

int DecaDuino::getCIRAccumulatorAsJSon(char* buf, uint16_t maxlen, unsigned int startIndex, int readLength){
    CIRSample_t samples[1016];
    int numSamples = getCIRAccumulator(samples,1016,startIndex,readLength);
    return CIRAccumulatorToJSon(samples, numSamples, buf, maxlen);
}

int DecaDuino::getCIRAccumulatorAsBase64JSon(char* buf, uint16_t maxlen, unsigned int startIndex, int readLength){
    CIRSample_t samples[1016];
    int numSamples = getCIRAccumulator(samples,1016,startIndex,readLength);
    return CIRAccumulatorToBase64JSon(samples, numSamples, buf, maxlen);
}

int DecaDuino::CIRAccumulatorToBase64JSon(CIRSample_t *samples, uint16_t numSamples, char* buf, uint16_t maxlen){
    unsigned int c=0; // total character count
    buf[c++] = '"';

    // If necessary, rewrite samples to network byte order, i.e. big endian
#ifndef BYTE_ORDER
    #error "BYTE_ORDER must be defined"
#endif
#if BYTE_ORDER==LITTLE_ENDIAN
    unsigned int i = 0;
    for (; i < numSamples ; i += 1){
        // poor man's htons :
        samples[i].r = ( (samples[i].r & 0xff00) >> 8) | ((samples[i].r & 0x00ff) << 8) ;
        samples[i].i = ( (samples[i].i & 0xff00) >> 8) | ((samples[i].i & 0x00ff) << 8) ;
    }
#endif

    // check if there is enough space to store the whole string
    if ( (c + encode_base64_length(numSamples*4)) >= (maxlen - 2) ){
        strncpy(buf,"buf too small to hold whole representation",maxlen);
        buf[maxlen-1] = '\0';
        return c;
    }
    c += encode_base64( (unsigned char*)samples, numSamples*4, (unsigned char *)&(buf[c]));   // 4 bytes per sample, struct should be packed because 16-bits fields.
    buf[c++] = '"';
    buf[c++] = '\0';
    return c;
}

void DecaDuino::readAllRXInfos(registerDump_t *registers, bool cir, int cir_first_index, int cir_num_samples){
    registers->RXM110K = this->getRXM110K();
    registers->RXPACC_NOSAT = this->getRXPACC_NOSAT();
    registers->SFD_LENGTH = this->getSFD_LENGTH();
    registers->LDE_IF = this->getLDEInterfaceRegister();
    registers->CHAN_CTRL = this->getChannelControlRegister();
    registers->RX_FINFO = this->getRxFrameInfoRegister();
    registers->RX_FQUAL = this->getRxQualityRegister();
    registers->RX_TIME = this->getRxTimeRegister();

    if (cir) {
        registers->CIR_length = this->getCIRAccumulator(registers->CIR,1017, cir_first_index, cir_num_samples);
    }
}

int DecaDuino::printAllRXInfos(char* to, int maxSize, bool cir, int cir_first_index, int cir_num_samples){


    registerDump_t registers;

    uint32_t read_duration = 0;
    read_duration = millis();
    readAllRXInfos(&registers, cir, cir_first_index, cir_num_samples);
    read_duration = millis() - read_duration;


    char info[128];
    this->getRxFrameInfoRegisterAsJSon(registers.RX_FINFO,info,128);

    char qual[128];
    this->getRxQualityRegisterAsJSon(registers.RX_FQUAL,qual,128);

    char stamp[95];
    this->getRxTimeRegisterAsJSon(registers.RX_TIME,stamp,95);


    char bigbuf[8192];
    bigbuf[0] = '"';
    bigbuf[1] = '"';
    bigbuf[2] = '\0';
    if (cir){
        this->CIRAccumulatorToBase64JSon(registers.CIR,registers.CIR_length,bigbuf,sizeof(bigbuf));
    }


    char channel[128];
    this->getChannelControlRegisterAsJSon(registers.CHAN_CTRL,channel,128);

    char LDEIf[256];
    this->getChannelLDEInterfaceAsJSon(registers.LDE_IF,LDEIf,256);

    return snprintf(to, maxSize,"{\"registerDump\":{\"RX_FINFO\": %s,\"RX_FQUAL\": %s,\"RX_TIME\": %s,\"ACC_MEM\": %s, \"ACC_MEM_first_index\":%d,\"CHAN_CTRL\": %s, \"RXPACC_NOSAT\": %lu, \"SFD_LENGTH\": %lu, \"RXM110K\": %u, \"LDE_IF\": %s}, \"registers_read_duration\":%d}",
            info,
            qual,
            stamp,
            bigbuf,
            cir_first_index,
            channel,
            this->getRXPACC_NOSAT(),
            this->getSFD_LENGTH(),
            this->getRXM110K(),
            LDEIf,
            read_duration
            );
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

	uint8_t ui8t;

#ifdef DECADUINO_DEBUG
	sprintf((char*)debugStr,"deepsleep request");
	Serial.println((char*)debugStr);
#endif

	readSpiSubAddress(DW1000_REGISTER_AON_CFG0, DW1000_REGISTER_OFFSET_AON_CFG0, &ui8t, 1);
	ui8t |= DW1000_REGISTER_AON_CFG0_SLEEP_EN_MASK;
	ui8t |= DW1000_REGISTER_AON_CFG0_WAKE_SPI_MASK;
	writeSpiSubAddress(DW1000_REGISTER_AON_CFG0, DW1000_REGISTER_OFFSET_AON_CFG0, &ui8t, 1);

	readSpiSubAddress(DW1000_REGISTER_AON_CTRL, DW1000_REGISTER_OFFSET_AON_CTRL, &ui8t, 1);
	ui8t |= DW1000_REGISTER_AON_CTRL_UPL_CFG_MASK;
	writeSpiSubAddress(DW1000_REGISTER_AON_CTRL, DW1000_REGISTER_OFFSET_AON_CTRL, &ui8t, 1);
	delay(1);

	// The DWM1000 is now deepsleepping

	trxStatus = DW1000_TRX_STATUS_DEEPSLEEP;
}


void DecaDuino::wakeRequest(void) {

	digitalWrite(_slaveSelectPin, LOW);
 	delayMicroseconds(600);
	digitalWrite(_slaveSelectPin, HIGH);
 	delay(7);
	trxStatus = DW1000_TRX_STATUS_IDLE;
}


float DecaDuino::getNLOSIndication(void) {

	float indicator = 0;
	uint8_t reg12[8];
	uint8_t reg15[14];
	uint16_t C;	//bytes 7 and 6 of reg12
	uint16_t F1;	//bytes 8 and 7 of reg15
	uint16_t F2;	//bytes 3 and 2 of reg12
	uint16_t F3;	//bytes 5 and 4 of reg12

	//read register 0x12:DW1000_REGISTER_RX_RFQUAL
	readSpi(DW1000_REGISTER_RX_RFQUAL, reg12, 8);
	C = reg12[7]*256 + reg12[6];
	F2 = reg12[3]*256 + reg12[2];
	F3 = reg12[5]*256+ reg12[4];

	//read register 0x15:DW1000_REGISTER_RX_TIME
	readSpi(DW1000_REGISTER_RX_TIME	, reg15, 14);
	F1 = reg15[8]*256 + reg12[7];

	//compute LOS/NLOS indicator
	indicator = 131072.0*C/(F1*F1 + F2*F2 + F3*F3);
#ifdef DECADUINO_DEBUG
	printf("%d\n",C);
	printf("%d\n",F1);
	printf("%d\n",F2);
	printf("%d\n",F3);
	printf("i=%f\n",indicator);
#endif
	return indicator;
}

/* offset from DRX_CONF_ID in bytes */
#define DRX_TUNE0b_OFFSET       (0x02)  /* sub-register 0x02 is a 16-bit tuning register. */
#define DRX_TUNE0b_LEN          (2)
#define DRX_TUNE0b_MASK         0xFFFF  /* 7.2.40.2 Sub-Register 0x27:02  DRX_TUNE0b */

/* offset from DRX_CONF_ID in bytes */
#define DRX_TUNE1b_OFFSET       0x06    /* 7.2.40.4 Sub-Register 0x27:06  DRX_TUNE1b */
#define DRX_TUNE1b_LEN          (2)
#define DRX_TUNE1b_MASK         0xFFFF
#define DRX_TUNE1b_110K         0x0064
#define DRX_TUNE1b_850K_6M8     0x0020
#define DRX_TUNE1b_6M8_PRE64    0x0010

#define SYS_CFG_RXM110K         0x00400000UL    /* Receiver Mode 110 kbps data rate */

// SFD Threshold
static const uint16_t DRX_TUNE0b[][2] =         /* [bitrate][SFD], with SFD == 0 for standard SFD, SFD == 1 for decawave-recommended SFD*/
{
    {0x000A, 0x0016},                                     /* 100 kbps */
    {0x0001, 0x0006},                                     /* 850 kbps */
    {0x0001, 0x0002}                                      /* 6.8 Mbps */
};

#define TX_FCTRL_TXBR_MASK      0x00006000UL    /* bit mask to access Transmit Bit Rate */
static const uint32_t TX_FCTRL_TXBR[] = {
    0x00000000UL,                               /* 100 kbps */
    0x00002000UL,                               /* 850 kbps */
    0x00004000UL                                /* 6.8 Mbps */
};

dw1000_datarate_t DecaDuino::getDataRate() {
	uint32_t tx_fctrl = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
    uint32_t tx_fctrl_txbr = tx_fctrl & TX_FCTRL_TXBR_MASK;
    for (int i = 0; i < sizeof(TX_FCTRL_TXBR); i++)
    {
        if (tx_fctrl_txbr == TX_FCTRL_TXBR[i]) return (dw1000_datarate_t)i;
    }
}

void DecaDuino::setDataRate(dw1000_datarate_t rate) {
    // DTUNE0
    _datarate = rate;
    int SFDIndex = _DWSFD ? 1 : 0;
    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE0b_OFFSET,
            (uint8_t*)&DRX_TUNE0b[rate][SFDIndex], DRX_TUNE0b_LEN);

    uint32_t tune1b;
	uint32_t sys_cfg = readSpiUint32(DW1000_REGISTER_SYS_CFG);
    if (rate == DW1000_DATARATE_110KBPS)
    {
        tune1b = DRX_TUNE1b_110K;
        sys_cfg |= SYS_CFG_RXM110K;
    }
    else
    {
        sys_cfg &= ~SYS_CFG_RXM110K;
        if (getPreambleLength() == 64)
        {
            tune1b = DRX_TUNE1b_6M8_PRE64;
        }
        else
        {
            tune1b = DRX_TUNE1b_850K_6M8;
        }
    }
    writeSpiUint32(DW1000_REGISTER_SYS_CFG, sys_cfg);
    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE1b_OFFSET,
            (uint8_t*)&tune1b, DRX_TUNE1b_LEN);

	uint32_t tx_fctrl = readSpiUint32(DW1000_REGISTER_TX_FCTRL);
    tx_fctrl &= ~TX_FCTRL_TXBR_MASK;
    tx_fctrl |= TX_FCTRL_TXBR[rate];
    writeSpiUint32(DW1000_REGISTER_TX_FCTRL, tx_fctrl);

    if (_DWSFD) {
        setDecaWaveSFD();
    }
    else {
        setStandardSFD();
    }
}

uint8_t DecaDuino::getNTM(void){
    uint8_t buf[1];
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    return (buf[0] & DW1000_REGISTER_LDE_INTERFACE_NTM_MASK) >> DW1000_REGISTER_LDE_INTERFACE_NTM_SHIFT;
}

bool DecaDuino::setNTM(uint8_t NTM){
    if (NTM > 31) return false;
    uint8_t buf[1];
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    buf[0] = (buf[0] & ~DW1000_REGISTER_LDE_INTERFACE_NTM_MASK) | (NTM << DW1000_REGISTER_LDE_INTERFACE_NTM_SHIFT);
    writeSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    return true;
}

uint8_t DecaDuino::getPMULT(void){
    uint8_t buf[1];
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    return (buf[0] & DW1000_REGISTER_LDE_INTERFACE_PMULT_MASK) >> DW1000_REGISTER_LDE_INTERFACE_PMULT_SHIFT;
}

bool DecaDuino::setPMULT(uint8_t PMULT){
    if (PMULT > 7) return false;
    uint8_t buf[1];
    readSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    buf[0] = (buf[0] & ~DW1000_REGISTER_LDE_INTERFACE_PMULT_MASK) | (PMULT << DW1000_REGISTER_LDE_INTERFACE_PMULT_SHIFT);
    writeSpiSubAddress(DW1000_REGISTER_LDE_INTERFACE,DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET,buf,1);
    return true;
}


void DecaDuino::setStandardSFD(){
    _DWSFD = false;
    uint32_t buf;
    buf = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
    buf = buf & (~( DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK) );
    writeSpiUint32(DW1000_REGISTER_CHAN_CTRL,buf);
    int rate = getDataRate();
    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE0b_OFFSET,
            (uint8_t*)&DRX_TUNE0b[rate][0], DRX_TUNE0b_LEN);
}

void DecaDuino::setDecaWaveSFD(){
    _DWSFD = true;
    uint32_t cfgSet;
    uint32_t cfgUnset;
    uint16_t drx_tun0;
    uint8_t sfdLength = 0;
    dw1000_datarate_t datarate = getDataRate();
    switch (datarate){
    case DW1000_DATARATE_6_8MBPS :
        cfgSet = 0x0;
        cfgUnset = DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK;
        drx_tun0 = DRX_TUNE0b[datarate][1];
        break;

    case DW1000_DATARATE_850KBPS :
        cfgSet = DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK;
        cfgUnset = 0x0;
        sfdLength = 16;
        drx_tun0 = DRX_TUNE0b[datarate][1];
        break;

    case DW1000_DATARATE_110KBPS :
        cfgSet = DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK;
        cfgUnset = DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK | DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK ;
        sfdLength = 64; // should not matter, here to try something
        drx_tun0 = DRX_TUNE0b[datarate][1];
        break;
    }

    uint32_t chanCtrlBuf = readSpiUint32(DW1000_REGISTER_CHAN_CTRL);
    chanCtrlBuf = chanCtrlBuf & ~cfgUnset;
    chanCtrlBuf = chanCtrlBuf | cfgSet;
    writeSpiUint32(DW1000_REGISTER_CHAN_CTRL,chanCtrlBuf);

    writeSpiSubAddress(DRX_CONF_ID, DRX_TUNE0b_OFFSET,
            (uint8_t*)&drx_tun0, DRX_TUNE0b_LEN);

    if (sfdLength) setSFD_LENGTH(sfdLength);
}


void DecaDuino::setDefaultChannelConfig(){
    disableNLOSTunings();
    setChannel(5);
    setPcode(4);
    setPrf(16);
    setDataRate(DW1000_DATARATE_6_8MBPS);
    setSmartTxPower();
    setPreambleLength(128);
    setSFDTimeout(4161); // default sensible value
}

void DecaDuino::enableNLOSTunings(){
    _NLOSOptims = true;
    setRxPrf(getRxPrf()); // will set the optim for NLOS if appropriate
    setNTM(7);
    setPMULT(0);
}

void DecaDuino::disableNLOSTunings(){
    _NLOSOptims = false;
    setRxPrf(getRxPrf()); // will unset the optim for NLOS if appropriate
    setNTM(13);
    setPMULT(3);
}

bool DecaDuino::getNLOSTunings(){
 return _NLOSOptims;
}

uint32_t DecaDuino::getDevID(void) {
    return readSpiUint32(DW1000_REGISTER_DEV_ID);
}
