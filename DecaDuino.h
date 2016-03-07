// DecaDuino.h
// 
// Another DecaWave DW1000 driver for Arduino
// See the README file in this directory for documentation
//  
/// \mainpage DecaDuino library for Arduino
/// 
/// Get the latest version of this documentation here: https://www.irit.fr/~Adrien.Van-Den-Bossche/decaduino/
/// 
/// DecaDuino is an Arduino library which provides a driver for the DecaWave DW1000 transceiver and modules based on this transceiver, such as DecaWave DWM1000. Since the DW1000 is based on a Ultra Wide Band (UWB) Physical layer, in addition to wireless communication, DecaDuino supports Time-of-Flight (ToF) ranging and can be used as an open framework for protocol evaluation.
///
/// DecaDuino is a <i>Physical-layer Service Access Point (PHY-SAP)</i>. It provides the two conventional <i>Physical-Data</i> (PD) and <i>Physical Layer Management Entity</i> (PLME) SAPs which enable MAC-level protocols to send/receive data and configure the transceiver (channel, transmission rate, preamble parameters...). Since this framework was designed to aid in the implementation of Time-of-Flight based ranging protocols, DecaDuino also provides access to the DW1000's Physical-level high precision timer (64GHz/40bit) which enables precise message timestamping at both transmission (t_TX) and reception (t_RX). Finally, DecaDuino implements DW1000's advanced synchronization/timestamping functionalities such as delayed transmission and receiver skew evaluation, which are required for efficient centimetre-level ranging protocols using Time-of-Flight.
///
/// DecaDuino comes with several Arduino examples implementing the most popular ranging protocols such as <i>Two-Way Ranging</i> (TWR) and <i>Symetrical Double-Sided Two-Way Ranging</i> (SDS-TWR).
///
/// \image html DecaDuinoStack.png
///
/// DecaDuino has been written by Adrien van den Bossche and Réjane Dalcé at the <a href='http://www.irit.fr'>Institut de Recherche en Informatique de Toulouse</a> (IRIT), France. 
///
/// \par Download
///
/// Get the lastest version of the library <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/decaduino/download/decaduino-lastest.zip'>here</a>.
/// 
/// \par Installation
/// 
/// DecaDuino depends on the <a href='https://github.com/xxxajk/spi4teensy3'>spi4teensy3 library</a>. Download spi4teensy3 and DecaDuino, unzip the files into the libraries sub-directory and relaunch the Arduino environment; you should see the library in the Sketch->Import Library menu, and example codes in File->Examples->DecaDuino.
///
/// \par Usage
/// 
/// Remember to import the Decaduino library in your sketches:
/// \code
/// #include <spi4teensy3.h>
/// #include <DecaDuino.h>
/// \endcode
/// For more details, please checkout the examples in the File->Examples->DecaDuino menu in the Arduino IDE.
///
/// \par Communication and users forum
///
/// \par Demonstrations 
///
/// In <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/DecaWiNo/20150914-DecaWiNo-SDS-TWR-RGB-strip-low_res.mp4'>this video</a>, a fixed node running DecaDuino executes a ranging session every 100ms with another node, using the TWR protocol. Once the distance to the other node is estimated, the fixed node represents the distance by driving an RGB LED strip: the LED corresponding to the estimated distance is powered up in blue. Note that the strip used in the video is 1m-long and the leds are spaced by 1.65cm. Using a LED strip gives a direct and real-time feedback of the ranging precision and accuracy using DecaDuino.
/// \image html TWR_led_strip.jpg
/// 
/// \par Revision History
/// 
/// \version 1.0 (15/03/2016) Initial release
/// 
/// \par Academic Publications
///
/// - Adrien Van den Bossche, Rejane Dalce, Nezo Ibrahim Fofana, Thierry Val, <i>DecaDuino: An Open Framework for Wireless Time-of-Flight Ranging Systems</i>, IFIP Wireless Days (WD 2016) conference, Toulouse, 23/03/2016-25/03/2016
/// 
/// \par Licence
///
/// DecaDuino's use is subject to licensing, GPL_V3 (http://www.gnu.org/copyleft/gpl.html) or Commercial. Please contact Adrien van den Bossche <vandenbo@irit.fr> for Commercial Licensing.
///
/// \page Hardware
/// 
/// \par Supported Hardware
///
/// DecaDuino supports PJRC Teensy 3.2/3.1 MCU and DecaWave DM1000 chip and DWM1000 module. 
/// 
/// \par Wiring
/// 
/// A wiring example between Teensy 3.2 and DWM1000 module is given here.
/// 
/// \image html Wiring.png
/// 
/// Notes:
/// - In the reception state, the DWM1000 consumes 110mA+ which is more than the Teensy 3.1 can provide. You may add a DC-DC converter on the board to supply VDD 3.3V to the DWM1000. The Teensy 3.2 solves this issue as it embeds an DC-DC converter that can provide 250mA+ on the 3.3V pin. 
/// - On the Teensy 3.2/3.1, the default SPI clock pin (SCK) is the pin 13, which the same pin than the onboard LED. We recommend using an alternative SPI clock pin (SCK_, pin 14) on the Teensy. This configuration can be achieved using the following instruction:
/// \code
/// SPI.setSCK(14);
/// \endcode
/// 
/// \par Hardware examples
///
/// - DecaWiNo: <i>Deca-Wireless Node</i>. The DecaWiNo is the first DecaDuino-compliant hardware built in our facility (IRIT). It includes a PJRC Teensy 3.1, a DecaWave DWM1000 module, a MCP1825 3.3V DC-DC converter and a 5mm RGB LED.
///
/// \image html DecaWiNo.jpg

#ifndef DecaDuino_h
#define DecaDuino_h

#include "Arduino.h"
#include <spi4teensy3.h>

//#define DECADUINO_DEBUG

#define DW1000_IRQ0_PIN 9
#define DW1000_IRQ1_PIN 0
#define DW1000_IRQ2_PIN 1
#define DW1000_CS0_PIN 10
#define DW1000_CS1_PIN 10 ///@todo Check Teensy3.1 other SlaveSelect pins
#define DW1000_CS2_PIN 10 ///@todo Check Teensy3.1 other SlaveSelect pins
#define MAX_NB_DW1000_FOR_INTERRUPTS 32
#define DEBUG_STR_LEN 256

#define RANGING_ERROR 0x00

#define TIME_UNIT 1/(499.2*128*1000000)

//#define DWM1000_DEFAULT_ANTENNA_DELAY_VALUE 0x7CCD 
//#define DWM1000_DEFAULT_ANTENNA_DELAY_VALUE 33000
#define DWM1000_DEFAULT_ANTENNA_DELAY_VALUE 32870 //@brief Calibration value for DWM1000 on IRIT's DecaWiNo, by Adrien van den Bossche <vandenbo@univ-tlse2.fr>

#define DW1000_TRX_STATUS_IDLE 0
#define DW1000_TRX_STATUS_TX 1
#define DW1000_TRX_STATUS_RX 2
#define DW1000_TRX_STATUS_SLEEP 3


// DW1000 register map

#define DW1000_REGISTER_DEV_ID 				0x00

#define DW1000_REGISTER_EUI 				0x01

#define DW1000_REGISTER_PANADR				0x03
#define DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET	0x00
#define DW1000_REGISTER_PANADR_PANID_OFFSET 		0x02

#define DW1000_REGISTER_SYS_CFG				0x04
#define DW1000_REGISTER_SYS_CFG_RXAUTR_MASK 		0x20000000

#define DW1000_REGISTER_SYS_TIME			0x06

#define DW1000_REGISTER_TX_FCTRL			0x08
#define DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK	0x000003FF

#define DW1000_REGISTER_TX_BUFFER			0x09

#define DW1000_REGISTER_DX_TIME				0x0A

#define DW1000_REGISTER_SYS_CTRL			0x0D
#define DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK		0x00000002
#define DW1000_REGISTER_SYS_CTRL_TXDLYS_MASK		0x00000004
#define DW1000_REGISTER_SYS_CTRL_TRXOFF_MASK		0x00000040
#define DW1000_REGISTER_SYS_CTRL_RXENAB_MASK		0x00000100

#define DW1000_REGISTER_SYS_MASK			0x0E
#define DW1000_REGISTER_SYS_MASK_MTXFRS_MASK		0x00000080
#define DW1000_REGISTER_SYS_MASK_MRXDFR_MASK		0x00002000
#define DW1000_REGISTER_SYS_MASK_MRXFCG_MASK		0x00004000

#define DW1000_REGISTER_SYS_STATUS			0x0F
#define DW1000_REGISTER_SYS_STATUS_IRQS_MASK		0x00000001
#define DW1000_REGISTER_SYS_STATUS_TXFRS_MASK		0x00000080
#define DW1000_REGISTER_SYS_STATUS_LDEDONE_MASK 	0x00000400
#define DW1000_REGISTER_SYS_STATUS_RXDFR_MASK		0x00002000
#define DW1000_REGISTER_SYS_STATUS_RXFCG_MASK		0x00004000
#define DW1000_REGISTER_SYS_STATUS_RXFCE_MASK		0x00008000

#define DW1000_REGISTER_RX_FINFO			0x10
#define DW1000_REGISTER_RX_FINFO_RXFLEN_MASK		0x000003FF

#define DW1000_REGISTER_RX_BUFFER			0x11

#define DW1000_REGISTER_RX_TTCKI			0x13

#define DW1000_REGISTER_RX_TTCKO			0x14

#define DW1000_REGISTER_RX_TIME				0x15

#define DW1000_REGISTER_TX_TIME				0x17

#define DW1000_REGISTER_TX_ANTD				0x18

#define DW1000_REGISTER_CHANNEL_CONTROL			0x1F
#define DW1000_REGISTER_CHANNEL_CONTROL_TX_CHANNEL_MASK	0x0000000F
#define DW1000_REGISTER_CHANNEL_CONTROL_RX_CHANNEL_MASK	0x000000F0

#define DW1000_REGISTER_AON_CTRL			0x2C
#define DW1000_REGISTER_OFFSET_AON_CTRL			0x02
#define DW1000_REGISTER_AON_CTRL_UPL_CFG_MASK		0x04

#define DW1000_REGISTER_AON_CFG0			0x2C
#define DW1000_REGISTER_OFFSET_AON_CFG0			0x06
#define DW1000_REGISTER_AON_CFG0_SLEEP_EN_MASK		0x01
#define DW1000_REGISTER_AON_CFG0_WAKE_PIN_MASK		0x02
#define DW1000_REGISTER_AON_CFG0_WAKE_SPI_MASK		0x04
#define DW1000_REGISTER_AON_CFG0_WAKE_CNT_MASK		0x08
#define DW1000_REGISTER_AON_CFG0_LPDIV_EN_MASK		0x10

#define DW1000_REGISTER_PMSC_CTRL0			0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL0		0x00

#define DW1000_REGISTER_PMSC_CTRL1			0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL1		0x04


class DecaDuino {

	public:
		/**
		* @brief DecaDuino Constructor
		* @param slaveSelectPin The slaveSelect pin number
		* @param interruptPin The interrupt pin number
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20140701
		*/
		DecaDuino(uint8_t slaveSelectPin = DW1000_CS0_PIN, uint8_t interruptPin = DW1000_IRQ0_PIN);

		/**
		* @brief Initializes DecaDuino and DWM1000 without addressing fields filtering (Promiscuous mode)
		* @return true if both DecaDuino and DWM1000 have been successfully initialized
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20140701
		*/
		boolean init();

		/**
		* @brief Initializes DecaDuino and DWM1000 with given Short Address and Pan Id
		* @param shortAddrAndPanId The 16-bit short address and 16-bit Pan Id as a 32-bit integer where short address in on the LSB.
		* @return true if both DecaDuino and DWM1000 have been successfully initialized
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20150905
		*/
		boolean init(uint32_t shortAddrAndPanId);

		/**
		* @brief Reset the DW1000 chip
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void resetDW1000();

		/**
		* @brief Reads len bytes on SPI at given address, and store data in buf
		* @param address The source address
                * @param buf The address of the buffer
                * @param len The message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void readSpi(uint8_t address, uint8_t* buf, uint16_t len);

		/**
		* @brief Reads len bytes on SPI at given address/subaddress, and store data in buf
		* @param address The source address
		* @param subAddress The source subAddress
                * @param buf The address of the buffer
                * @param len The message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);

		/**
		* @brief Reads a 4-byte word on SPI at given address
		* @param address The source address
		* @return The 4 bytes
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint32_t readSpiUint32(uint8_t address);

		/**
		* @brief Writes len bytes on SPI at given address from buf
		* @param address The destination address
                * @param buf The address of the buffer
                * @param len The message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void writeSpi(uint8_t address, uint8_t* buf, uint16_t len);

		/**
		* @brief Writes len bytes on SPI at given address/subaddress from buf
		* @param address The destination address
		* @param address The destination sub-address
                * @param buf The address of the buffer
                * @param len The message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);

		/**
		* @brief Writes a 4-byte word on SPI at given address
		* @param address The destination address
		* @param ui32t The 4-byte word to write on SPI
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void writeSpiUint32(uint8_t address, uint32_t ui32t);

		/**
		* @brief Stores the System Time Counter value in the variable referenced by the pointer passed as an input parameter
		* @param p The address of the uint64_t variable
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void getSystemTimeCounter(uint64_t *p);

		/**
		* @brief Returns the System Time Counter value
		* @return The System Time Counter value as a uint64_t
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint64_t getSystemTimeCounter(void);

		/**
		* @brief Gets the PanId (Personnal Area Network Identifier) stored in the DW1000's RAM
		* @return The PanId as an uint16_t value
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint16_t getPanId();

		/**
		* @brief Gets the ShortAddress (16-bit network address, aka IEEE short address) stored in the DW1000's RAM
		* @return The Short Address as an uint16_t value
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint16_t getShortAddress();

		/**
		* @brief Gets the Euid (Extended Unique IDentifier) stored in the DW1000's ROM
		* @return The Identifier as an uint64_t value
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint64_t getEuid();

		/**
		* @brief Sets the PanId (Personnal Area Network Identifier) in the DW1000's RAM
		* @param panId The 16-bit PANID (PAN Identifier)
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void setPanId(uint16_t panId);

		/**
		* @brief Sets the ShortAddress (16-bit network address, aka IEEE short address) in the DW1000's RAM
		* @param shortAddress The 16-bit short address
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void setShortAddress(uint16_t shortAddress);

		/**
		* @brief Sets both the ShortAddress and the PanId in the DW1000's RAM
		* @param shortAddress The 16-bit short address
		* @param panId The 16-bit PANID (PAN Identifier)
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void setShortAddressAndPanId(uint16_t shortAddress, uint16_t panId); 

		/**
		* @brief Sets both the ShortAddress and the PanId in the DW1000's RAM
		* @param shortAddressPanId The 16-bit short address and 16-bit Pan Id as a 32-bit integer where short address in on the LSB.
		* @return true if success, false otherwise
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		int setShortAddressAndPanId(uint32_t shortAddressPanId);

		/**
		* @brief Returns the currently configured radio channels configured
		* @return A byte which MSB is the X channel and the LSB is the X channel
		* @author Réjane Dalce
		* @date 20160109
		*/
 		uint8_t getChannel(void);
 
 		/**
		* @brief Sets the radio channels for TX and RX
		* @param channel The channel number to set. Valid values are: 1, 2, 3, 4, 5, 7. 
		* @return Indicates whether configuration went well or not
		* @author Réjane Dalce
		* @date 20160109
		*/
 		bool setChannel(uint8_t channel);

                /**
                * @brief Returns the preamble length
                * @return A byte representing the preamble length
                * @author François Despaux
                * @date 20160217
                */
		int getPreambleLength(void);

                /**
                * @brief Sets the preamble length
		* @param plength The preamble length to set. Valid values are: 64, 128, 256, 512, 1024, 1536, 2048, 4096.
                * @return Indicates whether configuration went well or not
                * @author François Despaux
                * @date 20160217
                */
		bool setPreambleLength(int plength);

		/**
		* @brief Returns an aligned timestamp to use with pdDataRequest() in case of delayed transmissions
		* @param wantedDelay The required delay to align the delayed transmission
		* @return the aligned timestamp
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20151028
		*/
		uint64_t alignDelayedTransmission ( uint64_t wantedDelay );

		/**
		* @brief Sends a len-byte frame from buf
		* @param buf The address of the buffer
                * @param len The message length
		* @return true if success, false otherwise
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t pdDataRequest(uint8_t* buf, uint16_t len);

		/**
		* @brief Sends a len-byte frame from buf with an optionnal delay
		* @param buf The address of the buffer
		* @param len The message length
		* @param delayed The delayed flag (true or false)
		* @param time The time to send, based on the DWM1000 System Time Counter at 64GHz
		* @return true if success, false otherwise
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t pdDataRequest(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time);

		/**
		* @brief Sends a len-byte frame from buf
		* @param buf The address of the buffer
		* @param len The message length
		* @return true if success, false otherwise
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t send(uint8_t* buf, uint16_t len);

		/**
		* @brief Sends a len-byte frame from buf with an optionnal delay
		* @param buf The address of the buffer
		* @param len The message length
		* @param delayed The delayed flag (true or false)
		* @param time The time to send, based on the DWM1000 System Time Counter at 64GHz
		* @return true if success, false otherwise
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t send(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time);

		/**
		* @brief Sets the RX buffer for future frame reception. Received bytes will be stored at the beginning of the buffer.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void setRxBuffer(uint8_t* buf, uint16_t *len);

		/**
		* @brief Sets the RX buffer for future frame reception. Received bytes will be stored at the end of the buffer of max size.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void setRxBuffer(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Sets transceiver mode to receive mode
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void plmeRxEnableRequest(void);

		/**
		* @brief Sets transceiver mode to receive mode. Received bytes will be stored at the end of the buffer of max size.
		* @param max The buffer size
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint16_t max);

		/**
		* @brief Sets transceiver mode to receive mode and set the RX buffer for future frame reception.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint8_t* buf, uint16_t *len);

		/**
		* @brief Sets transceiver mode to receive mode and set the RX buffer for future frame reception. Received bytes will be stored at the end of the buffer of max size.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size 
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Sets transceiver mode to idle mode.
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void plmeRxDisableRequest(void);

		/**
		* @brief Sets transceiver mode to sleep mode.
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void sleepRequest(void);

		/**
		* @brief Sets transceiver mode to deep sleep mode.
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		* @todo To be implemented
		*/
		void deepsleepRequest(void);

		/**
		* @brief Wakes the transceiver and go back to idle mode.
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		* @todo To be implemented
		*/
		void wakeRequest(void);

		/**
		* @brief Returns true of the a frame have been received.
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(void);

		/**
		* @brief Returns true of the a frame have been received, copy received bytes in buf and store message length in len. 
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(uint8_t* buf, uint16_t *len);

		/**
		* @brief Returns true of the a frame have been received, copy received bytes in buf and store message length in len. The received bytes shall be copied toward the end of the buffer of size max.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size 
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Returns true if the last transmission request as been succefully completed
		* @return true if the last transmission request has been succefully completed
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		bool hasTxSucceeded(void);

		/**
		* @brief Gets the DecaDuino transceiver status
		* @return the DecaDuino transceiver status
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t getTrxStatus(void);

		/**
		* @brief Gets the raw value from the DW1000's embedded temperature sensor
		* @return The temperature raw value
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t getTemperatureRaw(void);

		/**
		* @brief Gets the temperature value in celsius degrees from the DW1000's embedded temperature sensor
		* @return The temperature value in celsius degrees
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		* @todo To be implemented
		*/
		float getTemperature(void);

		/**
		* @brief Gets the raw value from the DW1000's embedded voltage sensor
		* @return The voltage raw value
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		uint8_t getVoltageRaw(void);

		/**
		* @brief Gets the voltage value in volts from the DW1000's embedded voltage sensor
		* @return The voltage value in volts
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		* @todo To be implemented
		*/
		float getVoltage(void);

		/**
		* @brief Builds an uint16 value from two uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint16_t
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20111123
		*/
		uint16_t decodeUint16 ( uint8_t *data );

		/**
		* @brief Formats an uint16 value as a list of uint8 values
		* @param from The uint16_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20111011
		*/
		void encodeUint16 ( uint16_t from, uint8_t *to );

		/**
		* @brief Builds an uint32 value from four uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint32_t
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20111123
		*/
		uint32_t decodeUint32 ( uint8_t *data );

		/**
		* @brief Formats an uint32 value as a list of uint8 values
		* @param from The uint32_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20111011
		*/
		void encodeUint32 ( uint32_t from, uint8_t *to );

		/**
		* @brief Builds an uint64 value from eight uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint64_t
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20140804
		*/
		uint64_t decodeUint64 ( uint8_t *data );

		/**
		* @brief Formats an uint64 value as a list of uint8 values
		* @param from The uint64_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20111011
		*/
		void encodeUint64 ( uint64_t from, uint8_t *to );

		/**
		* @brief Prints an uint64_t value on console
		* @param ui64 The uint64_t value
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void printUint64 ( uint64_t ui64 );

		/**
		* @brief Returns last transmitted frame timestamp based on the DWM1000 System Time Counter at 64GHz
		* @return Last transmitted frame timestamp
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20140905
		*/
		uint64_t getLastTxTimestamp();

		/**
		* @brief Returns last received frame timestamp based on the DWM1000 System Time Counter at 64GHz
		* @return Last received frame timestamp
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20140905
		*/
		uint64_t getLastRxTimestamp();

	       /**
		* @brief Returns last received frame's clock skew, also designated as clock offset in the Decawave documentation
		* @return Last received frame's clock skew
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20150905
		*/
		double getLastRxSkew();

		uint64_t euid;
		uint8_t *rxData;
		uint16_t *rxDataLen;
		uint16_t rxDataLenMax;
		uint8_t rxDataAvailable;
		uint8_t trxStatus;
		bool lastTxOK;
		uint64_t lastTxTimestamp;
		uint64_t lastRxTimestamp;
		double clkOffset;

	private:
		uint8_t debugStr[DEBUG_STR_LEN];
	
	protected:

		/**
		* @brief The first interrupt function
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		static void isr0();

		/**
		* @brief The second interrupt function
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		static void isr1();

		/**
		* @brief The third interrupt function
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		static void isr2();

		/**
		* @brief The global interrupt function
		* @return No return
		* @author Adrien van den Bossche <bossche@irit.fr>
		* @date 20141115
		*/
		void handleInterrupt();

		uint8_t _slaveSelectPin;
		uint8_t _interruptPin;
		static DecaDuino* _DecaDuinoInterrupt[];
};

#endif
