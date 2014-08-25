// DecaDuino.h
//
// DecaWave DW1000 driver for Arduino
// See the README file in this directory fdor documentation
// 
/// \mainpage DecaDuino library for Arduino
///
/// This is the main page
///
/// \par Supported Hardware
///
/// \par Installation
/// To install, unzip the library into the libraries sub-directory of your
/// Arduino application directory. Then launch the Arduino environment; you
/// should see the library in the Sketch->Import Library menu, and example
/// code in
///
/// \par Revision History
/// \version 1.0 Original release
/// 
/// To use the DecaDuino library, you must have
/// \code
/// #include <DecaDuino.h>
/// \endcode
/// At the top of your sketch.
/// 

#ifndef DecaDuino_h
#define DecaDuino_h

#include "Arduino.h"
#include <spi4teensy3.h>

#define DECADUINO_DEBUG

#define DW1000_IRQ0_PIN 9
#define DW1000_IRQ1_PIN 0
#define DW1000_IRQ2_PIN 1
#define DW1000_CS0_PIN 10
#define DW1000_CS1_PIN 10 // ToDo: check Teensy3.1 other SlaveSelect pins
#define DW1000_CS2_PIN 10 // ToDo: check Teensy3.1 other SlaveSelect pins
#define MAX_NB_DW1000_FOR_INTERRUPTS 32
#define BUFFER_MAX_LEN 1024

#define DW1000_REGISTER_DEV_ID 		0x00
#define DW1000_REGISTER_EUI 		0x01
#define DW1000_REGISTER_PANADR		0x03
#define DW1000_REGISTER_PANADR_SHORT_ADDRESS_OFFSET 0x00
#define DW1000_REGISTER_PANADR_PANID_OFFSET 0x02
#define DW1000_REGISTER_SYS_CFG		0x04
#define DW1000_REGISTER_SYS_CFG_RXAUTR_MASK 0x20000000
#define DW1000_REGISTER_TX_FCTRL	0x08
#define DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK 0x000003FF
#define DW1000_REGISTER_TX_BUFFER	0x09
#define DW1000_REGISTER_SYS_CTRL	0x0D
#define DW1000_REGISTER_SYS_CTRL_TXSTRT_MASK 0x00000002
#define DW1000_REGISTER_SYS_CTRL_TRXOFF_MASK 0x00000040
#define DW1000_REGISTER_SYS_CTRL_RXENAB_MASK 0x00000100
#define DW1000_REGISTER_SYS_MASK	0x0E
#define DW1000_REGISTER_SYS_MASK_MRXDFR_MASK 0x00002000
#define DW1000_REGISTER_SYS_MASK_MRXFCG_MASK 0x00004000
#define DW1000_REGISTER_SYS_STATUS	0x0F
#define DW1000_REGISTER_SYS_STATUS_RXDFR_MASK 0x00002000
#define DW1000_REGISTER_SYS_STATUS_RXFCG_MASK 0x00004000
#define DW1000_REGISTER_SYS_STATUS_RXFCE_MASK 0x00008000
#define DW1000_REGISTER_RX_FINFO	0x10
#define DW1000_REGISTER_RX_FINFO_RXFLEN_MASK 0x000003FF
#define DW1000_REGISTER_RX_BUFFER	0x11
#define DW1000_REGISTER_PMSC_CTRL0	0x36
#define DW1000_REGISTER_


class DecaDuino {

  public:
    DecaDuino(uint8_t slaveSelectPin = DW1000_CS0_PIN, uint8_t interruptPin = DW1000_IRQ0_PIN);
    boolean init();
    void resetDW1000();
    void dummy();
    void readSpi(uint8_t address, uint8_t* buf, uint16_t len);
    void readSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len);
    uint32_t readSpiUint32(uint8_t address);
    void writeSpi(uint8_t address, uint8_t* buf, uint16_t len);
    void writeSpiSubAddress(uint8_t address, uint8_t subAddress, uint8_t* buf, uint16_t len);
    void writeSpiUint32(uint8_t address, uint32_t ui32t);
    uint16_t getPanId();
    uint16_t getShortAddress();
    uint64_t getEuid();
    void setPanId(uint16_t panId);
    void setShortAddress(uint16_t shortAddress);
    void plmeRxEnableRequest(void);
    void plmeRxDisableRequest(void);
    void plmeDataRequest(uint8_t* buf, uint16_t len);

  private:
    /*
    * Return a UINT16 based on two UINT8
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20111123
    */
    uint16_t decodeUint16 ( uint8_t *data );

    /*
    * Place data from at to address
    * No return
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20111011
    */
    void encodeUint16 ( uint16_t from, uint8_t *to );

    /*
    * Return a UINT32 based on four UINT8
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20111123
    */
    uint32_t decodeUint32 ( uint8_t *data );

    /*
    * Place data from at to address
    * No return
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20111011
    */
    void encodeUint32 ( uint32_t from, uint8_t *to );

    /*
    * Return a UINT64 based on eight UINT8
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20140804
    */
    uint64_t decodeUint64 ( uint8_t *data );

    /*
    * Place data from at to address
    * No return
    * @author Adrien van den Bossche <bossche@irit.fr>
    * @date 20111011
    */
    void encodeUint64 ( uint64_t from, uint8_t *to );

    uint8_t buf[BUFFER_MAX_LEN];
    uint64_t euid;
#ifdef DECADUINO_DEBUG
    uint8_t debugStr[BUFFER_MAX_LEN];
#endif

  protected:
    static void isr0();
    static void isr1();
    static void isr2();
    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;

    static DecaDuino* _DecaDuinoInterrupt[];
    void handleInterrupt();
};


#endif
