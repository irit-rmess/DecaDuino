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

//#define DECADUINO_DEBUG

#define DW1000_IRQ0_PIN 9
#define DW1000_IRQ1_PIN 0
#define DW1000_IRQ2_PIN 1
#define DW1000_CS0_PIN 10
#define DW1000_CS1_PIN 10 // ToDo: check Teensy3.1 other SlaveSelect pins
#define DW1000_CS2_PIN 10 // ToDo: check Teensy3.1 other SlaveSelect pins
#define MAX_NB_DW1000_FOR_INTERRUPTS 32
#define DEBUG_STR_LEN 256

#define RANGING_ERROR 0x00

#define TIME_UNIT 1/(499.2*128*1000000)

#define DW1000_TRX_STATUS_IDLE 0
#define DW1000_TRX_STATUS_TX 1
#define DW1000_TRX_STATUS_RX 2
#define DW1000_TRX_STATUS_SLEEP 3

#define RANGING_PROTOCOL_TWR 1
#define RANGING_PROTOCOL_SDS_TWR 2
#define DEFAULT_RANGING_PROTOCOL RANGING_PROTOCOL_TWR

#define MSG_TYPE_SDSTWR_EMPTY 0
#define MSG_TYPE_SDSTWR_START 1
#define MSG_TYPE_SDSTWR_ACKREQ 2
#define MSG_TYPE_SDSTWR_ACK 3
#define MSG_TYPE_SDSTWR_DATA_REPLY 4

#define RX_RANGING_INIT_STATE 1
#define RX_RANGING_WAITING_FOR_START_STATE 2
#define RX_RANGING_SENDING_ACKREQ_STATE 3
#define RX_RANGING_WAITING_FOR_ACK_STATE 4
#define RX_RANGING_SENDING_DATA_REPLY_STATE 5

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
#define DW1000_REGISTER_SYS_MASK_MTXFRS_MASK 0x00000080
#define DW1000_REGISTER_SYS_MASK_MRXDFR_MASK 0x00002000
#define DW1000_REGISTER_SYS_MASK_MRXFCG_MASK 0x00004000

#define DW1000_REGISTER_SYS_STATUS	0x0F
#define DW1000_REGISTER_SYS_STATUS_IRQS_MASK  0x00000001
#define DW1000_REGISTER_SYS_STATUS_TXFRS_MASK 0x00000080
#define DW1000_REGISTER_SYS_STATUS_LDEDONE_MASK 0x00000400
#define DW1000_REGISTER_SYS_STATUS_RXDFR_MASK 0x00002000
#define DW1000_REGISTER_SYS_STATUS_RXFCG_MASK 0x00004000
#define DW1000_REGISTER_SYS_STATUS_RXFCE_MASK 0x00008000

#define DW1000_REGISTER_RX_FINFO	0x10
#define DW1000_REGISTER_RX_FINFO_RXFLEN_MASK 0x000003FF

#define DW1000_REGISTER_RX_BUFFER	0x11

#define DW1000_REGISTER_AON_CTRL	0x2C
#define DW1000_REGISTER_OFFSET_AON_CTRL	0x02
#define DW1000_REGISTER_AON_CTRL_UPL_CFG_MASK 0x04

#define DW1000_REGISTER_AON_CFG0	0x2C
#define DW1000_REGISTER_OFFSET_AON_CFG0	0x06
#define DW1000_REGISTER_AON_CFG0_SLEEP_EN_MASK 0x01
#define DW1000_REGISTER_AON_CFG0_WAKE_PIN_MASK 0x02
#define DW1000_REGISTER_AON_CFG0_WAKE_SPI_MASK 0x04
#define DW1000_REGISTER_AON_CFG0_WAKE_CNT_MASK 0x08
#define DW1000_REGISTER_AON_CFG0_LPDIV_EN_MASK 0x10

#define DW1000_REGISTER_PMSC_CTRL0	0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL0 0x00

#define DW1000_REGISTER_PMSC_CTRL1	0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL1 0x04

#define DW1000_REGISTER_


class DecaDuino {

  public:
    DecaDuino(uint8_t slaveSelectPin = DW1000_CS0_PIN, uint8_t interruptPin = DW1000_IRQ0_PIN);
    boolean init(uint32_t shorAddrPANID);
    boolean init();
    void resetDW1000();
    void dummy();
    void readSpi(uint8_t address, uint8_t* buf, uint16_t len);
    void readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);
    uint32_t readSpiUint32(uint8_t address);
    void writeSpi(uint8_t address, uint8_t* buf, uint16_t len);
    void writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);
    void writeSpiUint32(uint8_t address, uint32_t ui32t);
    uint16_t getPanId();
    uint16_t getShortAddress();
    uint64_t getEuid();
    void setPanId(uint16_t panId);
    void setShortAddress(uint16_t shortAddress);
    uint8_t plmeDataRequest(uint8_t* buf, uint16_t len);
    uint8_t send(uint8_t* buf, uint16_t len);
    void setRxBuffer(uint8_t* buf, uint16_t *len);
    float rangeNode(uint64_t destination, uint8_t protocol);
    uint8_t twrRequest(uint64_t destination);
    uint8_t sdsTwrRequest(uint64_t destination);
    void rangingEngine(void);
    void plmeRxEnableRequest(void);
    void plmeRxEnableRequest(uint8_t* buf, uint16_t *len);
    void plmeRxDisableRequest(void);
    void sleepRequest(void);
    void deepsleepRequest(void);
    void wakeRequest(void);
    uint8_t rxFrameAvailable(void);
    uint8_t rxFrameAvailable(uint8_t* buf, uint16_t *len);
    void test(void);
    bool lastTxOK;
    bool hasTxSucceeded(void);
    uint8_t getTrxStatus(void);

    uint64_t lastTxTimestamp;
    uint64_t lastRxTimestamp;
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
    void printUint64 ( uint64_t ui64 );

    //uint8_t buf[BUFFER_MAX_LEN];
    uint64_t euid;
    uint8_t *rxData;
    uint16_t *rxDataLen;
    uint8_t rxDataAvailable;
    uint8_t trxStatus;

uint64_t predictT5();

void print(uint64_t val);
uint64_t decodeUint64_2 ( uint8_t *data ) ;

double clkOffset;

//#ifdef DECADUINO_DEBUG
    uint8_t debugStr[DEBUG_STR_LEN];
//#endif
  private:
  
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
