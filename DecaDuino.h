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
/// DecaDuino supports the PJRC Teensy 3.2/3.1/3.0. Others Arduino boards have not been tested yet. User feedback on the topic will be greatly appreciated. For this purpose, please use the contact address indicated in the "Contact, feedback and users forum" section of this documentation.
///
/// DecaDuino is a <i>Physical-layer Service Access Point (PHY-SAP)</i>. It provides the two conventional <i>Physical-Data</i> (PD) and <i>Physical Layer Management Entity</i> (PLME) SAPs which enable MAC-level protocols to send/receive data and configure the transceiver (channel, transmission rate, preamble parameters...). Since this framework was designed to aid in the implementation of Time-of-Flight based ranging protocols, DecaDuino also provides access to the DW1000's Physical-level high precision timer (64GHz/40bit) which enables precise message timestamping at both transmission (t_TX) and reception (t_RX). Finally, DecaDuino implements DW1000's advanced synchronization/timestamping functionalities such as delayed transmission and receiver skew evaluation, which are required for efficient centimetre-level ranging protocols using Time-of-Flight.
///
/// DecaDuino comes with several Arduino examples implementing the most popular ranging protocols such as <i>Two-Way Ranging</i> (TWR) and <i>Symetrical Double-Sided Two-Way Ranging</i> (SDS-TWR).
///
/// \image html DecaDuinoStack.png
///
/// DecaDuino has been written by Adrien van den Bossche and Réjane Dalcé at the <a href='http://www.irit.fr'>Institut de Recherche en Informatique de Toulouse</a> (IRIT), France. Thanks to Thierry Val, François Despaux, Laurent Guerby, Ibrahim Fofana and Robert Try for their contributions to DecaDuino.
///
/// \par Download
///
/// Get the <a href='https://github.com/irit-irt/DecaDuino'>current release of the library on github.com</a>. Previous versions (before github.com hosting) are also available in the "Revision History" section of this documentation.
/// 
/// \par Installation
/// 
/// To use DecaDuino on a PJRC Teensy 3.2/3.1/3.0, install the <a href='http://www.pjrc.com/teensy/teensyduino.html'>Teensyduino add-on</a> first. Then, download DecaDuino, unzip the files into the libraries sub-directory and relaunch the Arduino environment; you should see the library in the Sketch->Import Library menu, and example sketches in File->Examples->DecaDuino.
///
/// \par Usage
/// 
/// Remember to import the SPI and Decaduino libraries in your sketches:
/// \code
/// #include <SPI.h>
/// #include <DecaDuino.h>
/// \endcode
/// For more details, please checkout the examples in the File->Examples->DecaDuino menu in the Arduino IDE. The sketches include both frame send/receive examples and ranging protocols implementation examples.
///
/// \par Contact, feedback and users forum
/// 
/// Please contact <a href='mailto:vandenbo_nospam@irit.fr?subject=[DecaDuino] '>Adrien van den Bossche</a> (remove _nospam) for any question concerning DecaDuino. 
///
/// \par Demonstrations 
///
/// <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/DecaWiNo/20150914-DecaWiNo-SDS-TWR-RGB-strip-low_res.mp4'>In this video</a>, a fixed node running DecaDuino executes a ranging session every 100ms with another node, using the TWR protocol. Once the distance to the other node is estimated, the fixed node represents the distance by driving an RGB LED strip: the LED corresponding to the estimated distance is powered up in blue. Note that the strip used in the video is 1m-long and the leds are spaced by 1.65cm. Using a LED strip gives a direct and real-time feedback of the ranging precision and accuracy using DecaDuino.
/// \htmlonly <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/DecaWiNo/20150914-DecaWiNo-SDS-TWR-RGB-strip-low_res.mp4'> \endhtmlonly
/// \image html TWR_led_strip.jpg
/// \htmlonly </a> \endhtmlonly
/// 
/// \par Revision History
/// 
/// - <a href='https://github.com/irit-irt/DecaDuino'>Current release on github.com</a>
///
/// - <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/decaduino/download/decaduino-1.0.zip'>1.0 (19/03/2016) Initial release</a>
/// 
/// \par Academic Publications
///
/// DecaDuino has been presented in this academic publication: <a target='_blank' href='https://www.irit.fr/~Adrien.Van-Den-Bossche/papers/WD2016_AVDB_RD_IF_TV_OpenWiNo.pdf'>Adrien Van den Bossche, Rejane Dalce, Nezo Ibrahim Fofana, Thierry Val, <i>DecaDuino: An Open Framework for Wireless Time-of-Flight Ranging Systems</i></a>, IFIP Wireless Days (WD 2016) conference, Toulouse, 23/03/2016-25/03/2016.
///
/// Academic Publications that references DecaDuino <a href='https://www.irit.fr/~Adrien.Van-Den-Bossche/projets_decaduino.php'>are listed here</a>. Please contact <a href='mailto:vandenbo_nospam@irit.fr?subject=[DecaDuino] '>Adrien van den Bossche</a> (remove _nospam) if you want to add your work to this list.
/// 
/// \par Licence
///
/// DecaDuino's use is subject to licensing, GPL_V3 (http://www.gnu.org/copyleft/gpl.html) or Commercial. Please contact <a href='mailto:vandenbo_nospam@irit.fr?subject=[DecaDuino] '>Adrien van den Bossche</a> (remove _nospam) for Commercial Licensing.
///
/// \page Hardware
/// 
/// \par Supported Hardware
///
/// DecaDuino supports PJRC Teensy 3.2/3.1/3.0 MCU and DecaWave DM1000 chip and DWM1000 module. 
///
/// Please report any successfull operation on others Arduino boards by using the contact address indicated in the "Contact, feedback and users forum" section of this documentation.
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
/// - <a target='_blank' href='http://wino.cc/decawino'>DecaWiNo: <i>Deca-Wireless Node</i></a>. The <a target='_blank' href='http://wino.cc/decawino'>DecaWiNo</a> is the first DecaDuino-compliant hardware built in our facility (IRIT). It includes a PJRC Teensy 3.1, a DecaWave DWM1000 module, a MCP1825 3.3V DC-DC converter and a 5mm RGB LED.
///
/// \image html DecaWiNo.jpg

#ifndef DecaDuino_h
#define DecaDuino_h

#include "Arduino.h"

//#define DECADUINO_DEBUG

#ifdef UWB_MODULE_DWM1001
#include <SPI.h>
#define DW1000_IRQ0_PIN 22
#else
#define DW1000_IRQ0_PIN 9
#endif
#define DW1000_IRQ1_PIN 0
#define DW1000_IRQ2_PIN 1
#define DW1000_CS0_PIN 10
#define DW1000_CS1_PIN 10 ///@todo Check Teensy3.1 other SlaveSelect pins
#define DW1000_CS2_PIN 10 ///@todo Check Teensy3.1 other SlaveSelect pins
#define MAX_NB_DW1000_FOR_INTERRUPTS 32
#define DEBUG_STR_LEN 256

#define RANGING_ERROR 0x00

#define DW1000_TIMEBASE 15.65E-12
#define AIR_SPEED_OF_LIGHT 299702547.235 // @brief speed of light in the air, defined by c/1.0003
#define RANGING_UNIT AIR_SPEED_OF_LIGHT*DW1000_TIMEBASE

#ifdef UWB_MODULE_DWM1001
#define DWM1000_DEFAULT_ANTENNA_DELAY_VALUE 32889   //@brief Calibration value for DWM1001-DEV, by Quentin Vey.
                                                    // This is an average of all the calibrations done for the
                                                    // frequencies and PRF that could be done in our testbed.
                                                    // Expect a 40cm bias if you use this value.
static const uint16_t calibratedAntennaDelay[7][2] = //@brief Calibration value for DWM1001-DEV, by Quentin Vey.
                            // These values have been measured in our testbed, at the distances recommended by DecaWave.
                            // The index are the following : antennaDelay[channel-1][ (PRF in MHz)>>6 ]
{
    {32959, 32967}, // only the value for PRF 64MHz has been calibrated, the same value is used for 16MHz due to the lack of a better value
    {32962, 32973}, // only the value for PRF 64MHz has been calibrated, the same value is used for 16MHz due to the lack of a better value
    {32934, 32929}, // only the value for PRF 64MHz has been calibrated, the same value is used for 16MHz due to the lack of a better value
    {32843, 32852}, // both values have been calibrated
    {32905, 32938}, // both values have been calibrated
    {    0,     0}, // channel 6 not implemented by DW1000
    {32823, 32833}, // both values have been calibrated
};
#else
#define DWM1000_DEFAULT_ANTENNA_DELAY_VALUE 32847 //@brief Calibration value for DWM1000 on IRIT's DecaWiNo, by Adrien van den Bossche <vandenbo at univ-tlse2.fr>
static const uint16_t calibratedAntennaDelay[7][2] =
{
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
    {DWM1000_DEFAULT_ANTENNA_DELAY_VALUE, DWM1000_DEFAULT_ANTENNA_DELAY_VALUE},
};
#endif


static const int TX_POWER_CHANNEL[] = {
        -1,
        0,
        0,
        1,
        2,
        3,
        -1,
        4
};


static const uint32_t smartTxPowerConf[5][2] {      //@brief recommended values for register 0x1E when smartTxPower is used with a 0dBi antenna.
                                                    // First index is TX_POWER_CHANNEL[channel], second index is  (PRF in MHz)>>6
    {0x15355575, 0x07274767},   // channels 1 & 2
    {0x0F2F4F6F, 0x2B4B6B8B},   // channel 3
    {0x1F1F3F5F, 0x3A5A7A9A},   // channel 4
    {0x0E082848, 0x25456585},   // channel 5
    {0x32527292, 0x5171B1D1}    // channel 7
};

static const uint32_t recommendedManualTxPowerConf[5][2] {    //@brief recommended values for register 0x1E when "manual" tx power is used with a 0dBi antenna.
                                                        // First index is TX_POWER_CHANNEL[channel], second index is  (PRF in MHz)>>6
    {0x75757575, 0x67676767},   // channels 1 & 2
    {0x6F6F6F6F, 0x8B8B8B8B},   // channel 3
    {0x5F5F5F5F, 0x9A9A9A9A},   // channel 4
    {0x48484848, 0x85858585},   // channel 5
    {0x92929292, 0xD1D1D1D1}    // channel 7
};

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
#define DW1000_REGISTER_SYS_CFG_PHR_MODE_MASK 		0x00030000
#define DW1000_REGISTER_SYS_CFG_PHR_MODE_SHIFT 		16
#define DW1000_REGISTER_SYS_CFG_RXM110K_MASK        0x00400000
#define DW1000_REGISTER_SYS_CFG_RXM110K_SHIFT       22
#define DW1000_REGISTER_SYS_CFG_DIS_SXTP_MASK       0x0040000
#define DW1000_REGISTER_SYS_CFG_DIS_SXTP_SHIFT      18

#define DW1000_REGISTER_SYS_TIME			0x06

#define DW1000_REGISTER_TX_FCTRL			0x08
#define DW1000_REGISTER_TX_FCTRL_FRAME_LENGTH_MASK	0x000003FF
#define DW1000_REGISTER_TX_FCTRL_TX_PRF_MASK        0x00030000
#define DW1000_REGISTER_TX_FCTRL_TX_PRF_SHIFT       16

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
#define DW1000_REGISTER_RX_FINFO_RXFLEN_SHIFT       0
#define DW1000_REGISTER_RX_FINFO_RXFNSPL_MASK       0x00000C00
#define DW1000_REGISTER_RX_FINFO_RXFNSPL_SHIFT      11
#define DW1000_REGISTER_RX_FINFO_RXBR_MASK          0x00003000
#define DW1000_REGISTER_RX_FINFO_RXBR_SHIFT         13
#define DW1000_REGISTER_RX_FINFO_RNG_MASK           0x00008000
#define DW1000_REGISTER_RX_FINFO_RNG_SHIFT          15
#define DW1000_REGISTER_RX_FINFO_RXPRFR_MASK        0x00030000
#define DW1000_REGISTER_RX_FINFO_RXPRFR_SHIFT       16
#define DW1000_REGISTER_RX_FINFO_RXPSR_MASK         0x000C0000
#define DW1000_REGISTER_RX_FINFO_RXPSR_SHIFT        18
#define DW1000_REGISTER_RX_FINFO_RXPACC_MASK		0xFFF00000
#define DW1000_REGISTER_RX_FINFO_RXPACC_SHIFT       20

#define DW1000_REGISTER_RX_BUFFER			0x11

#define DW1000_REGISTER_RX_RFQUAL			    0x12
#define DW1000_REGISTER_RX_RFQUAL_FPAMPL2_MASK  0xFFFF0000
#define DW1000_REGISTER_RX_RFQUAL_CIRE_MASK     0x0000FFFF

#define DW1000_REGISTER_RX_TTCKI			0x13

#define DW1000_REGISTER_RX_TTCKO			0x14

#define DW1000_REGISTER_RX_TIME				0x15

#define DW1000_REGISTER_TX_TIME				0x17

#define DW1000_REGISTER_TX_ANTD				0x18

#define DW1000_REGISTER_TX_POWER            0x1E

#define DW1000_REGISTER_CHAN_CTRL			0x1F
#define DW1000_REGISTER_CHAN_CTRL_TX_CHAN_MASK		0x0000000F
#define DW1000_REGISTER_CHAN_CTRL_TX_CHAN_SHIFT     0
#define DW1000_REGISTER_CHAN_CTRL_RX_CHAN_MASK		0x000000F0
#define DW1000_REGISTER_CHAN_CTRL_RX_CHAN_SHIFT     4
#define DW1000_REGISTER_CHAN_CTRL_DWSFD_MASK        0x00020000
#define DW1000_REGISTER_CHAN_CTRL_DWSFD_SHIFT       17
#define DW1000_REGISTER_CHAN_CTRL_RXPRF_MASK		0x000C0000
#define DW1000_REGISTER_CHAN_CTRL_RXPRF_SHIFT       18
#define DW1000_REGISTER_CHAN_CTRL_TNSSFD_MASK       0x00100000
#define DW1000_REGISTER_CHAN_CTRL_TNSSFD_SHIFT      20
#define DW1000_REGISTER_CHAN_CTRL_RNSSFD_MASK       0x00200000
#define DW1000_REGISTER_CHAN_CTRL_RNSSFD_SHIFT      21
#define DW1000_REGISTER_CHAN_CTRL_TX_PCODE_MASK		0x07C00000
#define DW1000_REGISTER_CHAN_CTRL_TX_PCODE_SHIFT    22
#define DW1000_REGISTER_CHAN_CTRL_RX_PCODE_MASK		0xF8000000
#define DW1000_REGISTER_CHAN_CTRL_RX_PCODE_SHIFT    27

#define DW1000_REGISTER_USR_SFD                     0x21
#define DW1000_REGISTER_USR_SFD_LENGTH_OFFSET       0x00

#define DW1000_REGISTER_DRX_CONF                    0x27
#define DW1000_REGISTER_OFFSET_DRX_TUNE1A           0x04
#define DW1000_REGISTER_OFFSET_DRX_TUNE1B           0x06
#define DW1000_REGISTER_OFFSET_DRX_TUNE2            0x08
#define DW1000_REGISTER_OFFSET_DRX_SFDTOC           0x20
#define DW1000_REGISTER_OFFSET_RXPACC_NOSAT         0x2C

#define DW1000_REGISTER_AGC_CTRL                            0x23
#define DW1000_REGISTER_OFFSET_AGC_TUNE1                    0x04
#define DW1000_REGISTER_OFFSET_AGC_TUNE2                    0x0C

#define DW1000_REGISTER_TX_CAL                  0x2A
#define DW1000_REGISTER_OFFSET_TC_PGDELAY       0x0B


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



#define DW1000_REGISTER_LDE_INTERFACE                       0x2E
#define DW1000_REGISTER_LDE_INTERFACE_LDE_THRESH_OFFSET     0x00
#define DW1000_REGISTER_LDE_INTERFACE_LDE_CFG1_OFFSET       0x0806
#define DW1000_REGISTER_LDE_INTERFACE_NTM_MASK              0x1F
#define DW1000_REGISTER_LDE_INTERFACE_NTM_SHIFT             0
#define DW1000_REGISTER_LDE_INTERFACE_PMULT_MASK            0xE0
#define DW1000_REGISTER_LDE_INTERFACE_PMULT_SHIFT           5
#define DW1000_REGISTER_LDE_INTERFACE_LDE_PPINDX_OFFSET     0x1000
#define DW1000_REGISTER_LDE_INTERFACE_LDE_PPAMPL_OFFSET     0x1002
#define DW1000_REGISTER_LDE_INTERFACE_LDE_RXANTD_OFFSET     0x1804
#define DW1000_REGISTER_LDE_INTERFACE_LDE_CFG2_OFFSET       0x1806
#define DW1000_REGISTER_LDE_INTERFACE_LDE_REPC_OFFSET       0x2804

#define DW1000_REGISTER_PMSC_CTRL0			0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL0		0x00

#define DW1000_REGISTER_PMSC_CTRL1			0x36
#define DW1000_REGISTER_OFFSET_PMSC_CTRL1		0x04

typedef enum {
    DW1000_DATARATE_110KBPS = 0,    // MUST be 0
    DW1000_DATARATE_850KBPS,
    DW1000_DATARATE_6_8MBPS
} dw1000_datarate_t;

typedef struct {
    uint16_t RXFLEN:10;         // Received Frame Length (include length extension RXFLE)
    uint8_t RXNSPL:2;           // Receive non-standard preamble length.
    uint8_t RXBR:2;             // Receive Bit Rate( 0b00 = 110 kbps, 0b01 =850 kbps, 0b10 = 6.8Mbps)
    uint8_t RNG:1;              // Receiver Ranging. This reflects the ranging bit in the received PHY header
    uint8_t RXPRFR:2;           // RX Pulse Repetition Rate report (0b01 = 16 MHz, 0b10 = 64 MHz)
    uint8_t RXPSR:2;            // RX Preamble Repetition (0b00 = 16 symbols, 0b01 = 64 symbols, 0b10= 1024 symbols, 0b11= 4096 symbols)
    uint16_t RXPACC:12;         // Preamble Accumulation Count
}RXFInfo_t;  // full content of the register 0x10 : Rx Frame Information Register

typedef struct {
    uint16_t STD_NOISE; // Standard Deviation of Noise
    uint16_t FP_AMPL2;  // First Path Amplitude point 2.
    uint16_t FP_AMPL3;  // First Path Amplitude point 3
    uint16_t CIR_PWR;// Channel Impulse Response Power
}RXFQual_t;  // full content of the register 0x12 : Rx Frame Quality Information

typedef struct {
    uint64_t RX_STAMP:40;   // timestamp of reception, 40-bits value (1/ (128*499.2×10^6 ) seconds). Valid if  LDEDONE status bit is set.
    uint16_t FP_INDEX;      // First path index
    uint16_t FP_AMPL1;      // First path Amplitude point 1
    uint64_t RX_RAWST:40;  // coarse timestamp of reception 40-bits value (1/ (128*499.2×10^6 ) seconds). Valid if RXPHD status bit is set
}RXTime_t;  // full content of the register 0x15 : Receive Time Stamp

typedef struct {
    uint8_t TX_CHAN:4;      // transmit channel
    uint8_t RX_CHAN:4;      // receive channel
    uint8_t DWSFD:1;        // non-standard Decawave proprietary SFD sequence
    uint8_t RXPRF:2;        // PRF used in the receiver
    uint8_t TNSSFD:1;       // user specified (non-standard) SFD in the transmitter
    uint8_t RNSSFD:1;       // user specified (non-standard) SFD in the receiver
    uint8_t TX_PCODE:5;     // preamble code used in the transmitter
    uint8_t RX_PCODE:5;     // preamble code used in the receiver
}channelCTRL_t;   // content of the register 0x1F : Channel control register

typedef struct {
    uint16_t LDE_THRESH;    // LDE Threshold report
    uint8_t  NTM:5;         // Noise Threshold Multiplier
    uint8_t  PMULT:3;       // Peak Multiplier
    uint16_t LDE_PPINDX;    // LDE Peak Path Index
    uint16_t LDE_PPAMPL;    // LDE Peak Path Amplitude
    uint16_t LDE_RXANTD;    // LDE Receive Antenna Delay configuration
    uint16_t LDE_CFG2;      // LDE Configuration Register 2
    uint16_t LDE_REPC;      // LDE Replica Coefficient configuration
}LDEInterface_t;    // content of the register 0x2E : Leading Edge Detection Interface

typedef struct {
    int16_t r; // real part of a sample in CIR memory
    int16_t i; // imaginary part of a sample in CIR memory
}CIRSample_t;   // single sample in CIR memory (register 0x25)

enum class COARSE_POWER_SETTING : uint8_t { // see DW1000 user manual 7.2.31.1
                                            // WARNING : order is very important
    COARSE_POWER_GAIN_18db = 0,
    COARSE_POWER_GAIN_15db,
    COARSE_POWER_GAIN_12db,
    COARSE_POWER_GAIN_9db,
    COARSE_POWER_GAIN_6db,
    COARSE_POWER_GAIN_3db,
    COARSE_POWER_GAIN_0db,
    COARSE_POWER_GAIN_OFF,
};


const char DW1000_DATARATE[][9] = {
    "100 Kbps",
    "850 Kbps",
    "6,8 Mbps"
};

class DecaDuino {

	public:
		/**
		* @brief DecaDuino Constructor
		* @param slaveSelectPin The slaveSelect pin number
		* @param interruptPin The interrupt pin number
		* @author Adrien van den Bossche
		* @date 20140701
		*/
		DecaDuino(uint8_t slaveSelectPin = DW1000_CS0_PIN, uint8_t interruptPin = DW1000_IRQ0_PIN);

		/**
		* @brief Initializes DecaDuino and DWM1000 without addressing fields filtering (Promiscuous mode)
		* @return true if both DecaDuino and DWM1000 have been successfully initialized
		* @author Adrien van den Bossche
		* @date 20140701
		*/
		boolean init();

		/**
		* @brief Initializes DecaDuino and DWM1000 with given Short Address and Pan Id
		* @param shortAddrAndPanId The 16-bit short address and 16-bit Pan Id as a 32-bit integer where short address in on the LSB.
		* @return true if both DecaDuino and DWM1000 have been successfully initialized
		* @author Adrien van den Bossche
		* @date 20150905
		*/
		boolean init(uint32_t shortAddrAndPanId);

		/**
		* @brief Reset the DW1000 chip
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void resetDW1000();

		/**
		* @brief Set PHR Mode
		* @param mode 0 for standard 127 bytes frame, 3 for extended 1023 bytes frame
		* @return No return
		* @author Laurent GUERBY
		* @date 20170329
		*/
		void setPHRMode(uint8_t mode);

		/**
		* @brief Returns the PHR Mode
		* @return PHR Mode
		* @author Laurent GUERBY
		* @date 20170329
		*/
		uint8_t getPHRMode(void);

        /**
        * @brief Converts power settings to the register value for manual power setting
        * @param coarse : sets the coarse (DA) power setting. See DW1000 user manual, 7.2.31.1.
        * @param fine : expressed in half-db. Sets the fine (mixer) power setting. See DW1000 user manual, 7.2.31.1. Allowed values : 0 (0 dB) to 31 (15.5 dB). Any value above 31 will be capped to 31.
        * @return Value to write into the register TXPOWSD or TXPOWPHR
        * @author Quentin Vey
        * @date 20190701
        */
        static uint8_t powerSettingsToRegisterValue(COARSE_POWER_SETTING coarse, uint8_t fine);

        /**
        * @brief Set power mode to smart (i.e. allows power boost for short frames).
        * @param trackChanges : whether the tx power conf should be updated when the channel or PRF are changed.
        * TX_POWER registers are set to their manufacturer default values
        * @return No return
        * @author Quentin Vey
        * @date 20200223
        */
        void setSmartTxPower(bool trackChanges = true);


        /**
        * @brief writes the appropriate value in 0x1E TX_POWER registers, according to the current tx channel and PRF
        * @return No return
        * @author Quentin Vey
        * @date 20200223
        */
        void writeSmartTxPowerConf();

        /**
        * @brief Check if TX power mode is smart.
        * @return true if DIS_STXP if set to 1, false otherwise
        * @author Quentin Vey
        * @date 20190628
        */
        bool isTxPowerSmart();

        /**
        * @brief Set power Mode to a manual value (same value for PHY header, SFD portion and data portion)
        * @param coarse : sets the coarse (DA) power setting. See DW1000 user manual, 7.2.31.1.
        * @param fine : expressed in half-db. Allowed values : 0 (0 dB) to 31 (15.5 dB). Sets the fine (mixer) power setting. See DW1000 user manual, 7.2.31.1.
        * @return No return
        * @author Quentin Vey
        * @date 20190628
        */
        void setManualTxPower(COARSE_POWER_SETTING coarse, unsigned int fine);

        /**
        * @brief Set power Mode to a manual value (same value for PHY header, SFD portion and data portion)
        * @param TX_POWER : value to write directly into register 0x1E
        * @return No return
        * @author Quentin Vey
        * @date 20200224
        */
        void setManualTxPowerRawFullRegister(uint32_t registerValue);

        /**
        * @brief Set power Mode to a manual value (same value for PHY header, SFD portion and data portion)
        * @param TX_POWER : value to write directly into register 0x1E (repeated 4 times)
        * @return No return
        * @author Quentin Vey
        * @date 20190701
        */
        void setManualTxPowerRaw(uint8_t registerValue);

        /**
        * @brief Set power mode to "manual" (i.e. without power boost), and sets the gain values according to what is recommended in DW1000 user manual.
        * @param trackChanges : whether the tx power conf should be updated when the channel or PRF are changed.
        * TX_POWER registers are set to their manufacturer default values
        * @return No return
        * @author Quentin Vey
        * @date 20200223
        */
        void setRecommendedFixedTxPower(bool trackChanges = true);

        /**
        * @brief writes the recommended value in 0x1E TX_POWER registers, according to the current tx channel and PRF
        * @return No return
        * @author Quentin Vey
        * @date 20200223
        */
        void writeRecommendedFixedTxPowerConf();

        /**
        * @brief Check if TX power mode is manual.
        * @return true if DIS_STXP if set to 0, false otherwise
        * @author Quentin Vey
        * @date 20190628
        */
        bool isTxPowerManual();

        /**
        * @brief Returns the content of TX_POWER register
        * @author Quentin Vey
        * @date 20190628
        */
        uint32_t getTX_POWER();

		/**
		* @brief Stores the System Time Counter value in the variable referenced by the pointer passed as an input parameter
		* @param p The address of the uint64_t variable
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void getSystemTimeCounter(uint64_t *p);

		/**
		* @brief Returns the System Time Counter value
		* @return The System Time Counter value as a uint64_t
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint64_t getSystemTimeCounter(void);

		/**
		* @brief Gets the PanId (Personnal Area Network Identifier) stored in the DW1000's RAM
		* @return The PanId as an uint16_t value
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint16_t getPanId();

		/**
		* @brief Gets the ShortAddress (16-bit network address, aka IEEE short address) stored in the DW1000's RAM
		* @return The Short Address as an uint16_t value
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint16_t getShortAddress();

		/**
		* @brief Gets the Euid (Extended Unique IDentifier) stored in the DW1000's ROM
		* @return The Identifier as an uint64_t value
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint64_t getEuid();

		/**
		* @brief Sets the PanId (Personnal Area Network Identifier) in the DW1000's RAM
		* @param panId The 16-bit PANID (PAN Identifier)
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void setPanId(uint16_t panId);

		/**
		* @brief Sets the ShortAddress (16-bit network address, aka IEEE short address) in the DW1000's RAM
		* @param shortAddress The 16-bit short address
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void setShortAddress(uint16_t shortAddress);

		/**
		* @brief Sets both the ShortAddress and the PanId in the DW1000's RAM
		* @param shortAddress The 16-bit short address
		* @param panId The 16-bit PANID (PAN Identifier)
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void setShortAddressAndPanId(uint16_t shortAddress, uint16_t panId); 

		/**
		* @brief Sets both the ShortAddress and the PanId in the DW1000's RAM
		* @param shortAddressPanId The 16-bit short address and 16-bit Pan Id as a 32-bit integer where short address in on the LSB.
		* @return true if success, false otherwise
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		int setShortAddressAndPanId(uint32_t shortAddressPanId);

		/**
		* @brief Returns the currently configured radio channels 
		* @return A byte which MSB is the X channel and the LSB is the X channel
		* @author Réjane Dalce
		* @date 20160109
		*/
 		uint8_t getChannelRaw(void);

		/**
		* @brief Returns the channel control register
		* @return A 4-byte register from the transceiver
		* @author Réjane Dalce
		* @date 20180806
		*/

		uint32_t getChanControl(void);

		/**
		* @brief Returns the currently configured radio channel
		* @return The channel value as an unsigned byte
		* @author Réjane Dalce
		* @date 20160109
		*/
 		uint8_t getChannel(void);

		/**
		* @brief Returns the currently configured Pulse Repetition Frequency
		* @return The PRF value as an unsigned byte
		* @author Réjane Dalce
		* @date 20161003
		*/
		uint8_t getRxPrf(void);

        /**
        * @brief Returns the currently configured Pulse Repetition Frequency
        * @return The PRF value as an unsigned byte
        * @author Quentin Vey
        * @date 20190717
        */
        uint8_t getTxPrf(void);

		/**
		* @brief Returns the currently configured Tx Preamble Code
		* @return The Preamble Code value as an unsigned byte
		* @author Réjane Dalce
		* @date 20161003
		*/
		
		

		
		
		uint8_t getFpAmpl1(void);
		
		/**
		* @brief Returns first path amplitude point 1
		* @return the amplitude as an uint8
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		uint16_t getFpAmpl2(void);
		/**
		* @brief Returns first path amplitude point 2
		* @return the amplitude as an uint16
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		uint16_t getFpAmpl3(void);
		
		/**
		* @brief Returns first path amplitude point 3
		* @return the amplitude as an uint16
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		uint16_t getRxPacc(void);
		/**
		* @brief Returns preamble accumulation count
		* @return preamble accumulation count as an uint16
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		double getFpPower(void);
		/**
		* @brief Returns first path amplitude power
		* @return first path amplitude power as a double (dBm)
		* @author Baptiste Pestourie
		* @date 20180614
		*/

		uint16_t getCirp(void);
		/**
		* @brief Returns Channel Impulse Response Power
		* @return CIRP as uint16
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		uint16_t getCire(void);
		/**
		* @brief Returns Standard Deviation of Channel Impulse Response Estimation
		* @return CIRE as uint16
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		double getRSSI(void);
		/**
		* @brief Returns received signal power
		* @return RSSI as double (dBm)
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		
		float getSNR(void);	
		/**
		* @brief Returns signal to noise ratio
		* @return SNR as float
		* @author Baptiste Pestourie
		* @date 20180614
		*/
		
		
		
		uint8_t getTxPcode(void);
 
		/**
		* @brief Returns the currently configured Rx Preamble Code
		* @return The Preamble Code value as an unsigned byte
		* @author Réjane Dalce
		* @date 20161003
		*/
		uint8_t getRxPcode(void);

        /**
         * @brief Gets the device identifier
         * @return The entire 32-bit DEV_ID register
         * @author Benjamin Freeman
         * @date 20181121
         */
        uint32_t getDevID(void);
 
 		/**
		* @brief Sets the radio channels for TX and RX
		* @param channel The channel number to set. Valid values are: 1, 2, 3, 4, 5, 7. 
		* @return Indicates whether configuration went well or not
		* @author Réjane Dalce
		* @date 20160109
		*/
 		bool setChannel(uint8_t channel);

		/**
        * @brief Sets the Pulse Repetition Frequency for both TX and RX
        * @param prf The PRF value to set. Valid values are: 16, 64
        * @return Indicates whether configuration went well or not
        * @author Quentin Vey
        * @date 20190717
        */
		bool setPrf(uint8_t prf);

        /**
        * @brief Sets the Tx and Rx Preamble Codes
        * @param pcode The Preamble Code to set. Valid values are: 1-20.
        * @return Indicates whether configuration went well or not
        * @author Quentin Vey
        * @date 20190724
        */
        bool setPcode(uint8_t pcode);

 		/**
		* @brief Sets the Tx Preamble Code
		* @param pcode The Preamble Code to set. Valid values are: 1-20.
		* @return Indicates whether configuration went well or not
		* @author Réjane Dalce
		* @date 20160310
		*/
		bool setTxPcode(uint8_t pcode);

 		/**
		* @brief Sets the Rx Preamble Code
		* @param pcode The Preamble Code to set. Valid values are: 1-20.
		* @return Indicates whether configuration went well or not
		* @author Réjane Dalce
		* @date 20160310
		*/
		bool setRxPcode(uint8_t pcode);

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
		* @author Adrien van den Bossche
		* @date 20151028
		*/
		uint64_t alignDelayedTransmission ( uint64_t wantedDelay );

		/**
		* @brief Sends a len-byte frame from buf
		* @param buf The address of the buffer
		* @param len The message length
		* @return true if success, false otherwise
		* @author Adrien van den Bossche
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
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t pdDataRequest(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time);

		/**
		* @brief Sends a len-byte frame from buf
		* @param buf The address of the buffer
		* @param len The message length
		* @return true if success, false otherwise
		* @author Adrien van den Bossche
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
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t send(uint8_t* buf, uint16_t len, uint8_t delayed, uint64_t time);

		/**
		* @brief Sets the RX buffer for future frame reception. Received bytes will be stored at the beginning of the buffer.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void setRxBuffer(uint8_t* buf, uint16_t *len);

		/**
		* @brief Sets the RX buffer for future frame reception. Received bytes will be stored at the end of the buffer of max size.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void setRxBuffer(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Sets transceiver mode to receive mode
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void plmeRxEnableRequest(void);

		/**
		* @brief Sets transceiver mode to receive mode. Received bytes will be stored at the end of the buffer of max size.
		* @param max The buffer size
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint16_t max);

		/**
		* @brief Sets transceiver mode to receive mode and set the RX buffer for future frame reception.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint8_t* buf, uint16_t *len);

		/**
		* @brief Sets transceiver mode to receive mode and set the RX buffer for future frame reception. Received bytes will be stored at the end of the buffer of max size.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size 
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void plmeRxEnableRequest(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Sets transceiver mode to idle mode.
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void plmeRxDisableRequest(void);

		/**
		* @brief Sets transceiver mode to sleep mode.
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void sleepRequest(void);

		/**
		* @brief Sets transceiver mode to deep sleep mode.
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		* @todo To be implemented
		*/
		void deepsleepRequest(void);

		/**
		* @brief Wakes the transceiver and go back to idle mode.
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		* @todo To be implemented
		*/
		void wakeRequest(void);

		/**
		* @brief Returns true if a frame has been received.
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(void);

		/**
		* @brief Returns true if a frame has been received, copy received bytes in buf and store message length in len. 
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(uint8_t* buf, uint16_t *len);

		/**
		* @brief Returns true if a frame has been received, copy received bytes in buf and store message length in len. The received bytes shall be copied toward the end of the buffer of size max.
		* @param buf The address of the buffer
		* @param len The address of the message length
		* @param max The buffer size 
		* @return true if a frame has been received, false otherwise.
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t rxFrameAvailable(uint8_t* buf, uint16_t *len, uint16_t max);

		/**
		* @brief Returns true if the last transmission request has been succefully completed
		* @return true if the last transmission request has been succefully completed
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		bool hasTxSucceeded(void);

		/**
		* @brief Gets the DecaDuino transceiver status
		* @return the DecaDuino transceiver status
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t getTrxStatus(void);

		/**
		* @brief Gets the raw value from the DW1000's embedded temperature sensor
		* @return The temperature raw value
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t getTemperatureRaw(void);

		/**
		* @brief Gets the temperature value in celsius degrees from the DW1000's embedded temperature sensor
		* @return The temperature value in celsius degrees
		* @author Adrien van den Bossche
		* @date 20141115
		* @todo To be implemented
		*/
		float getTemperature(void);

		/**
		* @brief Gets the raw value from the DW1000's embedded voltage sensor
		* @return The voltage raw value
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint8_t getVoltageRaw(void);

		/**
		* @brief Gets the voltage value in volts from the DW1000's embedded voltage sensor
		* @return The voltage value in volts
		* @author Adrien van den Bossche
		* @date 20141115
		* @todo To be implemented
		*/
		float getVoltage(void);


		/**
		* @brief Gets the content of register file: 0x15 (Receive Time Stamp).
		* @return content of the register
		* @date 20190527
		* @author Quentin Vey
		*/
		RXTime_t getRxTimeRegister();

		/**
        * @brief Gets the content of register file: 0x15 (Receive Time Stamp) as a JSON string.
        * @param buf address of the character array where the string will be written (should be at least 95 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxTimeRegisterAsJSon(char *buf, int maxlen);

        /**
        * @brief Gets the content of register file: 0x15 (Receive Time Stamp) as a JSON string.
        * @param data data to print
        * @param buf address of the character array where the string will be written (should be at least 95 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxTimeRegisterAsJSon(const RXTime_t &data, char *buf, int maxlen);

        /**
        * @brief Gets the content of register file: 0x12 (Rx Frame Quality Information).
        * @return content of the register
        * @date 20190527
        * @author Quentin Vey
        */
        RXFQual_t getRxQualityRegister();

        /**
        * @brief Gets the content of register file: 0x12 (Rx Frame Quality Information) as a JSon string.
        * @param buf address of the character array where the string will be written (should be at least 77 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxQualityRegisterAsJSon(char *buf, int maxlen);

        /**
        * @brief Gets the content of register file: 0x12 (Rx Frame Quality Information) as a JSon string.
        * @param data data to print
        * @param buf address of the character array where the string will be written (should be at least 77 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxQualityRegisterAsJSon(const RXFQual_t &data, char *buf, int maxlen);

        /**
        * @brief Gets the content of register file: 0x10 (Rx Frame Information).
        * @return content of the register
        * @date 20190527
        * @author Quentin Vey
        */
        RXFInfo_t getRxFrameInfoRegister();

        /**
        * @brief Gets the content of register file: 0x10 (Rx Frame Information) as a JSon string.
        * @param buf address of the character array where the string will be written (should be at least 90 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxFrameInfoRegisterAsJSon(char *buf, int maxlen);


        /**
        * @brief Gets the content of register file: 0x10 (Rx Frame Information) as a JSon string.
        * @param data data to print
        * @param buf address of the character array where the string will be written (should be at least 90 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190527
        * @author Quentin Vey
        */
        int getRxFrameInfoRegisterAsJSon(const RXFInfo_t &data, char *buf, int maxlen);

        /**
        * @brief Gets full content of register file: 0x1F (channel control).
        * @return content of the register
        * @date 20190603
        * @author Quentin Vey
        */
        channelCTRL_t getChannelControlRegister();

        /**
        * @brief Gets full content of register file: 0x1F (channel control) as a JSon string.
        * @param buf address of the character array where the string will be written (should be at least 128 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190603
        * @author Quentin Vey
        */
        int getChannelControlRegisterAsJSon(char *buf, int maxlen);

        /**
        * @brief Gets full content of register file: 0x1F (channel control) as a JSon string.
        * @param data data to print
        * @param buf address of the character array where the string will be written (should be at least 128 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190603
        * @author Quentin Vey
        */
        int getChannelControlRegisterAsJSon(const channelCTRL_t &data, char *buf, int maxlen);
        /**
        * @brief Gets full content of register file: 0x2E (LDE interface).
        * @return  content of the register
        * @date 20190603
        * @author Quentin Vey
        */
        LDEInterface_t getLDEInterfaceRegister();

        /**
        * @brief Gets full content of register file: 0x2E (channel control) as a JSon string.
        * @param buf address of the character array where the string will be written (should be at least 128 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190603
        * @author Quentin Vey
        */
        int getChannelLDEInterfaceAsJSon(char *buf, int maxlen);

        /**
        * @brief Gets full content of register file: 0x2E (channel control) as a JSon string.
        * @param data data to print
        * @param buf address of the character array where the string will be written (should be at least 128 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (excluding the trailing null byte)
        * @date 20190603
        * @author Quentin Vey
        */
        int getChannelLDEInterfaceAsJSon(const LDEInterface_t &data, char *buf, int maxlen);

        /**
        * @brief Gets bit 22 of register 0x04.
        * @return Receiver Mode 110 kbps data rate.
        * @date 20190603
        * @author Quentin Vey
        */
        uint8_t getRXM110K();

        /**
        * @brief Gets subregister 0x21:00.
        * @return length of the SFD sequence used (not used for standard SFD sequences).
        * @date 20190603
        * @author Quentin Vey
        */
        uint8_t getSFD_LENGTH();

        /**
        * @brief sets subregister 0x21:00.
        * @param length of the SFD sequence used (not used for standard SFD sequences).
        * @date 20190717
        * @author Quentin Vey
        */
        void setSFD_LENGTH(uint8_t SFD_LENGTH);

        /**
        * @brief sets subregister 0x27:20 DRX_STDTOC.
        * @param timeout for SFD detection (in units of preamble symbols).
        * @date 20190724
        * @author Quentin Vey
        */
        void setSFDTimeout(uint16_t timeout);

        /**
        * @brief Gets content of subregister file: 0x27:2C (Unsaturated accumulated preamble symbols).
        * @return Unsaturated accumulated preamble symbols
        * @date 20190603
        * @author Quentin Vey
        */
        uint16_t getRXPACC_NOSAT();

        /**
        * @brief Enables/disables CIR merory read (sets/unsets the FACE and AMCE bits)
        * @param enable true to enable CIR read, false to disable them
        * @date 20190527
        * @author Quentin Vey
        */
        void enableCIRAccumulatorRead(bool enable);

        /**
        * @brief Gets the content of register file: 0x25 (CIR memory accumulator). Will start reading
        * at startIndex, and read at most arrayLength values into buffer. There is a small bonus in
        * time (one less SPI read and one less array shift) if arrayLength is a least 1 element
        * larger that the actually required length (992|1016 if readLength==-1, readLength otherwise).
        * @param buffer address of a CIRSample_t array
        * @param maximum number of samples that *buffer can hold (should be at least 992 for a 16 MHz PRF, or 1016 for 64MHz PRF, ideally these values + 1 )
        * @param startIndex index where to start reading CIR samples
        * @param readLength Nomber of samples to read (-1 means read all samples)
        * @return number of samples written
        * @date 20190527
        * @author Quentin Vey
        */
        int getCIRAccumulator(CIRSample_t *buffer, size_t arrayLength, unsigned int startIndex = 0, int readLength = -1);

        /**
        * @brief Gets the content of register file: 0x25 (CIR memory accumulator) as a JSon array string.
        * @param buf address of the character array where the string will be written (maximum required : 29464 bytes long)
        * @param maxlen size of the character array
        * @param startIndex index where to start reading CIR samples
        * @param readLength Nomber of samples to read (-1 means read all samples)
        * @return numbers of characters written (or that would have been written)
        * @date 20190527
        * @author Quentin Vey
        */
        int getCIRAccumulatorAsJSon(char* buf, uint16_t maxlen, unsigned int startIndex = 0, int readLength = -1);

        /**
        * @brief Converts the content of register file: 0x25 (CIR memory accumulator) to a JSon array string.
        * @param data adress of the data to convert
        * @param dataLength number of samples to print
        * @param buf address of the character array where the string will be written (maximum required : 29464 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (or that would have been written)
        * @date 20190527
        * @author Quentin Vey
        */
        int CIRAccumulatorToJSon(CIRSample_t *samples, uint16_t numSamples, char* buf, uint16_t maxlen);

        /**
        * @brief Gets the content of register file: 0x25 (CIR memory accumulator) as a JSon array string, but with values encoded in base64.
        * @param buf address of the character array where the string will be written (maximum required : 29464 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (or that would have been written)
        * @date 20190611
        * @author Quentin Vey
        */
        int getCIRAccumulatorAsBase64JSon(char* buf, uint16_t maxlen, unsigned int startIndex = 0, int readLength = -1);

        /**
        * @brief Converts the content of register file: 0x25 (CIR memory accumulator) as a JSon array string but with values encoded in base64.
        * The data is an array of int16_t values written in big-endian, grouped by pairs (real part, imaginary part) :
        * byte  bit
        *       0-------------------7---------------------15-----------------------23------------------------31
        *    0  | sample_0_real_low |  sample_0_real_high | sample_0_imaginary_low | sample_0_imaginary_high |
        *    4  | sample_2_real_low |  sample_2_real_high | sample_2_imaginary_low | sample_2_imaginary_high |
        *                                                ...
        *    4n | sample_n_real_low |  sample_n_real_high | sample_n_imaginary_low | sample_n_imaginary_high |
        * @param data adress of the data to convert
        * @param dataLength number of samples to print
        * @param buf address of the character array where the string will be written (maximum required : 29464 bytes long)
        * @param maxlen size of the character array
        * @return numbers of characters written (or that would have been written)
        * @date 20190611
        * @author Quentin Vey
        */
        int CIRAccumulatorToBase64JSon(CIRSample_t *samples, uint16_t numSamples, char* buf, uint16_t maxlen);

		/**
		* @brief Builds an uint16 value from two uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint16_t
		* @author Adrien van den Bossche
		* @date 20111123
		*/
		uint16_t decodeUint16 ( uint8_t *data );

		/**
		* @brief Formats an uint16 value as a list of uint8 values
		* @param from The uint16_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche
		* @date 20111011
		*/
		void encodeUint16 ( uint16_t from, uint8_t *to );

		/**
		* @brief Builds an uint32 value from four uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint32_t
		* @author Adrien van den Bossche
		* @date 20111123
		*/
		uint32_t decodeUint32 ( uint8_t *data );

		/**
		* @brief Formats an uint32 value as a list of uint8 values
		* @param from The uint32_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche
		* @date 20111011
		*/
		void encodeUint32 ( uint32_t from, uint8_t *to );

		/**
		* @brief Builds an uint64 value from five uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint64_t
		*/ 
		uint64_t decodeUint40 ( uint8_t *data );

		/**
		* @brief Formats an uint64 value with only 5 LSbytes as a list of uint8 values
		* @param from The uint64_t value
		* @param to The address of the uint8_t buffer
		*/ 
		void encodeUint40 ( uint64_t from, uint8_t *to );	
		
		/**
		* @brief Builds an uint64 value from eight uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded uint64_t
		* @author Adrien van den Bossche
		* @date 20140804
		*/
		uint64_t decodeUint64 ( uint8_t *data );

		/**
		* @brief Formats an uint64 value as a list of uint8 values
		* @param from The uint64_t value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche
		* @date 20111011
		*/
		void encodeUint64 ( uint64_t from, uint8_t *to );

		/**
		* @brief Builds a float value from four uint8 values
		* @param data The address of the uint8_t buffer
		* @return The decoded float
		* @author Adrien van den Bossche
		* @date 20171020
		*/
		float decodeFloat ( uint8_t *data );

		/**
		* @brief Formats an float value as a list of uint8 values
		* @param from The float value
		* @param to The address of the uint8_t buffer
		* @return No return
		* @author Adrien van den Bossche
		* @date 20171020
		*/
		void encodeFloat ( float from, uint8_t *to );

		/**
		* @brief Prints an uint64_t value on console
		* @param ui64 The uint64_t value
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void printUint64 ( uint64_t ui64 );

		/**
		* @brief Returns last transmitted frame timestamp based on the DWM1000 System Time Counter at 64GHz
		* @return Last transmitted frame timestamp
		* @author Adrien van den Bossche
		* @date 20140905
		*/
		uint64_t getLastTxTimestamp();

		/**
		* @brief Returns last received frame timestamp based on the DWM1000 System Time Counter at 64GHz
		* @return Last received frame timestamp
		* @author Adrien van den Bossche
		* @date 20140905
		*/
		uint64_t getLastRxTimestamp();

		/**
		* @brief Returns last received frame's clock skew, also designated as clock offset in the Decawave documentation
		* @return Last received frame's clock skew
		* @author Adrien van den Bossche
		* @date 20150905
		*/
		double getLastRxSkew();

		/**
		* @brief Returns current antenna delay value
		* @return The current antenna delay value
		* @author Adrien van den Bossche
		* @date 20160915
		*/
		uint16_t getAntennaDelay();

		/**
		* @brief Sets the current antenna delay value
		* @param antennaDelay The antenna delay value
		* @param trackChanges Enables antenna delay updates on frequence or PRF changes
		* @return No return
		* @author Adrien van den Bossche
		* @date 20160915
		*/
		void setAntennaDelay(uint16_t newAntennaDelay, bool trackChanges=false);

		/**
        * @brief Sets the current antenna delay value to the one defined in calibratedAntennaDelay, and makes the antenna delay follow frequency and PRF changes
        * @return No return
        * @author Quentin Vey
        * @date 20200211
        */
		void useCalibratedAntennaDelay();

		/**
        * @brief One-time setting the current antenna delay value to the one defined in calibratedAntennaDelay for
        * current config. If you want this to be done automatically on channel and PRF change, call useCalibratedAntennaDelay().
        * @return No return
        * @author Quentin Vey
        * @date 20200211
        */
		void setCalibratedAntennaDelay();

		/**
		* @brief Gets the NLOS indication value associated with the latest reception. 
		* Interpretation is based on the result of 10log10(x), x being the return value of the function.
		* A result less than 6dB suggests a LOS channel while a result greater than 10dB indicates an NLOS channel.
		* @param None
		* @return The estimated NLOS indicator
		* @author Rejane Dalce
		* @date 20170420
		*/
		float getNLOSIndication(void);

        /**
         * @brief Gets the transmission data rate
         * @return The enum value of the data rate
         * @author Benjamin Freeman
         * @date 20181121
         */
        dw1000_datarate_t getDataRate();

        /**
         * @brief Sets the transmission data rate
         * @param rate The enum value of the data rate
         * @return No return
         * @author Benjamin Freeman
         * @date 20181121
         */
        void setDataRate(dw1000_datarate_t rate);

        /**
        * @brief returns the NTM (LDE's Noise Threshold Multiplier)
        * @author Quentin Vey
        * @date 20190717
        */
        uint8_t getNTM(void);

        /**
        * @brief Sets the NTM (LDE's Noise Threshold Multiplier)
        * @param NTM value to set. Must be between 0 and 31
        * @return Indicates whether configuration went well or not
        * @author Quentin Vey
        * @date 20190717
        */
        bool setNTM(uint8_t NTM);

        /**
        * @brief returns the PMULT (LDE's peak multiplier)
        * @author Quentin Vey
        * @date 20190717
        */
        uint8_t getPMULT(void);

        /**
        * @brief sets the PMULT (LDE's peak multiplier)
        * @param PMULT value to set. Must be between 0 and 7
        * @return Indicates whether configuration went well or not
        * @author Quentin Vey
        * @date 20190717
        */
        bool setPMULT(uint8_t PMULT);

        /**
        * @brief Makes use of the standard SFD in TX and RX
        * @author Quentin Vey
        * @date 20190717
        */
        void setStandardSFD();

        /**
        * @brief Makes use DecaWave-recommended SFD in TX and RX (see Table 21: Recommended SFD sequence configurations for best performance in DW1000 user manual)
        * @author Quentin Vey
        * @date 20190717
        */
        void setDecaWaveSFD();

        /**
        * @brief sets the channel settings to the defaults values
        * @author Quentin Vey
        * @date 20190717
        */
        void setDefaultChannelConfig();

        /**
        * @brief changes the tuning of some parameters to improve NLOS performance
        * @author Quentin Vey
        * @date 20190717
        */
        void enableNLOSTunings();

        /**
        * @brief Disables NLOS improvements
        * @author Quentin Vey
        * @date 20190717
        */
        void disableNLOSTunings();

        /**
        * @brief returns true if NLOS tunings are enabled
        * @author Quentin Vey
        * @date 20190717
        */
        bool getNLOSTunings();


	private:


        /**
        * @brief Sets the Pulse Repetition Frequency for RX
        * @param prf The PRF value to set. Valid values are: 16, 64.
        * @return Indicates whether configuration went well or not
        * @author Réjane Dalce
        * @date 20160310
        */
        bool setRxPrf(uint8_t prf);

        /**
        * @brief Sets the Pulse Repetition Frequency for TX
        * @param prf The PRF value to set. Valid values are: 16, 64
        * @return Indicates whether configuration went well or not
        * @author Quentin Vey
        * @date 20190717
        */
        bool setTxPrf(uint8_t prf);

        /**
        * @brief returns the recommended PAC size
        * @param preamble_len the expected length of the received preamble
        * @return recommened PAC size
        * @author Quentin Vey
        * @date 20190717
        */
        uint8_t recommendedPACSize(uint16_t preamble_len);

		/**
		* @brief Reads len bytes on SPI at given address, and store data in buf
		* @param address The source address
		* @param buf The address of the buffer
		* @param len The message length
		* @return No return
		* @author Adrien van den Bossche
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
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void readSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);

		/**
		* @brief Reads a 4-byte word on SPI at given address
		* @param address The source address
		* @return The 4 bytes
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		uint32_t readSpiUint32(uint8_t address);

		/**
		* @brief Writes len bytes on SPI at given address from buf
		* @param address The destination address
		* @param buf The address of the buffer
		* @param len The message length
		* @return No return
		* @author Adrien van den Bossche
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
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void writeSpiSubAddress(uint8_t address, uint16_t subAddress, uint8_t* buf, uint16_t len);

		/**
		* @brief Writes a 4-byte word on SPI at given address
		* @param address The destination address
		* @param ui32t The 4-byte word to write on SPI
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void writeSpiUint32(uint8_t address, uint32_t ui32t);

		/**
		* @brief Returns the antenna delay value in the DW1000 register
		* @return The antenna delay value in the register
		* @author Adrien van den Bossche
		* @date 20160915
		*/
		uint16_t getTXAntennaDelayReg();

		/**
		* @brief Sets the TX antenna delay value in the DW1000 register
		* @param antennaDelay The antenna delay value
		* @return No return
		* @author Adrien van den Bossche
		* @date 20160915
		*/
		void setTXAntennaDelayReg(uint16_t newAntennaDelay);

        /**
        * @brief Returns the RX antenna delay value in the DW1000 register
        * @return The antenna delay value in the register
        * @author Quentin Vey
        * @date 20190917
        */
        uint16_t getRXAntennaDelayReg();
		/**
        * @brief Sets the RX antenna delay value in the DW1000 register
        * @param antennaDelay The antenna delay value
        * @return No return
        * @author Quentin Vey
        * @date 20190917
        */
        void setRXAntennaDelayReg(uint16_t newAntennaDelay);

		uint8_t debugStr[DEBUG_STR_LEN];
		void spi_send ( uint8_t u8 );
		void spi_send ( uint16_t u16 );
		void spi_send ( uint8_t* buf, uint16_t len );
		void spi_receive ( uint8_t* buf, uint16_t len );

		uint16_t antennaDelay;
		bool _antennaDelayTracksChanges = true; // if true, then the antenna delay is changed to the calibrated value when PRF or channel is changed.
		bool _txPowerTracksChanges = true;      // if true, then the TX power config registry is changed to the recommended value when PRF or channel is changed.
		bool _DWSFD = false;    // use decawave-recommended SFD settings
		bool _NLOSOptims = false;   // enables NLOS optimizations recommended by decawave
	
	protected:

		/**
		* @brief The first interrupt function
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		static void isr0();

		/**
		* @brief The second interrupt function
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		static void isr1();

		/**
		* @brief The third interrupt function
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		static void isr2();

		/**
		* @brief The global interrupt function
		* @return No return
		* @author Adrien van den Bossche
		* @date 20141115
		*/
		void handleInterrupt();

		/**
		* @brief Current SPI-bus settings
		*/
		SPISettings currentSPISettings;
		
		/**
		* @brief Current EUID (Extended Unique IDentifier)
		*/
		uint64_t euid;

		/**
		* @brief The current (or last) PPDU
		*/
		uint8_t *rxData;

		/**
		* @brief The current PPDU length
		*/
		uint16_t *rxDataLen;
		/**
		* @brief The max PPDU length
		*/
		uint16_t rxDataLenMax;

		/**
		* @brief Flag indicating if last reception has data
		*/
		uint8_t rxDataAvailable;

		/**
		* @brief Transceiver status
		*/
		uint8_t trxStatus;

		/**
		* @brief Flag indicating if last transmission is done
		*/
		bool lastTxOK;

		/**
		* @brief Timestamp of last transmitted frame
		*/
		uint64_t lastTxTimestamp;

		/**
		* @brief Timestamp of last received frame
		*/
		uint64_t lastRxTimestamp;

		/**
		* @brief Last clock offset (aka clock skew)
		*/
		double clkOffset;
		uint8_t _slaveSelectPin;
		uint8_t _interruptPin;
		uint8_t _rxPrf = 16;
		static DecaDuino* _DecaDuinoInterrupt[];
};

#endif
