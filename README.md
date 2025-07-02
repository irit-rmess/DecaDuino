# DecaDuino

DecaDuino is an Arduino library which provides a driver for the DecaWave DW1000
transceiver, and modules based on this transceiver, such as DecaWave DWM1000.
Since the DecaWave DW1000/DWM1000 is based on a Ultra Wide Band (UWB) Physical
layer, DecaDuino can be used as an open framework for wireless Time-of-Flight
(ToF) ranging systems.

For more details on the DecaDuino library, get the latest version of the
documentation here: https://www.irit.fr/~Adrien.Van-Den-Bossche/decaduino/)

## Installation
You have to put this library in your Arduino libraries directory.

### dependencies
You need to have the library printfToSerial in your Arduino libraries directory (https://github.com/quent8192/printfToSerial).

### DecaWino (Teensy)

Instructions here: https://wino.cc/tutorials/teensywino-installing-software-development-tools/

### DWM1001-DEV (nRF5x)

Instructions here: https://github.com/irit-rmess/arduino-nRF5
(more details in french https://octeus.iut-blagnac.fr/enseignants/octeus2/tp/micro-tp-cha%C3%AEne-de-compilation/)

## Start using DecaDuino with simple example sketchs
All example sketches are available as usual in the Arduino IDE menu: 
```
File > Examples > DecaDuino
```

### DecaDuinoSender

This sketch shows how to use the DecaDuino library to send messages over the UWB
radio.

### DecaDuinoReceiverSniffer

This sketch shows how to use the DecaDuino library to receive messages over the
UWB radio. The received bytes are printed in HEX; this sketch can be used as a
frame sniffer to dump received messages.

### DecaDuinoChat

This sketch shows how to use the DecaDuino library to send and receive ascii
messages via the Serial port over the UWB radio. Commands are available to
change the radio's configuration: channel, datarate and preamble length.

### DecaDuinoJSON

This sketch shows how to use the DecaDuino library to control the UWB radio
with JSON over serial. The JSON format used is based on LoraServer's Gateway
Bridge JSON format (https://www.loraserver.io/lora-gateway-bridge/integrate/payload-types/json/).

## Use DecaDuino to implement ranging protocols

DecaDuino can be used to implement Time-of-Flight (ToF) ranging protocols
by timestamping frames at transmission and reception. The DecaDuino library
comes with a set of sketches that implement popular ranging protocols.

### Two-Way Ranging (TWR) protocol

The two example sketchs DecaDuinoTWR_client and DecaDuinoTWR_server propose a
simple implementation of the TWR protocol without addressing fields. 

Flash each example sketch on two nodes (client and server) and get the distance
between the two nodes. The documentation directory contains the client and
server state machine diagram.

### Symmetric Double-Sided Two-Way Ranging (SDS-TWR) protocol

The two example sketchs DecaDuinoSDSTWR_client and DecaDuinoSDSTWR_server
propose a simple implementation of the SDS-TWR protocol without addressing
fields. The SDS-TWR protocol implies more messages but is theorically better
than the TWR protocol since the clock skew effect is compensated by the
symetry of the exchanges. 

Flash each example sketch on two nodes (client and server) and get the distance
between the two nodes.

