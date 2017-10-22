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

## Start using DecaDuino with simple example sketchs
All example sketches are availabled as usual in the Arduino IDE menu: 
```
File > Examples > DecaDuino
```

### DecaDuinoSender

This sketch shows how to use the DecaDuino library to send messages over the UWB
radio

### DecaDuinoReceiverSniffer

This sketch shows how to use the DecaDuino library to receive messages over the
UWB radio. The received bytes are printed in HEX; this sketch can be used as a
frame sniffer to dump received messages.

### DecaDuinoChat


