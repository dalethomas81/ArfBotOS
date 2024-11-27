# Electrical

The AR4 robot comes with a very detailed build guide that includes electrical diagrams. For ArfBotOS, we will need to modify the electrical diagram slightly so that we can make use of the SPI port of the Teensy controller to connect to the EVB-LAN9252-SPI EtherCAT Slave Controller.  

### Fritzing

The electrical diagram found [here](https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Electrical/ElectricalDiagram.fzz) was created in Fritzing to help illustrate the connections.  

The schematic detailing all of the electrical connections can be found [here](https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Electrical/AR4-Schematic.pdf)  

### EtherCAT Slave Connections

The EtherCAT slave board (EVB-LAN9252-SPI) communicates with the Teensy 4.1 microcontroller using SPI. For this reason, the SPI pins 10, 11, 12, and 13 on the Teensy needed to be freed up opposed to the stock connections of the AR4.  

1. Solder short wires to the 6 connections for `gnd`, `3v3`, `miso`, `mosi`, `sck`, and `cs` on the bottom side of the EtherCAT Slave Controller.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Solder-Underside.png" alt="menu" width="600"/> 
2. Keeping the wires as short as possible to reduce EMI - connect them each to the appropriate terminal on the Teensy controller.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Wired.png" alt="menu" width="600"/>  

### Known Issues
The stepper drives for the AR4 are very noisy, electrically and the electronics such as the encoders and other fast, low-voltage signals can take a beating from them.  

It is recommended to put 0.01uF capacitors between these signals and ground to create a filter for this EMI.  

For example, connect one 0.01uF capacitor between each of the encoder inputs (A and B) and ground if you experience issues like encoders missing steps or if the axes sound jittery when they are moving.  