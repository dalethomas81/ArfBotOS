## EtherCAT Wedge

The EtherCAT Wedge is a device that mounts between the J1 base enclosure and J1 base lid of the AR4 and houses the EtherCAT communication card used to connect the AR4 drives with the CODESYS PLC.  

### Fabrication
I have included the Fusion 360 files so that you may modify it as you like. Additionally, you can print the *Wedge.stl* file directly using a slicer like Cura.  

Cura Settings:
- Supports - on
- Build plate adhesion - raft
- Nozzle Temp - 210C
- Bed Temp - 70C

### Installation

1. Print the EtherCAT Wedge using a 3d printer.
2. Remove the AR4 J1 base lid.
3. Rewire the encoders, stepper drives, and limit switches according to the Fritzing electrical drawing located [here](https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Electrical/README.MD). _note: the stock wiring of the AR4 needed to be modified slightly to accomodate SPI communication for EtherCAT._
4. Insert the EVB-LAN9252-SPI into the Wedge. This is made to be a snug fit.
5. Mount the wedge in between the base and lid and secure the lid using M3x40mm machine screws. _note: you will need to use a drill bit to open the holes enough to accomodate the larger machine screws. DO NOT DRILL INTO THE BASE HOLES!!! These will need to remain tight to hold the screw threads._  