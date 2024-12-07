# AR4 Robot
The AR4 is a fantastic open-source 6-axis robot designed by Chris Annin of [Annin Robotics](https://www.anninrobotics.com/). This directory will show you how to interface the AR4 with ArfBotOS.

- The electrical schedmatics will be located [here](/Robots/AR4/Electrical).  
- The setup procedure for tuning the drives, mastering the robot and programming the devices (microcontroller and EtherCAT card) will be located [here](/Robots/AR4/Setup).  
- Firmware for the Teensy microcontroller will be located [here](/Robots/AR4/Arduino).  

## EtherCAT Motor Control Unit (EMCU) Overview  
The EMCU is a crucial component in the robot's control system, facilitating communication between the PLC and the stepper controllers. It manages the conversion of frequency signals, interfaces with quadrature encoders, and processes calibration inputs. The EMCU uses EtherCAT as its communication protocol.  

<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Electrical/Images/EMCU-Overview-Dark.png" alt="EMCU" width="600"/> 

### Functionality Description:

#### Frequency to Pulse Train Output Conversion:
* Input: The PLC sends frequency signals for each of the 6 axes to the EMCU via EtherCAT.

* Process: The EMCU converts these frequency signals into PTO (Pulse Train Output) signals.

* Output: The PTO signals are then fed into each of the 6 stepper controllers, which drive the stepper motors.

#### Quadrature Encoder Interface:
* Input: The EMCU receives signals from 6 quadrature encoders, which include 12 hardware interrupts (A and B signals for each encoder).

* Process: It maintains a counter for each encoder to keep track of the position values.

* Output: These counter values are sent back to the PLC as abstracted incremental encoder values via EtherCAT.

#### Calibration Switches:
* Input: Each of the 6 axes has a calibration switch that the EMCU reads.

* Process: The EMCU processes the inputs from these switches.

* Output: The processed values are sent back to the PLC. The PLC uses these values to perform homing operations for the robot.

