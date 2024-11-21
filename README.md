# Welcome to the ArfBotOS wiki!
ArfBotOS is an operating system for a 6-axis robot and vision system running on CoDeSys, Arduino, and OpenCV. One of the main intentions of ArfBotOS is to give an example of how a typical industrial control system is programmed in IEC-61131 (structured text). This will give good experience to new Automation and Controls engineers. Please see the wiki [here](https://github.com/dalethomas81/ArfBotOS/wiki) for more.

ArfBotOS has a web-base HMI that is hosted via what is known in CoDeSys as a 'Visu'. From the **Main** HMI screen you can select a program to run and view its movement from the *Process Viewer* group box.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Resources/images/readme/hmi-main-pcs-corners.JPG" alt="menu" width="600"/> 

The architecture of the software implements the idea of *Processors* that handle different types of commands. For example, the *MoveCommandProcessor* parses a move command and moves the robot. Each processor extends a base function block that implements a PackML statemachine. From the **Main** HMI screen, you can view and control the *Orchestrator* and *Active Processor's* state machine.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Resources/images/readme/hmi-main-packml.JPG" alt="menu" width="600"/>

From the **Jogging** HMI screen you can jog the robot in all coordinate systems using the HMI buttons or a Playstation Dualsense controller.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Resources/images/readme/hmi-jog.JPG" alt="menu" width="600"/>
 
Disclaimer: The cost of the hardware in this project has been kept at a minimum to make it more attainable. However, there is a Codesys licensing cost associated with this project but the runtime will run in demo mode for 2 hours if you do not purchase the license. (You can restart the runtime to restart the 2 hour demo mode). The licensing cost approx $1000.00 USD and this will get you multi-core support on the Raspberry Pi as well as CNC/Robotics motion control.

## Hello World
If you already have ArfBotOS running and have it connected to a robot, you will want to create your first program. I have put together a Hello World project that will help you do that. You can find it in the wiki. Otherwise, please follow the installation and configuration instructions below to get started.

## The Robot
In theory, you can use any 6 axis robot that you would like with ArfBotOS. However, you will need to integrate the connections to the servos and limit switches. Additionally, you will need recompile ArfBotOS with the Denavit-Hartenburg (DH) parmeters for the bot of your choice. I chose to use the fantastic open source AR4 robot designed and developed by Chris Annin of [Annin Robotics](https://www.anninrobotics.com/)