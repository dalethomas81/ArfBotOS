# Firmware Installation

## Arduino

1. Install Arduino according to the instructions in the Wiki located [here](https://github.com/dalethomas81/ArfBotOS/wiki/Installation#installing-arduino).
2. Open the ArfBot.ino Arduino project file located here in the ArfBotOS repo `ArfBotOS\Robots\AR4\Arduino\ArfBot`.
3. Connect the Teensy 4.1 to your computer with a USB cable.
4. In the Arduino IDE, select the Teensy 4.1 board in the top drop-down.
5. Upload the firmward using the *Upload* button (arrow pointing right).  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/Arduino-Upload.png" alt="menu" width="400"/>

## EasyCAT  

1. Clone the [EasyCAT](https://github.com/dslemusp/EasyCAT) repo.
2. Open the Easy Configurator located here in the repo `EasyCAT\configurator\Exe\EasyCAT_Config_GUI.exe`.
3. With the Microchip EVB-LAN9252-SPI powered, connect an ethernet cable from your computer to the `In` port (left side).  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Arduino-Firmware-Flash.png" alt="menu" width="600"/>
4. Click on *Write EEPROM* and select the 32 input by 32 output configuration file located here in the repo `EasyCAT\configurator\StandardMode_BinFiles\EasyCAT_32_32_rev_1.bin`.
5. Wait for programming to be complete. if programming fails, refer to troubleshooting at www.bausano.net  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EasyCAT-Confirm.png" alt="menu" width="600"/>  

_note: if you are not using the EK1100, EL1809, and EL2809 (remote input and output) you may see a `Networkadapter opened` error in the EtherCAT Master device in CODESYS. In this case, you will need to disable the unused devices and download the program again._  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCat-Error-NetworkOpened.png" alt="menu" width="600"/>  

### Disable Unused IO

1. Make sure you are offline with the PLC from within CODESYS.
2. Find the EK1100 in the Device Tree.
3. Right-click and select Disable (this will not show if you are online with the PLC).  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Disable-Unused.png" alt="menu" width="600"/> 
4. Download the project by selecting Online->Multiple Download and following the prompts. The EtherCAT chain will now be restored working with only the AR4 robot connected.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Running-After-Disable-Unused.png" alt="menu" width="600"/>  

ArfBotOS will automatically map the IO of the device that is enabled in the Device tree. You can select `IoOption1` or `IoOption2` or neither. Simply right-click on the devices and enable/disable them according to how you will use it then download to the PLC.  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Resources/images/readme/IoOption.png" alt="menu" width="600"/>  
 