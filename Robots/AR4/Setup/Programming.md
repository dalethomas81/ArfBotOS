# Firmware Installation

The **AR4 robot** and **EasyCAT IO** (`IoOption2`) are the same hardware: a Teensy 4.1 plus a Microchip EVB-LAN9252-SPI. They need **different Arduino firmware**. Both EasyCAT boards then get the same 32+32 EEPROM image; station aliases (`1010` vs `1030`) distinguish them on EtherCAT.

| Role | Arduino sketch | EasyCAT EEPROM | Station alias |
| ---- | -------------- | -------------- | ------------- |
| AR4 robot | `Robots/AR4/Arduino/ArfBot/ArfBot.ino` | `EasyCAT_32_32_rev_1.bin` | 1010 |
| EasyCAT IO | `Robots/AR4/Arduino/RemoteIO/RemoteIO.ino` | `EasyCAT_32_32_rev_1.bin` | 1030 |

Do not flash `ArfBot.ino` onto the IO Teensy, or `RemoteIO.ino` onto the robot Teensy.

## Arduino (AR4)

1. Install Arduino according to the instructions in the Wiki located [here](https://github.com/dalethomas81/ArfBotOS/wiki/Installation#installing-arduino).
2. Open the ArfBot.ino Arduino project file located here in the ArfBotOS repo `ArfBotOS\Robots\AR4\Arduino\ArfBot`.
3. Connect the **robot** Teensy 4.1 to your computer with a USB cable.
4. In the Arduino IDE, select the Teensy 4.1 board in the top drop-down.
5. Upload the firmware using the *Upload* button (arrow pointing right).  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/Arduino-Upload.png" alt="menu" width="400"/>

## Arduino (EasyCAT IO)

Same IDE setup as above. Open `ArfBotOS\Robots\AR4\Arduino\RemoteIO\RemoteIO.ino`, connect the **IO** Teensy 4.1, and upload. This sketch maps digital inputs and outputs; it does not drive motors or read encoders.

## EasyCAT EEPROM

Repeat this on **each** EVB-LAN9252-SPI (the robot board and the IO board). The `.bin` is the same; do not skip the IO board.

1. Clone the [EasyCAT](https://github.com/dslemusp/EasyCAT) repo.
2. Open the Easy Configurator located here in the repo `EasyCAT\configurator\Exe\EasyCAT_Config_GUI.exe`.
3. With the Microchip EVB-LAN9252-SPI powered, connect an ethernet cable from your computer to the `In` port (left side).  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EtherCAT-Arduino-Firmware-Flash.png" alt="menu" width="600"/>
4. Click on *Write EEPROM* and select the 32 input by 32 output configuration file located here in the repo `EasyCAT\configurator\StandardMode_BinFiles\EasyCAT_32_32_rev_1.bin`.
5. Wait for programming to be complete. if programming fails, refer to troubleshooting at www.bausano.net  
<img src="https://github.com/dalethomas81/ArfBotOS/blob/main/Robots/AR4/Media/EasyCAT-Confirm.png" alt="menu" width="600"/>  

Do **not** right-click Disable unused IO. `IoOption1` (EK1100 / EL1809 / EL2809) and `IoOption2` (EasyCAT IO) stay in the tree as Optional slaves. Assign station aliases once, then you can hot-swap IO without a recompile. That procedure is in the wiki: [Installation — Station aliases](https://github.com/dalethomas81/ArfBotOS/wiki/Installation#station-aliases).  
 