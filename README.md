# ArfBotOS
 Operating system for a 6 axis robot running on CoDeSys
 
## Installation
Needed:
	-development pc running Windows
	-Raspberry Pi 4 with 4gb ram or greater
	-16gb micro sd card or larger (smaller makes backups easier)
	-sd card to usb adapter (or other solution for connecting sd card to pc)

### Imagining the Disk:
1. install/open raspberry pi imager.
	-https://www.raspberrypi.com/software/
2. insert sd card into dev pc
3. choose raspberry pi device from dropdown (raspberry pi 4)
4. choose operating system from dropdown (must be raspberry pi os 32-bit to make use of codesys multi-core)
5. choose storage (sd card you inserted earlier)
6. press `ctrl + shift + x` to open advanced settings (depending on the version of raspberry pi imager you may get to this by choosing next and 'edit settings' from the prompt)
7. on the GENERAL tab set the hostname to `ArfBot` (recommended but not required).
8. set username and password of the default user (recommend `ArfBot`).
9. set the SSID and password of your wifi network.
10. on the SERVICES tab select 'Enable SSH' and 'Use password authentication' (this will allow connecting remotely to ArfBotOS via command line).
11. click SAVE (then YES if you were prompted to apply OS customization settings).
12. follow the remaining prompts to start writing to the disk. this will take a bit of time.

### Booting Up and Connecting SSH
1. insert sd card into raspberry pi and power it up. if the settings entered in the raspberry pi imager are correct, the pi should connect to your wifi already. if not begin troubleshooting there.
2. open up a command prompt on your development pc and type `ssh ArfBot@ArfBot.local`.
3. after the security prompt and entering your password you should be connected.