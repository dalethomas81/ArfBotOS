# ArfBotOS
ArfBotOS is an operating system for a 6 axis robot running on CoDeSys, Arduino, and OpenCV. One of the main intentions of ArfBotOS is to give an example of how a typical industrial control system is programmed in IEC-61131. This wil give good experience to new Automation and Controls engineers.
 
Disclaimer: The cost of the hardware in this project has been kept at a minimum to make it more attainable. However, there is a Codesys licensing cost associated with this project but the runtime will run in demo mode for 2 hours if you do not purchase the license. (You can restart the runtime to restart the 2 hour demo mode). The licensing cost approx $695.00 USD and this will get you multi-core support on the Raspberry Pi as well as CNC/Robotics motion control.
 
## Raspberry Pi OS Installation and Configuration
#### Needed:
- development pc running Windows
- Raspberry Pi 4 with 4gb ram or greater
- 16gb micro sd card or larger (smaller makes backups easier)
- sd card to usb adapter (or other solution for connecting sd card to pc)

### Imagining the Disk:
1. install/open raspberry pi imager. https://www.raspberrypi.com/software/
2. insert sd card into dev pc
3. choose raspberry pi device from dropdown (raspberry pi 4)
4. choose operating system from dropdown (must be raspberry pi os 32-bit to make use of codesys multi-core) Note: choosing the headless option is recommended.
5. choose storage (sd card you inserted earlier)
6. press `ctrl + shift + x` to open advanced settings (depending on the version of raspberry pi imager you may get to this by choosing next and 'edit settings' from the prompt)
7. on the GENERAL tab set the hostname to 'ArfBot' (recommended but not required).
8. set username and password of the default user (recommend 'ArfBot').
9. set the SSID and password of your wifi network.
10. on the SERVICES tab select 'Enable SSH' and 'Use password authentication' (this will allow connecting remotely to ArfBotOS via command line).
11. click SAVE (then YES if you were prompted to apply OS customization settings).
12. follow the remaining prompts to start writing to the disk. this will take a bit of time.

### Booting Up and Connecting SSH
1. insert sd card into raspberry pi and power it up. if the settings entered in the raspberry pi imager are correct, the pi should connect to your wifi already. if not begin troubleshooting there.
2. open up a command prompt on your development pc and type `ssh ArfBot@ArfBot.local`.
3. after the security prompt and entering your password you should be connected.

### Backing Up The SD Card
1. plug a new sd card into pi. (make sure card is identical size and brand or make it larger if differnt brand)
2. run this command: `sudo dd bs=4M if=/dev/mmcblk0 of=/dev/sda`
3. it will take a long time. just wait. will see activity lights on pi.
4. boot with new card to test.

#### references:
- https://raspberrystreet.com/learn/how-to-backup-raspberrypi-sdcard

### Configuring i2c
1. from an SSH session install i2c tools `sudo apt install i2c-tools`
2. configure the boot config file `sudo nano /boot/firmware/config.txt`
3. add this line to the config file to enable the second 12c port `dtoverlay=i2c-gpio,bus=2,i2c_gpio_sda=22,i2c_gpio_scl=23`
4. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.  
5. to enable i2c type `sudo raspi-config` into command line. navigate to 'Interface Options'->'I2C' and select 'Yes' then 'Ok' then 'Finish'
6. if you would like to reboot now you can type `sudo reboot now`. otherwise you will need to reboot later for i2c to work.

#### notes: 
- if you would like to enable the second i2c port temporarily you can enter this in the command line. it will persist until a restart of the os. `dtoverlay i2c-gpio bus=2 i2c_gpio_sda=22 i2c_gpio_scl=23`  

#### references: 
- https://medium.com/cemac/creating-multiple-i2c-ports-on-a-raspberry-pi-e31ce72a3eb2

### Configuring Arducam
1. from an SSH session open the boot config file `sudo nano /boot/firmware/config.txt`
2. add these lines to the config file `camera_auto_detect=0` and `dtoverlay=imx477`
3. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.

#### references: 
- https://www.arducam.com/product/12-3mp-imx477-hq-camera-module-b024001/
- https://www.uctronics.com/download/Amazon/B024001_Manual.pdf
- https://content.helpme-codesys.com/en/CODESYS%20Control/_rtsl_linux_rbp_examples_camera.html

### Installing OpenCV
1. from an SSH seesion run this command to install OpenCV `sudo apt-get install python3-opencv`
2. open the raspberry pi configuration with this command `sudo raspi-config`
3. verify installation and version by typing the following commands. `python` `import cv2 as cv` `print(cv.__version__)`
TODO: is this command needed? `pip install "picamera[array]"`

#### references:
- https://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/template_matching/template_matching.html
- https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
- https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
- https://github.com/cozheyuanzhangde/Invariant-TemplateMatching

### Install Vision Command Service
1. run this command to make the service file and edit in nano `sudo nano /etc/systemd/system/PyServer.service`
2. copy and paste the following into the file:
```
[Unit]
Description=Runs Python Scripts Sent By Client
After=multi-user.target

[Service]
Type=simple
Restart=always
StandardOutput=append:/var/opt/codesys/PlcLogic/Application/Vision/VisionLog.txt
StandardError=append:/var/opt/codesys/PlcLogic/Application/Vision/VisionErrorLog.txt
ExecStart=/usr/bin/python /var/opt/codesys/PlcLogic/Application/Vision/PyServer.py

[Install]
WantedBy=multi-user.target
```
3. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.
4. run the following commands to start the service. (note if you are following along the installation in chronological order, then we have not installed the python scripts yet and as such the service will fail. we will install them later but we want to enable the service here at least)
- `sudo systemctl daemon-reload`
- `sudo systemctl enable PyServer.service`
- `sudo systemctl start PyServer.service` (optional)
- `sudo systemctl status PyServer.service` (optional - will see failed here if script not installed)

#### references:
- https://medium.com/codex/setup-a-python-script-as-a-service-through-systemctl-systemd-f0cc55a42267
- https://www.freedesktop.org/software/systemd/man/systemd.service.html
- https://stackoverflow.com/questions/37585758/how-to-redirect-output-of-systemd-service-to-a-file

### Install Vision Web Service
1. run the following commands to install flask:
- `sudo pip install flask --break-system-packages` TODO: install this in a virtual environment instead.
- `sudo pip install flask-wtf --break-system-packages` TODO: install this in a virtual environment instead.
- `sudo pip install flask-sqlalchemy` (optional)
- `sudo pip install flask-migrate` (optional)
2. run this command to make the service file and edit in nano `sudo nano /etc/systemd/system/VisionWeb.service`
3. copy and paste the following into the file:
```
[Unit]
Description=Hosts a web server for the vision application
After=multi-user.target

[Service]
Type=simple
Restart=always
Environment=FLASK_APP=vision.py
WorkingDirectory=/var/opt/codesys/PlcLogic/Application/Vision/VisionWebServer
ExecStart=/usr/bin/flask run -h arfbot.local -p 5000

[Install]
WantedBy=multi-user.target
```
4. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.
5. run the following commands to start the service. (note if you are following along the installation in chronological order, then we have not installed the python scripts yet and as such the service will fail. we will install them later but we want to enable the service here at least)
- `sudo systemctl daemon-reload`
- `sudo systemctl enable VisionWeb.service`
- `sudo systemctl start VisionWeb.service` (optional)
- `sudo systemctl status VisionWeb.service` (optional - will see failed here if script not installed)

#### references:
- https://blog.miguelgrinberg.com/post/the-flask-mega-tutorial-part-i-hello-world
- https://blog.miguelgrinberg.com/post/running-a-flask-application-as-a-service-with-systemd

### Install Controller Service
1. run the following command to clone pydualsense to your raspberry pi: `git clone https://github.com/flok/pydualsense` (this will make the next step easier)
2. run the following commands to install pydualsense:
- `sudo cp pydualsense/70-ps5-controller.rules /etc/udev/rules.d` (note the path to the rules file is where you cloned the repo in step 1)
- `sudo udevadm control --reload-rules`
- `sudo udevadm trigger`
- `sudo apt install libhidapi-dev`
- `sudo pip install --upgrade pydualsense --break-system-packages` TODO: install this in a virtual environment instead.
3. run this command to make the service file and edit in nano `sudo nano /etc/systemd/system/DualSenseController.service`
4. copy and paste the following into the file:
```
[Unit]
Description=Hosts a socket server for connecting to a Playstation DualSense controller
After=multi-user.target

[Service]
Type=simple
Restart=always
StandardOutput=append:/var/opt/codesys/PlcLogic/Application/Controller/ControllerLog.txt
StandardError=append:/var/opt/codesys/PlcLogic/Application/Controller/ControllerErrorLog.txt
ExecStart=/usr/bin/python /var/opt/codesys/PlcLogic/Application/Controller/DualSenseServer.py

[Install]
WantedBy=multi-user.target
```
5. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.
6. run the following commands to start the service. (note if you are following along the installation in chronological order, then we have not installed the python scripts yet and as such the service will fail. we will install them later but we want to enable the service here at least)
- `sudo systemctl daemon-reload`
- `sudo systemctl enable DualSenseController.service`
- `sudo systemctl start DualSenseController.service` (optional)
- `sudo systemctl status DualSenseController.service` (optional - will see failed here if script not installed)

#### references:
- https://github.com/flok/pydualsense

### Connecting DualSense Playstation Controller
1. run the following command to open Bluetooth control: `bluetoothctl`
2. at the the `[bluetooth]#` prompt enter these commands:
	`pairable on`
	`agent on`
	`default-agent`
	`scan on`
3. put the DualSense controller in paring mode by holding down the `PS` and `Share` buttons at the same time until the light around the trackpad starts flashing.
4. at the `[Bluetooth]#` prompt wait for a message that looks like this
	`[NEW] Device 12:23:34:45:56:67 devicename`
5. copy the mac address (from the above message) and enter this command to pair the controller (hint: it may be handy to type `scan off` to make the messages stop while you copy the mac address)
	`pair 12:23:34:45:56:67`
6. you will see a message asking to allow control. type `yes` to finish pairing.
7. enter the following command to allow the controller to connect automatically:
	`trust 12:23:34:45:56:67`
8. enter `exit` to leave bluetooth control.

#### references:
- https://bluedot.readthedocs.io/en/latest/pairpipi.html#using-the-command-line

## Installing Codesys

### Codesys Development and Runtime
1. download Codesys dev environment 32-bit 3.5.19.40 from the Codesys store and install. https://us.store.codesys.com/codesys.html
2. download Codesys Raspberry Pi Runtime 4.10.0 from the Codesys store and install. https://us.store.codesys.com/codesys-control-for-raspberry-pi-sl.html
3. download the Codesys Softmotion library 4.13.0 from the Codesys store and install. (will need to uninstall 4.15 first - details in lower section) https://us.store.codesys.com/codesys-softmotion-sl-bundle.html
4. download the OMAC PackML State Machine library 1.0.0.1 from the Codesys store and install. https://us.store.codesys.com/omac-packml-state-machine.html
5. open Codesys dev environment and from the Tools menu choose Update Raspberry Pi.
6. from the Raspberry Pi panel enter the username and password of the user you created when you imaged the Raspberry Pi.
7. enter the ip address of your Raspberry pi.
8. select install to install the Codesys runtime onto the Raspberry Pi.
9. when prompted make sure Multicore 32-bit (ARMv7) is selected and click Close to configure the runtime. otherwise select Change.
10. when prompted to install the gateway click Yes.

### Codesys Libraries
1. From the Start menu of Windows open the Codesys Installer (optionally you can find this in Tools menu of the Codesys IDE but you will need to close Codesys to uninstall/install anyway)
2. Find Codesys Softmotion in the Installed applications and uninstall it 4.15.0.0 (this comes prepacked in 3.5.19.40 of Codesys but gives compiler errors for the Adafruit PWM drives)
3. Browse for version 4.13.0.0 and install it.

### ArfBotOS Codesys Project On The Raspberry Pi
1. open ArfBot.project in Codesys from `ArfBot\Codesys` (if prompted to update any libraries select 'Do Not Update' for each selection unless you know what you are doing)
2. from the Build menu select Generate Code to compile. (cross your fingers)
3. from the device tree, double-click on 'Device' and open the 'Communication Settings' tab.
4. type in the ip address of the Raspberry Pi and hit enter. this will establish a connection to the gateway on the controller. (note that if you are having connectivity issues here you may have a different version of the runtime installed vs the gateway. you can check this by using the 'Update Raspberry Pi' and 'Update Edge Gateway' from the Tools menu)
4. if the project built successfully and a connection to the gateway is established, open the Online menu and select 'Multiple Download...' to download the code to the controller. keep the default selections and press 'OK'
5. during download, Codesys will install necessary script files in the `/var/opt/codesys/PlcLogic/Application/Vision` directory (this is why the services we installed earlier would fail). there are 2 files and 1 directory that you will have to manually copy over to get vision working.
6. from the Codesys development environment, double-click on Device and from the Files tab find the `cal.yaml` and `roi.yaml` files located in the ArfBot repository and copy them to the `PlcLogic/Application/Vision` directory. they can be found in the `ArfBot/OpenCV/CameraCalibration` and `ArfBot/OpenCV/FastTemplateMatching` folders, respectively. these files have initial values and will be dynamically generated later.
7. next, copy the `ArfBot/OpenCV/VisionWebServer` directory over to the `/var/opt/codesys/PlcLogic/Application/Vision` directory.
8. next, create a new directory in `PlcLogic/Application/Vision` called `Templates`.

### Configuring Codesys To Run Commands In Linux
1. run this command to open the Codesys Control config file and edit in nano `sudo nano /etc/CODESYSControl.cfg`
4. copy and paste the following into the file:
```
[SysProcess]
Command=AllowAll
```
5. type `ctrl + x` then `y` then `enter` to save and close the config file in nano.
6. perform the same actions as above for the user file `sudo nano /etc/CODESYSControl_User.cfg` (note this file will already have an entry for `SysProcess`. You will need to over write it with this new entry)

### Codesys licensing
1. You will need these 4 licenses (total $695.00USD):
- [CODESYS Control Standard L](https://us.store.codesys.com/codesys-control-standard-l-bundle.html) $239.00USD (L version gives ability to assign tasks to cores)
- [CODESYS SoftMotion Axes (8)](https://us.store.codesys.com/codesys-softmotion-axes-8.html) $264.00USD
- [SoftMotion Axis Groups/CNC Interpolators (1)](https://us.store.codesys.com/softmotion-axis-groups-cnc-1.html) $192.00USD
- [CODESYS Visualization S](https://us.store.codesys.com/codesys-visualization-m.html) $0.00USD (comes with Control Standard license)

2. Codesys licenses are married to hardware and cannot be transferred. Meaning that if your Raspberry Pi dies, you will need to purchase licenses again. Optionally, you can purchase the Codesys key that will allow you to assign the license to the key and move the license around to other controllers. Note that Codesys will charge $41.99USD for shipping the key to you. [Codesys Key](https://us.store.codesys.com/codesys-key.html)

##### notes: 
- Codesys uses application specific licenses for very high granularity of features. While this has its advantages, it does make figuring out what license you need for your project difficult.

##### references: 
- https://www.codesys.com/the-system/licensing.html

### Tuning The Drives
This needs to be done or the actual vs set positions will drift.
TODO

##### references:
- https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_example_poscontrol.html

## The Robot
In theory, you can use any 6 axis robot that you would like with ArfBotOS. However, you will need to integrate the connections to the servos and limit switches. Additionally, you will need recompile ArfBotOS with the Denavit-Hartenburg (DH) parmeters for the bot of your choice. I chose to use the fantastic open source AR4 robot designed and developed by Chris Annin of [Annin Robotics](https://www.anninrobotics.com/)

Below are detailed instructions on how to modify the AR4 connections to work with ArfBotOS.
TODO

## Handy Bash commands
- `pidof <process>` find the pid of a process so you can kill it
- `sudo kill -9 <pid>` SIGINT request to stop gracefully
- `sudo kill -15 <pid>` force kill
- `df -Bm` Get size of disk
- `sudo chown -R <user> /var/opt/codesys` ownership permissions of directory/file
- `sudo netstat -nlp | grep <port number>` see which app is using a specific port
- `sudo reboot now` restart
- `journalctl -u <service name>` see logs written by system services
- `journalctl -u <service name> --since=-5m` logs in the last 5 minutes
- `journalctl -u <service name> -n 25` last 25 log entries
- `journalctl -u <service name> -f` tail the logs
- `which <executable name>` locates the path of an executable