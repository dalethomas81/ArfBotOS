# Notes

## TODO

teensy
input 8 gpio 34
input 9 gpio 35
input 10 gpio 40
input 11 gpio 41
output 8 gpio 12
output 9 gpio 13
output 10 gpio 32
output 11 gpio 33

pi
input 0 gpio 4
input 1 gpio 5
input 2 gpio 6
input 3 gpio 13
input 4 gpio 17
input 5 gpio 22
input 6 gpio 26
input 7 gpio 27
output 0 gpio 12
output 1 gpio 16
output 2 gpio 18
output 3 gpio 20
output 4 gpio 21
output 5 gpio 23
output 6 gpio 24
output 7 gpio 25

add vision parts counts results in wiki

add counters, registers, inputs, and outputs to HMI
enabling jog opens jog hmi

controller:
	-permanent solution for pydualsense bug (copy over local for now?)
	-need to get steps put into tutorial
	-add validation steps to tutorial
	-address bug where server wont connect to controller on a fresh reboot (done)

camera
	-most likely needs calibration file
	-(maybe this is a setting in raspi-config?)
	-add validation steps to tutorial
	-verify web service

-backup sd card (after all issues are resolved with installation

-need to write values to movevelocity parameters when clicking on wrtie parameters button on tunning HMI

-address failure mode when electrical power is removed but robot stays enabled the set vs actual is off (because robot is loose and collapses). this leads to issues when restoring power. Servos try to move too fast to new position.

-add ability to copy current position in the set variable builder menu

-add enable parameter to max position difference on tuning page

-add failure bit for vision (perhaps add found number of parts to vision register?)

## Handy commands
- `pidof <process>` find the pid of a process so you can kill it
- `sudo kill -9 <pid>` SIGINT request to stop gracefully
- `sudo kill -15 <pid>` force kill
- `df -Bm` Get size of disk
- `sudo mkdir -p /var/opt/codesys` make directory and all parents
- `sudo chown -R <user> /var/opt/codesys` ownership permissions of directory/file
- `sudo rm -rf /var/opt/codesys` remove directory and all files
- `sudo netstat -nlp | grep <port number>` see which app is using a specific port
- `sudo reboot now` restart
- `journalctl -u <service name>` see logs written by system services
- `journalctl -u <service name> --since=-5m` logs in the last 5 minutes
- `journalctl -u <service name> -n 25` last 25 log entries
- `journalctl -u <service name> -f` tail the logs
- `which <executable name>` locates the path of an executable
- `scp -r <local directory> user@host.host:<remote>` copy local files to remote
- `sudo systemctl stop bluetooth.service` self explanatory
- `sudo ufw allow 9999` open a port
- `sudo apt-get install ufw`
- `sudo ufw enable`
- `raspinfo`
- `sudo apt-get install i2c-tools`
- `sudo i2cdetect -y 4` (4 is the bus to read)
- `ls /dev` (see all i2c buses)

