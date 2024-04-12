# Notes

## TODO

section in wiki on arduino

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

## Handy commands
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
- `scp -r <local directory> user@host.host:<remote>` copy local files to remote
- `sudo systemctl stop bluetooth.service` self explanatory
- `sudo ufw allow 9999` open a port
- `sudo apt-get install ufw`
- `sudo ufw enable `