#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  laser"
echo "ls01d usb cp210x connection as /dev/laser , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy 99-robot-usb-serial.rules to  /etc/udev/rules.d/"
echo "`rospack find bringup`/etc.udev.rules.d/99-robot-usb-serial.rules"
sudo cp /home/robot1/test/src/bringup/shell/etc.udev.rules.d/99-robot-usb-serial.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo " "
sudo service udev reload
sudo service udev restart
echo "finish !"
