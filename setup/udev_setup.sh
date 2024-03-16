#!/bin/bash
echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP="dialout", SYMLINK+="wit_imu"' >/etc/udev/rules.d/wit_imu.rules

service udev reload
sleep 2
service udev restart
