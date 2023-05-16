#!/bin/sh
# Script to start apps/scripts on the modem and load kernel modules
drv_isotp="/home/root/isotp.ko"
echo "Starting start_can.sh script"
/home/root/start_can.sh
sleep 2
echo "installing isotp kernel module"
insmod $drv_isotp
sleep 2
exit
