#!/bin/sh
echo "Starting start_can.sh"
insmod /usr/lib/modules/3.18.20/kernel/drivers/net/can/can-dev.ko
sleep 1
insmod /usr/lib/modules/3.18.20/kernel/drivers/net/can/spi/mcp251x.ko
sleep 1
drv="/home/root/can_iot.ko"
# remove the driver
rmmod $drv
# Enable level shifter on the CAN IoT card by driving IoT GPIO2 high
echo 4 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio4/direction
echo 1 >/sys/class/gpio/gpio4/value
# Take IoT card out of reset
echo 7 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio7/direction
echo 1 >/sys/class/gpio/gpio7/value
# Give a bit of time for the IoT card to come out of reset before loading drivers
sleep 1
# Bring driver back & iproute2 add in CAN
insmod $drv
/sbin/ip link set can0 type can bitrate 500000 triple-sampling on
/sbin/ifconfig can0 txqueuelen 5000
/sbin/ifconfig can0 up
sleep 1
ip -details -statistics link show can0
