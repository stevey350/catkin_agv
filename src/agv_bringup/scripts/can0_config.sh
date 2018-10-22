#!/bin/bash

sudo modprobe can
sudo modprobe can-raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000
sudo ifconfig can0 up

#sudo ifconfig can0 down
