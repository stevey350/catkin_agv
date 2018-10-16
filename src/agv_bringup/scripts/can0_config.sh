#!/bin/bash

sudo ip link set can0 type can bitrate 500000
sudo ifconfig can0 up

#sudo ifconfig can0 down
