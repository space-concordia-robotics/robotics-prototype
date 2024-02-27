#! /usr/bin/bash


busybox devmem 0x0c303018  w 0xc458
busybox devmem 0x0c303010  w 0xc400


sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 up type can bitrate 1000000

