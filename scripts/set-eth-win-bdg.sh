#!/bin/bash

# Enables the Jetson to access Internet shared from a Windows computer
# Setup: Windows computer connected to Internet and directly connected
# to Jetson ETH port. Windows configured to share Internet to this port.
# The ETH port on Windows side has default IP address 192.168.137.1.

sudo ifconfig eth0 192.168.137.2 netmask 255.255.255.0
echo "IP address on ETH0 set to 192.168.137.2"

sudo route add default gw 192.168.137.1
echo "Routing table updated (volatile)"
