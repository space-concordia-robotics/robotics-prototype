#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/Programming/robotics-prototype"

ETHERNET_INTERFACE="eth1"

sudo ip addr flush dev $ETHERNET_INTERFACE
sudo ip addr add 10.5.5.1/24 dev $ETHERNET_INTERFACE
sudo ip link set $ETHERNET_INTERFACE up

sudo systemctl stop dnsmasq
sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i $ETHERNET_INTERFACE --bind-dynamic
