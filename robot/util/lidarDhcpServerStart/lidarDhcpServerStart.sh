#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/Programming/robotics-prototype"

sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eth0 --bind-dynamic