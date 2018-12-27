#!/usr/bin/env bash
# This script checks if the internet can be reached from a regular ethernet connection type
# If the regular ethernet configuration cannot ping google severs, the rocket configuration is loaded

IS_ETHERNET_PLUGGED=(`cat /sys/class/net/eth0/carrier`)

# Network SSID for regular router and rocketM900, respectively
ROUTER_CONNECTION="WiredConnection1"
ROCKET_CONNECTION="RoverOBC"

# For testing if we have an internet connection
GOOGLE_SERVER="8.8.8.8"

#echo $IS_ETHERNET_PLUGGED

# exit script if ethernet cable not plugged in
if [ "$IS_ETHERNET_PLUGGED" -eq "0" ]; then
    echo "No ethernet cable plugged, exiting script."
    exit 1
fi

# switch to WiredConnection1
nmcli connection up $ROUTER_CONNECTION

pingResults=(`ping -c 1 $GOOGLE_SERVER`)

# if router connection does not provide internet access, switch to rocket connection
if [[ $pingResults == *"Network is unreachable"* ]]; then
    echo "Network is unreachable, selecting Rocket configuration"
    nmcli connection up $ROCKET_CONNECTION
fi
