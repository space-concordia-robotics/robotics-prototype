#!/usr/bin/env bash
# This script checks if the internet can be reached from a regular ethernet connection type
# If the regular ethernet configuration cannot ping google severs, the rocket configuration is loaded

STATUS_FILE="/home/odroid/configEthernet/status_done"

# "I'm working"
echo 0 > $STATUS_FILE

IS_ETHERNET_PLUGGED=(`cat /sys/class/net/eth0/carrier`)

echo "IS_ETHERNET_PLUGGED: $IS_ETHERNET_PLUGGED"

# Network SSID for regular router and rocketM900, respectively
ROUTER_CONNECTION="WiredConnection1"
ROCKET_CONNECTION="RoverOBC"

# For testing if we have an internet connection
GOOGLE_SERVER="8.8.8.8"

# exit script if ethernet cable not plugged in
if [ "$IS_ETHERNET_PLUGGED" -eq "0" ]; then
    echo "No ethernet cable plugged, exiting script."
    exit 1
fi

# switch to WiredConnection1
nmcli connection up $ROUTER_CONNECTION

# if router connection does not provide internet access, switch to rocket connection
if ping -c 1 -q $GOOGLE_SERVER &>/dev/null; then
    echo "Network is reachable, regular router configuration selected"
else
    echo "Network is unreachable, Rocket configuration selected"

    nmcli connection up $ROCKET_CONNECTION
fi

# signal done working for other services
echo 1 > $STATUS_FILE
