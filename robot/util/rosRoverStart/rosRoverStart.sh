#!/usr/bin/env bash
HOME="/home/odroid"
ETHERNET_CONFIG_STATUS="$HOME/configEthernet/status_done"
CATKIN_WS_SETUP="$HOME/catkin_ws/devel/setup.bash"
ROSLAUNCH_FILE="$HOME/rosRoverStart/rover.launch"
# wait for connection to configure itself via config-ethernet.service
while [ `cat $ETHERNET_CONFIG_STATUS` == "0" ]
do
    echo "Waiting for config-ethernet.service, retrying in 1 second"
    sleep 1
done

# source primary catkin_ws setup bash script and execute one launch script to rule them all
source $CATKIN_WS_SETUP && roslaunch $ROSLAUNCH_FILE
