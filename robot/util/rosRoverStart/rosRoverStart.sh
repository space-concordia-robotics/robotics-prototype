#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/Programming/robotics-prototype"
ETHERNET_CONFIG_STATUS="$HOME/configEthernet/status_done"
#CATKIN_WS_SETUP="$HOME/catkin_ws/devel/setup.bash" # catkin ws only for isolated prototyping
OPT_MELODIC_SETUP="/opt/ros/melodic/setup.bash" # ros system default workspace
ROS_PACKAGES_SETUP="$REPO_HOME/robot/rospackages/devel/setup.bash"
ROSLAUNCH_FILE="$REPO_HOME/robot/util/rosRoverStart/rover.launch"
# wait for connection to configure itself via config-ethernet.service
while [ `cat $ETHERNET_CONFIG_STATUS` == "0" ]
do
    echo "Waiting for config-ethernet.service, retrying in 1 second"
    sleep 1
done

# source venv
cd $REPO_HOME

source "$REPO_HOME/venv/bin/activate"

python3 "$REPO_HOME/setup.py" develop

# source primary catkin_ws setup bash script and execute one launch script to rule them all
source OPT_MELODIC_SETUP && source $ROS_PACKAGES_SETUP && roslaunch $ROSLAUNCH_FILE
