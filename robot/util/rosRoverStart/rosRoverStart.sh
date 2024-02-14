#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/Programming/robotics-prototype"
ETHERNET_CONFIG_STATUS="$HOME/configEthernet/status_done"
OPT_FOXY_SETUP="/opt/ros/foxy/setup.bash" # ros2 system default workspace
ROS_PACKAGES_SETUP="$REPO_HOME/robot/rospackages/install/local_setup.sh"
ROSLAUNCH_FILE="$REPO_HOME/robot/util/rosRoverStart/rover.xml"

# source venv
cd $REPO_HOME

source "$REPO_HOME/venv/bin/activate"

python3 "$REPO_HOME/setup.py" develop
PYTHONPATH=$PYTHONPATH:/home/$USER/Programming/robotics-prototype

# source primary catkin_ws setup bash script and execute one launch script to rule them all
source $OPT_FOXY_SETUP && source $ROS_PACKAGES_SETUP && ros2 launch $ROSLAUNCH_FILE
