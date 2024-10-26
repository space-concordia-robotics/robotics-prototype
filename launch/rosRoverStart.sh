#!/usr/bin/env bash
HOME="/home/nvidia"
REPO_HOME="$HOME/Programming/robotics-prototype"
OPT_HUMBLE_SETUP="/opt/ros/humble/setup.bash"
ROS_PACKAGES_SETUP="$REPO_HOME/robot/rospackages/install/local_setup.sh"
ROSLAUNCH_FILE="$REPO_HOME/launch/robot_ik.py"
CANBUS_SETUP_FILE="$REPO_HOME/scripts/configure-can0.sh"
ARM_CONFIGURE_FILE="$REPO_HOME/scripts/configure-arm.sh"

# source venv
cd $REPO_HOME

#source "$REPO_HOME/venv/bin/activate"
source "$HOME/.bashrc"

# python3 "$REPO_HOME/setup.py" develop
PYTHONPATH=$PYTHONPATH:/home/$USER/robotics-orin

# Setup canbus
source $CANBUS_SETUP_FILE
# Setup arm
source $ARM_CONFIGURE_FILE

# source primary catkin_ws setup bash script and execute one launch script to rule them all
source $OPT_HUMBLE_SETUP && source $ROS_PACKAGES_SETUP && ros2 launch $ROSLAUNCH_FILE
