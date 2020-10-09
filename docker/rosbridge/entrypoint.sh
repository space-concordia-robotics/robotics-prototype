#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/spaceuser/Programming/robotics-prototype/robot/rospackages/devel/setup.bash"

roslaunch --wait rosbridge_server rosbridge_websocket.launch
