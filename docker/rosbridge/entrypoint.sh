#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
roslaunch rosbridge_server rosbridge_websocket.launch
