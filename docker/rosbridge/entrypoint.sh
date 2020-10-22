#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/spaceuser/Programming/robotics-prototype/robot/rospackages/devel/setup.bash"

# Not recommended but doable. See: https://docs.docker.com/config/containers/multi-service_container/
roslaunch --wait robot/util/rosRoverStart/local_rover.launch & 

roslaunch --wait rosbridge_server rosbridge_websocket.launch
