#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
/home/spaceuser/Programming/robotics-prototype/robot/basestation/env.sh >| /home/spaceuser/Programming/robotics-prototype/robot/basestation/static/js/env.js

python3 /home/spaceuser/Programming/robotics-prototype/robot/basestation/app.py
