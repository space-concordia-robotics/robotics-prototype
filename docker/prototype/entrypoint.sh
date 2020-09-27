#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/spaceuser/robotics-prototype/robot/rospackages/devel/setup.bash"
/home/spaceuser/robotics-prototype/robot/basestation/env.sh >| /home/spaceuser/robotics-prototype/robot/basestation/static/js/env.js

python3 /home/spaceuser/robotics-prototype/robot/basestation/app.py
