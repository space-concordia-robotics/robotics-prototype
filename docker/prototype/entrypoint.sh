#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/robot/rospackages/devel/setup.bash"
/robot/basestation/env.sh >| /robot/basestation/static/js/env.js

python3 /robot/basestation/app.py
