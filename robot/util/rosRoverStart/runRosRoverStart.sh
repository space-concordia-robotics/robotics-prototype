#!/usr/bin/env bash
ROS_ROVER_START_DIR="/home/odroid/Programming/robotics-prototype/robot/util/rosRoverStart"
ROS_ROVER_START_SH="$ROS_ROVER_START_DIR/rosRoverStart.sh"
ROS_ROVER_START_LOG="$ROS_ROVER_START_DIR/rosRoverStart.log"
bash $ROS_ROVER_START_SH > $ROS_ROVER_START_LOG
