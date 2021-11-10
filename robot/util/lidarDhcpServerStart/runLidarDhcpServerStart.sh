#!/usr/bin/env bash
ROS_ROVER_START_DIR="/home/nvidia/Programming/robotics-prototype/robot/util/lidarDhcpServerStart"
ROS_ROVER_START_SH="$ROS_ROVER_START_DIR/lidarDhcpServerStart.sh"
ROS_ROVER_START_LOG="$ROS_ROVER_START_DIR/lidarDhcpServerStart.log"
bash $ROS_ROVER_START_SH > $ROS_ROVER_START_LOG
