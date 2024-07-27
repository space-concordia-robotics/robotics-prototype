#!/usr/bin/env bash

ROS_ROVER_START_DIR="/home/nvidia/Programming/robotics-orin/launch"
ROS_ROVER_START_SH="$ROS_ROVER_START_DIR/rosRoverStart.sh"
ROS_ROVER_START_LOG="$ROS_ROVER_START_DIR/rosRoverStart.log"
bash $ROS_ROVER_START_SH > $ROS_ROVER_START_LOG
