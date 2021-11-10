#!/usr/bin/env bash
LIDAR_DHCP_SERVER_START_DIR="/home/nvidia/Programming/robotics-prototype/robot/util/lidarDhcpServerStart"
LIDAR_DHCP_SERVER_START_SH="$LIDAR_DHCP_SERVER_START_DIR/lidarDhcpServerStart.sh"
LIDAR_DHCP_SERVER_START_LOG="$LIDAR_DHCP_SERVER_START_DIR/lidarDhcpServerStart.log"
bash $LIDAR_DHCP_SERVER_START_SH > $LIDAR_DHCP_SERVER_START_LOG
