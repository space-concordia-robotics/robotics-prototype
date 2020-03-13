#!/usr/bin/env bash

# since only one person can run the flask server at a time
# get current flask server host IP address from ifconfig, filter out only the IP address
HOST_IP=`ifconfig | grep "inet addr:" | head -1 | grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" | head -1`

ROS_MASTER_IP=`echo $ROS_MASTER_URI | grep -E -o "//.*:"`
ROS_MASTER_IP=`echo ${ROS_MASTER_IP:2:${#ROS_MASTER_IP}-3}`

# if string is empty, set to localhost by default
if [ -z "$HOST_IP" ] ;
then
    HOST_IP="localhost"
fi

echo "env = {"
echo "  HOST_IP: '$HOST_IP',"
echo "  ROS_MASTER_IP: '$ROS_MASTER_IP'"
echo "}"
