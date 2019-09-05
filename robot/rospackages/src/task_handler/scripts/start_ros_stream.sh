#!/usr/bin/env bash

# get list of available devices
DEVICES="$(v4l2-ctl --list-devices)"

# save output in a temp file
echo "$DEVICES" > tmp

# if no arguments supplied
if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
    # read last line (contains "first" assigned webcam port
    # for example on most laptops: '/dev/video0', on odroid: '/dev/video7'
    port=`tail -1 tmp`
    # filter out everything else except the port value
    port="$(echo -e "${port}" | sed -e 's/^[[:space:]]*//')"
    echo "port: $port"
  else
    # use supplied argument
    # TODO: cross check at this point if the provided port
    # is in the list of available ports
    echo "Port argument supplied: $1"
    port=$1
fi

# run the cv_camera_node
# web_video_server node must be running in background for this to work
rosrun cv_camera cv_camera_node _device_path:=$port
