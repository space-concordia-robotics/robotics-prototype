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
# the image width & image height are hard coded to meet the requirements of ERC 2K19
# the name of then node can't contain "/" so only use the suffix, i.e. remove preceding "/dev/"
# so in local mode you will get topic names like 'video0Cam', 'video1Cam', and so on
# in competition mode you will get specifically 'FrontCam', 'RearCam', and 'ArmScienceCam'
topicName="${port:5}Cam"
rosrun cv_camera cv_camera_node _device_path:=$port _image_width:=1280 _image_height:=720 __name:=$topicName
