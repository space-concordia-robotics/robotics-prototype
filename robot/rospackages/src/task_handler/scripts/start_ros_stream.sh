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

# These results are from a test which was run with 3 logitech c920
# all connected through the same usb hub to the 2.0 hub port of the odroid xu4
# Last test done on: 26/12/2019
# +---------------------------------+--------------------------+
# |resolution --> (width x height)  | max simultaneous streams |
# |---------------------------------|--------------------------|
# |240p  --> 352x240                | 3                        |
# |360p  --> 480x360                | 2                        |
# |480p  --> 858x480, SD            | 1                        |
# |720p  --> 1280x720, HD           | 1                        |
# |1080p --> 1920x1080, full HD     | 0                        |
# +---------------------------------+--------------------------+

roslaunch task_handler start_ros_stream.launch _device_path:=$port _image_width:=480 _image_height:=360 __name:=$topicName
#rosrun cv_camera cv_camera_node _device_path:=$port _image_width:=480 _image_height:=360 __name:=$topicName 
#roslaunch camera_vision autonomy.launch cv_camera_node_name:=$topicName 
