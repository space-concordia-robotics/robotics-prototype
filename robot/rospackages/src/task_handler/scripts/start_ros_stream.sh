#!/usr/bin/env bash
DEVICES="$(v4l2-ctl --list-devices)"

echo "$DEVICES" > tmp


if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
    port=`tail -1 tmp`
    port="$(echo -e "${port}" | sed -e 's/^[[:space:]]*//')"
    echo "port: $port"
  else
    echo "Port argument supplied: $1"
    port=$1
fi


rosrun cv_camera cv_camera_node _device_path:=$port
