#!/usr/bin/env bash

# since this command usually requires to be run as sudo
# this file was changed to be owned by root so it can be called
# from other scripts without needing to run the initial script as sudo
# this was achieved with `sudo chown root:root start_stream.sh`

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

# for inspiron 15 7000 gaming laptop built in webcam resolution: 1280/720 (848/480 works too), for our c920 cameras: 640/480
/usr/local/bin/mjpg_streamer  -i "input_uvc.so -r 640x480 -m 50000 -n -f 1 -d $port" -o "output_http.so -p 8090 -w /usr/local/share/mjpg-streamer/www/"

# to install dependencies:
# $ git clone https://github.com/jacksonliam/mjpg-streamer.git
# $ cd mjpg-streamer/mjpg-streamer-experimental
# $ sudo apt-get install cmake libjpeg62-dev
# $ make
# $ sudo make install

# NOTE: using mjpeg streams limits us to capturing them using more limited img tags
# instead of the newer video tags
