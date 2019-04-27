#!/usr/bin/env bash

# since this command usually requires to be run as sudo
# this file was changed to be owned by root so it can be called
# from other scripts without needing to run the initial script as sudo
# this was achieved with `sudo chown root:root start_stream.sh`
/usr/local/bin/mjpg_streamer  -i "input_uvc.so -r 1280x720 -m 50000 -n -f 25 -d /dev/video7" -o "output_http.so -p 8090 -w /usr/local/share/mjpg-streamer/www/"

# to install dependencies:
# $ git clone https://github.com/jacksonliam/mjpg-streamer.git
# $ cd mjpg-streamer/mjpg-streamer-experimental
# $ sudo apt-get install cmake libjpeg62-dev
# $ make
# $ sudo make install

# NOTE: using mjpeg streams limits us to capturing them using more limited img tags
# instead of the newer video tags
