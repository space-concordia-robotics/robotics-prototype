#!/bin/bash

# ffmpeg -threads 1 -f v4l2 -i /dev/video$1 -preset ultrafast -tune zerolatency -vcodec libx264 -probesize 32 -s 1920x1080 -r 20 -g 50 -f mpegts udp://192.168.0.119:4000$1

ffmpeg -threads 1 -f v4l2 -i /dev/video$1 -preset ultrafast -tune zerolatency -vcodec libx264 -probesize 32 -s 320x240 -r 20 -g 50 -f mpegts udp://192.168.0.100:4000$1
