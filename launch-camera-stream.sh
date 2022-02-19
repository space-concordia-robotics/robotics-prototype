#!/usr/bin/env bash

DEVICE=$1
PORT=$2

ffmpeg -f v4l2 -input_format yuyv422 -s 640x480 -i $DEVICE -c:v libx264 -profile:v baseline -trellis 0 -subq 1 -level 32 -preset superfast -tune zerolatency -crf 30 -threads 0 -bufsize 1 -refs 4 -coder 0 -b_strategy 0 -bf 0 -sc_threshold 0 -x264-params vbv-maxrate=2000:slice-max-size=1500:keyint=30:min-keyint=10: -pix_fmt yuv420p -an -f mpegts udp://192.168.0.135:$PORT
