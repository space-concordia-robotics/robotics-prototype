#!/usr/bin/env python3

import ffmpeg
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

rospy.init_node('video_encoder')

device_path = rospy.get_param('~device_path', '/dev/video1')

input_format = 'v4l2'
resolution = '640x480'

codec = 'libx264'
bitrate = '15K'
profile = 'baseline'
trellis = '0'
subq = '1'
level = '32'
preset = 'veryfast'
tune = 'zerolatency'
crf = '30'
threads = '0'
bufsize = '1'
refs = '4'
coder = '0'
b_strategy = '0'
bf = '0'
sc_threshold = '0'
x264_params = 'vbv-maxrate=2000:slice-max-size=1500:keyint=30:min-keyint=10:'

output_format = 'mpegts'
pixel_format = 'yuv420p'

input_stream = ffmpeg.input(device_path, format=input_format, s=resolution)

video_options = {
    'c:v': codec,
    'b:v': bitrate,
    'profile:v': profile,
    'trellis': trellis,
    'subq': subq,
    'level': level,
    'preset': preset,
    'tune': tune,
    'crf': crf,
    'threads': threads,
    'bufsize': bufsize,
    'refs': refs,
    'coder': coder,
    'b_strategy': b_strategy,
    'bf': bf,
    'sc_threshold': sc_threshold,
    'x264-params': x264_params,
    'pix_fmt': pixel_format,
}

output_stream = ffmpeg.output(input_stream.video, '-', format=output_format, **video_options)

process = ffmpeg.run_async(output_stream, pipe_stdout=True, pipe_stderr=True)
video_publisher = rospy.Publisher('encoded_video', CompressedImage, queue_size=1)

while not rospy.is_shutdown():
    encoded_video = process.stdout.read(10000) 
    if not encoded_video:
        break

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'h264'
    msg.data = encoded_video
    video_publisher.publish(msg)

process.communicate()
video_publisher.unregister()
