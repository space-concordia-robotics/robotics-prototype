import ffmpeg
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

# Initialize the ROS node
rospy.init_node('video_encoder')

# Get the device path from a ROS parameter
device_path = rospy.get_param('~device_path', '/dev/video0')

# Set the video input format and resolution
input_format = 'v4l2'
resolution = '640x480'

# Set the video codec options
codec = 'libx264'
bitrate = '15K'
profile = 'baseline'
trellis = '0'
subq = '1'
level = '32'
preset = 'superfast'
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

# Set the output format and pixel format
output_format = 'mpegts'
pixel_format = 'yuv420p'

# Define the input video stream
input_stream = ffmpeg.input(device_path, format=input_format, s=resolution)

# Define the video encoding options
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

# Define the output video stream
output_stream = ffmpeg.output(input_stream.video, '-', format=output_format, **video_options)

# Start the encoding process and create a video publisher
process = ffmpeg.run_async(output_stream, pipe_stdout=True)
video_publisher = rospy.Publisher('encoded_video', CompressedImage, queue_size=None)

# Continuously read encoded video frames and publish them over ROS
while not rospy.is_shutdown():
    encoded_video = process.stdout.read(1000) 
    if not encoded_video:
        break

    # Create a CompressedImage message and publish it
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'h264'
    msg.data = encoded_video
    video_publisher.publish(msg)

# Cleanup the resources
process.communicate()
video_publisher.unregister()
