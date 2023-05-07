import os
import select
import cv2
import numpy as np
import ffmpeg
import rospy
from sensor_msgs.msg import CompressedImage

# Set the ROS topic for the input H264 stream
input_topic = '/encoded_video'

# Create a ROS subscriber to receive the H264 stream
rospy.init_node('h264_decoder')
frame_width = 640
frame_height = 480
frame_rate = 15

video_options = {
    'analyzeduration': '100000',
    'probesize': '300000',
    'fflags': 'nobuffer'}

process = ffmpeg.input('pipe:', format='mpegts', **video_options).output('-', format='rawvideo', pix_fmt='bgr24').run_async(pipe_stdin=True, pipe_stdout=True)

os.set_blocking(process.stdout.fileno(), False)

raw_frame = bytes()

# Define the callback function to receive the H264 stream
def callback(msg):
    print('func start')

    # Decode the compressed H264 stream using FFmpeg
    input_bytes = bytes(msg.data)

    # Write the compressed H264 stream to FFmpeg's input pipe
    process.stdin.write(input_bytes)

    global raw_frame

    # Read the decoded video frames from FFmpeg
    new_frame = process.stdout.read(frame_width*frame_height*3-len(raw_frame))
    if new_frame is not None:
        raw_frame += new_frame

    print('func mid')

    if len(raw_frame) < frame_width*frame_height*3:
        return

    raw_frame = bytes(raw_frame)

    # Convert the raw video frame bytes to a NumPy array
    frame = (
        np
        .frombuffer(raw_frame, np.uint8)
        .reshape([frame_height, frame_width, 3])
    )

    raw_frame = bytes()
    print('func end')

rospy.Subscriber(input_topic, CompressedImage, callback)

# Enter the ROS spin loop
rospy.spin()

