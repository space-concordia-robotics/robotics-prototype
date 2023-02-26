#!/bin/python3

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
import time

# Init video writter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_video = None

frame_count = 0
start_time = time.time()

# Initialize ROS node
rospy.init_node('aruco_detector')

# Set up the image subscriber
image_topic = '/cv_camera/image_raw'
bridge = CvBridge()
def image_callback(msg):
    global fourcc
    global output_video
    global frame_count
    global start_time

    # Convert ROS image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect the Aruco markers in the frame
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Draw the detected markers on the frame
    frame = aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Show the processed frame
    if output_video is not None:
        output_video.write(frame)
    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)

    frame_count += 1
    if frame_count > 10 and output_video is None:
        elapsed_time = time.time() - start_time

        framerate = frame_count / elapsed_time
        rospy.loginfo('Estimated framerate: {:.2f} fps'.format(framerate))
        frame_count = 0
        start_time = time.time()
        
        output_video = cv2.VideoWriter('output_video.mp4', fourcc, framerate, (frame.shape[1], frame.shape[0]))
        

image_sub = rospy.Subscriber(image_topic, Image, image_callback)

# Start the main ROS loop
rospy.spin()

# Release the output video object
print('node shutdown')
if output_video is not None:
    output_video.release()

# Clean up
cv2.destroyAllWindows()
