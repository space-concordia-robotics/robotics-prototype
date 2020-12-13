#!/usr/bin/env python2
import sys

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class arTrackerDemo():
    def __init__(self):
        # necessary for handling images from topics
        self.node_name = 'ar_tracker'
        rospy.init_node(self.node_name)
        cv_camera_node_name = rospy.get_param('~cv_camera_node_name')

        self.is_marker_seen = False

        # what we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # create the cv_bridge object
        self.bridge = CvBridge()

        # subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber('/'+ cv_camera_node_name +'/image_raw', Image, self.image_callback)

        # subscribe to ar_track_alvar marker messages
        self.ar_track_marker_sub = rospy.Subscriber('/ar_pose_marker_'+cv_camera_node_name, AlvarMarkers, self.ar_track_alvar_callback)

        # create publisher for new feed with overlay
        self.image_pub = rospy.Publisher('/'+ cv_camera_node_name +'/image_ar', Image, queue_size=10)

        rate = rospy.Rate(1) # 1hz, take it easy 8-)

        rospy.loginfo('Waiting for image topics...')

    def ar_track_alvar_callback(self, ar_pose_marker):
        if ar_pose_marker.markers:
            self.is_marker_seen = True
        else:
            self.is_marker_seen = False

    def image_callback(self, ros_image):
        # use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError:
            traceback.print_exc()

        # convert the image to a numpy array since most cv2 functions
        # require numpy arrays
        frame = np.array(frame, dtype=np.uint8)

        shape = frame.shape
        height = shape[0]
        width = shape[1]

        # Starting coordinate
        # Represents the top left corner of rectangle
        starting_point = (width/2 - width/5, height/2 - height/5)

        # Ending coordinate
        # Represents the bottom right corner of rectangle
        ending_point = (width/2 + width/5, height/2 + width/5)

        if self.is_marker_seen:
            # Green color in BGR
            color = (0, 255, 0)
        else:
            # Red color in BGR
            color = (0, 0, 255)

        # Line thickness of 2 px
        thickness = 2

        # Draw a rectangle with blue line borders of thickness of 2 px
        image = cv2.rectangle(frame, starting_point, ending_point, color, thickness)

        # Publish the new overlay including image to topic '/camera/image_ar'
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish AR image: %s", e)


    def cleanup(self):
        print('shutting down ar_tracker node')

def main(args):
    try:
        arTrackerDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down ar_tracker node')

if __name__ == '__main__':
    main(sys.argv)

