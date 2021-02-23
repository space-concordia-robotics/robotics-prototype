#!/usr/bin/env python2
import sys

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class arTracker():

    #the size of the marker (14cmx14cm)
    markerSizeCM = 2.2

    def __init__(self):
        self.markers = []

        # necessary for handling images from topics
        self.node_name = 'ar_tracker'
        rospy.init_node(self.node_name)
        # make sure to use `rosparam set ~camera_node_name video0Cam` 
        #cv_camera_node_name = rospy.get_param('~camera_node_name')
        cv_camera_node_name = 'video0Cam'

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
        self.markers = ar_pose_marker.markers
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

        #@TODO: use these values and dynamically set max height/width values used in map_from_ar_pos.. function?
        shape = frame.shape
        height = shape[0]
        width = shape[1]

        # Line thiccness of 2 px
        thiccness = 2

        image = frame

        #@TODO: when done with the rest, make sure to remove the default red square!
        if self.is_marker_seen:
            # Green color in BGR
            color = (0, 255, 0)

            for i in range(len(self.markers)):
                #if self.markers[i]:
                starting_point, ending_point = self.map_from_ar_pos_to_screen_pos(self.markers[i])

                # Draw a rectangle with blue line borders of thiccness of 2 px
                image = cv2.rectangle(image, starting_point, ending_point, color, thiccness)

        else:
            # Red color in BGR
            color = (0, 0, 255)

            # Represents the top left corner of rectangle
            starting_point = (1, 1)
            # Represents the bottom right corner of rectangle
            ending_point = (width - 2, height - 2)
            image = cv2.rectangle(frame, starting_point, ending_point, color, thiccness)

        # Publish the new overlay including image to topic '/camera/image_ar'
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish AR image: %s", e)

    def map_from_ar_pos_to_screen_pos(self, markers):
        '''
        returns list of size 2 containing the starting and ending points of top left and bottom right corners as tuples. These are necessary inputs for drawing the rectangles.
        '''

        ar_pos = markers.pose.pose.position

        # these values are fine to hardcode as long as the default streaming resultion (360p) is used
        # once this becomes variable, it will be time to dynamify these settings
        #MAX_HEIGHT = 640
        #MAX_WIDTH  = 360
        MAX_HEIGHT = 848
        MAX_WIDTH  = 480

        MIDDLE_HEIGHT = MAX_HEIGHT / 2 
        MIDDLE_WIDTH = MAX_WIDTH / 2 


        INPUT_SCALE_FACTOR = 60
        ar_z = ar_pos.z / 10
    
        ar_x = ar_pos.x * INPUT_SCALE_FACTOR / ar_z
        ar_y = ar_pos.y * INPUT_SCALE_FACTOR / ar_z

        print("x: {}, y: {}, z: {}".format(ar_x, ar_y, ar_z))

        # quick mafs where we translate these ar tag coordinates from 3D space to 2D rectangles
        # possibly change the size too based on the z values

        #SIZE_FACTOR = 10*self.markerSizeCM/ar_z

        #HACK_X = 0
        #HACK_Y = 0
        HACK_X = -30 + int(ar_x * 0.01)
        HACK_Y = -20 + int(ar_y * 0.05)
        #start_point = (int(ar_x+ MIDDLE_HEIGHT + HACK_X) , int(ar_y+MIDDLE_WIDTH + HACK_Y))
        #end_point = int((ar_x + MIDDLE_HEIGHT+ 20 + HACK_X)/ar_z), int((ar_y +MIDDLE_WIDTH+ 20 + HACK_Y)/ar_z)
        start_point = (int(ar_x+ MIDDLE_HEIGHT + HACK_X) , int(ar_y+MIDDLE_WIDTH + HACK_Y))
        end_point = (int(ar_x + MIDDLE_HEIGHT+ 20 + HACK_X), int(ar_y +MIDDLE_WIDTH+ 20 + HACK_Y))

        return [start_point, end_point] 

    def cleanup(self):
        print('shutting down ar_tracker node')

def main(args):
    try:
        arTracker()
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down ar_tracker node')

if __name__ == '__main__':
    main(sys.argv)

