#!/usr/bin/env python3

import sys
import time

from Nav_funs import Distance, Direction
import rospy
from geometry_msgs.msg import Point
from arm_control.msg import RoverPosition, AntennaGoal

def subscriber_callback(message):
    rospy.loginfo(message)
    rover['latitude'] = message.x
    rover['longitude'] = message.y

    if gotAntennaPos:
        BS_to_Rover_dir = Direction(antenna['latitude'], antenna['longitude'], rover['latitude'], rover['longitude'])
        BS_to_Rover_dis = Distance(antenna['latitude'], antenna['longitude'], rover['latitude'], rover['longitude'])

        rotatorAngle = BS_to_Rover_dir - antenna['startDir'] + 180
        if rotatorAngle < 0:
            rotatorAngle += 360
        elif rotatorAngle > 360:
            rotatorAngle -= 360

        msg = Point()
        msg.desiredDir = rotatorAngle
        msg.distFromBase = BS_to_Rover_dis
        antennaPub.publish(msg)
    else:
        rospy.loginfo('Waiting for antenna position ROS parameters to be set')
    return

if __name__ == '__main__':
    node_name = 'antenna_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub functionality')

    subscribe_topic = '/rover_position'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, Point, subscriber_callback)

    antenna_pub_topic = '/antenna_goal'
    rospy.loginfo('Beginning to publish to "'+antenna_pub_topic+'" topic')
    antennaPub = rospy.Publisher(antenna_pub_topic, Point, queue_size=10)

    gotAntennaPos = False
    antenna = {'latitude':None, 'longitude':None, 'startDir':None, 'recommendedDir':None}
    rover = {'latitude':None, 'longitude':None, 'distance':None}

    rospy.loginfo('This node needs the antenna starting position (antenna_latitude, '+ \
    'antenna_longitude, antenna_start_dir) and will wait until it receives that')
    try:
        while not rospy.is_shutdown():
            if not gotAntennaPos:
                try:
                    antenna['latitude'] = rospy.get_param('antenna_latitude')
                    antenna['longitude'] = rospy.get_param('antenna_longitude')
                    antenna['startDir'] = rospy.get_param('antenna_start_dir')
                    gotAntennaPos = True
                    rospy.loginfo('Got antenna starting position!')
                except KeyError: # param not defined
                    pass
            else:
                pass
            rospy.Rate(100).sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
