#!/usr/bin/env python3

import sys
import time

from Nav_funs import Direction, Distance, Turning
import rospy
from arm_control.msg import RoverPosition, RoverGoal

def subscriber_callback(message):
    rospy.loginfo(message)
    rover['longitude'] = message.latitude
    rover['latitude'] = message.longitude
    rover['heading'] = message.heading

    if gotGpsPos:
        Rov_to_des_distance = Distance(rover['latitude'], rover['longitude'], \
        gpsGoal['latitude'], gpsGoal['longitude'])
        Rov_to_des_direction = Direction(rover['latitude'], rover['longitude'], \
        gpsGoal['latitude'], gpsGoal['longitude'])

        Direction_adjust = -Turning(Rov_to_des_direction, rover['heading'])

        msg = RoverGoal()
        msg.desiredDir = Direction_adjust
        msg.distToGoal = Rov_to_des_distance
        navigationPub.publish(msg)
    return

if __name__ == '__main__':
    node_name = 'navigation_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub functionality')

    subscribe_topic = '/rover_position'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, RoverPosition, subscriber_callback)

    navigation_pub_topic = '/rover_goal'
    rospy.loginfo('Beginning to publish to "'+navigation_pub_topic+'" topic')
    navigationPub = rospy.Publisher(navigation_pub_topic, RoverGoal, queue_size=10)

    gotGpsPos = False
    gpsGoal = {'latitude':None, 'longitude':None}
    rover = {'latitude':None, 'longitude':None, 'heading':None, 'distance':None}

    rospy.loginfo('This node needs the antenna starting position and will wait until it receives that')
    try:
        while not rospy.is_shutdown():
            if not gotGpsPos:
                try:
                    gpsGoal['latitude'] = rospy.get_param('gps_latitude')
                    gpsGoal['longitude'] = rospy.get_param('gps_longitude')
                    gotGpsPos = True
                    rospy.loginfo('Got GPS goal coordinates!')
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
