#!/usr/bin/env python3

import sys
import time

from Nav_funs import Distance, Direction
import rospy
from geometry_msgs.msg import Point
#from mcu_control.msg import RoverPosition, AntennaGoal


def subscriber_callback(message):
    rospy.loginfo(message)
    if message.x < -900 or message.y < -900:
        return
    rover['latitude'] = message.x
    rover['longitude'] = message.y

    if rospy.get_param('got_antenna_pos'):
        antenna_to_Rover_dir = Direction(
            antenna['latitude'], antenna['longitude'], rover['latitude'], rover['longitude'])
        antenna_to_Rover_dis = Distance(
            antenna['latitude'], antenna['longitude'], rover['latitude'], rover['longitude'])

        rotatorAngle = antenna_to_Rover_dir - antenna['startDir'] + 180
        if rotatorAngle < 0:
            rotatorAngle += 360
        elif rotatorAngle > 360:
            rotatorAngle -= 360

        msg = Point()
        msg.x = rotatorAngle / 10
        msg.y = antenna_to_Rover_dis
        antennaPub.publish(msg)
    else:
        rospy.loginfo('Waiting for antenna position ROS parameters to be set')
    return


if __name__ == '__main__':
    node_name = 'antenna_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub functionality')

    subscribe_topic = '/rover_position'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, Point, subscriber_callback)

    antenna_pub_topic = '/antenna_goal'
    rospy.loginfo('Beginning to publish to "' + antenna_pub_topic + '" topic')
    antennaPub = rospy.Publisher(antenna_pub_topic, Point, queue_size=10)

    gotAntennaPos = False
    antenna = {'latitude': None, 'longitude': None,
               'startDir': None, 'recommendedDir': None}
    rover = {'latitude': None, 'longitude': None, 'distance': None}

    rospy.loginfo('This node needs the antenna starting position (antenna_latitude, ' +
                  'antenna_longitude, antenna_start_dir) and will wait until it receives that')
    try:
        while not rospy.is_shutdown():

            if not rospy.get_param('got_antenna_pos'):
                try:  # rospy.has_param(param) works but requires code rethinking
                    antenna['latitude'] = rospy.get_param('antenna_latitude')
                    antenna['longitude'] = rospy.get_param('antenna_longitude')
                    antenna['startDir'] = rospy.get_param('antenna_start_dir')
                    rospy.set_param('got_antenna_pos', True)
                    rospy.loginfo('Got antenna starting position! ' + str(antenna['latitude']) + ' ' + str(
                        antenna['longitude']) + ' ' + str(antenna['startDir']))
                except KeyError:  # param not define
                    pass
            else:
                pass
            rospy.Rate(100).sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        # give ROS time to deal with the node closing (rosbridge especially)
        time.sleep(1)

    rospy.on_shutdown(shutdown_hook)
