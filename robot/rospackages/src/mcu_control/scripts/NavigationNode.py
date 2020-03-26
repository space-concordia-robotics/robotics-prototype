#!/usr/bin/env python3

from geometry_msgs.msg import Point
import rospy
from Nav_funs import Direction, Distance, Turning
import time
import sys
thismodule = sys.modules[__name__]
thismodule.gotGpsPos = False

def subscriber_callback(message):
    rospy.loginfo(message)
    if message.x < -900 or message.y < -900:
        return
    rover['latitude'] = message.x
    rover['longitude'] = message.y
    hasHeading = False
    if message.z > -900:
        rover['heading'] = message.z
        hasHeading = True

    if rospy.get_param('has_gps_goal'):
        rov_to_des_distance = Distance(rover['latitude'], rover['longitude'],
                                       gpsGoal['latitude'], gpsGoal['longitude'])
        rov_to_des_direction = Direction(rover['latitude'], rover['longitude'],
                                         gpsGoal['latitude'], gpsGoal['longitude'])

        if hasHeading:
            direction_adjust = Turning(rov_to_des_direction, rover['heading'])
        else:
            direction_adjust = -999  # no heading, give invalid number

        msg = Point()
        # note that direction is based on compass directions where E is 90 and W is -90
        msg.x = direction_adjust
        msg.y = rov_to_des_distance
        if(msg.y < 50):
            rospy.loginfo('moving on to next coorinates')

        rospy.loginfo('distance : ' + str(msg.y))
        navigationPub.publish(msg)
    else:
        rospy.loginfo(
            'Waiting for gps goal coordinate ROS parameters to be set')
    return


if __name__ == '__main__':
    node_name = 'navigation_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub functionality')

    subscribe_topic = '/rover_position'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, Point, subscriber_callback)

    navigation_pub_topic = '/rover_goal'
    rospy.loginfo('Beginning to publish to "' +
                  navigation_pub_topic + '" topic')
    navigationPub = rospy.Publisher(navigation_pub_topic, Point, queue_size=10)

    #gotGpsPos = False
    rospy.set_param('has_gps_goal', False)

    gpsGoal = {'latitude': None, 'longitude': None}
    rover = {'latitude': None, 'longitude': None,
             'heading': None, 'distance': None}

    rospy.loginfo('This node needs the gps goal coordinates (gps_latitude, gps_longitude) ' +
                  'and will wait until it receives that')
    try:
        while not rospy.is_shutdown():

            if not rospy.get_param('has_gps_goal'):
                try:  # rospy.has_param(param) works but requires code rethinking
                    gpsGoal['latitude'] = rospy.get_param('goal_latitude')
                    gpsGoal['longitude'] = rospy.get_param('goal_longitude')
                    rospy.set_param('has_gps_goal', True)

                    rospy.loginfo('Got GPS goal coordinates! ' + str(rospy.get_param(
                        'goal_latitude')) + " " + str(rospy.get_param('goal_longitude')))
                except KeyError:  # param not defined
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
