import sys
import time

from Nav_funs import Direction, Distance, Turning
import rospy
from geometry_msgs.msg import Point
#from mcu_control.msg import RoverPosition, RoverGoal

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

    if gotGpsPos:
        Rov_to_des_distance = Distance(rover['latitude'], rover['longitude'], \
        gpsGoal['latitude'], gpsGoal['longitude'])
        Rov_to_des_direction = Direction(rover['latitude'], rover['longitude'], \
        gpsGoal['latitude'], gpsGoal['longitude'])

        if hasHeading:
            Direction_adjust = Turning(Rov_to_des_direction, rover['heading'])
        else:
            Direction_adjust = -999 # no heading, give invalid number

        msg = Point()
        # note that direction is based on compass directions where E is 90 and W is -90
        msg.x = Direction_adjust
        msg.y = Rov_to_des_distance
        navigationPub.publish(msg)
    else:
        rospy.loginfo('Waiting for gps goal coordinate ROS parameters to be set')
    return

if __name__ == '__main__':
    node_name = 'navigation_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub functionality')

    subscribe_topic = '/rover_position'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, Point, subscriber_callback)

    navigation_pub_topic = '/rover_goal'
    rospy.loginfo('Beginning to publish to "'+navigation_pub_topic+'" topic')
    navigationPub = rospy.Publisher(navigation_pub_topic, Point, queue_size=10)

    gotGpsPos = False
    gpsGoal = {'latitude':None, 'longitude':None}
    rover = {'latitude':None, 'longitude':None, 'heading':None, 'distance':None}

    rospy.loginfo('This node needs the antenna starting position (antenna_longitude, '+ \
    'antenna_latitude, antenna_start_dir) and will wait until it receives that')
    rospy.loginfo('This node needs the gps goal coordinates (gps_latitude, gps_longitude) '+ \
    'and will wait until it receives that')
    try:
        while not rospy.is_shutdown():
            if not gotGpsPos:
                try: #rospy.has_param(param) works but requires code rethinking
                    gpsGoal['latitude'] = rospy.get_param('goal_latitude')
                    gpsGoal['longitude'] = rospy.get_param('goal_longitude')
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
