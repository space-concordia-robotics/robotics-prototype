#!/usr/bin/env python3

import sys
import time
import rospy
from mcu_control.msg import RoverMarker, RoverMarkerList
from std_msgs.msg import String


def create_marker_callback(message):
    marker = Rovermarker(message.name, message.color, message.longitude, message.latitude)
    marker_list.append(marker)


def delete_marker_callback(message):
    name = message.data
    for marker in marker_list:
        if marker.name == name:
            marker_list.remove(marker)

def set_as_current_marker_callback(message):
    name = message.data
    index = 0

    for marker in marker_list:
        if marker.name == name:
            marker_list.insert(0, marker_list.pop(index))
        index += 1

def restore_markers_callback(message):
    marker_list.clear()
    for marker in message.marker_list:
        marker_list.append(marker)

if __name__ == '__main__':
    node_name = 'markers_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub/service functionality')

    create_marker_sub_topic = 'create_marker'
    create_marker_sub = rospy.Subscriber(create_marker_sub_topic, Rovermarker, create_marker_callback)

    delete_marker_sub_topic = 'delete_marker'
    delete_marker_sub = rospy.Subscriber(delete_marker_sub_topic, String, delete_marker_callback)

    set_as_current_marker_sub_topic = 'set_as_current_marker'
    set_as_current_marker_sub = rospy.Subscriber(set_as_current_marker_sub_topic, String, set_as_current_marker_callback)

    restore_markers_sub_topic = 'restore_markers'
    restore_markers_sub = rospy.Subscriber(restore_markers_sub_topic, RovermarkerList, restore_markers_callback)

    marker_list_pub_topic = 'marker_list'
    rospy.loginfo('Beginning to publish to "' + marker_list_pub_topic + '" topic')
    marker_pub = rospy.Publisher(marker_list_pub_topic, RovermarkerList, queue_size=1)
    rate = rospy.Rate(2)  # 2Hz

    marker_list = []

    try:
        while not rospy.is_shutdown():
            marker_pub.publish(marker_list)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        # give ROS time to deal with the node closing (rosbridge especially)
        time.sleep(1)

    rospy.on_shutdown(shutdown_hook)
