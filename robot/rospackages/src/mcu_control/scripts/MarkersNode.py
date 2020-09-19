#!/usr/bin/env python3

import sys
import time
import rospy
import os
import json

from mcu_control.msg import RoverMarker, RoverMarkerList
from std_msgs.msg import String

json_file = 'markers.json'

def save_markers(markers, filename):
    with open(filename, "w") as f:
        marker_dict = {}
        for marker in markers:
            marker_dict[marker.name] = {'longitude' : marker.longitude, 'latitude' : marker.latitude, 'color' : marker.color}
        json.dump(marker_dict, f)

def load_markers(filename):
    if not os.path.exists(filename):
        return []
    else:
        with open(filename) as f:
            rospy.loginfo('Found marker.json file')
            marker_dict = json.load(f)
            markers = []
            for marker_name in marker_dict.keys():
                color = marker_dict[marker_name]['color']
                lat = marker_dict[marker_name]['latitude']
                long = marker_dict[marker_name]['longitude']
                marker = RoverMarker(marker_name, color, long, lat)
                markers.append(marker)
            return markers


def create_marker_callback(message):
    marker = RoverMarker(message.name, message.color, message.longitude, message.latitude)
    marker_list.append(marker)
    rospy.loginfo('Created marker ' + message.name)
    save_markers(marker_list, json_file)


def delete_marker_callback(message):
    name = message.data
    for marker in marker_list:
        if marker.name == name:
            rospy.loginfo('Deleted marker ' + name)
            marker_list.remove(marker)
            save_markers(marker_list, json_file)

def set_as_current_marker_callback(message):
    name = message.data
    index = 0

    for marker in marker_list:
        if marker.name == name:
            marker_list.insert(0, marker_list.pop(index))
        index += 1

if __name__ == '__main__':
    node_name = 'markers_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub/service functionality')

    create_marker_sub_topic = 'create_marker'
    create_marker_sub = rospy.Subscriber(create_marker_sub_topic, RoverMarker, create_marker_callback)

    delete_marker_sub_topic = 'delete_marker'
    delete_marker_sub = rospy.Subscriber(delete_marker_sub_topic, String, delete_marker_callback)

    set_as_current_marker_sub_topic = 'set_as_current_marker'
    set_as_current_marker_sub = rospy.Subscriber(set_as_current_marker_sub_topic, String, set_as_current_marker_callback)

    marker_list_pub_topic = 'marker_list'
    rospy.loginfo('Beginning to publish to "' + marker_list_pub_topic + '" topic')
    marker_pub = rospy.Publisher(marker_list_pub_topic, RoverMarkerList, queue_size=1)
    rate = rospy.Rate(2)  # 2Hz

    marker_list = load_markers(json_file)

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
