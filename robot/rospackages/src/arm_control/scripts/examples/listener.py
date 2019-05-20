#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data)

if __name__ == '__main__':
    node_name = 'listener'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" subscriber node')
    
    topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+topic+'" topic')
    sub = rospy.Subscriber(topic, String, callback)
    # use this to block if nothing else needs to happen
    # and you don't want the node to close. for callbacks/handlers
    rospy.spin()