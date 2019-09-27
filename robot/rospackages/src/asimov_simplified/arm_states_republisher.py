#!/usr/bin/env python3

from math import radians
import rospy
from sensor_msgs.msg import JointState

def subscriber_callback(data):
    msg = JointState()
    msg.header = data.header
    # joint names taken from the urdf file
    msg.name.append("upbase_joint")
    msg.name.append("prox_joint")
    msg.name.append("distal_joint")
    msg.name.append("wrist_flex_joint")
    msg.name.append("wrist_twist_joint")
    msg.name.append("finger1_joint")
    msg.name.append("finger2_joint")

    msg.position.append(radians(data.position[0]))
    msg.position.append(radians(data.position[1]))
    msg.position.append(radians(data.position[2]))
    msg.position.append(radians(data.position[3]))
    msg.position.append(radians(data.position[4]))
    msg.position.append(radians(data.position[5]))
    msg.position.append(msg.position[5])

    anglePub.publish(msg)

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" multidirectional node')

    angle_sub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to subscribe to "'+angle_sub_topic+'" topic')
    angleSub = rospy.Subscriber(angle_sub_topic, JointState, subscriber_callback)

    angle_pub_topic = '/joint_states'
    #angle_pub_topic = '/arm_republished_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    rospy.spin()
