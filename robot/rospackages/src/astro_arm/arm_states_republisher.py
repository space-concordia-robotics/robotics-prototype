#!/usr/bin/env python3

from math import radians
import rospy
from sensor_msgs.msg import JointState

def subscriber_callback(data):
    msg = JointState()
    msg.header = data.header
    # joint names taken from the urdf file
    msg.name.append("motor1")
    msg.name.append("motor2")
    msg.name.append("motor3")
    msg.name.append("motor4")

    msg.position.append(radians(data.position[0]))
    msg.position.append(radians(data.position[1]))
    msg.position.append(radians(data.position[2]))
    msg.position.append(radians(data.position[3]))

    anglePub.publish(msg)

if __name__ == '__main__':
    node_name = 'arm_republisher'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" multidirectional node')

    angle_sub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to subscribe to "'+angle_sub_topic+'" topic')
    angleSub = rospy.Subscriber(angle_sub_topic, JointState, subscriber_callback)

    angle_pub_topic = '/joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    rospy.spin()
