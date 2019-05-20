#!/usr/bin/env python3

import sys
import traceback
import time
import re

from ik-calculator import * # whatever i need

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState

def subscriber_callback(message):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo('sending: '+message.data)
    command = str.encode(message.data+'\n')
    ser.write(command) # send command to teensy

    startListening = time.time()
    # 300 ms timeout... could potentially be even less, needs testing
    while (time.time()-startListening < timeout):
        if ser.in_waiting:
            try:
                data_str = ser.readline().decode()
                if "Motor Angles" not in data_str:
                    data_str = "received "+data_str
                    data_str+=" at %f" % rospy.get_time()
                    rospy.loginfo(data_str)
                    break
                else:
                    publish_joint_states(data_str)
            except:
                rospy.logwarn('trouble reading from serial port')
    return

def publish_joint_states(message):
    # parse the data received from Teensy
    lhs,rhs = message.split('Motor Angles: ')
    lhs,rhs = rhs.split('\n')
    angles = lhs.split(', ')
    # create the message to be published
    msg = JointState()
    msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    try:
        for angle in angles:
            msg.position.append(float(angle))
    except:
        rospy.logwarn('trouble parsing motor angles:',sys.exc_info()[0])
        return

    # publish it
    anglePub.publish(msg)
    rospy.logdebug(msg.position)
    return

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    angle_pub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    feedback_pub_topic = '/arm_feedback'
    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    # maybe make a global param for publishing and subscribing this data
    # the arm node publishes at a rate, this node listens and publishes,
    # plus the website publishes/requests commands at its own rate...
    rate = rospy.Rate(20) # 20hz

    data = ""; getTime = ""; data_str = ""

    subscribe_topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    # subscriber callbacks are implicitly handled but only at the rate the node publishes at
    try:
        while not rospy.is_shutdown():
            if ser.in_waiting:
                try:
                    data = ser.readline().decode()
                    #if data[0]=='@':
                    if 'Motor Angles' in data and data[0]=='@':
                        publish_joint_states(data)
                    elif data is not '':
                        getTime = "received at %f" % rospy.get_time()
                        data_str = data + ' ' + getTime
                        rospy.logdebug(data_str)
                        feedbackPub.publish(data)
                except:
                    rospy.logwarn('trouble reading from serial port')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
