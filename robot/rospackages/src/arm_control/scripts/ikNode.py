#!/usr/bin/env python3

import sys
import traceback
import time
import re

import ik_calculator as IK # whatever i need

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
#from arm_control.msg import IkData

joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
desired_position = [0.0, 0.0, 0.0]
desired_wrist_angle = 0.0

def joint_state_callback(message):
    joint_angles = message.position
    rospy.loginfo(joint_angles)
    #rospy.logdebug(joint_angles)

def ik_command_callback(message):
    #todo: implement error handling if there's a big difference between current and previous position
    desired_position = [message.x, message.y, message.z]
    desired_wrist_angle = [message.wrist]
    rospy.loginfo(message)
    #rospy.logdebug(message)



    armCommandPub.publish(command)
    rospy.loginfo(command)
    #rospy.logdebug(command)

if __name__ == '__main__':
    node_name = 'ik_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub functionality')

    asimov = IK.Arm(4, IK.length_array, IK.minmax)
    rospy.loginfo('Initialized arm inverse kinematics model')
    startCalc=time.time()
    answer=asimov.computeIK(0.5,0.5,0.5,0)
    rospy.loginfo((time.time()-startCalc)*1000)
    rospy.loginfo(answer[0]);rospy.loginfo(answer[1]);rospy.loginfo(answer[2])
    answer=asimov.computeIK(0.45,0.55,0.45)
    rospy.loginfo(answer[0]);rospy.loginfo(answer[1]);rospy.loginfo(answer[2])

    joint_state_sub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to subscribe to "'+joint_state_sub_topic+'" topic')
    sub = rospy.Subscriber(joint_state_sub_topic, JointState, joint_state_callback)

    #ik_sub_topic = '/ik_command'
    #rospy.loginfo('Beginning to subscribe to "'+ik_sub_topic+'" topic')
    #sub = rospy.Subscriber(ik_sub_topic, IkData, ik_command_callback)

    publish_topic = '/arm_command'
    rospy.loginfo('Beginning to publish to "'+publish_topic+'" topic')
    armCommandPub = rospy.Publisher(publish_topic, String, queue_size=10)

    rospy.spin()

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
