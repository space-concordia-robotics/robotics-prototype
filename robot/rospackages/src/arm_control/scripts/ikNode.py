#!/usr/bin/env python3

import sys
import traceback
import time
import re

import ik_calculator as IK # whatever i need
import math #imported by ik_calculator but repeated just in case

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from arm_control.msg import IkCommand

joint_angles = [0.0, 0.0, 0.0, 0.0]
desired_position = [0.0, 0.0, 0.0]
desired_wrist_angle = 0.0

def joint_state_callback(message):
    for i in range(0,4):
        joint_angles[i]=math.radians(message.position[i])
    joint_angles[1]+=math.pi/2 #shift to arm reference frames in ik_calculator
    #rospy.loginfo(joint_angles)
    #rospy.logdebug(joint_angles)
    asimov.setCurrentAngles(joint_angles)
    rospy.loginfo(asimov.getCurrentAngles())

def ik_command_callback(message):
    #todo: implement error handling if there's a big difference between current and previous position
    desired_position = [message.x, message.y, message.z]
    desired_wrist_angle = message.wrist
    rospy.loginfo(message)
    #rospy.logdebug(message)

    result = asimov.computeIK(desired_position, desired_wrist_angle)
    rospy.loginfo(result)
    #rospy.logdebug(result)
    if result[0] is not None:
        result[0][1]-=math.pi/2 #shift back into arm reference system
        command = 'move'
        for angle in result[0]:
            command += ' '+str(round(math.degrees(angle),4))
        command += ' ~ ~' #last 2 joints are controlled by other means
        armCommandPub.publish(command)
        rospy.loginfo(command)
        #rospy.logdebug(command)
    else:
        rospy.logwarn(result[2])

if __name__ == '__main__':
    node_name = 'ik_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub functionality')

    # warning: the ik_calculator library sets the shoulder 0 angle as horizontal whereas in my code it's vertical
    angleLimits = [[math.radians(-175),math.radians(175)],[math.radians(90-70),math.radians(90+45)],\
    [math.radians(-115),math.radians(65)],[math.radians(-90),math.radians(75)]]
    #rospy.loginfo( 'angle limits: '+str(angleLimits) )
    asimov = IK.Arm(4, IK.length_array, angleLimits) #todo: set proper link lengths
    # warning: the ik_calculator library sets the shoulder 0 angle as horizontal whereas in my code it's vertical
    rospy.loginfo('Initialized arm inverse kinematics model')

    rospy.loginfo( 'default angles: '+str(asimov.getCurrentAngles()) )
    startCalc=time.time()
    position = asimov.computeFK()
    rospy.loginfo( 'end effector position at said angles: '+str(position) )
    position[1]-=0.0001 #distance a bit from the extreme endpoint
    angles = asimov.computeIK(position)
    rospy.loginfo( 'angles from inverse kinematics at slightly shifted position: '+str(angles) )
    latency=(time.time()-startCalc)*1000
    rospy.loginfo( 'calculations took: '+str(round(latency,3))+' ms' )

    joint_state_sub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to subscribe to "'+joint_state_sub_topic+'" topic')
    sub = rospy.Subscriber(joint_state_sub_topic, JointState, joint_state_callback)

    ik_sub_topic = '/ik_command'
    rospy.loginfo('Beginning to subscribe to "'+ik_sub_topic+'" topic')
    sub = rospy.Subscriber(ik_sub_topic, IkCommand, ik_command_callback)

    publish_topic = '/arm_command'
    rospy.loginfo('Beginning to publish to "'+publish_topic+'" topic')
    armCommandPub = rospy.Publisher(publish_topic, String, queue_size=10)

    rospy.spin()

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
