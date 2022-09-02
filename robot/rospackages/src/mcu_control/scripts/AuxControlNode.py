#! /usr/bin/env python3

# Commands are handled by arduino aux code
# scale tare -> OK
# scale read -> floating point number in grams
# voltage -> floating point number in volts
# fan on/off -> OK
# fan set <number 0 to 255> -> OK
# Anything illegal -> ERR

import sys
import traceback
import time
import re

import serial

import rospy
from std_msgs.msg import String

from collections import deque

command_queue = deque()

def send_queued_commands():
    if (len(command_queue) > 0):
        command = command_queue.popleft()
        send_command(command)

def send_command(command):
    if command is not None:
        ser.write((command + '\n').encode())

def receive_response():
    if ser.in_waiting > 0:
        response = ser.readline().decode("utf-8") 
        auxReplyPub.publish(response)

def command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to aux')

    command_queue.append(message.data)

if __name__ == '__main__':
    node_name = 'aux_node'
    rospy.init_node(node_name, anonymous = False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    aux_reply_pub_topic = '/aux_reply'
    rospy.loginfo('Begining to publish to "' + aux_reply_pub_topic + '" topic')
    auxReplyPub = rospy.Publisher(aux_reply_pub_topic, String, queue_size = 10)

    subscribe_topic = '/aux_command'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, String, command_callback)

    rosRate = rospy.Rate(20)

    ser = serial.Serial('/dev/ttyUSB0', 9600)

    try:
        while not rospy.is_shutdown():
            send_queued_commands()
            receive_response()

            rosRate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        ser.close()
        time.sleep(1)  # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
