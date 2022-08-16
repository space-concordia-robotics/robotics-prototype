#! /usr/bin/env python3

import sys
import traceback
import time
import re


import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents

watt_hours = []
pub = None
prevTime = None

def currentCallback(data):
    global watt_hours, pub, prevTime
    time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
    if prevTime is not None:
        delta = time - prevTime
        for i in range(len(data.effort)):
            watt_hours[i] += data.effort[i] * delta
    else:
        # if this is the first time, initialize the watt_hours array
        watt_hours = [0 for i in range(len(data.effort))]
    prevTime = time
    pub.publish("energy: " + str(watt_hours))




def start():
    rospy.init_node('power_report_node', anonymous = False)
    rospy.Subscriber('wheel_motor_currents', Currents, currentCallback)
    global pub
    pub = rospy.Publisher('power_consumption', String, queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    start()