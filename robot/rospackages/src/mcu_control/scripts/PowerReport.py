#! /usr/bin/env python3

from multiprocessing import current_process
import sys
import traceback
import time
import re


import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents, PowerConsumption, PowerReport

wheel_watt_hours = []
latest_power_report = PowerReport()
pub = None
prevTime = None

def wheelCurrentCallback(data):
    global wheel_watt_hours, pub, prevTime
    time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
    if prevTime is not None:
        # find power consumed since last datapoint
        delta = time - prevTime
        for i in range(len(data.effort)):
            wheel_watt_hours[i] += data.effort[i] * delta
        # now, sum up all wheels and put in report
        wheel_power = 0
        for x in wheel_watt_hours:
            wheel_power += x
        latest_power_report.report[0] = PowerConsumption("wheels", wheel_power)

        rospy.loginfo('wheel watt hours: ' + str(wheel_watt_hours))
    else:
        # if this is the first time, initialize the watt_hours array
        wheel_watt_hours = [0 for i in range(len(data.effort))]
        latest_power_report.report = [0]
    prevTime = time
    pub.publish(latest_power_report)




def start():
    rospy.init_node('power_report_node', anonymous = False)
    rospy.Subscriber('wheel_motor_currents', Currents, wheelCurrentCallback)
    global pub
    pub = rospy.Publisher('power_consumption', PowerReport, queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    start()