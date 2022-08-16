#! /usr/bin/env python3

from multiprocessing import current_process
import sys
import traceback
import time
import re


import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents, PowerConsumption, PowerReport
from mcu_control.srv import PowerReportProvider, PowerReportProviderRequest, PowerReportProviderResponse


wheel_watt_hours = []
latest_power_report = PowerReport()
pub = None
prevTime = None
running = False

def provide_report(data):
    return latest_power_report

def action_callback(message):
    global running
    if message.data == 'start':
        running = True
        initData()
    elif message.data == 'stop':
        running = False
    else:
        rospy.loginfo('invalid command')

def wheel_current_callback(data):
    global wheel_watt_hours, pub, prevTime, running
    if running:
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
        prevTime = time
        pub.publish(latest_power_report)


def initData():
    global wheel_watt_hours, latest_power_report
    wheel_watt_hours = [0, 0, 0, 0, 0, 0]
    latest_power_report = PowerReport()
    latest_power_report.report = [PowerConsumption(description='wheels', wattHours=0)]

def start():
    initData()
    rospy.init_node('power_report_node', anonymous = False)
    # subscribe to PDS feeds
    rospy.Subscriber('wheel_motor_currents', Currents, wheel_current_callback)
    # subscribe for start/stop commands
    rospy.Subscriber('power_report_command', String, action_callback)

    global pub
    pub = rospy.Publisher('power_consumption', PowerReport, queue_size = 10)
    s = rospy.Service('power_report_provider', PowerReportProvider, provide_report)

    rospy.spin()

if __name__ == '__main__':
    start()