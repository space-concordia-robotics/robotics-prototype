#! /usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import os
import datetime


import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Currents

# PDS Channel reference:
# 0-3 : arm motors
# 4   : mainboard
# 5   : unused
# 8-13: wheels
# 14  : unused?
# 15  : lidar

if __name__ == '__main__':
    port = serial.Serial('/dev/ttyUSB0', 115200)

    node_name = 'pds_node'
    rospy.init_node(node_name, anonymous = False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    folder = os.path.expanduser("~") + '/Power_Reports'
    if not os.path.exists(folder):
        os.makedirs(folder)

    timestamp = datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    filename = folder + '/Power-Report_' + timestamp + '.csv'
    try:
        with open(filename, 'w', newline='') as report_file:
            while not rospy.is_shutdown():
                if port.in_waiting > 0:
                    c = port.read(1)
                    if c == b'@':
                        timestamp = datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
                        report_file.write(timestamp)
                    report_file.write(c.decode("utf-8"))
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        port.close()
        time.sleep(1)  # give ROS time to deal with the node closing (rosbridge especially)
        report_file.close()

    rospy.on_shutdown(shutdown_hook)
