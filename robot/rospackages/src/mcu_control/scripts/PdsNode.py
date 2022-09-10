#! /usr/bin/env python3

import sys
import traceback
import time
import re
import serial


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


def parse_report(report, wheel_pub, arm_pub, control_pub):
    channel_powers = []
    for i in range(2, 18):
        channel = i - 2
        milliwats = int(report[i].split(',')[2][:-2])
        watts = milliwats / 1000
        channel_powers.append(watts)
    # rospy.loginfo(channel_powers)
    arm_pub.publish(Currents(channel_powers[0:4]))
    # mainboard + lidar is counted as the same
    control_pub.publish(Currents([channel_powers[4] + channel_powers[15]]))
    wheel_pub.publish(Currents(channel_powers[8:14]))



if __name__ == '__main__':
    # port = serial.Serial('/dev/ttyUSB1', 115200)

    node_name = 'pds_node'
    rospy.init_node(node_name, anonymous = False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    # Currents
    wheel_current_pub_topic = '/wheel_motor_powers'
    rospy.loginfo('Begining to publish to "' + wheel_current_pub_topic + '" topic')
    wheelCurrentPub = rospy.Publisher(wheel_current_pub_topic, Currents, queue_size = 10)

    arm_current_pub_topic = '/arm_motor_powers'
    rospy.loginfo('Begining to publish to "' + arm_current_pub_topic + '" topic')
    armCurrentPub = rospy.Publisher(arm_current_pub_topic, Currents, queue_size=10)

    control_current_pub_topic = '/control_power'
    rospy.loginfo('Begining to publish to "' + control_current_pub_topic + '" topic')
    controlCurrentPub = rospy.Publisher(control_current_pub_topic, Currents, queue_size=10)

    
    try:
        num_lines = 18
        report_arr = []

        report = ['@5591',': 14.944J',
        '# 0: 6362mV, 1mA, 100mW, 0.936J',
        '# 1: 6362mV, 1mA, 200mW, 0.936J',
        '# 2: 6362mV, 1mA, 300mW, 0.936J',
        '# 3: 6362mV, 1mA, 400mW, 0.936J',
        '# 4: 6362mV, 1mA, 500mW, 0.936J',
        '# 5: 6362mV, 1mA, 600mW, 0.936J',
        '# 6: 6362mV, 1mA, 1000mW, 0.936J',
        '# 7: 6362mV, 1mA, 2000mW, 0.936J',
        '# 8: 6362mV, 1mA, 30000mW, 0.932J',
        '# 9: 6362mV, 1mA, 40000mW, 0.932J',
        '#10: 6362mV, 1mA, 50000mW, 0.932J',
        '#11: 6362mV, 1mA, 60000mW, 0.932J',
        '#12: 6362mV, 1mA, 70000mW, 0.932J',
        '#13: 6362mV, 1mA, 80000mW, 0.932J',
        '#14: 6362mV, 1mA, 90000mW, 0.932J',
        '#15: 6362mV, 1mA, 100000mW, 0.932J']
        
        rate = rospy.Rate(5) # ROS Rate at 5Hz

        while not rospy.is_shutdown():
            if False:
            # if port.in_waiting > 0:
                rospy.loginfo('about to read')
                report_arr.clear()
                for i in range(num_lines):
                    line = port.read_until().decode('utf-8')
                    if '!Startup' in line:
                        line = port.read_until().decode('utf-8')
                        if '!Found 8PMIC' in line:
                            line = port.read_until().decode('utf-8')
                    report_arr.append(line)
                rospy.loginfo('power report')
                rospy.loginfo(report_arr)
            else:
                parse_report(report, wheelCurrentPub, armCurrentPub, controlCurrentPub)
                rate.sleep()

                

    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        port.close()
        time.sleep(1)  # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
