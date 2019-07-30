#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports #pyserial
import rospy
from std_msgs.msg import String

global ser
# todo: make initSerial return the serial object?
# then it can be a separate script to be imported
def initSerial():
    global ser
    # set up connection to arduino
    ports = list(serial.tools.list_ports.comports())
    is_arm = False

    if len(ports) == 1:
        rospy.loginfo("1 USB device detected")
        port = ports[0].name
        ser = serial.Serial('/dev/' + port, 115200)

        rospy.loginfo("clearing buffer")
        while ser.in_waiting:
            rospy.loginfo(ser.readline().decode())

        for i in 0, 3:
            who = ""
            rospy.loginfo("identifying MCU")
            ser.write(str.encode("who\n"))

            # CRITICAL: give time for MCU to respond
            time.sleep(1)

            while ser.in_waiting:
                who = ser.readline().decode()
                rospy.loginfo("who: \"" + who.strip() + "\"")
                if "arm" in who.strip():
                    rospy.loginfo("Arm MCU idenified!")
                    is_arm = True

    else:
        rospy.loginfo("No USB devices recognized, exiting")
        sys.exit(0)

    if is_arm:
        rospy.loginfo("Connected to port: " + port)
    else:
        rospy.loginfo("Incorrect MCU connected, terminating listener")
        sys.exit(0)


if __name__ == '__main__':
    node_name = 'arm_talker'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" publisher node')
    
    initSerial()

    topic = '/arm_angles'
    rospy.loginfo('Beginning to publish to "'+topic+'" topic')
    pub = rospy.Publisher(topic, String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    data = ""; getTime = ""; data_str = ""
    try:
        while not rospy.is_shutdown():
            if ser.in_waiting:
                data = ser.readline().decode()
                getTime = "received at %f" % rospy.get_time()
                data_str = data + ' ' + getTime
            else:
                data = ""; getTime = ""; data_str = ""
            rospy.loginfo(data_str)
            pub.publish(data_str)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass