#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports #pyserial
import rospy
from arm_server.srv import *

global ser

def init_serial():
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

def handle_client(req):
    global ser
    rospy.loginfo("received "+req.msg)
    data_str = ""
    msg = {ArmServerResponse(
        response = "no response",
        success = False
    )}
    ser.write(str.encode("ping\n"))
    while True:
        if ser.in_waiting:
            data_str = ser.readline().decode()
            if "pong" in data_str:
                data_str = "received "+data_str
                data_str+=" at %f" % rospy.get_time()
                msg.success = True;
                msg.response = data_str;
                break
    rospy.loginfo(msg)
    return msg

if __name__ == '__main__':
    node_name = 'arm_service'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" service node')

    init_serial()

    service_name = 'arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)
    # use this to block if nothing else needs to happen
    # and you don't want the node to close. for callbacks/handlers
    rospy.spin()
