#!/usr/bin/env python3

import sys
import traceback
import time
import re

import serial
import serial.tools.list_ports #pyserial
#from robot.comms.uart import Uart

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from arm_control.srv import *

global ser # make global so it can be used in other parts of the code

# 300 ms timeout... could potentially be even less, needs testing
timeout = 0.3 # to wait for a response from the MCU

# todo: test ros+website over network with teensy
# todo: make a MCU serial class that holds the port initialization stuff and returns a reference?
# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
def init_serial():
    baudrate = 115200
    # in a perfect world, you can choose the baudrate
    rospy.loginfo('Using %d baud by default', baudrate)
    # in a perfect world, usb vs uart will be set by ROS params
    usb = True; uart = False
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) == 1:
        rospy.loginfo('Using USB by default')
    if len(myargv) > 1:
        if myargv[1] == 'uart':
            usb=False; uart=True
            rospy.loginfo('Using UART and 115200 baud by default')
        elif myargv[1] == 'usb':
            usb=True; uart=False
            rospy.loginfo('Using USB and 115200 baud by default')
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    global ser #make global so it can be used in other parts of the code

    ports = list(serial.tools.list_ports.comports())

    startConnecting = time.time()
    if usb:
        '''
        if len(ports) == 1:
            rospy.loginfo("1 USB device detected")
            port = ports[0].name
            rospy.loginfo('Attempting to connect to /dev/'+port)
            ser = serial.Serial('/dev/' + port, baudrate)

            rospy.loginfo("clearing buffer...")
            while ser.in_waiting:
                ser.readline()

            rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
            for i in range(5):
                rospy.loginfo('attempt #%d...', i+1)
                startListening = time.time()
                ser.write(str.encode('who\n'))
                while (time.time()-startListening < timeout):
                    if ser.in_waiting: # if there is data in the serial buffer
                        response = ser.readline().decode()
                        rospy.loginfo('response: '+response)
                        if "arm" in response:
                            rospy.loginfo("Arm MCU idenified!")
                            rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                            rospy.loginfo('took %f ms to find the Arm MCU', (time.time()-startConnecting)*1000)
                            return
        '''
        if len(ports) > 0:
            rospy.loginfo("%d USB device(s) detected", len(ports))
            for portObj in ports:
                port = portObj.name
                rospy.loginfo('Attempting to connect to /dev/'+port)
                ser = serial.Serial('/dev/' + port, baudrate)

                rospy.loginfo("clearing buffer...")
                while ser.in_waiting:
                    ser.readline()

                rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
                for i in range(5):
                    rospy.loginfo('attempt #%d...', i+1)
                    startListening = time.time()
                    ser.write(str.encode('who\n'))
                    while (time.time()-startListening < timeout):
                        if ser.in_waiting: # if there is data in the serial buffer
                            response = ser.readline().decode()
                            rospy.loginfo('response: '+response)
                            if "arm" in response:
                                rospy.loginfo("Arm MCU idenified!")
                                rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                                rospy.loginfo('took %f ms to find the Arm MCU', (time.time()-startConnecting)*1000)
                                return
        else:
            rospy.logerr("No USB devices recognized, exiting")
            sys.exit(0)

    elif uart:
        port = 'ttySAC0'
        rospy.loginfo('Attempting to connect to /dev/'+port)
        try:
            ser = serial.Serial('/dev/' + port, baudrate)
        except:
            rospy.logerr('No UART device recognized, terminating arm node')
            sys.exit(0)

        rospy.loginfo("clearing buffer...")
        while ser.in_waiting:
            ser.readline()

        rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
        for i in range(5):
            rospy.loginfo('attempt #%d...', i+1)
            startListening = time.time()
            ser.write(str.encode('who\n'))
            while (time.time()-startListening < timeout):
                if ser.in_waiting: # if there is data in the serial buffer
                    response = ser.readline().decode()
                    rospy.loginfo('response: '+response)
                    if "arm" in response:
                        rospy.loginfo("Arm MCU idenified!")
                        rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                        rospy.loginfo('took %f ms to find the Arm MCU', (time.time()-startConnecting)*1000)
                        return

    rospy.logerr('Incorrect MCU connected, terminating listener')
    sys.exit(0)

def handle_client(req):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo('received '+req.msg+' from GUI')
    msg = ArmRequestResponse()
    msg.response = 'no response'
    msg.success = False
    ser.write(str.encode(req.msg+'\n')) # ping the teensy

    startListening = time.time()
    # 500 ms timeout... could potentially be even less, needs testing
    # would be cool to see how long it takes between sending and receiving
    while (time.time()-startListening < timeout):
        if ser.in_waiting:
            try:
                data = ser.readline().decode()
                data_str = 'received '+data
                data_str += "from Arm Teensy at %f" % rospy.get_time()
                rospy.loginfo(data_str)
                if "Motor Angles" not in data_str:
                    msg.response = data
                    msg.success = True
                    rospy.loginfo('took %f ms to receive response', (time.time()-startListening)*1000)
                    break
                else:
                    publish_joint_states(data_str)
            except:
                rospy.logwarn('trouble reading from serial port')
    rospy.loginfo('sending '+msg.response+' back to GUI')
    return msg

def subscriber_callback(message):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo('sending: '+message.data)
    command = str.encode(message.data+'\n')
    ser.write(command) # send command to teensy

    startListening = time.time()
    # 500 ms timeout... could potentially be even less, needs testing
    while (time.time()-startListening < timeout):
        if ser.in_waiting:
            try:
                data_str = ser.readline().decode()
                if "Motor Angles" not in data_str:
                    data_str = "received "+data_str
                    data_str+=" at %f" % rospy.get_time()
                    rospy.loginfo(data_str)
                    break
                else:
                    publish_joint_states(data_str)
            except:
                rospy.logwarn('trouble reading from serial port')
    return

def publish_joint_states(message):
    # parse the data received from Teensy
    lhs,rhs = message.split('Motor Angles: ')
    lhs,rhs = rhs.split('\n')
    angles = lhs.split(', ')
    # create the message to be published
    msg = JointState()
    msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    try:
        for angle in angles:
            msg.position.append(float(angle))
    except:
        rospy.logwarn('trouble parsing motor angles:',sys.exc_info()[0])
        return

    # publish it
    anglePub.publish(msg)
    rospy.logdebug(msg.position)
    return

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    init_serial()

    angle_pub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    feedback_pub_topic = '/arm_feedback'
    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    # maybe make a global param for publishing and subscribing this data
    # the arm node publishes at a rate, this node listens and publishes,
    # plus the website publishes/requests commands at its own rate...
    rate = rospy.Rate(20) # 20hz

    data = ""; getTime = ""; data_str = ""

    subscribe_topic = 'arm_command'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)

    # service requests are implicitly handled but only at the rate the node publishes at
    try:
        while not rospy.is_shutdown():
            if ser.in_waiting:
                try:
                    data = ser.readline().decode()
                    #if data[0]=='@':
                    if 'Motor Angles' in data and data[0]=='@':
                        publish_joint_states(data)
                    elif data is not '':
                        getTime = "received at %f" % rospy.get_time()
                        data_str = data + ' ' + getTime
                        rospy.logdebug(data_str)
                        feedbackPub.publish(data)
                except:
                    rospy.logwarn('trouble reading from serial port')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        ser.close() # good practice to close the serial port
        # do I need to clear the serial buffer too?
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
