#!/usr/bin/env python3

import sys
import traceback
import time
import re

import serial # pyserial
import serial.tools.list_ports # pyserial
#from robot.comms.uart import Uart
#import from mcuSerial import McuSerial # this isn't anything yet, just a copy of uart.py

import rospy
from std_msgs.msg import String, Float32, Float32MultiArray
from arm_control.srv import *

global ser # make global so it can be used in other parts of the code
mcuName = 'PDS'

# 300 ms timeout... could potentially be even less, needs testing
timeout = 0.3 # to wait for a response from the MCU

# todo: test ros+website over network with teensy
# todo: make a MCU serial class that holds the port initialization stuff and returns a reference?
# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
def init_serial():
    baudrate = 9600
    # in a perfect world, you can choose the baudrate
    rospy.loginfo('Using %d baud by default', baudrate)
    # in a perfect world, usb vs uart will be set by ROS params
    usb = False; uart = True
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) == 1:
        rospy.loginfo('Using UART by default')
    if len(myargv) > 1:
        if myargv[1] == 'uart':
            usb=False; uart=True
            rospy.loginfo('Using UART and 9600 baud by default')
        elif myargv[1] == 'usb':
            usb=True; uart=False
            rospy.loginfo('Using USB and 9600 baud by default')
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    global ser #make global so it can be used in other parts of the code

    ports = list(serial.tools.list_ports.comports())

    startConnecting = time.time()
    if usb:
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
                            if mcuName in response:
                                rospy.loginfo(mcuName+" MCU identified!")
                                rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                                rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
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
        for i in range(1,6):
            rospy.loginfo('attempt #%d...', i)
            startListening = time.time()
            ser.write(str.encode('who\n'))
            while (time.time()-startListening < timeout):
                if ser.in_waiting:
                    dat='';data=None
                    try:
                        dat = ser.readline().decode()
                        data = stripFeedback(dat)
                    except:
                        rospy.logwarn('trouble reading from serial port')
                    if data is not None:
                        if mcuName in data:
                            rospy.loginfo(mcuName+" MCU identified!")
                            rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                            rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
                            return
                    else:
                        rospy.loginfo('got raw message: '+dat)

    rospy.logerr('Incorrect MCU connected, terminating listener')
    sys.exit(0)

def subscriber_callback(message):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo('received: '+message.data+' command from GUI, sending to arm Teensy')
    command = str.encode(message.data+'\n')
    ser.write(command) # send command to teensy
    return

### currently the pds gives 3 types of feedback in one message
### so this needs to publish to 3 different topics:
### battery voltage, wheel motor current, battery temp
### currently the format is: "PDS,val,val,...,val\n"
### but 'PDS,' is stripped by stripFeedback()

### create a publish for each ex: currentPub.publish(msg) etc.
def publish_joint_states(message):
    # parse the data received from Teensy
    lhs,message = message.split('PDS,')
    voltages = message.split(',')
    currents = message.split('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,')
    temperatures = message.split('%.3f,%.3f,%.3f')
    # create the message to be published
    msg = JointState() ###
    msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work ##sets a timestamp
    try:
        for voltage in voltages:
            msg.position.append(float(voltage))
        for current in currents:
            msg.position.append(float(current))
        for temperature in temperatures:
            msg.position.append(float(temprature))
    except:
        rospy.logwarn('trouble parsing PDS data')
        return
    # publish it
    ##anglePub.publish(msg)
    voltagePub.publish(msg)
    currentPub.publish(msg)
    temperaturePub.publish(msg)
    rospy.logdebug(msg.position)
    return

def stripFeedback(data):
    startStrip='PDS,' 
    endStrip='\r\n'
    if data.startswith(startStrip) and data.count(startStrip) == 1:
        if data.endswith(endStrip) and data.count(endStrip) == 1:
            data,right = data.split(endStrip)
            left,data = data.split(startStrip)
            return data
    return None

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    init_serial()

    angle_pub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    v_bat_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "'+v_bat_topic+'" topic')
    vBatPub = rospy.Publisher(v_bat_topic, Float32, queue_size=10)

    feedback_pub_topic = '/arm_feedback'
    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    subscribe_topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    #I could have a topic that listens for JointStateso and constructs the message here...
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    # service requests are implicitly handled but only at the rate the node publishes at
    global ser
    try:
        while not rospy.is_shutdown():
            #if I try reading from the serial port inside callbacks, bad things happen
            #instead I send the data elsewhere if required but only read from serial here.
            #not sure if I need the same precautions when writing but so far it seems ok.
            if ser.in_waiting:
                data=''
                feedback=None
                try:
                    data = ser.readline().decode()
                    feedback = stripFeedback(data)
                except:
                    rospy.logwarn('trouble reading from serial port')
                if feedback is not None:
                    if 'Motor Angles' in feedback:
                        #rospy.loginfo(feedback)
                        publish_joint_states(feedback)
                    elif 'battery voltage' in feedback:
                        left,voltage = feedback.split('battery voltage: ')
                        vBatPub.publish(float(voltage))
                    else:
                        #rospy.loginfo(feedback)
                        if 'WARNING' in feedback:
                            rospy.logwarn(feedback)
                        #rospy.loginfo(feedback)
                        feedbackPub.publish(feedback)
                else:
                    rospy.loginfo('got raw data: '+data)
            rospy.Rate(100).sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node ('+node_name+') is shutting down')
        ser.close() # good practice to close the serial port
        # do I need to clear the serial buffer too?
        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)

