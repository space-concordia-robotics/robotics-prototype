#! /usr/bin/env python3

import sys
import traceback
import time
import re
import math

import serial  # pyserial
import serial.tools.list_ports  # pyserial
#from robot.comms.uart import Uart
#import from mcuSerial import McuSerial # this isn't anything yet, just a copy of uart.py

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from mcu_control.srv import *

global ser  # make global so it can be used in other parts of the code
mcuName = 'PDS'

# 300 ms timeout... could potentially be even less, needs testing
timeout = 1  # to wait for a response from the MCU

# todo: test ros+website over network with teensy
# todo: make a MCU serial class that holds the port initialization stuff and returns a reference?
# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
def init_serial():
    baudrate = 9600
    # in a perfect world, you can choose the baudrate
    rospy.loginfo('Using %d baud by default', baudrate)
    # in a perfect world, usb vs uart will be set by ROS params
    usb = False
    uart = True
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) == 1:
        rospy.loginfo('Using UART by default')
    if len(myargv) > 1:
        if myargv[1] == 'uart':
            usb = False
            uart = True
            rospy.loginfo('Using UART and 9600 baud by default')
        elif myargv[1] == 'usb':
            usb = True
            uart = False
            rospy.loginfo('Using USB and 9600 baud by default')
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    global ser  #make global so it can be used in other parts of the code

    ports = list(serial.tools.list_ports.comports())

    startConnecting = time.time()
    if usb:
        baudrate = 19200
        if len(ports) > 0:
            rospy.loginfo("%d USB device(s) detected", len(ports))
            for portObj in ports:
                port = portObj.name
                rospy.loginfo('Attempting to connect to /dev/' + port)
                ser = serial.Serial('/dev/' + port, baudrate)

                rospy.loginfo("clearing buffer...")
                while ser.in_waiting:
                    ser.readline()

                rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout * 1000)
                for i in range(5):
                    rospy.loginfo('attempt #%d...', i + 1)
                    startListening = time.time()
                    ser.write(str.encode('who\n'))
                    while time.time() - startListening < timeout:
                        if ser.in_waiting:  # if there is data in the serial buffer
                            response = ser.readline().decode()
                            rospy.loginfo('response: ' + response)
                            if mcuName in response:
                                rospy.loginfo(mcuName + " MCU identified!")
                                rospy.loginfo(
                                    'timeout: %f ms', (time.time() - startListening) * 1000
                                )
                                rospy.loginfo(
                                    'took %f ms to find the ' + mcuName + ' MCU',
                                    (time.time() - startConnecting) * 1000
                                )
                                return
        else:
            rospy.logerr("No USB devices recognized, exiting")
            sys.exit(0)

    elif uart:
        port = 'ttySAC0'
        rospy.loginfo('Attempting to connect to /dev/' + port)
        try:
            ser = serial.Serial('/dev/' + port, baudrate)
        except:
            rospy.logerr('No UART device recognized, terminating PDS node')
            sys.exit(0)

        rospy.loginfo("clearing buffer...")
        while ser.in_waiting:
            ser.readline()

        rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout * 1000)
        for i in range(1, 6):
            rospy.loginfo('attempt #%d...', i)
            startListening = time.time()
            ser.write(str.encode('who\n'))
            while time.time() - startListening < timeout:
                if ser.in_waiting:
                    dat = ''
                    data = None
                    try:
                        #dat = ser.readline().decode()
                        data = ser.readline().decode()
                        #data = stripFeedback(dat)
                    except Exception as e:
                        print("type error: " + str(e))
                        rospy.logwarn('trouble reading from serial port')
                    if data is not None:
                        if mcuName in data:
                            rospy.loginfo(mcuName + " MCU identified!")
                            rospy.loginfo('timeout: %f ms', (time.time() - startListening) * 1000)
                            rospy.loginfo(
                                'took %f ms to find the ' + mcuName + ' MCU',
                                (time.time() - startConnecting) * 1000
                            )
                            return
                    else:
                        rospy.loginfo('got raw message: ' + dat)

    rospy.logerr('Incorrect MCU connected, terminating listener')
    sys.exit(0)

requests = {
    'PDS T 1' : ['ON'],
    'PDS T 0' : ['OFF'],
    #'who' : ['pds'], #@TODO: MCU code can't handle this request, fix it
    'PDS M 1 1' : ['toggling'],
    'PDS M 1 0' : ['toggling'],
    'PDS M 2 1' : ['toggling'],
    'PDS M 2 0' : ['toggling'],
    'PDS M 3 1' : ['toggling'],
    'PDS M 3 0' : ['toggling'],
    'PDS M 4 1' : ['toggling'],
    'PDS M 4 0' : ['toggling'],
    'PDS M 5 1' : ['toggling'],
    'PDS M 5 0' : ['toggling'],
    'PDS M 6 1' : ['toggling'],
    'PDS M 6 0' : ['toggling']
}
def handle_client(req):
    global ser # specify that it's global so it can be used properly
    global reqFeedback
    global reqInWaiting
    pdsResponse = ArmRequestResponse()
    timeout = 0.3 # 300ms timeout
    reqInWaiting = True
    sinceRequest = time.time()
    rospy.loginfo('received '+req.msg+' request from GUI, sending to PDS MCU')
    ser.write(str.encode(req.msg+'\n')) # ping the teensy
    while pdsResponse.success is False and (time.time()-sinceRequest < timeout):
        if reqFeedback is not '':
            for request in requests:
                for response in requests[request]:
                    if request == req.msg and response in reqFeedback:
                        pdsResponse.response = reqFeedback
                        pdsResponse.success = True #a valid request and a valid response from the
                        break
            if pdsResponse.success:
                break
            else:
                pdsResponse.response = reqFeedback
        rospy.Rate(100).sleep()
    rospy.loginfo('took '+str(time.time()-sinceRequest)+' seconds, sending this back to GUI: ')
    rospy.loginfo(pdsResponse)
    reqFeedback=''
    reqInWaiting=False
    return pdsResponse

def subscriber_callback(message):
    global ser  # specify that it's global so it can be used properly
    rospy.loginfo('received: ' + message.data + ' command from GUI, sending to PDS')
    command = str.encode(message.data + '\n')
    ser.write(command)  # send command to PDS
    return

def publish_pds_data(message):
    # parse the data received from PDS
    # converts message from string to float
    dataPDS = message.split(',')  ###returns an array of ALL data from the PDS
    # create the message to be published
    voltage = Float32()
    current = JointState()
    temp = Point()
    fanSpeeds = Point()
    try:
        for i in range(12):
            if i < 1:
                voltage.data = float(dataPDS[i])
                rospy.loginfo('voltage=' + dataPDS[i])
            elif i < 7:
                current.effort.append(float(dataPDS[i]))
            elif i < 10:
                temp.x = checkTempErrorCase(float(dataPDS[7]))
                temp.y = checkTempErrorCase(float(dataPDS[8]))
                temp.z = checkTempErrorCase(float(dataPDS[9]))
                i = 9
            else:
                fanSpeeds.x = float(dataPDS[10])
                fanSpeeds.y = float(dataPDS[11])

        temps = ''
        for i in [dataPDS[7], dataPDS[8], dataPDS[9]]:
            temps += i.strip() + ','
        temps = temps[:-1]
        rospy.loginfo('temps=' + temps)

        # motor currents
        currents = ''
        for i in current.effort:
            currents += str(i).strip() + ','
        currents = currents[:-1]

        rospy.loginfo('currents=' + currents)
        # battery voltage
        voltagePub.publish(voltage)
        # 6 motor currents from M0-M5
        currentPub.publish(current)
        # temperatures of the battery
        tempPub.publish(temp)
        fanSpeedsPub.publish(fanSpeeds)
        nada,firstFlag = dataPDS[12].split(' ')
        flagsMsg = firstFlag+','+dataPDS[13]+','+dataPDS[14].strip('\r')
        flagsPub.publish(flagsMsg)
    except Exception as e:
        print("type error: " + str(e))
        rospy.logwarn('trouble parsing PDS sensor data')
        return
    return

# This function checks if the temp values from serial are equal to 999.9 (default error state) and replaces them with 'N/A' if they are.
# It is necessary because if the temperature is at the default error state, the front end will scream at you with a horrendous windows sound warning. 
# Changing it to 'N/A' prevents being assaulted by UI.
def checkTempErrorCase(x):
    if math.isclose(999.9, x, rel_tol=0.0001, abs_tol=0.0):
        return 'N/A'
    else:
        return x

def stripFeedback(data):
    startStrips = ['PDS ', 'Command', 'Motor']
    endStrips = ['\r\n', '\n']
    for strip in startStrips:
        if data.startswith(strip) and data.count(strip) == 1:
            try:
                data, right = data.split(endStrips[0])
            except:
                pass
            try:
                data, right = data.split(endStrips[1])
            except:
                pass
            #left, data = data.split(startStrip)
            return data
    return None

if __name__ == '__main__':
    node_name = 'pds_node'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    voltage_pub_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "' + voltage_pub_topic + '" topic')
    voltagePub = rospy.Publisher(voltage_pub_topic, Float32, queue_size=10)

    current_pub_topic = '/wheel_motor_currents'
    rospy.loginfo('Begining to publish to "' + current_pub_topic + '" topic')
    currentPub = rospy.Publisher(current_pub_topic, JointState, queue_size=10)

    temp_pub_topic = '/battery_temps'
    rospy.loginfo('Begining to publish to "' + temp_pub_topic + '" topic')
    tempPub = rospy.Publisher(temp_pub_topic, Point, queue_size=10)

    fan_speeds_pub_topic = '/fan_speeds'
    rospy.loginfo('Beginning to publish to "' + fan_speeds_pub_topic + '" topic')
    fanSpeedsPub = rospy.Publisher(fan_speeds_pub_topic, Point, queue_size=10)

    error_flags_topic = '/pds_flags'
    rospy.loginfo('Beginning to publish to "' + error_flags_topic + '" topic')
    flagsPub = rospy.Publisher(error_flags_topic, String, queue_size=10)

    feedback_pub_topic = '/pds_feedback'
    rospy.loginfo('Beginning to publish to "' + feedback_pub_topic + '" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    subscribe_topic = '/pds_command'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    service_name = '/pds_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)

    init_serial()

    # service requests are implicitly handled but only at the rate the node publishes at
    global ser
    global reqFeedback
    reqFeedback = ''
    global reqInWaiting
    reqInWaiting = False
    try:
        while not rospy.is_shutdown():
            #if I try reading from the serial port inside callbacks, bad things happen
            #instead I send the data elsewhere if required but only read from serial here.
            #not sure if I need the same precautions when writing but so far it seems ok.
            if ser.in_waiting:
                data = ''
                feedback = None
                try:
                     data = ser.readline().decode()
                     feedback = stripFeedback(data)
                except Exception as e:
                     print("type error: " + str(e))
                     rospy.logwarn('trouble reading from serial port')
                if feedback is not None:
                    if feedback.startswith('PDS '):
                        nada,feedback=feedback.split('PDS ')
                        publish_pds_data(feedback)
                    else:
                        if reqInWaiting:
                            reqFeedback += feedback+'\r\n' #pass data to request handler
                        else:
                            if 'WARNING' in feedback:
                                rospy.logwarn(feedback)
                            #rospy.loginfo(feedback)
                            feedbackPub.publish(feedback)
                else:
                    rospy.loginfo('got raw data: ' + data)
            rospy.Rate(100).sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        ser.close()  # good practice to close the serial port
        # do I need to clear the serial buffer too?
        time.sleep(1)  # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
