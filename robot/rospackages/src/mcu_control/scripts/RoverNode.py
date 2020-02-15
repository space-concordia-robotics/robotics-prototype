#!/usr/bin/env python3

import sys
import traceback
import time
import re

from robot.rospackages.src.mcu_control.scripts.serial_util import init_serial
import serial # pyserial
import serial.tools.list_ports # pyserial
#from robot.comms.uart import Uart
#import from mcuSerial import McuSerial # this isn't anything yet, just a copy of uart.py

import rospy
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import JointState
from mcu_control.srv import *

#global ser # make global so it can be used in other parts of the code
mcuName = 'Astro'

# 300 ms timeout... could potentially be even less, needs testing
timeout = 0.3 # to wait for a response from the MCU

# todo: test ros+website over network with teensy
# todo: make a MCU serial class that holds the port initialization stuff and returns a reference?
# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

requests = {
    'ping' : ['pong'],
    'who' : ['Happy Astro','Paralyzed Astro'],
    'activate' : ['ACTIVATED', 'Active'],
    'deactivate' : ['Inactive', 'inactive'],
    'open-loop' : ['loop status is: Open'],
    'close-loop' : ['loop status is: CLose'], # typo on purpose, LEAVE CLose ALONE (for now)
    'steer-off' : ['Wheel Control is Activated'],
    'steer-on' : ['Skid Steering is Activated'],
    'acc-on' : ['Limiter: Open'],
    'acc-off' : ['Limiter: CLose'],
    'reboot' : ['rebooting']
}
def handle_client(req):
    # feedback to tell if script itself is responsive
    if req.msg == 'ping':
        feedbackPub.publish('listener received ping request')

    global ser # specify that it's global so it can be used properly
    global reqFeedback
    global reqInWaiting
    roverResponse = ArmRequestResponse()
    timeout = 0.2 # 200ms timeout
    reqInWaiting=True
    sinceRequest = time.time()
    rospy.loginfo('received '+req.msg+' request from GUI, sending to rover Teensy')
    ser.write(str.encode(req.msg+'\n')) # ping the teensy
    while roverResponse.success is False and (time.time()-sinceRequest < timeout):
        if reqFeedback is not '':
            rospy.loginfo(reqFeedback)
            for request in requests:
                for response in requests[request]:
                    if request == req.msg and response in reqFeedback:
                        roverResponse.response = reqFeedback
                        roverResponse.success = True #a valid request and a valid response from the mcu
                        break
            if roverResponse.success:
                break
            else:
                roverResponse.response += reqFeedback
        rospy.Rate(100).sleep()
    rospy.loginfo('took '+str(time.time()-sinceRequest)+' seconds, sending this back to GUI: ')
    rospy.loginfo(roverResponse)
    reqFeedback=''
    reqInWaiting=False
    return roverResponse

def subscriber_callback(message):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo('received: '+message.data+' command from GUI, sending to rover Teensy')
    command = str.encode(message.data+'\n')
    ser.write(command) # send command to teensy
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return

def publish_joint_states(message):
    # parse the data received from Teensy
    try:
        lhs,message = message.split('Motor Speeds: ')
    except Exception as e:
        print("type error: " + str(e))
    speeds = message.split(', ')
    # create the message to be published
    msg = JointState()
    msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    try:
        for speed in speeds:
            #TODO: convert speed to SI units?
            msg.velocity.append(float(speed))
    except:
        rospy.logwarn('trouble parsing motor speeds')
        return
    # publish it
    speedPub.publish(msg)
    rospy.logdebug(msg.position)
    return

def publish_nav_states(message):
    heading,gps = message.split(' -- ')
    # parse heading, give -999 if invalid heading
    if 'OK' in heading:
        try:
            left,heading = heading.split(' ')
            roverHeading = float(heading)
        except:
            roverHeading = -999
            rospy.logwarn('bad string, got: ' + heading)
    else:
        if 'N/A' in heading:
            rospy.logwarn('IMU heading data unavailable')
        else:
            rospy.logwarn('bad string, got: ' + heading)
        roverHeading = -999
    # parse gps, give -999 for lat and long if invalid coords
    if 'OK' in gps:
        try:
            left,tempLat,tempLong = gps.split(' ')
            roverLatitude = float(tempLat)
            roverLongitude = float(tempLong)
        except:
            roverLatitude = roverLongitude = -999
            rospy.logwarn('bad string, got: ' + gps)
    else:
        if 'N/A' in gps:
            rospy.logwarn('GPS data unavailable')
        else:
            rospy.logwarn('bad string, got: ' + gps)
        roverLatitude = roverLongitude = -999

    msg = Point();
    msg.x = roverLatitude
    msg.y = roverLongitude
    msg.z = roverHeading
    navPub.publish(msg)
    return

def stripFeedback(data):
    startStrip='ASTRO '
    endStrip='\r\n'
    if data.startswith(startStrip) and data.count(startStrip) == 1:
        if data.endswith(endStrip) and data.count(endStrip) == 1:
            data,right = data.split(endStrip)
            left,data = data.split(startStrip)
            return data
    return None

if __name__ == '__main__':
    node_name = 'rover_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    init_serial(node_baudrate.get(rover),node_type.get(rover))

    speed_pub_topic = '/rover_joint_states'
    rospy.loginfo('Beginning to publish to "'+speed_pub_topic+'" topic')
    speedPub = rospy.Publisher(speed_pub_topic, JointState, queue_size=10)

    nav_pub_topic = '/rover_position'
    rospy.loginfo('Beginning to publish to "'+nav_pub_topic+'" topic')
    # will either make my own message type or use a standard one
    navPub = rospy.Publisher(nav_pub_topic, Point, queue_size=10)

    v_bat_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "'+v_bat_topic+'" topic')
    vBatPub = rospy.Publisher(v_bat_topic, Float32, queue_size=10)

    feedback_pub_topic = '/rover_feedback'
    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    subscribe_topic = '/rover_command'
    #first of all, it should subscribe to Twist and decide how to send it to the rover...
    #for now i might just have it subscribe to Strings tho
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)
    # the long way is for the gui to publish a twist and the node to convert it to throttle:steering
    # the short way is for the gui to send the command string directly. no Twist.

    service_name = '/rover_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    #TODO: change to RoverRequest? or just McuRequest? meaning change arm and mcu_control etc?
    serv = rospy.Service(service_name, ArmRequest, handle_client)

    # service requests are implicitly handled but only at the rate the node publishes at
    global ser
    global reqFeedback
    reqFeedback=''
    global reqInWaiting
    reqInWaiting=False
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
                if feedback is not None and feedback is not '':
                    if 'Motor Speeds' in feedback:
                        #rospy.loginfo(feedback)
                        publish_joint_states(feedback)
                    elif 'Battery voltage' in feedback:
                        try:
                            left,voltage = feedback.split('Battery voltage: ')
                        except Exception as e:
                            print("type error: " + str(e))
                        try:
                            vBatPub.publish(float(voltage))
                        except Exception as e:
                            print("type error: " + str(e))
                    elif 'GPS' in feedback: #use a better string? longer?
                        publish_nav_states(feedback)
                    elif 'Linear' in feedback:
                        pass
                    elif 'Desired' in feedback:
                        pass
                    else:
                        #rospy.loginfo(feedback)
                        if reqInWaiting:
                            reqFeedback += feedback+'\r\n' #pass data to request handler
                            #print(feedback)
                            #print(reqFeedback)
                        else:
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
