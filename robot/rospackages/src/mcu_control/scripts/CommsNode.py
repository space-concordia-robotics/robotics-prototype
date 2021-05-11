#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import struct

import rospy
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import JointState
from mcu_control.srv import *

from ArmCommands import arm_out_commands, arm_in_commands
from RoverCommands import rover_out_commands, rover_in_commands

ARM_SELECTED = 0
ROVER_SELECTED = 1
PDS_SELECTED = 2
SCIENCE_SELECTED = 3

in_commands = [arm_in_commands, rover_in_commands, None, None]
out_commands = [arm_out_commands, rover_out_commands, None, None]

def get_handler(commandId, selectedDevice):
    for in_command in in_commands[selectedDevice]:
        if commandId == in_command[1]:
            return in_command[2]
    return None

# Pin definitions
ARM_PIN = 11
WHEEL_PIN = 13
SCIENCE_PIN = 15

teensy_pins = [ARM_PIN, WHEEL_PIN, SCIENCE_PIN]

ser = serial.Serial('/dev/ttyACM0', 57600) # you sure this is good for the jetson tim?


def listen_arm():
    try:
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                commandID = ser.read()
                commandID = int.from_bytes(commandID, "big")
                handler = get_handler(commandID, ARM_SELECTED) # todo: pls change this to use whatever is actually selected
                # print("CommandID:", commandID)
                if handler == None:
                    # print("No command with ID ", commandID, " was found")
                    ser.read_until() # 0A
                    continue

                argsLen = ser.read()
                argsLen = int.from_bytes(argsLen, "big")
                # print("Number of bytes of arguments:", argsLen)
                args = None
                if argsLen > 0:
                    args = ser.read(argsLen)
                    # print("Raw arguments:", args)

                stopByte = ser.read()
                stopByte = int.from_bytes(stopByte, "big")
                # print("Stop byte:", stopByte)

                if stopByte != 16:
                    pass
                    # print("Warning : Invalid stop byte")

                try:
                    handler(args)
                except Exception as e:
                    print(e)
    except KeyboardInterrupt:
        print("Node shutting down due to operator shutting down the node.")
    ser.close()

def send_command(command_name, args, deviceToSendTo):
    for out_command in out_commands[deviceToSendTo]:
        if command_name == out_command[deviceToSendTo][0]:
            ser.write(len(args)) 
            ser.write(args) # todo: send each arg as a byte, also assume they will be strings so they will need to be casted to int/float first there's definitely more than one line of code to write
            return True
    return False

def subscriber_callback(message):
    ser = get_serial()
    rospy.loginfo('received: ' + message.data + ' command, sending to arm Teensy')
    command = str.encode(message.data + '\n')
    ser.write(command) # send command to teensy

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')
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
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    # serv = rospy.Service(service_name, ArmRequest, handle_client)
    listen_arm()


#def handle_client(req):
#    ser = get_serial()
#    global reqFeedback
#    global reqInWaiting
#    armResponse = ArmRequestResponse()
#    timeout = 0.1 # 100ms timeout
#    reqInWaiting=True
#    sinceRequest = time.time()
#    rospy.loginfo('received '+req.msg+' request from GUI, sending to arm Teensy')
#    ser.write(str.encode(req.msg+'\n')) # ping the teensy
#    while armResponse.success is False and (time.time()-sinceRequest < timeout):
#        if reqFeedback is not '':
#            print(reqFeedback)
#            for request in requests:
#                for response in requests[request]:
#                    if request == req.msg and response in reqFeedback:
#                        armResponse.response = reqFeedback
#                        armResponse.success = True #a valid request and a valid response from the
#                        break
#            if armResponse.success:
#                break
#            else:
#                armResponse.response += reqFeedback
#        rospy.Rate(100).sleep()
#    rospy.loginfo('took '+str(time.time()-sinceRequest)+' seconds, sending this back to GUI: ')
#    rospy.loginfo(armResponse)
#    reqFeedback=''
#    reqInWaiting=False
#    return armResponse


#def publish_joint_states(message):
#    # parse the data received from Teensy
#    lhs,message = message.split('Motor Angles: ')
#    angles = message.split(', ')
#    # create the message to be published
#    msg = JointState()
#    msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
#    try:
#        for angle in angles:
#            msg.position.append(float(angle))
#    except:
#        rospy.logwarn('trouble parsing motor angles')
#        return
#    # publish it
#    anglePub.publish(msg)
#    rospy.logdebug(msg.position)

#def stripFeedback(data):
#    startStrip='ARM '
#    endStrip='\r\n'
#    if data.startswith(startStrip) and data.count(startStrip) == 1:
#        if data.endswith(endStrip) and data.count(endStrip) == 1:
#            data,right = data.split(endStrip)
#            left,data = data.split(startStrip)
#            return data
#    return None

#if __name__ == '__main__':
#    node_name = 'arm_node'
#    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
#    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

#    search_success = init_serial(115200, mcuName)
    
#    if not search_success:
#        sys.exit(1)

#    angle_pub_topic = '/arm_joint_states'
#    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
#    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

#    v_bat_topic = '/battery_voltage'
#    rospy.loginfo('Beginning to publish to "'+v_bat_topic+'" topic')
#    vBatPub = rospy.Publisher(v_bat_topic, Float32, queue_size=10)

#    feedback_pub_topic = '/arm_feedback'
#    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
#    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

#    subscribe_topic = '/arm_command'
#    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
#    #I could have a topic that listens for JointStateso and constructs the message here...
#    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

#    service_name = '/arm_request'
#    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
#    serv = rospy.Service(service_name, ArmRequest, handle_client)

#    # service requests are implicitly handled but only at the rate the node publishes at
#    ser = get_serial()
#    global reqFeedback
#    reqFeedback=''
#    global reqInWaiting
#    reqInWaiting=False
#    try:
#        while not rospy.is_shutdown():
#            #if I try reading from the serial port inside callbacks, bad things happen
#            #instead I send the data elsewhere if required but only read from serial here.
#            #not sure if I need the same precautions when writing but so far it seems ok.
#            if ser.in_waiting:
#                data=''
#                feedback=None
#                try:
#                    data = ser.readline().decode()
#                    feedback = stripFeedback(data)
#                except:
#                    rospy.logwarn('trouble reading from serial port')
#                if feedback is not None:
#                    if 'Motor Angles' in feedback:
#                        #rospy.loginfo(feedback)
#                        publish_joint_states(feedback)
#                    elif 'battery voltage' in feedback:
#                        left,voltage = feedback.split('battery voltage: ')
#                        vBatPub.publish(float(voltage))
#                    else:
#                        #rospy.loginfo(feedback)
#                        if reqInWaiting:
#                            reqFeedback += feedback+'\r\n' #pass data to request handler
#                        else:
#                            if 'WARNING' in feedback:
#                                rospy.logwarn(feedback)
#                            #rospy.loginfo(feedback)
#                            feedbackPub.publish(feedback)
#                else:
#                    rospy.loginfo('got raw data: '+data)
#            rospy.Rate(100).sleep()
#    except rospy.ROSInterruptException:
#        pass

#    def shutdown_hook():
#        rospy.logwarn('This node ('+node_name+') is shutting down')
#        ser.close() # good practice to close the serial port
#        # do I need to clear the serial buffer too?
#        time.sleep(1) # give ROS time to deal with the node closing (rosbridge especially)

#    rospy.on_shutdown(shutdown_hook)
