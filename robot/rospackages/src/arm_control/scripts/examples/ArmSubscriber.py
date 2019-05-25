#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports #pyserial
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse #used as a placeholder for a proper service

global ser

# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

# set up connection to teensy. If only one USB device is connected it checks it's the arm teensy
# this should eventually check all usb ports or check uart like the original code does
def init_serial():
    global ser #make global so it can be used in other parts of the code

    ports = list(serial.tools.list_ports.comports())
    is_arm = False

    if len(ports) == 1:
        rospy.loginfo("1 USB device detected")
        port = ports[0].name
        ser = serial.Serial('/dev/' + port, 115200)

        rospy.loginfo("clearing buffer...")
        while ser.in_waiting:
            rospy.loginfo(ser.readline().decode())
            #ser.readline().decode() #test this to see if it works... we don't really need to see the output

        for i in 0, 3:
            who = ""
            rospy.loginfo("identifying MCU by sending 'who'")
            ser.write(str.encode("who\n"))

            # the following can probably be replaced by the comment block below but a timeout is needed
            # CRITICAL: give time for MCU to respond
            time.sleep(1)
            while ser.in_waiting:
                response = ser.readline().decode()
                rospy.loginfo("response: \"" + response.strip() + "\"")
                if "arm" in response.strip():
                    rospy.loginfo("Arm MCU idenified!")
                    is_arm = True
                    break

            ''' the following code probably responds faster but a timeout is needed:
            while True:
                if ser.in_waiting: # if there is data in the serial buffer
                    response = ser.readline().decode()
                    if "arm" in response:
                        rospy.loginfo("Arm MCU idenified!")
                        is_arm = True
                        break'''
    # todo: check for cases where multiple usb devices are connected
    #elif len(ports) >1:
    # todo: take uart possibility into account as well
    #if usb:
    #elif uart:
    else:
        rospy.loginfo("No USB devices recognized, exiting")
        sys.exit(0)

    if is_arm:
        rospy.loginfo("Connected to port: " + port)
    else:
        rospy.loginfo("Incorrect MCU connected, terminating listener")
        sys.exit(0)

# this used to be purely part of a publisher function but can actually be called within a service function
# in fact, I'm not even sure if it needs to be inside a function to work properly, I'm just copying examples and it works
# todo: instead of simply publishing the message, it should parse it and publish to 6 joint angle topics


# expects an empty request and returns a Trigger message with bool and string
# in this case, when the request is received it sends "ping" to the teensy and waits for pong indefinitely
# todo: implement a timeout
# todo: maybe publish/parse the string if it's not ping? this might cause errors but it's worth a shot
def handle_client(req):
    global ser # specify that it's global so it can be used properly
    rospy.loginfo("received ping request from client")
    data_str = ""
    response = TriggerResponse(
        success = False,
        message = "no response"
    )
    ser.write(str.encode("ping\n")) # ping the teensy
    while True:
        if ser.in_waiting:
            data_str = ser.readline().decode()
            if "pong" in data_str:
                data_str = "received "+data_str
                data_str+=" at %f" % rospy.get_time()
                response.success = True;
                response.message = data_str;
                break
    rospy.loginfo(response)
    # the Trigger handler expects a TriggerResponse object to be returned (goes back to client)
    # note that empty responses seem to break it (in python, according to google and my experience)
    # I was therefore forced to use a .srv with a response
    # todo: make a package for this and define my own .srv file
    return response

def subscriber_callback(data):
    rospy.loginfo(data)

if __name__ == '__main__':
    node_name = 'arm_listener'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" multidirectional node')
    
    init_serial()
    
    subscribe_topic = 'arm_command'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)
    # use this to block if nothing else needs to happen
    # and you don't want the node to close. for callbacks/handlers
    rospy.spin()