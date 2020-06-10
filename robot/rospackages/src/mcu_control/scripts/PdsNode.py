#! /usr/bin/env python3

import sys
import traceback
import time
import re

from robot.rospackages.src.mcu_control.scripts.SerialUtil import init_serial, get_serial

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, FanSpeeds, Voltage, Currents
from mcu_control.srv import *

mcuName = 'PDS'

# todo: test ros+website over network with teensy
# todo: make a MCU serial class that holds the port initialization stuff and returns a reference?
# todo: put similar comments and adjustments to code in the publisher and server demo scrips once finalized

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
    ser = get_serial()
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
                        pdsResponse.success = True # a valid request and a valid response from the
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

def pds_command_subscriber_callback(message):
    ser = get_serial()
    rospy.loginfo('received: ' + message.data + ' command from GUI, sending to PDS')
    command = str.encode(message.data + '\n')
    ser.write(command)  # send command to PDS
    return

def publish_pds_data(message):
    # parse the data received from PDS
    # converts message from string to float
    dataPDS = message.split(',')  # returns an array of ALL data from the PDS

    # create the message to be published
    voltage = Voltage()
    current = Currents()
    temp = ThermistorTemps()
    fanSpeed = FanSpeeds()

    try:
        voltage.data = float(dataPDS[0])

        current.effort = [float(data) for data in dataPDS[1:7]] # sexy list comprehension
        currents = ','.join([str(x) for x in current.effort])

        temp.therm1 = float(dataPDS[7])
        temp.therm2 = float(dataPDS[8])
        temp.therm3 = float(dataPDS[9])
        temps = ','.join([str(x) for x in [temp.therm1, temp.therm2, temp.therm3]]) # more sexy

        fanSpeed.fan1 = float(dataPDS[10])
        fanSpeed.fan2 = float(dataPDS[11])
        fanSpeeds = ','.join([str(x) for x in [fanSpeed.fan1, fanSpeed.fan2]])

        firstFlag = dataPDS[12].split(' ')
        flagsMsg = ','.join([str(x) for x in firstFlag + [data[13], dataPDS[14].strip('\r')]])

        rospy.loginfo('voltage= ' + str(voltage.data))
        rospy.loginfo('temps= ' + str(temps))
        rospy.loginfo('currents= ' + str(currents))
        rospy.loginfo('fan speeds= ' + str(fanSpeeds))
        
        voltagePub.publish(voltage)
        currentPub.publish(current)
        tempPub.publish(temp)
        fanSpeedsPub.publish(fanSpeed)
        flagsPub.publish(flagsMsg)

    except Exception as e:
        print("type error: " + str(e))
        rospy.logwarn('trouble parsing PDS sensor data')
        return
    return

def stripFeedback(data):
    startStrips = ['PDS ', 'Command', 'Motor']
    endStrips = ['\r\n', '\n']
    for strip in startStrips:
        if data.startswith(strip) and data.count(strip) == 1:
            data = re.split('\r\n|\n', data)[0]
            return data
    return None

if __name__ == '__main__':
    node_name = 'pds_node'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    voltage_pub_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "' + voltage_pub_topic + '" topic')
    voltagePub = rospy.Publisher(voltage_pub_topic, Voltage, queue_size=10)

    current_pub_topic = '/wheel_motor_currents'
    rospy.loginfo('Begining to publish to "' + current_pub_topic + '" topic')
    currentPub = rospy.Publisher(current_pub_topic, Currents, queue_size=10)

    temp_pub_topic = '/battery_temps'
    rospy.loginfo('Begining to publish to "' + temp_pub_topic + '" topic')
    tempPub = rospy.Publisher(temp_pub_topic, ThermistorTemps, queue_size=10)

    fan_speeds_pub_topic = '/fan_speeds'
    rospy.loginfo('Beginning to publish to "' + fan_speeds_pub_topic + '" topic')
    fanSpeedsPub = rospy.Publisher(fan_speeds_pub_topic, FanSpeeds, queue_size=10)

    error_flags_topic = '/pds_flags'
    rospy.loginfo('Beginning to publish to "' + error_flags_topic + '" topic')
    flagsPub = rospy.Publisher(error_flags_topic, String, queue_size=10)

    feedback_pub_topic = '/pds_feedback'
    rospy.loginfo('Beginning to publish to "' + feedback_pub_topic + '" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    subscribe_topic = '/pds_command'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, String, pds_command_subscriber_callback)

    service_name = '/pds_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)

    search_success = init_serial(19200, mcuName)

    if not search_success:
        sys.exit(1)

    # service requests are implicitly handled but only at the rate the node publishes at
    ser = get_serial()
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
                        feedback=feedback.split('PDS ')[1]
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
