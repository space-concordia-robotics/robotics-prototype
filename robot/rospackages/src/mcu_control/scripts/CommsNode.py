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
from robot.rospackages.src.mcu_control.srv import *

from robot.rospackages.src.mcu_control.scripts.ArmCommands import arm_out_commands, arm_in_commands
from robot.rospackages.src.mcu_control.scripts.RoverCommands import rover_out_commands, rover_in_commands

import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt

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

ser = None

STOP_BYTE = 0x0A


def main():
    # this will:
    # - check for any new messages to send to teensies
    # - prompt and receive messages from teensies
    # - will try to balance the load between sending/receiving so that teensies don't get too much data at once.

    receive_message() # tim pls let me test
    return

def receive_message():
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

def get_command(command_name, deviceToSendTo):
    for out_command in out_commands[deviceToSendTo]:
        if command_name == out_command[deviceToSendTo][0]:
            return out_command

    return None

def send_command(command_name, args, deviceToSendTo):
    command = get_command(command_name, deviceToSendTo)
    if command is not None:
        commandID = command[1]

        ser.write(commandID)
        ser.write(get_arg_bytes(command))

        data_types = [element[0] for element in command[2]]

        for argument in zip(args, data_types):
            data = argument[0]
            data_type = argument[1]

            if data_type == dt.ARG_UINT8_ID:
                ser.write(data)
            elif data_type == dt.ARG_FLOAT32_ID:
                ser.write(float(data)) #perhaps will fuck up

        # make sure to also send the number of bytes

        # if args is not None and number_of_arguments != 0:
        #     arg_bytes = get_arg_bytes(args)
        #     for arg_byte in arg_bytes:
        #         ser.write(arg_byte)
        #     ser.write(STOP_BYTE)
        # return True
    return False

def arm_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to arm Teensy')
    command, args = parse_command(message)

    send_command(command, args, ARM_SELECTED)

def parse_command(message):
    full_command = message.split(" ")
    if full_command is not None:
        command = full_command[1]
        args = full_command[2:]
        newArgs = []
        for arg in args:
            try:
                newArgs.append(float(arg))
            except ValueError:
                newArgs.append(arg)

        return command, newArgs
    return None, []

def get_arg_bytes(command_tuple):
    return sum(element[1] for element in command_tuple[2])

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 57600) # you sure this is good for the jetson tim?

    node_name = 'comms_node'
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

    arm_command_topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+arm_command_topic+'" topic')
    sub = rospy.Subscriber(arm_command_topic, String, arm_command_callback)

    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    # serv = rospy.Service(service_name, ArmRequest, handle_client)
    main()

