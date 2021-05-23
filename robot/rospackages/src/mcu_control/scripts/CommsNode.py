#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import struct
from collections import deque
import Jetson.GPIO as GPIO

import rospy
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import JointState
from robot.rospackages.src.mcu_control.srv import *

from robot.rospackages.src.mcu_control.scripts.ArmCommands import arm_out_commands, arm_in_commands
from robot.rospackages.src.mcu_control.scripts.RoverCommands import rover_out_commands, rover_in_commands
from robot.rospackages.src.mcu_control.scripts.CommsUtils import parse_command, get_arg_bytes
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt

ARM_SELECTED = 0
ROVER_SELECTED = 1
SCIENCE_SELECTED = 3
PDS_SELECTED = 4

in_commands = [arm_in_commands, rover_in_commands, None, None]
out_commands = [arm_out_commands, rover_out_commands, None, None]

def get_handler(commandId, selectedDevice):
    for in_command in in_commands[selectedDevice]:
        if commandId == in_command[1]:
            return in_command[2]
    return None

# Pin definitions Not correct as of May 23, 2021
ARM_PIN = 19
ROVER_PIN = 21
SCIENCE_PIN = 23
teensy_pins = [ARM_PIN, ROVER_PIN, SCIENCE_PIN]

GPIO.setmode(GPIO.BOARD)
GPIO.setup(teensy_pins, GPIO.OUT)

ser = None

STOP_BYTE = 0x0A

arm_queue= deque()
rover_queue = deque()
science_queue = deque()
command_queues = [arm_queue, rover_queue, science_queue]

def receive_message():
        start_time = time.time()
        while (time.time() - start_time < 0.5):

            if ser.in_waiting > 0:
                commandID = ser.read()
                commandID = int.from_bytes(commandID, "big")
                handler = get_handler(commandID, ARM_SELECTED) # todo: pls change this to use whatever is actually selected
                # print("CommandID:", commandID)
                if handler == None:
                    print("No command with ID ", commandID, " was found")
                    ser.read_until() # 0A

                argsLen = ser.read(2)
                # print(argsLen)
                argsLen = int.from_bytes(argsLen, "big")
                # print("Number of bytes of arguments:", argsLen)
                args = None
                if argsLen > 0:
                    args = ser.read(argsLen)
                    # print("Raw arguments:", args)

                stopByte = ser.read()
                stopByte = int.from_bytes(stopByte, "big")
                # print("Stop byte:", stopByte)

                if stopByte != STOP_BYTE:
                    pass
                    # print("Warning : Invalid stop byte")

                try:
                    handler(args)
                except Exception as e:
                    print(e)
                return


def get_command(command_name, deviceToSendTo):
    for out_command in out_commands[deviceToSendTo]:
        if command_name == out_command[deviceToSendTo][0]:
            return out_command

    return None


def send_command(cmd):
    command = get_command(cmd[0], cmd[2])
    if command is not None:
        commandID = command[1]

        # Must not forget to toggle TX2 RS-485 pin..

        ser.write(commandID)
        ser.write(get_arg_bytes(command).to_bytes(2, 'big'))

        data_types = [element[0] for element in command[2]]

        for argument in zip(cmd[1], data_types):
            data = argument[0]
            data_type = argument[1]

            if data_type == dt.ARG_UINT8_ID:
                ser.write(data)
            elif data_type == dt.ARG_FLOAT32_ID:
                ser.write(float(data)) #perhaps will fuck up

        # if args is not None and number_of_arguments != 0:
        #     arg_bytes = get_arg_bytes(args)
        #     for arg_byte in arg_bytes:
        #         ser.write(arg_byte)
        #     ser.write(STOP_BYTE)
        # return True

        # might need to flush serial if buffer gets overloaded
    return False


def arm_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to arm Teensy')
    command, args = parse_command(message.data)

    temp_list = [command, args, ARM_SELECTED]
    arm_queue.append(temp_list)


def main():
    try:
        while not rospy.is_shutdown():
            # Sends a command to each teensy if there is anything to send 
            for queue in command_queues:
                if len(queue) > 0:
                    cmd = queue.pop()
                    send_command(cmd)

            # Prompts each teensy to send something
            for teensy in teensy_pins:
                GPIO.output(teensy, 1)
                receive_message()
                GPIO.output(teensy, 0)

    except KeyboardInterrupt:
        print("Node shutting down due to operator shutting down the node.")
    ser.close()


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0', 57600) # you sure this is good for the jetson tim? # IDK, we will see


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

