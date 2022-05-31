#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import struct
from collections import deque
# import Jetson.GPIO as gpio
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import JointState
from robot.rospackages.src.mcu_control.srv import *
from mcu_control.msg import Voltage

from robot.rospackages.src.mcu_control.scripts.ArmCommands import arm_out_commands, arm_in_commands
from robot.rospackages.src.mcu_control.scripts.WheelsCommands import wheel_out_commands, wheel_in_commands
from robot.rospackages.src.mcu_control.scripts.DriveControls import *
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt
ARM_SELECTED = 0
ROVER_SELECTED = 1
PDS_SELECTED = 2
SCIENCE_SELECTED = 3

in_commands = [arm_in_commands, wheel_in_commands, None, None]
out_commands = [arm_out_commands, wheel_out_commands, None, None]

def get_handler(commandId, selectedDevice):
    for in_command in in_commands[selectedDevice]:
        if commandId == in_command[1]:
            return in_command[2]
    return None

# Pin definitions
NONE = [1,1,1]
ARM = [0,0,1]
ROVER = [0,0,0]
PDS = [0,1,0]
SCIENCE = [0,1,1]
TX2 = [1,0,0]
PIN_DESC=[ARM,ROVER,PDS,SCIENCE]
SW_PINS = [19,21,23]


ser = None

STOP_BYTE = 0x0A

arm_queue= deque()
rover_queue = deque()
science_queue = deque()

#gpio.setwarnings(False)
#gpio.setmode(gpio.BOARD)
#gpio.setup(SW_PINS, gpio.OUT)
#gpio.output(SW_PINS, NONE)

def twist_rover_callback(twist_msg):
    """ Handles the twist message and sends it to the wheel MCU """
    linear, angular = accelerate_twist(twist_msg)
    #print(linear)
    #rospy.loginfo('hi')
    args = twist_to_rover_command(linear, angular)
    #command = str.encode(string_command)
    #ser.write(command) # send move command to wheel teensy
    #ser.reset_input_buffer(

    rover_queue.append(['move_rover',args,ROVER_SELECTED])


    #ser.reset_output_buffer()

def main():

    try:
        while not rospy.is_shutdown():
            send_queued_commands()
            receive_message()

    except KeyboardInterrupt:
        print("Node shutting down due to shutting down node.")
    ser.close()


def send_queued_commands():
    if (len(arm_queue) > 0):
        arm_command = arm_queue.popleft()
        send_command(arm_command[0], arm_command[1], arm_command[2])

    if (len(rover_queue) > 0):
        rover_command = rover_queue.popleft()
        send_command(rover_command[0], rover_command[1], rover_command[2])

def receive_message():
    for device in range(2):
        #gpio.output(SW_PINS, PIN_DESC[device])
        if ser.in_waiting > 0:

            commandID = ser.read()
            print(commandID)
            commandID = int.from_bytes(commandID, "big")

            handler = get_handler(commandID, device)
            # print("CommandID:", commandID)
            if handler is None:
                print("No command with ID ", commandID, " was found")
                ser.read_until() # 0A

            argsLen = ser.read()
            # print(argsLen)
            argsLen = int.from_bytes(argsLen, "big")
            print("Number of bytes of arguments:", argsLen)
            args = None
            if argsLen > 0:
                args = ser.read(argsLen)
                # print("Raw arguments:", args)

            stopByte = ser.read()
            stopByte = int.from_bytes(stopByte, "big")
            # print("Stop byte:", stopByte)

            #gpio.output(SW_PINS, NONE)

            if stopByte != STOP_BYTE:
                # print("Warning : Invalid stop byte")
                pass

            try:
                handler(args)
            except Exception as e:
                print(e)


def get_command(command_name, deviceToSendTo):
    for out_command in out_commands[deviceToSendTo]:
        if command_name == out_command[0]:
            return out_command

    return None

def send_command(command_name, args, deviceToSendTo):
    command = get_command(command_name, deviceToSendTo)
    if command is not None:
        commandID = command[1]

        #gpio.output(SW_PINS, TX2)

        ser.write(commandID.to_bytes(1, 'big'))
        ser.write(get_arg_bytes(command).to_bytes(1, 'big'))
        #arg_length = len(command[2]).to_bytes(1,'big')
        #ser.write(arg_length)
        data_types = [element[0] for element in command[2]]
        for argument in zip(args, data_types):
            data = argument[0]
            data_type = argument[1]

            if data_type == dt.ARG_UINT8_ID:
                arg_int = int(data)
                if arg_int < 0:
                    arg_int = arg_int + 256

                ser.write(arg_int.to_bytes(1,'big'))

            elif data_type == dt.ARG_FLOAT32_ID:
                ser.write(bytearray(struct.pack(">f", data))) # This is likely correct now, will need to consult

        ser.write(STOP_BYTE.to_bytes(1, 'big'))
        ser.flush()
        #gpio.output(SW_PINS, NONE)
    return False

def arm_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to arm Teensy')
    command, args = parse_command(message)

    temp_struct = [command, args, ARM_SELECTED]
    arm_queue.append(temp_struct)

def rover_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to wheels Teensy')
    command, args = parse_command(message)

    temp_struct = [command, args, ROVER_SELECTED]
    rover_queue.append(temp_struct)


def rover_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to wheels Teensy')
    command, args = parse_command(message)

    temp_struct = [command, args, ROVER_SELECTED]
    rover_queue.append(temp_struct)


def parse_command(message):
    full_command = message.data.split(" ")
    if full_command is not None:
        command = full_command[0]
        args = full_command[1:]
        newArgs = []
        for arg in args:
            newArgs.append(float(arg))

        return command, newArgs
    return None, []

def get_arg_bytes(command_tuple):
    return sum(element[1] for element in command_tuple[2])

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 57600, timeout = 1)

    node_name = 'comms_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    angle_pub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    v_bat_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "'+v_bat_topic+'" topic')
    vBatPub = rospy.Publisher(v_bat_topic, Voltage, queue_size=10)

    # feedback_pub_topic = '/arm_feedback'
    # rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    # feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    arm_command_topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+arm_command_topic+'" topic')
    sub = rospy.Subscriber(arm_command_topic, String, arm_command_callback)

    rover_command_topic = '/rover_command'
    rospy.loginfo('Beginning to subscribe to "'+rover_command_topic+'" topic')
    sub = rospy.Subscriber(rover_command_topic, String, rover_command_callback)
    
    rover_twist_topic = '/rover_cmd_vel'
    rover_twist_sub = rospy.Subscriber(rover_twist_topic, Twist,twist_rover_callback)
    rospy.loginfo('Beginning to subscribe to "'+rover_twist_topic + '" topic')

    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    # serv = rospy.Service(service_name, ArmRequest, handle_client)
    main()

