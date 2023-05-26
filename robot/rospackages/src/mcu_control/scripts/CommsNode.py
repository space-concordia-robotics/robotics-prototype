#!/usr/bin/env python3
# *** To run in local mode, add 'local' as a cmdline argument ***

import sys
import traceback
import time
import re
import serial
import struct
from collections import deque
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import JointState
from robot.rospackages.src.mcu_control.srv import *

from robot.rospackages.src.mcu_control.scripts.CommandParser import *
from robot.rospackages.src.mcu_control.scripts.ArmCommands import arm_out_commands, arm_in_commands
from robot.rospackages.src.mcu_control.scripts.WheelsCommands import wheel_out_commands, wheel_in_commands
from robot.rospackages.src.mcu_control.scripts.ScienceCommands import science_out_commands, science_in_commands
from robot.rospackages.src.mcu_control.scripts.DriveControls import *
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt

if len(sys.argv) > 0:
    local_mode = "local" in sys.argv
    local_selected_device = SCIENCE_SELECTED
else:
    local_mode = False

if local_mode:
    # imports an object called gpio that does nothing when
    # any method is called on it, to stub gpio
    from robot.rospackages.src.mcu_control.scripts.CommandParser import emptyObject as gpio
else:
    import Jetson.GPIO as gpio


in_commands = [arm_in_commands, wheel_in_commands, None, science_in_commands]

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
SW_PINS = [15,13,11]


ser = None
# HACK bcs science teensy is USB
ser_science = None
ser_hardware = None

STOP_BYTE = 0x0A

arm_queue= deque()
rover_queue = deque()
science_queue = deque()

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(SW_PINS, gpio.OUT)
gpio.output(SW_PINS, NONE)

def twist_rover_callback(twist_msg):
    """ Handles the twist message and sends it to the wheel MCU """
    linear, angular = accelerate_twist(twist_msg)
    args = twist_to_rover_command(linear, angular)
    rover_queue.append(['move_rover',args,ROVER_SELECTED])

def main():

    try:
        while not rospy.is_shutdown():
            send_queued_commands()
            receive_message()

    except KeyboardInterrupt:
        print("Node shutting down due to shutting down node.")
    ser.close()


def send_queued_commands():
    global ser, ser_science

    if (len(arm_queue) > 0):
        arm_command = arm_queue.popleft()
        send_command(arm_command[0], arm_command[1], arm_command[2])

    if (len(rover_queue) > 0):
        rover_command = rover_queue.popleft()
        send_command(rover_command[0], rover_command[1], rover_command[2])
    
    if (len(science_queue) > 0):
        # HACK for science on rover
        if not local_mode:
            ser = ser_science
        science_command = science_queue.popleft()
        send_command(science_command[0], science_command[1], science_command[2])
        if not local_mode:
            ser = ser_hardware

if local_mode:
    # in local mode can only simulate connection to one device
    device_range = [local_selected_device]
else:
    device_range = range(4)

def receive_message():
    for device in device_range:
        if not local_mode:
            if device == SCIENCE_SELECTED:
                ser = ser_science
            else:
                ser = ser_hardware

        gpio.output(SW_PINS, PIN_DESC[device])
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
            # print("Number of bytes of arguments:", argsLen)
            args = None
            if argsLen > 0:
                args = ser.read(argsLen)
                # print("Raw arguments:", args)

            stopByte = ser.read()
            stopByte = int.from_bytes(stopByte, "big")
            # print("Stop byte:", stopByte)

            gpio.output(SW_PINS, NONE)

            if stopByte != STOP_BYTE:
                # print("Warning : Invalid stop byte")
                pass

            try:
                handler(args)
            except Exception as e:
                print(e)

def send_command(command_name, args, deviceToSendTo):
    print("port: ", ser.port)
    command = get_command(command_name, deviceToSendTo)
    if command is not None:
        commandID = command[1]

        gpio.output(SW_PINS, TX2)

        ser.write(commandID.to_bytes(1, 'big'))

        if commandID != 10:
            ser.write(get_arg_bytes(command).to_bytes(1, 'big'))
        
        #arg_length = len(command[2]).to_bytes(1,'big')
        #ser.write(arg_length)
        data_types = [element[0] for element in command[2]]

        # MEGA STUPID WORKAROUND, DIRE SITUATION, PLEASE DON'T REUSE THIS
        if commandID == 10:
            ser.write((12).to_bytes(1, 'big'))
            args, data_types = move_wheels_dumb_workaround(args)
            print(f'args: {args} | dataTypes: {data_types}')

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
        gpio.output(SW_PINS, NONE)
    return False

def move_wheels_dumb_workaround(args):
    # Wheel command is different on teensy, convert to ("move_wheels", 10, 12 * [dt.ARG_UINT8])
    newArgs = []

    for i in range(0,6):
        newArgs.append(int(args[i] < 0))
        newArgs.append(int(abs(args[i])))

    return newArgs, 12 * [dt.ARG_UINT8_ID]

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

def science_command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to science Teensy')
    command, args = parse_command(message)

    temp_struct = [command, args, SCIENCE_SELECTED]
    science_queue.append(temp_struct)

def get_arg_bytes(command_tuple):
    return sum(element[1] for element in command_tuple[2])

if __name__ == '__main__':
    if local_mode:
        ser = serial.Serial('/dev/ttyACM0', 57600, timeout = 1)
    else:
        start = time.time()
        # loop for a max of 60sec
        while time.time() - start < 60:
            try:
                ser_science = serial.Serial('/dev/ttyACM0', 57600, timeout = 1)
                ser_hardware = serial.Serial('/dev/ttyTHS2', 57600, timeout = 1)
                ser = ser_hardware
                break
            except serial.SerialException as e:
                rospy.logwarn("Retrying connection to science, error occured " + str(e))
                time.sleep(1)

    node_name = 'comms_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    angle_pub_topic = '/arm_joint_states'
    rospy.loginfo('Beginning to publish to "'+angle_pub_topic+'" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    feedback_pub_topic = '/arm_feedback'
    rospy.loginfo('Beginning to publish to "'+feedback_pub_topic+'" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

    arm_command_topic = '/arm_command'
    rospy.loginfo('Beginning to subscribe to "'+arm_command_topic+'" topic')
    sub = rospy.Subscriber(arm_command_topic, String, arm_command_callback)

    rover_command_topic = '/rover_command'
    rospy.loginfo('Beginning to subscribe to "'+rover_command_topic+'" topic')
    sub = rospy.Subscriber(rover_command_topic, String, rover_command_callback)
    
    rover_twist_topic = '/rover_cmd_vel'
    rover_twist_sub = rospy.Subscriber(rover_twist_topic, Twist,twist_rover_callback)
    rospy.loginfo('Beginning to subscribe to "'+rover_twist_topic + '" topic')

    science_command_topic = '/science_command'
    rospy.loginfo('Beginning to subscribe to "'+science_command_topic+'" topic')
    sub = rospy.Subscriber(science_command_topic, String, science_command_callback)    

    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    # serv = rospy.Service(service_name, ArmRequest, handle_client)
    main()


