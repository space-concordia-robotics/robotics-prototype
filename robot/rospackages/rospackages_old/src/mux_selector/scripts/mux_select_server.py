#!/usr/bin/env python3

import rospy
from mux_selector.srv import *

current_channel = 0

def handle_mux_select(req):
    if req.device == '?':
        global current_channel

        if local:
            return "Current: " + str(current_channel) + '\n'
        else:
            current_channel = get_current_channel()
            return "Current: " + str(current_channel) + '\n'

    response = "Switched MUX select to device {:s}".format(req.device)

    # call bash function to do the select
    response += "\n" + str(select_device(req.device))

    return response

def get_current_channel():
    global local
    global current_channel
    gpio_dir = '/sys/class/gpio'
    s1 = 0
    s0 = 0
    dev = 0

    if not local:
        # s1
        with open(gpio_dir + '/gpio30/value', 'r') as f:
            s1 = f.read()
        # s0
        with open(gpio_dir + '/gpio18/value', 'r') as f:
            s0 = f.read()

        # remove leading/trailing whitespace for comparisons
        s1 = s1.strip()
        s0 = s0.strip()

        if s1 == "0" and s0 == "0":
            dev = 3
        elif s1 == "0" and s0 == "1":
            dev = 2
        elif s1 == "1" and s0 == "0":
            dev = 1
        elif s1 == "1" and s0 == "1":
            dev = 0
    else:
        dev = current_channel

    return dev

def select_device(device):
    global current_channel
    global local
    s0_val = 0
    s1_val = 0

    if not device.isdecimal():
        print("Device argument must be numeric value")
        return

    device = int(device)

    # s1 most significant bit, s0 least significant bit
    # s0 --> gpio 18 (physical 15), s1 --> gpio 30 (physical 19)
    # NOTE: on the final PCB the select logic will be inverted
    # this is why selecting device 0 is mapped to (1, 1) --> (0, 0)
    if not device in (0, 1, 2, 3):
        print("Invalid value, must be 0 or 1")
    else:
        if device == 0:
            print("Selecting device 0: Rover")
            s0_val = 1
            s1_val = 1
        elif device == 1:
            print("Selecting device 1: Arm")
            s0_val = 0
            s1_val = 1
        elif device == 2:
            print("Selecting device 2: Science Payload")
            s0_val = 1
            s1_val = 0
        elif device == 3:
            print("Selecting device 3: PDS")
            s0_val = 0
            s1_val = 0

    current_channel = device
    gpio_dir = "/sys/class/gpio"

# consider removing following lines to make given "test" argument flag present
# or better: user ros params as per tatums suggestion
    if not local:
        # s0
        with open(gpio_dir + "/gpio18/value", "w") as f:
            f.write(str(s0_val))
        # s1
        with open(gpio_dir + "/gpio30/value", "w") as f:
            f.write(str(s1_val))

    s0_state = "s0: " + str(s0_val)
    s1_state = "s1: " + str(s1_val)
    print(s0_state)
    print(s1_state)


    return s0_state + "\n" + s1_state

def get_invalid_args(whitelisted_args, actual_args):
    ''' Returns list of unrecognized arguments '''
    wrong_args = []
    for arg in actual_args:
        if arg not in whitelisted_args:
            wrong_args.append(arg)
    return wrong_args

def remove_ros_arguments(args):
    ''' Returns list of arguments without ROS arguments '''
    non_ros_args = []
    for arg in args:
        if ":=" not in arg:
            non_ros_args.append(arg)
    return non_ros_args

def mux_select_server():
    global local
    local = False
    non_ros_args = remove_ros_arguments(sys.argv)
    wrong_args = get_invalid_args(['rosrun', __file__, 'local'], non_ros_args)
    if len(wrong_args) > 0:
        print('An error with passed arguments: ')
        for arg in wrong_args:
            print(arg)
        print('Exiting... Perhaps you mean\'t \'local\'?')
        sys.exit(0)
    if 'local' in non_ros_args:
        local = True

    rospy.init_node('mux_select_server')
    s = rospy.Service('mux_select', SelectMux, handle_mux_select)
    startup_msg = "Ready to respond to mux select commands"
    startup_msg = startup_msg + " (local mode)" if local else startup_msg

    print(startup_msg)
    rospy.spin()

if __name__ == "__main__":
    mux_select_server()
