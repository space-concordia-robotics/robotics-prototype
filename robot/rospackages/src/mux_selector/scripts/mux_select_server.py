#!/usr/bin/env python

import rospy
from mux_selector.srv import *

def handle_mux_select(req):
    response = "Switched MUX select to device {:s}".format(req.device)

    # call bash function to do the select
    response += "\n" + select_device(req.device)

    return response

def select_device(device):
    s0_val = 0
    s1_val = 0

    if not device.isnumeric():
        print("Device argument must be numeric value")
        return

    device = int(device)

    # s1 most significant bit, s0 least significant bit
    # s0 --> gpio 18 (physical 15), s1 --> gpio 21 (physical 13)
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
            print("Selecting device 3: Lidar")
            s0_val = 0
            s1_val = 0

    gpio_dir = "/sys/class/gpio"

# consider removing following lines to make given "test" argument flag present
# or better: user ros params as per tatums suggestion
    with open(gpio_dir + "/gpio18/value", "w") as f:
        f.write(str(s0_val))

    with open(gpio_dir + "/gpio21/value", "w") as f:
        f.write(str(s1_val))

    s0_state = "s0: " + str(s0_val)
    s1_state = "s1: " + str(s1_val)
    print(s0_state)
    print(s1_state)

    return s0_state + "\n" + s1_state

def mux_select_server():
    rospy.init_node('mux_select_server')
    s = rospy.Service('mux_select', SelectMux, handle_mux_select)
    print("Ready to respond to mux select commands")
    rospy.spin()

if __name__ == "__main__":
    mux_select_server()
