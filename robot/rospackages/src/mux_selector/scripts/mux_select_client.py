#!/usr/bin/env python

import sys
import datetime
import rospy
from mux_selector.srv import *

def mux_select_client(device):
    # convenience function that blocks until 'ping_response' is available
    rospy.wait_for_service('mux_select')

    try:
        mux_select = rospy.ServiceProxy('mux_select', SelectMux)
        resp1 = mux_select(device)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: {:s}".format(e))

def is_valid_request(device):

    if device.isnumeric():
        device = int(device)
    else:
        return False

    is_valid = device in (0, 1, 2, 3)

    return is_valid

def usage():
    help_msg = """USAGE:\nrosrun mux_selector mux_select_client.py [device number]
                  \nValid device options: [0, 1, 2, 3]"""
    return help_msg

if __name__ == "__main__":
    if len(sys.argv) > 2 or len(sys.argv) < 2:
        print(usage())
        #print((sys.argv[0]) + " [device number]")
        sys.exit(1)

    rospy.init_node("mux_select_client")

    # expecting 0, 1, 2, 3 for rover, arm, science, lidar
    dev = sys.argv[1]

    if not is_valid_request(dev):
        print("Invalid device chosen, valid device options: [0, 1, 2, 3]")
        sys.exit(0)

    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    print("Selecting MUX channel " + str(dev))
    print(sent_ts)
    print("---")

    print("Response: " + mux_select_client(dev))
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))
    print(received_st)

    diff = received - sent
    print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
