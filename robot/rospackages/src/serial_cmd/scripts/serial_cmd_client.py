#!/usr/bin/env python

import sys
import datetime
import rospy
from serial_cmd.srv import *

def serial_cmd_client(device):
    # convenience function that blocks until 'serial_cmd' is available
    rospy.wait_for_service('serial_cmd')

    try:
        serial_cmd = rospy.ServiceProxy('serial_cmd', SerialCmd)
        resp1 = serial_cmd(device)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: {:s}".format(e))

def usage():
    help_msg = "USAGE:\nrosrun serial_cmd serial_cmd_client.py [message]"
    return help_msg

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(usage())
        sys.exit(1)

    rospy.init_node("serial_cmd_client")

    msg = sys.argv[1]

    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    print("Sending serial command '" + str(msg) + "'")
    print(sent_ts)
    print("---")

    print("Response: " + serial_cmd_client(msg))
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))
    print(received_st)

    diff = received - sent
    print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
