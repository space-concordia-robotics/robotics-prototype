#!/usr/bin/env python

import sys
import rospy
from ping_acknowledgment.srv import *

def ping_response_client(msg):
    # convenience function that blocks until 'ping_response' is available
    rospy.wait_for_service('ping_response')

    try:
        ping_response = rospy.ServiceProxy('ping_response', PingResponse)
        resp1 = ping_response(msg)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: {:s}".format(e))

def usage():
    return "{:s} [msg]".format(sys.argv[0])

if __name__ == "__main__":
    if len(sys.argv) > 2:
        print(usage())

    msg = sys.argv[1]

    print("Pinging server")
    print(ping_response_client(msg))
