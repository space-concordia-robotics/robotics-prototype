#!/usr/bin/env python

import sys
import datetime
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

    rospy.init_node("ping_response_client")

    # Figure out if being called from roslaunch or roscore/python
    if "__name:=" in sys.argv[1]:
        msg = rospy.get_param("~ping_msg")
    else:
        msg = sys.argv[1]

    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    print("Pinging server with ping_acknowledgment service")
    print(sent_ts)
    print("---")

    print("Response: " + ping_response_client(msg))
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))
    print(received_st)

    diff = received - sent
    print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
