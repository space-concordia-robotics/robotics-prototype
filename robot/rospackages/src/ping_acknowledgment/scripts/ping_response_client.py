#!/usr/bin/env python3

import sys
import datetime
import rospy
from ping_acknowledgment.srv import *

#
# ping_response_client
# 
# Use without arguments to obtain detailed timestamps and latency data
# Use option "api" to just receive the time it took in millisecond for the ping.
#


def ping_server():
    # convenience function that blocks until 'ping_response' is available
    #rospy.wait_for_service('ping_response')

    try:
        ping_response = rospy.ServiceProxy('ping_response', PingResponse)
        resp1 = ping_response("")
        return resp1.response
    except rospy.ServiceException as e:
        return "Service call failed: " + str(e)


def usage():
    return "usage: {:s} [OPTIONS] [api]".format(sys.argv[0])


if __name__ == "__main__":
    if len(sys.argv) > 2:
        print(usage())

    api = False
    if len(sys.argv) == 2 and sys.argv[1] == "api":
        api = True


    rospy.init_node("ping_response_client")
    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    
    ping_server()
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))

    if not api:
        print("Pinging server with ping_acknowledgment service")
        print(sent_ts)
        print("---")

        print("Received Response")
        print(received_st)

    diff = received - sent
    if api:
        print(diff.microseconds / 1000)
    else:
        print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
