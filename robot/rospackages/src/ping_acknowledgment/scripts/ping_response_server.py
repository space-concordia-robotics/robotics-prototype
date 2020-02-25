#!/usr/bin/env python3

import os
from ping_acknowledgment.srv import *
import rospy

def handle_ping(req):
    with open(os.environ['HOME'] + '/Programming/robotics-prototype/robot/rospackages/src/ping_acknowledgment/scripts/meme.txt', 'r') as meme:
        returnmsg = meme.read()

    if req.ping == "rover_ip":
        response = os.environ["ROS_MASTER_URI"].split(":")
        response = response[0] + ":" + response[1]
    else:
        response = "To your '{:s}', I say \n".format(req.ping) + returnmsg

    print("Returning '" + response + "'")
    return response

def ping_response_server():
    rospy.init_node('ping_response_server')
    s = rospy.Service('ping_response', PingResponse, handle_ping)
    print("Ready to respond to ping")
    rospy.spin()

if __name__ == "__main__":
    ping_response_server()
