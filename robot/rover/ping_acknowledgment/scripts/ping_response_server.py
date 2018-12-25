#!/usr/bin/env python

from ping_acknowledgment.srv import *
import rospy

#@TODO: test if "handle_" is necessary
def handle_ping(req):
    response = "To your '%s', I say 'suh dude'"%(req.ping)
    print "Returning '" + response + "'"
    return response

def ping_response_server():
    rospy.init_node('ping_response_server')
    s = rospy.Service('ping_response', PingResponse, handle_ping)
    print "Ready to respond to ping"
    rospy.spin()

if __name__ == "__main__":
    ping_response_server()
