#!/usr/bin/env python3

import rospy
from arm_control.srv import *

def handle_client(req):
    rospy.loginfo("received "+req.msg)
    data_str = ""
    msg = ArmRequestResponse(
        "pong\n",
        True
    )
    rospy.loginfo(msg)
    return msg

if __name__ == '__main__':
    node_name = 'arm_service'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" service node')

    service_name = 'arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)
    # use this to block if nothing else needs to happen
    # and you don't want the node to close. for callbacks/handlers
    rospy.spin()
