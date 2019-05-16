#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse #used as a placeholder for a proper service

# expects an empty request and returns a Trigger message with bool and string
# in this case, when the request is received it sends "ping" to the teensy and waits for pong indefinitely
# todo: implement a timeout
# todo: maybe publish/parse the string if it's not ping? this might cause errors but it's worth a shot
def handle_client(req):
    rospy.loginfo("received request from client")
    data_str = ""
    response = TriggerResponse(
        success = True,
        message = "here is the response"
    )
    rospy.loginfo(response)
    # the Trigger handler expects a TriggerResponse object to be returned (goes back to client)
    # note that empty responses seem to break it (in python, according to google and my experience)
    # I was therefore forced to use a .srv with a response
    # todo: make a package for this and define my own .srv file
    return response

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" multidirectional node')
    
    publish_topic = '/arm_angles'
    rospy.loginfo('Beginning to publish to "'+publish_topic+'" topic')
    pub = rospy.Publisher(publish_topic, String, queue_size=10)
    rate = rospy.Rate(1)
    
    service_name = '/arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, Trigger, handle_client)
    
    # service requests are implicitly handled
    # but only at the rate the node publishes at
    try:
        i=0
        while not rospy.is_shutdown():
            i += 1
            rospy.loginfo(str(i))
            pub.publish(str(i))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass