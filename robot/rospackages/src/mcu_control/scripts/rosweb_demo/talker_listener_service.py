#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mcu_control.srv import *

# todo: implement a timeout
# todo: maybe publish/parse the string if it's not ping? this might cause errors but it's worth a shot
def handle_client(req):
    rospy.loginfo("received request from client")
    response = ArmRequestResponse(
        "you sent "+req.msg+'\n',
        True
    )
    rospy.loginfo(response)
    return response

def subscriber_callback(data):
    rospy.loginfo(data)

if __name__ == '__main__':
    node_name = 'arm_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" multidirectional node')

    publish_topic = '/arm_angles'
    rospy.loginfo('Beginning to publish to "'+publish_topic+'" topic')
    pub = rospy.Publisher(publish_topic, String, queue_size=10)
    rate = rospy.Rate(1)

    subscribe_topic = 'arm_command'
    rospy.loginfo('Beginning to subscribe to "'+subscribe_topic+'" topic')
    sub = rospy.Subscriber(subscribe_topic, String, subscriber_callback)

    service_name = 'arm_request'
    rospy.loginfo('Waiting for "'+service_name+'" service request from client')
    serv = rospy.Service(service_name, ArmRequest, handle_client)

    # service requests and subscriptions are implicitly handled
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
