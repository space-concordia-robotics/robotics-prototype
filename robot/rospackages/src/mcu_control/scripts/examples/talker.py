#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    node_name = 'talker'
    topic = '/arm_angles'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" publisher node')
    rospy.loginfo('Beginning to publish to "'+topic+'" topic')
    pub = rospy.Publisher('/arm_data', String, queue_size=10)
    rate = rospy.Rate(1)
    try:
        i=0
        while not rospy.is_shutdown():
            i += 1
            rospy.loginfo(str(i))
            pub.publish(str(i))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass