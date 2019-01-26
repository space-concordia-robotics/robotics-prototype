#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def rx_publisher():
    # publish to topic "odroid_rx"
    pub = rospy.Publisher('odroid_rx', String, queue_size=10)
    rospy.init_node('rx_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    hello = True

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        goodbye_str = "good bye world %s" % rospy.get_time()

        if hello == True:
            pub.publish(hello_str)
            rospy.loginfo(hello_str)
            hello = False
        else:
            pub.publish(goodbye_str)
            rospy.loginfo(goodbye_str)
            hello = True

        rate.sleep()

if __name__ == '__main__':
    try:
        rx_publisher()
    except rospy.ROSInterruptException:
        pass
