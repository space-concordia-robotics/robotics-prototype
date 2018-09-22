#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
import time

def talker():
    msg = Empty() # empty message
    # topic name: toggle_led
    pub = rospy.Publisher('toggle_led', Empty)
    # node name
    rospy.init_node('led_toggler', anonymous=True)
    #rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        pub.publish(msg)
        time.sleep(4)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
