#!/usr/bin/env python

import rospy
from mcu_control.msg import CADMouse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


def publisher():
    rospy.init_node('CADMouseNode', anonymous=True)
    pub = rospy.Publisher('CADMousePub', CADMouse)
    r = rospy.Rate(10)
    msg = CADMouse()
    msg.Twist = Twist(12, 13, 14)
    msg.Buttons[0] = 1
    msg.Buttons[1] = 0
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    publisher()