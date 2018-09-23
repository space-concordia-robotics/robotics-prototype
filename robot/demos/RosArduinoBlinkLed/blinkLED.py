#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
import time
import click
import sys

def talker():

    msg = Empty() # empty message
    # topic name: toggle_led
    pub = rospy.Publisher('toggle_led', Empty)
    # node name
    print("\nInitializing led_toggler node...")
    rospy.init_node('led_toggler', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    print("Press 't' to toggle the LED, or 'q' to quit\n")

    while not rospy.is_shutdown():
        input = click.getchar()

        if input == 't':
            print("Toggling LED")
            pub.publish(msg)
        elif input == 'q':
            print("\nExiting...")
            sys.exit(0)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
