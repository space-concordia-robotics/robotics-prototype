#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String

def callback(data):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_file = script_dir + "/odroid_rx.txt"

    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

    file = open(log_file, 'w')
    file.write(data.data)
    file.close()

def rx_subscriber():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_file = script_dir + "/odroid_rx.txt"

    # create file if doesn't exist
    file = open(log_file, 'w+')
    file.close()

    rospy.init_node('rx_subscriber', anonymous=True)

    rospy.Subscriber('odroid_rx', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rx_subscriber()
