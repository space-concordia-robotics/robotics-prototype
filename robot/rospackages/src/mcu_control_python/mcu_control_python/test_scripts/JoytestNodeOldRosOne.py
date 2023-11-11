#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import time

class PlayStation(Joy):
    _button_names = {"BUTTON_CROSS": 0,
                    "BUTTON_CIRCLE": 1,
                    "BUTTON_TRIANGLE": 2,
                    "BUTTON_SQUARE": 3,
                    "BUTTON_L1": 4,
                    "BUTTON_R1": 5,
                    "BUTTON_L2": 6,
                    "BUTTON_R2": 7,
                    "BUTTON_SHARE": 8,
                    "BUTTON_OPTION": 9,
                    "BUTTON_HOME": 10,
                    "BUTTON_L3": 11,
                    "BUTTON_R3": 12}
    _axis_names = {"JOY_LEFT_X": 0,
                    "JOY_LEFT_Y": 1,
                    "TRIGGER_L2": 2,
                    "JOY_RIGHT_X": 3,
                    "JOY_RIGHT_Y": 4,
                    "TRIGGER_R2": 5,
                    "DPAD_X": 6,
                    "DPAD_Y": 7}
    
    def __init__(self):
        Joy.__init__(self)
        self.buttons = [0 for x in range(len(PlayStation._button_names))]
        self.axes = [0.0 for x in range(len(PlayStation._axis_names))]
    
    def get_button(self, button_name):
        return self.buttons[PlayStation._button_names[button_name]]

    def set_button(self, button_name, value):
        self.buttons[PlayStation._button_names[button_name]] = value
    
    def get_axis(self, axis_name):
        return self.axes[PlayStation._axis_names[axis_name]]

    def set_axis(self, axis_name, value):
        self.axes[PlayStation._axis_names[axis_name]] = value
    
    def add_to_axis(self, axis_name, increment):
        self.axes[PlayStation._axis_names[axis_name]] += increment




if __name__ == "__main__":
    node_name = 'joy_test_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type

    joy_topic = '/joy'
    rospy.loginfo('Beginning to publish to "'+joy_topic+'" topic')
    joyPub = rospy.Publisher(joy_topic, Joy, queue_size=10)

    j = PlayStation()
    j.set_button("BUTTON_R1", 1) # enable

    # ping us into wheel mode
    time.sleep(1)
    joyPub.publish(j)
    j.set_button("BUTTON_SHARE", 1)
    joyPub.publish(j)
    j.set_button("BUTTON_SHARE", 0)
    time.sleep(1)

    rate = rospy.Rate(2)
    increment = 0.125

    # R2 partly pressed to test summation
    j.set_axis("TRIGGER_R2", 0.5)

    while not rospy.is_shutdown():
        j.add_to_axis("JOY_RIGHT_X", increment)

        if j.get_axis("JOY_RIGHT_X") >= 1.0 or j.get_axis("JOY_RIGHT_X") <= -1.0:
            # stay at max a little while
            for i in range(5):
                joyPub.publish(j)
                time.sleep(0.5)
            increment = -increment
        
        joyPub.publish(j)

        rate.sleep()

