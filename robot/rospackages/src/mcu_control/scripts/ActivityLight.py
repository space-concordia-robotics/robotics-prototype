#!/usr/bin/env python3

import rospy
from APA102 import APA102
from std_msgs.msg import String, Header, Float32
from enum import Enum

class LightState(Enum):
    LEFT_ON = 1
    RIGHT_ON = 2
    OFF = 3

def spi(light):
    print(light)

left_leds = [0, 1, 2, 3, 9, 10, 11, 12, 13, 14]
right_leds = [4, 5, 6, 7, 8, 15, 16, 17, 18, 19]

light = APA102(spi, 20)
state = LightState.OFF

def callback(data):
    global state
    if data.data == 'start':
        light.set_pins(left_leds, 255, 0, 0)
        light.send()
        state = LightState.LEFT_ON
    elif data.data == 'stop':
        light.set_to_off()
        light.send()
        state = LightState.OFF

def main():
    global state
    rospy.init_node("activity_light", anonymous = True)
    rate = rospy.Rate(1) # 1hz
    sub = rospy.Subscriber("ActivityLight", String, callback)
    print("initialized ActivityLight node")
    # rospy.spin()
    while not rospy.is_shutdown():
        if state == LightState.LEFT_ON:
            light.set_pins(right_leds, 255, 0, 0)
            light.send()
            state = LightState.RIGHT_ON
        elif state == LightState.RIGHT_ON:
            light.set_pins(left_leds, 255, 0, 0)
            light.send()
            state = LightState.LEFT_ON
        elif state == LightState.OFF:
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Shutting down activity light node")