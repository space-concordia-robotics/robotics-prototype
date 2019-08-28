#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    # topic name: arm_command, only remember the current message so commands don't queue up
    pub = rospy.Publisher('arm_command', String, queue_size=1)
    print("\nInitializing arm_controller node...")
    # node name is arm_controller
    rospy.init_node('arm_controller', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    print("Type in a motor command. Examples: 'ping', 'motor 1 angle 5.7', 'stop', 'motor 4 loop closed', 'motor 6 stop', move 4.5 ~ ~ ~ -6.35 -2'\n")

    while not rospy.is_shutdown():
        armCommand = String(input())
        rospy.loginfo(armCommand)
        pub.publish(armCommand)
        rate.sleep()

if __name__ == '__main__':
    try:
        arm_controller()
    except rospy.ROSInterruptException:
        pass
