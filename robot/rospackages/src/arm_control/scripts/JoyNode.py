#!/usr/bin/env python3

from Joy_class import Astro_Joy
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('joy_node',anonymous = False)
    rospy.loginfo('Initialized joy_node for pub functionality')

    arm_topic = '/arm_command'
    rospy.loginfo('Beginning to publish to '+arm_topic+' topic')
    armPub = rospy.Publisher(arm_topic, String, queue_size = 10)
    
    wheels_topic = '/rover_command'
    rospy.loginfo('Beginning to publish to '+wheels_topic+' topic')
    wheelPub = rospy.Publisher(wheels_topic, String, queue_size = 10)

    my_joy = Astro_Joy(30,30,True)
    # joystick instance (maximum throttle,fine steering,controlled throttle)

    try:
        while not rospy.is_shutdown():
            my_joy.switch()
            if my_joy.isArm:
                message = my_joy.arm_reset()
                if message is not None:
                    armPub.publish(message)
                message = my_joy.arm()
                if message is not None:
                    armPub.publish(message)
            elif my_joy.isRover:
                message = my_joy.wheels()
                if message is not None:
                    wheelPub.publish(message)
            rospy.sleep(.001)  # suitable delays are already in the wheels method
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('Terminating joy_node')
        rospy.sleep(0.3)

    rospy.on_shutdown(shutdown_hook)
