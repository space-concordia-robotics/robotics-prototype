#!/usr/bin/env python
"""
ROS node that takes Joy msgs and translates them to various msgs for
navigation/control purposes (e.g. Twist msgs for wheel control).

NOTE: Example node only, the actual handling of Twist msgs via Joy
is handled via a provided pkg 'teleop_twist_joy' (see 'teleop_twist_joy.launch'
for more info). In theory, this example node can be used if eventually
a need for the other buttons is required, this node will act as a translator
and process it to then dispatch to some topic.

For most XBOX 360 wired/wireless controllers, the following Joy msg
key map is valid.

sensor_msgs/Joy.axes:
    Idx     Name
    0       Left stick horizonal (+LEFT/-RIGHT) -- angular speed
    1       Left stick vertical (+UP/-DOWN) -- linear speed
    2       LT
    3       Right stick horizontal
    4       Right stick vertical
    5       RT
    6       Cross key left/right
    7       Cross key up/down
sensor_msgs/Joy.buttons:
    0       A
    1       B
    2       X
    3       Y
    4       LB
    5       RB
    6       Back
    7       Start
    8       Power
    9       Stick left button
    10      Stick right button

See http://wiki.ros.org/joy#Application for more keymaps.
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

LINEAR_AXIS_IDX = 1
ANGULAR_AXIS_IDX = 0

# Publishing to "rover/cmd_vel" topic
pub = rospy.Publisher('rover/cmd_vel', Twist, queue_size=1)


def callback(data):
    """Joy subscriber callback function.

    Receives sensor_msgs/Joy msgs (subscribed to joy topic) then
    converts them into geometry_msgs/Twist msgs.

    Args:
        data (sensor_msgs/Joy): contains joystick state

    data.axes: values are scaled [-1, 1]
    data.buttons: values are scaled [0, 1]
    """
    twist = Twist()

    # Vertical left stick axis -- linear rate
    twist.linear.x = data.axes[LINEAR_AXIS_IDX]

    # Horizontal left stick axis -- angular rate
    twist.angular.z = data.axes[ANGULAR_AXIS_IDX]

    pub.publish(twist)


def start():
    """Runs the bootstrap code."""
    rospy.init_node('rover_teleop_joy')

    # Subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()


if __name__ == '__main__':
    start()
