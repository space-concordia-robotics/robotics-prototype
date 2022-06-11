#!/usr/bin/env python3

import serial
from math import radians
import rospy
from sensor_msgs.msg import JointState

# Global variables
absSer = None

anglesReceived = 0

# angleData format: (absAddress, jointAngle)
# Address is hex number
angleData = dict([("90", None),
                 ("A0", None),
                 ("C0", None)])

def main():
    try:
        while not rospy.is_shutdown():
            if absSer.in_waiting > 0:
                message = absSer.readline().decode('utf-8')

                receive_angles(message[2:len(message)])
                publish_arm_angles(anglePub)

    except KeyboardInterrupt:
        print("Node shutting down due to shutting down node.")
    absSer.close()

def receive_angles(absDataString):
    absLine = absDataString.split(' ')

    absAddressHex = absLine[0]
    absAngle = float(absLine[1])

    if angleData[absAddressHex] is None:
        angleData[absAddressHex] = absAngle
        global anglesReceived
        anglesReceived += 1




def publish_arm_angles(anglePub):
    global anglesReceived
    if anglesReceived == len(angleData):
        msg = JointState()
        # msg.header = data.header

        # TODO: joint names will be taken from the urdf file
        for key in angleData:
            msg.name.append("angle_" + str(key))
            msg.position.append(radians(angleData[key]))

        # msg.name.append("upbase_joint")
        # msg.name.append("prox_joint")
        # msg.name.append("distal_joint")
        # msg.name.append("wrist_flex_joint")
        # msg.name.append("wrist_twist_joint")
        # msg.name.append("finger1_joint")
        # msg.name.append("finger2_joint")

        anglePub.publish(msg)
        anglesReceived = 0

        for address in angleData:
            angleData[address] = None

if __name__ == '__main__':
    absSer = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

    node_name = 'ArmPublisher'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    angle_pub_topic = '/joint_states'
    rospy.loginfo('Beginning to publish to "' + angle_pub_topic + '" topic')
    anglePub = rospy.Publisher(angle_pub_topic, JointState, queue_size=10)

    main()


