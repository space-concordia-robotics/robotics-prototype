#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import JointState
import threading



class AbsencNode(Node):
  
    def __init__(self):
        node_name = 'absenc_node'
        super().__init__(node_name)
        self.get_logger().info('Initialized "'+node_name+'" node for functionality')

        arm_topic = '/arm_command'
        self.arm_publisher = self.create_publisher(String, arm_topic, 10)
        self.get_logger().info('Created publisher for topic "'+arm_topic)

        ik_topic = '/joint_states'
        self.ik_sub = self.create_subscription(JointState, ik_topic, self.ik_callback, 10)
        self.get_logger().info('Created subscriber for topic "'+ik_topic)

        absenc_topic = '/absenc_states'
        self.absenc_sub = self.create_subscription(JointState, absenc_topic, self.absenc_callback, 10)
        self.get_logger().info('Created subscriber for topic "' + absenc_topic)

        # Angles reported from absenc
        self.abs_angles = None
        # Angles reported from IK
        self.ik_angles = None


    def ik_callback(self, message:JointState):
        # Receive values and convert to degrees
        self.ik_angles = [math.degrees(x) for x in list(message.position)]
    
    def absenc_callback(self, message):
        pass
    
    def publish_arm_command(self):
        if self.ik_angles == None or self.abs_angles == None:
            return
        
        arm_command = "set_motor_speeds "

        for absenc_angle, ik_angle in zip(self.abs_angles, self.ik_angles):
            # If very far move more quickly
            if abs(absenc_angle - ik_angle) > 15:
                if absenc_angle > ik_angle:
                    arm_command += "-255 "
                else:
                    arm_command += "255 "
            # if close move slowly
            elif abs(absenc_angle - ik_angle) > 5:
                if absenc_angle > ik_angle:
                    arm_command += "-100 "
                else:
                    arm_command += "100 "
            # if very close don't move
            else:
                arm_command += "0 "

        # Add two for last two pwm motors this isn't controlling yet
        arm_command += "0 0"

        self.arm_publisher.publish(arm_command)



def main(args=None):
    rclpy.init(args=args)

    absenc_node = AbsencNode()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(absenc_node, ), daemon=True)
    thread.start()

    loop_rate = absenc_node.create_rate(30)
    while rclpy.ok():
        try:
            absenc_node.publish_arm_command()
            loop_rate.sleep()
        except KeyboardInterrupt:
            print("Node shutting down due to shutting down node.")

    rclpy.shutdown()
