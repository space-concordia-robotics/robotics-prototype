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
from absenc_interface.msg import EncoderValues
from arm_controller.msg import ArmMotorValues
import threading



class AbsencNode(Node):
  
    def __init__(self):
        node_name = 'absenc_node'
        super().__init__(node_name)
        self.get_logger().info('Initialized "'+node_name+'" node for functionality')

        arm_topic = '/arm_command'
        self.arm_publisher = self.create_publisher(String, arm_topic, 10)
        self.get_logger().info('Created publisher for topic "'+arm_topic)

        arm_2_topic = '/arm_values'
        self.arm_2_publisher = self.create_publisher(ArmMotorValues, arm_2_topic, 10)
        #self.get_logger().info('Created publisher for topic "'+arm_topic)

        ik_topic = '/joint_states'
        self.ik_sub = self.create_subscription(JointState, ik_topic, self.ik_callback, 10)
        self.get_logger().info('Created subscriber for topic "'+ik_topic)

        absenc_topic = '/absenc_values'
        self.absenc_sub = self.create_subscription(EncoderValues, absenc_topic, self.absenc_callback, 10)
        self.get_logger().info('Created subscriber for topic "' + absenc_topic)

        # Angles reported from absenc
        # Angles reported from IK
        self.abs_angles = None
        self.ik_angles = None
        # self.ik_angles = [0.0, -30.0, 20.0, 10.0]

        # Controls if motor sign is aligned with encoder direction
        self.angle_signs = [1, -1, -1, -1]


    def ik_callback(self, message:JointState):
        # Receive values and convert to degrees
        self.ik_angles = [math.degrees(x) for x in list(message.position)]
    
    def absenc_callback(self, message: EncoderValues):
        angle_1 = message.angle_1 if message.angle_1 < 180 else message.angle_1 - 360
        angle_2 = message.angle_2
        angle_3 = 360 + message.angle_3 if message.angle_3 < -180 else message.angle_3
        self.abs_angles = [0, angle_1, angle_2, angle_3]
        self.get_logger().info(f"Angles Fixed: {self.abs_angles}")
    
    def publish_arm_command(self):
        if self.ik_angles == None or self.abs_angles == None:
            return
        
        arm_command = "set_motor_speeds "
        values_2 = []

        for absenc_angle, ik_angle, angle_sign in zip(self.abs_angles, self.ik_angles, self.angle_signs):
            # Here, the difference between the actual and desired angle is split into abs value and sign
            # This is done to make mapping the range easier (it has a minimum value of 55, it can simply
            # be added before the proper sign is applied)
            absolute_difference = abs(absenc_angle - ik_angle)
            difference_sign = 1 if absenc_angle - ik_angle >= 0 else -1

            if absolute_difference <= 1:
                arm_command += "0 "
                values_2.append(0)
            else:
                # Map the angle difference (at most 180 deg) onto 0-255 range with a min of 150
                # This implements a basic P controller, further from correct angle, faster it moves
                value = ((absolute_difference) / 180.0) * 2000 
                # Clamp value
                value = 255 if value >= 255 else value
                value *= difference_sign
                value = int(value) * angle_sign
                values_2.append(float(value))
                arm_command += f"{value} "
                # self.get_logger().info(f"Angle difference {}")
        print(arm_command)
        print(values_2)
        # Add two for last two pwm motors this isn't controlling yet
        arm_command += "0 0"
        msg = String()
        msg.data = arm_command

        msg2 = ArmMotorValues()
        msg2.val[0] = values_2[0]
        msg2.val[1] = values_2[1]
        msg2.val[2] = values_2[2]
        msg2.val[3] = values_2[3]
        msg2.val[4] = 0
        msg2.val[5] = 0
        
        # msg2.val[0] = values_2[0]
        # msg2.val[1] = values_2[1]
        # msg2.val[2] = values_2[2]
        # msg2.val[3] = values_2[3]
        # msg2.val[4] = values_2[4]
        # msg2.val[5] = values_2[5]
        

        self.arm_publisher.publish(msg)
        self.arm_2_publisher.publish(msg2)


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
            break

    rclpy.shutdown()
