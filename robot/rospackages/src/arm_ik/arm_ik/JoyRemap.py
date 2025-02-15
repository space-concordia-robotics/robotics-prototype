#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml
import os


class JoyRemap(Node):
    def __init__(self):
        super().__init__("joy_remap")
        self.is_message_received = False
        
        # Define uniform axis and button mappings
        self.axis_mappings = {
            'left_stick_x': 0,
            'left_stick_y': 1,
            'dpad_x': 2,
            'right_stick_x': 3,
            'right_stick_y': 4,
            'dpad_y': 5,
        }

        self.button_mappings = {
            'cross': 0,
            'circle': 1,
            'triangle': 2,
            'square': 3,
            'l1': 4,
            'r1': 5,
            'l2': 6,
            'r2': 7,
            'share': 8,
            'options': 9,
            'ps_button': 10,
            'trackpad': 11,
        }
        
        self.pub = self.create_publisher(Joy, "joy_remap", 10)
        self.sub = self.create_subscription(Joy, "joy", self.callback, 10)

    def callback(self, msg):
        self.is_message_received = True
        out_msg = Joy()
        out_msg.header = msg.header

        # Initialize axes and buttons based on the uniform mappings
        out_msg.axes = [0.0] * len(self.axis_mappings)
        out_msg.buttons = [0] * len(self.button_mappings)

        # Remap axes
        for i, (axis_name, axis_index) in enumerate(self.axis_mappings.items()):
            out_msg.axes[i] = msg.axes[axis_index] 

        # Remap buttons
        for i, (button_name, button_index) in enumerate(self.button_mappings.items()):
            out_msg.buttons[i] = msg.buttons[button_index] 

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutting down due to keyboard interrupt.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
