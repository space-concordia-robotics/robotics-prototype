#!/usr/bin/env python3

import json
import rclpy
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String

# DOCUMENTATIONS
# http://wiki.ros.org/joy

class JoyMapper(Node):
  
    def __init__(self):
        node_name = 'joymap_node'
        super().__init__(node_name)
        self.get_logger().info('Initialized "'+node_name+'" node for functionality')

        # Subscribe to the /joy topic
        map_topic = '/joy'
        # QoS settings
        qos_profile = QoSProfile(depth=10)
        self.map_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.get_logger().info('Created joy map subscriber for topic "'+map_topic)

        # Publisher for the rover's status
        self.status_publisher = self.create_publisher(String, '/rover/status', qos_profile)
        self.get_logger().info('Created rover status publisher for topic "/rover/status"')


    def joy_callback(self, report:Joy):
           
        # Handle joystick input and publish the rover's current status
        buttons = report.buttons
        axes = report.axes

        joy_data = {
            'buttons': buttons,
            'axes': axes
        }

        # Serialize the dictionary to a JSON string
        joy_data_str = json.dumps(joy_data)

        # Create a String message to send the JSON data
        msg = String()
        msg.data = joy_data_str

        # Publish the serialized joystick data
        self.status_publisher.publish(msg)

        self.get_logger().info(f"Published joystick data: {joy_data_str}")


def main(args=None):
    rclpy.init(args=args)
    joy_mapper_node = JoyMapper()

    try:
        rclpy.spin(joy_mapper_node)
    except KeyboardInterrupt:
        joy_mapper_node.get_logger().info("Node shutting down due to keyboard interrupt.")
    finally:
        joy_mapper_node.destroy_node()
        rclpy.shutdown()