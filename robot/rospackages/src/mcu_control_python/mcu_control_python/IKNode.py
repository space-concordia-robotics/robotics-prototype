#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import sys
from rclpy.node import Node
from rclpy.clock import Clock
from decimal import *

devpath = "/dev/cadmouse"

class IkNode(Node):
  
  def __init__(self):
    node_name = 'ik_node'
    super().__init__(node_name)
    self.get_logger().info('Initialized "'+node_name+'" node for pub functionality')
    self.x = Decimal(0)
    self.y = Decimal(0)
    self.z = Decimal(0)
    self.th = Decimal(0)
    # Set fixed point precision
    getcontext().prec = 10

    joy_topic = '/cad_mouse_joy'
    self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
    self.get_logger().info('Created publisher for topic "'+joy_topic)

  
  def joy_callback(self, message: Joy):
    # self.get_logger().info(f"Received from cad mouse {message.axes}")
    x, y, z, pitch, roll, yaw = message.axes
    self.x += Decimal(x / 150)
    self.y += Decimal(y / 150)
    self.z += Decimal(z / 150)
    self.th += Decimal(yaw / 150) # use rotation axis that is activated by spinning the top
    self.get_logger().info(f"Current location: {self.x} {self.y} {self.z} roll {self.th}")
    # Stamp {message.header.stamp}


def main(args=None):
  rclpy.init(args=args)

  cad_mouse_node = IkNode()

  try:
    rclpy.spin(cad_mouse_node)
  except KeyboardInterrupt:
    print("Node shutting down due to shutting down node.")
  rclpy.shutdown()
