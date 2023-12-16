#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import sys
from rclpy.node import Node
from rclpy.clock import Clock
import math

devpath = "/dev/cadmouse"

class IkNode(Node):
  
  def __init__(self):
    node_name = 'ik_node'
    super().__init__(node_name)
    self.get_logger().info('Initialized "'+node_name+'" node for pub functionality')
    # Cartesian coordinates of desired location of end effector
    self.x = 0
    self.y = 0
    self.z = 1
    # Keeps track of spin (think of it as roll angle)
    # Not touched by ik, sent straight to MCUs
    self.th = 0

    # Cylindrical coordinates
    # Starts out with the cylindrical plane in the XZ plane
    self.u = 0 # Horizontal on plane
    self.v = 1 # Vertical on plane (same as z)
    self.phi = 0 # angle

    joy_topic = '/cad_mouse_joy'
    self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
    self.get_logger().info('Created publisher for topic "'+joy_topic)

  
  def joy_callback(self, message: Joy):
    # self.get_logger().info(f"Received from cad mouse {message.axes}")
    x, y, z, pitch, roll, yaw = message.axes
    self.x += (x / 30000) # left-right of mouse
    self.y += (y / 30000) # forward-back of mouse
    self.z += (z / 30000) # vertical axis of mouse
    self.th += (yaw / 150) # use rotation axis that is activated by spinning the top
    self.calculate_cylindical()
    self.get_logger().info(f"Current location: {self.x} {self.y} {self.z} roll {self.th}")
    self.get_logger().info(f"Cylindical coordinates: u {self.u} v {self.v} phi {self.phi}")
    # Stamp {message.header.stamp}

  def calculate_cylindical(self):
    self.u =  math.sqrt(self.x ** 2 + self.y ** 2)
    self.v = self.z
    if self.x == 0:
      self.phi = 0
    else:
      self.phi = math.atan(self.y / self.x)


def main(args=None):
  rclpy.init(args=args)

  cad_mouse_node = IkNode()

  try:
    rclpy.spin(cad_mouse_node)
  except KeyboardInterrupt:
    print("Node shutting down due to shutting down node.")
  rclpy.shutdown()
