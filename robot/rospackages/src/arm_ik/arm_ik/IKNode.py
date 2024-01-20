#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import threading


devpath = "/dev/cadmouse"

def any_out_of_range(min, max, *values):
  for value in values:
    if value < min or value > max:
      return True
  return False


class IkNode(Node):
  
  def __init__(self):
    node_name = 'ik_node'
    super().__init__(node_name)
    self.get_logger().info('Initialized "'+node_name+'" node for pub functionality')

    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

    # Cartesian coordinates of desired location of end effector
    self.x = 1
    self.y = 0
    self.z = 1
    # Keeps track of spin (think of it as roll angle)
    # Not touched by ik, sent straight to MCUs
    self.th = 0
    # The angle of the last joint with respect to vertical
    self.pitch = 0

    # Joint lengths
    self.L1 = 1.354
    self.L2 = 1.333
    # self.L3 = 0.4992
    self.L3 = 1.250

    # Cylindrical coordinates and angles
    self.calculate_cylindical()
    self.calculate_angles()

    joy_topic = '/cad_mouse_joy'
    self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
    self.get_logger().info('Created publisher for topic "'+joy_topic)
  
  def publish_joint_state(self):
    joint_state = JointState()

    now = self.get_clock().now()
    joint_state.header.stamp = now.to_msg()
    joint_state.name = ["Shoulder Swivel", "Shoulder Flex", "Elbow Flex", "Wrist Flex"]
    joint_state.position = self.angles
    self.joint_pub.publish(joint_state)

  
  def calculate_angles(self):
    """ Performs IK calculation and stores values in self.angles.
        Returns True if within range, False otherwise"""
    try:
      # cu and cv are the coordinates of the critical point
      cu = self.u - self.L3 * math.sin(self.pitch)
      cv = self.v - self.L3 * math.cos(self.pitch)

      if cu == 0 or cv == 0:
        return
      
      if cu < 0:
        self.get_logger().warn(f"cu < 0")

      # In this context, cu and cv are lengths, so must be positive
      a1 = math.atan(abs(cv / cu))
      a2 = math.atan(abs(cu / cv))

      # self.get_logger().warn(f"cu {cu} cv {cv} a1 {a1} a2 {a2}")

      L = math.sqrt(cu ** 2 + cv ** 2)
      B1 = (self.L1 ** 2 + L ** 2 - self.L2 ** 2) / (2 * self.L1 * L)
      B2 = (self.L1 ** 2 + self.L2 ** 2 - L ** 2) / (2 * self.L1 * self.L2)
      B3 = (self.L2 ** 2 + L ** 2 - self.L1 ** 2) / (2 * self.L2 * L)
      if any_out_of_range(-1, 1, B1, B2, B3):
        self.get_logger().warn(f"Point {self.x} {self.y} {self.z} out of range")
        return False
      
      b1 = math.acos(B1)
      b2 = math.acos(B2)
      b3 = math.acos(B3)

      self.get_logger().info(f"cu {cu} cv {cv} a1 {a1} a2 {a2} L {L} b1 {b1} b2 {b2} b3 {b3}")

      # contains Shoulder Swivel, Shoulder Flex, Elbow Flex, Wrist Flex (in that order)
      if cu < 0:
        self.angles = [float(self.phi), -((math.pi / 2) - a1 + b1),
                  math.pi - b2, math.pi - (self.pitch + b3)]
      else:
        self.angles = [float(self.phi), (math.pi / 2) - (b1 + a1),
                          math.pi - b2, math.pi - (self.pitch + a2 + b3)]
      self.get_logger().info(f"angles: {self.angles}")
      # self.get_logger().info(f"pos: {self.x} {self.y} {self.z}")
      self.get_logger().info(f"u {self.u} v {self.v} pitch {self.pitch}")
      return True

    except ValueError as e:
      self.get_logger().error(f"Caught error {e} with coordinates (x,y,z) {self.x} {self.y} {self.z} (u,v,phi,pitch) {self.u} {self.v} {self.phi} {self.pitch}")
  
  def joy_callback(self, message: Joy):
    # self.get_logger().info(f"Received from cad mouse")
    old_values = (self.x, self.y, self.z, self.th, self.pitch)
    x, y, z, pitch, roll, yaw = message.axes
    self.x += (x / 30000) # left-right of mouse
    self.y += (-y / 30000) # forward-back of mouse
    self.z += (-z / 30000) # vertical axis of mouse
    self.th += (yaw / 150) # use rotation axis that is activated by spinning the top
    self.pitch += (roll / 30000)
    self.calculate_cylindical()
    # Perform IK. If out of range, restore point where it was
    if not self.calculate_angles():
      self.x, self.y, self.z, self.th, self.pitch = old_values
    # self.get_logger().info(f"Current location: {self.x} {self.y} {self.z} roll {self.th} pitch {self.pitch}")
    # self.get_logger().info(f"Cylindical coordinates: u {self.u} v {self.v} phi {self.phi}")

  def calculate_cylindical(self):
    self.u =  math.sqrt(self.x ** 2 + self.y ** 2)
    self.v = self.z
    if self.x == 0:
      self.phi = 0
    else:
      if self.x >= 0:
        self.phi = math.atan(self.y / self.x)
      else:
        # domain issue: atan is limited to -pi/2 to pi/2 so it loops over when x < 0
        self.phi = math.pi + math.atan(self.y / self.x)


def main(args=None):
  rclpy.init(args=args)

  ik_node = IkNode()
  # Spin in a separate thread
  thread = threading.Thread(target=rclpy.spin, args=(ik_node, ), daemon=True)
  thread.start()

  loop_rate = ik_node.create_rate(30)
  while rclpy.ok():
    try:
        ik_node.publish_joint_state()
        loop_rate.sleep()
    except KeyboardInterrupt:
        print("Node shutting down due to shutting down node.")

  rclpy.shutdown()
