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

    # Must set values when launching
    self.declare_parameter('joint_lengths', [1.0, 1.0, 1.0])
    self.declare_parameter('joint_angle_mins', [-180.0, -180.0, -180.0, -180.0])
    self.declare_parameter('joint_angle_maxes', [180.0, 180.0, 180.0, 180.0])
    self.declare_parameter('sensitivity', 1.0)

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
    lengths = self.get_parameter('joint_lengths').get_parameter_value().double_array_value
    self.L1 = lengths[0]
    self.L2 = lengths[1]
    self.L3 = lengths[2]
    # For convenience, store as array and as individual values
    self.lengths = lengths

    # Joint angle limits
    mins = self.get_parameter('joint_angle_mins').get_parameter_value().double_array_value
    maxes = self.get_parameter('joint_angle_maxes').get_parameter_value().double_array_value
    # Convert them to radians
    self.mins = [math.radians(x)  for x in mins]
    self.maxes = [math.radians(x)  for x in maxes]
    # Pitch is set not by IK but by the user, handled separately
    self.pitch_min = self.mins[3]
    self.pitch_max = self.maxes[3]

    # Sensitivity
    self.sensitivity = self.get_parameter('sensitivity').get_parameter_value().double_value
    
    # Get initial state from absolute encoders to initialize
    # the angle values and coordinates
    self.initialize_angles_coords()

    joy_topic = '/cad_mouse_joy'
    self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
    self.get_logger().info('Created publisher for topic "'+joy_topic)
  
  def initialize_angles_coords(self):
    # This will get the values from the actual absolute encoders. For now, hardcode example
    angles = [0.0, 0.79, 0.79, 0.79]
    self.angles = angles

    # Start in cylindrical coords
    # The first angle swivel, it's the phi angle in the cylindrical coords system
    self.phi = angles[0]

    # This calculates u and v from the angles
    # The last three angles are flex, so add the displacement caused by these joints
    self.u, self.v = self.coords_from_flex(angles[1:])

    # The pitch is the angle of the gripper (wrt the previous joint) 
    # it must be set as well since this is an angle which is set directly
    self.pitch = angles[-1]

    # Turn to cartesian (stored in self.x, y, z)
    self.calculate_cartesian()


  def coords_from_flex(self, angles):
    # Finds the 2d coords from a series of flex joints (ie arms rotating)
    # The angles are only relative to the previous joint, cumulative_angle holds the cumulative
    # angle from the previous joints
    cumulative_angle = 0.0
    u, v = 0.0, 0.0
    for angle, length in zip(angles, self.lengths):
      cumulative_angle += angle
      u += length * math.sin(cumulative_angle)
      v += length * math.cos(cumulative_angle)
    return u, v

  
  def publish_joint_state(self):
    # If not initialized yet, don't publish
    if self.angles:
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
      cv = self.v + self.L3 * math.cos(self.pitch)

      if cu < 0 and cv < 0:
        self.get_logger().warn(f"Point (xyz) {self.x} {self.y} {self.z} out of range")
        return False

      # In this context, cu and cv are lengths, so must be positive
      a1 = math.atan(abs(cv / cu))
      a2 = math.atan(abs(cu / cv))

      L = math.sqrt((cu ** 2) + (cv ** 2))
      B1 = (self.L1 ** 2 + L ** 2 - self.L2 ** 2) / (2 * self.L1 * L)
      B2 = (self.L1 ** 2 + self.L2 ** 2 - L ** 2) / (2 * self.L1 * self.L2)
      B3 = (self.L2 ** 2 + L ** 2 - self.L1 ** 2) / (2 * self.L2 * L)
      if any_out_of_range(-1, 1, B1, B2, B3):
        self.get_logger().warn(f"Point (xyz) {self.x} {self.y} {self.z} pitch {self.pitch} (uvp) {self.u} {self.v} {self.phi} out of range")
        return False
      
      b1 = math.acos(B1)
      b2 = math.acos(B2)
      b3 = math.acos(B3)

      # self.get_logger().info(f"u {self.u} v {self.v} pitch {self.pitch}")
      # self.get_logger().info(f"cu {cu} cv {cv} a1 {a1} a2 {a2} L {L} b1 {b1} b2 {b2} b3 {b3}")

      # contains Shoulder Swivel, Shoulder Flex, Elbow Flex, Wrist Flex (in that order)
      if cu < 0:
        angles = [float(self.phi), -((math.pi / 2) - a1 + b1),
                  math.pi - b2, math.pi - (self.pitch + b3)]
      elif cv < 0:
        angles = [float(self.phi), (math.pi) - (b1 + a2),
                  math.pi - b2, math.pi / 2 - (self.pitch + b3 + a1)]
      else:
        angles = [float(self.phi), (math.pi / 2) - (b1 + a1),
                          math.pi - b2, math.pi - (self.pitch + a2 + b3)]
      
      if self.validAngles(angles):
        self.angles = angles
      else:
        self.get_logger().warn(f"Outside joint limits")
        return False

      self.get_logger().info(f"angles: {self.angles}")
      return True

    except ValueError as e:
      self.get_logger().error(f"Caught error {e} with coordinates (x,y,z) {self.x} {self.y} {self.z} (u,v,phi,pitch) {self.u} {self.v} {self.phi} {self.pitch}")
  
  def joy_callback(self, message: Joy):
    # self.get_logger().info(f"Received from cad mouse")
    old_values = (self.x, self.y, self.z, self.th, self.pitch)
    x, y, z, pitch, roll, yaw = message.axes
    self.x += (self.sensitivity * x / 30000) # left-right of mouse
    self.y += (self.sensitivity * -y / 30000) # forward-back of mouse
    self.z += (self.sensitivity * -z / 30000) # vertical axis of mouse
    self.th += (self.sensitivity * yaw / 150) # use rotation axis that is activated by spinning the top

    # Pitch is the one angle which is directly set, so check bounds here
    new_pitch = self.pitch + (self.sensitivity * roll / 30000)
    if self.validPitch(new_pitch):
      self.pitch = new_pitch

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
        if self.y >= 0:
          # In second quadrant
          # domain issue: atan is limited to -pi/2 to pi/2 so it loops over when x < 0
          self.phi = math.pi + math.atan(self.y / self.x)
        else:
          # In third quadrant
          self.phi = -(math.pi - math.atan(self.y / self.x))
  
  
  def calculate_cartesian(self):
    # The v axis is the same as the z axis, the u is perpendicular to it
    self.z = self.v
    self.x = self.u * math.cos(self.phi)
    self.y = self.u * math.sin(self.phi)

  
  def validAngles(self, angles):
    for angle, min, max in zip(angles, self.mins, self.maxes):
      # Validate angle is between min and max
      if not(min <= angle and angle <= max):
        return False
      
    return True
  
  def validPitch(self, new_pitch):
    return self.pitch_min <= new_pitch and new_pitch <= self.pitch_max


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
