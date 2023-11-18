#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import sys
from rclpy.node import Node
from rclpy.clock import Clock

devpath = "/dev/cadmouse"

class CadMouseJoyNode(Node):
  
  def __init__(self):
    node_name = 'cad_mouse_joy_node'
    super().__init__(node_name)
    self.get_logger().info('Initialized "'+node_name+'" node for pub functionality')

    joy_topic = '/cad_mouse_joy'
    self.joy_pub = self.create_publisher(Joy, joy_topic, 1)
    self.get_logger().info('Created publisher for topic "'+joy_topic)

  def readMouseInput(self):
    axis = [0, 0, 0, 0, 0, 0]
    button = [False, False]
    battery = -1
    charged = False
    try:
      with open(devpath, "rb", buffering=0) as MouseByteStream:
        while rclpy.ok():
          data = MouseByteStream.read(16)
          id = data[0]
          if id == 1:
            for i in range(0, 6):
              axis_raw = data[2 * i + 1] + 256 * data[2 * i + 2]
              if axis_raw > 32768:
                axis_raw = axis_raw - 65536
              axis[i] = axis_raw
            print(axis)
          elif id == 3:
            button_raw = data[1]
            button[0] = (button_raw & 0x1) != 0
            button[1] = (button_raw & 0x2) != 0
            print("Button: " + str(button))
          elif id == 23:
            battery = data[1]
            charged = data[2] != 0
            print("Battery:" + str(battery) + ", Charging: " + str(charged))

          self.publishJoyMsg(axis, button)
    except KeyboardInterrupt:
      print("Node shutting down due to shutting down node.")
    except FileNotFoundError:
      print("-" * 40 + "\n")
      print("ERROR")
      print("ERROR: Could not find device file for 3D mouse. Make sure that the mouse is plugged "
            "in or its dongle is and it has battery. If that is the case, add the udev rule. "
            "This script will prompt you to add the udev rule. If it does not work, see"
            "https://github.com/space-concordia-robotics/robotics-documentation/blob/main/software/cad-mouse-setup.md")
      self.askAndAddUdevRule()

  def askAndAddUdevRule(self):
    import os
    import time
    print("-" * 20)
    print("Do you want to attempt to install and activate the udev rule? y/n")
    answer = input()
    if answer.lower() == 'y':
      print("attempting to copy the udev rule...")
      while os.getcwd().split("/")[-1] != "robotics-prototype" and os.getcwd() != "/":
        os.chdir("..")
      # os.system("cat robot/util/udev-rules/10-cadmouse.rules")
      os.system("sudo cp ./robot/util/udev-rules/10-cadmouse.rules /lib/udev/rules.d/10-cadmouse.rules && sudo udevadm trigger && echo done")
      print("attempting to run again...")
      time.sleep(4)
      self.readMouseInput()
  
  def publishJoyMsg(self, axis, button):
    joy_msg_header = Header()
    joy_msg_header.stamp = self.get_clock().now().to_msg()

    cad_mouse_joy_msg = Joy()
    cad_mouse_joy_msg.header = joy_msg_header
    axis_float = [x + 0.0 for x in axis]
    cad_mouse_joy_msg.axes = axis_float
    cad_mouse_joy_msg.buttons = button

    self.joy_pub.publish(cad_mouse_joy_msg)

def main(args=None):
  rclpy.init(args=args)

  cad_mouse_node = CadMouseJoyNode()

  # rate = comms_node.create_rate(10)

  try:
    cad_mouse_node.readMouseInput()

  except KeyboardInterrupt:
    print("Node shutting down due to shutting down node.")
  rclpy.shutdown()
