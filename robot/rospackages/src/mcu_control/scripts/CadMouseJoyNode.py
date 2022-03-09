#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

devpath = "/dev/hidraw1"

def publishJoyMsg(axis, button):
  joy_msg_header = Header()
  joy_msg_header.stamp = rospy.Time.now()

  cad_mouse_joy_msg = Joy()
  cad_mouse_joy_msg.header = joy_msg_header
  cad_mouse_joy_msg.axes = axis
  cad_mouse_joy_msg.buttons = button

  joy_pub.publish(cad_mouse_joy_msg)

def readMouseInput():
  battery = -1
  charged = False

  with open(devpath, "rb", buffering=0) as MouseByteStream:
    while True:
      axis = [0, 0, 0, 0, 0, 0]
      button = [False, False]

      data = MouseByteStream.read(16)
      id = data[0]
      if id == 1:
        for i in range(0, 6):
          axis_raw = data[2 * i + 1] + 256 * data[2 * i + 2]
          if axis_raw > 32768:
            axis_raw = axis_raw - 65536
          axis[i] = axis_raw
        publishJoyMsg(axis, button)
        print(axis)
      elif id == 3:
        button_raw = data[1]
        button[0] = (button_raw & 0x1) != 0
        button[1] = (button_raw & 0x2) != 0
        publishJoyMsg(axis, button)
        print("Button: " + str(button))
      elif id == 23:
        battery = data[1]
        charged = data[2] != 0
        print("Battery:" + str(battery) + ", Charging: " + str(charged))

if __name__ == '__main__':
  node_name = 'cad_mouse_joy'
  rospy.init_node(node_name, anonymous=False) # only allow one node of this type
  rospy.loginfo('Initialized "'+node_name+'" node for pub functionality')

  joy_topic = '/cad_mouse_joy'
  rospy.loginfo('Beginning to publish to "'+joy_topic+'" topic')
  joy_pub = rospy.Publisher(joy_topic, Joy, queue_size=1)

  readMouseInput()