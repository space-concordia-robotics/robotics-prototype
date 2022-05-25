#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

devpath = "/dev/cadmouse"

def publishJoyMsg(axis, button):
  joy_msg_header = Header()
  joy_msg_header.stamp = rospy.Time.now()

  cad_mouse_joy_msg = Joy()
  cad_mouse_joy_msg.header = joy_msg_header
  cad_mouse_joy_msg.axes = axis
  cad_mouse_joy_msg.buttons = button

  joy_pub.publish(cad_mouse_joy_msg)

# This function is not used.
def askAndAddUdevRule():
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
    readMouseInput()


def readMouseInput():
  axis = [0, 0, 0, 0, 0, 0]
  button = [False, False]
  battery = -1
  charged = False
  try:
    with open(devpath, "rb", buffering=0) as MouseByteStream:
      while not rospy.is_shutdown():
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

        publishJoyMsg(axis, button)
  except KeyboardInterrupt:
    print("Node shutting down due to shutting down node.")
  except FileNotFoundError:
    import os
    print("-" * 40 + "\n")
    print("ERROR")
    print("ERROR: Could not find device file for 3D mouse. Make sure that the mouse is plugged "
          "in or its dongle is and it has battery. If that is the case, add the udev rule, "
          "see https://github.com/space-concordia-robotics/robotics-documentation/blob/main/software/cad-mouse-setup.md")
    #askAndAddUdevRule()

if __name__ == '__main__':
  node_name = 'cad_mouse_joy_node'
  rospy.init_node(node_name, anonymous=False) # only allow one node of this type
  rospy.loginfo('Initialized "'+node_name+'" node for pub functionality')

  joy_topic = '/cad_mouse_joy'
  rospy.loginfo('Beginning to publish to "'+joy_topic+'" topic')
  joy_pub = rospy.Publisher(joy_topic, Joy, queue_size=1)

  readMouseInput()