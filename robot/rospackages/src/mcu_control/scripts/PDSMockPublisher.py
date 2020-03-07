#!/usr/bin/env python3
import sys
import time
import rospy
from std_msgs.msg import Float32
from decimal import *

getcontext().prec = 4

def PDS_pub():
  pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
  rospy.init_node('PDS_mock_pub', anonymous=True)
  rate = rospy.Rate(10) #hz

  # temp values -> should be extracted from the js file later
  voltage_max = Decimal(20.0)
  voltage_min = Decimal(10.0)
  voltage_safe = Decimal(15.0)
  speed = Decimal((0, (0, 1), -1))

  voltage_current = voltage_safe #starting voltage
  voltage_target = voltage_safe #default value

  while not rospy.is_shutdown():
    try:
      mode = rospy.get_param('PDS_mock_mode_voltage')
    except:
      print('PDS_mock_mode parameter error!')
      sys.exit()

    if (mode == 'stable'):
      voltage_target = voltage_safe
    elif (mode == 'rise'): 
      voltage_target = voltage_max
    elif (mode == 'fall'):
      voltage_target = voltage_min
    else :
      print('Invalid mode!')
      sys.exit()

    direction = voltage_target - voltage_current
    print(voltage_current)
    if (abs(direction) >= speed):
      if direction > 0:
        voltage_current += speed
      elif direction < 0:
        voltage_current -= speed

    pub.publish(voltage_current)
    rate.sleep()

if __name__ == '__main__':
  rospy.set_param('PDS_mock_mode_voltage', 'stable')
  try:
    PDS_pub();
  except rospy.ROSInterruptException:
    pass
      



