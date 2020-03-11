#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from decimal import *

getcontext().prec = 4


def shift_val(val, state_param_name, min_val, max_val, safe, speed):
    try:
      mode = rospy.get_param(state_param_name)
    except:
      print('PDS_mock_mode parameter error!')
      sys.exit()

    if (mode == 'stable'):
      target = safe
    elif (mode == 'rise'): 
      target = max_val
    elif (mode == 'fall'):
      target = min_val
    else :
      print('Invalid mode!')
      sys.exit()

    direction = target - val 
    if (abs(direction) >= speed):
      if direction > 0:
        val += speed 
      elif direction < 0:
        val -= speed
    
    return val

def PDS_pub():
  pub_voltage = rospy.Publisher('battery_voltage', Float32, queue_size=10)
  rospy.init_node('PDS_mock_pub', anonymous=True)
  rate = rospy.Rate(10) #hz
  
  # voltage values 
  voltage_max = Decimal(20.0)
  voltage_min = Decimal(10.0)
  voltage_safe = Decimal(15.0)
  voltage_speed = Decimal((0, (0, 1), -1))#0.1
  voltage_current = voltage_safe

  pub_temperature = rospy.Publisher('battery_temps', Point, queue_size=10)
  
  # temperature values
  temperature_max = Decimal(100.0)
  temperature_min = Decimal(-20.0)
  temperature_safe = Decimal(50.0)
  temperature_speed = Decimal(1)  
  temperature1_current = temperature_safe 
  temperature2_current = temperature_safe 
  temperature3_current = temperature_safe 

  while not rospy.is_shutdown():
    #shift values
    voltage_current = shift_val(voltage_current, 'PDS_mock_mode_voltage', voltage_min, voltage_max, voltage_safe, voltage_speed)
    temperature1_current = shift_val(temperature1_current, 'PDS_mock_mode_temp1', temperature_min, temperature_max, temperature_safe, temperature_speed)
    temperature2_current = shift_val(temperature2_current, 'PDS_mock_mode_temp2', temperature_min, temperature_max, temperature_safe, temperature_speed)
    temperature3_current = shift_val(temperature3_current, 'PDS_mock_mode_temp3', temperature_min, temperature_max, temperature_safe, temperature_speed)
 
        
    #publish values
    pub_voltage.publish(voltage_current)    
    pub_temperature.publish(Point(temperature1_current, temperature2_current, temperature3_current))

    rate.sleep()

if __name__ == '__main__':
  rospy.set_param('PDS_mock_mode_voltage', 'stable')
  rospy.set_param('PDS_mock_mode_temp1', 'stable')
  rospy.set_param('PDS_mock_mode_temp2', 'stable')
  rospy.set_param('PDS_mock_mode_temp3', 'stable')
  try:
    PDS_pub();
  except rospy.ROSInterruptException:
    pass
