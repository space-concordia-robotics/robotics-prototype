import sys
import time
import rospy
from std_msgs.msg import Float32

def PDS_pub():
  pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
  rospy.init_node('PDS_mock_pub', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  # temp values -> should be extracted from the js file later
  voltage_max = 16.8
  voltage_min = 12.5
  voltage_safe = 15
  speed = 0.1

  voltage_current = voltage_safe # starting voltage
  voltage_target = voltage_safe

  while not rospy.is_shutdown():
    try:
      mode = rospy.get_param('PDS_mock_mode')
    except:
      print('PDS_mock_mode parameter error!')
      sys.exit()
      

    if (mode == 'stable'): #rise until reaches max, then declines until min then rises, so on
      voltage_target = voltage_safe
    elif (mode == 'rise'): #rise indefinetly
      voltage_target = voltage_max
    elif (mode == 'fall'):
      voltage_target = voltage_min
    else :
      print('Invalid mode!')
      sys.exit()

    direction = voltage_target - voltage_current
    if (direction >= speed):
      if direction > 0:
        voltage_current += speed
      elif direction < 0:
        voltage_current -= speed

    pub.publish(voltage_current)
    print(voltage_current)
    rate.sleep()


if __name__ == '__main__':
  # Setting default mode parameter
  rospy.set_param('PDS_mock_mode', 'rise')

  try:
    PDS_pub()
  except rospy.ROSInterruptException:
    pass
