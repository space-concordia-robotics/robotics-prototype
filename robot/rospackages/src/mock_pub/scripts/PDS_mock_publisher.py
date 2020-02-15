import sys
import time
import rospy
from std_msgs.msg import Float32

# temp values -> should be extracted from the js file later
max_voltage = 16.5
min_voltage = 12.5

voltage_current = max_voltage - min_voltage

def get_time_millis():
  millis = int(round(time.time() * 1000))
  return millis

def PDS_pub():
  try:
    mode = rospy.get_param('PDS_mock_mode')
  except:
    print('PDS_mock_mode parameter error!')
    sys.exit()

  pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
  rospy.init_node('PDS_mock_pub', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  global voltage_current

  while not rospy.is_shutdown():
      if (mode == 'stable'): #rise until reaches max, then declines until min then rises, so on
        continue
      elif (mode == 'rise'): #rise indefinetly
        voltage_current += 1
      elif (mode == 'fall'):
        voltage_current -= 1
      else :
        print('Invalid mode!')
        sys.exit()

      pub.publish(voltage_current)
      print(voltage_current)
      rate.sleep()


if __name__ == '__main__':
  # Setting default mode parameter
  rospy.set_param('PDS_mock_mode', 'fall')

  try:
    PDS_pub()
  except rospy.ROSInterruptException:
    pass