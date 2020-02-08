import sys
import time
import rospy
from std_msgs.msg import Float32

#decimals for all pubs
DECIMALS = 2

# temp values -> should be extracted from the js file later
voltage_max = 14;
voltage_min = 3;

def get_time_millis():
  millis = int(round(time.time() * 1000))
  return millis


def PDS_pub():
  try:
    mode = sys.argv[1]
  except:
    print('please give a mode value!')
    sys.exit()

  pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
  rospy.init_node('PDS_mock_pub', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  if (mode == 'rise'):
    start_vol = 4 #volts
    speed = 1 #volts per second

    while not rospy.is_shutdown():
      time_now = get_time_millis()
      pub_vol = round(start_vol +  speed * ((time_now - start_time) / 1000), DECIMALS)
      pub.publish(pub_vol)
      print(pub_vol)
      rate.sleep()
  else :
    print('Invalid mode!')

if __name__ == '__main__':
  start_time = get_time_millis();

  try:
    PDS_pub()
  except rospy.ROSInterruptException:
    pass