import rospy
from std_msgs.msg import Float32

def PDS_pub():
  pub = rospy.Publisher('PDS_mock_publisher', Float32, queue_size=10)
  rospy.init_node('PDS_pub', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    pub.publish(420)
    rate.sleep()

if __name__ == '__main__':
  try:
    PDS_pub()
  except rospy.ROSInterruptException:
    pass