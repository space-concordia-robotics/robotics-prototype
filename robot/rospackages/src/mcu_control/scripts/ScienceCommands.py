import struct
import rospy
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.


feedback_pub_topic = '/science_feedback'
feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)



# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349

science_out_commands = [("next_test_tube", 42, [])]


science_in_commands = []
