import struct
import rospy
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.

feedback_pub_topic = '/science_feedback'
feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)

def handle_status(data):
    carousel_index, moving = struct.unpack('bb', data)  # 1 byte
    if moving:
        print("moving toward ", carousel_index)
        feedbackPub.publish("moving toward " + carousel_index)
    else:
        print("at index", carousel_index)
        feedbackPub.publish("at index" + carousel_index)

def handle_debug_str(data):
        debug_str_data = data.decode('utf-8')
        print("Debug:", debug_str_data)
        feedbackPub.publish(debug_str_data)


def handle_limit_switch_value(data):
    switchValue = struct.unpack('b', data)  # 1 byte
    print("Received limit switch value: ", switchValue)

# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349

science_out_commands = [("previous_test_tube", 41, []), ("next_test_tube", 42, []), ("go_to_test_tube", 43, [dt.ARG_UINT8]), 
                        ("spin_mix", 45, []), ("get_status", 39, []), ("estop", 25, []),
                        ("set_servo_angle", 26, [dt.ARG_FLOAT32]), ("get_virtual_angle", 37, [])]


science_in_commands = [("virtual angle", 38, handle_debug_str), ("status", 40, handle_status), ("limit_switch_value", 28, handle_limit_switch_value)]
