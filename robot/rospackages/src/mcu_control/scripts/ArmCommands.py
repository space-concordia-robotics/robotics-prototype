import struct
import rospy
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.


feedback_pub_topic = '/arm_feedback'
feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size=10)


def handle_debug_string(data):
    debug_str_data = data.decode('utf-8')
    print("Debug:", debug_str_data)
    feedbackPub.publish(debug_str_data)


def handle_pong(data):
    print("Pong")
    feedbackPub.publish("Pong")


def handle_send_motor_angles(data):
    motorAngles = struct.unpack(('f' * 6), data)
    print("Received", list(map(lambda f: str(f), motorAngles)))
    anglePub.publish(data) # todo: convert each value to the correct type

# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349
# arm_out_commands = [("move_motors_by", 76, 6 * [dt.ARG_FLOAT32]),
#                     ("set_motor_speeds", 78, 6 * [dt.ARG_FLOAT32]), ("ping", 75, []),
#                     ("invalid_test", 10, []), ("send_motor_angles", 77, [])]

arm_out_commands = [("set_motor_speeds", 78, 6 * [dt.ARG_FLOAT32]), ("ping", 75, []),
                    ("invalid_test", 10, []), ("debug_test", 79, []),]


# arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong),
#                    ("send_motor_angles", 2, handle_send_motor_angles)]
arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong)]
