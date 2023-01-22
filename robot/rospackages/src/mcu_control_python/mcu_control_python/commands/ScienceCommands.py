import struct
import robot.rospackages.src.mcu_control_python.mcu_control_python.definitions.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.

def handle_status(data):
    carousel_index, moving = struct.unpack('bb', data)  # 1 byte
    if moving:
        print("moving toward ", carousel_index)
    else:
        print("at index", carousel_index)

def handle_limit_switch_value(data):
    switchValue = struct.unpack('b', data)  # 1 byte
    print("Received limit switch value: ", switchValue)

# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349

science_out_commands = [("previous_test_tube", 41, []), ("next_test_tube", 42, []), ("go_to_test_tube", 43, [dt.ARG_UINT8]), ("start_calibrating", 44, [dt.ARG_UINT8]),
                        ("get_status", 39, []), ("estop", 25, []), ("set_servo_angle", 26, [dt.ARG_UINT8, dt.ARG_UINT8]),
                        ("read_limit_switch", 27, [dt.ARG_UINT8])]


science_in_commands = [("status", 40, handle_status), ("limit_switch_value", 28, handle_limit_switch_value)]
