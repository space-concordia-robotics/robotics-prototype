import struct
from rclpy.node import Node
import robot.rospackages.src.mcu_control_python.mcu_control_python.definitions.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.


def handle_debug_string(data):
    debug_str_data = data.decode('utf-8')
    print("Debug:", debug_str_data)


def handle_pong(data):
    print("Pong")


def handle_send_motor_angles(data):
    motorAngles = struct.unpack(('f' * 6), data)
    print("Received", list(map(lambda f: str(f), motorAngles)))
    # anglePub.publish(data) # todo: convert each value to the correct type


science_out_commands = [("science_command_1", 78, 6 * [dt.ARG_FLOAT32]), ("ping", 75, [])]

science_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong)]
