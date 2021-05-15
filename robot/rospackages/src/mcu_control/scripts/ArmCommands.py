import struct

import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt


# All handlers should start with handle_ , while unrelated functions should not.

def handle_debug_string(data):
    print("Debug:", data.decode('utf-8'))

def handle_pong(data):
    print("Pong")

def handle_send_motor_angles(data):
    motorAngles = struct.unpack('f' * 6, data)
    print("Received", list(map(lambda f: str(f), motorAngles)))
    # anglePub.publish(data) # todo: convert each value to the correct type

# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349
arm_out_commands = [("estop", 0, []),
                    ("reboot", 1, []),
                    ("motors_stop", 2, []),
                    ("reset_angles", 3, []),
                    ("home_motors", 4, []),
                    ("home", 5, [dt.ARG_UINT8, dt.ARG_UINT8]),
                    ("arm_speed", 6, [dt.ARG_FLOAT32]),
                    ("stop_single_motor", 7, [dt.ARG_UINT8]),
                    ("gear_ratio", 8, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("open_loop_gain", 9, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("pid_constants", 10, [dt.ARG_UINT8, dt.ARG_FLOAT32, dt.ARG_FLOAT32, dt.ARG_FLOAT32]),
                    ("motor_speed", 11, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("open_loop_state", 12, [dt.ARG_UINT8, dt.ARG_UINT8]),
                    ("reset_single_motor", 13, [dt.ARG_UINT8]),
                    ("budge_motors", 14, 6*[dt.ARG_UINT8]),
                    ("reset_single_motor", 15, 6*[dt.ARG_FLOAT32]),
                    ("ping", 16, []),
                    ("get_motor_angles", 17, 6*[dt.ARG_FLOAT32])]

arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong), ("send_motor_angles", 2, handle_send_motor_angles)]
