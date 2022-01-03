import struct

import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt


# All handlers should start with handle_ , while unrelated functions should not.

def handle_debug_string(data):
    print("Debug:", data.decode('utf-8'))

def handle_pong(data):
    print("Pongo")

def handle_send_motor_angles(data):
    motorAngles = struct.unpack('f' * 6, data)
    print("Received", list(map(lambda f: str(f), motorAngles)))
    # anglePub.publish(data) # todo: convert each value to the correct type

# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349
arm_out_commands = [("estop", 75, []),
                    ("reboot", 76, []),
                    ("motors_stop", 77, []),
                    ("reset_angles", 78, []),
                    ("home_motors", 79, [dt.ARG_UINT8]),
                    ("home", 80, [dt.ARG_UINT8, dt.ARG_UINT8]),
                    ("arm_speed", 81, [dt.ARG_FLOAT32]),
                    ("stop_single_motor", 82, [dt.ARG_UINT8]),
                    ("gear_ratio", 83, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("open_loop_gain", 84, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("pid_constants", 85, [dt.ARG_UINT8, dt.ARG_FLOAT32, dt.ARG_FLOAT32, dt.ARG_FLOAT32]),
                    ("motor_speed", 86, [dt.ARG_UINT8, dt.ARG_FLOAT32]),
                    ("open_loop_state", 87, [dt.ARG_UINT8, dt.ARG_UINT8]),
                    ("reset_single_motor", 88, [dt.ARG_UINT8]),
                    ("budge_motors", 89, 6*[dt.ARG_UINT8]),
                    ("reset_single_motor", 90, 6*[dt.ARG_FLOAT32]),
                    ("ping", 91, []),
                    ("get_motor_angles", 92, [])]

arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong), ("send_motor_angles", 2, handle_send_motor_angles)]
