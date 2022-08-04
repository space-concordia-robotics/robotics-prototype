import struct
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt


# All handlers should start with handle_ , while unrelated functions should not.
def handle_debug_string(data):
    print("Debug:", data.decode('utf-8'))


def handle_pong(data):
    print("Pong")


def handle_send_linear_velocity(data):
    linearVelocity = struct.unpack('f' * 1, data)  # 1 float
    print("Received", list(map(lambda f: str(f), linearVelocity)))


def handle_send_rotational_velocity(data):
    rotationalVelocity = struct.unpack('f' * 1, data)  # 1 float
    print("Received", list(map(lambda f: str(f), rotationalVelocity)))


def handle_send_current_velocity(data):
    currentVelocities = struct.unpack('f' * 6, data)  # 6 floats
    print("Received", list(map(lambda f: str(f), currentVelocities)))


def handle_send_desired_velocity(data):
    desiredVelocities = struct.unpack('f' * 6, data)  # 6 floats
    print("Received", list(map(lambda f: str(f), desiredVelocities)))


def handle_send_battery_voltage(data):
    vbatt = struct.unpack('f' * 1, data)  # 1 float
    print("Received", list(map(lambda f: str(f), vbatt)))


# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349
wheel_out_commands = [("set_motors", 0, [dt.ARG_UINT8]), ("motors_estop", 1, []),
                      ("close_motors_loop", 2, []), ("open_motors_loop", 3, []),
                      ("set_joystick", 4, [dt.ARG_UINT8]),
                      ("set_gps", 5, [dt.ARG_UINT8]), ("set_enc", 6, [dt.ARG_UINT8]),
                      ("set_acc", 7, [dt.ARG_UINT8]), ("get_rover_status", 8, []),
                      ("move_rover", 9, 4 * [dt.ARG_UINT8]),
                      ("move_wheel", 10, 3 * [dt.ARG_UINT8]),
                      ("get_linear_velocity", 11, []),
                      ("get_rotational_velocity", 12, []),
                      ("get_current_velocity", 13, 6 * []),
                      ("get_desired_velocity", 14, 6 * []),
                      ("get_battery_voltage", 15, []), ("ping", 16, []),
                      ("set_blink", 17, [dt.ARG_UINT8])]

wheel_in_commands = [
    ("debug_string", 0, handle_debug_string),
    ("ping", 1, handle_pong),
    ("send_linear_velocity", 2, handle_send_linear_velocity),
    ("send_rotational_velocity", 3, handle_send_rotational_velocity),
    ("send_current_velocity", 4, handle_send_current_velocity),
    ("send_desired_velocity", 5, handle_send_desired_velocity),
    ("send_battery_voltage", 6, handle_send_battery_voltage),
]
