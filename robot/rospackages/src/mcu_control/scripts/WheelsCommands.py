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
wheel_out_commands = [ ("ping", 0, []),
			("move_servo", 1, 2 * [dt.ARG_UINT8]),			
			("move_rover", 2, 2 * [dt.ARG_FLOAT32]),
			("move_wheel", 3, 3 * [dt.ARG_UINT8]), 
			("move_wheels", 4, 12 * [dt.ARG_UINT8]), 
			("motors_estop", 5, []),
            ("get_battery_voltage", 6, [])]
            ("blink_toggle", 17, [dt.ARG_UINT8]), 
            ("blink_color", 18, [dt.ARG_UINT8, dt.ARG_UINT8, dt.ARG_UINT8])]

wheel_in_commands = [
    ("debug_string", 0, handle_debug_string),
    ("ping", 1, handle_pong),
    ("send_linear_velocity", 2, handle_send_linear_velocity),
    ("send_rotational_velocity", 3, handle_send_rotational_velocity),
    ("send_current_velocity", 4, handle_send_current_velocity),
    ("send_desired_velocity", 5, handle_send_desired_velocity),
    ("send_battery_voltage", 6, handle_send_battery_voltage),
]
