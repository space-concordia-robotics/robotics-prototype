import struct

UINT8_SIZE = 1
UINT16_SIZE = 2
UINT32_SIZE = 4
FLOAT32_SIZE = 4

ARG_UINT8 = 0
ARG_UINT16 = 1
ARG_UINT32 = 2
ARG_FLOAT32 = 3
ARG_STRING = 4

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
arm_out_commands = [("estop", 0, []), ("reset_angles", 3, []), ("home_motors", 4, []), ("arm_speed", 6, [FLOAT32_SIZE])] # todo add all commands...
arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong), ("send_motor_angles", 2, handle_send_motor_angles)]
