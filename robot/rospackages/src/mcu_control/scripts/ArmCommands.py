import struct

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
arm_out_commands = [("estop", 0), ] # todo add all commands...
arm_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong), ("send_motor_angles", 2, handle_send_motor_angles)]