import struct

# All handlers should start with handle_ , while unrelated functions should not.

def handle_debug_string(data):
    print("Debug:", data.decode('utf-8'))

def handle_pong(data):
    print("Pong")

# todo : This file is currently placeholder but the actual commands from the link below should be added 
# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349
rover_out_commands = [("estop", 0), ] # todo add all commands...
rover_in_commands = [("debug_string", 0, handle_debug_string), ("ping", 1, handle_pong) ]

