#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports #pyserial

def get_commands_list():
    return """'q': quit\n
'p': ping\n
's': emergency stop all motors\n
'o': reset memorized angle values\n
'l': view key commands\n\n"""

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))

# set up connection to arduino
ports = list(serial.tools.list_ports.comports())
is_arm = False

if len(ports) == 1:
    print("1 USB device detected")
    port = ports[0].name
    ser = serial.Serial('/dev/' + port, 9600)

    print("clearing buffer")
    while ser.in_waiting:
        print(ser.readline().decode())

    for i in 0, 3:
        who = ""
        print("identifying MCU")
        ser.write(str.encode("who\n"))

        # CRITICAL: give time for MCU to respond
        time.sleep(1)

        while ser.in_waiting:
            who = ser.readline().decode()
            print("who: \"" + who.strip() + "\"")
            if "arm" in who.strip():
                print("Arm MCU idenified!")
                is_arm = True

else:
    print("No USB devices recognized, exiting")
    sys.exit(0)

if is_arm:
    print("Connected to port: " + port)
else:
    print("Incorrect MCU connected, terminating listener")
    sys.exit(0)

ROVER_PORT = 5000
BASE_PORT = 5005

print("Rover server listening on port {} \n".format(ROVER_PORT))
print("Ready for incoming drive cmds!\n")
print(get_commands_list())

RESPONSE_TIMEOUT = 75
PING_TIMEOUT = 1000
key_list = ['p', 'l', 's', 'o', 'q']

while True:
    while ser.in_waiting:
        print("ser.readline():", ser.readline().decode())

    try:
        command = 'p' #change this so i can actually send inputs lol
        print("command: " + command + "\n")

        if command in key_list:
            if command == 's':
                feedback = "cmd: s --> stop all motors"
                command = "stop\n"
                feedback += "\ncommand: " + command
                print(feedback)
                ser.write(str.encode(command))

            elif command == 'o':
                feedback = "cmd: o --> reset angle values"
                command = "reset\n"
                feedback += "\ncommand: " + command
                print(feedback)
                ser.write(str.encode(command))

            elif command == 'q':
                feedback = "\nTerminating listener."
                print(feedback)
                break

            elif command == 'l':
                print(get_commands_list())

            elif command == 'p':
                feedback = "cmd: p --> ping"
                command = "ping\n"
                feedback += "\ncommand: " + command
                print(feedback)

                for i in 0, 3:
                    response = ""
                    ser.write(str.encode(command))

                    # CRITICAL: give time for MCU to respond
                    time.sleep(0.3)

                    while ser.in_waiting:
                        response = ser.readline().decode()
                        if "pong" in response.strip():
                            print(response)
                            break

    except Exception:
        ser.close()
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break

    time.sleep(1)
