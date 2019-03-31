#!/usr/bin/env python3

import sys
import traceback
import time
import serial
import serial.tools.list_ports

# feature toggles
usb = True
uart = False

# if all the following are False then exit right away
local = True
competition = False
# dynamic not working properly, keep at False
dynamic = False

# note: since this program is in /usr/bin/ on the OBC
# it was necessary to also add the connection.py
# class in /usr/bin and change the following line to
# from connection import Connection
# for the startup service to work properly
from robot.comms.connection import Connection
from robot.comms.uart import Uart

def print_commands_list():
    print("""'q': quit\n
'w': forward\n
's': back\n
'a': left\n
'd': right\n
'i': increase throttle speed\n
'j': decrease throttle speed\n
'o': increase steering speed\n
'k': decrease steering speed\n
'l': list commands\n\n""")

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))
# pre-determined port for the ArmCommandListener
SERVER_PORT = 5010

if len(sys.argv) == 2:
    SERVER_PORT = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5000"
    )
    print("example usage: python ServerListener.py <port>")


if not local and not competition and not dynamic:
    print("""local, competition and dynamic flags set to false, exiting""")
    sys.exit(0)


if usb:
    # set up connection to arduino
    ports = list(serial.tools.list_ports.comports())
    first_port = ports[0].name
    print("Connecting to port: " + first_port)
    ser = serial.Serial('/dev/' + first_port, 9600)
elif uart:
    u = Uart("/dev/ttySAC0", 9600)

# for local testing
if local:
    ROVER_IP = "127.0.0.1" # local testing
# for competition
elif competition:
    ROVER_IP = "172.16.1.30" # competition ip
# physicial ip, does not need connection to internet to work
#elif dynamic:
#    ROVER_IP = ni.ifaddresses(ni.interfaces()[1])[AF_INET][0]['addr']

print("ROVER_IP: " + ROVER_IP)

c = Connection("c1", ROVER_IP, SERVER_PORT)

print("Rover server listening on port {} \n".format(SERVER_PORT))

print("Ready for incoming drive cmds!\n")

print_commands_list()

key_list = ['w', 'a', 's', 'd', 'i', 'j', 'l', 'o', 'k', 't', 'q']
REST = 49.5
throttle_speed = 0
steering_speed = 0
MIN_THROTTLE_SPEED = 0
MAX_THROTTLE_SPEED = 25
MIN_STEERING_SPEED = 0
MAX_STEERING_SPEED = 39


while True:
    #while ser.in_waiting:
    #    print(ser.readline().decode())

    try:
        command = c.receive()
        print("OPERATOR command: " + command + "\n")

        # for throttle speed, no backwards
        if command in key_list:
            if command == 'w':
                print("cmd: w --> Forward\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = str(REST + throttle_speed) + ":" + str(REST) + "\n"
                print("command: " + str(command))

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'a':
                print("cmd: a --> Left\n")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = str(REST + throttle_speed) + ":" + str(REST - steering_speed) + "\n"
                print("command: " + str(command))

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 's':
                print("cmd: s --> Back\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = str(REST - throttle_speed) + ":" + str(REST) + "\n"
                print("command: " + str(command))

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'd':
                print("cmd: d --> Right")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = str(REST + throttle_speed) + ":" + str(REST + steering_speed) + "\n"
                print("command: " + str(command))

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            # 't' --> reset to 0 on release key, otherwise motor keeps spinning
            # 45.5:45.5
            elif command == 't':
                print("cmd: t --> stop moving")
                command = str(REST) + ":" + str(REST) + "\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'i':
                print("cmd: i --> Increment throttle speed")

                if throttle_speed < MAX_THROTTLE_SPEED:
                    throttle_speed += 0.5
                    print("throttle speed: " + str(throttle_speed))
                else:
                    print("throttle speed upper limit reached")

            elif command == 'j':
                print("cmd: j --> Decrement throttle speed")

                if throttle_speed > MIN_THROTTLE_SPEED:
                    throttle_speed -= 0.5
                    print("throttle speed: " + str(throttle_speed))
                else:
                    print("throttle speed lower limit reached")

            elif command == 'o':
                print("cmd: o --> Increment steering speed")

                if steering_speed < MAX_STEERING_SPEED:
                    steering_speed += 0.5
                    print("steering speed: " + str(steering_speed))
                else:
                    print("steering speed upper limit reached")

            elif command == 'k':
                print("cmd: k --> Decrement steering speed")

                if steering_speed > MIN_STEERING_SPEED:
                    steering_speed -= 0.5
                    print("steering speed: " + str(steering_speed))
                else:
                    print("steering speed lower limit reached")

            elif command == 'q':
                print("\nTerminating connection.")
                break
            elif command == 'l':
                print_commands_list()
            elif command == 'b':
                if usb:
                    while ser.in_waiting:
                        print(ser.readline().decode())
                else:
                    print("UART RX not supported (yet)")

            if usb:
                # flush buffer to avoid overflowing it
                ser.reset_input_buffer()
                ser.reset_output_buffer()

    except Exception:
        if usb:
            ser.close()
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
