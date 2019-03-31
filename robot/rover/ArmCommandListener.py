#!/usr/bin/env python3

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports
#from netifaces import AF_INET, AF_INET6, AF_LINK, AF_PACKET, AF_BRIDGE
#import netifaces as ni

# feature toggles
usb = True
uart = False

# if all the following are False then exit right away
local = True
competition = False
#dynamic = True

# note: since this program is in /usr/bin/ on the OBC
# it was necessary to also add the connection.py
# class in /usr/bin and change the following line to
# from connection import Connection
# for the startup service to work properly
from robot.comms.connection import Connection
from robot.comms.uart import Uart

def print_commands_list():
    print("""'q': quit\n
'p': ping\n
'z': emergency stop all motors\n
'o': reset memorized angle values\n
'l': view key commands\n
'a': post buffered messages from Teensy\n
Keys 'w' to 'u': move motors 1-6 forwards\n
Keys 's' to 'j': move motors 1-6 backwards\n\n""")

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))
# pre-determined port for the ArmCommandListener
SERVER_PORT = 5005

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
elif dynamic:
    ROVER_IP = ni.ifaddresses(ni.interfaces()[1])[AF_INET][0]['addr']

print("ROVER_IP: " + ROVER_IP)

c = Connection("c1", ROVER_IP, SERVER_PORT)

print("Rover server listening on port {} \n".format(SERVER_PORT))

print("Ready for incoming drive cmds!\n")

print_commands_list()

RESPONSE_TIMEOUT = 75
PING_TIMEOUT = 1000
key_list = ['w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h', 'u', 'j', 'q', 'a', 'p', 'z', 'o', 'l']

while True:
    if usb:
        while ser.in_waiting:
            print(ser.readline().decode())

    try:
        command = c.receive()
        print("command: " + command + "\n")

        if command in key_list:
            if command == 'w':
                print("cmd: w --> m1: Forward\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge fwd ~ ~ ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 's':
                print("cmd: s --> m1: Back\n")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge back ~ ~ ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'e':
                print("cmd: e --> m2: Forward\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ fwd ~ ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'd':
                print("cmd: d --> m2: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ back ~ ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'r':
                print("cmd: r --> m3: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ fwd ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'f':
                print("cmd: f --> m3: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ back ~ ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 't':
                print("cmd: t --> m4: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ fwd ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'g':
                print("cmd: g --> m4: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ back ~ ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'y':
                print("cmd: y --> m5: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ fwd ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'h':
                print("cmd: h --> m5: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ back ~\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'u':
                print("cmd: u --> m6: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ fwd\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'j':
                print("cmd: j --> m6: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ back\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'z':
                print("cmd: z --> stop all motors")
                #mySocket.sendto(str.encode("stop"), (clientIP, CLIENT_PORT))
                command = "stop\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'o':
                print("cmd: o --> reset angle values")
                #mySocket.sendto(str.encode("reset"), (clientIP, CLIENT_PORT))
                command = "reset\n"

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'q':
                print("\nTerminating connection.")
                break
            elif command == 'l':
                print_commands_list()
            elif command == 'a':
                if usb:
                    while ser.in_waiting:
                        print(ser.readline().decode())
                else:
                    print("UART RX not supported (yet)")
            elif command == 'p':
                if usb:
                    print("cmd: p --> ping")
                    command = "ping\n"
                    ser.write(str.encode(command))
                    ping_time = current_millis()

                    while current_millis() - ping_time < PING_TIMEOUT:
                        response = ser.readline().decode()
                        #mySocket.sendto(str.encode(response), (clientIP, CLIENT_PORT))
                        print(response)

    except Exception:
        if usb:
            ser.close()
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break

