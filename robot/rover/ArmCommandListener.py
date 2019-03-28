#!/usr/bin/env python3

# make sure to run this script before ClientSender.py !
# This will listen on any incoming messages on the same network
# and display them as they get received,
# as well as send corresponding commands over serial to a usb connected arduino,
# along with acknowledgment message per drive command received

import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports
from robot.comms.connection import Connection

SERVER_PORT = 5005

# returns current time in milliseconds
currentMillis = lambda: int(round(time.time() * 1000))

if len(sys.argv) == 2:
    SERVER_PORT = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5000"
    )
    print("example usage: python ServerListener.py <port>")

# set up connection to arduino
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName)
ser = serial.Serial('/dev/' + firstPortName, 9600)
rover_ip = "172.16.1.30"

c = Connection("c1", rover_ip, SERVER_PORT)

print("Rover server listening on port {} \n".format(SERVER_PORT))

print("Ready for incoming drive cmds!\n")

RESPONSE_TIMEOUT = 75
PING_TIMEOUT = 1000
keyList = ['w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h', 'u', 'j', 'q', 'a', 'p', 'z', 'o']

while True:
    while ser.in_waiting:
        print(ser.readline().decode())

    try:
        command = c.receive()
        print("command: " + command + "\n")

        if command in keyList:
            if command == 'w':
                #returnMsg = "cmd: w --> m1: Forward"
                print("cmd: w --> m1: Forward\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge fwd ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 's':
                print("cmd: s --> m1: Back\n")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge back ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'e':
                print("cmd: e --> m2: Forward\n")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ fwd ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'd':
                print("cmd: d --> m2: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ back ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'r':
                print("cmd: r --> m3: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ fwd ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'f':
                print("cmd: f --> m3: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ back ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 't':
                print("cmd: t --> m4: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ fwd ~ ~"
                ser.write(str.encode(command))
            elif command == 'g':
                print("cmd: g --> m4: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ back ~ ~"
                ser.write(str.encode(command))
            elif command == 'y':
                print("cmd: y --> m5: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ fwd ~"
                ser.write(str.encode(command))
            elif command == 'h':
                print("cmd: h --> m5: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ back ~"
                ser.write(str.encode(command))
            elif command == 'u':
                print("cmd: u --> m6: Forward")
                #mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ fwd"
                ser.write(str.encode(command))
            elif command == 'j':
                print("cmd: j --> m6: Back")
                #mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ back"
                ser.write(str.encode(command))
            elif command == 'z':
                print("cmd: z --> stop all motors")
                #mySocket.sendto(str.encode("stop"), (clientIP, CLIENT_PORT))
                command = "stop"
                ser.write(str.encode(command))
            elif command == 'o':
                print("cmd: o --> reset angle values")
                #mySocket.sendto(str.encode("reset"), (clientIP, CLIENT_PORT))
                command = "reset"
                ser.write(str.encode(command))
            elif command == 'q':
                print("\nTerminating connection.")
                break
            elif command == 'a':
                while ser.in_waiting:
                    print(ser.readline().decode())
            elif command == 'p':
                print("cmd: p --> ping")
                command = "ping"
                ser.write(str.encode(command))
                pingTime = currentMillis()

                while currentMillis() - pingTime < PING_TIMEOUT:
                    response = ser.readline().decode()
                    #mySocket.sendto(str.encode(response), (clientIP, CLIENT_PORT))
                    print(response)

    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break

