#!/usr/bin/env python3
"""
This script was written to locally test sending UDP packets from flask to a listener script.
See the README in this folder for more details.
"""
import sys
import traceback
from robot.comms.connection import Connection

PORT_NUMBER = 5005
SIZE = 1024

if len(sys.argv) == 2:
    PORT_NUMBER = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5000"
    )
    print("example usage: python ServerListener.py <port>")

print("Ready for incoming drive cmds!\n")

c = Connection("c1", "127.0.0.1", 5005)

while True:
    try:
        command = c.receive()

        if command == 'w':
            print("cmd: w --> action: Forward")
            #mySocket.sendto(str.encode("Forward"), (clientIP, PORT_NUMBER))
            #ser.write(str.encode(command))
        elif command == 's':
            print("cmd: s --> action: Back")
            #mySocket.sendto(str.encode("Back"), (clientIP, PORT_NUMBER))
            #ser.write(str.encode(command))
        elif command == 'y':
            print("\nTerminating connection.")
            break

    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
