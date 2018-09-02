#!/usr/bin/env python3

import sys, click
from socket import socket, AF_INET, SOCK_DGRAM
import time

PORT_NUMBER = 5000
SIZE = 1024

if len(sys.argv) < 2:  # first one is the name of the file
    print(
        "please pass a destination ip address/port of the listener to send this message to, port is optional and will default to 5000"
    )
    print("example usage: python ClientSender.py 127.0.0.1 5000")
    sys.exit(1)

if len(sys.argv) == 3:
    PORT_NUMBER = int(sys.argv[2])
elif len(sys.argv) > 3:
    print("too many args")
    sys.exit(1)

SERVER_IP = sys.argv[1]

print(
    "Base station client sending command packets ('w' or 's') to IP {}, via port {}\nPress q to terminate connection.".
    format(SERVER_IP, PORT_NUMBER))

mySocket = socket(AF_INET, SOCK_DGRAM)
currentMillis = lambda: int(round(time.time() * 1000))
lastCmdSent = 0
THROTTLE_TIME = 100

while True:
    try:

        if currentMillis() - lastCmdSent > THROTTLE_TIME:
            key = click.getchar()

            if key == 'w':
                print("Sending key: " + key + ", cmd --> Forward")
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 's':
                print("Sending key: " + key + ", cmd --> Back")
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 'q':
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                print("\nTerminating connection.")
                break
    except:
        break
