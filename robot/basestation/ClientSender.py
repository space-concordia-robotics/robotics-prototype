#!/usr/bin/env python3

import sys
import click
from socket import socket, AF_INET, SOCK_DGRAM, gethostbyname # add SOCK_STREAM here for TCP
import time
import os
import subprocess
import re

PORT_NUMBER = 5000
SIZE = 1024

if len(sys.argv) < 2:  # first one is the name of the file
    print(
        "please pass a destination ip address/port of the listener to send this message to, port is optional and will default to 5000"
    )
    print("example usage: python ClientSender.py <ip of odroid> <optional port number, default: 5000>")
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

# use xset to set refresh/delay rates of keyboard input to something reasonable
TARGET_DELAY = 100
TARGET_REFRESH = 60
# these values are default on ubuntu 16.04
ORIGINAL_DELAY = 500
ORIGINAL_REFRESH = 33

# get original delay and refresh, in case not 500, 33
if os.name == "posix":
    xsetOutput = subprocess.run(['xset', '-q'], stdout=subprocess.PIPE)
    xsetConfig = xsetOutput.stdout.decode()
    lines = xsetConfig.splitlines() # delimit by newline into array
    delayAndRefreshStr = "empty"

    # get the specific line we are looking for
    for line in lines:
        if "delay" in line:
            delayAndRefreshStr = line

    xsetVals = re.findall(r'\d+', delayAndRefreshStr)
    ORIGINAL_DELAY = xsetVals[0]
    ORIGINAL_REFRESH = xsetVals[1]

    print("\nSetting delay rate to {} and refresh rate to {} ...".format(TARGET_DELAY, TARGET_REFRESH))
    setX = 'xset r rate ' + str(TARGET_DELAY) + ' ' + str(TARGET_REFRESH)
    subprocess.Popen(setX.split())

def setX(delay, refresh):
    setXCmd = 'xset r rate ' + str(delay) + ' ' + str(refresh)
    subprocess.Popen(setXCmd.split())

hostName = gethostbyname('0.0.0.0')
# UDP socket for sending drive cmds
mySocket = socket(AF_INET, SOCK_DGRAM)
# make socket reachable by any address (rather than only visible to same machine that it's running on)
mySocket.bind((hostName, PORT_NUMBER))

# potential TCP stuff?
## TCP socket for sending server the client's IP address
## TcpSocket = socket(AF_INET, SOCK_STREAM)

currentMillis = lambda: int(round(time.time() * 1000))
lastCmdSent = 0
THROTTLE_TIME = 100

# get and send client IP address over to server for feedback
ifconfigOutput = subprocess.run(['ifconfig'], stdout=subprocess.PIPE)
ifconfig = ifconfigOutput.stdout.decode()
lines = ifconfig.splitlines() # delimit by newline into array
myIpAddress = "ip:"
handshake = ""

for line in lines:
    if "inet addr" in line and not "127.0.0.1" in line:
        clientIP = re.findall(r'\d+\.\d+\.\d+\.\d+', line)[0]
        myIpAddress += clientIP

while True:
    try:
        print("Attempting to send: \"" + myIpAddress + "\" ...")
        mySocket.sendto(str.encode(myIpAddress), (SERVER_IP, PORT_NUMBER))
        print("IP address sent\n")
        #time.sleep(1)

        print("Listening for acknowledgement from server ...")

        # listen (right away) for incoming acknowledgement
        (handshake, addr) = mySocket.recvfrom(SIZE)
        handshake = handshake.decode()
        print("Acknowledgment received: " + str(handshake) + "\n")

        if handshake == "ip_known":
            break

    except:
        setX(ORIGINAL_DELAY, ORIGINAL_REFRESH)
        break

print("Ready for sending drive commands!\n")

while True:
    try:
        key = click.getchar()

        if currentMillis() - lastCmdSent > THROTTLE_TIME:
            # for debugging
            #print("waited {} milliseconds to move".format(currentMillis() - lastCmdSent))
            if key == 'w':
                print("Sending key: " + key)
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 's':
                print("Sending key: " + key)
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 'q':
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                print("\nTerminating connection.")
                print("Resetting delay rate back to {} and refresh rate back to {}".format(ORIGINAL_DELAY, ORIGINAL_REFRESH))
                setX(ORIGINAL_DELAY, ORIGINAL_REFRESH)
                break

            # wait till receive a response
            (feedback, addr) = mySocket.recvfrom(SIZE)

            if feedback:
                print(feedback.decode())

    except:
        setX(ORIGINAL_DELAY, ORIGINAL_REFRESH)
        break
