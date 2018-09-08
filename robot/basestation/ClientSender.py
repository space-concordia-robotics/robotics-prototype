#!/usr/bin/env python3

import sys, click
from socket import socket, AF_INET, SOCK_DGRAM
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

# use xset to set refresh/delay rates of keyboard input to something reasonable
TARGET_DELAY = 100
TARGET_REFRESH = 60
# these values are default on ubuntu 16.04
ORIGINAL_DELAY = 500
ORIGINAL_REFRESH = 33

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

    print("\nSetting delay rate to {} and refresh rate to {}".format(TARGET_DELAY, TARGET_REFRESH))
    setX = 'xset r rate ' + str(TARGET_DELAY) + ' ' + str(TARGET_REFRESH)
    subprocess.Popen(setX.split())

resetX = 'xset r rate ' + str(ORIGINAL_DELAY) + ' ' + str(ORIGINAL_REFRESH)

# setting up socket stuff
mySocket = socket(AF_INET, SOCK_DGRAM)
currentMillis = lambda: int(round(time.time() * 1000))
lastCmdSent = 0
THROTTLE_TIME = 100

# get and send client IP address over to server for feedback
ifconfigOutput = subprocess.run(['ifconfig'], stdout=subprocess.PIPE)
ifconfig = ifconfigOutput.stdout.decode()
lines = ifconfig.splitlines() # delimit by newline into array
myIpAddress = "ip:"

for line in lines:
    if "inet addr" in line and not "127.0.0.1" in line:
        clientIP = re.findall(r'\d+\.\d+\.\d+\.\d+', line)[0]
        myIpAddress += clientIP

print("Sending " + myIpAddress + "...")

while True:
    mySocket.sendto(str.encode(myIpAddress), (SERVER_IP, PORT_NUMBER))
    time.sleep(1)
    print("Reattempting")

while True:
    try:
        key = click.getchar()

        if currentMillis() - lastCmdSent > THROTTLE_TIME:
            print("waited {} milliseconds to move".format(currentMillis() - lastCmdSent))
            if key == 'w':
                print("Sending key: " + key + ", command: Forward")
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 's':
                print("Sending key: " + key + ", command: Back")
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                lastCmdSent = currentMillis()
            elif key == 'q':
                mySocket.sendto(str.encode(key), (SERVER_IP, PORT_NUMBER))
                print("\nTerminating connection.")
                print("\nResetting delay rate back to {} and refresh rate back to {}".format(ORIGINAL_DELAY, ORIGINAL_REFRESH))
                subprocess.Popen(resetX.split())
                break

    except:
        break
