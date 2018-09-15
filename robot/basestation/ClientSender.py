#!/usr/bin/env python3

# make sure that you start ServerListener.py process on odroid first before running this script!

import sys
import click
from socket import socket, AF_INET, SOCK_DGRAM, gethostbyname # add SOCK_STREAM here for TCP
import time
import os
import subprocess
import re

# get original delay and refresh, in case not 500, 33
def getOriginalDelayAndRefresh():
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
        originalDelay = xsetVals[0]
        originalRefresh = xsetVals[1]

        #setX = 'xset r rate ' + str(TARGET_DELAY) + ' ' + str(TARGET_REFRESH)
        #subprocess.Popen(setX.split())

        return (originalDelay, originalRefresh)

# set delay and refresh rate using UNIX xset
def setX(delay, refresh):
    setXCmd = 'xset r rate ' + str(delay) + ' ' + str(refresh)
    print("\nSetting delay rate to {} and refresh rate to {} ...".format(delay, refresh))
    subprocess.Popen(setXCmd.split())

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
originalDelay = 500
originalRefresh = 33

originalXsetVals = getOriginalDelayAndRefresh()
originalDelay = originalXsetVals[0]
originalRefresh = originalXsetVals[1]

hostName = gethostbyname('0.0.0.0')
# UDP socket for sending drive cmds
mySocket = socket(AF_INET, SOCK_DGRAM)
# make socket reachable by any address (rather than only visible to same machine that it's running on)
mySocket.bind((hostName, PORT_NUMBER))

currentMillis = lambda: int(round(time.time() * 1000))
lastCmdSent = 0
THROTTLE_TIME = 100

# get my own IP address
def getMyIP():
    ifconfigOutput = subprocess.run(['ifconfig'], stdout=subprocess.PIPE)
    ifconfig = ifconfigOutput.stdout.decode()
    lines = ifconfig.splitlines() # delimit by newline into array
    myIpAddress = ""

    for line in lines:
        if "inet addr" in line and not "127.0.0.1" in line:
            myIpAddress = re.findall(r'\d+\.\d+\.\d+\.\d+', line)[0]

    return myIpAddress

handshake = ""
# format: 'ip:192.168.2.13'
clientIP = "ip:" + getMyIP()

# send client IP address over to server for feedback
while True:
    try:
        print("Attempting to send: \"" + clientIP + "\" ...")
        mySocket.sendto(str.encode(clientIP), (SERVER_IP, PORT_NUMBER))
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
        setX(originalDelay, originalRefresh)
        break

setX(TARGET_DELAY, TARGET_REFRESH)
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
                #print("Resetting delay rate back to {} and refresh rate back to {}".format(originalDelay, originalRefresh))
                setX(originalDelay, originalRefresh)
                break

            # wait till receive a response
            (feedback, addr) = mySocket.recvfrom(SIZE)

            if feedback:
                print(feedback.decode())

    except:
        setX(originalDelay, originalRefresh)
        break

