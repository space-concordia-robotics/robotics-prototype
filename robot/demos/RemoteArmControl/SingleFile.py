#!/usr/bin/env python3

# make sure that you start ServerListener.py process on odroid first before running this script!

import click
from socket import socket, AF_INET, SOCK_DGRAM, gethostbyname # add SOCK_STREAM here for TCP
import time
import os
import subprocess
import re
import sys, traceback
import serial
import serial.tools.list_ports

# returns current time in milliseconds
currentMillis = lambda: int(round(time.time() * 1000))

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

SIZE = 1024

print(
    "Base station client sending command packets ('w' or 's') to Teensy\nPress q to terminate connection.")

# use xset to set refresh/delay rates of keyboard input to something reasonable
TARGET_DELAY = 100
TARGET_REFRESH = 60
# these values are default on ubuntu 16.04
originalDelay = 500
originalRefresh = 33
# get original xset delay and refresh values
originalXsetVals = getOriginalDelayAndRefresh()
originalDelay = originalXsetVals[0]
originalRefresh = originalXsetVals[1]

# for controlling command throughput
lastCmdSent = 0
THROTTLE_TIME = 100

# set up connection to arduino
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName)
ser = serial.Serial('/dev/' + firstPortName, 115200)

# send pinh over to Teensy for feedback
# while True:
#     try:
#         print("Attempting to send: ping ...")
#         ser.write(str.encode("ping"))
#         print("ping sent\n")
#         #time.sleep(1)
#
#         print("Listening for acknowledgement from Teensy ...")
#         ans = ser.readline()
#         print(ans)
#         #print(ans[2:][:-5])
#         # how do i listen to Teensy?
#
#         # listen (right away) for incoming acknowledgement
#         #(handshake, addr) = mySocket.recvfrom(SIZE)
#         #handshake = handshake.decode()
#         #print("Acknowledgment received: " + str(handshake) + "\n")
#
#         #if handshake == IP_KNOWN:
#         #    break
#
#     except:
#         setX(originalDelay, originalRefresh)
#         break

setX(TARGET_DELAY, TARGET_REFRESH)
print("Ready for sending drive commands!\n")

while True:
    while ser.in_waiting:
        print(ser.readline())
    try:
        key = click.getchar()

        if currentMillis() - lastCmdSent > THROTTLE_TIME:
            # for debugging
            #print("waited {} milliseconds to move".format(currentMillis() - lastCmdSent))
            if key == 'w':
                print("Sending key: " + key)
                print("cmd: w --> m1: Forward")
                command = "budge fwd ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 's':
                print("Sending key: " + key)
                print("cmd: s --> m1: Back")
                command = "budge back ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'e':
                print("Sending key: " + key)
                print("cmd: e --> m2: Forward")
                command = "budge ~ fwd ~ ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'd':
                print("Sending key: " + key)
                print("cmd: d --> m2: Back")
                command = "budge ~ back ~ ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'r':
                print("Sending key: " + key)
                print("cmd: r --> m3: Forward")
                command = "budge ~ ~ fwd ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'f':
                print("Sending key: " + key)
                print("cmd: f --> m3: Back")
                command = "budge ~ ~ back ~ ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 't':
                print("Sending key: " + key)
                print("cmd: t --> m4: Forward")
                command = "budge ~ ~ ~ fwd ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'g':
                print("Sending key: " + key)
                print("cmd: g --> m4: Back")
                command = "budge ~ ~ ~ back ~ ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'y':
                print("Sending key: " + key)
                print("cmd: y --> m5: Forward")
                command = "budge ~ ~ ~ ~ fwd ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'h':
                print("Sending key: " + key)
                print("cmd: h --> m5: Back")
                command = "budge ~ ~ ~ ~ back ~"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'u':
                print("Sending key: " + key)
                print("cmd: u --> m6: Forward")
                command = "budge ~ ~ ~ ~ ~ fwd"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'j':
                print("Sending key: " + key)
                print("cmd: j --> m6: Back")
                command = "budge ~ ~ ~ ~ ~ back"
                ser.write(str.encode(command))
                lastCmdSent = currentMillis()
            elif key == 'q':
                print("\nTerminating connection.")
                #print("Resetting delay rate back to {} and refresh rate back to {}".format(originalDelay, originalRefresh))
                setX(originalDelay, originalRefresh)
                break

            # wait till receive a response
            #(feedback, addr) = mySocket.recvfrom(SIZE)

            #if feedback:
            #    print(feedback.decode())

    except:
        setX(originalDelay, originalRefresh)
        break
