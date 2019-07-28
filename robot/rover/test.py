#!/usr/bin/env python3
import serial
import time
import sys

# default port
#port = "/dev/ttyACM0"
port = "/dev/ttySAC0"
baudrate = 9600
msg = "who\n"
who = False
listen = False

if len(sys.argv) <= 3:
    print("Not enough args, look at source code")
    sys.exit(0)
elif len(sys.argv) == 4:
    '''
    if sys.argv[1] == "who":
        print("asking who")
        listen = True
    elif sys.argv[1] == "listen":
        listen = True
        who = False
    else:
        port = sys.argv[1]
        baudrate = sys.argv[2]
        msg = sys.argv[3]
    '''
    port = sys.argv[1]
    baudrate = sys.argv[2]
    msg = sys.argv[3]

ser = serial.Serial(port, baudrate)

print("getting rid of extra stuff")


# show me what you got
try:
    while ser.in_waiting:
        print('ser.readline().decode():', ser.readline().decode())
except:
    pass

while True:

    print("sending msg:", msg)
    ser.write(str.encode(msg))

    while ser.in_waiting:
        print("response: ", ser.readline().decode())

    time.sleep(1)
    '''
    if who:
        print("cmd sent:", msg)
        ser.write(str.encode(msg))

    if listen:
        while ser.in_waiting:
            print(ser.readline().decode())

    repeat = input("again? y/n")

    if repeat == "y":
        pass
    else:
        break
    '''
'''
for i in 0, 3:
    who = ""
    print("sending cmd")
    ser.write(str.encode("who\n"))

    while ser.in_waiting:
        who = ser.readline().decode()
        print("who: " + who)

    if who.strip() == "arm":
        print("It is the arm MCU!")
    elif who.strip() == "rover":
        print("It is the rover MCU!")
    time.sleep(1)
'''
