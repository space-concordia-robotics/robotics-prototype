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
usb = False
uart = True

# if all the following are False then exit right away
local = False
competition = True
#dynamic = True

# note: since this program is in /usr/bin/ on the OBC
# it was necessary to also add the connection.py
# class in /usr/bin and change the following line to
# from connection import Connection
# for the startup service to work properly
from robot.comms.connection import Connection
from robot.comms.uart import Uart

def get_commands_list():
    return """'q': quit\n
'p': ping\n
'z': emergency stop all motors\n
'o': reset memorized angle values\n
'l': view key commands\n
'a': post buffered messages from Teensy\n
Keys 'w' to 'u': move motors 1-6 forwards\n
Keys 's' to 'j': move motors 1-6 backwards\n\n"""

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))

if len(sys.argv) == 2:
    ROVER_PORT = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5000"
    )
    print("example usage: python ServerListener.py <port>")


if not local and not competition: #and not dynamic:
    print("local, competition flags set to false, exiting")
    sys.exit(0)

if usb:
    # set up connection to arduino
    ports = list(serial.tools.list_ports.comports())
    is_arm = False

    if len(ports) == 1:
        print("1 USB device detected")
        port = ports[0].name
        ser = serial.Serial('/dev/' + port, 9600)

        print("clearing buffer")
        while ser.in_waiting:
            print(ser.readline().decode())

        for i in 0, 3:
            who = ""
            print("identifying MCU")
            ser.write(str.encode("who\n"))

            # CRITICAL: give time for MCU to respond
            time.sleep(1)

            while ser.in_waiting:
                who = ser.readline().decode()
                print("who: \"" + who.strip() + "\"")
                if "arm" in who.strip():
                    print("Arm MCU idenified!")
                    is_arm = True

    elif len(ports) == 2:
        print("2 USB devices detected")
        port = ports[1].name
        ser = serial.Serial('/dev/' + port, 9600)

        print("clearing buffer")
        while ser.in_waiting:
            print(ser.readline().decode())

        for i in 0, 3:
            who = ""
            print("identifiying MCU")
            ser.write(str.encode("who\n"))

            # CRITICAL: give time for the MCU to respond
            time.sleep(1)

            while ser.in_waiting:
                who = ser.readline().decode()
                print("who: " + who)

                if "arm" in who.strip():
                    print("Arm MCU idenified!")
                    is_arm = True

        if not is_arm:
            port = ports[0].name
            ser = serial.Serial('/dev/' + port, 9600)

            print("clearing buffer")
            while ser.in_waiting:
                print(ser.readline().decode())

            for i in 0, 3:
                who = ""
                print("identifying MCU")
                ser.write(str.encode("who\n"))

                # CRITICAL: give time for MCU to respond
                time.sleep(1)

                while ser.in_waiting:
                    who = ser.readline().decode()
                    print("who: " + who)
                    if "arm" in who.strip():
                        print("Arm MCU idenified!")
                        is_arm = True

    else:
        print("No USB devices recognized, exiting")
        sys.exit(0)

    if is_arm:
        print("Connected to port: " + port)

    else:
        print("Incorrect MCU connected, terminating listener")
        sys.exit(0)
elif uart:
    u = Uart("/dev/ttySAC0", 9600, timeout=3)

ROVER_PORT = ""
BASE_PORT = ""
# for local testing
if local:
    ROVER_IP = "127.0.0.1"
    BASE_IP = ROVER_IP
    ROVER_PORT = 5005
    BASE_PORT = 5010
# for competition
elif competition:
    ROVER_IP = "172.16.1.30"
    BASE_IP = "172.16.1.20"
    ROVER_PORT = 5015
    BASE_PORT = ROVER_PORT
# physicial ip, does not need connection to internet to work
# elif dynamic:
#     ROVER_IP = ni.ifaddresses(ni.interfaces()[1])[AF_INET][0]['addr']

print("ROVER_IP: " + ROVER_IP)

receiver = Connection("arm_command_receiver", ROVER_IP, ROVER_PORT)
sender = Connection("arm_feedback_sender", BASE_IP, BASE_PORT)


print("Rover server listening on port {} \n".format(ROVER_PORT))

print("Ready for incoming drive cmds!\n")

print(get_commands_list())

RESPONSE_TIMEOUT = 75
PING_TIMEOUT = 1000
key_list = ['w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h', 'u', 'j', 'q', 'a', 'p', 'z', 'o', 'l']

while True:
    if usb:
        while ser.in_waiting:
            print("ser.readline():", ser.readline().decode())

    try:
        command = receiver.receive()
        print("command: " + command + "\n")

        if command in key_list:
            if command == 'w':
                feedback = "cmd: w --> m1: Forward\n"
                command = "budge fwd ~ ~ ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 's':
                feedback = "cmd: s --> m1: Back\n"
                command = "budge back ~ ~ ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'e':
                feedback = "cmd: e --> m2: Forward\n"
                command = "budge ~ fwd ~ ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'd':
                feedback = "cmd: d --> m2: Back"
                command = "budge ~ back ~ ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'r':
                feedback = "cmd: r --> m3: Forward"
                command = "budge ~ ~ fwd ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)


                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'f':
                feedback = "cmd: f --> m3: Back"
                command = "budge ~ ~ back ~ ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 't':
                feedback = "cmd: t --> m4: Forward"
                command = "budge ~ ~ ~ fwd ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'g':
                feedback = "cmd: g --> m4: Back"
                command = "budge ~ ~ ~ back ~ ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'y':
                feedback = "cmd: y --> m5: Forward"
                command = "budge ~ ~ ~ ~ fwd ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'h':
                feedback = "cmd: h --> m5: Back"
                command = "budge ~ ~ ~ ~ back ~\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'u':
                feedback = "cmd: u --> m6: Forward"
                command = "budge ~ ~ ~ ~ ~ fwd\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'j':
                feedback = "cmd: j --> m6: Back"
                command = "budge ~ ~ ~ ~ ~ back\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'z':
                feedback = "cmd: z --> stop all motors"
                command = "stop\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'o':
                feedback = "cmd: o --> reset angle values"
                command = "reset\n"
                feedback += "\ncommand: " + command
                print(feedback)
                sender.send(feedback)

                if usb:
                    ser.write(str.encode(command))
                elif uart:
                    u.tx(command)

            elif command == 'q':
                feedback = "\nTerminating listener."
                print(feedback)
                sender.send(feedback)
                break

            elif command == 'l':
                print(get_commands_list())
                sender.send(get_commands_list())

            elif command == 'a':
                if usb:
                    data = ""
                    while ser.in_waiting:
                        data += ser.readline().decode()
                    print(data)
                    sender.send(data)
                elif uart:
                    data = u.rx()
                    print(data)
                    sender.send(data)
                    #print("UART RX not supported (yet)")
            elif command == 'p':
                if usb:
                    feedback = "cmd: p --> ping"
                    command = "ping\n"
                    feedback += "\ncommand: " + command

                    for i in 0, 3:
                        response = ""
                        ser.write(str.encode(command))

                        # CRITICAL: give time for MCU to respond
                        time.sleep(0.3)

                        while ser.in_waiting:
                            response = ser.readline().decode()
                            if "pong" in response.strip():
                                sender.send(response)
                                break

                elif uart:
                    print("sending ping command over UART")
                    u.tx(command)

    except Exception:
        if usb:
            ser.close()
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
