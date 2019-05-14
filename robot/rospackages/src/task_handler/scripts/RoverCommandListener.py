#!/usr/bin/env python3

import sys
import traceback
import time
import serial
import serial.tools.list_ports

# feature toggles
usb = False
uart = True

# if all the following are False then exit right away
local = False
competition = True
# dynamic not working properly, keep at False
dynamic = False

# note: since this program is in /usr/bin/ on the OBC
# it was necessary to also add the connection.py
# class in /usr/bin and change the following line to
# from connection import Connection
# for the startup service to work properly
from robot.comms.connection import Connection
from robot.comms.uart import Uart

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))

def get_commands_list():
    return """'q': quit\n
'w': forward\n
's': back\n
'a': left\n
'd': right\n
'i': increase throttle speed\n
'j': decrease throttle speed\n
'o': increase steering speed\n
'k': decrease steering speed\n
'l': list commands\n\n"""

# returns current time in milliseconds
current_millis = lambda: int(round(time.time() * 1000))

if len(sys.argv) == 2:
    ROVER_PORT = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5010"
    )
    print("example usage: python ServerListener.py <port>")


if not local and not competition and not dynamic:
    print("local, competition and dynamic flags set to false, exiting")
    sys.exit(0)


if usb:
    # set up connection to arduino
    ports = list(serial.tools.list_ports.comports())
    is_rover = False

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
                print("who: " + who)

            if who.strip() == "rover":
                print("Rover MCU identified!")
                is_rover = True
    elif len(ports) == 2:
        print("2 USB devices detected")
        port = ports[1].name
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

            if who.strip() == "rover":
                print("Rover MCU identified!")
                is_rover = True

        if not is_rover:
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

                if who.strip() == "rover":
                    print("Rover MCU identified!")
                    is_rover = True

    else:
        print("No USB devices recognized, exiting")
        sys.exit(0)

    if is_rover:
        print("Connected to port: " + port)
    else:
        print("Incorrect MCU connected, terminating listener")
        sys.exit(0)
elif uart:
    u = Uart("/dev/ttySAC0", 9600, timeout=1)

# for local testing
if local:
    ROVER_IP = "127.0.0.1"
    ROVER_PORT = 5020
    BASE_IP = ROVER_IP
    BASE_PORT = 5025
# for competition
elif competition:
    ROVER_IP = "172.16.1.30"
    ROVER_PORT = 5030
    BASE_IP = "172.16.1.20"
    BASE_PORT = ROVER_PORT
# attempt to get: physicial ip, which should not need connection to internet to work
#elif dynamic:
#    ROVER_IP = ni.ifaddresses(ni.interfaces()[1])[AF_INET][0]['addr']

print("ROVER_IP: " + ROVER_IP)
print("BASE_IP: " + BASE_IP)

# Create connection object to send/receive data with base-station
receiver = Connection("rover_drive_receiver", ROVER_IP, ROVER_PORT)
sender = Connection("rover_feedback_sender", BASE_IP, BASE_PORT)

print("Rover server listening on port {} \n".format(ROVER_PORT))

print("Ready for incoming drive cmds!\n")

print(get_commands_list())

key_list = ['w', 'a', 's', 'd', 'i', 'j', 'l', 'o', 'k', 'm', 'n', 't', 'b', 'q']
# Arduino sketch considers this value to be the signal for the motors to not move
REST = 49.5

# initialize throttle/steering speeds to 0
throttle_speed = 0
steering_speed = 0

# impose safety limits, theoretical limit at 49.5
MIN_THROTTLE_SPEED = 0
MAX_THROTTLE_SPEED = 25
MIN_STEERING_SPEED = 0
MAX_STEERING_SPEED = 39

# for controlling command throughput
last_cmd_sent = 0 # to keep track of the time of the last command sent
THROTTLE_TIME = 25 # time to wait before receiving next command


while True:
    #while ser.in_waiting:
    #    print(ser.readline().decode())

    try:
        # only receive commands if last command was sent
        # THROTTLE_TIME ago (in milliseconds)
        if current_millis() - last_cmd_sent > THROTTLE_TIME:

            command = receiver.receive()

            # for throttle speed, no backwards
            if command in key_list:
                print("OPERATOR command: " + command + " recognized\n")

                if command == 'w':
                    feedback = "cmd: w --> Forward\n"
                    command = str(REST + throttle_speed) + ":" + str(REST) + "\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                    last_cmd_sent = current_millis()

                elif command == 'a':
                    feedback = "cmd: a --> Left\n"
                    command = str(REST + throttle_speed) + ":" + str(REST - steering_speed) + "\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                    last_cmd_sent = current_millis()

                elif command == 's':
                    feedback = "cmd: s --> Back\n"
                    command = str(REST - throttle_speed) + ":" + str(REST) + "\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                    last_cmd_sent = current_millis()

                elif command == 'd':
                    feedback = "cmd: d --> Right"
                    command = str(REST + throttle_speed) + ":" + str(REST + steering_speed) + "\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                    last_cmd_sent = current_millis()

                elif command == 'm':
                    feedback = "cmd: m --> enable motor controls"
                    command = "activate\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                elif command == 'n':
                    feedback = "cmd: n --> disable motor controls"
                    command = "deactivate\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                # 't' --> reset to 0 on release key, otherwise motor keeps spinning
                # 45.5:45.5
                elif command == 't':
                    feedback = "cmd: t --> stop moving"
                    command = str(REST) + ":" + str(REST) + "\n"
                    feedback += "\ncommand: " + str(command)
                    print(feedback)
                    sender.send(feedback)

                    if usb:
                        ser.write(str.encode(command))
                    elif uart:
                        u.tx(command)

                    last_cmd_sent = current_millis()

                elif command == 'i':
                    feedback = "cmd: i --> Increment throttle speed"

                    if throttle_speed < MAX_THROTTLE_SPEED:
                        throttle_speed += 0.5
                        feedback += "\nthrottle speed: " + str(throttle_speed)
                    else:
                        feedback += "\nthrottle speed upper limit reached"

                    print(feedback)
                    sender.send(feedback)

                elif command == 'j':
                    feedback = "cmd: j --> Decrement throttle speed"

                    if throttle_speed > MIN_THROTTLE_SPEED:
                        throttle_speed -= 0.5
                        feedback += "\nthrottle speed: " + str(throttle_speed)
                    else:
                        feedback += "\nthrottle speed lower limit reached"

                    print(feedback)
                    sender.send(feedback)

                elif command == 'o':
                    feedback = "cmd: o --> Increment steering speed"

                    if steering_speed < MAX_STEERING_SPEED:
                        steering_speed += 0.5
                        feedback += "\nsteering speed: " + str(steering_speed)
                    else:
                        feedback += "\nsteering speed upper limit reached"

                    print(feedback)
                    sender.send(feedback)

                elif command == 'k':
                    feedback = "cmd: k --> Decrement steering speed"

                    if steering_speed > MIN_STEERING_SPEED:
                        steering_speed -= 0.5
                        feedback += "\nsteering speed: " + str(steering_speed)
                    else:
                        feedback += "\nsteering speed lower limit reached"

                    print(feedback)
                    sender.send(feedback)

                elif command == 'q':
                    feedback = "\nTerminating connection."

                    print(feedback)
                    sender.send(feedback)

                    break

                elif command == 'l':
                    print(get_commands_list())
                    sender.send(get_commands_list())

                elif command == 'b':
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
            else:
                print("UNKOWN command " + command + "\n")

            if usb:
                # flush buffer to avoid overflowing it
                ser.reset_input_buffer()
                ser.reset_output_buffer()

    except Exception:
        if usb:
            ser.close()
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
