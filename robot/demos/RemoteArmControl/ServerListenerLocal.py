#!/usr/bin/env python3

# make sure to run this script before ClientSender.py !
# This will listen on any incoming messages on the same network
# and display them as they get received,
# as well as send corresponding commands over serial to a usb connected arduino,
# along with acknowledgment message per drive command received

from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys
import traceback
import time
import re
import serial
import serial.tools.list_ports

import subprocess

# returns current time in milliseconds
currentMillis = lambda: int(round(time.time() * 1000))

CLIENT_PORT = 5000
SERVER_PORT = 5001
SIZE = 1024

if len(sys.argv) == 2:
    PORT_NUMBER = int(sys.argv[1])
elif len(sys.argv) >= 3:
    print(
        "too many arguments, one optional argument is the port number, otherwise default to 5000"
    )
    print("example usage: python ServerListener.py <port>")

hostName = gethostbyname('0.0.0.0')
mySocket = socket(AF_INET, SOCK_DGRAM)
mySocket.bind((hostName, SERVER_PORT))
#TcpSocket = socket(AF_INET, SOCK_STREAM)

# set up connection to arduino
'''
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName + "\n")
ser = serial.Serial('/dev/' + firstPortName, 115200)
'''
device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
df = subprocess.check_output("lsusb")
devices = []
for i in df.split('\n'):
    if i:
        info = device_re.match(i)
        if info:
            dinfo = info.groupdict()
            dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            devices.append(dinfo)
print devices

'''
should look like:
[
{'device': '/dev/bus/usb/001/009', 'tag': 'Apple, Inc. Optical USB Mouse [Mitsumi]', 'id': '05ac:0304'},
{'device': '/dev/bus/usb/001/001', 'tag': 'Linux Foundation 2.0 root hub', 'id': '1d6b:0002'},
{'device': '/dev/bus/usb/001/002', 'tag': 'Intel Corp. Integrated Rate Matching Hub', 'id': '8087:0020'},
{'device': '/dev/bus/usb/001/004', 'tag': 'Microdia ', 'id': '0c45:641d'}
]
so the following should work:
'''
for device in devices:
	if 'Teensyduino' in device['tag']:
		print(device['device'])
		ser = serial.Serial(device['device'], 115200)
		break

clientIP = ""
IP_KNOWN = "ip_known"

print("Rover server listening on port {} \n".format(SERVER_PORT))

try:
    (data, addr) = mySocket.recvfrom(SIZE)
    clientIpMsg = data.decode()

    # for debugging
    #print("clientIpMsg = " + clientIpMsg)
    clientIP = re.findall('\d+\.\d+\.\d+\.\d+', clientIpMsg)[0]

    if clientIP:
        print("Client IP received: " + str(clientIP) + "\n")
        time.sleep(1)
        print("Sending acknowledgment\n")
        mySocket.sendto(str.encode(IP_KNOWN), (clientIP, CLIENT_PORT))
        print("Acknowledgement sent")
except:
    print("Exception in user code:")
    print("-"*60)
    traceback.print_exc(file=sys.stdout)
    print("-"*60)

print("Ready for incoming drive cmds!\n")

RESPONSE_TIMEOUT = 75
PING_TIMEOUT = 1000
keyList = ['w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h', 'u', 'j', 'q', 'a', 'p']

while True:
    while ser.in_waiting:
        print(ser.readline().decode())
    try:
        (data, addr) = mySocket.recvfrom(SIZE)
        command = data.decode()
        print("command: " + command + "\n")

        # # wait till receive a response
        # try:
        #     checkResponse = currentMillis()
        #     while checkResponse < RESPONSE_TIMEOUT:
        #         (data, addr) = mySocket.recvfrom(SIZE)
        #     if data:
        #         command = data.decode()
        #         print("command: " + command + "\n")
        # except:
        #     continue

        if command in keyList:
            if command == 'w':
                #returnMsg = "cmd: w --> m1: Forward"
                print("cmd: w --> m1: Forward\n")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge fwd ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 's':
                print("cmd: s --> m1: Back\n")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge back ~ ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'e':
                print("cmd: e --> m2: Forward\n")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ fwd ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'd':
                print("cmd: d --> m2: Back")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ back ~ ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'r':
                print("cmd: r --> m3: Forward")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ fwd ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 'f':
                print("cmd: f --> m3: Back")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ back ~ ~ ~"
                ser.write(str.encode(command))
            elif command == 't':
                print("cmd: t --> m4: Forward")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ fwd ~ ~"
                ser.write(str.encode(command))
            elif command == 'g':
                print("cmd: g --> m4: Back")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ back ~ ~"
                ser.write(str.encode(command))
            elif command == 'y':
                print("cmd: y --> m5: Forward")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ fwd ~"
                ser.write(str.encode(command))
            elif command == 'h':
                print("cmd: h --> m5: Back")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ back ~"
                ser.write(str.encode(command))
            elif command == 'u':
                print("cmd: u --> m6: Forward")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ fwd"
                ser.write(str.encode(command))
            elif command == 'j':
                print("cmd: j --> m6: Back")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "budge ~ ~ ~ ~ ~ back"
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
                    mySocket.sendto(str.encode(response), (clientIP, CLIENT_PORT))
                    print(response)

    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
