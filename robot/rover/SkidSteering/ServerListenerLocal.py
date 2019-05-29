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
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName + "\n")
ser = serial.Serial('/dev/' + firstPortName, 9600)
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
keyList = ['q', 'x', 'p', 'w', 'a', 's', 'd', 'l', 'm']

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
                print("cmd: w --> Forward\n")
                mySocket.sendto(str.encode("Forward"), (clientIP, CLIENT_PORT))
                command = "w"
                ser.write(str.encode(command))
            elif command == 's':
                print("cmd: s --> Back\n")
                mySocket.sendto(str.encode("Back"), (clientIP, CLIENT_PORT))
                command = "s"
                ser.write(str.encode(command))
            elif command == 'a':
                print("cmd: a --> Rotate Left")
                mySocket.sendto(str.encode("Left"), (clientIP, CLIENT_PORT))
                command = "a"
                ser.write(str.encode(command))
            elif command == 'd':
                print("cmd: d --> Rotate Right")
                mySocket.sendto(str.encode("Right"), (clientIP, CLIENT_PORT))
                command = "d"
                ser.write(str.encode(command))
            elif command == 'l':
                print("cmd: l --> Less speed")
                mySocket.sendto(str.encode("Less Speed"), (clientIP, CLIENT_PORT))
                command = "l"
                ser.write(str.encode(command))
            elif command == 'm':
                print("cmd: m --> More speed")
                mySocket.sendto(str.encode("More Speed"), (clientIP, CLIENT_PORT))
                command = "m"
                ser.write(str.encode(command))
            elif command == 'q':
                print("\nTerminating connection.")
                break
            elif command == 'x':
                while ser.in_waiting:
                    print(ser.readline().decode())
            elif command == 'p':
                print("cmd: p --> ping")
                command = "p"
                ser.write(str.encode(command))
                #response = ser.readline().decode()
                #print("response: " + str(response))
                #mySocket.sendto(str.encode(response), (clientIP, CLIENT_PORT))

                pingTime = currentMillis()
                try:
                    while currentMillis() - pingTime < PING_TIMEOUT:
                        response = ser.readline().decode()
                        mySocket.sendto(str.encode(response), (clientIP, CLIENT_PORT))
                        print("response: " + str(response))
                except:
                    print("benis")
                    continue

    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
