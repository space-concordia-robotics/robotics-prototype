#!/usr/bin/env python3

# make sure to run this script before ClientSender.py !
# This will listen on any incoming messages on the same network
# and display them as they get received,
# as well as send corresponding commands over serial to a usb connected arduino,
# along with acknowledgment message per drive command received

from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys, traceback
import serial
import serial.tools.list_ports
import time
import re

PORT_NUMBER = 5000
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
mySocket.bind((hostName, PORT_NUMBER + 1))
#TcpSocket = socket(AF_INET, SOCK_STREAM)

# set up connection to arduino
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName)
ser = serial.Serial('/dev/' + firstPortName, 9600)
clientIP = ""
IP_KNOWN = "ip_known"

print("Rover server listening on port {} \n".format(PORT_NUMBER))

try:
    (data, addr) = mySocket.recvfrom(SIZE)
    clientIpMsg = data.decode()

    # for debugging
    #print("clientIpMsg = " + clientIpMsg)
    clientIP = re.findall('\d+\.\d+\.\d+\.\d+', clientIpMsg)[0]

    if clientIP:
        print("Client IP received: " + str(clientIP))
        time.sleep(1)
        print("Sending acknowledgment")
        mySocket.sendto(str.encode(IP_KNOWN), (clientIP, PORT_NUMBER))
        print("Acknowledgement sent")
except:
    print("Exception in user code:")
    print("-"*60)
    traceback.print_exc(file=sys.stdout)
    print("-"*60)

print("Ready for incoming drive cmds!\n")

while True:
    try:
        (data, addr) = mySocket.recvfrom(SIZE)
        command = data.decode()

        if command == 'w':
            print("cmd: w --> action: Forward")
            mySocket.sendto(str.encode("Forward"), (clientIP, PORT_NUMBER))
            ser.write(str.encode(command))
        elif command == 's':
            print("cmd: s --> action: Back")
            mySocket.sendto(str.encode("Back"), (clientIP, PORT_NUMBER))
            ser.write(str.encode(command))
        elif command == 'q':
            print("\nTerminating connection.")
            break
    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)
        break
