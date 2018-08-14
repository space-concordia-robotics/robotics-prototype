#!/usr/bin/env python3

# will listen on any incoming messages on the same network
# and display them as they get received,
# as well as send corresponding commands over serial to a usb connected arduino

from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys, traceback
import serial
import serial.tools.list_ports

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
mySocket.bind((hostName, PORT_NUMBER))

# set up connection to arduino
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name
print("Connecting to port: " + firstPortName)
ser = serial.Serial('/dev/' + firstPortName, 9600)

print("Rover server listening on port {} \n".format(PORT_NUMBER))

while True:
    try:
        (data, addr) = mySocket.recvfrom(SIZE)
        command = data.decode()

        if command == 'w':
            print("Forward")
            ser.write(str.encode(command))
        elif command == 's':
            print("Back")
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
