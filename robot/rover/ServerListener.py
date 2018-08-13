#!/usr/bin/env python3
# will listen on any incoming messages on the same network
# and display them as they get received
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys

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

print("Rover server listening on port {} \n".format(PORT_NUMBER))

while True:
	try:
	    (data, addr) = mySocket.recvfrom(SIZE)
	    command = data.decode()

	    if command == 'w':
	        print("Forward")
	    elif command == 's':
	        print("Back")
	    elif command == 'q':
	        print("\nTerminating connection.")
	        break
	except:
		break
