# will listen on any incoming messages on the same network
# and display them as they get received
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys

PORT_NUMBER = 5000

if len(sys.argv) == 2:
	PORT_NUMBER = int(sys.argv[1])
elif len(sys.argv) >= 3:
	print "too many arguments, one optional argument is the port number, otherwise default to 5000"
	print "example usage: python ServerListener.py <port>"

SIZE = 1024

hostName = gethostbyname( '0.0.0.0' )

mySocket = socket( AF_INET, SOCK_DGRAM )
mySocket.bind( (hostName, PORT_NUMBER) )

print ("Test server listening on port %s \n" % (PORT_NUMBER))

while True:
        (data,addr) = mySocket.recvfrom(SIZE)
        print data
sys.exit()
