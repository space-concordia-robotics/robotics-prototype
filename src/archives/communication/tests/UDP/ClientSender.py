# given the ip address of the destination (and optionally port number, otherwise default to 5000)
# send some kind of message over to the receiver (running ServerReceiver.py)

import sys
from socket import socket, AF_INET, SOCK_DGRAM
from time import sleep

PORT_NUMBER = 5000
SIZE        = 1024

if len(sys.argv) < 2: # first one is the name of the file
    print "please pass a destination ip address/port of the listener to send this message to, port is optional and will default to 5000"
    print "example usage: python ClientSender.py 138.203.97.167 5000"
    sys.exit(1)

if len(sys.argv) == 3:
    PORT_NUMBER = int(sys.argv[2])
elif len(sys.argv) > 3:
    print "too many args"
    sys.exit(1)

SERVER_IP = sys.argv[1]

print ("Test client sending packets to IP %s, via port %s\n" % (SERVER_IP, PORT_NUMBER))

mySocket = socket( AF_INET, SOCK_DGRAM )
inner_space   = ""
left_padding  = "         "
inner_incr    = False
movement      = 1

i = 0
while True:
    if i == 0:
        inner_incr = True
    elif i == 10:
        inner_incr = False

    if inner_incr:
        i += movement
        inner_space += " "*movement
    else:
        i -= movement
        inner_space = inner_space[:len(inner_space) - movement]

    new_line = left_padding + inner_space + "|+|" + left_padding + "|+|"
    print new_line
    sleep(0.2)
    mySocket.sendto(new_line, (SERVER_IP,PORT_NUMBER))

