#!/usr/bin/env python3
"""
The intention of this class is to provide an easy way to send/receive messages
accross a network using objects.

Testing locally or on multiple machines will affect the way the objects should be created.

------------------------------------------------------------------------------------------
LOCAL
------------------------------------------------------------------------------------------
For local testing you may create corresponding pairs of receiver/sender connection objects,
making sure to either specify the localhost ("127.0.0.1") ip address for both objects
or using the machines ip address for both. If you are simply using one script to send data
and another to receive it, then you may assign the same port number to both objects.
If you want to have bi-directional communication, you must create a pair of receiver/sender
connection objects for each script that uses them, and make sure that you choose different
ports for the communicating pairs.

For an example of one directional communication, see testSend.py/testRecv.py.
testRecv.py is meant to be run before testSend.py.

Bi-directional configuration:

--> server side
[...]
sender = Connection("sender", "127.0.0.1", 5005)
receiver = Connection("receiver", "127.0.0.1", 5010)
[...]

--> client side
receiver = Connection("receiver", "127.0.0.1", 5005)
sender = Connection("sender", "127.0.0.1", 5010)
------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------
REMOTE (MULTIPLE MACHINES)
------------------------------------------------------------------------------------------
For remote testing it is the same idea as for local testing except that now when creating
sender/receiver object pairs across different scripts on different computers on the same
network you _must_ specify the ip address of the receiving machine for a given pair.

As an example for bi-directional communication see testRemoteSendAndRecv.py/testRemoteRecvAndSend.py
testRemoteRecvAndSend.py is meant to be run before testRemoteSendAndRecv.py.

As you can see, the same port can be used for all objects in this configuration.
"""
import socket
import select

class Connection:
    __active_ctr = 0

    # during intialization, pass only the rover IP and common communication port
    def __init__(self, name, ip, port):
        type(self).__active_ctr += 1
        self.name = name
        self.ip = ip
        self.port = port
        print("total active:", type(self).__active_ctr)

    def __del__(self):
        type(self).__active_ctr -= 1

    def get_total_active(self):
        return type(self).__active_ctr

    def send(self, msg):
        print("UDP target IP:", self.ip)
        print("UDP target port:", self.port)
        print("message:", msg)

        # Internet --> AF_INET, UDP --> SOCK_DGRAM
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(str.encode(msg), (self.ip, self.port))

    def receive(self, timeout=0):
        # Internet --> AF_INET, UDP --> SOCK_DGRAM
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("binding to")
        print("self.ip: ", self.ip)
        print("self.port: ", self.port)
        sock.bind((self.ip, self.port))

        # wait first until data available
        if timeout > 0:
            sock.setblocking(0)

            ready = select.select([sock], [], [], int(timeout))

            if ready[0]:
                data = sock.recv(1024)
                print("received message:", data.decode())

                return data.decode()
            else:
                print("Timeout limit exceeded, no data received")

                return ""

        else: # wait indefinitely
            while True:
                data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
                print("received message:", data.decode())

                return data.decode()

"""
c1 = Connection("c1", "127.0.0.1", 5005)
c2 = Connection("c2", "127.0.0.1", 5005)

c1.send("127.0.0.1", 5005)
c2.send("127.0.0.1", 5005)
c1.receive("127.0.0.1", 5005)
"""
