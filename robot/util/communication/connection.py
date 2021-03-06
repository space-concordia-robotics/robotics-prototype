#!/usr/bin/env python3
import socket
class Connection:
    __active_ctr = 0

    def __init__(self, name, ip, port):
        type(self).__active_ctr += 1
        self.name = name
        self.ip = ip
        self.port = port

    def __del__(self):
        type(self).__active_ctr -= 1

    def get_total_active(self):
        return type(self).__active_ctr

    def send(self, target_ip, target_port):
        msg = self.name
        print("UDP target IP:", target_ip)
        print("UDP target port:", target_port)
        print("message:", msg)

        # Internet --> AF_INET, UDP --> SOCK_DGRAM
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(str.encode(msg), (target_ip, target_port))

    def receive(self, source_ip, source_port):
        # Internet --> AF_INET, UDP --> SOCK_DGRAM
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((source_ip, source_port))

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
