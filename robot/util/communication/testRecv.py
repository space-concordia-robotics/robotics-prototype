#!/usr/bin/env python3
from connection import Connection

rover_ip = "192.168.129.137"

c2 = Connection("c2", rover_ip, 5005)
data = c2.receive(rover_ip, 5005)
