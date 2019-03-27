#!/usr/bin/env python3
from connection import Connection

rover_ip = "127.0.0.1"

c2 = Connection("c2", rover_ip, 5005)
data = c2.receive()
