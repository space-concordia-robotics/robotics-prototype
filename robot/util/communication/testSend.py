#!/usr/bin/env python
from connection import Connection

rover_ip = "192.168.129.139"

c1 = Connection("c1", rover_ip, 5005)
c1.send(rover_ip, 5005)
