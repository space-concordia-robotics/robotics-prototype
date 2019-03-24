#!/usr/bin/env python
from connection import Connection

rover_ip = "127.0.0.1"

c1 = Connection("c1", rover_ip, 5005)
c1.send("oh hello")
