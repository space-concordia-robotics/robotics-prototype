#!/usr/bin/env python3
from connection import Connection

c1 = Connection("c1", "127.0.0.1", 5005)
c1.send("127.0.0.1", 5005)
