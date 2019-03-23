#!/usr/bin/env python3
from connection import Connection

c2 = Connection("c2", "127.0.0.1", 5005)
data = c2.receive("127.0.0.1", 5005)
