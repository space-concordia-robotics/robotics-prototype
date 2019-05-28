#!/usr/bin/env python3
from robot.comms.connection import Connection
from time import sleep

# This code is meant to run first on the odroid
receiver = Connection("receiver", "172.16.1.30", 5005)
sender = Connection("sender", "172.16.1.20", 5005)

feedback = receiver.receive()

print("GOT:", feedback)

sleep(1)

msg = "was gucci"

sender.send(msg)

print("SENT:", msg)
