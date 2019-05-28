#!/usr/bin/env python3
from robot.comms.connection import Connection

sender = Connection("sender", "172.16.1.30", 5005)
receiver = Connection("sender", "172.16.1.20", 5005)

msg = "suh dude"

sender.send(msg)

print("SENT:", msg)

feedback = receiver.receive()

print("GOT:", feedback)
