#!/usr/bin/env python3
from connection import Connection
import threading


def sender(ip):
    c1 = Connection("c1", ip, 5005)
    c1.send("hello there")

def receiver(ip):
    c2 = Connection("c2", ip, 5005)
    data = c2.receive()

    return data

def receiver_timeout(ip):
    c2 = Connection("c2", ip, 5005)
    data = c2.receive(timeout=3)

    return data

receiver_thread = threading.Thread(target=receiver, args=("127.0.0.1",))
receiver_timeout_thread = threading.Thread(target=receiver_timeout, args=("127.0.0.1",))
#sender_thread = threading.Thread(target=sender, args=("127.0.0.1",))

receiver_thread.start()
sender_thread.start()
#receiver_timeout_thread.start()
