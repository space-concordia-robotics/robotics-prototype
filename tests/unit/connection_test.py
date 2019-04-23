#!/usr/bin/env python3
from robot.comms.connection import Connection
import threading
import time

def sender(ip, msg):
    c1 = Connection("c1", ip, 5005)
    c1.send(msg)

def receiver(ip, result):
    c2 = Connection("c2", ip, 5005)
    data = c2.receive()

    result[0] = data

    return data

def receiver_timeout(ip, result):
    c2 = Connection("c2", ip, 5005)
    data = c2.receive(timeout=3)

    result[0] = data

    return data

def test_send_receive_success():
    results = [None]

    receiver_thread = threading.Thread(target=receiver, args=("127.0.0.1", results,))
    # set expected message as second argument
    sender_thread = threading.Thread(target=sender, args=("127.0.0.1", "hello there"))

    receiver_thread.start()
    time.sleep(0.1)
    sender_thread.start()

    # wait until daemon threads have completed their work
    receiver_thread.join()
    sender_thread.join()

    print("results", results)

    assert "hello there" in results[0]

def test_send_receive_timeout_success():
    results = [None]

    receiver_thread = threading.Thread(target=receiver_timeout, args=("127.0.0.1", results,))
    receiver_thread.start()

    # wait to test for timeout functionality
    time.sleep(1)

    # set expected message as second argument
    sender_thread = threading.Thread(target=sender, args=("127.0.0.1", "hello there"))
    sender_thread.start()

    # wait until dameon threads have completed their work
    receiver_thread.join()
    sender_thread.join()

    print("results", results)

    assert "hello there" in results[0]

def test_send_receive_timeout_fail():
    results = [None]

    receiver_thread = threading.Thread(target=receiver_timeout, args=("127.0.0.1", results,))
    receiver_thread.start()

    # wait to test for timeout functionality
    time.sleep(1)

    # wait until dameon threads have completed their work
    receiver_thread.join()

    print("results", results)

    assert results[0] == ""
