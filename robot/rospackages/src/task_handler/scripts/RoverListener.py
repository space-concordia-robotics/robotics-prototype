#!/usr/bin/env python

import signal
import subprocess
import os
import sys
import time

class RoverListener:
    #command_listener = "./RoverCommandListener.py"
    command_listener = "/home/beep/Programming/robotics-prototype/robot/rospackages/src/task_handler/scripts/RoverCommandListener.py"
    #command_listener = "/usr/bin/RoverCommandListener.py"

    def __init__(self):
        self.p1_pid = -1
        self.p1 = 0

    def start_listener(self):
        if os.path.isfile(self.command_listener):
            print("Starting rover listener service...")
            # non-blocking
            self.p1 = subprocess.Popen(["python3", self.command_listener], stdout=subprocess.PIPE)
            self.p1_pid = self.p1.pid
            print("process id: " + str(self.p1_pid))
        else:
            print("Failed to start rover listener")

    def stop_listener(self):
        if self.p1_pid > -1:
            print("Terminating stream...")
            self.p1.send_signal(signal.SIGINT)
            print("Stream terminated")
            self.steam_pid = -1
            return True
        else:
            print("Failed to close rover listener")
            return False

if __name__ == "__main__":

    try:
        dispatcher = RoverListener()
        dispatcher.start_listener()

        while True:
            continue

    except KeyboardInterrupt:
        dispatcher.stop_listener()
        sys.exit(0)
