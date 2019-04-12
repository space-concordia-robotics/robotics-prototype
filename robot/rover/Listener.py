#!/usr/bin/env python3

import signal
import subprocess
import os
import sys
import time

class Listener:
    def __init__(self, script, type):
        # initialize pid to -1 to imply process hasn't yet started
        self.p1_pid = -1
        # the actual process object itself
        self.p1 = 0
        # location of the script to run
        self.script = script
        # which language to run the script with
        self.type = type

    def start(self):
        if os.path.isfile(self.script):
            print("Starting listener service...")
            # non-blocking
            self.p1 = subprocess.Popen([self.type, self.script], stdout=subprocess.PIPE)
            self.p1_pid = self.p1.pid
            print("process id: " + str(self.p1_pid))

            time.sleep(1)

            poll = self.p1.poll()

            # None value indicates that the process hasn't terminated yet
            if poll != None:
                print("Something went wrong, process defunct (already terminated)")

        else:
            print("Failed to start process, script does not exist")

    def stop(self):
        if self.p1_pid > -1:
            print("Terminating process...")
            self.p1.send_signal(signal.SIGINT)
            print("Process terminated")
            self.p1_pid = -1
            return True
        else:
            print("Failed to terminate process")
            return False

# quick test for verification
if __name__ == "__main__":

    try:
        dispatcher = Listener("/home/odroid/Programming/robotics-prototype/robot/rover/RoverCommandListener.py", "python3")
        dispatcher.start()

        while True:
            continue

    except KeyboardInterrupt:
        dispatcher.stop()
        sys.exit(0)
