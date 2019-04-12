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
        if self.p1_pid > -1:
            print("Listener already running")
            return False

        if os.path.isfile(self.script):
            print("Starting listener service...")
            # non-blocking
            self.p1 = subprocess.Popen([self.type, self.script], stdout=subprocess.PIPE)
            self.p1_pid = self.p1.pid

            # allow some time to pass
            time.sleep(1)

            poll = self.p1.poll()

            #print("self.p1_pid: " + str(self.p1_pid))

            # None value indicates that the process hasn't terminated yet
            if poll != None:
                print("Something went wrong, process defunct (already terminated)")
                self.p1_pid = -1

                return False
            else:

                return True
        else:
            print("Failed to start process, script does not exist")

            return False

    def stop(self):
        if self.is_running():
            print("Terminating process...")
            self.p1.send_signal(signal.SIGINT)
            print("Process terminated")
            self.p1_pid = -1
            return True

        print("Failed to terminate process")
        return False

    def is_running(self):
        if self.p1_pid >= 0:
            return True

        return False

# quick test for verification
if __name__ == "__main__":

    try:
        dispatcher = Listener("/home/odroid/Programming/robotics-prototype/robot/rover/ArmCommandListener.py", "python3")
        dispatcher.start()

        print("Is running: " + str(dispatcher.is_running()))

        while True:
            continue

    except KeyboardInterrupt:
        dispatcher.stop()
        sys.exit(0)
