#!/usr/bin/env python3

import signal
import subprocess
import os
import sys
import time
from robot.basestation.app import run_shell, get_pid

class Listener:
    def __init__(self, script, type, args='', force_kill=False, children=0):
        # initialize pid to -1 to imply process hasn't yet started
        self.p1_pid = -1
        # the actual process object itself
        self.p1 = 0
        # location of the script to run
        self.script = script
        # which language to run the script with
        self.type = type
        # when SIGINT is not enough to kill process
        self.force_kill = force_kill
        # amount of children processes spawned
        self.children = children
        # arguments for scripts
        self.args = args
        self.name = "none"

    def start(self):
        if self.is_running():
            print("Listener already running")
            return False

        if os.path.isfile(self.script):
            print("Attempting to start listener service...")
            # non-blocking
            if self.args:
                self.p1 = subprocess.Popen([self.type, self.script, self.args], stdout=subprocess.PIPE)
            else:
                self.p1 = subprocess.Popen([self.type, self.script], stdout=subprocess.PIPE)

            self.p1_pid = self.p1.pid

            # allow some time to pass, listener scripts need more time
            if 'Node' in self.script:
                time.sleep(2)
            else:
                time.sleep(0.5)

            poll = self.p1.poll()

            #print("self.p1_pid: " + str(self.p1_pid))

            # None value indicates that the process hasn't terminated yet
            if poll != None:
                print("Something went wrong, process defunct (already terminated)")
                self.p1_pid = -1

                return False
            else:
                print("Assigned process id: " + str(self.p1_pid))
                return True
        else:
            print("Failed to start process, script does not exist")

            return False

    def stop(self):
        if self.is_running():
            print("Terminating process...")

            if self.force_kill:
                # edge case due to current implementation of video streamer
                if "start_ros_stream" in self.script:
                    real_pid = get_pid("cv_camera_node")

                    if real_pid != -1:
                        output, error = run_shell("kill -9", str(real_pid))
                        print("kill -9", str(real_pid))
                    else:
                        return False
            else:
                self.p1.kill()


            print("Process terminated")
            self.p1_pid = -1

            return True

        print("Failed to terminate process")
        return False

    def is_running(self):
        if self.p1_pid >= 0:
            poll = self.p1.poll()

            if poll != None:
                print("Something went wrong, process defunct (already terminated)")
                self.p1_pid = -1
                return False
        else:
            return False

        return True


    def get_args(self):
        return self.args

    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

# quick test for verification
if __name__ == "__main__":

    try:
        current_dir = os.path.dirname(os.path.realpath(__file__)) + "/"
        dispatcher = Listener(current_dir + "start_stream.sh", "bash")
        #dispatcher = Listener(current_dir + "start_stream.sh", "bash", "/dev/video0")
        dispatcher.start()

        print("Is running: " + str(dispatcher.is_running()))

        while True:
            continue

    except KeyboardInterrupt:
        dispatcher.stop()
        sys.exit(0)
