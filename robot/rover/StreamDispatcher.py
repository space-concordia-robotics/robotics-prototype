#!/usr/bin/env python

import signal
import subprocess
import os
import sys
import time

# for now methods that aren't fully implemented return False
class StreamDispatcher:
    stream_starter = "./start_stream.sh"

    def __init__(self):
        #self.current_fps = 25  # set default value
        #self.fps_cap = 30  # this should be a upper limit constant based of experimentation/known physical limits
        #self.status = False  # ON == true, OFF == false
        #self.brightness = 10  # decide on default value via experimentation
        #self.color = False  # turn off color by default
        self.p1_pid = -1
        self.p1 = 0

    def start_stream(self):
        if os.path.isfile(self.stream_starter):
            print("Starting camera stream...")
            # non-blocking
            self.p1 = subprocess.Popen(["sh", self.stream_starter], stdout=subprocess.PIPE)
            self.p1_pid = self.p1.pid
            print("process id: " + str(self.p1_pid))
        else:
            print("Failed to start camera stream")

    def stop_stream(self):
        if self.p1_pid > -1:
            print("Terminating stream...")
            self.p1.send_signal(signal.SIGINT)
            print("Stream terminated")
            self.steam_pid = -1
            return True
        else:
            print("Failed to close video stream")
            return False

if __name__ == "__main__":

    try:
        dispatcher = StreamDispatcher()
        dispatcher.start_stream()

        while True:
            continue

    except KeyboardInterrupt:
        dispatcher.stop_stream()
        sys.exit(0)

    # other attemps

    # register Ctrl + c to call back function stop_stream
    #signal.signal(signal.SIGINT, dispatcher.stop_stream)

    # using signal over atexit to avoid empty while True
    #atexit.register(dispatcher.stop_stream())

    # wait for user to press ctrl + c
    #signal.pause()
