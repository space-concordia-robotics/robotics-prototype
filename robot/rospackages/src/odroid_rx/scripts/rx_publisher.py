#!/usr/bin/env python3

# NOTE: the str(output, "utf-8") only works if we run this with python2

import signal
import atexit
import sys
import os
import subprocess
import rospy
from std_msgs.msg import String

def rx_publisher():
    log_folder = "/home/odroid"
    log_file = log_folder + "/received.txt"

    # create log file if it does not exist
    if not os.path.isfile(log_file):
        print("Please create a file for logging: /home/odroid/received.txt")
        sys.exit(1)

    # publish to topic "odroid_rx"
    pub = rospy.Publisher('odroid_rx', String, queue_size=10)
    rospy.init_node('rx_publisher', anonymous=True)
    rate = rospy.Rate(4) # 4hz

    p1 = start_logger()

    # cleanup tasks will run in reverse order
    atexit.register(stop_logger, p=p1)

    old_data = ""

    while not rospy.is_shutdown():
        data = get_rx()
        #print("data: " + data)

        #print("data != old_data --> " + str(data != old_data))

        # only publish if data changed
        #@TODO: replace with "if the logfile is modified"
        if data != old_data:
            pub.publish(data)
            rospy.loginfo(data)
            old_data = data
            #print("old_data: " + old_data)

        rate.sleep()

def get_rx():
    p = subprocess.Popen(["cat", "/home/odroid/received.txt"], stdout=subprocess.PIPE)
    output, error = p.communicate()
    output = str(output, "utf-8")

    #print("output: " + output)

    return output

def start_logger():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    logger_starter = script_dir + "/rx_logger.sh"

    if os.path.isfile(logger_starter):
        # initialize process id for rx_logger
        p1_pid = -1
        # initialize variable for process
        p1 = 0

        print("Starting odroid rx data logger...")
        p1 = subprocess.Popen(["sh", logger_starter], stdout=subprocess.PIPE)
        p1_pid = p1.pid
        print("process id: " + str(p1_pid))

        return p1
    else:
        print("Failed to start odroid rx data logger")
        print("Cannot locate " + str(logger_starter))
        sys.exit(1)

def stop_logger(p):
    if p.pid > -1:
        print("Terminating logger...")
        p.send_signal(signal.SIGINT)
        print("Logger terminated")

        return True
    else:
        print("Failed to close logger")
        return False

if __name__ == '__main__':
    try:
        rx_publisher()
    except rospy.ROSInterruptException:
        pass
