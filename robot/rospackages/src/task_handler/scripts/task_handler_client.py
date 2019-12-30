#!/usr/bin/env python3

import sys
import datetime
import glob
import rospy
from task_handler.srv import *

def task_handler_client(r_task, r_status, r_args=""):
    # convenience function that blocks until 'task_handler' is available
    #rospy.wait_for_service('task_handler')

    try:
        task_handle = rospy.ServiceProxy('task_handler', HandleTask)
        resp1 = task_handle(task, int(status), r_args)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: {:s}".format(str(e)))

def is_valid_request(r_task, r_status, r_args):
    """
    Validate client request parameters
    r_task: requested task
    r_status: status ON/OFF, represented by 1 and 0 respectively
    r_args: optional args
    """

    global is_local
    global ports
    global competition_ports
    global known_tasks

    r_task = str(r_task)
    r_status = int(r_status)

    # exception case of requesting available ports, should run regardless of comp or local mode
    if r_task == 'camera_ports' and r_status == 2:
        return True

    if is_local:
        ports = glob.glob('/dev/video[0-9]*')
    else:
        # competition mode
        ports = competition_ports

    if r_task in known_tasks and r_status in [0, 1, 2]:
        if r_args and r_task == "camera_stream" and not r_args in ports:
            return False
        return True

    return False

def usage():
    """
    Return string showcasing proper usage of this client script
    """
    global ports
    global competition_ports
    global known_tasks

    help_msg = "USAGE:\nrosrun task_handler task_handler_client.py [task] [status] optional:[args]"
    help_msg += "\nValid task options: ["
    help_msg += "'" + "', '".join(known_tasks) + "']"
    help_msg  += "\nValid status options: [0, 1, 2, 3]"

    if is_local:
        help_msg +="\nValid camera stream args: ["
        help_msg += "'" + "' '".join(ports) + "']"
    else:
        help_msg  += "\nValid camera stream args: ["
        help_msg += "'" + "' '".join(competition_ports) + "']"
        help_msg  += "\n\nTo see valid stream args in local mode run:\nrosrun task_handler task_handler_client.py camera_stream 1 /dev/video local"
        help_msg  += "\nOr just use: rosrun task_handler task_handler_client.py camera_ports 2"
    return help_msg

if __name__ == '__main__':
    # check for invalid ranges for args
    if len(sys.argv) < 2 or len(sys.argv) > 5:
        print(usage())
        sys.exit(1)

    is_local = False
    ports = []

    # all known tasks to be handled by the handler
    known_tasks = ['arm_listener', 'rover_listener', 'science_listener', 'camera_stream', 'camera_ports']
    # symlinks fixed to specific physical usb ports
    competition_ports = ['/dev/videoFront', '/dev/videoRear', '/dev/videoArmScience']

    if "local" in sys.argv:
        print("supported options:")
        print("- camera_stream")
        is_local = True

    rospy.init_node("task_handler_client")

    task = sys.argv[1]
    status = int(sys.argv[2])

    if len(sys.argv) >= 4:
        args = sys.argv[3]
    else:
        # if not enough args passed, then explicitly pass empty string for validation function to handle
        args = ""

    if not is_valid_request(task, status, args):
        print("Invalid format")
        print(usage())
        sys.exit(0)

    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    action = ""
    if status == 0:
        action = "Terminating"
    elif status == 1:
        action = "Running"
    elif status == 2:
        action = "Checking"
    print(action, str(task))
    print(sent_ts)
    print("---")

    print("Response: " + str(task_handler_client(task, status, args)))
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))
    print(received_st)

    diff = received - sent
    print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
