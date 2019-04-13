#!/usr/bin/env python3

import sys
import datetime
import rospy
from task_handler.srv import *

def task_handler_client(r_task, r_status):
    # convenience function that blocks until 'task_handler' is available
    rospy.wait_for_service('task_handler')

    try:
        task_handle = rospy.ServiceProxy('task_handler', HandleTask)
        resp1 = task_handle(task, int(status))
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: {:s}".format(str(e)))

def is_valid_request(r_task, r_status):
    """
    Validate client request parameters
    r_task: requested task
    r_status: status ON/OFF, represented by 1 and 0 respectively
    """
    r_task = str(r_task)
    r_status = int(r_status)

    # all known tasks to be handled by the handler
    known_tasks = ["rover_listener", "arm_listener"]

    if r_task in known_tasks and r_status in [0, 1]:
        return True

    return False

def usage():
    """
    Return string showcasing proper usage of this client script
    """
    help_msg = """USAGE:\nrosrun task_handler task_handler_client.py [task] [status]
                  \nValid task options: ['rover_listener', 'arm_listener']
                  \nValid status options: [0, 1]"""
    return help_msg

if __name__ == "__main__":
    if len(sys.argv) < 3 or len(sys.argv) > 3:
        print(usage())
        sys.exit(1)

    rospy.init_node("task_handler_client")

    task = sys.argv[1]
    status = sys.argv[2]

    if not is_valid_request(task, status):
        print("Invalid format")
        print(usage())
        sys.exit(0)

    sent = datetime.datetime.now()
    sent_ts = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (sent.microsecond / 10000))
    print("Running" if status else "Terminating", str(task))
    print(sent_ts)
    print("---")

    print("Response: " + str(task_handler_client(task, status)))
    received = datetime.datetime.now()
    received_st = sent.strftime('%Y-%m-%dT%H:%M:%S') + ('-%02d' % (received.microsecond / 10000))
    print(received_st)

    diff = received - sent
    print("Latency: " + str(diff.total_seconds() * 1000) + " ms")
