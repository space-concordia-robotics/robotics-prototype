#!/usr/bin/env python

import rospy
from RoverListener import RoverListener
from task_handler.srv import *

running_tasks = [RoverListener()]

def handle_task_request(req):
    response = "\n" + handle_task(req.task, req.status)

    return response

def handle_task(task, status):

    response = "Nothing happened"

    global running_tasks

    known_tasks = ["rover_listener"]

    if task in known_tasks and status in [0, 1]:
        if task == "rover_listener":
            if status == 1:
                if running_tasks[0].start_listener():
                    response = "Started " + task
                else:
                    response = "Failed to start " + task
            else:
                if len(running_tasks) >= 1 and isinstance(running_tasks[0], RoverListener):
                    if running_tasks[0].stop_listener():
                        response = "Stopped " + task
                    else:
                        response = task + " not running, cannot terminate it"

    return response + "\n"

def task_handler_server():
    rospy.init_node('task_handler_server')
    s = rospy.Service('task_handler', HandleTask, handle_task_request)
    print("Ready to respond to task handling requests")
    rospy.spin()

if __name__ == "__main__":
    task_handler_server()
