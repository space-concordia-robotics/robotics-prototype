#!/usr/bin/env python3

import rospy
from Listener import Listener
from task_handler.srv import *

# for local testing use "./RoverCommandListenr.py"
#scripts = ["./RoverCommandListener.py", "/./ArmCommandListener.py"]
scripts = ["/usr/bin/RoverCommandListener.py", "/usr/bin/ArmCommandListener.py"]
running_tasks = [Listener(scripts[0], "python3"), Listener(scripts[1], "python3")]
known_tasks = ["rover_listener", "arm_listener"]

def handle_task_request(req):
    response = "\n" + handle_task(req.task, req.status)

    return response

def handle_task(task, status):

    response = "Nothing happened"

    global running_tasks

    if task in known_tasks and status in [0, 1]:
        if task in known_tasks:
            # set index for corresponding listener object in array
            i = 0 if "rover" in task else 1

            if status == 1:

                if running_tasks[i].start():
                    response = "Started " + task
                else:
                    response = "Failed to start " + task

                    if running_tasks[i].is_running():
                        response += ", already running"
                    else: # in this case it is worth trying to start the task
                        if running_tasks[i].start():
                            response = "Started " + task
            else:
                if len(running_tasks) >= 1 and isinstance(running_tasks[i], Listener):
                    if running_tasks[i].stop():
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
