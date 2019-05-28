#!/usr/bin/env python3

import rospy
import os
from Listener import Listener
from task_handler.srv import *
import glob

current_dir = os.path.dirname(os.path.realpath(__file__)) + "/"
arm_control_dir = current_dir+'/../../arm_control/scripts/'
scripts = [current_dir + "RoverCommandListener.py", arm_control_dir + "ArmNode.py", current_dir + "ScienceCommandListener.py", current_dir + "start_stream.sh"]
running_tasks = [Listener(scripts[0], "python3"), Listener(scripts[1], "python3"), Listener(scripts[2], "python3"), Listener(scripts[3], "bash", "", 1, True)]
known_tasks = ["rover_listener", "arm_listener", "science_listener", "camera_stream"]
known_listeners = known_tasks[:-1]

i = 0
for task in running_tasks:
    task.set_name(known_tasks[i])
    i += 1

# return the response string after calling the task handling function
def handle_task_request(req):
    response = "\n" + handle_task(req.task, req.status, req.args)

    return response

def handle_task(task, status, args):

    response = "Nothing happened"

    global running_tasks

    if status in [0, 1, 2] and task in known_tasks:
        # set index for corresponding listener object in array
        i = 0

        for t in known_tasks:
            if t.partition("_")[0] in task:
                chosen_task = t
                print('task chosen:', chosen_task)

                if status == 2:
                    response = chosen_task
                    is_running_str = " is running" if running_tasks[i].is_running() else " is not running"
                    return response + is_running_str + "\n"

                if chosen_task in known_listeners:
                    other_listeners = [x for x in known_listeners if x != chosen_task]

                    for listener in running_tasks:
                        if listener.get_name() in other_listeners and listener.is_running():
                            response = listener.get_name() + " is running, kill it before running " + chosen_task
                            return response + "\n"

                # reinitialize Listener object with proper arguments if necessary, or quit early if nonesense request
                if chosen_task == "camera_stream":
                    if running_tasks[i].is_running() and args != running_tasks[i].get_args():
                        response = "Camera stream already running on port " + running_tasks[i].get_args()
                        response += "\nTurn this stream of before starting or stopping a new one\n"
                        return response
                    # set appropriate usb port in args
                    elif args and not running_tasks[i].is_running():
                        ports = glob.glob('/dev/tty[A-Za-z]*')
                        if args in ports:
                            running_tasks[i] = Listener(scripts[i], "bash", args, 1, True)
                        else:
                            response = "Requested port not available, is the camera properly plugged into the USB port?"
                            return response + "\n"
                if chosen_task == "arm_listener":
                    if args and not running_tasks[i].is_running():
                        if args == 'usb' or args == 'uart':
                            running_tasks[i] = Listener(scripts[i], "python", args)
                        else:
                            response = "Requested serial type is invalid"
                            return response+'\n'
                break
            i += 1


        if status == 1:

            if running_tasks[i].start():
                response = "Started " + chosen_task
            else:
                response = "Failed to start " + chosen_task

                if running_tasks[i].is_running():
                    response += ", already running"
                else: # in this case it is worth trying to start the chosen_task
                    if running_tasks[i].start():
                        response = "Started " + chosen_task
        else:
            if len(running_tasks) >= 1 and isinstance(running_tasks[i], Listener):
                if running_tasks[i].stop():
                    response = "Stopped " + chosen_task
                else:
                    response = chosen_task + " not running, cannot terminate it"

    return response + "\n"

def task_handler_server():
    rospy.init_node('task_handler_server')
    s = rospy.Service('task_handler', HandleTask, handle_task_request)
    print("Ready to respond to task handling requests")
    rospy.spin()

if __name__ == "__main__":
    task_handler_server()
