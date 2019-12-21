#!/usr/bin/env python3

import rospy
import os
import time
import sys
from robot.rospackages.src.task_handler.scripts.Listener import Listener
from task_handler.srv import *
import glob

# return the response string after calling the task handling function
def handle_task_request(req):
    response = "\n" + handle_task(req.task, req.status, req.args)

    return response

# process task, status and corresponding args sent from client
# return response message
def handle_task(task, status, args):
    response = "Nothing happened"

    global running_tasks
    global stream_ctr
    global local

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
                    args_str = ""

                    if running_tasks[i].is_running() and running_tasks[i].get_args():
                        args_str = " with args: " + running_tasks[i].get_args()

                    return response + is_running_str + args_str + "\n"

                if chosen_task in known_listeners:
                    other_listeners = [x for x in known_listeners if x != chosen_task]

                    for listener in running_tasks:
                        if listener.get_name() in other_listeners and listener.is_running():
                            response = listener.get_name() + " is running, kill it before running " + chosen_task
                            return response + "\n"

                # reinitialize Listener object with proper arguments if necessary, or quit early if nonesense request
                if chosen_task == "camera_stream":
                    # else if running local mode
                    if local:
                        ports = glob.glob('/dev/video[0-9]*')
                    else:
                        # if in compeition mode/running on OBC
                        ports = glob.glob('/dev/tty[A-Za-z]*')

                    max_streams = len(ports)
                    print('args:', args)
                    print('ports:', ports)
                    print('stream_ctr:', stream_ctr)
                    print('max_streams:', max_streams)
                    print('status:', status)

                    if args in ports and stream_ctr < max_streams or status == 0:
                        pass
                    else:
                        response = "Requested port not available, is the camera properly plugged into the USB port?"
                        return response + "\n"
                elif chosen_task in known_listeners:
                    if args and not running_tasks[i].is_running():
                        if args == 'usb' or args == 'uart':
                            running_tasks[i] = Listener(scripts[i], "python", args)
                        else:
                            response = "Requested serial type is invalid"
                            return response+'\n'
                break
            i += 1


        # process start request
        if status == 1:
            if task == "camera_stream":
                # check if already started, if not then start it
                if args in active_ports:
                    response = "Failed to start " + chosen_task
                else:
                    stream = Listener(scripts[4], "bash", args, 1, True)
                    if stream.start():
                        # wait a second just in case it fails to start
                        time.sleep(1)

                        if stream.is_running():
                            response = "Started " + chosen_task + " on port " + args
                            active_ports.append(args)
                            stream.set_name(args)
                            active_streams.append(stream)
                            stream_ctr += 1
                        else:
                            response = "Failed to start " + chosen_task + " on port " + args + ", process defunct"
                    else:
                        response = "Failed to start " + chosen_task
            else:
                if running_tasks[i].start():
                    response = "Started " + chosen_task
                else:
                    response = "Failed to start " + chosen_task

                    if running_tasks[i].is_running():
                        response += ", already running"
                    else: # in this case it is worth trying to start the chosen_task
                        if running_tasks[i].start():
                            response = "Started " + chosen_task

        # process stop request
        elif status == 0:
            if task == "camera_stream":
                # temp copy so no weird stuff happens in following for loop
                active_streams_copy = active_streams.copy()
                print("Attempting to terminate camera_stream")
                print("active_ports: ", active_ports)
                print("active_streams: ", active_streams)

                for s in active_streams:
                    print(s.get_name())

                print("stream_ctr: ", stream_ctr)

                if args in active_ports:
                    for stream in active_streams_copy:
                        if stream.get_name() == args:
                            print('stream.get_name()', stream.get_name())
                            print('args', args)
                            node_name = args[args.rindex('/')+1:]

                            if stream.stop("__name:=" + node_name):
                                time.sleep(1)
                                if not stream.is_running():
                                    response = "Stopped " + chosen_task + " on port: " + args
                                    stream_ctr -= 1
                                    active_ports.remove(args)
                                    active_streams.remove(stream)
                                else:
                                    response = "Failed to stop " + chosen_task + " on port: " + args
                            else:
                                response = "Failed to stop " + chosen_task + " on port: " + args
                        else:
                            print("Not: " + stream.get_name())
                else:
                    response = "No active stream found on port: " + args + ", nothing to terminate"
            elif len(running_tasks) >= 1 and isinstance(running_tasks[i], Listener):
                if running_tasks[i].stop():
                    response = "Stopped " + chosen_task
                else:
                    response = chosen_task + " not running, cannot terminate it"

    return response + "\n"

def task_handler_server():
    global local
    rospy.init_node('task_handler_server')
    s = rospy.Service('task_handler', HandleTask, handle_task_request)
    ready_msg = "Ready to respond to task handling requests"

    if local:
        ready_msg += " (local mode)"

    print(ready_msg)
    rospy.spin()

if __name__ == "__main__":
    local = False

    if "local" in sys.argv:
        print("Task handler local mode supported tasks")
        print("--> camera_stream")
        local = True

    # this value is set at runtime based off how many video port devices are identified
    max_streams = 0
    # how many streams are currently on
    stream_ctr = 0

    current_dir = os.path.dirname(os.path.realpath(__file__)) + "/"
    mcu_control_dir = current_dir + '../../mcu_control/scripts/'

    # tasks that can be run
    scripts = [mcu_control_dir + "RoverNode.py", mcu_control_dir + "ArmNode.py", mcu_control_dir + "ScienceNode.py", mcu_control_dir + "PdsNode.py", current_dir + "start_ros_stream.sh"]

    # set up listener objects
    running_tasks = [Listener(scripts[0], "python3"), Listener(scripts[1], "python3"), Listener(scripts[2], "python3"), Listener(scripts[3], "python3"), Listener(scripts[4], "bash", "", 1, True)]

    # expected client arguments for choosing task
    known_tasks = ["rover_listener", "arm_listener", "science_listener", "pds_listener", "camera_stream"]
    known_listeners = known_tasks[:-1]

    # keep track of currently running streams
    active_ports = []
    active_streams = []

    for i in range(len(running_tasks)):
        running_tasks[i].set_name(known_tasks[i])

    task_handler_server()
