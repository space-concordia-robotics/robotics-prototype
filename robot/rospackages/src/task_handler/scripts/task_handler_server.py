#!/usr/bin/env python3

import rospy
import os
import time
import sys
import glob
from robot.rospackages.src.task_handler.scripts.Listener import Listener
from task_handler.srv import *

# return the response string after calling the task handling function
def handle_task_request(req):
    response = "\n" + handle_task(req.task, req.status, req.args)

    return response

# given a regex, return all matching existing port names
def get_ports(pattern):
    ports = glob.glob(pattern)
    print('ports', ports)
    return ports

# return all available usb camera port names
# pitfall: on odroid (and perhaps other OBCs) if we would use the local mode regex
# it will return extra unusable video ports, on regular computers the same issue DNE
def get_available_camera_ports():
    global is_local

    if is_local:
        print('local port check')
        video_ports = get_ports('/dev/video[0-9]')
    else:
        print('comp port check')
        # this regex makes sure to get our symlinks for camera ports
        video_ports = get_ports('/dev/video[A-Z][a-z]*')

    return video_ports

# validate camera stream starting/stopping task requests
def validate_camera_task(args, status, active_stream_ctr, is_local=False):
    '''
        Check if the camera_stream request is valid and worth processing
        return true if the request is valid
    '''

    # if running local mode
    if is_local:
        ports = get_ports('/dev/video[0-9]*')
    else:
        # if in competition mode or running on OBC
        ports = get_ports('/dev/video[A-Za-z]*')

    max_streams = len(ports)

    print('args:', args)
    print('ports:', ports)
    print('active_stream_ctr:', active_stream_ctr)
    print('max_streams:', max_streams)
    print('status:', status)

    if args in ports and active_stream_ctr < max_streams or status == 0:
        return True
    else:
        response = "Requested port not available, is the camera properly plugged into the USB port?"
        return False


def start_camera_stream(port, active_ports, active_stream_ctr):
    """
        Attempt to start a camera stream
        Returns an empty string if no issues encountered, otherwise an error message is returned
    """
    FAIL_MSG = 'Failed to start camera stream on port ' + port
    response = ''

    # check if already started, if not then start it
    if port in active_ports:
        response = FAIL_MSG + ', already active port selected'
    else:
        stream = Listener(scripts[4], 'bash', port, 1, True)
        
        if stream.start():
            # wait a second just in case it fails to start
            time.sleep(1)

            if stream.is_running():
                active_ports.append(port)
                stream.set_name(port)
                active_streams.append(stream)
                active_stream_ctr += 1
                return ''
            else:
                response = FAIL_MSG + ', process defunct'
        else:
            response = FAIL_MSG
    
    return response

def stop_camera_task(port, active_ports, active_stream_ctr):
    '''
    Attempt to stop a camera stream
    Returns an empty string if no issues are encountered, otherwise an error message is returned
    '''
    response = ''
    FAIL_MSG = 'Failed to stop camera stream on port ' + port

    # temp copy so no weird stuff happens in following for loop
    active_streams_copy = active_streams.copy()
    print('Attempting to terminate camera_stream')
    print('active_ports: ', active_ports)
    print('active_streams: ', active_streams)

    for s in active_streams:
        print(s.get_name())

    print('active_stream_ctr: ', active_stream_ctr)

    if port in active_ports:
        for stream in active_streams_copy:
            if stream.get_name() == port:
                print('stream.get_name()', stream.get_name())
                print('port', port)
                # get everything after the '/dev/' prefix
                node_name = port[port.rindex('/')+1:]

                # keyword to filter for process to kill via ps aux | grep <pattern>
                process_identifier = '__name:=' + node_name

                if stream.stop(process_identifier):
                    time.sleep(1)

                    if not stream.is_running():
                        active_stream_ctr -= 1
                        active_ports.remove(port)
                        active_streams.remove(stream)
                    else:
                        response = 'Failed to stop camera stream on port: ' + port
                else:
                    response = 'Failed to stop camera stream on port: ' + port
    else:
        response = "No active stream found on port: " + port + ', nothing to terminate'

    return response

# process task, status and corresponding args sent from client
# return response message
def handle_task(task, status, args):
    response = 'Nothing happened'

    print('task:', task)
    print('status:', status)
    print('args:', args)

    if task == 'camera_ports' and status == 2:
        return ','.join(get_available_camera_ports()) + '\n'

    global running_tasks
    global active_stream_ctr
    global is_local
    global active_streams

    if status in [0, 1, 2] and task in known_tasks:
        # set index for corresponding listener object in array
        i = 0

        for t in known_tasks:
            if t.partition("_")[0] in task:
                chosen_task = t
                print('task chosen:', chosen_task)

                # check if aleady running task or check for available ports
                if status == 2:
                    if task == 'camera_stream':
                        if len(active_streams) == 0:
                            return ''
                        else:
                            active_stream_names = [s.get_name() for s in active_streams]
                            return ','.join(active_stream_names)
                    else:
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
                    if not validate_camera_task(args, status, active_stream_ctr, is_local):
                        response = "Requested port not available, is the camera properly plugged into the USB port?"
                        return response + '\n'
                elif chosen_task in known_listeners:
                    if args and not running_tasks[i].is_running():
                        if args == 'usb' or args == 'uart':
                            running_tasks[i] = Listener(scripts[i], "python", args)
                        else:
                            response = "Requested serial type is invalid"
                            return response + '\n'
                break
            i += 1

        # process start request
        if status == 1:
            if task == 'camera_stream':
                response_tmp = start_camera_stream(args, active_ports, active_stream_ctr)
                response = response_tmp if response_tmp != '' else 'Started camera stream on port ' + args
            else:
                if running_tasks[i].start():
                    response = 'Started ' + chosen_task
                else:
                    response = 'Failed to start ' + chosen_task

                    if running_tasks[i].is_running():
                        response += ', already running'
                    else: # in this case it is worth trying to start the chosen_task
                        if running_tasks[i].start():
                            response = 'Started ' + chosen_task

        # process stop request
        elif status == 0:
            if task == 'camera_stream':
                response_tmp = stop_camera_task(args, active_ports, active_stream_ctr)
                response = response_tmp if response_tmp != '' else 'Stopped camera stream on port ' + args
            elif len(running_tasks) >= 1 and isinstance(running_tasks[i], Listener):
                if running_tasks[i].stop():
                    response = 'Stopped ' + chosen_task
                else:
                    response = chosen_task + ' not running, cannot terminate it'

    return response + '\n'

def task_handler_server(is_local=False):
    rospy.init_node('task_handler_server')
    s = rospy.Service('task_handler', HandleTask, handle_task_request)
    ready_msg = "Ready to respond to task handling requests"

    if is_local:
        ready_msg += " (local mode)"

    print(ready_msg)
    rospy.spin()

if __name__ == "__main__":
    is_local = False

    if "local" in sys.argv:
        print("Task handler local mode supported tasks")
        print("--> camera_stream")
        is_local = True

    # this value is set at runtime based off how many video port devices are identified
    max_streams = 0
    # how many streams are currently on
    active_stream_ctr = 0

    current_dir = os.path.dirname(os.path.realpath(__file__)) + "/"
    mcu_control_dir = current_dir + '../../mcu_control/scripts/'

    # tasks that can be run
    scripts = [mcu_control_dir + "RoverNode.py", mcu_control_dir + "ArmNode.py", mcu_control_dir + "ScienceNode.py", mcu_control_dir + "PdsNode.py", current_dir + "start_ros_stream.sh"]

    # set up listener objects
    running_tasks = [Listener(scripts[0], "python3"), Listener(scripts[1], "python3"), Listener(scripts[2], "python3"), Listener(scripts[3], "python3"), Listener(scripts[4], "bash", "", 1, True)]

    # expected client arguments for choosing task
    known_tasks = ['rover_listener', 'arm_listener', 'science_listener', 'pds_listener', 'camera_stream', 'camera_ports']
    known_listeners = known_tasks[:-2]

    # keep track of currently running streams
    active_ports = []
    active_streams = []

    for i in range(len(running_tasks)):
        running_tasks[i].set_name(known_tasks[i])

    task_handler_server(is_local)
