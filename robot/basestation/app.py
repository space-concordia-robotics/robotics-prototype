#!/usr/bin/env python
"""Flask server controller.

Flask is light-weight and modular so this is actually all we need to set up a simple HTML page.
"""

import os
import subprocess
from subprocess import Popen, PIPE
from urllib.parse import unquote
import flask
from flask import jsonify, request
from robot.comms.connection import Connection
import time
import datetime
from shlex import split

import robot.basestation.stream_capture as stream_capture
from robot.basestation.stream_capture import start_recording_feed, stop_recording_feed
import robot.basestation.ros_utils as ros_utils
from robot.basestation.ros_utils import fetch_ros_master_uri, fetch_ros_master_ip

app = flask.Flask(__name__)


def run_shell(cmd, args="", print_output=True):
    """Run script command supplied as string.

    Returns tuple of output and error.
    """
    cmd_list = cmd.split()
    arg_list = args.split()

    for arg in arg_list:
        cmd_list.append(str(arg))

    if print_output:
        print("arg_list:", arg_list)
        print("cmd_list:", cmd_list)

    process = subprocess.Popen(cmd_list, stdout=subprocess.PIPE)
    output, error = process.communicate()

    return output, error


def get_pid(keyword):
    cmd = "ps aux"
    output, error = run_shell(cmd)

    ting = output.decode().split('\n')

    # print(ting)

    for line in ting:
        if keyword in line:
            #print("FOUND PID:", line)
            words = line.split()
            print("PID:", words[1])

            return words[1]

    return -1

# Once we launch this, this will route us to the "/" page or index page and
# automatically render the Robot GUI
@app.route("/")
@app.route("/arm")
def index():
    """Current landing page, the arm panel."""
    return flask.render_template("pages/Arm.html", roverIP=fetch_ros_master_ip())


@app.route("/rover")
def rover():
    """Rover control panel."""
    return flask.render_template("pages/Rover.html", roverIP=fetch_ros_master_ip())


@app.route("/science")
def science():
    """Science page."""
    return flask.render_template("pages/Science.html", roverIP=fetch_ros_master_ip())


@app.route("/pds")
def pds():
    """PDS page."""
    return flask.render_template("pages/PDS.html", roverIP=fetch_ros_master_ip())


@app.route("/navigation")
def navigation():
    """Navigation page."""
    return flask.render_template("pages/Navigation.html", roverIP=fetch_ros_master_ip())


# routes for science page
@app.route('/science/numSections')
def numSections():
    return '4'


@app.route("/stream")
def stream():
    """Stream page."""
    return flask.render_template("pages/Stream.html", roverIP=fetch_ros_master_ip())


@app.route('/science/initialSection')
def initialSection():
    return '0'

# routes for navigation pages
@app.route('/navigation/inputTemplates/goal-buttons/<count>', methods=["GET"])
def goal_Buttons(count):
    return flask.render_template("elements/navigation/htmlTemplates/goal/goal-buttons.html", count=count)


@app.route('/navigation/inputTemplates/new-goal-coordinates-btn/<goalId>', methods=["GET"])
def new_Goal_Button(goalId):
    return flask.render_template("elements/navigation/htmlTemplates/goal/new-goal-coordinates-btn.html", goalId=goalId)


@app.route('/navigation/inputTemplates/antenna-DD/<target>', methods=["GET"])
def antenna_DD(target):
    return flask.render_template("elements/navigation/htmlTemplates/antenna/antenna-DD-input-template.html", target=target)


@app.route('/navigation/inputTemplates/antenna-DDM/<target>', methods=["GET"])
def antenna_DDM(target):
    return flask.render_template("elements/navigation/htmlTemplates/antenna/antenna-DDM-input-template.html", target=target)


@app.route('/navigation/inputTemplates/antenna-DMS/<target>', methods=["GET"])
def antenna_DMS(target):
    return flask.render_template("elements/navigation/htmlTemplates/antenna/antenna-DMS-input-template.html", target=target)


@app.route('/navigation/inputTemplates/goal-DD/<target>/<count>', methods=["GET"])
def goal_DD(target, count):
    return flask.render_template("elements/navigation/htmlTemplates/goal/goal-DD-input-template.html", target=target, count=count)


@app.route('/navigation/inputTemplates/goal-DDM/<target>/<count>', methods=["GET"])
def goal_DDM(target, count):
    return flask.render_template("elements/navigation/htmlTemplates/goal/goal-DDM-input-template.html", target=target, count=count)


@app.route('/navigation/inputTemplates/goal-DMS/<target>/<count>', methods=["GET"])
def goal_DMS(target, count):
    return flask.render_template("elements/navigation/htmlTemplates/goal/goal-DMS-input-template.html", target=target,  count=count)


# globals to store the html data in body of the request to it can be returned when another request is made to that same url.
antenna_modal_serverHTML = ''
antenna_stats_serverHTML = ''
goal_modal_serverHTML = ''
goal_stats_serverHTML = ''
navQueue_serverHTML = ''


@app.route('/navigation/cached_content/antenna_stats', methods=["POST", "GET"])
def antenna_stats_server():
    if request.method == 'POST':
        global antenna_stats_serverHTML
        data = request.get_json()
        antenna_stats_serverHTML = flask.json.dumps(data)
        return jsonify(success=True)
    if request.method == 'GET':
        return jsonify(antenna_stats_serverHTML)


@app.route('/navigation/cached_content/antenna_modal', methods=["POST", "GET"])
def antenna_modal_server():
    if request.method == 'POST':
        global antenna_modal_serverHTML
        antenna_modal_serverHTML = (request.get_data().decode('UTF-8'))
        return flask.render_template_string(antenna_modal_serverHTML)
    if request.method == 'GET':
        return flask.render_template_string(antenna_modal_serverHTML)


@app.route('/navigation/cached_content/goal_modal', methods=["POST", "GET"])
def goal_modal_server():
    if request.method == 'POST':
        global goal_modal_serverHTML
        goal_modal_serverHTML = (request.get_data().decode('UTF-8'))
        return flask.render_template_string(goal_modal_serverHTML)
    if request.method == 'GET':
        return flask.render_template_string(goal_modal_serverHTML)


@app.route('/navigation/cached_content/goal_stats', methods=["POST", "GET"])
def goal_stats_server():
    if request.method == 'POST':
        global goal_stats_serverHTML
        data = request.get_json()
        goal_stats_serverHTML = flask.json.dumps(data)
        return jsonify(success=True)
    if request.method == 'GET':
        return jsonify(goal_stats_serverHTML)


@app.route('/navigation/cached_content/navQueue', methods=["POST", "GET"])
def navQueue():
    if request.method == 'POST':
        global navQueue_serverHTML
        data = request.get_json()
        navQueue_serverHTML = flask.json.dumps(data)
        return jsonify(success=True)
    if request.method == 'GET':
        return jsonify(navQueue_serverHTML)

    """Pings ROS_MASTER_URI and return response object with resulting outputs.

    Pings rover first directly with Unix ping command,
    then using ros ping_acknowledgment service.

    Returns JSON object with the following fields:
    success -- whether requests was successful
    ping_msg -- output of Unix ping command
    ros_msg -- output of the ROS ping_acknowledgment service
    """
    ping_output, error = run_shell("ping -c 1 " + fetch_ros_master_ip())
    ping_output = ping_output.decode()

    print("Output: " + ping_output)

    if "Destination Net Unreachable" in ping_output:
        error_msg = "Basestation has no connection to network, aborting ROS ping."
        return jsonify(success=False, ping_msg=ping_output, ros_msg=error_msg)

    if "Destination Host Unreachable" in ping_output:
        error_msg = "Rover has no connection to network, aborting ROS ping."
        return jsonify(success=False, ping_msg=ping_output, ros_msg=error_msg)

    if error:
        print("Error: " + error.decode())

    ros_output, error = run_shell(
        "rosrun ping_acknowledgment ping_response_client.py hello")
    ros_output = ros_output.decode()

    print("Pinging rover")
    print("Output: " + ros_output)

    if error:
        print("Error: " + error.decode())

    return jsonify(success=True, ping_msg=ping_output, ros_msg=ros_output)


# only to be used when hacky implementation is fixed
# see odroid_rx package for details
@app.route("/odroid_rx", methods=["POST"])
def odroid_rx():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_file = script_dir + "/../rospackages/src/odroid_rx/scripts/odroid_rx.txt"
    print("odroid_rx")

    # query the topic exactly once
    output, error = run_shell("cat", log_file)
    output = str(output, "utf-8")

    print("output: " + output)

    return jsonify(success=True, odroid_rx=output)

# Rover controls
@app.route("/rover_drive", methods=["POST"])
def rover_drive():
    print("rover_drive")

    cmd = str(request.get_data('cmd'), "utf-8")
    print("cmd: " + cmd)
    # remove fluff, only command remains
    if cmd:
        cmd = cmd.split("=")[1]
        # decode URI
        cmd = unquote(cmd)

    if local:
        rover_ip = "127.0.0.1"
        base_ip = rover_ip
        rover_port = 5020
        base_port = 5025
    else:
        rover_ip = "172.16.1.30"
        base_ip = "172.16.1.20"
        rover_port = 5030
        base_port = rover_port
    print("cmd: " + cmd)
    sender = Connection("rover_drive_sender", rover_ip, rover_port)

    error = str(None)

    try:
        sender.send(cmd)
    except OSError:
        error = "Network is unreachable"
        print(error)

    receiver = Connection("rover_drive_receiver", base_ip, base_port)
    feedback = str(None)
    error = str(None)

    try:
        feedback = receiver.receive(timeout=2)
    except OSError:
        error = "Network error"
        print(error)

    print("feedback:", feedback)

    if not feedback:
        feedback = "Timeout limit exceeded, no data received"

    return jsonify(success=True, cmd=cmd, feedback=feedback, error=error)

# capture image
@app.route("/capture_image", methods=["POST", "GET"])
def capture_image():
    stream_url = "http://" + fetch_ros_master_ip() + \
        ":8080/stream?topic=/cv_camera/image_raw"
    #lserror, lsoutput = run_shell("ls -1q img* | wc -l")
    # p1 = subprocess.Popen(split("ls -1q img*"), stdout=subprocess.PIPE)
    # p2 = subprocess.Popen(split("wc -l"), stdin=p1.stdout)
    # output, error = p2.communicate()
    output, error = run_shell('ls')
    output = output.decode()
    print('output:', output)
    i = 0

    if 'img' in output:
        i = output.rfind('img')
        i = int(output[i + 3]) + 1  # shift by 'img'
        print('i', i)

    error, output = run_shell("ffmpeg -i " + stream_url +
                              " -ss 00:00:01.500 -f image2 -vframes 1 img" + str(i) + ".jpg")
    msg = "success"

    if error:
        msg = "F"

    print('msg', msg)

    return jsonify(msg=msg)


@app.route("/initiate_feed_recording/<stream>", methods=["POST", "GET"])
def initiate_feed_recording(stream):
    return start_recording_feed(stream)


@app.route("/stop_feed_recording/<stream>", methods=["POST", "GET"])
def stop_feed_recording(stream):
    return stop_recording_feed(stream)


if __name__ == "__main__":

    # feature toggles
    # the following two are used for UDP based communication with the Connection class
    global local
    local = False
    # print("fetch_ros_master_ip:", fetch_ros_master_ip())
    #
    # # either local or competition
    # ros_master_ip = fetch_ros_master_ip()
    # if ros_master_ip in ["127.0.0.1", "localhost"]
    #     local = True

    app.run(debug=True, host='0.0.0.0')
    # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
