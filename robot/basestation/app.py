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
import robot.util.utils as utils
from robot.util.utils import run_shell
from shlex import split

import robot.basestation.stream_capture as stream_capture
from robot.basestation.stream_capture import start_recording_feed, stop_recording_feed, \
is_recording_stream, stream_capture, add_rotation
import robot.basestation.ros_utils as ros_utils
from robot.basestation.ros_utils import fetch_ros_master_uri, fetch_ros_master_ip

app = flask.Flask(__name__)

# Once we launch this, this will route us to the "/" page or index page and
# automatically render the Robot GUI
@app.route("/")
@app.route("/arm")
def index():
    """Current landing page, the arm panel."""
    return flask.render_template("pages/Arm.html", roverIP=fetch_ros_master_ip())

@app.route("/camerapopup")
def camerapopup():
    """Camera Pop-up."""
    return flask.render_template("pages/CameraPopUp.html", roverIP=fetch_ros_master_ip())

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

@app.route("/stream")
def stream():
    """Streams page."""
    return flask.render_template("pages/Streams.html", roverIP=fetch_ros_master_ip())

# routes for science page
@app.route('/science/numSections')
def numSections():
    return '4'

@app.route('/science/initialSection')
def initialSection():
    return '0'

@app.route("/ping_rover")
def ping_rover():
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

    ros_output, error = run_shell("rosrun ping_acknowledgment ping_response_client.py hello")
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

@app.route("/capture_image/", methods=["POST", "GET"])
def capture_image():
    stream_url= request.args['stream_url']
    rotation = int(request.args['camera_rotation'])
    success, message = stream_capture(stream_url, rotation)
    return jsonify(success=success, msg=message)

@app.route("/initiate_feed_recording/", methods=["POST", "GET"])
def initiate_feed_recording():
    stream_url = request.args['stream_url']
    if is_recording_stream(stream_url):
        return jsonify(success=False, msg="Stream is already recording")
    else:
        success, message = start_recording_feed(stream_url)
        return jsonify(success=success, msg=message)

@app.route("/stop_feed_recording/", methods=["POST", "GET"])
def stop_feed_recording():
    stream_url = request.args['stream_url']
    rotation = int(request.args['camera_rotation'])
    if is_recording_stream(stream_url):
        success, message = add_rotation(stream_url, rotation)

        if not success:
            print('add_rotation method failed:', message)

        success, message = stop_recording_feed(stream_url)

        if not success:
            print('stop_recording_feed method failed:', message)
        return jsonify(success=success, msg=message)
    else:
        return jsonify(success=False, msg="Attempted to stop stream that was not recording")

@app.route("/is_recording/", methods=["POST", "GET"])
def is_recording():
    stream_url = request.args['stream_url']
    return jsonify(is_recording=is_recording_stream(stream_url))

if __name__ == "__main__":

    # feature toggles
    # the following two are used for UDP based communication with the Connection class
    global local
    local = False

    app.run(debug=True, host='0.0.0.0')
    # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
