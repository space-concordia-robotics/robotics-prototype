#!/usr/bin/env python
"""Flask server controller.

Flask is light-weight and modular so this is actually all we need to set up a simple HTML page.
"""

import os
import subprocess
import urllib
from urllib.parse import urlparse, unquote
import flask
from flask import jsonify, request
# import time

# TODO: Rename modules to lowercase
from robot.basestation.motor import Motor
from robot.basestation.serialport import SerialPort
from robot.basestation.microcontroller import Microcontroller

app = flask.Flask(__name__)


def fetch_ros_master_uri():
    """Fetch and parse ROS Master URI from environment variable.

    The parsed URI is returned as a urllib.parse.ParseResult instance.

    Returns:
        urllib.parse.ParseResult: 6-tuple instance with various attributes.

    Attributes (urllib.parse.ParseResult):
    - hostname -- the ip address or the dns resolvable name
    - port -- the port number
    - etc...

    See https://docs.python.org/3/library/urllib.parse.html?highlight=urlparse#urllib.parse.urlparse
    """
    return urlparse(os.environ["ROS_MASTER_URI"])


def fetch_ros_master_ip():
    """Fetch only the hostname (host IP) portion of the parse URI."""
    return fetch_ros_master_uri().hostname


def run_shell(cmd, arg=""):
    """Run script command supplied as string.

    Returns tuple of output and error.
    """
    cmd_list = cmd.split()
    cmd_list.append(str(arg))
    process = subprocess.Popen(cmd_list, stdout=subprocess.PIPE)
    output, error = process.communicate()

    return output, error


# Once we launch this, this will route us to the "/" page or index page and
# automatically render the Robot GUI
@app.route("/")
def index():
    """Current landing page, the arm panel."""
    return flask.render_template("AsimovOperation.html", roverIP=fetch_ros_master_ip())


@app.route("/rover")
def rover():
    """Rover control panel."""
    return flask.render_template("Rover.html", roverIP=fetch_ros_master_ip())


@app.route("/science")
def science():
    """Science page."""
    return flask.render_template("Science.html", roverIP=fetch_ros_master_ip())


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


@app.route("/select_mux", methods=["POST", "GET"])
def select_mux():
    print("select_mux")
    dev = str(request.get_data(), "utf-8")
    print("dev : " + dev)

    output, error = run_shell("rosrun mux_selector mux_select_client.py", dev)
    output = str(output, "utf-8")
    print("output: " + output)

    return jsonify(success=True, dev=dev, output=output)


@app.route("/serial_cmd", methods=["POST", "GET"])
def serial_cmd():
    print("serial_cmd")

    cmd = str(request.get_data('cmd'), "utf-8")
    print("cmd: " + cmd)
    # remove fluff, only command remains
    if cmd:
        cmd = cmd.split("=")[1]
        # decode URI
        cmd = unquote(cmd)

    print("cmd: " + cmd)

    ros_cmd = "rosrun serial_cmd serial_cmd_client.py '" + cmd + "'"
    print("ros_cmd: " + ros_cmd)

    output, error = run_shell("rosrun serial_cmd serial_cmd_client.py", cmd)
    output = str(output, "utf-8")

    print("output: " + output)

    return jsonify(success=True, cmd=cmd, output=output)


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


# Declare some placeholder values for `Motor.__init__` parameters
max_angle = 160
min_angle = 0
max_current = 3
min_current = 0
home_angle = 0

# Initialize Motor class object instance variables to map to various motor button objects
m1 = Motor("1", max_angle, min_angle, max_current, min_current, home_angle)
m2 = Motor("2", max_angle, min_angle, max_current, min_current, home_angle)
m3 = Motor("3", max_angle, min_angle, max_current, min_current, home_angle)
m4 = Motor("4", max_angle, min_angle, max_current, min_current, home_angle)
m5 = Motor("5", max_angle, min_angle, max_current, min_current, home_angle)
m6 = Motor("6", max_angle, min_angle, max_current, min_current, home_angle)

# Initialize a Port instance used to connect to teensy via serial
# port = SerialPort(path="/dev/cu.usbmodem3049051", baudrate=9600, timeout=1)
serial_port = SerialPort()

# Intialize `Microcontroller` object representing the mother Arduino object
# containing `Motor` instance array
teensy = Microcontroller("teensy", serial_port, [m1, m2, m3, m4, m5, m6])


# Automatic controls
@app.route("/mousedown_btn_pitch_up")
def mousedown_btn_pitch_up():
    btn_id = "mousedown_btn_pitch_up"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_pitch_down")
def mousedown_btn_pitch_down():
    btn_id = "mousedown_btn_pitch_down"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_roll_left")
def mousedown_btn_roll_left():
    btn_id = "mousedown_btn_roll_left"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_roll_right")
def mousedown_btn_roll_right():
    btn_id = "mousedown_btn_roll_right"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_claw_open")
def mousedown_btn_claw_open():
    btn_id = "mousedown_btn_claw_open"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_claw_close")
def mousedown_btn_claw_close():
    btn_id = "mousedown_btn_claw_close"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_up")
def mousedown_btn_arm_up():
    btn_id = "mousedown_btn_arm_up"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_down")
def mousedown_btn_arm_down():
    btn_id = "mousedown_btn_arm_down"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_left")
def mousedown_btn_arm_left():
    btn_id = "mousedown_btn_arm_left"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_right")
def mousedown_btn_arm_right():
    btn_id = "mousedown_btn_arm_right"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_backward")
def mousedown_btn_arm_backward():
    btn_id = "mousedown_btn_arm_backward"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_arm_forward")
def mousedown_btn_arm_forward():
    btn_id = "mousedown_btn_arm_forward"
    print(btn_id)
    return jsonify(success=True, button=btn_id)


# Manual controls
@app.route("/mousedown_btn_motor1_ccw")
def mousedown_btn_motor1_ccw():
    btn_id = "mousedown_btn_motor1_ccw"
    print(btn_id)
    teensy.motors.get(name="1").rotate(20, direction='ccw')
    # teensy.write(m1.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor1_cw")
def mousedown_btn_motor1_cw():
    btn_id = "mousedown_btn_motor1_cw"
    print(btn_id)
    teensy.motors.get(name="1").rotate(20, direction='cw')
    # teensy.write(m1.name, "0")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor2_ccw")
def mousedown_btn_motor2_ccw():
    btn_id = "mousedown_btn_motor2_ccw"
    print(btn_id)
    teensy.motors.get(name="2").rotate(20, direction='ccw')
    # teensy.write(m2.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor2_cw")
def mousedown_btn_motor2_cw():
    btn_id = "mousedown_btn_motor2_cw"
    print(btn_id)
    teensy.motors.get(name="2").rotate(20, direction='cw')
    # teensy.write(m2.name, "0")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor3_ccw")
def mousedown_btn_motor3_ccw():
    btn_id = "mousedown_btn_motor3_ccw"
    print(btn_id)
    teensy.motors.get(name="3").rotate(20, direction='ccw')
    # teensy.write(m3.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor3_cw")
def mousedown_btn_motor3_cw():
    btn_id = "mousedown_btn_motor3_cw"
    print(btn_id)
    teensy.motors.get(name="3").rotate(20, direction='cw')
    # teensy.write(m3.name, "0")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor4_ccw")
def mousedown_btn_motor4_ccw():
    btn_id = "mousedown_btn_motor4_ccw"
    print(btn_id)
    teensy.motors.get(name="4").rotate(20, direction='ccw')
    # teensy.write(m4.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor4_cw")
def mousedown_btn_motor4_cw():
    btn_id = "mousedown_btn_motor4_cw"
    print(btn_id)
    teensy.motors.get(name="4").rotate(20, direction='cw')
    # teensy.write(m4.name, "0")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor5_ccw")
def mousedown_btn_motor5_ccw():
    btn_id = "mousedown_btn_motor5_ccw"
    print(btn_id)
    teensy.motors.get(name="5").rotate(20, direction='ccw')
    # teensy.write(m5.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor5_cw")
def mousedown_btn_motor5_cw():
    btn_id = "mousedown_btn_motor5_cw"
    print(btn_id)
    teensy.motors.get(name="5").rotate(20, direction='cw')
    # teensy.write(m5.name, "0")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor6_ccw")
def mousedown_btn_motor6_ccw():
    btn_id = "mousedown_btn_motor6_ccw"
    print(btn_id)
    teensy.motors.get(name="6").rotate(20, direction='ccw')
    # teensy.write(m6.name, "1")
    return jsonify(success=True, button=btn_id)


@app.route("/mousedown_btn_motor6_cw")
def mousedown_btn_motor6_cw():
    btn_id = "mousedown_btn_motor6_cw"
    print(btn_id)
    teensy.motors.get(name="6").rotate(20, direction='cw')
    # teensy.write(m6.name, "0")
    return jsonify(success=True, button=btn_id)


app.run(debug=True)
# add param `host= '0.0.0.0'` if you want to run on your machine's IP address
