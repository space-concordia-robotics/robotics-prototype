#!/usr/bin/env python3
# This is the controller page of the Flask GUI
# Flask is light-weight and modular so this is actually all we need to set up a simple HTML page

import flask
# import time

# TODO: Rename modules to lowercase
from robot.basestation.motor import Motor
from robot.basestation.serialport import SerialPort
from robot.basestation.microcontroller import Microcontroller

app = flask.Flask(__name__)


# Once we launch this, this will route us to the "../" page or index page and
# automatically render the Rover GUI
@app.route("/")
def index():
    return flask.render_template("AsimovOperation.html")


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
    print("mousedown_btn_pitch_up")
    return ""


@app.route("/mousedown_btn_pitch_down")
def mousedown_btn_pitch_down():
    print("mousedown_btn_pitch_down")
    return ""


@app.route("/mousedown_btn_roll_left")
def mousedown_btn_roll_left():
    print("mousedown_btn_roll_left")
    return ""


@app.route("/mousedown_btn_roll_right")
def mousedown_btn_roll_right():
    print("mousedown_btn_roll_right")
    return ""


@app.route("/mousedown_btn_claw_open")
def mousedown_btn_claw_open():
    print("mousedown_btn_claw_open")
    return ""


@app.route("/mousedown_btn_claw_close")
def mousedown_btn_claw_close():
    print("mousedown_btn_claw_close")
    return ""


@app.route("/mousedown_btn_arm_up")
def mousedown_btn_arm_up():
    print("mousedown_btn_arm_up")
    return ""


@app.route("/mousedown_btn_arm_down")
def mousedown_btn_arm_down():
    print("mousedown_btn_arm_down")
    return ""


@app.route("/mousedown_btn_arm_left")
def mousedown_btn_arm_left():
    print("mousedown_btn_arm_left")
    return ""


@app.route("/mousedown_btn_arm_right")
def mousedown_btn_arm_right():
    print("mousedown_btn_arm_right")
    return ""


@app.route("/mousedown_btn_arm_back")
def mousedown_btn_arm_back():
    print("mousedown_btn_arm_back")
    return ""


@app.route("/mousedown_btn_arm_forward")
def mousedown_btn_arm_forward():
    print("mousedown_btn_arm_forward")
    return ""


# Manual controls
@app.route("/mousedown_btn_motor1_ccw")
def mousedown_btn_motor1_ccw():
    print("mousedown_btn_motor1_ccw")
    teensy.motors.get(name="1").rotate(20, direction='ccw')
    # teensy.write(m1.name, "1")
    return ""


@app.route("/mousedown_btn_motor1_cw")
def mousedown_btn_motor1_cw():
    print("/mousedown_btn_motor1_cw")
    teensy.motors.get(name="1").rotate(20, direction='cw')
    # teensy.write(m1.name, "0")
    return ""


@app.route("/mousedown_btn_motor2_ccw")
def mousedown_btn_motor2_ccw():
    print("mousedown_btn_motor2_ccw")
    teensy.motors.get(name="2").rotate(20, direction='ccw')
    # teensy.write(m2.name, "1")
    return ""


@app.route("/mousedown_btn_motor2_cw")
def mousedown_btn_motor2_cw():
    print("mousedown_btn_motor2_cw")
    teensy.motors.get(name="2").rotate(20, direction='cw')
    # teensy.write(m2.name, "0")
    return ""


@app.route("/mousedown_btn_motor3_ccw")
def mousedown_btn_motor3_ccw():
    print("mousedown_btn_motor3_ccw")
    teensy.motors.get(name="3").rotate(20, direction='ccw')
    # teensy.write(m3.name, "1")
    return ""


@app.route("/mousedown_btn_motor3_cw")
def mousedown_btn_motor3_cw():
    print("mousedown_btn_motor3_cw")
    teensy.motors.get(name="3").rotate(20, direction='cw')
    # teensy.write(m3.name, "0")
    return ""


@app.route("/mousedown_btn_motor4_ccw")
def mousedown_btn_motor4_ccw():
    print("mousedown_btn_motor4_ccw")
    teensy.motors.get(name="4").rotate(20, direction='ccw')
    # teensy.write(m4.name, "1")
    return ""


@app.route("/mousedown_btn_motor4_cw")
def mousedown_btn_motor4_cw():
    print("mousedown_btn_motor4_cw")
    teensy.motors.get(name="4").rotate(20, direction='cw')
    # teensy.write(m4.name, "0")
    return ""


@app.route("/mousedown_btn_motor5_ccw")
def mousedown_btn_motor5_ccw():
    print("mousedown_btn_motor5_ccw")
    teensy.motors.get(name="5").rotate(20, direction='ccw')
    # teensy.write(m5.name, "1")
    return ""


@app.route("/mousedown_btn_motor5_cw")
def mousedown_btn_motor5_cw():
    print("mousedown_btn_motor5_cw")
    teensy.motors.get(name="5").rotate(20, direction='cw')
    # teensy.write(m5.name, "0")
    return ""


@app.route("/mousedown_btn_motor6_ccw")
def mousedown_btn_motor6_ccw():
    print("mousedown_btn_motor6_ccw")
    teensy.motors.get(name="6").rotate(20, direction='ccw')
    # teensy.write(m6.name, "1")
    return ""


@app.route("/mousedown_btn_motor6_cw")
def mousedown_btn_motor6_cw():
    print("mousedown_btn_motor6_cw")
    teensy.motors.get(name="6").rotate(20, direction='cw')
    # teensy.write(m6.name, "0")
    return ""


app.run(
    debug=True
)  # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
