#!/usr/bin/env python3
# This is the controller page of the Flask GUI
# Flask is light-weight and modular so this is actually all we need to set up a simple HTML page

import flask
# T0DO: Rename modules to lowercase
from robot.basestation.Motor import Motor
from robot.basestation.Port import Port
from robot.basestation.Microcontroller import Microcontroller

app = flask.Flask(__name__)


# Once we launch this, this will route us to the "../" page or index page and
# automatically render the Rover GUI
@app.route("/")
def index():
    return flask.render_template("AsimovOperation.html")


# Declare some placeholder values for `Motor.__init__` parameters
max_angle = 180
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
port = Port(path="/dev/cu.usbmodem3049051", baudrate=9600, timeout=1)

# Intialize `Microcontroller` object representing the mother Arduino object
# containing `Motor`` instance array
teensy = Microcontroller("Arduino", port, [m1, m2, m3, m4, m5, m6])


# Automatic controls
@app.route("/click_btn_pitch_up")
def click_pitch_up():
    print("click_btn_pitch_up")
    return ""


@app.route("/click_btn_pitch_down")
def click_btn_pitch_down():
    print("click_btn_pitch_down")
    return ""


@app.route("/click_btn_roll_left")
def click_btn_roll_left():
    print("click_btn_roll_left")
    return ""


@app.route("/click_btn_roll_right")
def click_btn_roll_right():
    print("click_btn_roll_right")
    return ""


@app.route("/click_btn_claw_open")
def click_btn_claw_open():
    print("click_btn_claw_open")
    return ""


@app.route("/click_btn_claw_close")
def click_btn_claw_close():
    print("click_btn_claw_close")
    return ""


@app.route("/click_btn_arm_up")
def click_btn_arm_up():
    print("click_btn_arm_up")
    return ""


@app.route("/click_btn_arm_down")
def click_btn_arm_down():
    print("click_btn_arm_down")
    return ""


@app.route("/click_btn_arm_left")
def click_btn_arm_left():
    print("click_btn_arm_left")
    return ""


@app.route("/click_btn_arm_right")
def click_btn_arm_right():
    print("click_btn_arm_right")
    return ""


@app.route("/click_btn_arm_back")
def click_btn_arm_back():
    print("click_btn_arm_back")
    return ""


@app.route("/click_btn_arm_forward")
def click_btn_arm_forward():
    print("click_btn_arm_forward")
    return ""


# Manual controls
@app.route("/click_btn_motor1_ccw")
def click_btn_motor1_ccw():
    print("click_btn_motor1_ccw")
    # teensy.write("Motor 1", teensy.read("Motor 1") - 10)
    teensy.write(m1.name, "1")
    return ""


@app.route("/click_btn_motor1_cw")
def click_btn_motor1_cw():
    print("click_btn_motor1_cw")
    # teensy.write(m1.name, teensy.read("Motor 1") + 10)
    teensy.write(m1.name, "0")
    return ""


@app.route("/click_btn_motor2_ccw")
def click_btn_motor2_ccw():
    print("click_btn_motor2_ccw")
    teensy.write(m2.name, "1")
    return ""


@app.route("/click_btn_motor2_cw")
def click_btn_motor2_cw():
    print("click_btn_motor2_cw")
    teensy.write(m2.name, "0")
    return ""


@app.route("/click_btn_motor3_ccw")
def click_btn_motor3_ccw():
    print("click_btn_motor3_ccw")
    teensy.write(m3.name, "1")
    return ""


@app.route("/click_btn_motor3_cw")
def click_btn_motor3_cw():
    print("click_btn_motor3_cw")
    teensy.write(m3.name, "0")
    return ""


@app.route("/click_btn_motor4_ccw")
def click_btn_motor4_ccw():
    print("click_btn_motor4_ccw")
    teensy.write(m4.name, "1")
    return ""


@app.route("/click_btn_motor4_cw")
def click_btn_motor4_cw():
    print("click_btn_motor4_cw")
    teensy.write(m4.name, "0")
    return ""


@app.route("/click_btn_motor5_ccw")
def click_btn_motor5_ccw():
    print("click_btn_motor5_ccw")
    teensy.write(m5.name, "1")
    return ""


@app.route("/click_btn_motor5_cw")
def click_btn_motor5_cw():
    print("click_btn_motor5_cw")
    teensy.write(m5.name, "0")
    return ""


@app.route("/click_btn_motor6_ccw")
def click_btn_motor6_ccw():
    print("click_btn_motor6_ccw")
    teensy.write(m6.name, "1")
    return ""


@app.route("/click_btn_motor6_cw")
def click_btn_motor6_cw():
    print("click_btn_motor6_cw")
    teensy.write(m6.name, "0")
    return ""


app.run(
    debug=True
)  # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
