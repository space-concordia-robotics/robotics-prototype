#!/usr/bin/env python3
# This is the controller page of the Flask GUI
# Flask is light-weight and modular so this is actually all we need to set up a simple HTML page

import flask

app = flask.Flask(__name__)

# set the rover's IP
# rover static ip (tenatative): 192.168.1.20
ROVER_IP = "127.31.43.134"

# Once we launch this, this will route us to the "../" page or index page and
# automatically render the Rover GUI
@app.route("/")
def index():
    return flask.render_template("AsimovOperation.html", roverIP=ROVER_IP)


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
    return ""


@app.route("/click_btn_motor1_cw")
def click_btn_motor1_cw():
    print("click_btn_motor1_cw")
    return ""


@app.route("/click_btn_motor2_ccw")
def click_btn_motor2_ccw():
    print("click_btn_motor2_ccw")
    return ""


@app.route("/click_btn_motor2_cw")
def click_btn_motor2_cw():
    print("click_btn_motor2_cw")
    return ""


@app.route("/click_btn_motor3_ccw")
def click_btn_motor3_ccw():
    print("click_btn_motor3_ccw")
    return ""


@app.route("/click_btn_motor3_cw")
def click_btn_motor3_cw():
    print("click_btn_motor3_cw")
    return ""


@app.route("/click_btn_motor4_ccw")
def click_btn_motor4_ccw():
    print("click_btn_motor4_ccw")
    return ""


@app.route("/click_btn_motor4_cw")
def click_btn_motor4_cw():
    print("click_btn_motor4_cw")
    return ""


@app.route("/click_btn_motor5_ccw")
def click_btn_motor5_ccw():
    print("click_btn_motor5_ccw")
    return ""


@app.route("/click_btn_motor5_cw")
def click_btn_motor5_cw():
    print("click_btn_motor5_cw")
    return ""


@app.route("/click_btn_motor6_ccw")
def click_btn_motor6_ccw():
    print("click_btn_motor6_ccw")
    return ""


@app.route("/click_btn_motor6_cw")
def click_btn_motor6_cw():
    print("click_btn_motor6_cw")
    return ""


app.run(
    debug=True
)  # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
