from flask import Flask, render_template, jsonify
import serial
import sys
import time
import threading
import random
# https://github.com/MartensCedric/sc-robotics-initiation/blob/master/binaryclock.py

app = Flask(__name__)

temperatures = [15, 15, 15, 15]
currents = [1, 1, 1, 1]
reading = True

@app.route('/')
def main_page():
    return render_template('main.html')


@app.route('/fanoff')
def fan_off():
    # Send fan off command
    print("Sending Fan off command")
    return ('', 204)

@app.route('/fanon')
def fan_on():
    # Send fan on command
    print("Sending Fan on command")
    return ('', 204)

@app.route('/motoroff')
def motoroff():
    # Send motor off command
    print("Sending motor off command")
    return ('', 204)

@app.route('/motoron')
def motoron():
    # Send motor on command
    print("Sending motor on command")
    return ('', 204)

@app.route('/temperatures')
def get_temperatures():
    return (jsonify(temperatures), 200)

# have a loop that collects telemetry
def read_serial():
    global temperatures
    while reading:
        try:
            #getting telemetry
            # if ever you have an issue of that the port is already used, it might
            # be because there's a thread in the background you can talk to me
            time.sleep(0.2)
            temperatures = [random.randint(0, 25), random.randint(0, 25), random.randint(0, 25), random.randint(0, 25)]
        except Exception as e: print(e)

thread = threading.Thread(target = read_serial)
thread.start()
