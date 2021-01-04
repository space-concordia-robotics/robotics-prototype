from flask import Flask, render_template
import serial
import sys
import time
app = Flask(__name__)

temperatures = [15, 15, 15, 15]
currents = [1, 1, 1, 1]

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
    return (temperatures, 200)

# have a loop that collects telemetry

while True:
    time.sleep(0.5)
    # update telemetry
    temperatures = [10, 10, 10, 10] # from device
