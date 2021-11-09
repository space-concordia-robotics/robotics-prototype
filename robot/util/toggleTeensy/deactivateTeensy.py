#!/usr/bin/env python3
import Jetson.GPIO as gpio
from enablePins import enablePins

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)

gpio.setup(enablePins, gpio.OUT)
gpio.output(enablePins, gpio.LOW)
