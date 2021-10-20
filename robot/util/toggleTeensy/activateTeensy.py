#!/usr/bin/env python3
import Jetson.GPIO as gpio

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)

gpio.setup(33, gpio.OUT)
gpio.output(33, gpio.HIGH)
