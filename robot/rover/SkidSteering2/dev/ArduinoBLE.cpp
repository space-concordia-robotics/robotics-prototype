/*
Name: ArduinoBlue.cpp
Created: 6/28/2017 11:00:39 AM
Author: Jae An
Contact: jaean37@gmail.com
*/

#include "ArduinoBlue.h"
#include <Arduino.h>

ArduinoBlue::ArduinoBlue(Stream &output) :
        _bluetooth(output)
{
}

bool ArduinoBlue::checkBluetooth() {

    bool isDataRead = _bluetooth.available() > 0;

    while (_bluetooth.available() > 0) {
        int intRead = _bluetooth.read();

        // Check for transmission starting
        // If a new transmission starts process the transmission
        if (intRead == DRIVE_TRANSMISSION) {
            processDriveTransmission();
            _currentTransmission = NO_TRANSMISSION;
        }
        else if (intRead == BUTTON_TRANSMISSION) {
            processButtonTransmission();
            _currentTransmission = NO_TRANSMISSION;
        }
        else if (intRead == SLIDER_TRANSMISSION) {
            processSliderTransmission();
            _currentTransmission = NO_TRANSMISSION;
        }
        else if (intRead == TEXT_TRANSMISSION) {
            processTextTransmission();
            _currentTransmission = NO_TRANSMISSION;
        }
        else if (intRead == PATH_TRANSMISSION) {
            processPathTransmission();
            _currentTransmission = NO_TRANSMISSION;
        }
        else if (intRead == CONNECTION_CHECK) {
            _bluetooth.print(CONNECTION_CHECK);
            _currentTransmission = NO_TRANSMISSION;
        }
    }

    return isDataRead;
}

// Stores short transmission into the signal array
void ArduinoBlue::storeShortTransmission() {
    unsigned long prevMillis = millis();
    int intRead;
    while (millis() - prevMillis < SHORT_TRANSMISSION_TIMEOUT) {
        if (_bluetooth.available()) {
            intRead = _bluetooth.read();
            if (intRead == TRANSMISSION_END) break;
            pushToSignalArray(intRead);
        }
    }
}

void ArduinoBlue::processDriveTransmission() {
    storeShortTransmission();
    _throttle = _signal[0];
    _steering = _signal[1];
    clearSignalArray();
}

void ArduinoBlue::processButtonTransmission() {
    storeShortTransmission();
    _button = _signal[0];
    clearSignalArray();
}

void ArduinoBlue::processSliderTransmission() {
    storeShortTransmission();
    _sliderId = _signal[0];
    _sliderVal = _signal[1];
    clearSignalArray();
}

void ArduinoBlue::processTextTransmission() {
    _text = readString();
    clearSignalArray();
}

void ArduinoBlue::processPathTransmission() {
    clearSignalArray();
}

String ArduinoBlue::readString() {
    String s;
    int intRead;
    unsigned long prevTime = millis();

    // Read until end of transmission, timeout is reached, or maximum number of characters is reached.
    prevTime = millis();
    while (millis() - prevTime < TEXT_TRANSMISSION_TIMEOUT) {
        if (_bluetooth.available()) {
            intRead = _bluetooth.read();
            // break the loop if end of transmission
            if (intRead == TRANSMISSION_END) {
                break;
            }
            s += char(intRead);
        }
    }
    return s;
}

void ArduinoBlue::pushToSignalArray(int elem) {
    if ( !(_signalLength + 1 == MAX_SHORT_SIGNAL_LENGTH) ) {
        _signal[_signalLength] = elem;
        _signalLength++;
    }
    else {
        Serial.println("ArduinoBlue: Transmission error...");
    }
}

void ArduinoBlue::clearSignalArray() {
    for (int i = 0; i < _signalLength; i++) {
        _signal[i] = -1;
    }
    _signalLength = 0;
}

int ArduinoBlue::getButton() {
    checkBluetooth();
    int btn = _button;
    _button = -1;
    return btn;
}

int ArduinoBlue::getSliderId() {
    checkBluetooth();
    int id = _sliderId;
    _sliderId = -1;
    return id;
}

int ArduinoBlue::getSliderVal() {
    int val = _sliderVal;
    _sliderVal = -1;
    return val;
}

int ArduinoBlue::getThrottle() {
    checkBluetooth();
    return _throttle;
}

int ArduinoBlue::getSteering() {
    checkBluetooth();
    return _steering;
}

void ArduinoBlue::sendMessage(String msg) {
    _bluetooth.print(msg);
}

bool ArduinoBlue::isConnected() {
    _bluetooth.print(CONNECTION_CHECK);
    // wait for 500 ms
    delay(500);
    if (_bluetooth.available()) {
        return _bluetooth.read() == CONNECTION_CHECK;
    }
    return false;
}

String ArduinoBlue::getText() {
    checkBluetooth();
    String ret = _text;
    _text = "";
    return ret;
}
