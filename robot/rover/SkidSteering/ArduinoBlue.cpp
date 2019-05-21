/*
Name: ArduinoBlue.cpp
Created: 6/28/2017 11:00:39 AM
Author: Jae An
Contact: jaean37@gmail.com
*/

#include "ArduinoBlue.h"
#include <Arduino.h>


typedef union {
	float number;
	uint8_t bytes[4];
} FLOATUNION_t;

ArduinoBlue::ArduinoBlue(Stream &output) :
        _bluetooth(output)
{
}

bool ArduinoBlue::checkBluetooth() {

    bool isDataRead = _bluetooth.available() > 0;

    while (_bluetooth.available() > 0) {
        uint8_t intRead = _bluetooth.read();

        // Check for transmission starting
        // If a new transmission starts process the transmission
        if (intRead == DRIVE_TRANSMISSION) {
            processDriveTransmission();
        }
        else if (intRead == BUTTON_TRANSMISSION) {
            processButtonTransmission();
        }
        else if (intRead == SLIDER_TRANSMISSION) {
            processSliderTransmission();
        }
        else if (intRead == TEXT_TRANSMISSION) {
            processTextTransmission();
        }
        else if (intRead == PATH_TRANSMISSION) {
            processPathTransmission();
        }
        else if (intRead == CONNECTION_CHECK) {
            _bluetooth.print(CONNECTION_CHECK);
        }
    }

    return isDataRead;
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

// TODO: Implement this.
void ArduinoBlue::processPathTransmission() {
	//noInterrupts();
	_pathAvailable = storePathTransmission();
    clearSignalArray();
	//interrupts();
}

void ArduinoBlue::sendLocation(float xPos, float yPos, float headingAngle, float xGoal, float yGoal) {
	_bluetooth.print((char)LOCATION_TRANSMISSION_START);
	sendFloatAsBytes((float)xPos);
	sendFloatAsBytes((float)yPos);
	sendFloatAsBytes((float)headingAngle);
	sendFloatAsBytes((float)xGoal);
	sendFloatAsBytes((float)yGoal);
}

// TODO: May not be needed.
void ArduinoBlue::sendFloatAsBytes(float num) {
	FLOATUNION_t floatUnion;
	floatUnion.number = num;
	_bluetooth.print((char)floatUnion.bytes[0]);
	_bluetooth.print((char)floatUnion.bytes[1]);
	_bluetooth.print((char)floatUnion.bytes[2]);
	_bluetooth.print((char)floatUnion.bytes[3]);
}

float ArduinoBlue::bytesToFloat(uint8_t u1, uint8_t u2, uint8_t u3, uint8_t u4) {
	FLOATUNION_t floatUntion;
	floatUntion.bytes[0] = u1;
	floatUntion.bytes[1] = u2;
	floatUntion.bytes[2] = u3;
	floatUntion.bytes[3] = u4;
	return floatUntion.number;
}

bool ArduinoBlue::storePathTransmission() {
	const int BYTES_PER_FLOAT = 4;

	// Delete the previously stored path if available.
	delete[] _pathX;
	delete[] _pathY;

	unsigned long prevMillis = millis();
	uint8_t intRead;
	int bytesReadIteration = 0;
	int numbersRead = 0;
	uint8_t ary[BYTES_PER_FLOAT];
	_pathLength = 9999; // Get in the loop

	// Read the float values corresponding to the path data.
	while (numbersRead <= _pathLength * 2) {
		if (_bluetooth.available()) {

			intRead = _bluetooth.read();

			ary[bytesReadIteration] = intRead;
			bytesReadIteration++;

			if (bytesReadIteration == BYTES_PER_FLOAT) {
				// Convert the byte array to float array.
				float number = bytesToFloat(ary[0], ary[1], ary[2], ary[3]);
				numbersRead++;

				// The first path data transmitted is the length.
				if (numbersRead == 1) {
					_pathLength = (int)number;
					_pathX = new float[_pathLength];
					_pathY = new float[_pathLength];
					// Serial.print("Path length: "); Serial.println(_pathLength);
				}
				// Subsequent path data transmission are the coordinates.
				else {
					if (numbersRead % 2 == 0) {
						_pathX[numbersRead/2 - 1] = number;
						// Serial.print(number);
					}
					else {
						_pathY[numbersRead/2 - 1] = number;
						// Serial.print("\t"); Serial.println(number);
					}
					//Serial.print("Number Read: "); Serial.println(numberReadCount);
				}
				bytesReadIteration = 0;
			}
		}
		if (millis() - prevMillis > PATH_TRANSMISSION_TIMEOUT) {
			//Serial.println("Error: path transmission took too long.");
			//return false;
		}
	}

	return true;
}

// Stores short transmission into the signal array
void ArduinoBlue::storeShortTransmission() {
	unsigned long prevMillis = millis();
	uint8_t intRead;
	while (millis() - prevMillis < SHORT_TRANSMISSION_TIMEOUT) {
		if (_bluetooth.available()) {
			intRead = _bluetooth.read();
			if (intRead == TRANSMISSION_END) break;
			pushToSignalArray(intRead);
		}
	}
}

String ArduinoBlue::readString() {
    String s;
    uint8_t intRead;
    unsigned long prevTime = millis();

    // Read until newline character or timeout is reached
    prevTime = millis();
    while (millis() - prevTime < TEXT_TRANSMISSION_TIMEOUT) {
        if (_bluetooth.available()) {
            intRead = _bluetooth.read();
            // break the loop if end of transmission
            if (intRead == TRANSMISSION_END) break;
            s.concat((char)intRead);
        }
    }
    return s;
}

void ArduinoBlue::pushToSignalArray(uint8_t elem) {
    if (elem < 0) {
        Serial.print("neg");
    }
    if ( !(_signalLength + 1 == MAX_SHORT_SIGNAL_LENGTH) ) {
        _signal[_signalLength] = elem;
        _signalLength++;
    }
    else {
        Serial.println("ArduinoBlue: Transmission error...");
		Serial.print("elem: "); Serial.println(elem);
    }
}

void ArduinoBlue::clearSignalArray() {
    for (uint8_t i = 0; i < _signalLength; i++) {
        _signal[i] = DEFAULT_VALUE;
    }
    _signalLength = 0;
}

bool ArduinoBlue::isPathAvailable() {
	bool toReturn = _pathAvailable;
	_pathAvailable = false;
	return toReturn;
}

int ArduinoBlue::getButton() {
    checkBluetooth();
    uint8_t btn = _button;
    _button = DEFAULT_VALUE;
    if (btn == DEFAULT_VALUE) return -1;
    return btn;
}

int ArduinoBlue::getSliderId() {
    checkBluetooth();
    uint8_t id = _sliderId;
    _sliderId = DEFAULT_VALUE;
    if (id == DEFAULT_VALUE) return -1;
    return id;
}

int ArduinoBlue::getSliderVal() {
    uint8_t val = _sliderVal;
    _sliderVal = DEFAULT_VALUE;
    if (val == DEFAULT_VALUE) return -1;
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

float * ArduinoBlue::getPathArrayX() {
	return _pathX;
}

float * ArduinoBlue::getPathArrayY() {
	return _pathY;
}

int ArduinoBlue::getPathLength() {
	return _pathLength;
	//return tempPathLength;
}

void ArduinoBlue::sendText(String msg) {
    _bluetooth.print(((char)TEXT_SEND_TRANSMISSION) + msg + ((char)TRANSMISSION_END));
}

// for backwards compatibility
void ArduinoBlue::sendMessage(String msg) {
    sendText(msg);
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
