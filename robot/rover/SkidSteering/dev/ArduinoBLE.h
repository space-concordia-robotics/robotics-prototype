/*
Name: ArduinoBlue.cpp
Created: 6/28/2017 11:00:39 AM
Author: Jae An
Contact: jaean37@gmail.com
*/

#ifndef _ArduinoBlue_h
#define _ArduinoBlue_h
#include <Arduino.h>

#define CONNECTION_CHECK 249
#define TRANSMISSION_END 250
#define DRIVE_TRANSMISSION 251
#define BUTTON_TRANSMISSION 252
#define SLIDER_TRANSMISSION 253
#define TEXT_TRANSMISSION 254
#define PATH_TRANSMISSION 255
#define NO_TRANSMISSION -1

#define TEXT_TRANSMISSION_TIMEOUT 5000 // ms
#define SHORT_TRANSMISSION_TIMEOUT 500

const int DEFAULT_STEERING = 49;
const int DEFAULT_THROTTLE = 49;
const int MAX_SHORT_SIGNAL_LENGTH = 3;

class ArduinoBlue
{
public:
    ArduinoBlue(Stream &output);
    int getButton();
    int getSliderId();
    int getSliderVal();
    int getThrottle();
    int getSteering();
    bool checkBluetooth();
    bool isConnected();
    void sendMessage(String msg);
    String getText();
private:
    Stream & _bluetooth;
    int _signal[MAX_SHORT_SIGNAL_LENGTH];
    int _signalLength = 0;
    int _throttle = DEFAULT_STEERING;
    int _steering = DEFAULT_THROTTLE;
    int _sliderVal = -1;
    int _sliderId = -1;
    int _button = -1;
    int _currentTransmission = NO_TRANSMISSION;
    String _text;
    void clearSignalArray();
    void pushToSignalArray(int elem);
    void storeShortTransmission();
    void processDriveTransmission();
    void processButtonTransmission();
    void processSliderTransmission();
    void processTextTransmission();
    void processPathTransmission();
    String readString();
};

#endif