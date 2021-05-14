// USB : Debug, UART : Production
#define USB

#include <Arduino.h>
#include "include/commands/DemoCommandCenter.h"
#include <cstdint>

const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t ENABLE_PIN = 10; // THIS IS A PLACE HOLDER UNTIL FLOW CONTROL CAN BE IMPLEMENTED

internal_comms::CommandCenter* commandCenter = new DemoCommandCenter();

void setup()
{
    //Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, 1); // The 1 assumes that we are currently on the Arm teensy.
}

void loop()
{ 
    if (Serial.available() > 0) {
        commandCenter->readCommand();
    } 

    commandCenter->sendMessage();
}
