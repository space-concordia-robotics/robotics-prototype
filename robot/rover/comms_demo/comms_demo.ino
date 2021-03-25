#include <Arduino.h>
#include "include/commands/DemoCommandCenter.h"
#include <cstdint>

const uint8_t TX_TEENSY_4_0_PIN = 1;
const uint8_t RX_TEENSY_4_0_PIN = 0;
const uint8_t ENABLE_PIN = 10; // THIS IS A PLACE HOLDER UNTIL FLOW CONTROL CAN BE IMPLEMENTED

internal_comms::CommandCenter* commandCenter = new DemoCommandCenter();

void loop()
{
    if(Serial.available() > 0)
        commandCenter->readCommand();

}

void setup()
{
    commandCenter->startSerial(TX_TEENSY_4_0_PIN, RX_TEENSY_4_0_PIN, ENABLE_PIN);
}
