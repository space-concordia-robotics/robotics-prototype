#include <Arduino.h>
#include "include/commands/DemoCommandCenter.h"
#include <cstdint>

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
