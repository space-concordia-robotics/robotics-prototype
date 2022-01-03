// USB : Debug, UART : Production
#define USB

#include <Arduino.h>
#include "include/commands/DemoCommandCenter.h"
#include <cstdint>

const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t ENABLE_PIN = 10; // This is the pin that the TX2 enables to say "you can send"
const uint8_t TRANSMIT_PIN = 11; // This is the pin that the teensy uses to flip its own RS-485 into transmit

internal_comms::CommandCenter* commandCenter = new DemoCommandCenter();

void setup()
{
    //Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, TRANSMIT_PIN);
}

void loop()
{ 
    if (Serial.available() > 0) {
        commandCenter->readCommand();
    } 

    commandCenter->sendMessage();
}
