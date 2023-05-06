// USB : Debug, UART : Production
#define USB

#include <SPI.h>
#include "Arduino.h"

#define IMAGE_SIZE 2048
uint16_t data[IMAGE_SIZE] = {0};
uint8_t marker[4] = {0xde, 0xad, 0xbe, 0xef};

void setup()
{
    Serial.begin(9600);
    SPI.begin();
}

void sendBuffer(uint16_t* buf, size_t len) {
    for (int i = 0; i < len; i++) {
        // Send big-endian (most significant first)
        Serial.write(data[i] >> 8);
        Serial.write(data[i] & 0xff);
    }
}

void loop()
{
    // wait on signal somehow?
    for (int i = 0; i < IMAGE_SIZE; i++) {
        data[i] = SPI.transfer16(0xFF);
    }

    // Send start bytes, payload, end bytes
    Serial.write(marker, 4);
    sendBuffer(data, IMAGE_SIZE);
    Serial.write(marker, 4);
}

