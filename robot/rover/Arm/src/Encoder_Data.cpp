#include "include/Encoder_Data.h"

/**
 * Zero initializes all the data
 */
EncoderData::EncoderData() {
  angle = 0.0;
  temperatureData = 0;
  CRC = 0;
}

void EncoderData::setData(unsigned char* tempData) {
  int rawAngle = tempData[2] + (tempData[3] << 8);
  angle = (rawAngle * 360) / 65536.0;
  int rawTemperature = tempData[4];
  temperatureData = (rawTemperature);
  CRC = tempData[5];
}

unsigned char EncoderData::getAddress(unsigned char receivedAddress) {
  unsigned char lower = receivedAddress & 15;
  unsigned char higher = ((~receivedAddress) & 240) >> 4;
  if (lower == higher) {
    return lower;
  } else {
    return 255;
  }
}