#include "include/MagneticEncoder.h"

/**
 * Zero initializes all the data
 */
//EncoderData::EncoderData() {
//  angle = 0.0;
//  temperatureData = 0;
//  /CRC = 0;
//}

//void EncoderData::setData(unsigned char* tempData) {
//  int rawAngle = tempData[2] + (tempData[3] << 8);
//  angle = (rawAngle * 360) / 65536.0;
//  int rawTemperature = tempData[4];
//  temperatureData = (rawTemperature);
//  CRC = tempData[5];
//}

//unsigned char EncoderData::getAddress(unsigned char receivedAddress) {
//  unsigned char lower = receivedAddress & 15;
//  unsigned char higher = ((~receivedAddress) & 240) >> 4;
//  if (lower == higher) {
//    return lower;
//  } else {
//    return 255;
//  }
//}

void TLE5012MagneticEncoder::setData(unsigned char *tempData) {

}

void TLE5012MagneticEncoder::SPIWrite16(uint8_t reg, uint16_t val) {
    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    uint16_t command = (1 << 15) | (reg << 4) | 0x1;

    digitalWrite(CS_PIN, LOW);
    SPI.transfer16(command);


    SPI.transfer16(val);


    uint16_t safety = SPI.transfer16(0x00);

    uint8_t message[6];

    uint8_t CRC = CRC8(message,sizeof(message));

    Serial.write(safety);
    Serial.write(CRC);

    if(CRC != (uint8_t)(safety & 0x00FF)){
        // CRC issue
    }

    digitalWrite(CS_PIN,HIGH);

    SPI.endTransaction();
}

void TLE5012MagneticEncoder::SPIRead16(uint8_t reg, uint16_t *data, uint8_t size) {
    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    uint16_t command = (1 << 15) | (reg << 4) | (size & 0x0F);

    digitalWrite(CS_PIN, LOW);
    SPI.transfer16(command);

    for(int i = 0 ; i < size ; i++){

        uint16_t data_packet = SPI.transfer16(0x00);
        data[i] = data_packet;
    }

    uint16_t safety = SPI.transfer16(0x00);
    uint16_t message[2] = {
            static_cast<uint16_t>((command >>8) | (command << 8)),
            static_cast<uint16_t>((data[0] >> 8) | (data[0] <<8))
    };

    uint8_t CRC = CRC8((uint8_t* )message,sizeof(message));

    if(CRC != (safety & 0x00FF)){
        // CRC issue
    }
    digitalWrite(CS_PIN,HIGH);

    SPI.endTransaction();
}
