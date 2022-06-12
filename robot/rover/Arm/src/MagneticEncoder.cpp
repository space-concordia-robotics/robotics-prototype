#include "MagneticEncoder.h"

int checkAddress(uint8_t txAddress, uint8_t rxAddress) {

    uint8_t lower = txAddress & 0xF0;
    if(rxAddress == (lower | (lower >>4))){

        return OK;
    }
    return ADDRESS_FAIL;
}

int EncoderWrite16(uint8_t address, uint8_t reg, uint16_t val) {
    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    // Form the command from the format specified by the datasheet. The leading 0 signifies write
    uint16_t command = (0 << 15) | (reg << 4) | 0x1;

    // SPI transaction must start by pulling down the CS line
    digitalWrite(CS_PIN, LOW);

    // Address check (1 byte)
    uint8_t receivedAddress = SPI.transfer(address);

    int status = checkAddress(address, receivedAddress);

    if(status != OK)
        return status;

    // Send the command to the encoder
    SPI.transfer16(command);

    // Transmit the word, note that we do not care about what is returned
    SPI.transfer16(0x00);

    // Safety word (2 bytes) contains the CRC (last byte) and some status flags
    uint16_t safety = SPI.transfer16(0x00);

    // Terminate SPI transaction
    digitalWrite(CS_PIN,HIGH);

    SPI.endTransaction();
    // Must swap endianness to verify CRC
    uint16_t message[2] = {
            static_cast<uint16_t>((command >> 8) | (command << 8)),
            static_cast<uint16_t>((val >> 8) | (val << 8))
    };
    // Verify CRC, using the command and the data words
    uint8_t CRC = CRC8((uint8_t* )message,sizeof(message));

    // Compare the CRC byte from the safety word and the computed byte
    if(CRC != (safety & 0x00FF)){
        return CRC_FAIL;
    }
    return OK;
}

int EncoderRead16(uint8_t address, uint8_t reg, uint16_t *data, uint8_t size) {

    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    // Form the command from the format specified by the datasheet. The leading 1 signifies read.
    uint16_t command = (1 << 15) | (reg << 4) | (size & 0x0F);

    // SPI transaction must start by pulling down the CS line
    digitalWrite(CS_PIN, LOW);

    // Address check (1 byte)
    uint8_t receivedAddress = SPI.transfer(address);

//    Serial.println(receivedAddress,HEX);
//    Serial.println(address,HEX);
    if(checkAddress(address, receivedAddress) != OK){
        return ADDRESS_FAIL;
    }
    // Send the command to the encoder
    SPI.transfer16(command);

    // Receive the desired amount of words. Since it's SPI, but we aren't transmitting anything, use a placeholder value.
    for(int i = 0 ; i < size ; i++){
        data[i] = SPI.transfer16(0x00);
    }

    uint16_t safety = SPI.transfer16(0x00);

    // Terminate SPI transaction
    digitalWrite(CS_PIN,HIGH);

    SPI.endTransaction();

    // Must swap endianness to verify CRC
    uint16_t message[2] = {
            static_cast<uint16_t>((command >> 8) | (command << 8)),
            static_cast<uint16_t>((data[0] >> 8) | (data[0] << 8))
    };
    // Verify CRC, using the command and the data words
    uint8_t CRC = CRC8((uint8_t*)message,sizeof(message));

    // Compare the CRC byte from the safety word and the computed byte
    if(CRC != (safety & 0xFF)){
        return CRC_FAIL;
    }
    return OK;
}