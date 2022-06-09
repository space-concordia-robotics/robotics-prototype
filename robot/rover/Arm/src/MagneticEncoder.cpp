#include "MagneticEncoder.h"

// Example usage  : Send 0xB0, get 0xBB
int TLE5012MagneticEncoder::checkAddress(uint8_t txAddress, uint8_t rxAddress) {

    uint8_t lower = txAddress & 0xF0;

    if(rxAddress == (lower | (lower >>4))){
        return SUCCESS;
    }
    return ADDRESS_FAIL;
}

int TLE5012MagneticEncoder::SPIWrite16(uint8_t reg, uint16_t val) {
    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    // Form the command from the format specified by the datasheet. The leading 0 signifies write
    uint16_t command = (0 << 15) | (reg << 4) | 0x1;

    // SPI transaction must start by pulling down the CS line
    digitalWrite(CS_PIN, LOW);

    // Address check (1 byte)
    uint8_t receivedAddress = SPI.transfer(this->address);

//    status = checkAddress(this->address, receivedAddress);
//    if(status != SUCCESS)
//        return ADDRESS_FAIL;

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
    return SUCCESS;
}

int TLE5012MagneticEncoder::SPIRead16(uint8_t reg, uint16_t *data, uint8_t size) {

    SPI.beginTransaction(TLE5012B_SPI_SETTINGS);

    // Form the command from the format specified by the datasheet. The leading 1 signifies read.
    uint16_t command = (1 << 15) | (reg << 4) | (size & 0x0F);

    // SPI transaction must start by pulling down the CS line
    digitalWrite(CS_PIN, LOW);

    // Address check (1 byte)
    uint8_t receivedAddress = SPI.transfer(this->address);
    Serial.write(receivedAddress);

    //status = checkAddress(this->address, receivedAddress);

    //    if(status != SUCCESS)
//        return ADDRESS_FAIL;

    // Send the command to the encoder
    SPI.transfer16(command);

    // Receive the desired amount of words. Since it's SPI, but we aren't transmitting anything, use a placeholder value.
//    uint16_t buffer[size];
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
    return SUCCESS;
}

TLE5012MagneticEncoder::TLE5012MagneticEncoder(uint8_t slaveAddress) : address(slaveAddress) {
    SPI.begin();
    pinMode(10,OUTPUT);
}

void TLE5012MagneticEncoder::computeAngle() {

    uint16_t x_raw;
    uint16_t y_raw;
    uint16_t t_raw;
    uint16_t t250;
    uint16_t tco_x_t;
    uint16_t tco_y_t;
    uint16_t x_offset;
    uint16_t y_offset;
    uint16_t synch;
    uint16_t ortho;
    uint16_t ang_base;

    SPIRead16(REG_ADC_X,&x_raw,1);
    SPIRead16(REG_ADC_Y,&y_raw,1);
    SPIRead16(REG_T_RAW,&t_raw,1);
    SPIRead16(REG_TCO_X,&tco_x_t,1);
    SPIRead16(REG_TCO_Y,&tco_y_t,1);
    SPIRead16(REG_X_OFFSET,&x_offset,1);
    SPIRead16(REG_Y_OFFSET,&y_offset,1);
    SPIRead16(REG_SYNCH,&synch,1);
    SPIRead16(REG_IFAB,&ortho,1);
    SPIRead16(REG_MOD3,&ang_base,1);
    SPIRead16(REG_T25O,&t250,1);

    uint16_t o_x = x_offset + ((tco_x_t * (t_raw - t250 - 439)) >> 10);
    uint16_t o_y = y_offset + ((tco_x_t * (t_raw - t250 - 439)) >> 10);

    uint16_t x_1 = x_raw - o_x;
    uint16_t y_1 = y_raw - o_y;

    uint16_t y_2 = (y_1 * synch + (16384)) >> 14;


}
