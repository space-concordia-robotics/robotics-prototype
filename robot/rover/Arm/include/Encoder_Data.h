#ifndef ENCDATA
#define ENCDATA

class EncoderData {
 public:
  float angle;
  unsigned char temperatureData;
  unsigned char CRC;

  EncoderData();
  /**
   * setting the data in the object from the byte array
   * (assumes that byte 0 is the preamble, and bytes
   * following that are :
   * address
   * angle (2 bytes, LSbyte first)
   * temp data
   * CRC
   * If the data is invalid, returns without setting.
   */
  void setData(unsigned char* tempData);
  /**
   * Returns the address from the received data if it is valid
   * (with respect to the complement) or 255 otherwise
   */
  static unsigned char getAddress(unsigned char receivedAddress);
};
#endif