#ifndef ABTINENCODER_H
#define ABTINENCODER_H

#include "PinSetup.h"

const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix

volatile int m1EncoderCount = 0;
volatile int m2EncoderCount = 0;
volatile int m3EncoderCount = 0;
volatile int m4EncoderCount = 0;

/*
  void m1_encoder_interrupt() {
  static unsigned int oldM1EncoderState = 0;
  oldM1EncoderState <<= 2;  //move by two bits (multiply by 4);
  //read all bits on D register. shift ro right
  //so pin 2 and 3 are now the lowest bits
  //then AND this with 0X03 (0000 0011) to zero everything else
  //then OR this with the last encoder state to get a 4 byte number
  oldM1EncoderState |= ((PIND >> 2) & 0x03);
  //AND this number with 0X0F to make sure its a
  //4 bit unsigned number (0 to 15 decimal)
  //then use that number as an index from the array to add or deduct a 1 from the
  //count
  m1EncoderCount += dir[(oldM1EncoderState & 0x0F)];
  }
*/

/*
volatile int encoderCount;
int encoderPort = M2_ENCODER_PORT;
int encoderShift = M2_ENCODER_SHIFT;

void encoder_interrupt() {
  static unsigned int oldEncoderState = 0b1011; // solves bug where the encoder counts backwards for one count
  Serial.println(encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((encoderPort >> encoderShift) & 0x03);
  encoderCount += dir[(oldEncoderState & 0x0F)];
}
*/

void m2_encoder_interrupt() {
  static unsigned int oldM2EncoderState = 0b1011; // solves bug where the encoder counts backwards for one count
  //Serial.println(M2_ENCODER_PORT,BIN);
  Serial.println(m2EncoderCount);
  oldM2EncoderState <<= 2;
  oldM2EncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  //Serial.println((oldM2EncoderState&0x0F),BIN);
  m2EncoderCount += dir[(oldM2EncoderState & 0x0F)];
}

/*
  void m3_encoder_interrupt() {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((PIND >> 2) & 0x03);
  m3EncoderCount += dir[(oldEncoderState & 0x0F)];
  }

  void m4_encoder_interrupt() {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((PIND >> 2) & 0x03);
  m4EncoderCount += dir[(oldEncoderState & 0x0F)];
  }
*/

#endif
