//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include <cstdint>

#include <Arduino.h>
#include "Updatable.h"
#include "Servo.h"
#include "../../smart_servo/include/LSSServoMotor.h"

enum CarouselState {
    Not_Moving,
    Moving_Carousel
};

// Forward declaration
class Carousel;


class Carousel : public Updatable {
 private:
  LSSServoMotor servo;
  int servoId;
  char queryBuffer[15]; // Buffer for queries sent to servo

  // Used to ensure that a break in communication does not cause a desync.
  // The idea is to send the servo absolute commands about where to go, but
  // if they are kept in the range 0-360 degrees, when crossing from 360-0 (test tube 5 to 0),
  // the servo goes the long way around. Keeping track of the virtual angle the servo stores (which
  // can go beyond 360 deg) prevents that. See https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HPositioninDegrees28D29
  int32_t virtualAngle;

  // Used as an offset relative to the absolute encoder on the servo
  int32_t angleOffset;
  
  // current test tube, in the range of 0-5
  int8_t currentTestTube;

  CarouselState state;
 

 public:
  const static int8_t NUM_TEST_TUBES = 6;
  const static uint16_t DEGREES_PER_TEST_TUBE = (int)((360.0 / NUM_TEST_TUBES) * 10); 
  
  // sets up interrupt callback
  void setup();
  
  // Basic commands
  void goToTestTube(uint8_t testTubeId);
  void moveNTestTubes(int testTubesToMove);
  void nextTestTube();
  void previousTestTube();
  void spinMix();

  void estop();
  void setServoAngle(float angle);

  bool queryServoStopped();
  
  // getters
  int8_t getCarouselIndex() const;
  CarouselState getState() const;
  bool isMoving() const;
  
  void startAutoTesting();
  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel(int servoId, int32_t angleOffset);
};

#endif  // ROVER_CAROUSEL_H
