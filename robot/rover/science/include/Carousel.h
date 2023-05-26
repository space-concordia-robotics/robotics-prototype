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
    Moving_Carousel,
    Spin_Mix_Positive,
    Spin_Mix_Negative,
    Negative_Correct
};

// Forward declaration
class Carousel;


class Carousel : public Updatable {
 private:
  LSSServoMotor servo;
  int servoId;
  char queryBuffer[15]; // Buffer for queries sent to servo

  // Used as an offset relative to the absolute encoder on the servo
  int32_t angleOffset;
  
  // current test tube, in the range of 0-5
  int8_t currentTestTube;

  CarouselState state;
 

 public:
  const static int8_t NUM_TEST_TUBES = 6;
  const static uint16_t DEGREES_PER_TEST_TUBE = (int)((360.0 / NUM_TEST_TUBES) * 10); 

  // current virtual angle of carousel  
  int32_t virtualAngle;

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
