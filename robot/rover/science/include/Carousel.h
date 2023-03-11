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
    Uncalibrated,
    Calibrating,
    Not_Moving,
    Moving_Carousel,
    Correcting_cw,
    Correcting_ccw
};

// Forward declaration
class Carousel;


class Carousel : public Updatable {
 private:
  unsigned long timeStopped = 0;

  // speed used when spinning to mix/calibration, 10s degrees/s
  // ie 100 means 10 degrees per second
  static const int16_t spin_speed = 100;
  static const int16_t calibration_speed = 50;

  LSSServoMotor servo;
  int servoId;
  
  // current cuvette, in the range of 0-5
  uint8_t currentCuvette;
  // holds if moving, calibrating, etc
  CarouselState state;

  // Used to correct a move, if the smart servo drifts
  bool limitSwitchHit;
  static unsigned long const MAX_CORRECT_TIME = 1000;
  
  // For automation
  enum AutomationState {
    SpinningCarousel,
    Advancing,
    NotAutomating
  };

  AutomationState automationState;

  void handleAutomation();
  bool isAutomating();

  // checks switch that toggles every time it moves past a carousel
  void checkSwitch();

 public:
  const static uint8_t NUM_CUVETTES = 6;
  const static uint16_t DEGREES_PER_CUVETTE = (int)((360.0 / NUM_CUVETTES) * 10); 
  
  // sets up interrupt callback
  void setup();
  
  // Basic commands
  void startCalibrating();
  void goToCuvette(uint8_t cuvetteId);
  void moveNCuvettes(int cuvettesToMove);
  void nextCuvette();
  void previousCuvette();
  void spinMix();

  bool queryServoStopped();
  
  // getters
  int8_t getCarouselIndex() const;
  CarouselState getState() const;
  bool isMoving() const;
  
  void startAutoTesting();
  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel(int servoId);
};

#endif  // ROVER_CAROUSEL_H
