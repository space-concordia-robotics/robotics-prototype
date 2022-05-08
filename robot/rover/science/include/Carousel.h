//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include <cstdint>

#include "../internal_comms/include/LSSServoMotor.h"
#include "Updatable.h"

class Carousel : public Updatable {
 protected:
  // current cuvette, in the range of 0-7
  uint8_t currentCuvette;

 private:
  int currentServoPosition();
  unsigned long timeCorrectionStarted = 0;
  float degreesCurrentMove = 0;
  /**
   * max amount of time to continue correcting a carousel move
   */
  static const unsigned long CORRECTION_MAX = 500;
  /**
   * stores the position the last time the motor was stopped.
   */
  int previousPositionTenths = 0;
  enum State {
    Uncalibrated,
    Calibrating,
    Not_Moving,
    Moving_Carousel,
    Correcting_Move_Pos,
    Correcting_Move_Neg
  };
  LSSServoMotor* theServo;
  uint8_t servoID;
  State state;

 public:
  void home();
  void startCalibrating();
  void goToCuvette(uint8_t cuvetteId);
  void moveNCuvettes(int cuvettesToMove);
  void moveByDegrees(float degrees);
  void nextCuvette();
  void previousCuvette();
  uint8_t getCurrentCuvette() const { return currentCuvette; }

  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel(LSSServoMotor* theServo, uint8_t servoID);
};

#endif  // ROVER_CAROUSEL_H
