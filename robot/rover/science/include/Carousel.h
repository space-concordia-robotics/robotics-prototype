//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include <cstdint>

#include "../internal_comms/include/LSSServoMotor.h"
#include "Stoppable.h"
#include "Updatable.h"

class Carousel : public Stoppable, public Updatable {
 protected:
  // current cuvette, in the range of 0-7
  uint8_t currentCuvette;

 private:
  enum State { Uncalibrated, Calibrating, Operating };
  LSSServoMotor* theServo;
  uint8_t servoID;
  State state;

 public:
  void home();
  void startCalibrating();
  void goToCuvette(uint8_t cuvetteId);
  void moveByDegrees(float degrees);
  void nextCuvette();
  void previousCuvette();
  uint8_t getCurrentCuvette() const { return currentCuvette; }

  virtual void eStop() override;
  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel(LSSServoMotor* theServo, uint8_t servoID);
};

#endif  // ROVER_CAROUSEL_H
