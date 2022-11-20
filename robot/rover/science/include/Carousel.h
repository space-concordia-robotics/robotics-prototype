//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include <cstdint>

#include "Updatable.h"
#include "Servo.h"

// Forward declaration
class Carousel;


class Carousel : public Updatable {
 protected:
  // current cuvette, in the range of 0-7
  uint8_t currentCuvette;

 private:
  unsigned long timeCorrectionStarted = 0;
  float degreesCurrentMove = 0;
  // Timestamp when btn 0 was last pressed, for debounce
  unsigned long btn0LastPulse;
  // min milliseconds between button rising edges.
  // If button rising is detected faster than this,
  // assume it is bouncing 
  static unsigned long const DEBOUNCE_THRESHOLD = 100;
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
  State state;
  // Keeps track of the number of times
  // the limit switch has transitioned to HIGH
  int limitSwitchPulses;
  // Stores the number of cuvettes to move
  int cuvettesToMove;

 public:
  const static uint8_t NUM_CUVETTES = 8;
  // Required for the button interrupt
  static Carousel* instance;
  // sets up interrupt callbacks
  static void setup();

  void home();
  void startCalibrating();
  void goToCuvette(uint8_t cuvetteId);
  void moveNCuvettes(int cuvettesToMove);
  void nextCuvette();
  void previousCuvette();
  uint8_t getCurrentCuvette() const { return currentCuvette; }

  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel();
};

#endif  // ROVER_CAROUSEL_H
