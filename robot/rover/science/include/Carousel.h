//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include <cstdint>

#include "Updatable.h"
#include "Servo.h"

enum CarouselState {
    Uncalibrated,
    Calibrating,
    Not_Moving,
    Moving_Carousel
};

// Forward declaration
class Carousel;


class Carousel : public Updatable {
 private:
  // min milliseconds to wait after seeing switch keeping a solid state
  // to register that it has actually changed
  static unsigned long const DEBOUNCE_THRESHOLD = 50;

  static const uint8_t stopped_speed = 90;
  // speeds used when going from one test tube to the other
  static const uint8_t ccw_speed = 110;
  static const uint8_t cw_speed = 70;
  // speeds used when spinning to mix sample with reagent
  static const uint8_t cw_spin_speed = 0;
  static const uint8_t ccw_spin_speed = 180;
  
  // For debounce
  unsigned long lastDebounceTime;
  int lastButtonState;
  int buttonState;
  
  // current cuvette, in the range of 0-5
  uint8_t currentCuvette;
  // holds if moving, calibrating, etc
  CarouselState state;

  // Keeps track of the number of times the limit switch has
  // transitioned to HIGH since current move has started
  int limitSwitchPulses;
  int cuvettesToMove;

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
  
  // sets up interrupt callback
  void setup();
  
  // Basic commands
  void startCalibrating();
  void goToCuvette(uint8_t cuvetteId);
  void moveNCuvettes(int cuvettesToMove);
  // speed is adjusted based on whether it's moving clockwise or counterclockwise 
  void moveNCuvettes(int cuvettesToMove, uint8_t cw_speed, uint8_t ccw_speed);
  void nextCuvette();
  void previousCuvette();
  void spinMix();
  
  // getters
  int8_t getCarouselIndex() const;
  CarouselState getState() const;
  bool isMoving() const;
  
  void startAutoTesting();
  virtual void update(unsigned long deltaMicroSeconds) override;
  virtual ~Carousel();
  Carousel();
};

#endif  // ROVER_CAROUSEL_H
