// USB : Debug, UART : Production
#define UART

#include <include/Fan.h>
#include <include/Funnel.h>

#include "Arduino.h"
//#include "CommandCenter.h"
#include "include/Carousel.h"
#include "include/HAL.h"
#include "include/RamanSetup.h"
#include "include/SciencePinSetup.h"
#include "include/commands/ScienceCommandCenter.h"

const uint8_t NUMBER_OF_UPDATABLES = 3;

const uint8_t TX_TEENSY_4_0_PIN = 1;
const uint8_t RX_TEENSY_4_0_PIN = 0;
const uint8_t ENABLE_PIN =
    10;  // THIS IS A PLACE HOLDER UNTIL FLOW CONTROL CAN BE IMPLEMENTED
const uint8_t TRANSMIT_PIN = 11;  // PLACE HOLDER

void updateSystems();

Carousel* carousel = new Carousel(HAL::smartServo(), CAROUSEL_MOTOR_ID);
Fan* fan = new Fan();
Funnel* funnel = new Funnel();

internal_comms::CommandCenter* commandCenter = new ScienceCommandCenter();

Updatable* updatables[NUMBER_OF_UPDATABLES] = {carousel, fan, funnel};
unsigned long time = micros();

/* -----
This contains the methods needed by command center
  ------
  */
void carousel_move_degrees(float degrees) { carousel->moveByDegrees(degrees); }
void carousel_previous_cuvette() { carousel->previousCuvette(); }
void carousel_next_cuvette() { carousel->nextCuvette(); }

void setup() {
  HAL::pinSetup();
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  //  carousel->startCalibrating();
  commandCenter->startSerial(TX_TEENSY_4_0_PIN, RX_TEENSY_4_0_PIN, ENABLE_PIN,
                             TRANSMIT_PIN);
}

void loop() {
  if (Serial1.available() > 0) {
    commandCenter->readCommand();
  }

  updateSystems();
}

/**
 * Calculates delta for each system and calls their update method.
 * The delta calculation is inaccurate due to the fact that the delta
 * is not exact for each system. An improvement would be to have a separate dt
 * for each system.
 */
void updateSystems() {
  for (auto& updatable : updatables) {
    unsigned long delta = micros() - time;
    updatable->update(delta);
  }

  time = micros();
}