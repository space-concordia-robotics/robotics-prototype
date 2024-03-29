// USB : Debug, UART : Production
#define USB

#include <Servo.h>

#include <Servo.cpp>

#include "Arduino.h"
#include "include/Carousel.h"
#include "include/SciencePinSetup.h"
#include "include/commands/ScienceCommandCenter.h"

#define NUMBER_OF_UPDATABLES 1;

const uint8_t TX_TEENSY_4_0_PIN = 1;
const uint8_t RX_TEENSY_4_0_PIN = 0;
const uint8_t ENABLE_PIN =
    10;  // THIS IS A PLACE HOLDER UNTIL FLOW CONTROL CAN BE IMPLEMENTED
const uint8_t TRANSMIT_PIN = 11;  // PLACE HOLDER

void updateSystems();

internal_comms::CommandCenter* commandCenter = new ScienceCommandCenter();

// TODO: set the servo id to a correct one
Carousel *carousel = new Carousel(0, 740);
Updatable* updatables[1] = {carousel};
unsigned long time = micros();

/* -----
This contains the methods needed by command center
  ------
*/
int8_t  carousel_get_carousel_index() { return carousel->getCarouselIndex(); }
bool carousel_get_moving() { return carousel->isMoving(); }
int32_t carousel_get_virtual() {return carousel->virtualAngle; }

void carousel_previous_test_tube() { carousel->moveNTestTubes(-1); }
void carousel_next_test_tube() { carousel->moveNTestTubes(1); }
void carousel_go_to_test_tube(uint8_t index) { carousel->goToTestTube(index); }
void carousel_spin_mix() {carousel->spinMix(); }

void carousel_estop() {carousel->estop(); }
void carousel_set_servo_angle(float angle) {carousel->setServoAngle(angle); }

void setup() {
  pinMode(LED, OUTPUT);
  SMART_SERVO_SERIAL.begin(115200);

  Serial.begin(9600);
  
  carousel->setup();
  commandCenter->startSerial(TX_TEENSY_4_0_PIN, RX_TEENSY_4_0_PIN, ENABLE_PIN,
                             TRANSMIT_PIN);
  // signal setup is done by toggling builtin LED.
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

void loop() {
  if (Serial.available() > 0) {
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