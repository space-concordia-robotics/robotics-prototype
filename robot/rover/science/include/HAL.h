// Hardware abstraction layer

#ifndef HAL_H
#define HAL_H

#include <Servo.h>
#include <etl/array.h>

#include "Arduino.h"
#include "include/SciencePinSetup.h"

/**
 * Typedef for the function that is called back when
 * a limit switch is hit. The int parameter is the state of
 * the pin.
 */
typedef void (*pinCallback)(int);
typedef void (*voidCallback)();

/**
 * Class that holds onto a switch callback and the switch ID it's
 * tied to. Inputs that could be received directly from serial are
 * uint8_t because that is one common type we're sending; else int is used
 * since it works a bit better with the Arduibo library.
 */
class SwitchCallback {
 public:
  int switchId;
  pinCallback cb;
  SwitchCallback(int switchId, pinCallback cb) : switchId(switchId), cb(cb){};
};

class HAL {
 public:
  /**
   * @brief Sets up pin modes.
   */
  static void pinSetup();
  /**
   * @brief Emergency stop.
   */
  static void estop();
  
  /**
   * @brief Moves the given pwm servo
   *
   * @param id from 0-3
   * @param angle Value from 0-180.
   */
  static void servo(uint8_t id, uint8_t angle);
  /**
   * Read the limit switch
   * @param switchId Id, from 0-1, of switch to read.
   * @return 1 if pressed, 0 if not pressed
   */
  static uint8_t readLimitSwitch(uint8_t switchId);
  /**
   * @brief Adds function that will be called when the limit switch changes
   * state.
   *
   * @param switchId From 0-3
   * @param callback int-taking function (the param is the pin state) to be
   * called back
   */
  static void addLimitSwitchCallback(int switchId, pinCallback callback);

  /**
   * @brief Adds function that will be called when there is a power problem
   * (for now, either HV or LV).
   *
   * @param callback Void function that will be called
   */
  static void addPowerCallback(voidCallback callback);

 private:
  static void handleSwitches(int switchId);
  static void handleSwitch0();
  static void handleSwitch1();
  static void handlePower();
  static SwitchCallback switchCallbacks[5];
  static int switchCallbacksSize;
  static voidCallback powerCallbacks[3];
  static int powerCallbacksSize;
  static etl::array<Servo*, NUM_SERVOS> servos;
};

#endif