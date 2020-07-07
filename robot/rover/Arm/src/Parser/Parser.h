#ifndef PARSER_H
#define PARSER_H
#include "../PinSetup/PinSetup.h"
#include "../RobotMotor/RobotMotor.h"
#define MOTOR_NOT_COMMANDED "~" //!< this character means a motor is not to be moved

// initially I wanted this to be generic to all the teensies,
// but the types of commands may be so different that it's
// extra overhead. to be rethought later...

//! This struct holds all the data about a specific command sent to it from the Odroid.
struct commandInfo {
  // commands that apply to the teensy in general
  bool pingCommand = false; //!< for ping command
  bool whoCommand = false; //!< for asking which teensy it is
  bool stopAllMotors = false; //!< for stopping all motors
  bool rebootCommand = false; //!< reboots teensy with watchdog
  bool resetAllMotors = false; //!< for resetting all angle values
  bool armSpeedCommand = false; //!< for adjusting overall arm speed
  float armSpeedMultiplier = 1.0; //!< what speed multiplier the arm should have
  bool homeAllMotors = false; //!< to search for limit switches and obtain absolute position
  int homingStyle = SINGLE_ENDED_HOMING; //!< by default only home to one limit switch

  // commands that apply to a specific motor
  int whichMotor = 0; //!< which motor was requested to do something

  bool stopSingleMotor = false; //!< for stopping a single motor
  bool resetSingleMotor = false; //!< mostly for debugging/testing, reset the angle variable
  bool switchDir = false; //!< for switching the direction logic

  bool homeCommand = false; //!< for homing a single motor
  bool loopCommand = false; //!< for choosing between open loop or closed loop control
  int loopState = 0; //!< what type of loop state it is

  bool gearCommand = false; //!< to change gear ratio
  float gearRatioVal = 0.0; //!< what's the new gear ratio
  bool openLoopGainCommand = false; //!< to change open loop gain
  float openLoopGain = 0.0; //!< adjusts how long to turn a motor in open loop
  bool pidCommand = false; //!< adjusts pid constants
  bool kpCommand = false; //!< adjusts p constant
  bool kiCommand = false; //!< adjusts i constant
  bool kdCommand = false; //!< adjusts d constant
  float kp = 0.0; //<! pid proportional gain
  float ki = 0.0; //<! pid proportional gain
  float kd = 0.0; //<! pid proportional gain
  bool motorSpeedCommand = false; //!< for changing motor speed
  float motorSpeed = 0.0; //!< what motor speed to set to

  bool budgeCommand = false; //!< for turning without angle requests
  int directionsToMove[NUM_MOTORS] = {0, 0, 0, 0, 0, 0}; //!< direction of budge control
  bool multiMove = false; //!< for controlling multiple motors simultaneously
  bool motorsToMove[NUM_MOTORS] = {false, false, false, false, false, false}; //!< which motor to move
  float anglesToReach[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //!< motor angles
};

/*! \brief This class is used to parse incoming messages to the arm teensy.

   It can break up any message passed to it using parseCommand()
   and place the information in a commandInfo struct. It can then
   can verify the commandInfo to make sure all the parameters
   are correct. It also holds a helper function to make sure
   that commands expecting numbers actually receive numbers
   rather than letters or some other type of characters.

   \todo Clean up parsing code to make it easier to read and reduce repetition.
   \todo set motor software angle (not just 0 but other values)
   \todo stop multiple motors (multimove style)
*/
class Parser {
  public:
    bool isValidNumber(String str, char type);
    bool parseWord(char *msg, String cmdWord, bool & cmdBool);
    bool parseKeyValue(char *msg, String cmdWord, float & cmdValue, bool & cmdBool, char *restMsg);
    void parseCommand(commandInfo & cmd, char *restOfMessage);
    //void parseRosCommand(commandInfo & cmd, String message);
    bool verifCommand(commandInfo cmd);
};



#endif
