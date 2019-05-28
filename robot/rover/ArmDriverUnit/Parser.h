#ifndef PARSER_H
#define PARSER_H
#include "PinSetup.h"
#include "RobotMotor.h"
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

/*! \brief Checks if the string passed as input contains an integer or a float.
   @param[in] str     The input string to be checked
   @param[in] type    The type of number to check for. 'f' for float or 'd' for integer
*/
bool Parser::isValidNumber(String str, char type) {
  if (str.length() == 0) {
    return false;
  }
  for (unsigned int i = 0; i < str.length(); i++) {
    if (type == 'f') {
      if (!isDigit(str[i]) && !(str[i] == '.') && !(str[i] == '-')) {
        return false;
      }
    }
    if (type == 'd') {
      if (!isDigit(str[i]) && !(str[i] == '-')) {
        return false;
      }
    }
  }
  return true;
}

bool Parser::parseWord(char *msg, String cmdWord, bool & cmdBool) {
  if (String(msg) == cmdWord) {
    cmdBool = true;
#ifdef DEBUG_PARSING
    UART_PORT.print("ARM $S,Success: parsed ");
    UART_PORT.print(cmdWord);
    UART_PORT.println(" command");
#endif
    return true;
  }
  else {
    return false;
  }
}

bool Parser::parseKeyValue(char *msg, String cmdWord, float & cmdValue, bool & cmdBool, char *restMsg) {
  if (String(msg) == cmdWord) {
    msg = strtok_r(NULL, " ", &restMsg); // go to next msg element (arm speed multiplier)
    if (isValidNumber(String(msg), 'f')) {
      cmdValue = atof(msg);
      cmdBool = true;
#ifdef DEBUG_PARSING
      UART_PORT.print("ARM $S,Success: parsed ");
      UART_PORT.print(cmdWord);
      UART_PORT.print(" value of ");
      UART_PORT.println(cmdValue);
#endif
      return true;
    }
    else {
#ifdef DEBUG_PARSING
      UART_PORT.print("ARM $E,Error: parsed invalid ");
      UART_PORT.print(cmdWord);
      UART_PORT.println(" input");
#endif
      return false;
    }
  }
  else {
    return false;
  }
}

/*! \brief Goes through an array of characters, splits it up at each space
   character, then modifies the appropriate variables held in
   the commandInfo input parameter based on the message.
   @param[in] cmd           The commandInfo struct where the results go
   @param[in] restOfMessage A pointer to the array of characters to be parsed
*/
void Parser::parseCommand(commandInfo & cmd, char *restOfMessage) {
  // check for emergency stop has precedence
  char *msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // look for first element (first tag)
  if (parseWord(msgElem, "ping", cmd.pingCommand)) return;
  else if (parseWord(msgElem, "who", cmd.whoCommand)) return;
  else if (parseWord(msgElem, "stop", cmd.stopAllMotors)) return;
  else if (parseWord(msgElem, "reboot", cmd.rebootCommand)) return;
  else if (parseWord(msgElem, "reset", cmd.resetAllMotors)) return;
  else if (String(msgElem) == "home") {
    // msgElem is a char array so it's safer to convert to string first
    msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (both limits or just 1?)
    if (String(msgElem) == "double") {
      cmd.homingStyle = DOUBLE_ENDED_HOMING;
    }
    cmd.homeAllMotors = true; // regardless, home the motor if "home"
#ifdef DEBUG_PARSING
    UART_PORT.println("ARM $S,Success: parsed command to home all motor joints in mode ");
    UART_PORT.println(cmd.homingStyle);
#endif
  }
  else if (parseKeyValue(msgElem, "armspeed", cmd.armSpeedMultiplier, cmd.armSpeedCommand, restOfMessage)) return;
  // check for simultaneous motor control
  else if (String(msgElem) == "move" || String(msgElem) == "budge") {
    // msgElem is a char array so it's safer to convert to string first
    bool isValidCommand = false; bool isValidBudge = false; bool isValidMove = false;
    int i = 0;
    if (String(msgElem) == "budge") {
      isValidBudge = true;
      isValidCommand = true;
    }
    else {
      isValidMove = true;
      isValidCommand = true;
    }
    do {
      msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (motor 1 angle)
      if (isValidBudge) {
        if (String(msgElem) == "fwd") {
          cmd.directionsToMove[i] = COUNTER_CLOCKWISE;
          cmd.motorsToMove[i] = true; // set bool in move motors bool array to true
        }
        else if (String(msgElem) == "back") {
          cmd.directionsToMove[i] = CLOCKWISE;
          cmd.motorsToMove[i] = true; // set bool in move motors bool array to true
        }
        else if (String(msgElem) == MOTOR_NOT_COMMANDED) {
          // don't do anything to move motors bool array
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $S,Success: parsed motor ");
          UART_PORT.print(i + 1);
          UART_PORT.println(" to maintain old desired position");
#endif
        }
        else {
          isValidBudge = false;
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $E,Error: parsed invalid budge for  motor ");
          UART_PORT.println(i + 1);
#endif
          break;
        }
      }
      else if (isValidMove) {
        if (isValidNumber(String(msgElem), 'f')) {
          cmd.anglesToReach[i] = atof(msgElem); // update value in motor angles array
          cmd.motorsToMove[i] = true; // set bool in move motors bool array to true
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $S,Success: parsed angle of ");
          UART_PORT.print(cmd.anglesToReach[i]);
          UART_PORT.print(" degrees for motor ");
          UART_PORT.println(i + 1);
#endif
        }
        else if (String(msgElem) == MOTOR_NOT_COMMANDED) {
          // don't do anything to move motors bool array
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $S,Success: parsed motor ");
          UART_PORT.print(i + 1);
          UART_PORT.println(" to maintain old desired position");
#endif
        }
        else {
          isValidMove = false;
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $E,Error: parsed invalid angle of ");
          UART_PORT.print(msgElem);
          UART_PORT.print(" degrees for motor ");
          UART_PORT.println(i + 1);
#endif
          break;
        }
      }
      i++;
    }
    while (i < NUM_MOTORS); // this skips everything after the 6th val but i want it to not skip but i can't get it to work
    if (i == NUM_MOTORS) {
      if (isValidBudge) {
        cmd.budgeCommand = true;
      }
      else if (isValidMove) {
        cmd.multiMove = true;
      }
      else isValidCommand = false;
    }
    else if (i < NUM_MOTORS) {
#ifdef DEBUG_PARSING
      UART_PORT.println("ARM $E,Error: parsed insufficient input arguments for simultaneous motor control");
#endif
    }
    else if (i > NUM_MOTORS) { // this doesn't actually get checked right now
#ifdef DEBUG_PARSING
      UART_PORT.println("ARM $E,Error: parsed too many input arguments for simultaneous motor control");
#endif
    }
    else if (!isValidCommand) {
#ifdef DEBUG_PARSING
      UART_PORT.println("ARM $E,Error: parsed invalid simultaneous motor control command");
#endif
    }
  }
  else if (String(msgElem) == "motor") { // check for motor command
    // msgElem is a char array so it's safer to convert to string first
    msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (motor number)
    // if ( isValidNumber(String(msgElem),'d') )
    cmd.whichMotor = atoi(msgElem);
    if (cmd.whichMotor > 0 && cmd.whichMotor <= NUM_MOTORS) {
#ifdef DEBUG_PARSING
      UART_PORT.print("ARM $S,Success: parsed motor ");
      UART_PORT.println(cmd.whichMotor);
#endif
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (direction tag)
      if (parseWord(msgElem, "stop", cmd.stopSingleMotor)) return;
      else if (String(msgElem) == "home") { // check for homingcommand
        // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (both limits or just 1?)
        if (String(msgElem) == "double") {
          cmd.homingStyle = DOUBLE_ENDED_HOMING;
        }
        cmd.homeCommand = true; // regardless, home the motor if "home"
#ifdef DEBUG_PARSING
        UART_PORT.print("ARM $S,Success: parsed command to home motor ");
        UART_PORT.print(cmd.whichMotor);
        UART_PORT.print(" joint in mode ");
        UART_PORT.println(cmd.homingStyle);
#endif
      }
      else if (parseKeyValue(msgElem, "angle", cmd.anglesToReach[cmd.whichMotor - 1], cmd.motorsToMove[cmd.whichMotor - 1], restOfMessage)) {
        cmd.multiMove = true;
        return;
      }
      else if (parseKeyValue(msgElem, "gear", cmd.gearRatioVal, cmd.gearCommand, restOfMessage)) return;
      // the following has no bad value error checking!!!!
      else if (parseKeyValue(msgElem, "opengain", cmd.openLoopGain, cmd.openLoopGainCommand, restOfMessage)) return;
      else if (parseKeyValue(msgElem, "kp", cmd.kp, cmd.kpCommand, restOfMessage)) {
        cmd.pidCommand = true;
        return;
      }
      else if (parseKeyValue(msgElem, "ki", cmd.ki, cmd.kiCommand, restOfMessage)) {
        cmd.pidCommand = true;
        return;
      }
      else if (parseKeyValue(msgElem, "kd", cmd.kd, cmd.kdCommand, restOfMessage)) {
        cmd.pidCommand = true;
        return;
      }
      else if (parseKeyValue(msgElem, "speed", cmd.motorSpeed, cmd.motorSpeedCommand, restOfMessage)) return;
      else if (String(msgElem) == "loop") { // check for loop state command
        // msgElem is a char array so it's safer to convert to string first
        cmd.loopCommand = true;
        msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (desired angle value)
        if (String(msgElem) == "open") {
          cmd.loopState = OPEN_LOOP;
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $S,Success: parsed open loop state (");
          UART_PORT.print(cmd.loopState);
          UART_PORT.println(") request");
#endif
        }
        else if (String(msgElem) == "closed") {
          cmd.loopState = CLOSED_LOOP;
#ifdef DEBUG_PARSING
          UART_PORT.print("ARM $S,Success: parsed closed loop state (");
          UART_PORT.print(cmd.loopState);
          UART_PORT.println(") request");
#endif
        }
        else {
#ifdef DEBUG_PARSING
          UART_PORT.println("ARM $E,Error: parsed unknown loop state");
#endif
        }
      }
      else if (parseWord(msgElem, "reset", cmd.resetSingleMotor)) return;
      else if (parseWord(msgElem, "switchdirection", cmd.switchDir)) return;
      else {
#ifdef DEBUG_PARSING
        UART_PORT.print("ARM $E,Error: parsed unknown motor ");
        UART_PORT.print(cmd.whichMotor);
        UART_PORT.println(" command");
#endif
      }
    }
    else {
#ifdef DEBUG_PARSING
      UART_PORT.println("ARM $E,Error: parsed invalid motor number");
#endif
    }
  }
  else {
#ifdef DEBUG_PARSING
    UART_PORT.println("ARM $E,Error: parsed unknown motor command");
#endif
  }
}

/*! \brief Goes through the commandInfo input parameter and checks to
   make sure all the inputs are valid before allowing the main
   code to execute the command.
   @param[in] cmd The commandInfo struct to be verified
*/
bool Parser::verifCommand(commandInfo cmd) {
  if (cmd.pingCommand) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified ping command");
#endif
    return true;
  }
  else if (cmd.whoCommand) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified who command");
#endif
    return true;
  }
  else if (cmd.stopAllMotors) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified command to stop all motors");
#endif
    return true;
  }
  else if (cmd.rebootCommand) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified command to reboot arm teensy");
#endif
    return true;
  }
  else if (cmd.homeAllMotors) {
#ifdef DEBUG_VERIFYING
    UART_PORT.print("ARM $S,Success: verified command to home all motor joints in mode ");
    UART_PORT.println(cmd.homingStyle);
#endif
    return true;
  }
  else if (cmd.homeCommand) {
#ifdef DEBUG_VERIFYING
    UART_PORT.print("ARM $S,Success: verified command to home motor joint");
    UART_PORT.print(cmd.whichMotor);
    UART_PORT.print(" in mode ");
    UART_PORT.println(cmd.homingStyle);
#endif
    return true;
  }
  else if (cmd.resetAllMotors) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified command to reset all motor angle values");
#endif
    return true;
  }
  else if (cmd.armSpeedCommand) {
    if (cmd.armSpeedMultiplier < 0) {
#ifdef DEBUG_VERIFYING
      UART_PORT.println("ARM $E,Error: invalid arm speed multiplier");
#endif
      return false;
    }
    else {
#ifdef DEBUG_VERIFYING
      UART_PORT.println("ARM $S,Success: verified command to update arm speed multiplier");
#endif
      return true;
    }
  }
  else if (cmd.budgeCommand) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (cmd.motorsToMove[i]) {
        if (cmd.directionsToMove[i] == 0) {
          return false;
        }
      }
    }
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified command to budge all motors for directions: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (cmd.motorsToMove[i]) {
        UART_PORT.print(cmd.directionsToMove[i]);
      }
      else {
        UART_PORT.print(MOTOR_NOT_COMMANDED);
      }
      UART_PORT.print(" ");
    }
    UART_PORT.println("");
#endif
    return true;
  }
  else if (cmd.multiMove) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (cmd.motorsToMove[i]) {
        if (cmd.anglesToReach[i] < MIN_JOINT_ANGLE || cmd.anglesToReach[i] > MAX_JOINT_ANGLE) {
#ifdef DEBUG_VERIFYING
          UART_PORT.print("ARM $E,Error: invalid angle of ");
          UART_PORT.print(cmd.anglesToReach[i]);
          UART_PORT.print(" degrees for motor ");
          UART_PORT.println(i + 1);
#endif
          return false;
        }
        else {
#ifdef DEBUG_VERIFYING
          UART_PORT.print("ARM $S,Success: verified angle of ");
          UART_PORT.print(cmd.anglesToReach[i]);
          UART_PORT.print(" degrees for motor ");
          UART_PORT.println(i + 1);
#endif
        }
      }
    }
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $S,Success: verified command to move all motors for angles: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (cmd.motorsToMove[i]) {
        UART_PORT.print(cmd.anglesToReach[i]);
      }
      else {
        UART_PORT.print(MOTOR_NOT_COMMANDED);
      }
      UART_PORT.print(" ");
    }
    UART_PORT.println("");
#endif
    return true;
  }
  else if (cmd.whichMotor > 0 && cmd.whichMotor <= RobotMotor::numMotors) { // 0 means there was an invalid command and therefore motors shouldn't be controlled
    if (cmd.stopSingleMotor) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to stop motor ");
      UART_PORT.println(cmd.whichMotor);
#endif
      return true;
    }
    else if (cmd.gearCommand) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to set gear ratio to ");
      UART_PORT.println(cmd.gearRatioVal);
#endif
      return true;
    }
    else if (cmd.openLoopGainCommand) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to set open loop gain to ");
      UART_PORT.println(cmd.openLoopGain);
#endif
      return true;
    }
    else if (cmd.pidCommand) {
      if (cmd.kpCommand) {
#ifdef DEBUG_VERIFYING
        UART_PORT.print("ARM $S,Success: verified command to set ");
        UART_PORT.print("kp gain to ");
        UART_PORT.println(cmd.kp);
#endif
        return true;
      }
      else if (cmd.kiCommand) {
#ifdef DEBUG_VERIFYING
        UART_PORT.print("ARM $S,Success: verified command to set ");
        UART_PORT.print("ki gain to ");
        UART_PORT.println(cmd.ki);
#endif
        return true;
      }
      else if (cmd.kdCommand) {
#ifdef DEBUG_VERIFYING
        UART_PORT.print("ARM $S,Success: verified command to set ");
        UART_PORT.print("kd gain to ");
        UART_PORT.println(cmd.kd);
#endif
        return true;
      }
      else {
#ifdef DEBUG_VERIFYING
        UART_PORT.println("ARM $E,Error: not a command for kp, ki or kd");
#endif
        return false;
      }
    }
    else if (cmd.motorSpeedCommand) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to set motor speed to ");
      UART_PORT.println(cmd.motorSpeed);
#endif
      return true;
    }
    else if (cmd.loopCommand) {
      if (cmd.loopState == OPEN_LOOP || cmd.loopState == CLOSED_LOOP) {
#ifdef DEBUG_VERIFYING
        UART_PORT.print("ARM $S,Success: verified command to set motor ");
        UART_PORT.print(cmd.whichMotor);
        if (cmd.loopState == OPEN_LOOP) {
          UART_PORT.println(" to open loop");
        }
        if (cmd.loopState == CLOSED_LOOP) {
          UART_PORT.println(" to closed loop");
        }
#endif
        return true;
      }
      else {
#ifdef DEBUG_VERIFYING
        UART_PORT.println("ARM $E,Error: invalid loop state");
#endif
        return false;
      }
    }
    else if (cmd.resetSingleMotor) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to reset motor ");
      UART_PORT.print(cmd.whichMotor);
      UART_PORT.println(" saved angle value");
#endif
      return true;
    }
    else if (cmd.switchDir) {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $S,Success: verified command to switch motor ");
      UART_PORT.print(cmd.whichMotor);
      UART_PORT.println(" direction logic");
#endif
      return true;
    }
    else {
#ifdef DEBUG_VERIFYING
      UART_PORT.print("ARM $E,Error: invalid command for motor ");
      UART_PORT.println(cmd.whichMotor);
#endif
      return false;
    }
  }
  else if (cmd.whichMotor < 0 || cmd.whichMotor >= RobotMotor::numMotors) {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $E,Error: invalid motor index");
#endif
    return false;
  }
  else {
#ifdef DEBUG_VERIFYING
    UART_PORT.println("ARM $E,Error: invalid motor command");
#endif
    return false;
  }
}

#endif
