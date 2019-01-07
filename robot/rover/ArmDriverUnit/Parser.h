
#ifndef PARSER_H
#define PARSER_H
#include "PinSetup.h"
#include "RobotMotor.h"
#define MOTOR_NOT_COMMANDED "~" // this character means a motor is not to be moved
// this struct is specific to the arm teensy at the moment.
// it should either be generalized or put in the main code,
// but it can't be put in the main code because commandInfo
// needs to be defined where it's used as an input parameter
struct commandInfo
{
  int whichMotor = 0; // which motor was requested to do something
  int whichDirection = 0; // set the direction
  int whichSpeed = 0; // set the speed
  unsigned int whichTime = 0; // how long to turn for
  bool angleCommand = false; // for regular operations, indicates that it's to control an angle
  float whichAngle = 0.0; // for regular operations, which angle to go to
  bool loopCommand = false; // for choosing between open loop or closed loop control
  int loopState = 0; // what type of loop state it is
  bool resetCommand = false; // indicates that something should be reset
  bool resetAngleValue = false; // mostly for debugging/testing, reset the angle variable
  bool resetJointPosition = false; // for moving a joint to its neutral position
  bool stopSingleMotor = false; // for stopping a single motor
  bool stopAllMotors = false; // for stopping all motors
  bool switchDir = false; // for switching the direction logic
  bool multiMove = false; // for controlling multiple motors simultaneously
  bool motorsToMove[NUM_MOTORS] =
  {
    false, false, false, false, false, false
  }; // which motor to move
  float anglesToReach[NUM_MOTORS] =
  {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  }; // motor angles
};

class Parser
{
  public:
    bool isValidNumber(String str, char type);
    void parseCommand(commandInfo & cmd, char * restOfMessage);
    bool verifCommand(commandInfo cmd);
};

bool Parser::isValidNumber(String str, char type)
{
  if (str.length() == 0)
    return false;
  for (unsigned int i = 0; i < str.length(); i++)
  {
    if (type == 'f')
    {
      if (!isDigit(str[i]) && !(str[i] == '.') && !(str[i] == '-'))
        return false;
    }
    if (type == 'd')
    {
      if (!isDigit(str[i]) && !(str[i] == '-'))
        return false;
    }
  }
  return true;
}

void Parser::parseCommand(commandInfo & cmd, char * restOfMessage)
{
  // check for emergency stop has precedence
  char * msgElem = strtok_r(restOfMessage, " ", & restOfMessage); // look for first element (first tag)
  if (String(msgElem) == "stop")
  {
    // msgElem is a char array so it's safer to convert to string first
    cmd.stopAllMotors = true;

  #ifdef DEBUG_PARSING
    UART_PORT.println("$S,Success: parsed emergency command to stop all motors");
  #endif

  }
  // check for simultaneous motor control
  else
    if (String(msgElem) == "move")
    {
      // msgElem is a char array so it's safer to convert to string first
      bool isValidCommand = true;
      int i = 0;
      do
      {
        msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (motor 1 angle)
        if (isValidNumber(String(msgElem), 'f'))
        {
          cmd.anglesToReach[i] = atof(msgElem); // update value in motor angles array
          cmd.motorsToMove[i] = true; // set bool in move motors bool array to true

  #ifdef DEBUG_PARSING
          UART_PORT.print("$S,Success: parsed motor ");
          UART_PORT.print(i + 1);
          UART_PORT.print(" desired angle ");
          UART_PORT.print(cmd.anglesToReach[i]);
          UART_PORT.println(" degrees");
  #endif

        }
        else
          if (String(msgElem) == MOTOR_NOT_COMMANDED)
          {
            // don't do anything to move motors bool array

  #ifdef DEBUG_PARSING
            UART_PORT.print("$S,Success: parsed motor ");
            UART_PORT.print(i + 1);
            UART_PORT.println(" to maintain old desired position");
  #endif

          }
        else
        {
          isValidCommand = false;

  #ifdef DEBUG_PARSING
          UART_PORT.print("$E,Error: motor ");
          UART_PORT.print(i + 1);
          UART_PORT.print(" angle ");
          UART_PORT.print(msgElem);
          UART_PORT.println(" is invalid");
  #endif

        }
        i++;
      }
      while (i < NUM_MOTORS); // this skips everything after the 6th val but i want it to not skip but i can't get it to work
      if (isValidCommand && i == NUM_MOTORS)
      {
        cmd.multiMove = true;
      }
      else
        if (i < NUM_MOTORS)
        {

  #ifdef DEBUG_PARSING
          UART_PORT.println("$E,Error: not enough input arguments for simultaneous motor control");
  #endif

        }
      else
        if (i > NUM_MOTORS)
          // this doesn't actually get checked right now
      {

  #ifdef DEBUG_PARSING
        UART_PORT.println("$E,Error: too many input arguments for simultaneous motor control");
  #endif

      }
      else
        if (!isValidCommand)
        {

  #ifdef DEBUG_PARSING
          UART_PORT.println("$E,Error: simultaneous motor control command not understood");
  #endif

        }
    }
  // check for motor command
  else
    if (String(msgElem) == "motor")
    {
      // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (motor number)
      // if ( isValidNumber(String(msgElem),'d') )
      cmd.whichMotor = atoi(msgElem);

  #ifdef DEBUG_PARSING
      UART_PORT.print("parsed motor ");
      UART_PORT.println(cmd.whichMotor);
  #endif

      // check for motor stop command has precedence
      msgElem = strtok_r(NULL, " ", & restOfMessage); // find the next message element (direction tag)
      if (String(msgElem) == "stop")
      {
        // msgElem is a char array so it's safer to convert to string first
        cmd.stopSingleMotor = true;

  #ifdef DEBUG_PARSING
        UART_PORT.println("$S,Success: parsed request to stop single motor");
  #endif

      }
      // check for angle command
      else
        if (String(msgElem) == "angle")
        {
          // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          if (isValidNumber(String(msgElem), 'f'))
          {
            cmd.angleCommand = true;
            cmd.whichAngle = atof(msgElem); // converts to float;

  #ifdef DEBUG_PARSING
            UART_PORT.print("$S,Success: parsed desired angle ");
            UART_PORT.println(cmd.whichAngle);
  #endif

          }
          else
          {

  #ifdef DEBUG_PARSING
            UART_PORT.println("$E,Error: desired angle is not a number");
  #endif

          }
        }
      // check for loop state command
      else
        if (String(msgElem) == "loop")
        {
          // msgElem is a char array so it's safer to convert to string first
          cmd.loopCommand = true;
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          if (String(msgElem) == "open")
          {
            cmd.loopState = OPEN_LOOP;

  #ifdef DEBUG_PARSING
            UART_PORT.print("$S,Success: parsed open loop state (");
            UART_PORT.print(cmd.loopState);
            UART_PORT.println(") request");
  #endif

          }
          else
            if (String(msgElem) == "closed")
            {
              cmd.loopState = CLOSED_LOOP;

  #ifdef DEBUG_PARSING
              UART_PORT.print("$S,Success: parsed closed loop state (");
              UART_PORT.print(cmd.loopState);
              UART_PORT.println(") request");
  #endif

            }
          else
          {

  #ifdef DEBUG_PARSING
            UART_PORT.println("$E,Error: unknown loop state");
  #endif

          }
        }
      // check for angle reset command
      else
        if (String(msgElem) == "reset")
        {
          // msgElem is a char array so it's safer to convert to string first
          cmd.resetCommand = true;
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          if (String(msgElem) == "angle")
          {
            cmd.resetAngleValue = true;

  #ifdef DEBUG_PARSING
            UART_PORT.println("$S,Success: parsed request to reset angle value");
  #endif

          }
          else
            if (String(msgElem) == "position")
            {
              cmd.resetJointPosition = true;

  #ifdef DEBUG_PARSING
              UART_PORT.println("$S,Sucess: parsed request to reset joint position");
  #endif

            }
          else
          {

  #ifdef DEBUG_PARSING
            UART_PORT.println("$E,Error: unknown reset request");
  #endif

          }
        }
      else
        if (String(msgElem) == "switch direction")
        {
          cmd.switchDir = true;

  #ifdef DEBUG_PARSING
          UART_PORT.println("$S,Success: parsed request to switch direction logic");
  #endif

        }
      else
      {

  #ifdef DEBUG_PARSING
        UART_PORT.print("$E,Error: unknown motor ");
        UART_PORT.print(cmd.whichMotor);
        UART_PORT.println(" command");
  #endif

      }
    }
  else
  {

  #ifdef DEBUG_PARSING
    UART_PORT.println("$E,Error: unknown motor command");
  #endif

  }
}

bool Parser::verifCommand(commandInfo cmd)
{
  if (cmd.stopAllMotors)
  {

  #ifdef DEBUG_VERIFYING
    UART_PORT.println("$S,Success: command to stop all motors verified");
  #endif

    return true;
  }
  else
    if (cmd.multiMove)
    {
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        if (cmd.motorsToMove[i])
        {
          if (cmd.anglesToReach[i] < MIN_JOINT_ANGLE || cmd.anglesToReach[i] > MAX_JOINT_ANGLE)
            {

  #ifdef DEBUG_VERIFYING
              UART_PORT.print("$E,Error: angle of ");
              UART_PORT.print(cmd.anglesToReach[i]);
              UART_PORT.print(" degrees invalid for motor ");
              UART_PORT.println(i+1);
  #endif

              return false;
            }
          else
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.print("$S,Success: angle of ");
            UART_PORT.print(cmd.anglesToReach[i]);
            UART_PORT.print(" degrees valid for motor ");
            UART_PORT.println(i+1);
  #endif

          }
        }
      }

  #ifdef DEBUG_VERIFYING
      UART_PORT.println("$S,Success: command to move all motors verified for angles: ");
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        if (cmd.motorsToMove[i])
        {
          UART_PORT.print(cmd.anglesToReach[i]);
        }
        else
        {
          UART_PORT.print(MOTOR_NOT_COMMANDED);
        }
        UART_PORT.print(" ");
      }
      UART_PORT.println("");
  #endif

      return true;
    }
  // 0 means there was an invalid command and therefore motors shouldn't be controlled
  else
    if (cmd.whichMotor > 0 && cmd.whichMotor <= RobotMotor::numMotors)
    {
      if (cmd.stopSingleMotor)
      {

  #ifdef DEBUG_VERIFYING
        UART_PORT.print("$S,Success: command to stop motor ");
        UART_PORT.print(cmd.whichMotor);
        UART_PORT.println(" verified");
  #endif

        return true;
      }
      else
        if (cmd.angleCommand)
        {
          if (cmd.whichAngle < MIN_JOINT_ANGLE || cmd.whichAngle > MAX_JOINT_ANGLE)
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.print("$E,Error: angle of ");
            UART_PORT.print(cmd.whichAngle);
            UART_PORT.print(" degrees invalid for motor ");
            UART_PORT.println(cmd.whichMotor);
  #endif

            return false;
          }
          else
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.print("$S,Success: command to move motor ");
            UART_PORT.print(cmd.whichMotor);
            UART_PORT.print(" ");
            UART_PORT.print(cmd.whichAngle);
            UART_PORT.println(" degrees verified");
  #endif

            return true;
          }
        }
      else
        if (cmd.loopCommand)
        {
          if (cmd.loopState == OPEN_LOOP || cmd.loopState == CLOSED_LOOP)
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.print("$S,Success: command to set motor ");
            UART_PORT.print(cmd.whichMotor);
            if (cmd.loopState == OPEN_LOOP)
              UART_PORT.println(" to open loop verified");
            if (cmd.loopState == CLOSED_LOOP)
              UART_PORT.println(" to closed loop verified");
  #endif

            return true;
          }
          else
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.println("$E,Error: invalid loop state");
  #endif

            return false;
          }
        }
      else
        if (cmd.resetCommand)
        {
          if (cmd.resetAngleValue || cmd.resetJointPosition)
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.print("$S,Success: command to reset motor ");
            UART_PORT.print(cmd.whichMotor);
            if (cmd.resetAngleValue)
              UART_PORT.println(" saved angle value verified");
            if (cmd.resetJointPosition)
              UART_PORT.println(" physical joint position verified");
  #endif

            return true;
          }
          else
          {

  #ifdef DEBUG_VERIFYING
            UART_PORT.println("$E,Error: invalid reset request");
  #endif

            return false;
          }
        }
      else
        if (cmd.switchDir)
        {

  #ifdef DEBUG_PARSING
          UART_PORT.print("$S,Success: command to switch motor ");
          UART_PORT.print(cmd.whichMotor);
          UART_PORT.println(" direction logic verified");
  #endif

          return true;
        }
      else

  #ifdef DEBUG_VERIFYING
      UART_PORT.print("$E,Error: command for motor ");
      UART_PORT.print(cmd.whichMotor);
      UART_PORT.println(" not recognized");
  #endif

      return false;
    }
  else
    if
  (cmd.whichMotor < 0 || cmd.whichMotor >= RobotMotor::numMotors)
  {

  #ifdef DEBUG_VERIFYING
    UART_PORT.println("$E,Error: requested motor index out of bounds");
  #endif

    return false;
  }
  else
  {

  #ifdef DEBUG_VERIFYING
    UART_PORT.println("$E,Error: command not recognized");
  #endif

    return false;
  }
}

#endif
