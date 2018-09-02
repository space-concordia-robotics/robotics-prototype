#ifndef PARSER_H
#define PARSER_H

#include <Arduino.h>
#include "ArmMotor.h"

class Parser
{
public:
  int motor;
  int direction;
  int speed;
  unsigned int time;

  Parser();
  void parseCommand(String command);

  // private:
  // String command; // serial buffer used for early- and mid-stage tesing without ROSserial
  // int tempSpeedVar;
  // unsigned int tempTimeVar;
};

Parser::Parser(void)
{
}

void Parser::parseMotorCommand(String command)
{
  // Vector<String> parameters = command.split(" ");
  // Vector<String> parameters;
  for (msgElem) {
    msgElem = strtok(NULL, " ");
    parameters.add(msgElem);
  }

  for (int i = 0; i < parameters.length; i+=2)
  {                                       //strtok_r() splits message by a delimiter string
    char *msgElem = strtok(command, " "); // look for first element (first tag)
    if (String(msgElem) == "motor")
    {                              // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " "); // go to next msg element (motor number)
    }

    msgElem = strtok(NULL, " "); // find the next message element (direction tag)
    if (String(msgElem) == "direction")
    {                              // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " "); // go to next msg element (direction)
      switch (*msgElem)
      {         // determines motor direction
      case '0': // arbitrarily (for now) decided 0 is clockwise
        whichDir = CLOCKWISE;
        break;
      case '1': // arbitrarily (for now) decided 1 is counter-clockwise
        whichDir = COUNTER_CLOCKWISE;
        break;
      }
    }

    msgElem = strtok(NULL, " "); // find the next message element (speed tag)
    if (String(msgElem) == "speed")
    {                               // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " ");  // find the next message element (integer representing speed level)
      tempSpeedVar = atoi(msgElem); // converts to int
      if (tempSpeedVar <= MAX_SPEED)
      {                            // make sure the subtraction made sense. Tf it's above 9, it doesn't
        whichSpeed = tempSpeedVar; // set the actual speed
      }
    }

    msgElem = strtok(NULL, " "); // find the next message element (time tag)
    if (String(msgElem) == "time")
    {                              // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " "); // find the next message element (time in seconds)
      tempTimeVar = atoi(msgElem); // converts to int
      if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME)
      { // don't allow budge movements to last a long time
        whichTime = tempTimeVar;
      }
    }
  }
}

#endif
