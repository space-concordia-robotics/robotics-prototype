#include "RobotMotor.h"
int RobotMotor::numMotors = 0; // must initialize variable outside of class

RobotMotor::RobotMotor() {
  numMotors++;
  movementDone = true; // by default the movement has finished so the motors don't need to move
  hasAngleLimits = false; // only set to true when setAngleLimits() is called
  isOpenLoop = true; // by default don't use PID
  hasRamping = false; // by default don't ramp the speed
  imaginedAngle = 0.0;
  rotationDirection = 0; // by default invalid value
  directionModifier = 1; // this flips the direction sign if necessary;
  isBudging = false;
  hasRamping = false;
  hasLimitSwitches = false;
  homingType = SINGLE_ENDED_HOMING;
  homingDone = true;
  setMotorSpeed(30); // 30% speed by default;
  openLoopGain = 0.005; // temp open loop control
  homingPass = 0;
  atSafeAngle = true;
  startedZeroing = false;
  encoderModifier = 1;
}

void RobotMotor::attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes) // :
// encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
  hasEncoder = true;
  encoderPinA = encA;
  encoderPinB = encB;
  encoderPort = port;
  encoderShift = shift;
  encoderResolution = encRes;
  encoderResolutionReciprocal = 1 / (float) encRes;
}

void RobotMotor::attachLimitSwitches(char type, int switch1, int switch2) {
  hasLimitSwitches = true;
  limitSwitchType = type;
  if (type == FLEXION_SWITCH) {
    limSwitchFlex = switch1;
    limSwitchExtend = switch2;
  }
  else if (type == REVOLUTE_SWITCH) {
    limSwitchCw = switch1;
    limSwitchCcw = switch2;
  }
  else if (type == GRIPPER_SWITCH) {
    limSwitchOpen = switch1;
  }
}

void RobotMotor::setAngleLimits(float minH, float maxH, float minS, float maxS) {
  minJointAngle = minS;
  maxJointAngle = maxS;
  minHardAngle = minH;
  maxHardAngle = maxH;
  minSoftAngle = minS;
  maxSoftAngle = maxS;
  hasAngleLimits = true;
}

bool RobotMotor::withinJointAngleLimits(float angle) {
  if (!hasAngleLimits || (angle >= minJointAngle && angle <= maxJointAngle)) {
    return true;
  }
  else {
    return false;
  }
}

bool RobotMotor::setDesiredAngle(float angle) {
  //! \todo try to make everything go through this by putting conditions based on movement mode?
  if (withinJointAngleLimits(angle)) {
    desiredAngle = angle;
    return true;
  }
  else {
    return false;
  }
}

float RobotMotor::getDesiredAngle(void) {
  return desiredAngle;
}

int RobotMotor::calcDirection(float error) {
  if (error >= 0) {
    rotationDirection = directionModifier * COUNTER_CLOCKWISE;
  }
  else {
    rotationDirection = directionModifier * CLOCKWISE;
  }
  return rotationDirection;
}

void RobotMotor::switchDirectionLogic(void) {
  directionModifier = directionModifier * -1;
}

int RobotMotor::getDirectionLogic(void) {
  return directionModifier;
}

void RobotMotor::setGearRatio(float ratio) {
  gearRatio = ratio;
  gearRatioReciprocal = 1 / ratio;
}

void RobotMotor::setOpenLoopGain(float loopGain) {
  openLoopGain = loopGain;
}

void RobotMotor::setMotorSpeed(float motorSpeed) {
  openLoopSpeed = motorSpeed;
  pidController.setOutputLimits(-motorSpeed, motorSpeed);
}

float RobotMotor::getMotorSpeed(void) {
  if (isOpenLoop){
    return openLoopSpeed;
  }
  else {
    return pidController.getMaxOutputValue();
  }
}

//! \todo needs to be tested to make sure that a call to calcCurrentAngle will return the same thing as getCurrentAngle
void RobotMotor::setSoftwareAngle(float angle) {
  if (isOpenLoop) {
    imaginedAngle = angle;
  }
  else {
    currentAngle = angle;
    encoderCount = directionModifier * angle * gearRatio * encoderResolution / 360.0; // does adding directionModifier fix it?
  }
}

float RobotMotor::getSoftwareAngle(void) {
  if (isOpenLoop) {
    return imaginedAngle;
  }
  else {
    return currentAngle;
  }
}

void RobotMotor::checkForActualPress(void) {
  if (sinceTrigger >= TRIGGER_DELAY) {
    // if the last interrupt was a press (meaning it's stabilized and in contact)
    // then there's a real press
    if (triggerState != 0) {
      actualPress = true;
      limitSwitchState = triggerState;
    }
    // otherwise it's not a real press
    // so the limit switch state should stay whatever it used to be
    // and so should actualPress
    else {
      ;
    }
    // either way, we should reset the triggered bool in wait for the next trigger
    triggered = false;
  }
}

void RobotMotor::goToSafeAngle(void) {
#ifdef DEBUG_SWITCHES
  UART_PORT.print("ARM motor");
  UART_PORT.print(" limit switch state ");
  UART_PORT.println(limitSwitchState);
#endif
  stopRotation(); // stop turning of course
  // check which switch was hit, update the current angle as the max/min hardware angle,
  // then move the angle back to the max/min software angle
  if (limitSwitchState == COUNTER_CLOCKWISE) {
#ifdef DEBUG_SWITCHES
    UART_PORT.print("ARM motor is at hard angle ");
    UART_PORT.print(maxHardAngle);
    UART_PORT.print(" and turning cw to soft angle ");
    UART_PORT.println(maxSoftAngle);
#endif
    setSoftwareAngle(maxHardAngle);
    forceToAngle(maxSoftAngle);
  }
  if (limitSwitchState == CLOCKWISE) {
#ifdef DEBUG_SWITCHES
    UART_PORT.print("ARM motor is at hard angle ");
    UART_PORT.print(minHardAngle);
    UART_PORT.print(" and turning ccw to soft angle ");
    UART_PORT.println(minSoftAngle);
#endif
    setSoftwareAngle(minHardAngle);
    forceToAngle(minSoftAngle);
  }
  // now that the behaviour is complete we can reset these,
  // in wait for the next trigger to be confirmed
  actualPress = false;
  limitSwitchState = 0;
}

void RobotMotor::homeMotor(char homingDir) {
  homingPass++;
#ifdef DEBUG_HOMING
  UART_PORT.print("ARM homeMotor pass ");
  UART_PORT.println(homingPass);
#endif
  if (homingDir == 'i') { //(neg, cw)
#ifdef DEBUG_HOMING
    UART_PORT.println("ARM homeMotor inwards");
#endif
    // set homing direction inwards
    forceToAngle(1.5 * minHardAngle);
  }
  else if (homingDir == 'o') { //(pos, ccw)
#ifdef DEBUG_HOMING
    UART_PORT.println("ARM homeMotor outwards");
#endif
    forceToAngle(5 * maxHardAngle);
  }
  homingDone = false;
}

void RobotMotor::stopHoming(void) {
  homingDone = true;
  homingPass = 0;
  atSafeAngle = true;
  startedZeroing = false;
}
