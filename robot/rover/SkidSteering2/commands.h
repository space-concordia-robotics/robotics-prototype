#ifndef COMMANDS_H
#define COMMANDS_H

//
// Created by Fauzi Ameen on 2019-05-21
// Bluetooth removed to simplify code and reduce sources of error by Josh Glazer on 2019-05-30.
//

#include "motors.h"
#include "helpers.h"

class Commands {
  public:
    String activate_cmd = "activate";
    String deactivate_cmd = "deactivate";
    String ping_cmd = "ping";
    String who_cmd = "who";
    String reboot_cmd = "reboot";
    String closeLoop_cmd = "close-loop";
    String openLoop_cmd = "open-loop";
    String steerOff_cmd = "steer-off";
    String steerOn_cmd = "steer-on";
    String gpsOff_cmd = "gps-off";
    String gpsOn_cmd = "gps-on";
    String encOn_cmd = "enc-on";
    String encOff_cmd = "enc-off";
    String accOn_cmd = "acc-on";
    String accOff_cmd = "acc-off";
    String status_cmd = "status";
    String stop_cmd = "stop";

    bool isActivated = false;
    bool isOpenLoop = true; // No PID controller
    bool isSteering = true;
    bool isGpsImu = true;
    bool isEnc = true;

    bool imuError = false;
    String imuErrorMsg = "ASTRO IMU may be malfunctioning, timeout reached";
    bool gpsError = false;
    String gpsErrorMsg = "ASTRO GPS could not be initialized";

    int prevThrottle = 0;
    int prevSteering = 0;

    void setupMessage(void);
    void handler(String cmd, String sender);
    void systemStatus(void);

    void activate(String sender);
    void deactivate(String sender);
    void ping(void);
    void who(void);
    void rebootTeensy(void); //!< reboots the teensy using the watchdog timer
    void closeLoop(void);
    void openLoop(void);
    void steerOff(void);
    void steerOn(void);
    void gpsOff(void);
    void gpsOn(void);
    void encOn(void);
    void encOff(void);
    void accOn(void);
    void accOff(void);
    void status(void);
    void stop(void);
    void controlWheelMotors(String cmd);
    void controlCameraMotors(String cmd);
};

void Commands::setupMessage(void) {
  UART_PORT.println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
  UART_PORT.println("ASTRO setup complete");
  systemStatus();
}

void Commands::handler(String cmd, String sender) {
  UART_PORT.println("ASTRO GOT: " + cmd);
  if (cmd == activate_cmd) {
    activate(sender);
  }
  else if (cmd == deactivate_cmd) {
    deactivate(sender);
  }
  else if (cmd == ping_cmd) {
    ping();
  }
  else if (cmd == who_cmd) {
    who();
  }
  else if (cmd == reboot_cmd) {
    rebootTeensy();
  }
  else if (cmd == closeLoop_cmd) {
    closeLoop();
  }
  else if (cmd == openLoop_cmd) {
    openLoop();
  }
  else if (cmd == steerOff_cmd) {
    steerOff();
  }
  else if (cmd == steerOn_cmd) {
    steerOn();
  }
  else if (cmd == gpsOff_cmd) {
    gpsOff();
  }
  else if (cmd == gpsOn_cmd) {
    gpsOn();
  }
  else if (cmd == encOn_cmd) {
    encOn();
  }
  else if (cmd == encOff_cmd) {
    encOff();
  }
  else if (cmd == accOn_cmd) {
    accOn();
  }
  else if (cmd == accOff_cmd) {
    accOff();
  }
  else if (cmd == stop_cmd) {
    stop();
  }
  else if (cmd == status_cmd) {
    systemStatus();
  }
  // this is not safe enough, needs much stricter verification
  else if (cmd.indexOf(":") > 0) {
    controlWheelMotors(cmd);
  }
  else if (cmd[0] == '!' || cmd[0] == '@' || cmd[0] == '#' || cmd[0] == '$') {
    controlCameraMotors (cmd);
  }
  else {
    UART_PORT.println("ASTRO BOOOOOOO!!! Wrong command, try a different one, BIAAAAAA");
  }
}

void Commands::systemStatus(void) {
  UART_PORT.println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
  UART_PORT.println("ASTRO Astro has " + String(RobotMotor::numMotors) + " motors");
  UART_PORT.println("ASTRO Wheels: " + String(isActivated ? "ACTIVE" : "INACTIVE"));
  UART_PORT.println("ASTRO Steering: " + String(isSteering ? "Steering Control" : "Motor Control"));
  UART_PORT.println("ASTRO Encoders: " + String(isEnc ? "ON" : "OFF"));
  UART_PORT.println("ASTRO GPS " + String(gpsError ? "ERROR: " + gpsErrorMsg : "Success"));
  UART_PORT.println("ASTRO IMU " + String(imuError ? "ERROR: " + imuErrorMsg : "Success"));
  UART_PORT.println("ASTRO Nav Stream: " + String(isGpsImu ? "ON" : "OFF"));
  UART_PORT.print("ASTRO Motor loop statuses: ");
  for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
    UART_PORT.print(String(motorList[i].isOpenLoop ? "Open" : "CLose"));
    if (i != RobotMotor::numMotors - 1) UART_PORT.print(", ");
  }
  UART_PORT.println("");

  UART_PORT.print("ASTRO Motor accel: ");
  for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
    UART_PORT.print((motorList[i].accLimit) ? "ON" : "OFF");
    if (i != RobotMotor::numMotors - 1) UART_PORT.print(", ");
  }
  UART_PORT.println("");

  UART_PORT.println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

void Commands::activate(String sender) {
  if (isActivated) {
    UART_PORT.println("ASTRO Rover is Already ACTIVATED");
  }
  else {
    isActivated = true;
    UART_PORT.println("ASTRO Rover Wheels are Active");
  }
}
void Commands::deactivate(String sender) {
  if (isActivated) {
    stop();
  }
  isActivated = false;
  UART_PORT.println("ASTRO Rover Wheels are Inactive");
}

void Commands::ping(void) {
  UART_PORT.println("ASTRO pong");
}
void Commands::who(void) {
  if (isActivated) {
    UART_PORT.println("ASTRO Happy Astro");
  }
  else {
    UART_PORT.println("ASTRO Paralyzed Astro");
  }
}

void Commands::rebootTeensy(void) {
  UART_PORT.println("ASTRO rebooting wheel teensy... hang on a sec");
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  // The next 2 lines set the time-out value.
  WDOG_TOVALL = 15; // This is the value (ms) that the watchdog timer compare itself to.
  WDOG_TOVALH = 0; // End value (ms) WDT compares itself to.
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
  WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
  while (true); // infinite do nothing loop -- wait for the countdown
}

void Commands::closeLoop(void) {
  if (isActivated) {
    stop();
  }
  UART_PORT.println("ASTRO Turning encoders on first...");
  encOn();
  maxOutputSignal = MAX_RPM_VALUE; minOutputSignal = MIN_RPM_VALUE;
  UART_PORT.println("ASTRO Bo!");
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].isOpenLoop = false;
    String msg = "ASTRO Motor " + String(i + 1);
    msg += String(" loop status is: ");
    msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
    UART_PORT.println(msg);
  }
}
void Commands::openLoop(void) {
  if (isActivated) {
    stop();
  }
  maxOutputSignal = MAX_PWM_VALUE; minOutputSignal = MIN_PWM_VALUE;
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].isOpenLoop = true;
    String msg = "ASTRO Motor " + String(i + 1);
    msg += String(" loop status is: ");
    msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
    UART_PORT.println(msg);
  }
}

void Commands::steerOff(void) {
  if (isActivated ) {
    stop();
  }
  isSteering = false;
  UART_PORT.println("ASTRO Individual Wheel Control is Activated");
  if (isOpenLoop) {
    UART_PORT.println("ASTRO Speed range is -255 to 255 (open loop)");
    UART_PORT.println("ASTRO commands are now: motorNumber:PWM");
  }
  else {
    UART_PORT.println("ASTRO Speed range is -30 to 30 (closed loop)");
    UART_PORT.println("ASTRO commands are now: motorNumber:RPM");
  }
}
void Commands::steerOn(void) {
  if (isActivated) {
    stop();
  }
  isSteering = true;
  UART_PORT.println("ASTRO Skid Steering is Activated");
  if (isOpenLoop) {
    UART_PORT.println("ASTRO Throttle and Steering ranges are -49 to 49 (open loop)");
    UART_PORT.println("ASTRO commands are now: throttle:steering");
  }
  else {
    UART_PORT.println("ASTRO Throttle range is -30 to 30 (closed loop)");
    UART_PORT.println("ASTRO Steering range is -49 to 49");
    UART_PORT.println("ASTRO commands are now: RPM:steering");
  }
}

void Commands::gpsOff(void) {
  isGpsImu = false;
  UART_PORT.println("ASTRO GPS and IMU Serial Stream is now Disabled");
}
void Commands::gpsOn(void) {
  isGpsImu = true;
  UART_PORT.println("ASTRO GPS and IMU Serial Stream is now Enabled");
}

void Commands::encOn(void) {
  if (isActivated) {
    isEnc = true;
    UART_PORT.println("ASTRO Velocity Readings Stream from Motor Encoders is ON");
  }
  else if (!isActivated) {
    isEnc = true;
    UART_PORT.println("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated");
    for (i = 0; i < RobotMotor::numMotors; i++) {
      UART_PORT.print("ASTRO Motor ");
      UART_PORT.print(i);
      UART_PORT.print(" current velocity: ");
      UART_PORT.println(motorList[i].getCurrentVelocity());
      //delay(80);
    }
  }
}
void Commands::encOff(void) {
  isEnc = false;
  UART_PORT.println("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
}

void Commands::accOn(void) {
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].accLimit = true;
    String msg = "ASTRO Motor " + String(i + 1);
    msg += " Acceleration Limiter: ";
    msg += String(motorList[i].accLimit ? "Open" : "CLose");
    UART_PORT.println(msg);
  }
}
void Commands::accOff(void) {
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].accLimit = false;
    String msg = "ASTRO Motor " + String(i + 1);
    msg += " Acceleration Limiter: ";
    msg += String(motorList[i].accLimit ? "Open" : "CLose");
    UART_PORT.println(msg);
  }
}

void Commands::controlWheelMotors(String cmd) {
  if (!isActivated) {
    UART_PORT.println("ASTRO Astro isn't activated yet!!!");
  }
  else {
    if (isSteering) {
      throttle = getValue(cmd, ':', 0).toFloat();
      steering = getValue(cmd, ':', 1).toFloat();
      UART_PORT.println("ASTRO Throttle: " + String(throttle) + String(" -- Steering: ") + String(steering));
      //        if (!isOpenLoop){
      //            throttle = map(throttle, -30, 30, -49, 49);
      //            UART_PORT.println("ASTRO Desired Speed: " + String(throttle) + String("RPM ") + String(" Steering: ") + String(steering));
      //        }
      velocityHandler(throttle, steering);
      String msg = "ASTRO left: " + String(desiredVelocityLeft);
      msg += " -- right: " + String(desiredVelocityRight);
      msg += " maxOutput: " + String(MAX_PWM_VALUE);
      UART_PORT.println(msg);
    }
    else {
      motorNumber = getValue(cmd, ':', 0).toFloat();
      int motorSpeed = getValue(cmd, ':', 1).toFloat();
      steering = 0;
      int dir = 1;
      if (throttle < 0 ) {
        dir = - 1;
      }
      motorList[motorNumber].calcCurrentVelocity();
      motorList[motorNumber].setVelocity(dir , abs(motorSpeed), motorList[motorNumber].getCurrentVelocity());
      UART_PORT.println("ASTRO " + String(motorList[motorNumber].motorName) + String("'s desired speed: ") + String(motorList[motorNumber].getDesiredVelocity()) + String(" PWM ") + String(motorList[motorNumber].direction));
    }
  }
}

//! !FB, @FS, #RB, $RS
void Commands::controlCameraMotors(String cmd) {
  if (!isActivated) {
    UART_PORT.println("ASTRO Astro isn't activated yet!!!");
  }
  else {
    // this can be refactored with servoList[] - and more, probably
    char servoSymbol = cmd[0];
    String angleStr = cmd.remove(0, 1);
    int angleInt = cmd.toInt();
    String servoName;
    switch (servoSymbol) {
      case '!':
        servoName = "Front camera positional tilt base";
        if ( (0 <= angleInt) && (angleInt <= 180) ) {
          UART_PORT.println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
          frontBase.write(angleInt);
        }
        else {
          UART_PORT.println("ASTRO "+servoName+": choose values from 0 to 180");
        }
      case '@':
        servoName = "Front camera Side continuous servo";
        if ( (0 <= angleInt) && (angleInt <= 180)) {
          UART_PORT.println("ASTRO "+servoName+" is moving at rate: " + angleStr);
          frontSide.write(angleInt);
        }
        else {
          UART_PORT.println("ASTRO "+servoName+": choose values from 0 to 180");
        }
      case '#':
        servoName = "Rear camera positional tilt base";
        if ( (0 <= angleInt) && (angleInt <= 180) ) {
          UART_PORT.println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
          rearBase.write(angleInt);
        }
        else {
          UART_PORT.println("ASTRO "+servoName+": choose values from 0 to 180");
        }
      case '$':
        servoName = "Rear camera Side continuous servo";
        if ( (0 <= angleInt) && (angleInt <= 180)) {
          UART_PORT.println("ASTRO "+servoName+" is moving at rate: " + angleStr);
          frontSide.write(angleInt);
        }
        else {
          UART_PORT.println("ASTRO "+servoName+": choose values from 0 to 180");
        }
    }
  }
}

void Commands::stop(void) {
  velocityHandler(0, 0);
  for (int i = 0; i < RobotMotor::numMotors; i++) {
    String msg = "ASTRO " + String(motorList[i].motorName);
    msg += "'s desired speed: " + String(motorList[i].getCurrentVelocity());
    msg += " PWM " + String(motorList[i].direction);
    UART_PORT.println(msg);
  }
}

#endif
