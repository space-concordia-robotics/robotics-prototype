#ifndef COMMANDS_H
#define COMMANDS_H
#include "ArduinoBlue.h"
#include "DcMotor.h"
#include <Servo.h>
// Created by Fauzi Ameen on 2019-05-21
// Bluetooth removed to simplify code and reduce sources of error by Josh Glazer on 2019-05-30.
//
class Commands {
    ArduinoBlue* phone = nullptr;
    SoftwareSerial* bluetooth = nullptr;
    DcMotor* motorList = nullptr;
    Servo* servoList = nullptr;
public:
    Commands();
    String s[4] = {"USB", "Ble-Serial", "Ble", "UART"};
    String activate_cmd = "activate";
    String deactivate_cmd = "deactivate";
    String ping_cmd = "ping";
    String who_cmd = "who";
    String reboot_cmd = "reboot";
    String bleOn_cmd = "ble-on";
    String bleOff_cmd = "ble-off";
    String closeLoop_cmd = "close-loop";
    String openLoop_cmd = "open-loop";
    String joystickOn_cmd = "joystick-on";
    String joystickOff_cmd = "joystick-off";
    String steerOff_cmd = "steer-off";
    String steerOn_cmd = "steer-on";
    String gpsOff_cmd = "gps-off";
    String gpsOn_cmd = "gps-on";
    String encOn_cmd = "enc-on";
    String encOff_cmd = "enc-off";
    String accOn_cmd = "acc-on";
    String accOff_cmd = "acc-off";
    String ttoOn_cmd = "tto-on";
    String ttoOff_cmd = "tto-off";
    String status_cmd = "status";
    String stop_cmd = "stop";

    elapsedMillis sinceThrottle; // timer for reading battery, gps and imu data


    bool isActivated = false;
    bool isOpenLoop = true; //  PID controller
    bool bluetoothMode = true;
    bool isJoystickMode = true;
    bool isSteering = true;
    bool isGpsImu = true;
    bool isEnc = true;
    bool throttleTimeOut = true;

    bool imuError = false;
    String imuErrorMsg = "ASTRO IMU may be malfunctioning, timeout reached";
    bool gpsError = false;
    String gpsErrorMsg = "ASTRO GPS could not be initialized";

    int prevThrottle = 0;
    int prevSteering = 0;
    void setupMessage(void);
    void handler(String cmd, String sender);
    void bleHandler(void);
    void systemStatus(void);

    void activate(String sender);
    void deactivate(String sender);
    void ping(void);
    void who(void);
    void rebootTeensy(void); //!< reboots the teensy using the watchdog timer
    void bleOn(String sender);
    void bleOff(String sender);
    void closeLoop(void);
    void openLoop(void);
    void joystickOn(void);
    void joystickOff(void);
    void steerOff(void);
    void steerOn(void);
    void gpsOff(void);
    void gpsOn(void);
    void encOn(void);
    void encOff(void);
    void accOn(void);
    void accOff(void);
    void ttoOn(void);
    void ttoOff(void);
    void status(void);
    void stop(bool timeout = false);
    void controlWheelMotors(String cmd);
    void controlCameraMotors(String cmd);
    void setPhone(ArduinoBlue* phone);
    void setBluetooth(SoftwareSerial* bluetooth);
    void setMotorList(DcMotor* motorList);
    void setServoList(Servo* servoList);
};

#endif
