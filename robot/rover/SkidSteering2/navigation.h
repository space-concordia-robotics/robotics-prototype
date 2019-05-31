#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "commands.h"

#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include "TinyGPS++.h"
#include <Wire.h>
#include <LSM303.h>  // contains a sketch for calibrating

#define MAX_IMU_TIMEOUTS 10

I2CGPS myI2CGPS;  // I2C object
TinyGPSPlus gps;   // GPS object
LSM303 compass;

void initNav(Commands & cmdObj) {
  myI2CGPS.begin(Wire, 400000);
  if (myI2CGPS.begin(Wire, 400000) == false) { // Wire corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
    cmdObj.gpsError = true;
    UART_PORT.println(cmdObj.gpsErrorMsg);
  }

  compass.init();
  compass.enableDefault();
  compass.setTimeout(SENSOR_TIMEOUT); // maybe not the best value, but needs to be at least smaller than print interval
  compass.m_min = (LSM303::vector <int16_t>) {
    -1794, +1681, -2947
  };
  compass.m_max = (LSM303::vector <int16_t>) {
    +3359, +6531, +2016
  };
  compass.read();
    if (compass.timeoutOccurred()) {
      cmdObj.imuError = true;
      UART_PORT.println(cmdObj.imuErrorMsg);
    }
    else {
      UART_PORT.println("ASTRO IMU seems to be working, got an initial reading");
    }
}
//min: { -1794,  +1681,  -2947 }    max: { +3359,  +6531,  +2016 }

void navHandler(Commands & cmdObj) {
  // if gps didn't start properly it just doesn't work
  // if imu times out it will still continuously attempt to read values.. was causing delays before, to fix
  static int imuTimeoutCnt = 0;
  
  if (cmdObj.isGpsImu) {
    bool gotGps = false;
    if (!cmdObj.gpsError) {
      if (myI2CGPS.available()) {         // returns the number of available bytes from the GPS module
        gps.encode(myI2CGPS.read());       // Feeds the GPS parser
        if (gps.time.isUpdated()) {        // Checks to see if new GPS info is available
          if (gps.location.isValid()) { // checks if valid location data is available
            gotGps = true;
          }
        }
      }
    }

    if (imuTimeoutCnt == MAX_IMU_TIMEOUTS) {
      imuTimeoutCnt++;
      cmdObj.imuError = true;
      cmdObj.imuErrorMsg = "ASTRO IMU timed out too many times, giving up";
      UART_PORT.println(cmdObj.imuErrorMsg);
      UART_PORT.print("ASTRO HEADING-N/A");
    }
    else if (imuTimeoutCnt <= MAX_IMU_TIMEOUTS) {
      compass.read();
      if (compass.timeoutOccurred()) {
        imuTimeoutCnt++;
        cmdObj.imuError = true;
        UART_PORT.print("ASTRO HEADING-N/A");
      }    
      else {
        imuTimeoutCnt = 0;
        cmdObj.imuError = false;
        heading = compass.heading(LSM303::vector<int> { -1, 0, 0 });
        UART_PORT.print("ASTRO HEADING-OK ");
        UART_PORT.print(heading);
      }
    }
    else {
      UART_PORT.print("ASTRO HEADING-N/A");
    }
    UART_PORT.print(" -- ");

    if (!gotGps) {
      UART_PORT.println("GPS-N/A");
    }
    else {
      UART_PORT.print("GPS-OK ");
      UART_PORT.print(gps.location.lat(), 6); // print the latitude with 6 digits after the decimal
      UART_PORT.print(" "); // space
      UART_PORT.print(gps.location.lng(), 6); // print the longitude with 6 digits after the decimal
      UART_PORT.println("");
    }
  }
}

#endif
