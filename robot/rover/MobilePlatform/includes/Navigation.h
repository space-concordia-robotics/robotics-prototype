#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include "TinyGPS++.h"
#include <Wire.h>
#include <LSM303.h>  // contains a sketch for calibrating
#include "Commands.h"

#define MAX_IMU_TIMEOUTS 10

I2CGPS gpsSpark;  // I2C object
TinyGPSPlus gpsPlus;   // GPS object
LSM303 compass;

inline void initNav(Commands & cmdObj) {
  if (!gpsSpark.begin(Wire, 400000)) { // Wire corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
    cmdObj.gpsError = true;
    Helpers::get().println(cmdObj.gpsErrorMsg);
  }
  else {
   Helpers::get().println("ASTRO GPS successfully initialized");
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
    Helpers::get().println(cmdObj.imuErrorMsg);
  }
  else {
    Helpers::get().println("ASTRO IMU seems to be working, got an initial reading");
  }
}
//min: { -1794,  +1681,  -2947 }    max: { +3359,  +6531,  +2016 }

inline bool readGps(Commands & cmdObj) {
  bool gotGps = false;
  if (!cmdObj.gpsError) {
    elapsedMillis sinceStart;
    // read gps data and don't block
    while (gpsSpark.available() && sinceStart < 2) {
      gpsPlus.encode(gpsSpark.read());
    }
    if (gpsPlus.time.isUpdated()) {        // Checks to see if new GPS info is available
      //UART_PORT.println("ASTRO gps time updated");
      if (gpsPlus.location.isValid()) { // checks if valid location data is available
        //UART_PORT.println("ASTRO got valid gps location");
        gotGps = true;
      }
    }
  }
  return gotGps;
  // 38/20/04, 111/0/52.6
  // 38+0.33333+.0011111=38.33444444
  // 111+0+0.01461111=111.014611111
  // from gps: 38.3345 (last digits have error)
  // from gps: -111.0145 (last digits have error)
}

inline void navHandler(Commands & cmdObj) {
  // if gps didn't start properly it just doesn't work
  // if imu times out it will still continuously attempt to read values.. was causing delays before, to fix
  static int imuTimeoutCnt = 0;

  if (cmdObj.isGpsImu) {
    bool gotGps = readGps(cmdObj);

    if (imuTimeoutCnt == MAX_IMU_TIMEOUTS) {
      imuTimeoutCnt++;
      cmdObj.imuError = true;
      cmdObj.imuErrorMsg = "ASTRO IMU timed out too many times, giving up";
      Helpers::get().println(cmdObj.imuErrorMsg);
      Helpers::get().print("ASTRO HEADING-N/A");
    }
    else if (imuTimeoutCnt <= MAX_IMU_TIMEOUTS) {
      compass.read();
      if (compass.timeoutOccurred()) {
        imuTimeoutCnt++;
        cmdObj.imuError = true;
        Helpers::get().print("ASTRO HEADING-N/A");
      }
      else {
        imuTimeoutCnt = 0;
        cmdObj.imuError = false;
        heading = compass.heading(LSM303::vector<int> { -1, 0, 0 });
        Helpers::get().print("ASTRO HEADING-OK ");
        Helpers::get().print(heading);
      }
    }
    else {
      Helpers::get().print("ASTRO HEADING-N/A");
    }
    Helpers::get().print(" -- ");

    if (!gotGps) {
     Helpers::get().println("GPS-N/A");
    }
    else {
      Helpers::get().print("GPS-OK ");
      Helpers::get().printres(gpsPlus.location.lat(), 5); // print the latitude with 6 digits after the decimal
      Helpers::get().print(" "); // space
      Helpers::get().printres(gpsPlus.location.lng(), 5); // print the longitude with 6 digits after the decimal
      Helpers::get().println("");
    }
  }
}

#endif
