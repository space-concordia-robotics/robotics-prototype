#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "commands.h"

#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include "TinyGPS++.h"
#include <Wire.h>
#include <LSM303.h>  // contains a sketch for calibrating

I2CGPS myI2CGPS;  // I2C object
TinyGPSPlus gps;   // GPS object
LSM303 compass;

void initNav(Commands & cmdObj) {
  myI2CGPS.begin(Wire, 400000);
  if (myI2CGPS.begin(Wire, 400000) == false) { // Wire corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
    cmdObj.error = true;
    cmdObj.errorMessage = "ASTRO GPS wires aren't connected properly, please reboot\n";
    cmdObj.errorMsg();
  }

  else if (!cmdObj.error) {
    compass.init();
    compass.enableDefault();
    compass.setTimeout(100);
    compass.m_min = (LSM303::vector <int16_t>) {
      -1794, +1681, -2947
    };
    compass.m_max = (LSM303::vector <int16_t>) {
      +3359, +6531, +2016
    };
  }
}
//min: { -1794,  +1681,  -2947 }    max: { +3359,  +6531,  +2016 }

void navHandler(Commands & cmdObj) {
  if (!cmdObj.error && cmdObj.isGpsImu) {
    bool gotGps = false;
    if (myI2CGPS.available()) {         // returns the number of available bytes from the GPS module
      gps.encode(myI2CGPS.read());       // Feeds the GPS parser
      if (gps.time.isUpdated()) {        // Checks to see if new GPS info is available
        if (gps.location.isValid()) { // checks if valid location data is available
          gotGps = true;
        }
      }
    }

    compass.read();
    heading = compass.heading(LSM303::vector<int> { -1, 0, 0 });
    if (compass.timeoutOccurred()) {
      UART_PORT.print("ASTRO HEADING-N/A");
    }
    else {
      UART_PORT.print("ASTRO HEADING-OK ");
      UART_PORT.print(heading);
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
