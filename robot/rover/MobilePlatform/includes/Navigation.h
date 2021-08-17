#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include "TinyGPS++.h"
#include <Wire.h>
#include <LSM303.h>  // contains a sketch for calibrating

#define MAX_IMU_TIMEOUTS 10

#define SENSOR_TIMEOUT 20

typedef struct {
    I2CGPS gpsSpark;  // I2C object
    TinyGPSPlus gpsPlus;   // GPS object
    LSM303 compass;

    bool gpsError;
    bool imuError;
    float heading;
    uint8_t imuTimeout;

} navigationState;


void initNav();
bool readGps();

void navHandler();

#endif
