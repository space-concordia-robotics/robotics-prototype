#include "Navigation.h"

/*initNav(){
    if (!gpsSpark.begin(Wire, 400000)) { // Wire corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
        gpsError = true;
        //Log error
    }
    else{
        //Log sucess
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
        imuError = true;
        //Log erorr
    }
    else {
        //Log sucess;
    }
}
bool Navigation::readGps(){
    bool gotGps = false;
    if (!gpsError) {
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
}
void Navigation::navHandler(){
    if (!imuError) {
        bool gotGps = readGps();

        if (imuTimeout == MAX_IMU_TIMEOUTS) {
            imuTimeout++;
            imuError = true;

        }
        else if (imuTimeout <= MAX_IMU_TIMEOUTS) {
            compass.read();
            if (compass.timeoutOccurred()) {
                imuTimeout++;
                imuError = true;
            }
            else {
                imuTimeout = 0;
                imuError = false;
                heading = compass.heading(LSM303::vector<int> { -1, 0, 0 });
            }
        }


        if(!gpsError) {
            //Helpers::get().printres(gpsPlus.location.lat(), 5); // print the latitude with 6 digits after the decimal
            //Helpers::get().printres(gpsPlus.location.lng(), 5); // print the longitude with 6 digits after the decimal
        }
    }
}*/