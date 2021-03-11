#ifndef SETUP_H
#define SETUP_H

#include "variables.h"
#include <Stepper.h> 

/*
 * Load Settings
 */

class Setup
{    
    int NUM_MOTORS = 6;
    int motorState[6];
    int motorPins[6] = {1,2,3,4,5,6};    
    
    public:
        int fan=5, motor_input=7, multiSense=15, SEn=12;
    
        Setup();
        void enable_device(String message, int pin1);
        void disable_device(String message, int pin1);        
        void enable_all_motors();
        void disable_all_motors();
        
        int get_status(int pin1);
        void get_status_motors();
        
        float load_voltage();    
};

class Motor : public Setup
{
    public:
        Motor();    
};

#endif
