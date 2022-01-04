#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>

/*
 * Library deals with load operation and protection
 * To avoid complexity, motor pins and current sensing pins are predefined
 */

class Setup
{    
    int motorState[6];
    int motorPins[6] = {13,12,11,9,6,A5};    
    
    public:
           
        Setup();
        void enable_device(String message, int pin1);
        void disable_device(String message, int pin1);        
        void enable_all_motors();
        void disable_all_motors();
        float lookup(float value);
        
        int get_status(int pin1);
        void get_status_motors();
        
        float load_current(int load_switch);    
};

class Motor : public Setup
{
    public:
        Motor();    
};

#endif
