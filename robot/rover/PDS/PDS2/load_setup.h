#ifndef SETUP_H
#define SETUP_H

#include "variables.h"
#include <Stepper.h> 

/*
 * Load Settings
 */

class Setup
{
    int pin;
    
    public:
    
        Setup();
        void enable_device(String message, int pin1);
        void disable_device(String message, int pin1);
        int get_status(int pin1);
        float load_voltage();
        //void load_settings();
        /*void enable_multisense();
        void disable_multisense();
        void enable_fans();
        void disable_fans();*/        
};

class Motor : public Setup
{
    public:
        Motor();    
};

#endif
